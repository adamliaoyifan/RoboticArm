"""Live adapter for schedule(): real CLI processes, ROS windows, raw events.

Every number handed to the assembler is measured. Rows and queries are built
from receipts, status responses, and CLOCK_MONOTONIC boundaries only; nothing
is synthesized from a boolean or a label. Official counts stay at zero unless
schedule() is invoked with scored=True under an open S3, which this module
never sets.
"""
from __future__ import annotations

import json
import os
import struct
import sys
import threading
import time
from pathlib import Path

import campaign_executor as executor
import repair_s0_contract as contract
import repair_s0_judge

_CLOUD_TOPIC = "/s2_g6_sched_cloud"


def resources_from_usage(usage):
    """Map a CLI resource_usage row onto the judge's resources dict.

    The mapping never defaults to a pass: attribution is complete only when
    the product certified the read-view/cache scope, and out-of-scope classes
    are carried through untouched.
    """
    if not isinstance(usage, dict):
        return None
    read_view = usage.get("read_view_bytes")
    cache = usage.get("cache_bytes")
    certification = usage.get("certification")
    return {
        "read_view_bytes": read_view,
        "cache_bytes": cache,
        "attribution_complete": (
            certification == "READ_VIEW_CACHE_ATTRIBUTED"
            and usage.get("attribution_complete") is True),
        "unmarked_allocations": usage.get("unmarked_allocations", True),
        "certification": certification,
        "store_tree_bytes": usage.get("store_tree_bytes"),
        "uncovered_classes": list(usage.get("uncovered_classes") or []),
    }


def _pack_cloud(points):
    """Pack [x, y, z] triples into a FLOAT32 xyz PointCloud2 payload."""
    buffer = bytearray(len(points) * 12)
    for index, (x, y, z) in enumerate(points):
        struct.pack_into("<fff", buffer, index * 12, float(x), float(y), float(z))
    return bytes(buffer)


def cloud_from_frame(frame):
    """A real PointCloud2 carrying every point of an emitted frame."""
    from sensor_msgs.msg import PointCloud2, PointField
    points = frame["points_m"]
    cloud = PointCloud2()
    cloud.header.frame_id = frame.get("optical_frame", "d555_color_optical_frame")
    cloud.header.stamp.sec = 0
    cloud.header.stamp.nanosec = 0
    cloud.height = 1
    cloud.width = len(points)
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = 12 * len(points)
    for offset, name in enumerate("xyz"):
        field = PointField()
        field.name = name
        field.offset = offset * 4
        field.datatype = PointField.FLOAT32
        field.count = 1
        cloud.fields.append(field)
    cloud.data = _pack_cloud(points)
    return cloud, len(points)


def _emit_frames(eval_native, geometry, config, group, count, dest, group_name=None):
    """Materialize `count` real frames for a group through the native emitter."""
    import yaml
    dest = Path(dest)
    dest.mkdir(parents=True, exist_ok=True)
    emitted = dest / "emitted"
    completed = executor.launch_process([
        eval_native, "--geometry", geometry, "--config", config, "--gate", "emit-window",
        "--group", group_name or group["id"], "--boxes", str(group["boxes"]),
        "--cloud", str(group["cloud"]), "--outside", str(group["outside"]),
        "--cross", str(group["cross"]), "--warmup", "0", "--repeat", str(count),
        "--out", str(emitted),
    ], timeout=900)
    if completed.returncode != 0:
        raise RuntimeError((completed.stderr or completed.stdout)[-400:])
    frames = []
    for path in sorted((emitted / "frames").glob("f*.yaml"))[:count]:
        frames.append(yaml.safe_load(path.read_text()))
    if len(frames) < count:
        raise RuntimeError("generator wrote %d frames" % len(frames))
    return frames


class _WindowRun:
    """One ROS window: fresh observer, independent sender, raw event records."""

    def __init__(self, adapter, window, group, out):
        self.adapter = adapter
        self.window = window
        self.group = group
        self.out = Path(out)
        self.session = None
        self.observer = None
        self.publisher = None
        self.origin_ns = None          # wall CLOCK_MONOTONIC ns of window open
        self.clock_origin = None       # adapter clock elapsed_s at window open
        self.sent = []                 # per-input publish records
        self.real_ids = {}             # logical input id -> real observation id
        self.health_marks = []         # (mono_ns, parsed dict)
        self.sender_ident = None
        self._sender = None
        self._deadlines = []
        self._frames = []
        self._frame_index = 0
        self.rss_samples = []

    def _open_session(self):
        """Fresh observer process and client session for this window."""
        parent = str(Path(__file__).resolve().parents[1])
        if parent not in sys.path:
            sys.path.insert(0, parent)
        import s2_rehearsal
        import rclpy
        from sensor_msgs.msg import PointCloud2
        self.out.mkdir(parents=True, exist_ok=True)
        geometry = self.group["geometry_file"]
        config = self.group["config_file"]
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(self.adapter.domain)
        os.environ["ROS_DOMAIN_ID"] = str(self.adapter.domain)
        if not rclpy.ok():
            rclpy.init()
        self.session = s2_rehearsal.Session()
        self.observer = s2_rehearsal.start_observer(
            self.adapter.node, geometry, config, self.out, [
                "-p", "input_mode:=subscription", "-p", "sensor_id:=cam0",
                "-p", "optical_frame:=d555_color_optical_frame",
                "-p", "cloud_topic:=%s" % _CLOUD_TOPIC, "-p", "behavior_test_hooks:=false",
            ], env)
        self.session.wait_service(self.session.status)
        self.session.broadcaster.sendTransform(
            s2_rehearsal.transform(self.session.next_stamp()))
        self.publisher = self.session.node.create_publisher(
            PointCloud2, _CLOUD_TOPIC, executor._cloud_publisher_qos())
        self._rss_stop = threading.Event()
        self._rss_thread = threading.Thread(target=self._sample_rss, daemon=True)
        self._rss_thread.start()

    def _sample_rss(self):
        while not self._rss_stop.is_set():
            if self.observer is not None:
                self.rss_samples.append(executor._rss_kib(self.observer.proc.pid))
            self._rss_stop.wait(2.0)

    def start(self, input_count):
        self._open_session()
        geometry = self.group["geometry_file"]
        config = self.group["config_file"]
        self._frames = _emit_frames(
            self.adapter.eval_native, geometry, config, self.group, input_count,
            self.out)
        time.sleep(0.3)
        self.origin_ns = time.monotonic_ns()
        self.clock_origin = self.adapter.clock.elapsed_s
        self._deadlines = [
            self.origin_ns + int(round(index / float(self.window["input_hz"]) * 1e9))
            for index in range(input_count)]
        self._sender = threading.Thread(target=self._publish_all, daemon=True)
        self._sender.start()

    def start_for_o8(self, frame_count):
        """O8: same session, no sender thread — frames publish in pace."""
        self._open_session()
        time.sleep(0.3)
        self.origin_ns = time.monotonic_ns()
        self.clock_origin = self.adapter.clock.elapsed_s

    def _publish_all(self):
        self.sender_ident = threading.get_ident()
        for index, deadline_ns in enumerate(self._deadlines):
            delay_s = (deadline_ns - time.monotonic_ns()) / 1e9
            if delay_s > 0:
                time.sleep(delay_s)
            frame = self._frames[index % len(self._frames)]
            stamp = self.session.next_stamp()
            message, point_count = cloud_from_frame(frame)
            message.header.stamp.sec = int(stamp // 1_000_000_000)
            message.header.stamp.nanosec = int(stamp % 1_000_000_000)
            call_start_ns = time.monotonic_ns()
            self.publisher.publish(message)
            publish_return_ns = time.monotonic_ns()
            logical = executor._input_id(self.window, index)
            self.real_ids[logical] = str(stamp)
            self.sent.append({
                "index": index,
                "input_id": logical,
                "planned_ns": deadline_ns,
                "call_start_mono_ns": call_start_ns,
                "publish_return_mono_ns": publish_return_ns,
                "input_stamp_ns": stamp,
                "observation_id": str(stamp),
                "points": point_count,
            })

    def wall_deadline(self, clock_deadline):
        """Map a scheduler clock deadline onto the wall CLOCK_MONOTONIC domain."""
        return self.origin_ns + int(round((clock_deadline - self.clock_origin) * 1e9))

    def sleep_to(self, wall_ns, label):
        delay_s = (wall_ns - time.monotonic_ns()) / 1e9
        if delay_s > 0:
            time.sleep(delay_s)
        elif delay_s < -1.0:
            # Record the overrun instead of hiding it behind a clamped zero.
            self.adapter.overruns.append(
                {"boundary": label, "late_by_s": -delay_s,
                 "window": self.window["id"]})

    def sample_health(self):
        import yaml
        mark = len(self.session.health_events)
        parsed = None
        if mark:
            _, payload = self.session.health_events[mark - 1]
            loaded = yaml.safe_load(payload)
            if isinstance(loaded, dict) and "tree_nodes" in loaded:
                parsed = loaded
        return parsed

    def stop(self):
        if getattr(self, "_rss_stop", None) is not None:
            self._rss_stop.set()
        if getattr(self, "_rss_thread", None) is not None:
            self._rss_thread.join(timeout=3)
        if self._sender is not None:
            self._sender.join(timeout=10)
        if self.session is not None:
            self.session.close()
        if self.observer is not None:
            self.observer.stop()
        self.session = None
        self.observer = None


class LiveScheduleAdapter:
    """OfflineAdapter protocol on real processes and ROS windows.

    schedule() owns stage order, the O5 gate, and the budget; this adapter only
    measures. profile_frame runs one cumulative CLI process per group plus one
    fresh single-frame process per scored sample for cold-start init; ros
    windows run on a fresh observer each with an independent sender thread on
    absolute deadlines.
    """

    def __init__(self, cli, eval_native, node, geometry, config, out,
                 frame_limit=None, input_limit=None, domain=46):
        self.clock = executor.MonotonicClock()
        self.cli = cli
        self.eval_native = eval_native
        self.node = node
        self.geometry = geometry
        self.config = config
        self.out = Path(out)
        self.frame_limit = frame_limit
        self.input_limit = input_limit
        self.domain = domain
        self.profile = executor.LiveProfileAdapter(
            cli, eval_native, geometry, config, self.out / "profile", clock=self.clock)
        self._groups = {}
        self._window = None
        self._stability = None
        self.overruns = []
        self.previous_domain = os.environ.get("ROS_DOMAIN_ID")

    # ------------------------------------------------------------------ O5/O6
    def profile_frame(self, group, index, warmup, dest):
        key = group["id"]
        if key not in self._groups:
            # One extra frame: the after-update surface moment of scored
            # sample i is the first surface call of frame i+1, so the last
            # scored sample needs a successor document.
            total = (self.frame_limit or (contract.WARMUP_SAMPLES + contract.SCORED_SAMPLES)) + 1
            import yaml
            spawn_ns = time.monotonic_ns()
            samples = self.profile.run_cumulative(group, total, Path(dest).parent / "cumulative")
            documents = list(yaml.safe_load_all(
                (Path(dest).parent / "cumulative" / "samples.yaml").read_text()))
            first_row_ns = int(documents[0].get("monotonic_start_ns") or 0)
            resources = resources_from_usage(documents[-1].get("resource_usage"))
            rss = documents[-1].get("process_peak_rss_kib")
            self._groups[key] = {
                "samples": samples,
                "warm_init_s": max(0.0, (first_row_ns - spawn_ns) / 1e9) if first_row_ns else None,
                "resources": resources,
                "rss_kib": rss,
                "frame_receipts": list(getattr(self.profile, "frame_receipts", None) or []),
                "sample_problems": list(getattr(self.profile, "sample_problems", None) or []),
            }
        cached = self._groups[key]
        offset = index if index < contract.WARMUP_SAMPLES else index - contract.WARMUP_SAMPLES
        if warmup:
            sample = dict(cached["samples"][offset])
            sample["warm_init_s"] = cached["warm_init_s"]
            return sample
        # Cold start for this scored sample: one fresh single-frame process.
        cold = self._cold_start(group, offset, Path(dest))
        sample = dict(cached["samples"][offset])
        sample["init_s"] = cold["init_s"]
        sample["rss_kib"] = cold["rss_kib"]
        sample["warm_init_s"] = cached["warm_init_s"]
        if cached["resources"] is not None:
            sample["resources"] = cached["resources"]
        return sample

    def _cold_start(self, group, offset, dest):
        import yaml
        run = Path(dest) / ("cold-%03d" % offset)
        run.mkdir(parents=True, exist_ok=True)
        frames = _emit_frames(
            self.eval_native, self.group_geometry(group), self.group_config(group),
            group, 1, run)
        import capture_replay
        manifest = {
            "case_id": "cold-%s-%03d" % (group["id"], offset),
            "mode": "off", "surface": False, "state_digest": False,
            "frames": [{
                "path": "emitted/frames/f000.yaml",
                "sha256": capture_replay.sha256_file(run / "emitted" / "frames" / "f000.yaml"),
                "warmup": False, "expect_status": "INTEGRATED",
            }],
        }
        (run / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False))
        samples_path = run / "samples.yaml"
        spawn_ns = time.monotonic_ns()
        completed = executor.launch_process([
            self.cli, "--geometry", self.group_geometry(group), "--config", self.group_config(group),
            "--manifest", str(run / "manifest.yaml"), "--out", str(samples_path)],
            timeout=900)
        (run / "cli.log").write_text((completed.stdout or "") + "\n" + (completed.stderr or ""))
        if completed.returncode != 0:
            raise RuntimeError((completed.stderr or completed.stdout)[-400:])
        row = next(yaml.safe_load_all(samples_path.read_text()))
        first_row_ns = int(row.get("monotonic_start_ns") or 0)
        return {
            # Declared upper bound: spawn -> first sample row, fixture load included.
            "init_s": max(0.0, (first_row_ns - spawn_ns) / 1e9) if first_row_ns else None,
            "rss_kib": row.get("process_peak_rss_kib"),
            "observation_id": frames[0].get("observation_id"),
        }

    def group_geometry(self, group):
        return group.get("geometry_file") or self.geometry

    def group_config(self, group):
        return group.get("config_file") or self.config

    # ------------------------------------------------------------------- O7
    def ros_send(self, window, index, input_id, deadline=None, actual=None):
        if self._window is None or self._window.window["id"] != window["id"]:
            if self._window is not None:
                self._finish_window()
            group = self._window_group(window)
            count = window["inputs"] if self.input_limit is None else min(
                window["inputs"], self.input_limit)
            self._window = _WindowRun(self, window, group, self.out / "o7" / window["id"])
            self._window.start(count)
        # The sender thread owns the deadline; this call only fixes the plan.
        return None

    def _window_group(self, window):
        return next(item for item in self.plan_groups if item["id"] == window["group_id"])

    def ros_query(self, window, index, deadline=None):
        run = self._window
        query_hz = float(window.get("query_hz") or 1)
        wall_ns = run.wall_deadline(run.clock_origin + index / query_hz)
        run.sleep_to(wall_ns, "query-%d" % index)
        started_ns = time.monotonic_ns()
        # The query may be due at window open, racing the first publish; wait
        # briefly for the sender to register its target before querying.
        first_deadline = time.monotonic() + 2.0
        while not run.real_ids and time.monotonic() < first_deadline:
            time.sleep(0.01)
        recovery_first = window["kind"] == "recovery" and index == 0
        if recovery_first:
            # Started with the first send; the wait covers the target input.
            target = run.real_ids.get(executor._input_id(window, 0))
            result = self._wait_chain(run, target, started_ns)
            result["op"] = "wait_snapshot"
            result["expected_observation_id"] = executor._input_id(window, 0)
            result["target_observation_id"] = target
        elif window["kind"] == "recovery":
            result = self._immediate_query(run, started_ns)
            result["op"] = "immediate"
            result["expected_observation_id"] = None
        else:
            target = self._latest_integrated_id(run)
            result = self._wait_chain(run, target, started_ns)
            result["op"] = "wait_snapshot"
            result["expected_observation_id"] = None
            result["target_observation_id"] = target
        result["query_id"] = "q-%d" % index
        return result

    def _latest_integrated_id(self, run):
        for record in reversed(run.sent):
            logical = record["input_id"]
            real = run.real_ids.get(logical)
            if real is None:
                continue
            status = run.session.query(1, real)
            if executor._field(status, "status") == "COMPLETED":
                return real
        return run.real_ids.get(run.sent[0]["input_id"]) if run.sent else None

    def _wait_chain(self, run, target, started_ns):
        expected = {
            "epoch": 1,
            "sensor_id": "cam0",
            "observation_id": target,
            "timeout_ms": contract.WAIT_TIMEOUT_MS,
            "max_age_ms": contract.WAIT_MAX_AGE_MS,
        }
        chain = executor._read_snapshot_chain(run.session, expected)
        ended_ns = time.monotonic_ns()
        identity = chain.get("identity") or [None, None, None]
        return {
            "status": chain.get("wait_status"),
            "started_mono_ns": started_ns,
            "ended_mono_ns": ended_ns,
            "latency_ms": (ended_ns - started_ns) / 1e6,
            "age_ms": None if chain.get("age_ns") is None else chain["age_ns"] / 1e6,
            "covers_observation": chain.get("identity_stable") is True,
            "acquire_ms": chain.get("wait_ms"),
            "roi_ms": chain.get("cells_ms"),
            "surface_ms": chain.get("surface_pages_ms"),
            "epoch": identity[0],
            "observed_revision": identity[2],
            "content_hash": chain.get("content_sha256") or "",
            "chain_total_ms": chain.get("total_ms"),
        }

    def _immediate_query(self, run, started_ns):
        from uld_occ_observer.srv import GetSnapshot, ReadCells, ReadSurface, ReleaseSnapshot
        session = run.session
        request = GetSnapshot.Request()
        request.expected_epoch = 1
        request.max_age_ms = 30000
        acquire_started_ns = time.monotonic_ns()
        lease = session.call(session.snapshot, request, 5)
        acquire_ms = (time.monotonic_ns() - acquire_started_ns) / 1e6
        identity = executor._identity_tuple(executor._field(lease, "identity"))
        handle = lease.handle or ""
        status = lease.status
        reasons = []
        surface_ms = 0.0
        cells_ms = 0.0
        release_status = None
        content = []
        seen = set()
        if status == "OK" and handle:
            surface = session.node.create_client(ReadSurface, "/uld_occ_observer/read_surface")
            roi = session.node.create_client(ReadCells, "/uld_occ_observer/read_cells")
            session.wait_service(surface, timeout=5)
            cursor = 0
            while True:
                page_request = ReadSurface.Request()
                page_request.handle = handle
                page_request.cursor = int(cursor)
                page_request.limit = 512
                page_started_ns = time.monotonic_ns()
                page = session.call(surface, page_request, 5)
                surface_ms += (time.monotonic_ns() - page_started_ns) / 1e6
                columns = [executor._field(page, name) for name in ("x", "y")]
                width = None if columns[0] is None else len(columns[0])
                if width is not None:
                    for x, y in zip(columns[0], columns[1]):
                        seen.add((int(x), int(y)))
                content.append([list(item) if item is not None else None for item in
                                [executor._field(page, name) for name in
                                 ("x", "y", "active_count", "known_count", "has_occupied", "top_z")]])
                if page.status != "OK" or page.done:
                    break
                cursor = int(page.next_cursor)
            session.wait_service(roi, timeout=5)
            cell_request = ReadCells.Request()
            cell_request.handle = handle
            cell_request.begin = [0, 0, 0]
            cell_request.end = [16, 16, 16]
            cell_request.cursor = 0
            cell_request.limit = 4096
            cells_started_ns = time.monotonic_ns()
            cell_page = session.call(roi, cell_request, 5)
            cells_ms = (time.monotonic_ns() - cells_started_ns) / 1e6
            content.append([int(item) for item in list(cell_page.states or [])])
        if handle:
            try:
                release = session.node.create_client(ReleaseSnapshot, "/uld_occ_observer/release_snapshot")
                session.wait_service(release, timeout=5)
                release_request = ReleaseSnapshot.Request()
                release_request.handle = handle
                release_status = session.call(release, release_request, 5).status
            except Exception:
                release_status = "release_failed"
        if release_status != "OK":
            reasons.append("release")
        ended_ns = time.monotonic_ns()
        return {
            "status": status if status == "OK" else "snapshot_%s" % status,
            "started_mono_ns": started_ns,
            "ended_mono_ns": ended_ns,
            "latency_ms": (ended_ns - started_ns) / 1e6,
            "age_ms": executor._field(lease, "age_ns") / 1e6
                if executor._field(lease, "age_ns") is not None else None,
            "covers_observation": not reasons,
            "acquire_ms": acquire_ms,
            "roi_ms": cells_ms,
            "surface_ms": surface_ms,
            "epoch": identity[0],
            "observed_revision": identity[2],
            "content_hash": executor._content_sha256(content),
            "release_status": release_status,
            "columns": len(seen),
        }

    def ros_collect(self, window, index, input_id, deadline=None):
        run = self._window
        record = next((item for item in run.sent if item["input_id"] == input_id), None)
        real = run.real_ids.get(input_id)
        expected = {"epoch": 1, "sensor_id": "cam0", "observation_id": real}
        last = run.session.query(1, real) if real else None
        events = executor._receipt_events(run.session, real) if real else []
        terminal = executor.interpret_observation_status(last, expected, bool(events))
        row = {
            "input_id": input_id,
            "phase": executor._phase(window),
            "publish_planned_ns": record["planned_ns"] if record else None,
            "publish_end_ns": record["publish_return_mono_ns"] if record else None,
            "input_stamp_ns": record["input_stamp_ns"] if record else None,
            "clock_domain": "CLOCK_MONOTONIC",
            "callback_enter_ns": events[0].get("callback_enter_ns") if events else None,
            "target_observation_id": real,
            "disposition": terminal["disposition"],
            "terminal_count": len(events),
            "receipt_received": bool(events),
            "queue_age_ms": None,
            "observed_revision": None,
        }
        if events:
            row["queue_age_ms"] = events[0].get("queue_ms")
            row["integrate_ms"] = events[0].get("total_ms")
        receipt = executor._field(last, "receipt")
        identity = executor._field(receipt, "identity")
        if identity is not None:
            row["observed_revision"] = executor._field(identity, "observed_revision")
        if terminal["disposition"] == "rejected":
            row["reject_reason"] = executor._field(receipt, "status") or terminal["disposition"]
        else:
            row["reject_reason"] = None
        row["receipt_problems"] = executor._receipt_event_problems(events, real or "")
        return row

    def ros_health(self, window):
        run = self._window
        if window["kind"] != "normal":
            return None
        duration = float(window["duration_s"])
        origin = run.origin_ns
        times = []
        for received_ns, payload in run.session.health_events:
            relative = (received_ns - origin) / 1e9
            if -1.0 <= relative <= duration + 5.0:
                # Window convention: t=0 is the window open; a health sample
                # received before it counts as the open sample, clamped to 0.
                times.append(round(max(0.0, relative), 6))
        return sorted(times) if len(times) >= 2 else None

    def ros_drain(self, window, deadline=None):
        run = self._window
        deadline_wall = run.wall_deadline(deadline if deadline is not None
                                          else run.clock_origin)
        run.sleep_to(deadline_wall, "drain")
        remaining_ids = []
        for logical, real in run.real_ids.items():
            status = run.session.query(1, real)
            state = executor._field(status, "status")
            if state not in ("COMPLETED", "REJECTED", "EXPIRED"):
                remaining_ids.append(logical)
        deadline_wait = time.monotonic() + 30
        while remaining_ids and time.monotonic() < deadline_wait:
            time.sleep(0.5)
            still = []
            for logical in remaining_ids:
                state = executor._field(run.session.query(1, run.real_ids[logical]), "status")
                if state not in ("COMPLETED", "REJECTED", "EXPIRED"):
                    still.append(logical)
            remaining_ids = still
        return len(remaining_ids)

    def _finish_window(self):
        run = self._window
        if run is None:
            return
        payload = {
            "window": run.window["id"],
            "sender_thread": run.sender_ident,
            "spin_thread": run.session.thread.ident if run.session else None,
            "sent": run.sent,
            "overruns": [item for item in self.overruns if item.get("window") == run.window["id"]],
        }
        executor._write(run.out / "send_log.json", payload)
        run.stop()
        if self.previous_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = self.previous_domain

    # ------------------------------------------------------------------- O8
    def stability_frame(self, run_id, index):
        if (self._stability is None or self._stability.get("run_id") != run_id):
            if self._stability is not None:
                self._stability["run"].stop()
            o8 = self.plan_o8
            base_id = o8["ray_group"][len("o8_fixed_"):]
            group = next(item for item in self.plan_groups if item["id"] == base_id)
            run = _WindowRun(self, {"id": "o8-%s" % run_id, "kind": "o8", "input_hz": 1},
                             group, self.out / "o8" / run_id)
            # 固定射线: one emitted frame under the ray group's own seed,
            # republished with incrementing stamps for the whole run.
            run.o8_frames = _emit_frames(
                self.eval_native, self.group_geometry(group), self.group_config(group),
                group, 1, run.out / "rays",
                group_name=o8["ray_group"])
            run.start_for_o8(o8["saturation_frames"])
            self._stability = {"run_id": run_id, "run": run, "receipts": [], "stamps": []}
        run = self._stability["run"]
        # The scheduler's clock advance is a no-op on the monotonic clock: the
        # adapter itself paces the frames to the real window marks.
        o8 = self.plan_o8
        mark_s = (index + 1) * (float(o8["duration_s"]) / o8["saturation_frames"])
        run.sleep_to(run.origin_ns + int(mark_s * 1e9), "o8-frame-%d" % index)
        frame = run.o8_frames[0]
        stamp = run.session.next_stamp()
        message, _points = cloud_from_frame(frame)
        message.header.stamp.sec = int(stamp // 1_000_000_000)
        message.header.stamp.nanosec = int(stamp % 1_000_000_000)
        run.publisher.publish(message)
        self._stability["stamps"].append(str(stamp))

    def stability_run(self, run_id):
        state = self._stability
        run = state["run"]
        # Saturation: the changed/rebuilt counters of the late receipts.
        changed_keys = 0
        rebuilt = 0
        for observation in state["stamps"][-5:]:
            events = executor._receipt_events(run.session, observation)
            if not events:
                continue
            status = run.session.query(1, observation)
            receipt = executor._field(status, "receipt")
            changed_keys = max(changed_keys, int(executor._field(receipt, "changed_keys") or 0))
            rebuilt = max(rebuilt, int(executor._field(receipt, "rebuilt_blocks") or 0))
        # Node stability over the post-warmup window and three 60 s RSS medians.
        import statistics
        warmup_s = float(self.plan_o8["warmup_s"])
        duration_s = float(self.plan_o8["duration_s"])
        origin = run.origin_ns
        nodes = []
        for received_ns, payload in run.session.health_events:
            import yaml
            loaded = yaml.safe_load(payload)
            if not isinstance(loaded, dict) or "tree_nodes" not in loaded:
                continue
            relative = (received_ns - origin) / 1e9
            if warmup_s <= relative <= duration_s:
                nodes.append(int(loaded["tree_nodes"]))
        rss = run.rss_samples
        thirds = [rss[index:index + max(1, len(rss) // 3)] for index in
                  range(0, len(rss), max(1, len(rss) // 3))][:3] if rss else []
        medians = [int(statistics.median([sample for sample in third if sample is not None]))
                   for third in thirds if third]
        return {
            "run_id": run_id,
            "saturation_probability_keys": changed_keys,
            "saturation_geometry_blocks": rebuilt,
            "node_count_stable": bool(nodes) and len(set(nodes)) == 1,
            "rss_median_kib": medians,
        }

    def hold(self, run_id, kind, repeat):
        state = self._stability
        run = state["run"]
        session = run.session
        seconds = float(self.plan_o8.get(
            "no_input_s" if kind == "no_input" else "occlusion_s", 10.0))
        # A hold starts from a quiescent map: wait for every stability frame
        # to reach a terminal state, otherwise a straggler integration moves
        # the map during the "no input" hold.
        self._await_terminal(run, state["stamps"])
        before = self._hold_probe(run)
        if kind == "occlusion":
            # Occlusion input: the same cloud identity replayed, which the
            # observer rejects at the ledger (duplicates never re-update the
            # map). Real transport, nothing accepted — an all-outside cloud
            # does not work: its rays still cross the container and integrate.
            if not getattr(run, "occlusion_stamp", None):
                run.occlusion_stamp = state["stamps"][-1] if state["stamps"] else None
            if run.occlusion_stamp:
                stamp = int(run.occlusion_stamp)
                frame = run.o8_frames[0]
                message, _points = cloud_from_frame(frame)
                message.header.stamp.sec = int(stamp // 1_000_000_000)
                message.header.stamp.nanosec = int(stamp % 1_000_000_000)
                run.publisher.publish(message)
        # The hold spans real seconds; the scheduler's advance is a no-op here.
        time.sleep(seconds)
        after = self._hold_probe(run)
        return {
            "kind": kind,
            "probability_unchanged": (
                before is not None and after is not None
                and before.get("content_hash") is not None
                and before.get("content_hash") == after.get("content_hash")),
            "freshness_advanced": (
                before is not None and after is not None
                and before.get("last_integrated_stamp_ns") is not None
                and (after.get("last_integrated_stamp_ns") or -1)
                > before["last_integrated_stamp_ns"]),
            "probe_before": before,
            "probe_after": after,
            "duration_s": seconds,
        }

    def _await_terminal(self, run, observation_ids, timeout_s=30.0):
        pending = [item for item in observation_ids if item]
        deadline = time.monotonic() + timeout_s
        while pending and time.monotonic() < deadline:
            still = []
            for observation in pending:
                state = executor._field(run.session.query(1, observation), "status")
                if state not in ("COMPLETED", "REJECTED", "EXPIRED"):
                    still.append(observation)
            pending = still
            if pending:
                time.sleep(0.2)
        return not pending

    def _hold_probe(self, run):
        session = run.session
        try:
            from uld_occ_observer.srv import GetSnapshot, ReadCells, ReleaseSnapshot
            request = GetSnapshot.Request()
            request.expected_epoch = 1
            request.max_age_ms = 30000
            lease = session.call(session.snapshot, request, 5)
            if lease.status != "OK":
                return None
            identity = executor._identity_tuple(executor._field(lease, "identity"))
            roi = session.node.create_client(ReadCells, "/uld_occ_observer/read_cells")
            session.wait_service(roi, timeout=5)
            cell_request = ReadCells.Request()
            cell_request.handle = lease.handle
            cell_request.begin = [0, 0, 0]
            cell_request.end = [16, 16, 16]
            cell_request.cursor = 0
            cell_request.limit = 4096
            page = session.call(roi, cell_request, 5)
            states = [int(item) for item in list(page.states or [])]
            release = session.node.create_client(ReleaseSnapshot, "/uld_occ_observer/release_snapshot")
            session.wait_service(release, timeout=5)
            release_request = ReleaseSnapshot.Request()
            release_request.handle = lease.handle
            session.call(release, release_request, 5)
            return {
                "content_hash": executor._content_sha256([states]),
                "last_integrated_stamp_ns": identity[3] if len(identity) > 3 else None,
            }
        except Exception as exc:
            return {"error": "%s:%s" % (type(exc).__name__, str(exc)[:160])}

    # ------------------------------------------------------------- lifecycle
    def bind_plan(self, plan):
        self.plan_groups = plan["o6_groups"]
        self.plan_o8 = plan["o8"]

    def close(self):
        if self._window is not None:
            self._finish_window()
            self._window = None
        if self._stability is not None:
            self._stability["run"].stop()
            self._stability = None
        if self.previous_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = self.previous_domain
        # Mirror the wiring path: leave rclpy down so the next live entry
        # initializes under its own domain instead of inheriting this one.
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()
