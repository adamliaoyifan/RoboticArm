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

# O8 occlusion input, in ULD/container coordinates. The sensor sits outside
# the -x end of the container (optical origin maps to ULD (-1.15, 0, 1.20),
# looking along +x). A plane of valid depth-range points at x = -0.62 sits
# between the entrance wall (-0.745) and the interior region the fixed rays
# map (from -0.58 = cell 8 on): every occluder ray ends at the plane, so no
# ray crosses into the old region. Cell 6 at 2 cm resolution is the plane
# slab; the oracle region starts at cell 9, leaving cells 7-8 as margin.
# y stays below the chamfer cut on +y (floor_y 0.55, wall_z 0.9).
_OCCLUDER_X_M = -0.62
_OCCLUDER_MAX_CELL_X = 7          # inclusive bound of cells the plane may touch
_OCCLUDER_Y = (-0.85, 0.52)
_OCCLUDER_Z = (0.75, 1.85)
_OCCLUDER_STEP_M = 0.03
_ORACLE_MIN_CELL_X = 9
_SENSOR_POSE_ULD = (-1.15, 0.0, 1.20)


def occluder_points():
    """Dense plane points in the OPTICAL frame (depth axis = optical z).

    ULD (x, y, z) -> optical (-y, 1.20 - z, x + 1.15). The plane's depth is
    0.53 m, inside the fixture's valid band [0.52, 6.0].
    """
    y0, y1 = _OCCLUDER_Y
    z0, z1 = _OCCLUDER_Z
    count_y = int((y1 - y0) / _OCCLUDER_STEP_M) + 1
    count_z = int((z1 - z0) / _OCCLUDER_STEP_M) + 1
    points = []
    for iy in range(count_y):
        for iz in range(count_z):
            y = y0 + iy * _OCCLUDER_STEP_M
            z = z0 + iz * _OCCLUDER_STEP_M
            points.append((-y, _SENSOR_POSE_ULD[2] - z, _OCCLUDER_X_M - _SENSOR_POSE_ULD[0]))
    return points


def _probability_unchanged(before, after):
    """Voxel-level comparison: complete states AND log-odds over the region.

    Both probes must be error-free, cover the same non-empty cell count, and
    carry both hashes. The surface geometry hash is deliberately NOT part of
    this: sub-threshold log-odds changes leave the surface summary alone.
    """
    if not isinstance(before, dict) or not isinstance(after, dict):
        return False
    if "error" in before or "error" in after:
        return False
    cells_before, cells_after = before.get("cells"), after.get("cells")
    if not isinstance(cells_before, int) or cells_before <= 0 or cells_before != cells_after:
        return False
    return (isinstance(before.get("states_hash"), str)
            and before.get("states_hash") == after.get("states_hash")
            and before.get("log_odds_hash") == after.get("log_odds_hash"))


def _surface_unchanged(before, after):
    """Secondary geometric summary comparison (surface columns)."""
    if not isinstance(before, dict) or not isinstance(after, dict):
        return False
    if "error" in before or "error" in after:
        return False
    return (isinstance(before.get("surface_hash"), str)
            and before.get("surface_hash") == after.get("surface_hash"))


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


def _peak_resources_from_documents(documents):
    """Run-covering resource peak, not the last row's current value.

    Byte counters that the product reports per row are scanned across every
    document; a class missing from any row keeps its observed maximum and a
    class missing everywhere stays missing (the judge rejects it). Peaks the
    product reports directly (read_view_bytes_peak and friends) are carried
    alongside; the scalar fields stay the observed maxima so the judge sees
    conservative numbers.
    """
    rows = [row.get("resource_usage") for row in documents
            if isinstance(row.get("resource_usage"), dict)]
    if not rows:
        return None
    def peak(key):
        values = [row.get(key) for row in rows if isinstance(row.get(key), int)]
        return max(values) if values else None
    certifications = {row.get("certification") for row in rows}
    complete = all(
        row.get("certification") == "READ_VIEW_CACHE_ATTRIBUTED"
        and row.get("attribution_complete") is True for row in rows)
    uncovered = []
    for row in rows:
        for name in row.get("uncovered_classes") or []:
            if name not in uncovered:
                uncovered.append(name)
    return {
        "read_view_bytes": peak("read_view_bytes"),
        "cache_bytes": peak("cache_bytes"),
        "read_view_bytes_peak": peak("read_view_bytes_peak"),
        "cache_bytes_peak": peak("cache_bytes_peak"),
        "store_tree_bytes": peak("store_tree_bytes"),
        "attribution_complete": complete,
        "unmarked_allocations": any(row.get("unmarked_allocations") for row in rows),
        "certification": ("READ_VIEW_CACHE_ATTRIBUTED"
                          if certifications == {"READ_VIEW_CACHE_ATTRIBUTED"}
                          else sorted(str(item) for item in certifications)[0]),
        "uncovered_classes": uncovered,
        "rows_scanned": len(rows),
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


def publish_occluder(session, publisher, points):
    """Publish the occluder plane once and return its terminal receipt."""
    from sensor_msgs.msg import PointCloud2, PointField
    stamp = session.next_stamp()
    cloud = PointCloud2()
    cloud.header.frame_id = "d555_color_optical_frame"
    cloud.header.stamp.sec = int(stamp // 1_000_000_000)
    cloud.header.stamp.nanosec = int(stamp % 1_000_000_000)
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
    publisher.publish(cloud)
    observation = str(stamp)
    last, terminal = executor._wait_integrated(session, observation, 10.0)
    events = executor._receipt_events(session, observation)
    receipt = executor._field(last, "receipt")
    return {
        "observation_id": observation,
        "events": len(events),
        "status": executor._field(last, "status"),
        "disposition": terminal["disposition"],
        "changed_keys": executor._field(receipt, "changed_keys"),
        "rebuilt_blocks": executor._field(receipt, "rebuilt_blocks"),
    }


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
                # Timestamped so window statistics bucket by real sample time,
                # never by splitting the array into equal shares.
                self.rss_samples.append(
                    (time.monotonic_ns(), executor._rss_kib(self.observer.proc.pid)))
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

    def continue_as(self, window, input_count):
        """Continue the SAME observer, session, and map into a new window.

        The recovery window must measure the overloaded node draining back to
        service, not a freshly started one: no new observer process, no map
        reset. The returned continuity record pins the process identity and
        the boundary stamps so the artifact proves the reuse.
        """
        group = self.adapter._window_group(window)
        previous_id = self.window["id"]
        if self._sender is not None:
            self._sender.join(timeout=30)
            self._sender = None
        boundary_ns = time.monotonic_ns()
        revision = self._identity_snapshot()
        self.window = window
        self.group = group
        self._frames = _emit_frames(
            self.adapter.eval_native, self.adapter.group_geometry(group),
            self.adapter.group_config(group),
            group, input_count, self.out / "windows" / window["id"])
        self.origin_ns = time.monotonic_ns()
        self.clock_origin = self.adapter.clock.elapsed_s
        self._deadlines = [
            self.origin_ns + int(round(index / float(window["input_hz"]) * 1e9))
            for index in range(input_count)]
        self._sender = threading.Thread(target=self._publish_all, daemon=True)
        self._sender.start()
        return {
            "reused_observer": True,
            "observer_pid": self.observer.proc.pid if self.observer is not None else None,
            "overload_window": previous_id,
            "recovery_window": window["id"],
            "drain_boundary_mono_ns": getattr(self, "drain_deadline_mono_ns", None),
            "recovery_origin_mono_ns": self.origin_ns,
            "epoch": None if revision is None else revision[0],
            "observed_revision_at_recovery_open": None if revision is None else revision[2],
            "continuation_gap_s": (self.origin_ns - boundary_ns) / 1e9,
        }

    def _identity_snapshot(self):
        """(epoch, map_revision, observed_revision) of the current map, or None."""
        try:
            from uld_occ_observer.srv import GetSnapshot
            request = GetSnapshot.Request()
            request.expected_epoch = 1
            request.max_age_ms = 300000
            request.allow_stale = True
            lease = self.session.call(self.session.snapshot, request, 5)
            if lease.status != "OK":
                return None
            return executor._identity_tuple(executor._field(lease, "identity"))
        except Exception:
            return None

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
        self._continuity = None
        self.overruns = []
        self.previous_domain = os.environ.get("ROS_DOMAIN_ID")

    # ------------------------------------------------------------------ O5/O6
    def profile_frame(self, group, index, warmup, dest):
        key = group["id"]
        if key not in self._groups:
            # The cumulative run emits warmup rows first, then the scored rows,
            # plus one successor: the after-update surface moment of scored
            # sample i is the first surface call of frame i+1. The scheduler's
            # formal indices are exactly the row indices: warmup rows 0..2,
            # scored rows 3..32. A limited rehearsal still emits the warmup
            # rows so index alignment is identical to the formal run.
            total = contract.WARMUP_SAMPLES + (self.frame_limit or contract.SCORED_SAMPLES) + 1
            import yaml
            spawn_ns = time.monotonic_ns()
            samples = self.profile.run_cumulative(group, total, Path(dest).parent / "cumulative")
            documents = list(yaml.safe_load_all(
                (Path(dest).parent / "cumulative" / "samples.yaml").read_text()))
            first_row_ns = int(documents[0].get("monotonic_start_ns") or 0)
            resources = _peak_resources_from_documents(documents)
            # Cumulative-process peak RSS over the whole run, never overwritten
            # by a cold-start single-frame process below.
            rss = max((int(row.get("process_peak_rss_kib") or 0) for row in documents), default=0)
            self._groups[key] = {
                "samples": samples,
                "warm_init_s": max(0.0, (first_row_ns - spawn_ns) / 1e9) if first_row_ns else None,
                "resources": resources,
                "rss_kib": rss,
                "frame_receipts": list(getattr(self.profile, "frame_receipts", None) or []),
                "sample_problems": list(getattr(self.profile, "sample_problems", None) or []),
            }
        cached = self._groups[key]
        # Formal scored indices are 3..32 and must select rows 3..32 of the
        # cumulative run: the three warmup rows are never rescored and the
        # last three scored rows are never dropped.
        offset = index
        if warmup:
            sample = dict(cached["samples"][offset])
            sample["warm_init_s"] = cached["warm_init_s"]
            return sample
        # Cold start for this scored sample: one fresh single-frame process.
        cold = self._cold_start(group, offset, Path(dest))
        sample = dict(cached["samples"][offset])
        sample["init_s"] = cold["init_s"]
        sample["rss_kib"] = cached["rss_kib"]
        sample["cold_start_rss_kib"] = cold["rss_kib"]
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
            count = window["inputs"] if self.input_limit is None else min(
                window["inputs"], self.input_limit)
            if (self._window is not None and self._window.window["kind"] == "overload"
                    and window["kind"] == "recovery"):
                # The recovery window continues the overloaded node: same
                # observer process, same map, no restart between drain and
                # recovery. The continuity record lands in the artifact.
                self._continuity = self._window.continue_as(window, count)
            else:
                if self._window is not None:
                    self._finish_window()
                group = self._window_group(window)
                self._window = _WindowRun(self, window, group, self.out / "o7" / window["id"])
                self._window.start(count)
        # The sender thread owns the deadline; this call only fixes the plan.
        return None

    def ros_continuity(self, window):
        if window["kind"] == "recovery":
            return getattr(self, "_continuity", None)
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
        # briefly for the sender to register THIS window's first target. A
        # continued window still carries the previous window's ids, so the
        # wait is for the specific logical id, not for any id at all.
        recovery_first = window["kind"] == "recovery" and index == 0
        if recovery_first:
            first_logical = executor._input_id(window, 0)
            first_deadline = time.monotonic() + 2.0
            while first_logical not in run.real_ids and time.monotonic() < first_deadline:
                time.sleep(0.01)
            target = run.real_ids.get(first_logical)
            result = self._wait_chain(run, target, started_ns)
            result["op"] = "wait_snapshot"
            result["expected_observation_id"] = first_logical
            result["target_observation_id"] = target
        else:
            # Normal windows and recovery queries 1..29 use the original
            # immediate acquire chain; a deferred WaitSnapshot would hide the
            # acquisition latency these gates measure.
            result = self._immediate_query(run, started_ns)
            result["op"] = "immediate"
            result["expected_observation_id"] = None
        result["query_id"] = "q-%d" % index
        return result

    def _wait_chain(self, run, target, started_ns):
        if target is None:
            # The sender never registered the target id: an honest timeout,
            # never a synthesized wait over a null observation.
            return {
                "status": "TIMEOUT",
                "started_mono_ns": started_ns,
                "ended_mono_ns": time.monotonic_ns(),
                "latency_ms": (time.monotonic_ns() - started_ns) / 1e6,
                "age_ms": None,
                "covers_observation": False,
                "acquire_ms": None,
                "roi_ms": None,
                "surface_ms": None,
                "epoch": None,
                "observed_revision": None,
                "content_hash": "",
                "chain_total_ms": None,
            }
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
        if lease.status == "NO_DATA":
            # Query 0 races the window's first integrate on a store that has
            # never published: wait for the first input to reach terminal
            # (bounded), then acquire once more. The wait lands inside this
            # request's latency, never hidden. Later queries see a live map.
            first = run.sent[0]["observation_id"] if run.sent else None
            if first:
                self._await_terminal(run, [first], timeout_s=2.0)
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
        # The drain window is fixed: one status pass exactly at the deadline,
        # no top-up waiting afterwards. Whatever is still in flight is the
        # honest observation the overload judge evaluates.
        run.drain_deadline_mono_ns = deadline_wall
        remaining_ids = []
        for logical, real in run.real_ids.items():
            status = run.session.query(1, real)
            state = executor._field(status, "status")
            if state not in ("COMPLETED", "REJECTED", "EXPIRED"):
                remaining_ids.append(logical)
        run.drain_record = {
            "window": window["id"],
            "drain_deadline_mono_ns": deadline_wall,
            "observed_mono_ns": time.monotonic_ns(),
            "in_flight": len(remaining_ids),
            "in_flight_ids": remaining_ids,
        }
        return len(remaining_ids)

    def _finish_window(self):
        run = self._window
        if run is None:
            return
        payload = {
            "window": run.window["id"],
            "sender_thread": run.sender_ident,
            "spin_thread": run.session.thread.ident if run.session else None,
            "observer_pid": run.observer.proc.pid if run.observer is not None else None,
            "sent": run.sent,
            "overruns": [item for item in self.overruns if item.get("window") == run.window["id"]],
        }
        if getattr(run, "drain_record", None) is not None:
            payload["drain"] = run.drain_record
        if getattr(self, "_continuity", None) is not None:
            payload["continuity"] = self._continuity
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
            if self._window is not None:
                # The O7 stage's last window must not outlive its stage: two
                # live observers on one domain split every service call
                # between different maps, and the stability probes would read
                # whichever answered first.
                self._finish_window()
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
        import statistics
        import yaml
        # The saturation check is its own 30-frame pass, after the paced
        # window: every frame gets a fresh stamp, must reach a matched
        # terminal receipt, and is verified frame by frame for zero change.
        # A frame without a successful terminal receipt is recorded as such
        # and never silently skipped.
        self._await_terminal(run, state["stamps"], timeout_s=30.0)
        saturation_frames = int(self.plan_o8["saturation_frames"])
        receipts = []
        published_stamps = []
        for index in range(saturation_frames):
            stamp = run.session.next_stamp()
            frame = run.o8_frames[0]
            message, _points = cloud_from_frame(frame)
            message.header.stamp.sec = int(stamp // 1_000_000_000)
            message.header.stamp.nanosec = int(stamp % 1_000_000_000)
            run.publisher.publish(message)
            observation = str(stamp)
            published_stamps.append(observation)
            last, terminal = self._wait_terminal(run, observation, timeout_s=10.0)
            events = executor._receipt_events(run.session, observation)
            receipt = executor._field(last, "receipt")
            identity = executor._field(receipt, "identity") or {}
            entry = {
                "index": index,
                "observation_id": observation,
                "events": len(events),
                "status": executor._field(last, "status"),
                "disposition": terminal["disposition"],
                "epoch": executor._field(identity, "epoch"),
                "sensor_id": executor._field(receipt, "sensor_id"),
                "receipt_observation_id": executor._field(receipt, "observation_id"),
                "changed_keys": executor._field(receipt, "changed_keys"),
                "rebuilt_blocks": executor._field(receipt, "rebuilt_blocks"),
                "queue_ms": executor._field(receipt, "queue_ms"),
                "total_ms": executor._field(receipt, "total_ms"),
                "callback_enter_ns": events[0].get("callback_enter_ns") if events else None,
            }
            receipts.append(entry)
        changed_values = [entry["changed_keys"] for entry in receipts]
        rebuilt_values = [entry["rebuilt_blocks"] for entry in receipts]
        changed_keys = max((int(value) for value in changed_values
                            if isinstance(value, (int, float))), default=0)
        rebuilt = max((int(value) for value in rebuilt_values
                       if isinstance(value, (int, float))), default=0)
        # Node stability over the post-warmup window, with the sample times
        # saved: a single lucky sample must not certify stability.
        warmup_s = float(self.plan_o8["warmup_s"])
        duration_s = float(self.plan_o8["duration_s"])
        origin = run.origin_ns
        nodes = []
        node_times = []
        for received_ns, payload in run.session.health_events:
            loaded = yaml.safe_load(payload)
            if not isinstance(loaded, dict) or "tree_nodes" not in loaded:
                continue
            relative = (received_ns - origin) / 1e9
            if warmup_s <= relative <= duration_s:
                nodes.append(int(loaded["tree_nodes"]))
                node_times.append(round(relative, 3))
        # Raw RSS samples with times so the judge re-buckets and recomputes.
        rss_samples = [
            [round((mark_ns - origin) / 1e9, 3), value]
            for mark_ns, value in run.rss_samples
            if isinstance(value, int)]
        # Three real 60 s RSS windows over the run, bucketed by sample time.
        window_s = contract.O8_RSS_WINDOW_S
        windows = []
        counts = []
        for start in (0, window_s, 2 * window_s):
            bucket = [value for elapsed, value in rss_samples
                      if start <= elapsed < start + window_s]
            windows.append(int(statistics.median(bucket)) if bucket else None)
            counts.append(len(bucket))
        row = {
            "run_id": run_id,
            "saturation_frames_expected": saturation_frames,
            "saturation_frames_verified": sum(
                1 for entry in receipts
                if entry["events"] == 1 and entry["status"] == "COMPLETED"
                and entry["disposition"] == "integrated"
                and entry["receipt_observation_id"] == entry["observation_id"]
                and isinstance(entry["changed_keys"], int)
                and isinstance(entry["rebuilt_blocks"], int)),
            "saturation_stamps": published_stamps,
            "saturation_receipts": receipts,
            "saturation_probability_keys": changed_keys,
            "saturation_geometry_blocks": rebuilt,
            "node_count_stable": bool(nodes) and len(set(nodes)) == 1,
            "node_health_times_s": sorted(node_times),
            "node_health_values": [value for _time, value in sorted(zip(node_times, nodes))],
            "node_health_count": len(nodes),
            "rss_median_kib": windows,
            "rss_window_counts": counts,
            "rss_samples": rss_samples,
        }
        executor._write(run.out / ("saturation-%s.json" % run_id), row)
        return row

    def _wait_terminal(self, run, observation_id, timeout_s=10.0):
        """Status + interpreted disposition once this id reaches a terminal state."""
        last, terminal = executor._wait_integrated(run.session, observation_id, timeout_s)
        return last, terminal

    def hold(self, run_id, kind, repeat):
        state = self._stability
        run = state["run"]
        session = run.session
        seconds = float(self.plan_o8.get(
            "no_input_s" if kind != "occlusion" else "occlusion_s", 10.0))
        # A hold starts from a quiescent map: wait for every stability frame
        # to reach a terminal state, otherwise a straggler integration moves
        # the map during the hold.
        self._await_terminal(run, state["stamps"])
        hold_started_ns = time.monotonic_ns()
        before = self._hold_probe(run)
        row = {
            "kind": kind,
            "duration_s": seconds,
            "probe_before": before,
            "probe_after": None,
            "duration_measured_s": None,
        }
        if kind == "occlusion":
            # Real geometric blocking: a plane of valid points between the
            # sensor and the mapped interior. Every occluder ray ends at the
            # plane, so no ray crosses into the old region; the wall itself
            # integrates (accepted new data) while the old region beyond the
            # plane must keep its probabilities. The rays and the oracle
            # definition are saved with the hold.
            points = occluder_points()
            occluder_receipt = self._publish_occluder(run, points)
            time.sleep(seconds)
            after = self._hold_probe(run)
            row["occlusion_input"] = {
                "shape": "plane",
                "plane_uld_x_m": _OCCLUDER_X_M,
                "plane_max_cell_x": _OCCLUDER_MAX_CELL_X,
                "y_range_m": list(_OCCLUDER_Y),
                "z_range_m": list(_OCCLUDER_Z),
                "point_count": len(points),
                "points_sha256": executor._content_sha256(points),
                "rays_file": "occlusion-rays-%d.json" % repeat,
                "oracle_min_cell_x": _ORACLE_MIN_CELL_X,
                "sensor_uld_origin_m": [-1.15, 0.0, 1.2],
            }
            executor._write(run.out / ("occlusion-rays-%d.json" % repeat),
                            {"repeat": repeat, "origin_uld_m": [-1.15, 0.0, 1.2],
                             "plane_uld_x_m": _OCCLUDER_X_M,
                             "points_optical_m": points})
            row["occluder_receipt"] = occluder_receipt
            row["input_accepted"] = (
                occluder_receipt is not None
                and occluder_receipt.get("status") == "COMPLETED")
            row["region_probability_unchanged"] = _probability_unchanged(before, after)
            row["surface_geometry_unchanged"] = _surface_unchanged(before, after)
            row["freshness_advanced"] = self._freshness_advanced(before, after)
        elif kind == "duplicate_id":
            # Dedup check, separate from occlusion: the same cloud identity
            # replayed. The duplicate is rejected unstored at admission, so
            # the ledger keeps the original record and only the receipt
            # stream shows the DUPLICATE; the map and the freshness (the
            # stamp is identical) must not move.
            stamp = state["stamps"][-1] if state["stamps"] else None
            duplicate_receipt = None
            if stamp:
                events_before = len(executor._receipt_events(run.session, stamp))
                frame = run.o8_frames[0]
                message, _points = cloud_from_frame(frame)
                stamp_ns = int(stamp)
                message.header.stamp.sec = stamp_ns // 1_000_000_000
                message.header.stamp.nanosec = stamp_ns % 1_000_000_000
                run.publisher.publish(message)
                time.sleep(1.0)
                events = executor._receipt_events(run.session, stamp)
                duplicate_receipt = {
                    "observation_id": stamp,
                    "events_before": events_before,
                    "events": len(events),
                    "duplicate_event": any(
                        event.get("status") == "DUPLICATE" for event in events),
                    "new_events": len(events) - events_before,
                }
            time.sleep(seconds)
            after = self._hold_probe(run)
            row["duplicate_receipt"] = duplicate_receipt
            row["duplicate_rejected"] = (
                duplicate_receipt is not None
                and duplicate_receipt.get("duplicate_event") is True)
            row["probability_unchanged"] = _probability_unchanged(before, after)
            row["surface_geometry_unchanged"] = _surface_unchanged(before, after)
            row["freshness_advanced"] = self._freshness_advanced(before, after)
        else:
            time.sleep(seconds)
            after = self._hold_probe(run)
            row["probability_unchanged"] = _probability_unchanged(before, after)
            row["surface_geometry_unchanged"] = _surface_unchanged(before, after)
            row["freshness_advanced"] = self._freshness_advanced(before, after)
        row["probe_after"] = after
        row["duration_measured_s"] = round(
            (after.get("probe_mono_ns", time.monotonic_ns()) - hold_started_ns) / 1e9, 3) \
            if after and "error" not in after else None
        executor._write(run.out / ("hold-%s-%d.json" % (kind, repeat)), row)
        return row

    def _freshness_advanced(self, before, after):
        return (
            before is not None and after is not None
            and before.get("last_integrated_stamp_ns") is not None
            and (after.get("last_integrated_stamp_ns") or -1)
            > before["last_integrated_stamp_ns"])

    def _publish_occluder(self, run, points):
        return publish_occluder(run.session, run.publisher, points)

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

    def _oracle_region(self, run):
        """The explicit old region beyond the occluder margin, as cell bounds.

        The grid shape is derived from the fixture geometry (ceil of extent
        over resolution) and cross-checked against the full-grid surface
        column count on first use; a mismatch is an error, never a guess.
        """
        cached = getattr(self, "_oracle_region_cache", None)
        if cached is not None:
            return cached
        import yaml
        geometry_file = self.group_geometry(run.group)
        config_file = self.group_config(run.group)
        try:
            geometry = yaml.safe_load(Path(geometry_file).read_text())
            config = yaml.safe_load(Path(config_file).read_text())
            descriptor = json.loads(geometry["descriptor_json"])
            resolution = float(config["resolution_m"])
            shape = [int(-(-extent // resolution)) for extent in (
                descriptor["length"], descriptor["width"],
                descriptor["ceiling_z"] - descriptor["floor_z"])]
            if min(shape) <= _ORACLE_MIN_CELL_X:
                raise ValueError("oracle bound outside grid")
            region = {
                "begin": [_ORACLE_MIN_CELL_X, 0, 0],
                "end": shape,
                "shape": shape,
                "cells": (shape[0] - _ORACLE_MIN_CELL_X) * shape[1] * shape[2],
                "resolution_m": resolution,
                "geometry_hash": geometry.get("descriptor_sha256"),
            }
        except Exception as exc:
            return {"error": "%s:%s" % (type(exc).__name__, str(exc)[:160])}
        self._oracle_region_cache = region
        return region

    def _hold_probe(self, run):
        """Old-region probability oracle: complete voxel states + log-odds.

        One lease pages the WHOLE explicit region (every cell including
        unknown and out-of-domain ones) plus the full surface as a secondary
        geometric summary. Any page error, empty-before-done page, repeated
        cursor, identity mismatch, or failed release is an error — never a
        small clean hash that would compare equal to another error.
        """
        session = run.session
        region = self._oracle_region(run)
        if "error" in region:
            return dict(region)
        try:
            from uld_occ_observer.srv import GetSnapshot, ReadCells, ReadSurface, ReleaseSnapshot
            started_ns = time.monotonic_ns()
            request = GetSnapshot.Request()
            request.expected_epoch = 1
            # A hold deliberately ages the map for tens of seconds: the probe
            # reads the stale-but-consistent snapshot instead of refusing it.
            request.max_age_ms = 300000
            request.allow_stale = True
            lease = session.call(session.snapshot, request, 5)
            if lease.status != "OK":
                return {"error": "snapshot_%s" % lease.status}
            identity = executor._identity_tuple(executor._field(lease, "identity"))
            cells = session.node.create_client(ReadCells, "/uld_occ_observer/read_cells")
            surface = session.node.create_client(ReadSurface, "/uld_occ_observer/read_surface")
            release_client = session.node.create_client(
                ReleaseSnapshot, "/uld_occ_observer/release_snapshot")
            for client in (cells, surface, release_client):
                session.wait_service(client, timeout=5)
            seen_cursors = set()
            states = []
            log_odds = []
            cursor = 0
            pages = 0
            while True:
                page_request = ReadCells.Request()
                page_request.handle = lease.handle
                page_request.begin = list(region["begin"])
                page_request.end = list(region["end"])
                page_request.cursor = int(cursor)
                page_request.limit = 65536
                page = session.call(cells, page_request, 10)
                if page.status != "OK":
                    return {"error": "cells_page_%s" % page.status}
                if int(cursor) in seen_cursors:
                    return {"error": "cells_page_repeat_cursor"}
                seen_cursors.add(int(cursor))
                page_states = list(page.states or [])
                page_odds = list(page.log_odds or [])
                if len(page_states) != len(page_odds):
                    return {"error": "cells_page_field_mismatch"}
                pages += 1
                states.extend(page_states)
                log_odds.extend(round(float(value), 6) for value in page_odds)
                if page.done:
                    break
                if not page_states:
                    return {"error": "cells_page_empty_before_done"}
                cursor = int(page.next_cursor)
            # Secondary geometric summary: full-grid surface columns filtered
            # to the same region.
            columns = []
            total_columns = 0
            surface_cursor = 0
            surface_pages = 0
            while True:
                page_request = ReadSurface.Request()
                page_request.handle = lease.handle
                page_request.cursor = int(surface_cursor)
                page_request.limit = 512
                page = session.call(surface, page_request, 10)
                if page.status != "OK":
                    return {"error": "surface_page_%s" % page.status}
                xs = executor._field(page, "x")
                fields = {name: executor._field(page, name) for name in
                          ("y", "active_count", "known_count", "has_occupied", "top_z")}
                if xs is None or any(fields[name] is None for name in fields):
                    return {"error": "surface_page_missing_fields"}
                total_columns += len(xs)
                for index, x in enumerate(xs):
                    if int(x) < _ORACLE_MIN_CELL_X:
                        continue
                    columns.append([
                        int(x), int(fields["y"][index]),
                        int(fields["active_count"][index]),
                        int(fields["known_count"][index]),
                        bool(fields["has_occupied"][index]),
                        float(fields["top_z"][index]),
                    ])
                surface_pages += 1
                if page.done:
                    break
                if not xs:
                    return {"error": "surface_page_empty_before_done"}
                surface_cursor = int(page.next_cursor)
            release_request = ReleaseSnapshot.Request()
            release_request.handle = lease.handle
            release_status = session.call(release_client, release_request, 5).status
            if release_status != "OK":
                return {"error": "release_%s" % release_status}
            if len(states) != region["cells"]:
                return {"error": "cells_incomplete:%d/%d" % (len(states), region["cells"])}
            # Shape cross-check: the full-grid surface must carry exactly
            # shape.x*shape.y columns, or the derived bounds are wrong.
            shape = region["shape"]
            if total_columns != shape[0] * shape[1]:
                return {"error": "shape_mismatch:%d/%d"
                        % (total_columns, shape[0] * shape[1])}
            return {
                "states_hash": executor._content_sha256(states),
                "log_odds_hash": executor._content_sha256(log_odds),
                "cells": len(states),
                "surface_hash": executor._content_sha256(sorted(columns)),
                "surface_columns": len(columns),
                "region": {"begin": region["begin"], "end": region["end"],
                           "shape": region["shape"], "resolution_m": region["resolution_m"]},
                "identity": {"epoch": identity[0], "map_revision": identity[1],
                             "observed_revision": identity[2],
                             "last_integrated_stamp_ns": identity[3] if len(identity) > 3 else None},
                "release": release_status,
                "pages": pages,
                "pages_done": True,
                "probe_mono_ns": started_ns,
                "duration_ms": (time.monotonic_ns() - started_ns) / 1e6,
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
        try:
            import rclpy
        except ImportError:
            return
        if rclpy.ok():
            rclpy.shutdown()
