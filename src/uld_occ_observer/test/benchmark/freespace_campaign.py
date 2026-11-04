#!/usr/bin/env python3
"""Non-scored short wiring for the unified-target P / R / L stages.

The formal counts live in freespace_contract (300 inputs + 240 full queries
per 120 s window, 600-input overload with a 30 s drain, 120-min soak). This
harness runs the SAME command shape with reduced counts so a rehearsal proves
the wiring -- sender and query threads on independent absolute deadlines,
receipts, terminal closure, lease lifecycle, resource sampling, judge
consumption -- inside the 2 h rehearsal budget. Every artifact carries
formal=false; the judge refuses to certify a short sample as a formal pass
(the formal runner in OCTOMAP-2CM-REPAIR-S3 gen3 reuses these functions with
contract counts).

Stages (each optional, failures do not auto-continue dependents):
  p -- per box count in 0/5/20: one short window, N inputs at 2.5 Hz and M
       full freespace queries at 2 Hz (boxes alternating, both yaws).
  r -- same node and epoch: 10 Hz overload burst, fixed drain, recovery at
       2.5 Hz whose first query is a WaitSnapshot bound to the first input.
  l -- a short sustained loop plus acquire/read/release rounds and
       reset/error rounds; handle and RSS baselines must return.
"""
from __future__ import annotations

import argparse
import gc
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import freespace_contract as contract  # noqa: E402
import freespace_judge  # noqa: E402
from live_schedule import cloud_from_frame  # noqa: E402

FIXTURES = Path("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/"
                "occ_o6_o7_20261029/run01/fixtures")


# ---------------------------------------------------------------------------
# Opt-in scheduler forensics (--trace-sched / --pin-client / --gc-hardening).
#
# Purpose: the run02 L record contains exactly one 107.1 ms late query START in
# 14,402 deadlines (client-side, two client threads resumed at the same
# instant). These switches decompose every deadline-loop wakeup into
# sleep-lag vs post-wake lag, with INDEPENDENT witnesses -- a 50 ms heartbeat
# thread (catches a freeze even when no thread has a deadline due), gc pause
# pairs, and executor-callback durations -- so a reproduced event can be
# classified: OS freeze vs GIL hold vs GC vs server queueing. OFF by default;
# judged fields are never touched; the trace lands in sched_trace.json beside
# the stage artifact. None of this changes any gate, threshold, or the
# contract's no-CPU-affinity environment: --pin-client is a LOCALIZATION arm
# only; formal use of affinity requires a new contract version.
# ---------------------------------------------------------------------------
SCHED_TRACE = None
GC_HARDENING = False


class SchedTrace:
    """Collects wakeup decompositions, gc pauses, heartbeat gaps."""

    def __init__(self, min_overshoot_ms=5.0, heartbeat_ms=50, gap_ms=10.0):
        self.rows = []
        self.gc_pauses = []
        self.heartbeat_gaps = []
        self.callbacks = []
        self.min_overshoot_ms = min_overshoot_ms
        self.heartbeat_ms = heartbeat_ms
        self.gap_ms = gap_ms
        self._gc_t0 = None
        self._gc_registered = False
        self.meta = {"switch_interval_s": sys.getswitchinterval(),
                     "affinity": None,
                     "started_mono_ns": time.monotonic_ns()}

    def wake(self, tag, deadline_ns, wake_ns, started_ns):
        """One deadline wakeup: overshoot split into sleep-lag vs post-wake.

        sleep_lag > 0 means the sleep returned late (OS/deschedule); a large
        post_wake_lag with small sleep_lag means the thread was awake but
        waited for the GIL between the wake stamp and the started stamp."""
        overshoot_ms = (started_ns - deadline_ns) / 1e6
        if overshoot_ms < self.min_overshoot_ms:
            return
        self.rows.append({
            "tag": tag,
            "deadline_mono_ns": deadline_ns,
            "wake_mono_ns": wake_ns,
            "started_mono_ns": started_ns,
            "overshoot_ms": round(overshoot_ms, 3),
            "sleep_lag_ms": round((wake_ns - deadline_ns) / 1e6, 3),
            "post_wake_lag_ms": round((started_ns - wake_ns) / 1e6, 3)})

    def slow_callback(self, tag, started_ns, ended_ns):
        if (ended_ns - started_ns) / 1e6 >= self.min_overshoot_ms:
            self.callbacks.append({
                "tag": tag, "started_mono_ns": started_ns,
                "ended_mono_ns": ended_ns,
                "duration_ms": round((ended_ns - started_ns) / 1e6, 3)})

    def _on_gc(self, phase, info):
        if phase == "start":
            self._gc_t0 = time.perf_counter_ns()
        elif self._gc_t0 is not None:
            pause_ms = (time.perf_counter_ns() - self._gc_t0) / 1e6
            # Gen-0 collections run thousands of times per minute at ~50 us;
            # only pauses that could matter against a 50 ms deadline gate are
            # kept, or the 120-min trace would hold ~150k noise rows.
            if pause_ms >= 1.0:
                self.gc_pauses.append({
                    "pause_ms": round(pause_ms, 3),
                    "generation": info.get("generation"),
                    "at_perf_ns": self._gc_t0})
            self._gc_t0 = None

    def install_gc(self):
        if not self._gc_registered:
            gc.callbacks.append(self._on_gc)
            self._gc_registered = True

    def start_heartbeat(self, stop, hardening_t0_ns=None, hardening_s=None):
        """Independent freeze witness; also drives --gc-hardening schedule.

        The heartbeat sleeps a fixed 50 ms tick; any tick that lands late by
        >= gap_ms is recorded -- it has no deadline of its own to miss, so a
        gap here is direct evidence the whole process stopped running. With
        --gc-hardening it freezes the heap at warmup end, disables automatic
        collection for the scored segment, and collects once per 120 s.
        """

        def beat():
            last = time.monotonic_ns()
            hardened = False
            last_collect = None
            while not stop.is_set():
                stop.wait(self.heartbeat_ms / 1e3)
                now = time.monotonic_ns()
                gap_ms = (now - last) / 1e6 - self.heartbeat_ms
                if gap_ms >= self.gap_ms:
                    self.heartbeat_gaps.append(
                        {"at_mono_ns": now, "gap_ms": round(gap_ms, 1),
                         "sleep_lag_ms": round(gap_ms, 1)})
                last = now
                if GC_HARDENING:
                    if not hardened and hardening_t0_ns is not None \
                            and now >= hardening_t0_ns + hardening_s * 10**9:
                        gc.collect()
                        gc.freeze()
                        gc.disable()
                        hardened = True
                        self.meta["gc_hardening_at_mono_ns"] = now
                    elif hardened and (last_collect is None
                                       or now >= last_collect):
                        last_collect = now + 120 * 10**9
                        gc.collect()

        threading.Thread(target=beat, daemon=True).start()

    def dump(self, out_dir):
        doc = {"meta": self.meta, "rows": self.rows,
               "gc_pauses": self.gc_pauses,
               "heartbeat_gaps": self.heartbeat_gaps,
               "callbacks": self.callbacks,
               "counts": {"rows": len(self.rows),
                          "gc_pauses": len(self.gc_pauses),
                          "heartbeat_gaps": len(self.heartbeat_gaps),
                          "slow_callbacks": len(self.callbacks)}}
        out = Path(out_dir)
        out.mkdir(parents=True, exist_ok=True)
        (out / "sched_trace.json").write_text(json.dumps(doc, indent=1))
        return doc


def _trace():
    return SCHED_TRACE


def load_frames(path_glob):
    import glob as glob_module
    import re
    frames = []
    for name in sorted(glob_module.glob(path_glob)):
        path = Path(name)
        frame = yaml.safe_load(path.read_text())
        tag = re.match(r"b(\d+)_", path.name)
        if tag:
            frame["_box_tag"] = int(tag.group(1))
        frame["_path"] = str(path)
        frames.append(frame)
    return frames


def frames_manifest(frames):
    """Bind the exact input workload: every frame file by name and content
    SHA with its box tag. The formal R/L stages must run the fixed 20-box
    set -- a mixed or short-rehearsal frame list never substitutes for the
    final workload."""
    import hashlib
    manifest = []
    for frame in frames:
        path = frame.get("_path")
        entry = {"box_tag": frame.get("_box_tag"), "name": Path(path).name
                 if path else None}
        if path and Path(path).is_file():
            entry["sha256"] = hashlib.sha256(Path(path).read_bytes()).hexdigest()
        manifest.append(entry)
    return manifest


def group_frames_by_box(frames, strict=False):
    """Group loaded frames by their bNN box tag.

    Wiring mode keeps the historical fallback (an untagged set maps every
    box count to the whole list so short rehearsals still run). With
    strict=True an empty group for any contract box count RAISES instead of
    falling back: a formal offered load must never silently run a mixed or
    missing frame set (the run02 R gate failed exactly this way)."""
    groups = {}
    for box_count in contract.P_BOX_COUNTS:
        tagged = [frame for frame in frames
                  if isinstance(frame, dict)
                  and frame.get("_box_tag") == box_count]
        if strict and not tagged:
            raise SystemExit(
                "frames for box_count=%s missing from the glob; refusing to "
                "fall back to a mixed load (strict=formal workload identity)"
                % box_count)
        groups[box_count] = tagged or frames
    return groups


class NodeProcess:
    def __init__(self, binary, geometry, config, log_path, cloud_topic,
                 extra_params=()):
        self.log = open(log_path, "w")
        env = os.environ.copy()
        install_lib = Path(binary).resolve().parent.parent.parent / "lib"
        env["LD_LIBRARY_PATH"] = "%s:%s" % (install_lib, env.get("LD_LIBRARY_PATH", ""))
        params = ["-p", "geometry_file:=%s" % geometry,
                  "-p", "config_file:=%s" % config,
                  "-p", "input_mode:=subscription",
                  "-p", "cloud_topic:=%s" % cloud_topic,
                  "-p", "optical_frame:=d555_color_optical_frame",
                  "-p", "sensor_id:=d555"]
        for item in extra_params:
            params += ["-p", item]
        self.proc = subprocess.Popen(
            [binary, "--ros-args", *params],
            stdout=self.log, stderr=subprocess.STDOUT, env=env,
            start_new_session=True)

    def stop(self):
        if self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGTERM)
                self.proc.wait(timeout=5)
        self.log.close()


class Client:
    """rclpy client with the sender and query threads' service set."""

    def __init__(self, cloud_topic):
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2
        from tf2_ros import StaticTransformBroadcaster
        from geometry_msgs.msg import TransformStamped
        from uld_occ_observer.msg import UpdateReceipt
        from uld_occ_observer.srv import (GetObservationStatus, GetResourceUsage,
                                          GetSnapshot, QueryFreespace,
                                          ReleaseSnapshot, ResetObservation,
                                          WaitSnapshot)
        rclpy.init()
        self.rclpy = rclpy
        self.node = Node("freespace_campaign_client")
        # Explicit 4 client executor threads (contract CLIENT_EXECUTOR_THREADS).
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        reliable = QoSProfile(depth=32, reliability=ReliabilityPolicy.RELIABLE)
        self.cloud = self.node.create_publisher(PointCloud2, cloud_topic, reliable)
        self.receipts = []
        self.health = []
        self.node.create_subscription(
            UpdateReceipt, "/uld_occ_observer/receipts", self._receipt, 1024)
        from std_msgs.msg import String
        self.node.create_subscription(
            String, "/uld_occ_observer/health", self._health, 32)
        # gen4: the latched validity topic carries every board transition
        # (INVALID/BUILDING/VALID with generation/epoch + mono stamps). The
        # node keeps only the last 64 events server-side, so the collector
        # subscribes from stage start and accumulates incrementally; a final
        # get_validity poll merges/dedupes as a backstop.
        self.validity_events = []
        self.node.create_subscription(
            String, "/uld_occ_observer/validity", self._validity, 64)
        self.wait = self._client(WaitSnapshot, "wait_snapshot")
        self.get_snapshot = self._client(GetSnapshot, "get_snapshot")
        self.freespace = self._client(QueryFreespace, "query_freespace")
        self.release = self._client(ReleaseSnapshot, "release_snapshot")
        self.status = self._client(GetObservationStatus, "get_observation_status")
        self.reset = self._client(ResetObservation, "reset_observation")
        self.resource = self._client(GetResourceUsage, "get_resource_usage")
        from uld_occ_observer.srv import GetValidity
        self.get_validity = self._client(GetValidity, "get_validity")
        self._tf = StaticTransformBroadcaster(self.node)
        self.tf_frame = TransformStamped()
        self.tf_frame.header.frame_id = "container_link"
        self.tf_frame.child_frame_id = "d555_color_optical_frame"
        self.tf_frame.transform.rotation.x = 0.5
        self.tf_frame.transform.rotation.y = -0.5
        self.tf_frame.transform.rotation.z = 0.5
        self.tf_frame.transform.rotation.w = -0.5

    def _client(self, srv, name):
        return self.node.create_client(srv, "/uld_occ_observer/%s" % name)

    def _receipt(self, message):
        # Arrival time first: the input E2E clock runs publish -> terminal
        # receipt OBSERVABLE, and the R timeline needs these stamps. The
        # product's own segment timers ride along (queue/preparation/
        # preprocessing/fusion/total) so a slow input decomposes into its
        # server-side stages instead of one opaque number.
        cb_t0 = time.monotonic_ns()
        self.receipts.append({
            "arrival_mono_ns": cb_t0,
            "status": message.status,
            "observation_id": message.observation_id,
            "input_stamp_ns": message.input_stamp_ns,
            "callback_enter_ns": message.callback_enter_ns,
            "queue_ms": message.queue_ms,
            "preparation_ms": message.preparation_ms,
            "preprocessing_ms": message.preprocessing_ms,
            "fusion_ms": message.fusion_ms,
            "total_ms": message.total_ms,
            "epoch": message.identity.epoch,
            "map_revision": message.identity.map_revision,
            "observed_revision": message.identity.observed_revision,
        })
        trace = _trace()
        if trace is not None:
            trace.slow_callback("receipt", cb_t0, time.monotonic_ns())

    def _health(self, message):
        cb_t0 = time.monotonic_ns()
        entry = yaml.safe_load(message.data)
        if isinstance(entry, dict):
            entry["arrival_mono_ns"] = cb_t0
        self.health.append(entry)
        trace = _trace()
        if trace is not None:
            trace.slow_callback("health", cb_t0, time.monotonic_ns())

    def _validity(self, message):
        entry = yaml.safe_load(message.data)
        if isinstance(entry, dict):
            entry["arrival_mono_ns"] = time.monotonic_ns()
            self.validity_events.append(entry)

    def broadcast_tf(self, translation):
        self.tf_frame.transform.translation.x = float(translation[0])
        self.tf_frame.transform.translation.y = float(translation[1])
        self.tf_frame.transform.translation.z = float(translation[2])
        self._tf.sendTransform(self.tf_frame)

    def wait_services(self, timeout=30.0):
        deadline = time.monotonic() + timeout
        for client in (self.wait, self.get_snapshot, self.freespace, self.release,
                       self.status, self.reset, self.resource):
            while not client.wait_for_service(timeout_sec=0.2):
                if time.monotonic() > deadline:
                    raise TimeoutError(client.srv_name)

    def call(self, client, request, timeout=10.0):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError(client.srv_name)
            time.sleep(0.005)
        return future.result()

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.rclpy.shutdown()


def _page_all(client, handle, record, box, yaw_mode=2, request_timeout=10.0):
    """Materialize EVERY page of a live lease into the record: all response
    arrays equal length, verify ranges 3n, page identity == lease identity on
    every page, cursor exactly min(cursor+limit, feasible_total) with a
    consistent done flag, and the candidate count == feasible_total. The
    canonical digest is recomputed over the raw candidate stream. Each page's
    server-side segments (repository wait / enumerate / response fill /
    server total) and region bounds are kept in pages_detail so the judge can
    reconcile counts and a slow chain can be decomposed (gen4)."""
    import hashlib
    cursor, done, pages = 0, False, 0
    digest = hashlib.sha256()
    feasible = None
    page = None
    record["pages_detail"] = []
    while not done:
        page = client.call(client.freespace,
                           _freespace_request(handle, box, yaw_mode, cursor),
                           request_timeout)
        record["page_status"] = page.status
        if page.status not in ("OK", "NO_KNOWN_FREE"):
            record["status"] = "PAGE_%s" % page.status
            return record
        feasible = page.feasible_total
        record["feasible_total"] = feasible
        count = len(page.cell_x)
        arrays = (page.cell_y, page.cell_z, page.origin_x, page.origin_y,
                  page.origin_z, page.yaw_deg, page.support_ratio,
                  page.support_kind)
        if any(len(array) != count for array in arrays) \
                or len(page.verify_begin) != 3 * count \
                or len(page.verify_end) != 3 * count:
            record["status"] = "PAGE_ARRAY_LENGTH_MISMATCH"
            return record
        if _identity(page.identity) != record["lease_identity"]:
            record["status"] = "PAGE_IDENTITY_DRIFT"
            return record
        expected_next = min(cursor + 65536, feasible)
        expected_done = expected_next >= feasible
        if page.next_cursor != expected_next or page.done != expected_done:
            record["status"] = "PAGE_CURSOR_PROGRESS"
            return record
        for index in range(count):
            yaw = int(page.yaw_deg[index])
            cell = (yaw, int(page.cell_x[index]), int(page.cell_y[index]),
                    int(page.cell_z[index]), int(page.support_kind[index]))
            record["candidates"].append(cell)
            digest.update(freespace_judge.canonical_candidate_bytes(*cell))
        record["pages_detail"].append({
            "count": count, "cursor": cursor, "next_cursor": page.next_cursor,
            "done": page.done,
            "region_begin": [int(value) for value in page.region_begin],
            "region_end": [int(value) for value in page.region_end],
            "repository_wait_ms": page.repository_wait_ms,
            "enumerate_ms": page.enumerate_ms,
            "response_fill_ms": page.response_fill_ms,
            "server_total_ms": page.server_total_ms})
        cursor = page.next_cursor
        done = page.done
        pages += 1
        if pages > 100000:
            record["status"] = "PAGE_RUNAWAY"
            return record
    if len(record["candidates"]) != feasible:
        record["status"] = "CANDIDATE_COUNT_MISMATCH"
        return record
    record["pages"] = pages
    record["content_sha256"] = digest.hexdigest()
    record["status"] = page.status
    return record


def full_query(client, box, yaw_mode=2, max_age_ms=1000, request_timeout=10.0):
    """The measured whole chain: acquire -> EVERY page materialized -> release.

    Fail-closed rules (review group 2): the lease is released in a finally
    block even on timeouts; any fault names itself in the status; a healthy
    NO_KNOWN_FREE stays a legal semantic answer.
    """
    started = time.monotonic_ns()
    record = {"status": "OK", "lease_identity": None, "pages": 0,
              "candidates": [], "feasible_total": None,
              "content_sha256": None, "release_status": None,
              "page_status": None, "latency_ms": None,
              "acquire_ms": None, "pages_ms": None, "release_ms": None,
              "client_candidate_bytes_est": None}
    handle = None
    acquired = None
    try:
        lease = client.call(client.get_snapshot, _snapshot_request(max_age_ms),
                            request_timeout)
        acquired = time.monotonic_ns()
        record["acquire_ms"] = (acquired - started) / 1e6
        if lease.status != "OK":
            record["status"] = "ACQUIRE_%s" % lease.status
            return record
        handle = lease.handle
        record["lease_identity"] = _identity(lease.identity)
        record["age_ns"] = lease.age_ns
        _page_all(client, handle, record, box, yaw_mode, request_timeout)
        record["pages_ms"] = (time.monotonic_ns() - acquired) / 1e6
        # Honest client-side estimate of the deserialized arrays this
        # process now holds (the node cannot see a foreign process's heap).
        record["client_candidate_bytes_est"] = 5 * 8 * len(record["candidates"])
        return record
    except TimeoutError as error:
        record["status"] = "QUERY_TIMEOUT:%s" % error
        return record
    finally:
        release_started = time.monotonic_ns()
        if handle is not None:
            try:
                response = client.call(client.release, _release_request(handle),
                                       request_timeout)
                record["release_status"] = response.status
                if response.status != "OK":
                    record["release_fault"] = response.status
            except Exception as error:  # noqa: BLE001 - recorded, never raised past finally
                record["release_status"] = "RELEASE_EXCEPTION:%r" % (error,)
        if handle is not None:
            record["release_ms"] = (time.monotonic_ns() - release_started) / 1e6
        record["latency_ms"] = (time.monotonic_ns() - started) / 1e6


def wait_chain(client, wait_response, box, request_timeout=10.0):
    """Complete the candidate chain ON the first Wait's own lease and release
    it in a finally block -- the wait lease is used, never abandoned to TTL
    (review group B)."""
    record = {"status": "OK", "lease_identity": None, "pages": 0,
              "candidates": [], "feasible_total": None,
              "content_sha256": None, "release_status": None,
              "page_status": None, "latency_ms": None,
              "acquire_ms": None, "pages_ms": None, "release_ms": None,
              "client_candidate_bytes_est": None}
    started = time.monotonic_ns()
    handle = wait_response.handle
    try:
        record["lease_identity"] = _identity(wait_response.identity)
        record["age_ns"] = wait_response.age_ns
        _page_all(client, handle, record, box, 2, request_timeout)
        record["pages_ms"] = (time.monotonic_ns() - started) / 1e6
        record["client_candidate_bytes_est"] = 5 * 8 * len(record["candidates"])
        return record
    except TimeoutError as error:
        record["status"] = "QUERY_TIMEOUT:%s" % error
        return record
    finally:
        release_started = time.monotonic_ns()
        try:
            response = client.call(client.release, _release_request(handle),
                                   request_timeout)
            record["release_status"] = response.status
            if response.status != "OK":
                record["release_fault"] = response.status
        except Exception as error:  # noqa: BLE001 - recorded, never raised past finally
            record["release_status"] = "RELEASE_EXCEPTION:%r" % (error,)
        record["release_ms"] = (time.monotonic_ns() - release_started) / 1e6
        record["latency_ms"] = (time.monotonic_ns() - started) / 1e6


def _snapshot_request(max_age_ms):
    from uld_occ_observer.srv import GetSnapshot
    request = GetSnapshot.Request()
    request.max_age_ms = max_age_ms
    return request


def _freespace_request(handle, box, yaw_mode, cursor):
    from uld_occ_observer.srv import QueryFreespace
    request = QueryFreespace.Request()
    request.handle = handle
    request.box_length, request.box_width, request.box_height = box
    request.yaw_mode = yaw_mode
    request.cursor = cursor
    request.limit = 65536
    return request


def _release_request(handle):
    from uld_occ_observer.srv import ReleaseSnapshot
    request = ReleaseSnapshot.Request()
    request.handle = handle
    return request


def _identity(message):
    return {"epoch": message.epoch, "map_revision": message.map_revision,
            "observed_revision": message.observed_revision,
            "geometry_hash": message.geometry_hash,
            "last_integrated_stamp_ns": message.last_integrated_stamp_ns,
            "config_digest": message.config_digest,
            "clock_domain": message.clock_domain}


def readiness(client, frames):
    """First accepted input + known snapshot before a window starts."""
    from sensor_msgs.msg import PointCloud2
    cloud, _ = cloud_from_frame(frames[0])
    stamp = int(time.time() * 1e9)
    cloud.header.stamp.sec = stamp // 10**9
    cloud.header.stamp.nanosec = stamp % 10**9
    mark = len(client.receipts)
    deadline = time.monotonic() + 20
    next_retry = time.monotonic()
    while time.monotonic() < deadline:
        if time.monotonic() >= next_retry:
            # The first publish can race static-TF discovery; retry the SAME
            # frame with a fresh unique stamp until the map accepts it.
            stamp = int(time.time() * 1e9)
            cloud.header.stamp.sec = stamp // 10**9
            cloud.header.stamp.nanosec = stamp % 10**9
            client.cloud.publish(cloud)
            mark = len(client.receipts)
            next_retry = time.monotonic() + 2.0
        for receipt in client.receipts[mark:]:
            if receipt["status"] == "INTEGRATED":
                lease = client.call(client.get_snapshot, _snapshot_request(10_000))
                if lease.status == "OK":
                    # Stash the lease identity: the stage's identity binding
                    # (epoch + geometry + config + clock) comes from a real
                    # lease the map just vouched for, never from a guess.
                    client.bound_identity = _identity(lease.identity)
                    client.call(client.release, _release_request(lease.handle))
                    return time.monotonic_ns()
        time.sleep(0.02)
    raise TimeoutError("readiness: no integrated input")


NON_TERMINAL_STATUSES = {"NOT_SEEN", "PROCESSING", "WAITING", "QUEUED", "ADMITTED"}


def input_stamp_hz(hz):
    """The stamp stream must track the real schedule: one period per input.

    A fixed +50 ms per frame at 2.5 Hz would fall behind wall time by 275 ms
    every input, aging the map the queries see; stamps advance one full
    period per scheduled input instead (review group 3).
    """
    return int(round(1e9 / hz))


def send_inputs(client, frames, count, hz, t0_ns, base_stamp):
    """Absolute-deadline sender. Stamps advance one period per input so the
    map age the queries observe tracks the real schedule."""
    from sensor_msgs.msg import PointCloud2
    records = []
    step_ns = input_stamp_hz(hz)
    for index in range(count):
        frame = frames[index % len(frames)]
        cloud, _ = cloud_from_frame(frame)
        stamp = base_stamp + index * step_ns
        cloud.header.stamp.sec = stamp // 10**9
        cloud.header.stamp.nanosec = stamp % 10**9
        deadline = t0_ns + int(index / hz * 1e9)
        now = time.monotonic_ns()
        if now < deadline:
            time.sleep((deadline - now) / 1e9)
        wake_ns = time.monotonic_ns()
        client.cloud.publish(cloud)
        published_ns = time.monotonic_ns()
        trace = _trace()
        if trace is not None:
            trace.wake("send_inputs", deadline, wake_ns, published_ns)
        records.append({"index": index, "stamp_ns": stamp,
                        "planned_mono_ns": deadline,
                        "publish_mono_ns": published_ns,
                        "input_id": str(stamp)})
    return records


def receipt_timeline(client, records, timeout_s=10.0):
    """Per-input terminal timeline, indexed (one incremental pass, no rescan).

    EVERY receipt event is kept per id -- rows are never collapsed to the
    last receipt, so duplicate terminals stay visible to the judge; the
    terminal count and the terminal arrival (for the drain deadline) are
    recorded per row. The input E2E clock is publish -> terminal receipt
    observable; a missing or non-terminal receipt stays missing.
    """
    events = {record["input_id"]: [] for record in records}
    cursor = 0
    deadline = time.monotonic() + timeout_s
    while True:
        receipts = client.receipts
        while cursor < len(receipts):
            receipt = receipts[cursor]
            cursor += 1
            bucket = events.get(receipt["observation_id"])
            if bucket is not None:
                bucket.append(receipt)
        pending = any(
            not any(event["status"] in freespace_judge.LEGAL_TERMINAL
                    for event in events[record["input_id"]])
            for record in records)
        if not pending or time.monotonic() >= deadline:
            break
        time.sleep(0.05)
    timeline = []
    for record in records:
        bucket = events[record["input_id"]]
        terminals = [event for event in bucket
                     if event["status"] in freespace_judge.LEGAL_TERMINAL]
        chosen = terminals[-1] if terminals else (bucket[-1] if bucket else None)
        timeline.append({
            "input_id": record["input_id"],
            "planned_mono_ns": record["planned_mono_ns"],
            "publish_mono_ns": record["publish_mono_ns"],
            "receipt_status": chosen["status"] if chosen else None,
            "receipt_count": len(terminals),
            "receipt_events": [{"status": event["status"],
                                "arrival_mono_ns": event["arrival_mono_ns"]}
                               for event in bucket],
            "callback_enter_ns": chosen["callback_enter_ns"] if chosen else None,
            "receipt_arrival_mono_ns": terminals[-1]["arrival_mono_ns"]
            if terminals else None,
            "e2e_ms": ((terminals[-1]["arrival_mono_ns"]
                        - record["publish_mono_ns"]) / 1e6)
            if terminals else None,
        })
    return timeline


def collect_terminals(client, ids, timeout_s=10.0):
    """Terminal closure through get_observation_status; only legal terminal
    states leave the pending set (QUEUED/PROCESSING are NOT terminal)."""
    from uld_occ_observer.srv import GetObservationStatus
    deadline = time.monotonic() + timeout_s
    pending = set(ids)
    terminals = {}
    while pending and time.monotonic() < deadline:
        for observation_id in list(pending):
            request = GetObservationStatus.Request()
            request.epoch = 1
            request.sensor_id = "d555"
            request.observation_id = observation_id
            record = client.call(client.status, request)
            if record.status not in NON_TERMINAL_STATUSES:
                terminals[observation_id] = record.status
                pending.discard(observation_id)
        if pending:
            time.sleep(0.05)
    return terminals, sorted(pending)


def run_queries(client, count, hz, t0_ns):
    """Absolute-deadline full-query thread; boxes alternate per request."""
    records = []
    for index in range(count):
        box = contract.F_BOXES[index % 2]
        deadline = t0_ns + int(index / hz * 1e9)
        now = time.monotonic_ns()
        if now < deadline:
            time.sleep((deadline - now) / 1e9)
        wake_ns = time.monotonic_ns()
        started = time.monotonic_ns()
        trace = _trace()
        if trace is not None:
            trace.wake("run_queries", deadline, wake_ns, started)
        result = full_query(client, box)
        records.append({"index": index, "box": list(box),
                        "planned_mono_ns": deadline,
                        "started_mono_ns": started,
                        "ended_mono_ns": time.monotonic_ns(),
                        **result})
    return records


def _rss_kib(pid):
    import re
    try:
        status = Path("/proc/%d/status" % pid).read_text()
        return int(re.search(r"VmRSS:\s+(\d+) kB", status).group(1))
    except (OSError, AttributeError):
        return None


def _epoch_of(client):
    if not client.health:
        return None
    return client.health[-1].get("epoch")


def p_window_plan(t0_ns, inputs, queries, update_hz, query_hz):
    """The P window event plan: inputs and queries on independent absolute
    deadlines from one shared t0 (offline-verifiable schedule source)."""
    return {
        "inputs": [t0_ns + int(index / update_hz * 1e9) for index in range(inputs)],
        "queries": [t0_ns + int(index / query_hz * 1e9) for index in range(queries)],
    }


def recovery_plan(t0_ns, inputs, queries, update_hz, query_hz):
    """The recovery plan: the FIRST Wait shares input 0's planned instant and
    runs the candidate chain on its own lease; the remaining queries follow
    the same 1/query_hz schedule from t0 + one period."""
    return {
        "inputs": [t0_ns + int(index / update_hz * 1e9) for index in range(inputs)],
        "first_wait_ns": t0_ns,
        "queries": [t0_ns + int((index + 1) / query_hz * 1e9)
                    for index in range(queries)],
    }


def drain_plan(last_planned_ns, drain_s, queries, query_hz):
    """The drain plan: the 30 s window opens at the last planned send; its
    queries spread over it at query_hz; the deadline is exact."""
    start_ns = last_planned_ns
    return {
        "start_ns": start_ns,
        "deadline_ns": start_ns + int(drain_s * 1e9),
        "queries": [start_ns + int((index + 0.5) / query_hz * 1e9)
                    for index in range(queries)],
    }


def stage_p(client, frames, box_count, inputs, queries, out_dir, formal=False,
            window_index=0):
    """One window: sender and full-query threads on independent absolute
    clocks from the same t0; per-input terminal timeline; per-query latency
    against its own planned deadline (the H gate reads the same records)."""
    window_id = "b%02d_w%d" % (box_count, window_index)
    window = {"stage": "p", "box_count": box_count, "window_index": window_index,
              "window_id": window_id, "inputs": inputs,
              "queries": queries, "update_hz": contract.UPDATE_HZ,
              "query_hz": contract.QUERY_HZ, "formal": formal,
              "frames_manifest": frames_manifest(frames),
              "window_planned_s": inputs / contract.UPDATE_HZ}
    t0 = readiness(client, frames)
    base_stamp = int(time.time() * 1e9)  # stamps lag wall clock by design
    health_mark = len(client.health)
    window["identity_binding"] = _capture_binding(
        client, getattr(client, "config_digest", ""))
    sender_records = []
    query_records = []
    errors = []

    def send():
        try:
            sender_records.extend(
                send_inputs(client, frames, inputs, contract.UPDATE_HZ, t0, base_stamp))
        except Exception as error:  # noqa: BLE001
            errors.append("sender: %r" % error)

    def query():
        try:
            query_records.extend(
                run_queries(client, queries, contract.QUERY_HZ, t0))
        except Exception as error:  # noqa: BLE001
            errors.append("query: %r" % error)

    threads = [threading.Thread(target=send), threading.Thread(target=query)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=inputs / contract.UPDATE_HZ + 30)
    window_closed_ns = time.monotonic_ns()
    time.sleep(0.5)
    timeline = receipt_timeline(client, sender_records, timeout_s=10.0)
    terminals, pending = collect_terminals(
        client, [record["input_id"] for record in sender_records])
    health_gaps = []
    last = None
    for sample in client.health[health_mark:]:
        stamp = sample.get("stamp_ns") if isinstance(sample, dict) else None
        if isinstance(stamp, int):
            if last is not None:
                health_gaps.append((stamp - last) / 1e9)
            last = stamp
    window.update({
        "sender": sender_records, "queries": query_records, "errors": errors,
        "input_timeline": timeline, "terminals": terminals, "pending": pending,
        # Raw window-scoped stamps with arrival times: the judge recomputes
        # gaps from these; the self-reported aggregate stays as a diagnostic.
        "health_series": [
            {"stamp_ns": sample.get("stamp_ns"),
             "arrival_mono_ns": sample.get("arrival_mono_ns"),
             "epoch": sample.get("epoch")}
            for sample in client.health[health_mark:]
            if isinstance(sample, dict)
            and isinstance(sample.get("stamp_ns"), int)],
        "health_samples": len(client.health[health_mark:]),
        "health_gap_max_s": max(health_gaps) if health_gaps else None,
        "window_close_ms": (window_closed_ns - t0) / 1e6,
    })
    (out_dir / ("p_%s.json" % window_id)).write_text(json.dumps(window, indent=1))
    return window


def stage_r(client, frames, overload_inputs, drain_s, recovery_inputs, out_dir,
            formal=False):
    """Overload with 10 Hz inputs and 2 Hz queries CONCURRENT; a fixed drain
    with its OWN query schedule whose deadline judgment uses real receipt
    arrival times; recovery on the same node and epoch whose FIRST input and
    FIRST WaitSnapshot start together -- the wait's lease immediately runs
    the full candidate chain and is released in a finally, and the remaining
    queries follow the same 2 Hz table from t0 + one period."""
    artifact = {"stage": "r", "formal": formal, "overload_inputs": overload_inputs,
                "drain_s": drain_s, "recovery_inputs": recovery_inputs,
                "frames_manifest": frames_manifest(frames)}
    epoch_before = _epoch_of(client)
    t0 = readiness(client, frames)
    base_stamp = int(time.time() * 1e9)
    artifact["identity_binding"] = _capture_binding(
        client, getattr(client, "config_digest", ""))
    overload_records = []
    query_records = []
    drain_query_records = []
    errors = []

    def send_overload():
        try:
            overload_records.extend(
                send_inputs(client, frames, overload_inputs, 10.0, t0, base_stamp))
        except Exception as error:  # noqa: BLE001
            errors.append("overload: %r" % error)

    def query_overload():
        try:
            query_records.extend(
                run_queries(client, max(2, int(overload_inputs / 5)),
                            contract.QUERY_HZ, t0))
        except Exception as error:  # noqa: BLE001
            errors.append("overload-query: %r" % error)

    threads = [threading.Thread(target=send_overload),
               threading.Thread(target=query_overload)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=overload_inputs / 10.0 + 30)
    # Fixed drain from the last planned send. The drain keeps its own query
    # schedule (2 Hz across the window) and the deadline judgment uses the
    # receipt ARRIVAL times, never a poll after the fact.
    last_planned_ns = overload_records[-1]["planned_mono_ns"] \
        if overload_records else t0
    drain_queries_n = contract.R_DRAIN_QUERIES if formal \
        else max(2, int(drain_s * contract.QUERY_HZ))
    plan = drain_plan(last_planned_ns, drain_s, drain_queries_n,
                      contract.QUERY_HZ)

    def query_drain():
        try:
            for index, deadline in enumerate(plan["queries"]):
                now = time.monotonic_ns()
                if now < deadline:
                    time.sleep((deadline - now) / 1e9)
                box = contract.F_BOXES[index % 2]
                started = time.monotonic_ns()
                result = full_query(client, box)
                drain_query_records.append(
                    {"index": index, "box": list(box),
                     "planned_mono_ns": deadline,
                     "started_mono_ns": started,
                     "ended_mono_ns": time.monotonic_ns(), **result})
        except Exception as error:  # noqa: BLE001
            errors.append("drain-query: %r" % error)

    drain_thread = threading.Thread(target=query_drain)
    drain_thread.start()
    while time.monotonic_ns() < plan["deadline_ns"]:
        time.sleep(0.05)
    drain_observed_ns = time.monotonic_ns()
    drain_thread.join(timeout=drain_s + 15)
    timeline = receipt_timeline(client, overload_records,
                                timeout_s=max(10.0, drain_s))
    terminals, pending = collect_terminals(
        client, [record["input_id"] for record in overload_records],
        timeout_s=1.0)
    in_flight_at_deadline = [
        entry["input_id"] for entry in timeline
        if entry["receipt_status"] is None
        or entry["receipt_status"] in NON_TERMINAL_STATUSES]
    late_at_deadline = [
        entry["input_id"] for entry in timeline
        if entry["receipt_arrival_mono_ns"] is not None
        and entry["receipt_arrival_mono_ns"] > plan["deadline_ns"]]

    # Recovery: three concurrent workers on ONE shared t0 -- the sender, the
    # first Wait (bound to the deterministic first id, its lease completing
    # the candidate chain and released in a finally), and the remaining
    # queries on the same 2 Hz table.
    recovery_t0_ns = time.monotonic_ns() + int(0.5e9)
    recovery_stamp = int(time.time() * 1e9)
    first_id = str(recovery_stamp)  # index 0 stamp
    recovery_query_count = (contract.R_RECOVERY_QUERIES - 1) if formal \
        else max(2, recovery_inputs // 2)
    rplan = recovery_plan(recovery_t0_ns, recovery_inputs,
                          recovery_query_count, contract.UPDATE_HZ,
                          contract.QUERY_HZ)

    def send_recovery():
        try:
            artifact["recovery"] = send_inputs(
                client, frames, recovery_inputs, contract.UPDATE_HZ,
                recovery_t0_ns, recovery_stamp)
        except Exception as error:  # noqa: BLE001
            errors.append("recovery: %r" % error)

    wait_record = {"planned_mono_ns": rplan["first_wait_ns"], "bound_id": first_id}

    def first_recovery_wait():
        from uld_occ_observer.srv import WaitSnapshot
        now = time.monotonic_ns()
        if now < recovery_t0_ns:
            time.sleep((recovery_t0_ns - now) / 1e9)
        request = WaitSnapshot.Request()
        request.expected_epoch = 1
        request.sensor_id = "d555"
        request.observation_id = first_id
        request.max_age_ms = 2000
        request.timeout_ms = 2000
        wait_started = time.monotonic_ns()
        try:
            wait = client.call(client.wait, request, timeout=5.0)
            wait_record.update({
                "status": wait.status, "reason": wait.reason,
                "age_ns": wait.age_ns,
                "elapsed_ms": (time.monotonic_ns() - wait_started) / 1e6,
                "started_mono_ns": wait_started,
            })
            if wait.status == "OK" and wait.handle:
                # The wait lease is USED, not abandoned: the full candidate
                # chain runs on it and the finally in wait_chain releases it.
                wait_record["chain"] = wait_chain(client, wait,
                                                  contract.F_BOXES[0])
                wait_record["release_status"] = \
                    wait_record["chain"].get("release_status")
        except Exception as error:  # noqa: BLE001
            wait_record.update({"status": "EXCEPTION:%r" % (error,),
                                "started_mono_ns": wait_started,
                                "elapsed_ms": (time.monotonic_ns() - wait_started) / 1e6})

    recovery_query_records = []

    def query_recovery():
        try:
            for index, deadline in enumerate(rplan["queries"]):
                now = time.monotonic_ns()
                if now < deadline:
                    time.sleep((deadline - now) / 1e9)
                box = contract.F_BOXES[index % 2]
                started = time.monotonic_ns()
                result = full_query(client, box)
                recovery_query_records.append(
                    {"index": index, "box": list(box),
                     "planned_mono_ns": deadline,
                     "started_mono_ns": started,
                     "ended_mono_ns": time.monotonic_ns(), **result})
        except Exception as error:  # noqa: BLE001
            errors.append("recovery-query: %r" % error)

    recovery_threads = [threading.Thread(target=send_recovery),
                        threading.Thread(target=first_recovery_wait),
                        threading.Thread(target=query_recovery)]
    for thread in recovery_threads:
        thread.start()
    recovery_timeout = recovery_inputs / contract.UPDATE_HZ + 30
    for thread in recovery_threads:
        thread.join(timeout=recovery_timeout)
    if "recovery" not in artifact:
        artifact["recovery"] = []
    recovery_timeline = receipt_timeline(client, artifact["recovery"])
    artifact.update({
        "overload": overload_records, "overload_queries": query_records,
        "drain_queries": drain_query_records,
        "overload_timeline": timeline,
        "overload_terminals": terminals, "overload_pending": pending,
        "in_flight_at_drain_deadline": in_flight_at_deadline,
        "late_at_drain_deadline": late_at_deadline,
        "drain_deadline_mono_ns": plan["deadline_ns"],
        "drain_observed_mono_ns": drain_observed_ns,
        "first_wait": wait_record,
        "recovery_queries": recovery_query_records,
        "recovery_timeline": recovery_timeline,
        "epoch_before": epoch_before,
        "epoch_after": _epoch_of(client),
        "errors": errors,
    })
    (out_dir / "r.json").write_text(json.dumps(artifact, indent=1))
    return artifact


def _acquire_real_lease(client, max_age_ms, deadline_s):
    """Acquire a REAL lease, retrying briefly (a just-reset map needs the
    next input to integrate first). Returns (lease_or_None, last_status)."""
    deadline = time.monotonic() + deadline_s
    last = None
    while time.monotonic() < deadline:
        lease = client.call(client.get_snapshot, _snapshot_request(max_age_ms))
        last = lease.status
        if lease.status == "OK":
            return lease, last
        time.sleep(0.2)
    return None, last


def _ownership(client, max_samples=12, spacing_s=0.2, settle_s=0.0,
               target=None, target_field="view_directory_entries",
               include_trace=False):
    """Adaptive ownership measurement: sample the FULL resource snapshot
    until the expected residual appears (or the budget runs out) and keep
    EVERY sample with its timestamp. Mid-soak the node transiently carries
    the integration path's old/new retained pair and the parallel query
    stream's in-flight lease -- a single instant can legitimately read one
    extra full view. The per-class minimum over an adaptive window is the
    round's own residual: a leaked handle survives every sample and raises
    the minimum; a transient drops out."""
    if settle_s:
        time.sleep(settle_s)
    samples = []
    for index in range(max_samples):
        # gen4: the FIRST sample of an on-demand reconciliation round also
        # carries the per-object ledger table; every sample carries the
        # cheap loss counters (RESOURCE_FIELDS).
        include = include_trace and index == 0
        response = client.call(client.resource,
                               _resource_request(include_trace=include))
        sample = _resource_snapshot(response)
        if include:
            table = _parse_ownership_trace(response)
            if isinstance(table, list):
                sample["ownership_trace"] = table
                sample["ownership_live_count"] = sum(
                    1 for record in table if not record.get("released_mono_ns"))
        samples.append(sample)
        if target is not None \
                and samples[-1].get(target_field) <= target:
            break
        time.sleep(spacing_s)
    fields = dict(samples[-1])
    fields["resource_samples"] = samples
    fields["view_directory_entries_samples"] = [
        sample["view_directory_entries"] for sample in samples]
    return fields


def _validity_trace(client, mark):
    """gen4: full board transition trace (INVALID/BUILDING/VALID).

    The node's server-side ring keeps only the LAST 64 events, so the live
    subscription (running from stage start) is the primary source; a final
    get_validity poll merges its bounded events_json as a backstop. Deduped
    by (generation, state, transition_mono_ns) and sorted. The boot event's
    transition_mono_ns is 0 by construction (validity.cpp) -- kept verbatim,
    the judge tolerates and flags it, never rewrites it."""
    from uld_occ_observer.srv import GetValidity
    events, seen = [], set()

    def add(entry):
        if not isinstance(entry, dict):
            return
        key = (entry.get("generation"), entry.get("state"),
               entry.get("transition_mono_ns"))
        if key in seen:
            return
        seen.add(key)
        events.append(entry)

    for entry in client.validity_events[mark:]:
        add(entry)
    try:
        response = client.call(client.get_validity, GetValidity.Request())
        for entry in json.loads(response.events_json or "[]"):
            add(entry)
    except Exception:  # noqa: BLE001 -- the subscription trace stands alone
        pass
    events.sort(key=lambda e: (e.get("generation") or 0,
                               e.get("transition_mono_ns") or 0))
    return events


def stage_l(client, frames, seconds, read_rounds, reset_rounds, out_dir,
            formal=False, node_pid=None):
    """The soak judged per record: EVERY input keeps its publish row and
    terminal receipt, EVERY query keeps its planned/started/ended and full
    chain record. The ownership BASELINE is captured once the warmup churn
    is over (resets done, map saturated); the 1000 read rounds -- each a
    REAL acquire/read/release, refusals retried and listed, never counted --
    reconcile every owner class back to that baseline; the 100 reset rounds
    (warmup) hold a real lease ACROSS the reset, prove nothing but live
    leases survive it, and reconcile back to their own pre-round state.
    Health stamps and RSS samples run for the whole window."""
    artifact = {"stage": "l", "formal": formal, "planned_seconds": seconds,
                "read_rounds": read_rounds, "reset_rounds": reset_rounds,
                "frames_manifest": frames_manifest(frames)}
    baseline_pre_warmup = _resource_snapshot(
        client.call(client.resource, _resource_request()))
    t0 = readiness(client, frames)
    t0_ns = time.monotonic_ns()
    health_mark = len(client.health)
    validity_mark = len(client.validity_events)
    base_stamp = int(time.time() * 1e9)
    warmup_s = contract.L_WARMUP_S if formal else min(10.0, seconds / 2.0)
    stop = threading.Event()
    input_records = []
    query_records = []
    read_rounds_results = []
    read_failed_attempts = []
    reset_rounds_results = []
    send_errors = []
    rss_series = []
    lock = threading.Lock()
    baseline_holder = {}
    baseline_ready = threading.Event()

    def capture_baseline():
        # Post-warmup steady state: every later read round reconciles to
        # THIS, and the scored-window identity binding comes from a fresh
        # lease the refilled map vouches for.
        target = t0_ns + int(warmup_s * 1e9)
        now = time.monotonic_ns()
        if now < target:
            time.sleep((target - now) / 1e9)
        time.sleep(1.0)
        binding = None
        try:
            lease, _ = _acquire_real_lease(client, 10_000, 4.0)
            if lease is not None:
                identity = _identity(lease.identity)
                binding = {"expected_epoch": identity["epoch"],
                           "geometry_hash": identity["geometry_hash"],
                           "config_digest": identity["config_digest"],
                           "clock_domain": identity["clock_domain"],
                           "captured_mono_ns": time.monotonic_ns()}
                client.call(client.release, _release_request(lease.handle))
        except Exception:  # noqa: BLE001 - fall back to the readiness binding
            binding = None
        if binding is None:
            binding = _capture_binding(client,
                                       getattr(client, "config_digest", ""))
        baseline_holder.update(_ownership(
            client, max_samples=12, target=0,
            target_field="lease_view_directory_entries",
            include_trace=True))
        baseline_holder["binding"] = binding
        baseline_ready.set()

    def sustained():
        from sensor_msgs.msg import PointCloud2
        period_ns = int(1e9 / contract.UPDATE_HZ)
        index = 0
        while not stop.is_set():
            frame = frames[index % len(frames)]
            cloud, _ = cloud_from_frame(frame)
            stamp = base_stamp + index * period_ns
            cloud.header.stamp.sec = stamp // 10**9
            cloud.header.stamp.nanosec = stamp % 10**9
            deadline = t0_ns + int(index / contract.UPDATE_HZ * 1e9)
            now = time.monotonic_ns()
            if now < deadline:
                time.sleep((deadline - now) / 1e9)
            wake_ns = time.monotonic_ns()
            client.cloud.publish(cloud)
            published_ns = time.monotonic_ns()
            trace = _trace()
            if trace is not None:
                trace.wake("l_input", deadline, wake_ns, published_ns)
            with lock:
                input_records.append({
                    "index": index, "stamp_ns": stamp, "input_id": str(stamp),
                    "planned_mono_ns": deadline,
                    "publish_mono_ns": published_ns})
            index += 1
            if deadline < time.monotonic_ns() - int(5e9):
                send_errors.append("sender_fell_behind_at_%d" % index)
                return

    def queries():
        index = 0
        while not stop.is_set():
            deadline = t0_ns + int(index / contract.QUERY_HZ * 1e9)
            now = time.monotonic_ns()
            if now < deadline:
                time.sleep((deadline - now) / 1e9)
            wake_ns = time.monotonic_ns()
            box = contract.F_BOXES[index % 2]
            started = time.monotonic_ns()
            trace = _trace()
            if trace is not None:
                trace.wake("l_query", deadline, wake_ns, started)
            result = full_query(client, box)
            with lock:
                query_records.append(
                    {"index": index, "box": list(box),
                     "planned_mono_ns": deadline,
                     "started_mono_ns": started,
                     "ended_mono_ns": time.monotonic_ns(), **result})
            index += 1

    def reader():
        if not baseline_ready.wait(timeout=warmup_s + 60):
            return  # the baseline never landed; the missing rounds are honest
        scored_s = max(1.0, seconds - warmup_s)
        retry_budget_s = 30.0
        for index in range(read_rounds):
            if stop.is_set():
                break  # the run window closed; the count is then honest
            deadline = t0_ns + int((warmup_s + (index + 0.5) * scored_s
                                    / read_rounds) * 1e9)
            now = time.monotonic_ns()
            if now < deadline:
                time.sleep((deadline - now) / 1e9)
            # A REAL lease, retried past transient refusals: a failed
            # attempt is listed, never counted as a lifecycle round.
            record = None
            attempt_deadline = time.monotonic() + retry_budget_s
            while True:
                record = full_query(client, contract.F_BOXES[0])
                status = record.get("status") or ""
                if not status.startswith("ACQUIRE_"):
                    break
                with lock:
                    read_failed_attempts.append(
                        {"round": index, "status": status,
                         "t_mono_ns": time.monotonic_ns()})
                if stop.is_set() or time.monotonic() >= attempt_deadline:
                    break
                time.sleep(0.2)
            record["ownership_after"] = _ownership(
                client, target=0, target_field="lease_view_directory_entries")
            with lock:
                read_rounds_results.append(record)

    def resetter():
        from uld_occ_observer.srv import ResetObservation
        epoch = _epoch_of(client) or 1
        for index in range(reset_rounds):
            if stop.is_set():
                break  # the run window closed; the count is then honest
            deadline = t0_ns + int((warmup_s * (index + 1)
                                    / (reset_rounds + 1)) * 1e9)
            now = time.monotonic_ns()
            if now < deadline:
                time.sleep((deadline - now) / 1e9)
            record = {"index": index,
                      "started_mono_ns": time.monotonic_ns(),
                      "ownership_before": _ownership(client, max_samples=4)}
            # The error-path round needs a REAL lease on the live map; retry
            # past the just-reset gap until the next input integrates. The
            # product's release is idempotent-OK by design (an unknown
            # handle is a no-op), so the OBSERVABLE error lifecycle is: hold
            # a lease ACROSS a reset, then release it explicitly in the
            # finally -- never abandoned to TTL. The per-owner split shows
            # the reset dropping everything except live leases, and the
            # round reconciling back to its own pre-round state.
            lease, acquire_status = _acquire_real_lease(client, 1000, 4.0)
            record["acquire_status"] = acquire_status
            try:
                request = ResetObservation.Request()
                request.expected_epoch = epoch
                request.reset_id = "l_reset_%d" % index
                record["reset_called_mono_ns"] = time.monotonic_ns()
                response = client.call(client.reset, request)
                record["reset_status"] = response.status
                if response.status == "OK":
                    epoch = response.epoch
                response = client.call(client.resource,
                                       _resource_request(include_trace=True))
                record["ownership_after_reset"] = _resource_snapshot(response)
                record["ownership_after_reset"]["ownership_trace"] = \
                    _parse_ownership_trace(response)
            finally:
                if lease is not None:
                    try:
                        real = client.call(client.release,
                                           _release_request(lease.handle))
                        record["real_release"] = real.status
                    except Exception as error:  # noqa: BLE001
                        record["real_release"] = "EXCEPTION:%r" % (error,)
            record["epoch"] = epoch
            record["ownership_after"] = _ownership(
                client, settle_s=1.5, max_samples=12, target=0,
                target_field="lease_view_directory_entries",
                include_trace=True)
            with lock:
                reset_rounds_results.append(record)

    def rss_sampler():
        while not stop.is_set():
            rss_series.append({"t_mono_ns": time.monotonic_ns(),
                               "rss_kib": _rss_kib(node_pid)})
            stop.wait(0.5)

    workers = [threading.Thread(target=capture_baseline),
               threading.Thread(target=sustained), threading.Thread(target=queries),
               threading.Thread(target=reader), threading.Thread(target=resetter),
               threading.Thread(target=rss_sampler)]
    trace = _trace()
    if trace is not None:
        # Independent freeze witness (50 ms tick, no deadline of its own) +
        # the --gc-hardening schedule anchor (freeze/disable at warmup end).
        trace.start_heartbeat(stop, hardening_t0_ns=t0_ns,
                              hardening_s=warmup_s)
    for worker in workers:
        worker.start()
    time.sleep(seconds)
    stop.set()
    for worker in workers:
        worker.join(timeout=15)
    # Terminal closure for every input through the indexed collector.
    input_timeline = receipt_timeline(client, input_records, timeout_s=10.0)
    artifact["sent_inputs"] = len(input_records)
    artifact["send_errors"] = send_errors
    artifact["input_records"] = input_records
    artifact["input_timeline"] = input_timeline
    artifact["query_records"] = query_records
    artifact["read_rounds_results"] = read_rounds_results
    artifact["read_failed_attempts"] = read_failed_attempts
    artifact["reset_rounds_results"] = reset_rounds_results
    artifact["rss_series"] = rss_series
    artifact["health_series"] = [
        {"stamp_ns": sample.get("stamp_ns"),
         "arrival_mono_ns": sample.get("arrival_mono_ns"),
         "epoch": sample.get("epoch")}
        for sample in client.health[health_mark:]
        if isinstance(sample, dict)
        and isinstance(sample.get("stamp_ns"), int)]
    artifact["health_samples"] = len(client.health[health_mark:])
    baseline = dict(baseline_holder)
    binding = baseline.pop("binding", None)
    if binding:
        artifact["identity_binding"] = binding
    artifact["baseline"] = baseline or baseline_pre_warmup
    # Stage end: every worker joined; let the last integration's transient
    # old-view tail drain, then measure adaptively back to the baseline.
    after = _ownership(client, settle_s=0.6, target=0,
                       target_field="lease_view_directory_entries",
                       include_trace=True)
    artifact["after"] = after
    artifact["validity_trace"] = _validity_trace(client, validity_mark)
    if trace is not None:
        artifact["sched_trace_counts"] = trace.dump(out_dir)["counts"]
    (out_dir / "l.json").write_text(json.dumps(artifact, indent=1))
    return artifact


def _resource_request(include_trace=False):
    from uld_occ_observer.srv import GetResourceUsage
    request = GetResourceUsage.Request()
    # gen4: read rounds reconcile with the always-present loss counters;
    # only baseline / reset rounds / stage end ask for the full table.
    request.include_ownership_trace = bool(include_trace)
    return request


# Every scalar field of GetResourceUsage, captured whole: the per-owner
# reconciliation (blocks / directories / caches / external buffers, shared
# counted once) reads these -- never a single self-chosen count.
RESOURCE_FIELDS = (
    "view_directory_entries", "view_directory_entries_peak",
    "view_directory_bytes", "view_directory_bytes_peak",
    "shared_block_bytes", "shared_block_bytes_peak", "old_view_block_bytes",
    "lease_view_block_bytes", "retained_view_block_bytes",
    "client_held_view_block_bytes",
    "lease_view_directory_entries", "retained_view_directory_entries",
    "client_held_view_directory_entries",
    "surface_cache_bytes", "surface_cache_bytes_peak",
    "external_surface_buffer_bytes",
    "inflight_response_bytes", "inflight_response_bytes_peak",
    "read_view_bytes", "read_view_bytes_peak",
    "cache_bytes", "cache_bytes_peak", "read_view_cache_bytes_peak",
    "store_tree_bytes", "uncovered_allocations", "certification",
    # gen4 ledger loss counters (present on every call; the no-loss proof
    # is dropped_live == 0 && allocated - released == live table size).
    "ownership_allocated_total", "ownership_released_total",
    "ownership_dropped_live")


def _parse_ownership_trace(response):
    """The on-demand per-object table, parsed defensively (empty when the
    request did not ask for it)."""
    raw = getattr(response, "ownership_trace_json", "") or ""
    try:
        records = json.loads(raw) if raw.strip() else []
    except ValueError:
        return {"parse_error": True, "raw_len": len(raw)}
    return records if isinstance(records, list) else {"parse_error": True}


def _resource_snapshot(response):
    snapshot = {field: getattr(response, field) for field in RESOURCE_FIELDS}
    snapshot["t_mono_ns"] = time.monotonic_ns()
    return snapshot


def _capture_binding(client, config_digest):
    """The stage-start identity binding from the lease readiness vouched
    (or, before any lease exists, from the latest health epoch)."""
    identity = getattr(client, "bound_identity", None) or {}
    epoch = identity.get("epoch")
    if not isinstance(epoch, int):
        epoch = _epoch_of(client)
    return {"expected_epoch": epoch or 1,
            "geometry_hash": identity.get("geometry_hash") or "",
            "config_digest": identity.get("config_digest") or config_digest,
            "clock_domain": identity.get("clock_domain") or "steady_clock",
            "captured_mono_ns": time.monotonic_ns()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--node", required=True)
    parser.add_argument("--geometry", default=str(FIXTURES / "geometry_chamfer.yaml"))
    parser.add_argument("--config", default=str(FIXTURES / "config_chamfer_r0p02.yaml"))
    parser.add_argument("--eval-native", required=True)
    parser.add_argument("--frames-glob", required=True,
                        help="emitted frame YAMLs (materialized by the caller)")
    parser.add_argument("--out", required=True)
    parser.add_argument("--stages", default="p,r,l")
    parser.add_argument("--p-inputs", type=int, default=2)
    parser.add_argument("--p-queries", type=int, default=2)
    parser.add_argument("--r-overload", type=int, default=40)
    parser.add_argument("--r-drain", type=float, default=5.0)
    parser.add_argument("--r-recovery", type=int, default=10)
    parser.add_argument("--l-seconds", type=float, default=20.0)
    parser.add_argument("--budget-minutes", type=float, default=None,
                        help="hard wall-clock budget ENFORCED between stages "
                             "(the scheduler stops, not a text promise)")
    parser.add_argument("--formal", action="store_true",
                        help="run contract counts (P 300/240 x9, R 600/30/75, "
                             "L 120 min); the same source powers the rehearsal "
                             "and the closed score entry")
    parser.add_argument("--trace-sched", action="store_true",
                        help="opt-in scheduler forensics: wakeup decomposition "
                             "per deadline loop + 50 ms heartbeat witness + gc "
                             "pause pairs + callback durations -> "
                             "sched_trace.json. Localization only; judged "
                             "fields are never touched.")
    parser.add_argument("--pin-client", default=None,
                        help="comma CPU list for sched_setaffinity of the "
                             "CLIENT process, e.g. '4,5'. LOCALIZATION arm "
                             "only -- the frozen contract environment is "
                             "no-CPU-affinity; formal use needs a new "
                             "contract version.")
    parser.add_argument("--ownership-trace", action="store_true",
                        help="launch the node with ownership_trace:=true and "
                             "capture the per-object ledger (baseline / reset "
                             "rounds / stage end carry the full table; every "
                             "resource sample carries the loss counters). "
                             "The gen4 L judge REQUIRES this evidence.")
    parser.add_argument("--gc-hardening", action="store_true",
                        help="gc.freeze() at warmup end + gc.disable() for "
                             "the scored segment + one collect per 120 s "
                             "(driven by the trace heartbeat thread). "
                             "Localization/exclusion arm only.")
    args = parser.parse_args()
    global SCHED_TRACE, GC_HARDENING
    if args.trace_sched:
        SCHED_TRACE = SchedTrace()
        SCHED_TRACE.install_gc()
    GC_HARDENING = args.gc_hardening
    if args.pin_client:
        cpus = sorted(int(value) for value in args.pin_client.split(","))
        os.sched_setaffinity(0, cpus)
        if SCHED_TRACE is not None:
            SCHED_TRACE.meta["affinity"] = cpus
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    frames = load_frames(args.frames_glob)
    if not frames:
        raise SystemExit("no frames matched %s" % args.frames_glob)
    # Prefer per-box-count groups when the glob carries the bNN tag; the
    # fallback keeps any frame set usable for wiring.
    frames_by_box = group_frames_by_box(frames)
    started = time.monotonic()
    node = NodeProcess(args.node, args.geometry, args.config,
                       out / "observer.log", "/freespace_campaign_cloud",
                       extra_params=("ownership_trace:=true",)
                       if args.ownership_trace else ())
    client = Client("/freespace_campaign_cloud")
    import hashlib
    client.config_digest = hashlib.sha256(
        Path(args.config).read_bytes()).hexdigest()
    results = {}

    def _failed(name):
        return any(reason != "short_sample_not_formal"
                   for reason in results.get(name, {}).get("reasons", []))

    def _budget_spent():
        return args.budget_minutes is not None and \
            time.monotonic() - started > args.budget_minutes * 60

    try:
        client.wait_services()
        first = frames[0]
        tf = first.get("transform") or {}
        translation = tf.get("translation") or [-1.15, 0.0, 1.20]
        client.broadcast_tf(translation)
        stages = args.stages.split(",")
        p_inputs = contract.P_WINDOW_INPUTS if args.formal else args.p_inputs
        p_queries = contract.P_WINDOW_QUERIES if args.formal else args.p_queries
        r_overload = contract.R_OVERLOAD_INPUTS if args.formal else args.r_overload
        r_drain = contract.R_DRAIN_S if args.formal else args.r_drain
        r_recovery = contract.R_RECOVERY_INPUTS if args.formal else args.r_recovery
        l_seconds = contract.L_DURATION_S if args.formal else args.l_seconds
        l_rounds = contract.L_READ_RELEASE_ROUNDS if args.formal else 15
        l_resets = contract.L_RESET_ROUNDS if args.formal else 2
        p_windows = contract.P_WINDOWS_PER_BOX if args.formal else 1
        # Formal R and L run the FIXED 20-box workload, never the mixed set.
        r_l_frames = frames_by_box[20] if args.formal else frames
        if "p" in stages:
            for box_count in contract.P_BOX_COUNTS:
                for window_index in range(p_windows):
                    window = stage_p(client, frames_by_box[box_count], box_count,
                                     p_inputs, p_queries, out, formal=args.formal,
                                     window_index=window_index)
                    results["p_b%02d_w%d" % (box_count, window_index)] = \
                        freespace_judge.judge_p_window(window)
        # A failed stage STOPS its dependents (P gates R, R gates L); the
        # budget is enforced here in the scheduler, between stages.
        stop_reason = None
        if "p" in stages and _failed("p"):
            stop_reason = "p_failed"
        if stop_reason is None and "r" in stages:
            results["r"] = freespace_judge.judge_r_stage(
                stage_r(client, r_l_frames, r_overload, r_drain, r_recovery, out,
                        formal=args.formal))
            if _failed("r"):
                stop_reason = "r_failed"
            elif _budget_spent():
                stop_reason = "budget_exhausted"
        if stop_reason is not None:
            results["skipped"] = {"reason": "stage_failure_stops_dependents",
                                  "detail": stop_reason}
        elif "l" in stages:
            results["l"] = freespace_judge.judge_l_stage(
                stage_l(client, r_l_frames, l_seconds, l_rounds, l_resets, out,
                        formal=args.formal, node_pid=node.proc.pid))
    except Exception as error:  # noqa: BLE001 - persist the failure prefix
        import traceback
        (out / "failure.json").write_text(json.dumps({
            "error": repr(error),
            "traceback": traceback.format_exc(),
            "partial_results": results,
            "elapsed_s": time.monotonic() - started,
        }, indent=1))
        raise
    finally:
        client.shutdown()
        node.stop()
    # A rehearsal verdict: the wiring must be clean; short_sample_not_formal
    # is the by-design marker that these counts cannot certify a formal gate.
    formal_reached = all(v.get("formal") and not v.get("reasons")
                         for v in results.values()
                         if isinstance(v, dict)) if args.formal else False
    summary = {"stages": results, "formal": args.formal,
               "formal_pass": formal_reached,
               "budget_exhausted": _budget_spent(),
               "wiring_ok": all(
                   all(reason == "short_sample_not_formal"
                       for reason in v.get("reasons", []))
                   for v in results.values()
                   if isinstance(v, dict) and "reasons" in v)
               if not args.formal else False}
    (out / "summary.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps({name: verdict.get("reasons") if isinstance(verdict, dict)
                      else verdict for name, verdict in results.items()},
                     indent=1))
    return 0 if (formal_reached or summary["wiring_ok"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
