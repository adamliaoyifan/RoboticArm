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
        frames.append(frame)
    return frames


class NodeProcess:
    def __init__(self, binary, geometry, config, log_path, cloud_topic):
        self.log = open(log_path, "w")
        env = os.environ.copy()
        install_lib = Path(binary).resolve().parent.parent.parent / "lib"
        env["LD_LIBRARY_PATH"] = "%s:%s" % (install_lib, env.get("LD_LIBRARY_PATH", ""))
        self.proc = subprocess.Popen(
            [binary, "--ros-args",
             "-p", "geometry_file:=%s" % geometry,
             "-p", "config_file:=%s" % config,
             "-p", "input_mode:=subscription",
             "-p", "cloud_topic:=%s" % cloud_topic,
             "-p", "optical_frame:=d555_color_optical_frame",
             "-p", "sensor_id:=d555"],
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
        self.wait = self._client(WaitSnapshot, "wait_snapshot")
        self.get_snapshot = self._client(GetSnapshot, "get_snapshot")
        self.freespace = self._client(QueryFreespace, "query_freespace")
        self.release = self._client(ReleaseSnapshot, "release_snapshot")
        self.status = self._client(GetObservationStatus, "get_observation_status")
        self.reset = self._client(ResetObservation, "reset_observation")
        self.resource = self._client(GetResourceUsage, "get_resource_usage")
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
        # receipt OBSERVABLE, and the R timeline needs these stamps.
        self.receipts.append({
            "arrival_mono_ns": time.monotonic_ns(),
            "status": message.status,
            "observation_id": message.observation_id,
            "input_stamp_ns": message.input_stamp_ns,
            "callback_enter_ns": message.callback_enter_ns,
            "epoch": message.identity.epoch,
            "map_revision": message.identity.map_revision,
            "observed_revision": message.identity.observed_revision,
        })

    def _health(self, message):
        self.health.append(yaml.safe_load(message.data))

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


def _canonical_candidate_bytes(yaw, x, y, z, kind):
    """Same canonical stream the O9 profile CLI hashes, so a Python-run chain
    and a C++-run chain agree on content identity."""
    return ("%d,%d,%d,%d,%d;" % (yaw, x, y, z, kind)).encode()


def full_query(client, box, yaw_mode=2, max_age_ms=1000, request_timeout=10.0):
    """The measured whole chain: acquire -> EVERY page materialized -> release.

    Fail-closed rules (review group 2): the lease is released in a finally
    block even on timeouts; every response array must have the same length
    and the verify ranges 3n; the page identity must equal the lease identity
    on every page; the cursor must advance exactly to min(cursor+limit,
    feasible_total) with a consistent done flag; the materialized candidate
    count must equal feasible_total; the FULL candidate list is kept and a
    canonical content digest is recorded. Any fault names itself in the
    status; a healthy NO_KNOWN_FREE stays a legal semantic answer.
    """
    import hashlib
    started = time.monotonic_ns()
    record = {"status": "OK", "lease_identity": None, "pages": 0,
              "candidates": [], "feasible_total": None,
              "content_sha256": None, "release_status": None,
              "page_status": None, "latency_ms": None}
    handle = None
    try:
        lease = client.call(client.get_snapshot, _snapshot_request(max_age_ms),
                            request_timeout)
        if lease.status != "OK":
            record["status"] = "ACQUIRE_%s" % lease.status
            return record
        handle = lease.handle
        record["lease_identity"] = _identity(lease.identity)
        record["age_ns"] = lease.age_ns
        cursor, done, pages = 0, False, 0
        digest = hashlib.sha256()
        feasible = None
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
                digest.update(_canonical_candidate_bytes(*cell))
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
    except TimeoutError as error:
        record["status"] = "QUERY_TIMEOUT:%s" % error
        return record
    finally:
        if handle is not None:
            try:
                response = client.call(client.release, _release_request(handle),
                                       request_timeout)
                record["release_status"] = response.status
                if response.status != "OK":
                    record["release_fault"] = response.status
            except Exception as error:  # noqa: BLE001 - recorded, never raised past finally
                record["release_status"] = "RELEASE_EXCEPTION:%r" % (error,)
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
            "last_integrated_stamp_ns": message.last_integrated_stamp_ns}


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
        client.cloud.publish(cloud)
        records.append({"index": index, "stamp_ns": stamp,
                        "planned_mono_ns": deadline,
                        "publish_mono_ns": time.monotonic_ns(),
                        "input_id": str(stamp)})
    return records


def receipt_timeline(client, records, timeout_s=10.0):
    """Per-input terminal timeline from the receipts the node published.

    The input E2E clock is publish -> terminal receipt observable; a missing
    or non-terminal receipt stays missing (never guessed).
    """
    deadline = time.monotonic() + timeout_s
    by_id = {}
    while time.monotonic() < deadline:
        by_id = {}
        for record in records:
            for receipt in client.receipts:
                if receipt["observation_id"] == record["input_id"]:
                    current = by_id.get(record["input_id"])
                    if current is None or receipt["arrival_mono_ns"] > current["arrival_mono_ns"]:
                        by_id[record["input_id"]] = receipt
        pending = [record["input_id"] for record in records
                   if record["input_id"] not in by_id
                   or by_id[record["input_id"]]["status"] in NON_TERMINAL_STATUSES]
        if not pending:
            break
        time.sleep(0.05)
    timeline = []
    for record in records:
        receipt = by_id.get(record["input_id"])
        timeline.append({
            "input_id": record["input_id"],
            "planned_mono_ns": record["planned_mono_ns"],
            "publish_mono_ns": record["publish_mono_ns"],
            "receipt_status": receipt["status"] if receipt else None,
            "callback_enter_ns": receipt["callback_enter_ns"] if receipt else None,
            "receipt_arrival_mono_ns": receipt["arrival_mono_ns"] if receipt else None,
            "e2e_ms": ((receipt["arrival_mono_ns"] - record["publish_mono_ns"]) / 1e6)
            if receipt and receipt["status"] not in NON_TERMINAL_STATUSES else None,
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
        started = time.monotonic_ns()
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


def stage_p(client, frames, box_count, inputs, queries, out_dir, formal=False):
    """One window: sender and full-query threads on independent absolute
    clocks from the same t0; per-input terminal timeline; per-query latency
    against its own planned deadline (the H gate reads the same records)."""
    window = {"stage": "p", "box_count": box_count, "inputs": inputs,
              "queries": queries, "update_hz": contract.UPDATE_HZ,
              "query_hz": contract.QUERY_HZ, "formal": formal,
              "window_planned_s": inputs / contract.UPDATE_HZ}
    t0 = readiness(client, frames)
    base_stamp = int(time.time() * 1e9)  # stamps lag wall clock by design
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
    for sample in client.health:
        stamp = sample.get("stamp_ns") if isinstance(sample, dict) else None
        if isinstance(stamp, int):
            if last is not None:
                health_gaps.append((stamp - last) / 1e9)
            last = stamp
    window.update({
        "sender": sender_records, "queries": query_records, "errors": errors,
        "input_timeline": timeline, "terminals": terminals, "pending": pending,
        "health_samples": len(client.health),
        "health_gap_max_s": max(health_gaps) if health_gaps else None,
        "window_close_ms": (window_closed_ns - t0) / 1e6,
    })
    (out_dir / ("p_b%02d.json" % box_count)).write_text(json.dumps(window, indent=1))
    return window


def stage_r(client, frames, overload_inputs, drain_s, recovery_inputs, out_dir,
            formal=False):
    """Overload with 10 Hz inputs and 2 Hz queries CONCURRENT; a fixed drain
    whose deadline judgment uses real receipt times; recovery on the same
    node and epoch whose FIRST input and FIRST WaitSnapshot start together.
    The full publish/callback/receipt/wait timeline is preserved so a
    TIMEOUT is diagnosed from evidence instead of blamed on throughput."""
    artifact = {"stage": "r", "formal": formal, "overload_inputs": overload_inputs,
                "drain_s": drain_s, "recovery_inputs": recovery_inputs}
    epoch_before = _epoch_of(client)
    t0 = readiness(client, frames)
    base_stamp = int(time.time() * 1e9)
    overload_records = []
    query_records = []
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
    # Fixed drain from the last planned send; the deadline check uses the
    # receipts that actually arrived, not a poll after the fact.
    last_planned_ns = overload_records[-1]["planned_mono_ns"] \
        if overload_records else t0
    drain_deadline_ns = last_planned_ns + int(drain_s * 1e9)
    while time.monotonic_ns() < drain_deadline_ns:
        time.sleep(0.05)
    drain_observed_ns = time.monotonic_ns()
    timeline = receipt_timeline(client, overload_records,
                                timeout_s=max(10.0, drain_s))
    terminals, pending = collect_terminals(
        client, [record["input_id"] for record in overload_records],
        timeout_s=0.0 or 1.0)
    in_flight_at_deadline = [
        entry["input_id"] for entry in timeline
        if entry["receipt_status"] is None
        or entry["receipt_status"] in NON_TERMINAL_STATUSES]

    # Recovery: the first input publish and the first WaitSnapshot begin at
    # the same instant (two threads, one shared t0). The wait is bound to the
    # FIRST recovery id, which is deterministic before publishing.
    recovery_t0_ns = time.monotonic_ns() + int(0.5e9)
    recovery_stamp = int(time.time() * 1e9)
    first_id = str(recovery_stamp)  # index 0 stamp

    def send_recovery():
        try:
            artifact["recovery"] = send_inputs(
                client, frames, recovery_inputs, contract.UPDATE_HZ,
                recovery_t0_ns, recovery_stamp)
        except Exception as error:  # noqa: BLE001
            errors.append("recovery: %r" % error)

    wait_record = {}

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
                "bound_id": first_id,
            })
        except Exception as error:  # noqa: BLE001
            wait_record.update({"status": "EXCEPTION:%r" % (error,),
                                "started_mono_ns": wait_started,
                                "bound_id": first_id,
                                "elapsed_ms": (time.monotonic_ns() - wait_started) / 1e6})

    recovery_threads = [threading.Thread(target=send_recovery),
                        threading.Thread(target=first_recovery_wait)]
    for thread in recovery_threads:
        thread.start()
    for thread in recovery_threads:
        thread.join(timeout=15)
    if "recovery" not in artifact:
        artifact["recovery"] = []
    recovery_queries = run_queries(
        client, max(2, recovery_inputs // 2), contract.QUERY_HZ,
        recovery_t0_ns + int(0.2e9))
    recovery_timeline = receipt_timeline(client, artifact["recovery"])
    artifact.update({
        "overload": overload_records, "overload_queries": query_records,
        "overload_timeline": timeline,
        "overload_terminals": terminals, "overload_pending": pending,
        "in_flight_at_drain_deadline": in_flight_at_deadline,
        "drain_deadline_mono_ns": drain_deadline_ns,
        "drain_observed_mono_ns": drain_observed_ns,
        "first_wait": wait_record,
        "recovery_queries": recovery_queries,
        "recovery_timeline": recovery_timeline,
        "epoch_before": epoch_before,
        "epoch_after": _epoch_of(client),
        "errors": errors,
    })
    (out_dir / "r.json").write_text(json.dumps(artifact, indent=1))
    return artifact


def stage_l(client, frames, seconds, read_rounds, reset_rounds, out_dir,
            formal=False, node_pid=None):
    """Sustained loop on ABSOLUTE clocks (query latency never stretches the
    send period), per-frame terminal timeline, a 1 Hz raw RSS series off the
    node process, full query records, and reset/error rounds whose leases
    are released through their error paths."""
    artifact = {"stage": "l", "formal": formal, "planned_seconds": seconds,
                "read_rounds": read_rounds, "reset_rounds": reset_rounds}
    baseline = client.call(client.resource, _resource_request())
    t0 = readiness(client, frames)
    base_stamp = int(time.time() * 1e9)
    stop = threading.Event()
    send_index = [0]
    query_records = []
    send_errors = []
    rss_series = []

    def sustained():
        from sensor_msgs.msg import PointCloud2
        period_ns = int(1e9 / contract.UPDATE_HZ)
        next_deadline_ns = time.monotonic_ns()
        while not stop.is_set():
            frame = frames[send_index[0] % len(frames)]
            cloud, _ = cloud_from_frame(frame)
            stamp = base_stamp + send_index[0] * period_ns
            cloud.header.stamp.sec = stamp // 10**9
            cloud.header.stamp.nanosec = stamp % 10**9
            now = time.monotonic_ns()
            if now < next_deadline_ns:
                time.sleep((next_deadline_ns - now) / 1e9)
            client.cloud.publish(cloud)
            send_index[0] += 1
            next_deadline_ns += period_ns
            if next_deadline_ns < time.monotonic_ns() - int(5e9):
                send_errors.append("sender_fell_behind_at_%d" % send_index[0])
                next_deadline_ns = time.monotonic_ns()

    def queries():
        index = 0
        period_ns = int(1e9 / contract.QUERY_HZ)
        next_deadline_ns = time.monotonic_ns()
        while not stop.is_set():
            now = time.monotonic_ns()
            if now < next_deadline_ns:
                time.sleep((next_deadline_ns - now) / 1e9)
            query_records.append(full_query(client, contract.F_BOXES[index % 2]))
            index += 1
            next_deadline_ns += period_ns

    def rss_sampler():
        while not stop.is_set():
            rss_series.append({"t_mono_ns": time.monotonic_ns(),
                               "rss_kib": _rss_kib(node_pid)})
            stop.wait(1.0)

    workers = [threading.Thread(target=sustained), threading.Thread(target=queries),
               threading.Thread(target=rss_sampler)]
    for worker in workers:
        worker.start()
    time.sleep(seconds)
    stop.set()
    for worker in workers:
        worker.join(timeout=15)
    artifact["sent_inputs"] = send_index[0]
    artifact["send_errors"] = send_errors
    artifact["query_records"] = query_records
    artifact["rss_series"] = rss_series
    artifact["health_samples"] = len(client.health)
    rounds = []
    for _ in range(read_rounds):
        rounds.append(full_query(client, contract.F_BOXES[0]))
    artifact["read_rounds_results"] = rounds
    resets = []
    epoch = _epoch_of(client) or 1
    for index in range(reset_rounds):
        from uld_occ_observer.srv import ResetObservation
        request = ResetObservation.Request()
        request.expected_epoch = epoch
        request.reset_id = "l_reset_%d" % index
        response = client.call(client.reset, request)
        resets.append(response.status)
        if response.status == "OK":
            epoch = response.epoch
        # Error path: acquire a lease and let the release FAIL on purpose by
        # releasing through a bogus handle; the real lease must still expire
        # by TTL, proven by the next round answering on the new epoch.
        lease = client.call(client.get_snapshot, _snapshot_request(60_000))
        if lease.status == "OK":
            try:
                client.call(client.release, _release_request("bogus"), timeout=1.0)
            except TimeoutError:
                pass
    artifact["reset_statuses"] = resets
    after = client.call(client.resource, _resource_request())
    artifact["baseline"] = _resource_fields(baseline)
    artifact["after"] = _resource_fields(after)
    (out_dir / "l.json").write_text(json.dumps(artifact, indent=1))
    return artifact


def _resource_request():
    from uld_occ_observer.srv import GetResourceUsage
    return GetResourceUsage.Request()


def _resource_fields(response):
    return {"view_directory_entries": response.view_directory_entries,
            "read_view_cache_bytes_peak": response.read_view_cache_bytes_peak,
            "certification": response.certification}


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
    parser.add_argument("--formal", action="store_true",
                        help="run contract counts (P 300/240 x9, R 600/30/75, "
                             "L 120 min); the same source powers the rehearsal "
                             "and the closed score entry")
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    frames = load_frames(args.frames_glob)
    if not frames:
        raise SystemExit("no frames matched %s" % args.frames_glob)
    # Prefer per-box-count groups when the glob carries the bNN tag; the
    # fallback keeps any frame set usable for wiring.
    frames_by_box = {}
    for box_count in contract.P_BOX_COUNTS:
        tagged = [frame for frame in frames
                  if isinstance(frame, dict) and frame.get("_box_tag") == box_count]
        frames_by_box[box_count] = tagged or frames
    node = NodeProcess(args.node, args.geometry, args.config,
                       out / "observer.log", "/freespace_campaign_cloud")
    client = Client("/freespace_campaign_cloud")
    results = {}
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
        if "p" in stages:
            for box_count in contract.P_BOX_COUNTS:
                for window_index in range(p_windows):
                    window = stage_p(client, frames_by_box[box_count], box_count,
                                     p_inputs, p_queries, out, formal=args.formal)
                    results["p_b%02d_w%d" % (box_count, window_index)] = \
                        freespace_judge.judge_p_window(window)
        if "r" in stages:
            results["r"] = freespace_judge.judge_r_stage(
                stage_r(client, frames, r_overload, r_drain, r_recovery, out,
                        formal=args.formal))
        if "l" in stages:
            results["l"] = freespace_judge.judge_l_stage(
                stage_l(client, frames, l_seconds, l_rounds, l_resets, out,
                        formal=args.formal, node_pid=node.proc.pid))
    finally:
        client.shutdown()
        node.stop()
    # A rehearsal verdict: the wiring must be clean; short_sample_not_formal
    # is the by-design marker that these counts cannot certify a formal gate.
    formal_reached = all(v.get("formal") and not v.get("reasons")
                         for v in results.values()) if args.formal else False
    summary = {"stages": results, "formal": args.formal,
               "formal_pass": formal_reached,
               "wiring_ok": all(
                   all(reason == "short_sample_not_formal"
                       for reason in v.get("reasons", []))
                   for v in results.values()) if not args.formal else False}
    (out / "summary.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps({name: verdict.get("reasons") for name, verdict in results.items()},
                     indent=1))
    return 0 if (formal_reached or summary["wiring_ok"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
