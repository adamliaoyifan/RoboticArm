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
        self.executor = MultiThreadedExecutor()
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
        self.receipts.append(message)

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


def full_query(client, box, yaw_mode=2, max_age_ms=1000):
    """The measured full chain: acquire -> every page -> release."""
    started = time.monotonic_ns()
    lease = client.call(client.get_snapshot, _snapshot_request(max_age_ms))
    if lease.status != "OK":
        return {"status": lease.status, "latency_ms": (time.monotonic_ns() - started) / 1e6}
    cursor, done, pages, candidates = 0, False, 0, 0
    identity = None
    while not done:
        request = _freespace_request(lease.handle, box, yaw_mode, cursor)
        page = client.call(client.freespace, request)
        if page.status not in ("OK", "NO_KNOWN_FREE"):
            client.call(client.release, _release_request(lease.handle))
            return {"status": page.status,
                    "latency_ms": (time.monotonic_ns() - started) / 1e6}
        identity = _identity(page.identity)
        candidates += len(page.cell_x)
        cursor = page.next_cursor
        done = page.done
        pages += 1
    client.call(client.release, _release_request(lease.handle))
    return {"status": page.status if pages else "NO_PAGE", "identity": identity,
            "pages": pages, "candidates": candidates,
            "feasible_total": page.feasible_total,
            "latency_ms": (time.monotonic_ns() - started) / 1e6}


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
            if receipt.status == "INTEGRATED":
                lease = client.call(client.get_snapshot, _snapshot_request(10_000))
                if lease.status == "OK":
                    client.call(client.release, _release_request(lease.handle))
                    return time.monotonic_ns()
        time.sleep(0.02)
    raise TimeoutError("readiness: no integrated input")


def send_inputs(client, frames, count, hz, t0_ns, base_stamp):
    """Absolute-deadline sender; returns per-input publish records."""
    from sensor_msgs.msg import PointCloud2
    records = []
    for index in range(count):
        frame = frames[index % len(frames)]
        cloud, _ = cloud_from_frame(frame)
        stamp = base_stamp + index * (10**9 // 20)  # unique, increasing, >=50ms apart
        cloud.header.stamp.sec = stamp // 10**9
        cloud.header.stamp.nanosec = stamp % 10**9
        deadline = t0_ns + int(index / hz * 1e9)
        now = time.monotonic_ns()
        if now < deadline:
            time.sleep((deadline - now) / 1e9)
        planned = deadline
        client.cloud.publish(cloud)
        records.append({"index": index, "stamp_ns": stamp,
                        "planned_mono_ns": planned,
                        "publish_mono_ns": time.monotonic_ns(),
                        "input_id": str(stamp)})
    return records


def run_queries(client, count, hz, t0_ns):
    """Absolute-deadline full-query thread; boxes alternate per request."""
    records = []
    for index in range(count):
        box = contract.F_BOXES[index % 2]
        deadline = t0_ns + int(index / hz * 1e9)
        now = time.monotonic_ns()
        if now < deadline:
            time.sleep((deadline - now) / 1e9)
        planned = deadline
        started = time.monotonic_ns()
        result = full_query(client, box)
        records.append({"index": index, "box": list(box), "planned_mono_ns": planned,
                        "started_mono_ns": started, "ended_mono_ns": time.monotonic_ns(),
                        **result})
    return records


def collect_terminals(client, ids, timeout_s=10.0):
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
            if record.status not in ("NOT_SEEN", "PROCESSING", "WAITING"):
                terminals[observation_id] = record.status
                pending.discard(observation_id)
        if pending:
            time.sleep(0.05)
    return terminals, sorted(pending)


def stage_p(client, frames, box_count, inputs, queries, out_dir):
    window = {"stage": "p", "box_count": box_count, "inputs": inputs,
              "queries": queries, "update_hz": contract.UPDATE_HZ,
              "query_hz": contract.QUERY_HZ, "formal": False}
    t0 = readiness(client, frames)
    base_stamp = int(time.time() * 1e9)  # stamps lag wall clock
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
    time.sleep(0.5)
    ids = [record["input_id"] for record in sender_records]
    terminals, pending = collect_terminals(client, ids)
    window.update({
        "sender": sender_records, "queries": query_records, "errors": errors,
        "terminals": terminals, "pending": pending,
        "health_samples": len(client.health),
    })
    (out_dir / ("p_b%02d.json" % box_count)).write_text(json.dumps(window, indent=1))
    return window


def stage_r(client, frames, overload_inputs, drain_s, recovery_inputs, out_dir):
    artifact = {"stage": "r", "formal": False, "overload_inputs": overload_inputs,
                "drain_s": drain_s, "recovery_inputs": recovery_inputs}
    t0 = readiness(client, frames)
    base_stamp = int(time.time() * 1e9)  # stamps lag wall clock
    # Overload: 10 Hz inputs while 2 Hz queries run; explicit pressure is
    # allowed, every id must still reach a terminal state.
    overload = send_inputs(client, frames, overload_inputs, 10.0, t0, base_stamp)
    queries = run_queries(client, max(2, overload_inputs // 5), contract.QUERY_HZ, t0)
    drain_deadline = time.monotonic() + drain_s
    while time.monotonic() < drain_deadline:
        time.sleep(0.1)
    ids = [record["input_id"] for record in overload]
    terminals, pending = collect_terminals(client, ids, timeout_s=max(10.0, drain_s))
    # Recovery on the SAME node and epoch.
    recovery_t0 = time.monotonic_ns()
    recovery_stamp = int(time.time() * 1e9)
    recovery = send_inputs(client, frames, recovery_inputs, contract.UPDATE_HZ,
                           recovery_t0, recovery_stamp)
    first_id = recovery[0]["input_id"]
    from uld_occ_observer.srv import WaitSnapshot
    request = WaitSnapshot.Request()
    request.expected_epoch = 1
    request.sensor_id = "d555"
    request.observation_id = first_id
    request.max_age_ms = 2000
    request.timeout_ms = 2000
    wait_started = time.monotonic_ns()
    wait = client.call(client.wait, request, timeout=5.0)
    wait_ms = (time.monotonic_ns() - wait_started) / 1e6
    recovery_queries = run_queries(client, max(2, recovery_inputs // 2),
                                   contract.QUERY_HZ, recovery_t0)
    recovery_ids = [record["input_id"] for record in recovery]
    recovery_terminals, recovery_pending = collect_terminals(client, recovery_ids)
    artifact.update({
        "overload": overload, "overload_queries": queries,
        "overload_terminals": terminals, "overload_pending": pending,
        "recovery": recovery, "first_wait": {
            "status": wait.status, "reason": wait.reason, "age_ns": wait.age_ns,
            "elapsed_ms": wait_ms, "bound_id": first_id,
        },
        "recovery_queries": recovery_queries,
        "recovery_terminals": recovery_terminals,
        "recovery_pending": recovery_pending,
        "epoch_after": _epoch_of(client),
    })
    (out_dir / "r.json").write_text(json.dumps(artifact, indent=1))
    return artifact


def _epoch_of(client):
    if not client.health:
        return None
    return client.health[-1].get("epoch")


def stage_l(client, frames, seconds, read_rounds, reset_rounds, out_dir):
    artifact = {"stage": "l", "formal": False, "seconds": seconds,
                "read_rounds": read_rounds, "reset_rounds": reset_rounds}
    baseline = client.call(client.resource, _resource_request())
    t0 = readiness(client, frames)
    base_stamp = int(time.time() * 1e9)  # stamps lag wall clock
    stop = threading.Event()

    def sustained():
        index = 0
        while not stop.is_set():
            frame = frames[index % len(frames)]
            from sensor_msgs.msg import PointCloud2
            cloud, _ = cloud_from_frame(frame)
            stamp = base_stamp + index * (10**9 // 20)
            cloud.header.stamp.sec = stamp // 10**9
            cloud.header.stamp.nanosec = stamp % 10**9
            client.cloud.publish(cloud)
            stop.wait(1.0 / contract.UPDATE_HZ)
            index += 1

    query_errors = []

    def queries():
        index = 0
        while not stop.is_set():
            try:
                full_query(client, contract.F_BOXES[index % 2])
            except Exception as error:  # noqa: BLE001
                query_errors.append("%r" % error)
            stop.wait(1.0 / contract.QUERY_HZ)
            index += 1

    workers = [threading.Thread(target=sustained), threading.Thread(target=queries)]
    for worker in workers:
        worker.start()
    time.sleep(seconds)
    stop.set()
    for worker in workers:
        worker.join(timeout=10)
    rounds = []
    for _ in range(read_rounds):
        result = full_query(client, contract.F_BOXES[0])
        rounds.append(result)
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
        # A released lease through an error path: acquire then let the scope
        # end; the next full query must still answer on the new epoch.
        lease = client.call(client.get_snapshot, _snapshot_request(60_000))
        if lease.status == "OK":
            try:
                client.call(client.release, _release_request(lease.handle), timeout=1.0)
            except TimeoutError:
                pass  # the error path: the lease expires by TTL instead
    after = client.call(client.resource, _resource_request())
    artifact.update({
        "query_errors": query_errors,
        "read_rounds_results": rounds,
        "reset_statuses": resets,
        "baseline": _resource_fields(baseline),
        "after": _resource_fields(after),
    })
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
        if "p" in stages:
            for box_count in contract.P_BOX_COUNTS:
                results["p_b%02d" % box_count] = freespace_judge.judge_p_window(
                    stage_p(client, frames_by_box[box_count], box_count,
                            args.p_inputs, args.p_queries, out))
        if "r" in stages:
            results["r"] = freespace_judge.judge_r_stage(
                stage_r(client, frames, args.r_overload, args.r_drain,
                        args.r_recovery, out))
        if "l" in stages:
            results["l"] = freespace_judge.judge_l_stage(
                stage_l(client, frames, args.l_seconds, 15, 2, out))
    finally:
        client.shutdown()
        node.stop()
    # A rehearsal verdict: the wiring must be clean; short_sample_not_formal
    # is the by-design marker that these counts cannot certify a formal gate.
    summary = {"stages": results,
               "wiring_ok": all(
                   all(reason == "short_sample_not_formal"
                       for reason in v.get("reasons", []))
                   for v in results.values())}
    (out / "summary.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps({name: verdict.get("reasons") for name, verdict in results.items()},
                     indent=1))
    return 0 if summary["wiring_ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
