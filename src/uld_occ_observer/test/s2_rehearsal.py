#!/usr/bin/env python3
"""S2 non-scoring rehearsal. Hooks are used only for the wait-capacity barrier.

The functional pass and the trace capture start a second node with
behavior_test_hooks false. Client callbacks use 4 threads. A sender thread
publishes clouds and does not process responses.
"""
import argparse
import json
import os
import signal
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "benchmark"))
import capture_replay  # noqa: E402

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from uld_occ_observer.msg import UpdateReceipt
from uld_occ_observer.srv import (
    GetObservationStatus,
    GetResourceUsage,
    GetSnapshot,
    IntegrateObservation,
    SetBehaviorHook,
    WaitSnapshot,
)

FRAME = "d555_color_optical_frame"
SENSOR = "cam0"
CLOUD = "/s2_rehearsal_cloud"
DOMAIN = "43"


class Observer:
    def __init__(self, binary, ros_args, log_path, env):
        self.log = Path(log_path).open("w")
        self.proc = subprocess.Popen(
            [binary, "--ros-args", *ros_args],
            stdout=self.log, stderr=subprocess.STDOUT, env=env, start_new_session=True)

    def stop(self):
        if self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGTERM)
                self.proc.wait(timeout=5)
        self.log.close()


def cloud_message(stamp_ns):
    cloud = PointCloud2()
    cloud.header.frame_id = FRAME
    cloud.header.stamp.sec = int(stamp_ns // 1_000_000_000)
    cloud.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
    cloud.height = 1
    cloud.width = 1
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = 12
    for name in ("x", "y", "z"):
        field = PointField()
        field.name = name
        field.offset = len(cloud.fields) * 4
        field.datatype = PointField.FLOAT32
        field.count = 1
        cloud.fields.append(field)
    cloud.data = struct.pack("<fff", 0.0, 0.0, 1.0)
    return cloud


def transform(stamp_ns):
    message = TransformStamped()
    message.header.frame_id = "container_link"
    message.header.stamp.sec = int(stamp_ns // 1_000_000_000)
    message.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
    message.child_frame_id = FRAME
    message.transform.translation.x = -1.15
    message.transform.translation.z = 1.20
    message.transform.rotation.x = 0.5
    message.transform.rotation.y = -0.5
    message.transform.rotation.z = 0.5
    message.transform.rotation.w = -0.5
    return message


class Session:
    def __init__(self):
        self.node = Node("s2_rehearsal_client")
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.health = []
        # (time.monotonic_ns(), payload): health receive events with their
        # CLOCK_MONOTONIC boundary, for window-relative health gap evidence.
        self.health_events = []
        self.receipts = []
        # (time.monotonic_ns(), message): raw receipt events for counting and
        # duplicate detection. The receipts list stays for compatibility.
        self.receipt_events = []
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(String, "/uld_occ_observer/health", self._health, reliable)
        # Frozen plan §2B: the receipt ledger subscription depth is 1024.
        self.node.create_subscription(UpdateReceipt, "/uld_occ_observer/receipts", self._receipt, 1024)
        self.wait = self.node.create_client(WaitSnapshot, "/uld_occ_observer/wait_snapshot")
        self.snapshot = self.node.create_client(GetSnapshot, "/uld_occ_observer/get_snapshot")
        self.status = self.node.create_client(GetObservationStatus, "/uld_occ_observer/get_observation_status")
        self.integrate = self.node.create_client(IntegrateObservation, "/uld_occ_observer/integrate_observation")
        self.resource = self.node.create_client(GetResourceUsage, "/uld_occ_observer/get_resource_usage")
        self.hook = self.node.create_client(SetBehaviorHook, "/uld_occ_observer/set_behavior_hook")
        self.cloud = self.node.create_publisher(PointCloud2, CLOUD, reliable)
        self.broadcaster = StaticTransformBroadcaster(self.node)
        self.checks = []
        self._last_stamp = 0

    def _health(self, message):
        self.health_events.append((time.monotonic_ns(), message.data))
        self.health.append(message.data)

    def _receipt(self, message):
        self.receipts.append(message)
        self.receipt_events.append((time.monotonic_ns(), message))

    def close(self):
        self.executor.shutdown()
        self.thread.join(timeout=3)
        self.node.destroy_node()

    def call(self, client, request, timeout):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError(client.srv_name)
            time.sleep(0.01)
        return future.result()

    def wait_service(self, client, timeout=20.0):
        deadline = time.monotonic() + timeout
        while not client.wait_for_service(timeout_sec=0.2):
            if time.monotonic() > deadline:
                raise TimeoutError(client.srv_name)

    def check(self, name, passed, detail=""):
        self.checks.append({"name": name, "pass": bool(passed), "detail": str(detail)})
        print(("PASS " if passed else "FAIL ") + name + ("" if passed else " " + str(detail)), flush=True)

    def hook_set(self, name, value):
        self.wait_service(self.hook, 5)
        request = SetBehaviorHook.Request()
        request.name = name
        request.value = int(value)
        result = self.call(self.hook, request, 5)
        if not result.accepted:
            raise RuntimeError(result.status)
        return result.status

    def next_stamp(self):
        stamp = max(self.node.get_clock().now().nanoseconds, self._last_stamp + 1_000_000)
        self._last_stamp = stamp
        return stamp

    def query(self, epoch, observation_id):
        request = GetObservationStatus.Request()
        request.epoch = int(epoch)
        request.sensor_id = SENSOR
        request.observation_id = observation_id
        return self.call(self.status, request, 5)

    def wait_for(self, epoch, observation_id, timeout_ms):
        request = WaitSnapshot.Request()
        request.expected_epoch = int(epoch)
        request.sensor_id = SENSOR
        request.observation_id = observation_id
        request.timeout_ms = int(timeout_ms)
        request.max_age_ms = 2000
        return self.call(self.wait, request, timeout_ms / 1000.0 + 2.0)


def start_observer(binary, geometry, config, log_dir, extra, env):
    args = ["-p", f"geometry_file:={geometry}", "-p", f"config_file:={config}", *extra]
    log = Path(log_dir) / f"observer-{len(list(Path(log_dir).glob('observer-*.log')))}.log"
    return Observer(binary, args, log, env)


def wait_capacity(session, binary, geometry, config, out, env):
    observer = start_observer(binary, geometry, config, out, [
        "-p", "input_mode:=service", "-p", "sensor_id:=cam0", "-p", "behavior_test_hooks:=true",
    ], env)
    try:
        session.wait_service(session.hook)
        session.wait_service(session.wait)
        session.hook_set("wait_phase", 1)
        futures = []
        for index in range(8):
            futures.append(session.wait.call_async(_wait(1, f"held-{index}", 1500)))
        queued = ""
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            queued = session.hook_set("wait_accounting", 0)
            if queued == "checking=0 queued=8":
                break
            time.sleep(0.02)
        session.hook_set("wait_phase", 2)
        checking = ""
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            checking = session.hook_set("wait_accounting", 0)
            if checking == "checking=8 queued=0":
                break
            time.sleep(0.02)
        ninth = session.wait_for(1, "ninth", 2000)
        after = session.hook_set("wait_accounting", 0)
        session.check("eight waits are queued before the check starts",
                      queued == "checking=0 queued=8", queued)
        session.check("the check holds eight and the queue is empty",
                      checking == "checking=8 queued=0", checking)
        session.check("ninth wait is pressure while those eight are in checking",
                      ninth.status == "WAIT_PRESSURE" and after == "checking=8 queued=0",
                      f"{ninth.status} {after}")
        session.hook_set("wait_phase", 0)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not all(item.done() for item in futures):
            time.sleep(0.02)
    finally:
        try:
            session.hook_set("wait_phase", 0)
        except Exception:
            pass
        observer.stop()


def _wait(epoch, observation_id, timeout_ms):
    request = WaitSnapshot.Request()
    request.expected_epoch = epoch
    request.sensor_id = SENSOR
    request.observation_id = observation_id
    request.timeout_ms = timeout_ms
    request.max_age_ms = 2000
    return request


def functional(session, binary, geometry, config, out, env, geometry_hash):
    failed = start_observer(binary, geometry, config, out, [
        "-p", "input_mode:=subscription", "-p", "sensor_id:=cam0",
        "-p", "optical_frame:=" + FRAME, "-p", f"cloud_topic:={CLOUD}",
        "-p", "behavior_test_hooks:=false",
    ], env)
    try:
        session.wait_service(session.status)
        deadline = time.monotonic() + 5.0
        while session.cloud.get_subscription_count() < 1 and time.monotonic() < deadline:
            time.sleep(0.05)
        health = _health_until(session, 3.0)
        session.check("startup health is ready",
                      health is not None and int(health["epoch"]) == 1, str(health))
        stamp = session.next_stamp()
        message = cloud_message(stamp)
        cloud_path = Path(out) / "tf_absent_cloud.json"
        cloud_path.write_text(json.dumps({
            "stamp_ns": stamp, "frame_id": message.header.frame_id, "data_sha256": __import__("hashlib").sha256(bytes(message.data)).hexdigest(),
        }) + "\n")
        session.cloud.publish(message)
        observation_id = str(stamp)
        status = _poll(session, 1, observation_id, lambda item: item.status == "REJECTED")
        first = None if status is None else {"status": status.status, "reason": status.receipt.status}
    finally:
        failed.stop()
    if first is None or first["status"] != "REJECTED":
        session.check("tf failure is captured and replays", False, str(first))
    else:
        replayed = start_observer(binary, geometry, config, out, [
            "-p", "input_mode:=subscription", "-p", "sensor_id:=cam0",
            "-p", "optical_frame:=" + FRAME, "-p", f"cloud_topic:={CLOUD}",
            "-p", "behavior_test_hooks:=false",
        ], env)
        try:
            deadline = time.monotonic() + 5.0
            while session.cloud.get_subscription_count() < 1 and time.monotonic() < deadline:
                time.sleep(0.05)
            session.cloud.publish(message)
            second_status = _poll(session, 1, observation_id, lambda item: item.status == "REJECTED")
            second = None if second_status is None else {
                "status": second_status.status, "reason": second_status.receipt.status}
        finally:
            replayed.stop()
        incomplete = Path(out) / "incomplete_failure.json"
        incomplete.write_text(json.dumps({"observation_id": observation_id, "capture_complete": False}) + "\n")
        refused = capture_replay.replay_failure(incomplete, Path(out) / "unused-replay")
        session.check("tf failure is re-executed",
                      second is not None and second == first and first["reason"] not in ("", None),
                      "first=%s second=%s" % (first, second))
        session.check("incomplete failure bundle cannot replay",
                      refused["replay_possible"] is False and refused["executed"] is False, str(refused))

    static = transform(session.node.get_clock().now().nanoseconds)
    static.header.stamp.sec = 0
    static.header.stamp.nanosec = 0
    session.broadcaster.sendTransform(static)
    time.sleep(0.3)
    trace_path = Path(out) / "functional.trace.jsonl"
    traced = os.environ.copy()
    traced.update(env)
    traced["ULD_OCC_INTERFACE_TRACE"] = "1"
    traced["ULD_OCC_TRACE_PATH"] = str(trace_path)
    traced["ULD_OCC_TRACE_RUN_ID"] = "s2-functional"
    node = start_observer(binary, geometry, config, out, [
        "-p", "input_mode:=service", "-p", "sensor_id:=cam0", "-p", "behavior_test_hooks:=false",
    ], traced)
    try:
        session.wait_service(session.integrate)
        started = time.monotonic()
        wait_future = session.wait.call_async(_wait(1, "recover-1", 2000))
        time.sleep(0.05)
        response = _integrate(session, geometry_hash, "recover-1")
        deadline = time.monotonic() + 2.0
        while not wait_future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        elapsed = time.monotonic() - started
        waited = wait_future.result() if wait_future.done() else None
        stored = session.query(1, "recover-1")
        session.check("recovery wait returns within 2s",
                      response.receipt.status == "INTEGRATED" and waited is not None and waited.status == "OK"
                      and elapsed <= 2.0 and waited.age_ns <= 2_000_000_000
                      and stored.status == "COMPLETED" and stored.receipt.status == response.receipt.status
                      and stored.receipt.identity.observed_revision == response.receipt.identity.observed_revision,
                      f"elapsed={elapsed:.3f} wait={None if waited is None else waited.status}")
        ids = []

        def send_three():
            for _ in range(3):
                ids.append(_integrate(session, geometry_hash, f"send-{len(ids)}").receipt.observation_id)
        sender = threading.Thread(target=send_three)
        sender.start()
        sender.join(timeout=15)
        unique = {item.observation_id for item in session.receipts if item.observation_id.startswith("send-")}
        session.check("independent sender keeps one terminal per id",
                      sender.is_alive() is False and unique == {"send-0", "send-1", "send-2"}, str(sorted(unique)))
        usage = session.call(session.resource, GetResourceUsage.Request(), 5)
        out_of_scope = {"middleware_transport_buffers", "tf_buffer", "fingerprint_table"}
        session.check("resource certification is scoped read-view/cache attribution",
                      usage.certification == "READ_VIEW_CACHE_ATTRIBUTED"
                      and not usage.uncovered_allocations
                      and set(usage.uncovered_classes) <= out_of_scope
                      and usage.shared_block_bytes >= 0 and usage.surface_cache_bytes >= 0,
                      "%s uncovered=%s" % (usage.certification, sorted(usage.uncovered_classes)))
    finally:
        node.stop()
    # The functional flow performs exactly these service integrates, in this
    # order, under the s2-functional run id: the expected operation set is
    # known here and supplied to the certifier — a trace is never certified
    # against nothing.
    expected_operations = []
    for name in ("recover-1", "send-0", "send-1", "send-2"):
        expected_operations.append({"event": "integrate_start", "request_id": name, "status": ""})
        expected_operations.append({"event": "integrate_end", "request_id": name, "status": "INTEGRATED"})
    traced = capture_replay.certify_trace(
        trace_path, {"operations": expected_operations, "run_id": "s2-functional"})
    session.check("live trace is complete and is not by itself a replay",
                  traced["capture_complete"] is True and traced["replay_possible"] is False, str(traced))
    stripped = Path(out) / "trace_without_footer.jsonl"
    body = [line for line in trace_path.read_text().splitlines() if '"trace_footer"' not in line]
    stripped.write_text("\n".join(body) + "\n")
    session.check("trace without a footer is incomplete",
                  capture_replay.certify_trace(stripped)["capture_complete"] is False, "")


def _health_until(session, timeout):
    mark = len(session.health)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if len(session.health) > mark:
            return yaml.safe_load(session.health[-1])
        time.sleep(0.05)
    return None


def _poll(session, epoch, observation_id, ready):
    deadline = time.monotonic() + 2.0
    last = None
    while time.monotonic() < deadline:
        last = session.query(epoch, observation_id)
        if ready(last):
            return last
        time.sleep(0.05)
    return last


def _integrate(session, geometry_hash, observation_id):
    stamp = session.next_stamp()
    request = IntegrateObservation.Request()
    request.sensor_id = SENSOR
    request.observation_id = observation_id
    request.clock_domain = "ros"
    request.geometry_hash = geometry_hash
    request.epoch = 1
    request.cloud = cloud_message(stamp)
    request.uld_from_optical = transform(stamp)
    return session.call(session.integrate, request, 8)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--node", required=True)
    parser.add_argument("--geometry", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    geometry = yaml.safe_load(Path(args.geometry).read_text())
    geometry_hash = json.loads(geometry["descriptor_json"])["geometry_hash"]
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = DOMAIN
    rclpy.init()
    session = Session()
    error = ""
    try:
        wait_capacity(session, args.node, args.geometry, args.config, out, env)
        functional(session, args.node, args.geometry, args.config, out, env, geometry_hash)
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        print("FAIL harness " + error, flush=True)
    finally:
        session.close()
        if rclpy.ok():
            rclpy.shutdown()
    failed = [item for item in session.checks if not item["pass"]]
    payload = {"checks": session.checks, "passed": len(session.checks) - len(failed),
               "failed": len(failed), "harness_error": error,
               "client_executor_threads": 4, "behavior_hooks_on_functional": False}
    (out / "rehearsal.json").write_text(json.dumps(payload, indent=2) + "\n")
    return 1 if failed or error else 0


if __name__ == "__main__":
    raise SystemExit(main())
