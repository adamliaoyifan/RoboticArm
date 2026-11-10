#!/usr/bin/env python3
"""Live S1 gen3 checks for status, wait, and resource services.

No Gazebo, no scoring, no planning. The observer is started and stopped here.
"""
import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import threading

from uld_occ_observer.msg import UpdateReceipt
from uld_occ_observer.srv import (
    GetObservationStatus,
    GetResourceUsage,
    GetSnapshot,
    IntegrateObservation,
    ReadSurface,
    ReleaseSnapshot,
    ResetObservation,
    SetBehaviorHook,
    WaitSnapshot,
)

FRAME = "d555_color_optical_frame"
SENSOR = "cam0"
CLOUD_TOPIC = "/s1_g3_cloud"


class Observer:
    def __init__(self, binary, ros_args, log_path):
        self.log_path = Path(log_path)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log = self.log_path.open("w")
        env = os.environ.copy()
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

    def alive(self):
        return self.proc.poll() is None


class Client:
    def __init__(self):
        self.node = Node("s1_g3_client")
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.health = []
        self.receipts = []
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(String, "/uld_occ_observer/health", self._health, reliable)
        self.node.create_subscription(UpdateReceipt, "/uld_occ_observer/receipts", self._receipt, 100)
        self.integrate_srv = self._client(IntegrateObservation, "integrate_observation")
        self.status = self._client(GetObservationStatus, "get_observation_status")
        self.wait = self._client(WaitSnapshot, "wait_snapshot")
        self.resource = self._client(GetResourceUsage, "get_resource_usage")
        self.snapshot = self._client(GetSnapshot, "get_snapshot")
        self.surface = self._client(ReadSurface, "read_surface")
        self.release = self._client(ReleaseSnapshot, "release_snapshot")
        self.reset = self._client(ResetObservation, "reset_observation")
        self.hook = self._client(SetBehaviorHook, "set_behavior_hook")
        self.cloud = self.node.create_publisher(PointCloud2, CLOUD_TOPIC, reliable)
        self.broadcaster = StaticTransformBroadcaster(self.node)
        self._last_stamp = 0
        self.checks = []

    def _client(self, srv, name):
        return self.node.create_client(srv, f"/uld_occ_observer/{name}")

    def _health(self, message):
        self.health.append(message.data)

    def _receipt(self, message):
        self.receipts.append(message)

    def close(self):
        self.executor.shutdown()
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
        return bool(passed)

    def latest_health(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        mark = len(self.health)
        while time.monotonic() < deadline:
            if len(self.health) > mark:
                return yaml.safe_load(self.health[-1])
            time.sleep(0.05)
        if self.health:
            return yaml.safe_load(self.health[-1])
        raise TimeoutError("health")

    def set_param(self, name, value):
        self.wait_service(self.hook, 5)
        request = SetBehaviorHook.Request()
        request.name = name
        request.value = int(value)
        result = self.call(self.hook, request, 5)
        if not result.accepted:
            raise RuntimeError(result.status)
        return True

    def next_stamp(self):
        stamp = max(self.node.get_clock().now().nanoseconds, self._last_stamp + 1_000_000)
        self._last_stamp = stamp
        return stamp

    def cloud_message(self, stamp_ns):
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
        import struct
        cloud.data = struct.pack("<fff", 0.0, 0.0, 1.0)
        return cloud

    def transform(self, stamp_ns):
        transform = TransformStamped()
        transform.header.frame_id = "container_link"
        transform.header.stamp.sec = int(stamp_ns // 1_000_000_000)
        transform.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
        transform.child_frame_id = FRAME
        transform.transform.translation.x = -1.15
        transform.transform.translation.z = 1.20
        transform.transform.rotation.x = 0.5
        transform.transform.rotation.y = -0.5
        transform.transform.rotation.z = 0.5
        transform.transform.rotation.w = -0.5
        return transform

    def broadcast_static_tf(self):
        transform = self.transform(self.node.get_clock().now().nanoseconds)
        transform.header.stamp.sec = 0
        transform.header.stamp.nanosec = 0
        self.broadcaster.sendTransform(transform)

    def integrate(self, geometry_hash, observation_id, epoch, stamp_ns=None):
        if stamp_ns is None:
            stamp_ns = self.next_stamp()
        request = IntegrateObservation.Request()
        request.sensor_id = SENSOR
        request.observation_id = observation_id
        request.clock_domain = "ros"
        request.geometry_hash = geometry_hash
        request.epoch = int(epoch)
        request.cloud = self.cloud_message(stamp_ns)
        request.uld_from_optical = self.transform(stamp_ns)
        return self.call(self.integrate_srv, request, 8), stamp_ns

    def query(self, epoch, observation_id):
        request = GetObservationStatus.Request()
        request.epoch = int(epoch)
        request.sensor_id = SENSOR
        request.observation_id = observation_id
        return self.call(self.status, request, 5)

    def wait_for(self, epoch, observation_id, timeout_ms, max_age_ms=2000):
        request = WaitSnapshot.Request()
        request.expected_epoch = int(epoch)
        request.sensor_id = SENSOR
        request.observation_id = observation_id
        request.max_age_ms = int(max_age_ms)
        request.timeout_ms = int(timeout_ms)
        started = time.monotonic()
        response = self.call(self.wait, request, timeout_ms / 1000.0 + 2.0)
        return response, time.monotonic() - started

    def usage(self):
        return self.call(self.resource, GetResourceUsage.Request(), 5)

    def publish_cloud(self, stamp_ns=None):
        if stamp_ns is None:
            stamp_ns = self.next_stamp()
        self.cloud.publish(self.cloud_message(stamp_ns))
        return stamp_ns, str(stamp_ns)

    def wait_match(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while self.cloud.get_subscription_count() < 1:
            if time.monotonic() > deadline:
                raise TimeoutError("cloud subscription")
            time.sleep(0.05)


def same_receipt(left, right):
    return (left.status == right.status and left.sensor_id == right.sensor_id and
            left.observation_id == right.observation_id and left.input_stamp_ns == right.input_stamp_ns and
            left.identity.observed_revision == right.identity.observed_revision and
            left.identity.epoch == right.identity.epoch)


def run_service(client, observer_factory):
    observer = observer_factory(["-p", "input_mode:=service", "-p", "sensor_id:=cam0",
                                 "-p", "behavior_test_hooks:=true"])
    try:
        client.wait_service(client.integrate_srv)
        client.wait_service(client.status)
        client.wait_service(client.wait)
        client.wait_service(client.resource)
        health = client.latest_health()
        geometry = health["geometry_hash"]
        epoch = int(health["epoch"])
        missing = client.query(epoch, "never-seen")
        client.check("status not seen", missing.status == "NOT_SEEN" and missing.receipt.status == "",
                     missing.status + " " + missing.receipt.status)
        done, stamp = client.integrate(geometry, "frame-a", epoch)
        stored = client.query(epoch, "frame-a")
        published = next((item for item in reversed(client.receipts) if item.observation_id == "frame-a"), None)
        client.check("service integrate", done.receipt.status == "INTEGRATED", done.receipt.status)
        client.check("status matches service response",
                     stored.status == "COMPLETED" and same_receipt(stored.receipt, done.receipt),
                     f"{stored.status} {stored.receipt.status}")
        client.check("published receipt matches query",
                     published is not None and same_receipt(published, stored.receipt),
                     "missing" if published is None else published.status)
        before = client.latest_health()["map_revision"]
        duplicate, _ = client.integrate(geometry, "frame-a", epoch)
        again = client.query(epoch, "frame-a")
        after = client.latest_health()
        client.check("duplicate is not integrated again",
                     duplicate.receipt.status == "DUPLICATE" and again.status == "COMPLETED" and
                     again.receipt.identity.observed_revision == stored.receipt.identity.observed_revision and
                     after["map_revision"] == before,
                     duplicate.receipt.status)
        wait_future = client.wait.call_async(_wait_request(epoch, "frame-b", 2000, 2000))
        time.sleep(0.05)
        started = time.monotonic()
        second, _ = client.integrate(geometry, "frame-b", epoch)
        deadline = time.monotonic() + 2.0
        while not wait_future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        elapsed = time.monotonic() - started
        if wait_future.done():
            waited = wait_future.result()
            client.check("wait returns the completed observation within 2s",
                         waited.status == "OK" and elapsed <= 2.0 and waited.age_ns <= 2_000_000_000 and
                         waited.identity.observed_revision == second.receipt.identity.observed_revision,
                         f"{waited.status} elapsed={elapsed:.3f} age={waited.age_ns}")
        else:
            client.check("wait returns the completed observation within 2s", False, "no response")
        mismatch, _ = client.integrate(geometry, "wrong-epoch", 99)
        mismatch_status = client.query(99, "wrong-epoch")
        client.check("service epoch mismatch is rejected",
                     mismatch.receipt.status == "EPOCH_MISMATCH" and mismatch_status.status == "REJECTED" and
                     mismatch_status.receipt.status == "EPOCH_MISMATCH" and
                     mismatch_status.receipt.observation_id == "wrong-epoch",
                     f"{mismatch.receipt.status} query={mismatch_status.status}/{mismatch_status.receipt.status}")
        snap = GetSnapshot.Request()
        snap.max_age_ms = 5000
        snap.allow_stale = True
        first = client.call(client.snapshot, snap, 5)
        page = ReadSurface.Request()
        page.handle = first.handle
        page.cursor = 0
        page.limit = 64
        client.call(client.surface, page, 5)
        one = client.usage()
        second_handle = client.call(client.snapshot, snap, 5)
        two = client.usage()
        client.check("second handle does not double the surface cache",
                     first.status == "OK" and one.surface_cache_bytes > 0 and
                     two.surface_cache_bytes == one.surface_cache_bytes,
                     f"{one.surface_cache_bytes} -> {two.surface_cache_bytes}")
        client.check("resource certification is scoped read-view/cache attribution",
                     two.certification == "READ_VIEW_CACHE_ATTRIBUTED" and
                     not two.uncovered_allocations and
                     set(two.uncovered_classes) <= {"middleware_transport_buffers", "tf_buffer",
                                                    "fingerprint_table"} and
                     two.inflight_response_bytes == 0 and
                     two.inflight_response_bytes_peak > 0,
                     f"{two.certification} peak={two.inflight_response_bytes_peak} classes={list(two.uncovered_classes)}")
        client.set_param("receipt_retention_ms", 200)
        client.integrate(geometry, "expire-me", epoch)
        time.sleep(0.45)
        expired = client.query(epoch, "expire-me")
        client.check("query past retention returns expired", expired.status == "EXPIRED", expired.status)
        client.set_param("receipt_retention_ms", 120000)
        pressure_ok = True
        detail = ""
        futures = []
        for index in range(8):
            futures.append(client.wait.call_async(_wait_request(epoch, f"pending-{index}", 700, 2000)))
        time.sleep(0.05)
        ninth, ninth_elapsed = client.wait_for(epoch, "pending-9", 700)
        pressure_ok = ninth.status == "WAIT_PRESSURE" and ninth_elapsed < 0.5
        detail = f"{ninth.status} elapsed={ninth_elapsed:.3f}"
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not all(item.done() for item in futures):
            time.sleep(0.02)
        client.check("ninth wait is pressure", pressure_ok, detail)
        reset = ResetObservation.Request()
        reset.expected_epoch = epoch
        reset.reset_id = "s1g3-service"
        reset_response = client.call(client.reset, reset, 5)
        usage = client.usage()
        client.check("reset returns resource counts to the empty baseline",
                     reset_response.status == "OK" and reset_response.epoch == epoch + 1 and
                     usage.shared_block_bytes == 0 and usage.surface_cache_bytes == 0,
                     f"epoch={reset_response.epoch} blocks={usage.shared_block_bytes} cache={usage.surface_cache_bytes}")
        epoch = int(reset_response.epoch)
        client.set_param("hold_integrate", True)
        blocked = client.integrate_srv.call_async(_integrate_request(client, geometry, "held-a", epoch))
        time.sleep(0.3)
        still_held = not blocked.done()
        mark = len(client.health)
        health_during = None
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not blocked.done():
            if len(client.health) > mark:
                health_during = yaml.safe_load(client.health[-1])
                break
            time.sleep(0.05)
        client.check("health is published while integrate holds the store lock",
                     still_held and health_during is not None and not blocked.done(),
                     "no health" if health_during is None else str(health_during.get("epoch")))
        timeout_response, timeout_elapsed = client.wait_for(epoch, "nobody", 250)
        client.check("wait times out while the store lock is still held",
                     timeout_response.status == "TIMEOUT" and timeout_elapsed < 1.2 and not blocked.done(),
                     f"{timeout_response.status} elapsed={timeout_elapsed:.3f} held={not blocked.done()}")
        for name in ("held-b", "held-c"):
            client.integrate_srv.call_async(_integrate_request(client, geometry, name, epoch))
            time.sleep(0.05)
        full, _ = client.integrate(geometry, "held-d", epoch)
        full_status = client.query(epoch, "held-d")
        full_wait, full_elapsed = client.wait_for(epoch, "held-d", 1000)
        client.check("service worker queue full is rejected",
                     full.receipt.status == "QUEUE_FULL" and full_status.status == "REJECTED" and
                     full_status.receipt.status == "QUEUE_FULL" and full_wait.status == "REJECTED" and
                     full_wait.reason == "QUEUE_FULL" and full_elapsed < 1.0 and not blocked.done(),
                     f"{full.receipt.status} wait={full_wait.status}/{full_wait.reason} elapsed={full_elapsed:.3f}")
        client.set_param("hold_integrate", False)
        deadline = time.monotonic() + 5.0
        while not blocked.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        held_result = blocked.result() if blocked.done() else None
        client.check("held integrate completes after release",
                     held_result is not None and held_result.receipt.status == "INTEGRATED",
                     "pending" if held_result is None else held_result.receipt.status)
        return geometry
    finally:
        try:
            client.set_param("hold_integrate", False)
        except Exception:
            pass
        observer.stop()


def _wait_request(epoch, observation_id, timeout_ms, max_age_ms):
    request = WaitSnapshot.Request()
    request.expected_epoch = int(epoch)
    request.sensor_id = SENSOR
    request.observation_id = observation_id
    request.timeout_ms = int(timeout_ms)
    request.max_age_ms = int(max_age_ms)
    return request


def _integrate_request(client, geometry_hash, observation_id, epoch):
    stamp_ns = client.next_stamp()
    request = IntegrateObservation.Request()
    request.sensor_id = SENSOR
    request.observation_id = observation_id
    request.clock_domain = "ros"
    request.geometry_hash = geometry_hash
    request.epoch = int(epoch)
    request.cloud = client.cloud_message(stamp_ns)
    request.uld_from_optical = client.transform(stamp_ns)
    return request


def run_subscription(client, observer_factory):
    failed = observer_factory(["-p", "input_mode:=subscription", "-p", "sensor_id:=cam0",
                               "-p", "optical_frame:=" + FRAME, "-p", "cloud_topic:=" + CLOUD_TOPIC,
                               "-p", "behavior_test_hooks:=true"])
    try:
        client.wait_match()
        stamp, observation_id = client.publish_cloud()
        status = _poll_status(client, 1, observation_id, lambda item: item.status not in ("QUEUED", "PROCESSING", "NOT_SEEN"))
        waited, elapsed = client.wait_for(1, observation_id, 2000)
        client.check("tf failure wakes the waiter as rejected",
                     status is not None and status.status == "REJECTED" and status.receipt.status not in ("", "INTEGRATED", "TIMEOUT") and
                     waited.status == "REJECTED" and waited.reason == status.receipt.status and elapsed < 1.5,
                     f"query={None if status is None else status.status + '/' + status.receipt.status} wait={waited.status}/{waited.reason} elapsed={elapsed:.3f}")
    finally:
        failed.stop()
    client.broadcast_static_tf()
    time.sleep(0.3)
    paused = observer_factory(["-p", "input_mode:=subscription", "-p", "sensor_id:=cam0",
                               "-p", "optical_frame:=" + FRAME, "-p", "cloud_topic:=" + CLOUD_TOPIC,
                               "-p", "behavior_test_hooks:=true", "-p", "adapter_paused:=true"])
    try:
        client.wait_match()
        ids = [client.publish_cloud()[1] for _ in range(3)]
        time.sleep(0.3)
        third = client.query(1, ids[2])
        first = client.query(1, ids[0])
        refused, elapsed = client.wait_for(1, ids[2], 2000)
        usage = client.usage()
        client.check("third raw message is queue full",
                     third.status == "REJECTED" and third.receipt.status == "QUEUE_FULL" and
                     first.status == "QUEUED" and refused.status == "REJECTED" and
                     refused.reason == "QUEUE_FULL" and elapsed < 1.0 and usage.entry_queue_messages == 2 and
                     usage.entry_queue_payload_bytes > 0,
                     f"third={third.status}/{third.receipt.status} first={first.status} wait={refused.status}/{refused.reason} queued={usage.entry_queue_messages}")
        reset = ResetObservation.Request()
        reset.expected_epoch = 1
        reset.reset_id = "s1g3-sub"
        reset_response = client.call(client.reset, reset, 5)
        client.set_param("adapter_paused", False)
        adapted = _poll_status(client, 1, ids[0], lambda item: item.status == "REJECTED")
        health = client.latest_health()
        client.check("old epoch message is not written after reset",
                     reset_response.epoch == 2 and adapted is not None and adapted.receipt.status == "EPOCH_MISMATCH" and
                     int(health["epoch"]) == 2 and int(health["map_revision"]) == 0 and int(health["known_cells"]) == 0,
                     f"reset={reset_response.epoch} status={None if adapted is None else adapted.receipt.status} rev={health.get('map_revision')} cells={health.get('known_cells')}")
        client.set_param("hold_integrate", True)
        held_stamp, held_id = client.publish_cloud()
        held = _poll_status(client, 2, held_id, lambda item: item.status == "PROCESSING")
        time.sleep(0.3)
        if held is not None and held.status == "PROCESSING":
            held = client.query(2, held_id)
        mark = len(client.health)
        health_during = None
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if len(client.health) > mark:
                health_during = yaml.safe_load(client.health[-1])
                break
            time.sleep(0.05)
        client.check("subscription health continues while a frame is blocked",
                     held is not None and held.status == "PROCESSING" and health_during is not None,
                     "not processing" if held is None else held.status)
        extra = [client.publish_cloud()[1] for _ in range(3)]
        time.sleep(0.4)
        rejected = client.query(2, extra[2])
        reject_wait, reject_elapsed = client.wait_for(2, extra[2], 1500)
        client.check("subscription worker queue full is rejected",
                     rejected.status == "REJECTED" and rejected.receipt.status == "QUEUE_FULL" and
                     reject_wait.status == "REJECTED" and reject_wait.reason == "QUEUE_FULL" and reject_elapsed < 1.0,
                     f"{rejected.status}/{rejected.receipt.status} wait={reject_wait.status}/{reject_wait.reason}")
        client.set_param("hold_integrate", False)
        finished = _poll_status(client, 2, held_id, lambda item: item.status in ("COMPLETED", "REJECTED"))
        health = client.latest_health()
        client.check("blocked frame integrates after the hold is released",
                     finished is not None and finished.status == "COMPLETED" and int(health["map_revision"]) > 0,
                     "pending" if finished is None else f"{finished.status} rev={health.get('map_revision')}")
        del held_stamp
    finally:
        try:
            client.set_param("hold_integrate", False)
            client.set_param("adapter_paused", False)
        except Exception:
            pass
        paused.stop()


def _poll_status(client, epoch, observation_id, ready, timeout=2.0):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = client.query(epoch, observation_id)
        if ready(last):
            return last
        time.sleep(0.05)
    return last


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--node", required=True)
    parser.add_argument("--geometry", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--result", required=True)
    parser.add_argument("--log-dir", required=True)
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = "41"
    checks = []
    error = ""
    rclpy.init()
    client = Client()
    try:
        def start(extra):
            log = Path(args.log_dir) / f"observer-{len(list(Path(args.log_dir).glob('observer-*.log')))}.log"
            ros_args = ["-p", f"geometry_file:={args.geometry}", "-p", f"config_file:={args.config}", *extra]
            return Observer(args.node, ros_args, log)
        run_service(client, start)
        run_subscription(client, start)
        checks = client.checks
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        checks = client.checks
        print("FAIL harness " + error, flush=True)
    finally:
        client.close()
        if rclpy.ok():
            rclpy.shutdown()
    failed = [item for item in checks if not item["pass"]]
    payload = {"checks": checks, "failed": len(failed), "passed": len(checks) - len(failed),
               "harness_error": error}
    Path(args.result).parent.mkdir(parents=True, exist_ok=True)
    Path(args.result).write_text(json.dumps(payload, indent=2) + "\n")
    return 1 if failed or error else 0


if __name__ == "__main__":
    raise SystemExit(main())
