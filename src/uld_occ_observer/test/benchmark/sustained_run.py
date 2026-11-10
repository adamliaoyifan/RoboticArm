"""O7 sustained-run ledger and ROS harness.

The simulated clock covers send independence, late completion, rejection,
duplicate receipts, loss, drain timeout, and count conservation. The ROS
runner publishes on its own thread and does not wait for the previous frame.
"""
from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import fusion_report

HEALTH_GAP_S = 1.0
QUEUE_LIMIT_MS = 1000.0
DRAIN_S = 30.0
QUERY_P95 = fusion_report.REFERENCE_GATES["query"]["p95_ms"]
QUERY_MAX = fusion_report.REFERENCE_GATES["query"]["max_ms"]
INTEGRATE_30K = fusion_report.REFERENCE_GATES["integrate_30k"]
SENSOR_ROTATION = (0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0)
SENSOR_TRANSLATION = (-1.15, 0.0, 1.20)
OPTICAL_FRAME = "d555_color_optical_frame"


def rotation_to_quat(matrix):
    r00, r01, r02, r10, r11, r12, r20, r21, r22 = matrix
    trace = r00 + r11 + r22
    if trace > 0:
        scale = 0.5 / math.sqrt(trace + 1.0)
        return ((r21 - r12) * scale, (r02 - r20) * scale, (r10 - r01) * scale, 0.25 / scale)
    if r00 > r11 and r00 > r22:
        scale = 2.0 * math.sqrt(1.0 + r00 - r11 - r22)
        return (0.25 * scale, (r01 + r10) / scale, (r02 + r20) / scale, (r21 - r12) / scale)
    if r11 > r22:
        scale = 2.0 * math.sqrt(1.0 + r11 - r00 - r22)
        return ((r01 + r10) / scale, 0.25 * scale, (r12 + r21) / scale, (r02 - r20) / scale)
    scale = 2.0 * math.sqrt(1.0 + r22 - r00 - r11)
    return ((r02 + r20) / scale, (r12 + r21) / scale, 0.25 * scale, (r10 - r01) / scale)


def schedule(duration_s, hz):
    items = []
    planned = 0.0
    step = 1.0 / hz
    while planned < duration_s - 1e-9:
        items.append({"planned": planned})
        planned += step
    return items


def normal_schedule():
    return {"inputs": schedule(120.0, 2.0), "queries": schedule(120.0, 1.0),
            "duration_s": 120.0, "input_hz": 2.0, "query_hz": 1.0}


def overload_schedule():
    return {
        "overload_inputs": schedule(60.0, 10.0),
        "overload_queries": schedule(60.0, 1.0),
        "recovery_inputs": schedule(30.0, 2.0),
        "recovery_queries": schedule(30.0, 1.0),
        "overload_s": 60.0,
        "recovery_s": 30.0,
        "drain_s": DRAIN_S,
    }


class Ledger:
    def __init__(self):
        self.sent = {}
        self.receipts = {}

    def send(self, input_id, planned, actual, phase):
        if input_id in self.sent:
            raise ValueError("duplicate input id")
        self.sent[input_id] = {"planned": planned, "actual": actual, "phase": phase}

    def receipt(self, input_id, status, queue_ms, total_ms, received_at):
        self.receipts.setdefault(str(input_id), []).append({
            "status": status, "queue_ms": queue_ms, "total_ms": total_ms, "received_at": received_at})

    def summary(self, phase=None):
        sent_ids = [input_id for input_id, item in self.sent.items()
                    if phase is None or item["phase"] == phase]
        success = rejected = duplicates = unknown = 0
        pending = []
        for input_id in sent_ids:
            items = self.receipts.get(input_id) or []
            if not items:
                pending.append(input_id)
                continue
            duplicates += len(items) - 1
            if items[0]["status"] == "INTEGRATED":
                success += 1
            else:
                rejected += 1
        if phase is None:
            unknown = len([input_id for input_id in self.receipts if input_id not in self.sent])
        return {
            "sent": len(sent_ids),
            "success": success,
            "rejected": rejected,
            "pending": len(pending),
            "duplicate_receipts": duplicates,
            "unknown_receipts": unknown,
            "balanced": len(sent_ids) == success + rejected + len(pending) and unknown == 0,
        }


def simulate_publish(inputs, complete_after_s, reject_ids=(), drop_ids=(), duplicate_ids=(), drain_s=30.0):
    """Advance a virtual clock. Sending never waits for completion."""
    ledger = Ledger()
    events = []
    clock = 0.0
    for index, item in enumerate(inputs):
        clock = item["planned"] if item["planned"] >= clock else clock
        input_id = item.get("id", "in-%06d" % index)
        ledger.send(input_id, item["planned"], clock, item.get("phase", "score"))
        if input_id not in drop_ids:
            status = "QUEUE_FULL" if input_id in reject_ids else "INTEGRATED"
            events.append((clock + complete_after_s, input_id, status))
    stop = max((item["planned"] for item in inputs), default=0.0)
    for when, input_id, status in sorted(events):
        if when <= stop + drain_s:
            ledger.receipt(input_id, status, 0.0, 1.0, when)
            if input_id in duplicate_ids:
                ledger.receipt(input_id, status, 0.0, 1.0, when)
    return ledger


def health_covers(stamps, start, end, limit=HEALTH_GAP_S):
    """The scoring window, including both endpoints, has no receive gap of limit or more."""
    ordered = sorted(stamps)
    begin = None
    for stamp in ordered:
        if stamp <= start:
            begin = stamp
        else:
            break
    closing = next((stamp for stamp in ordered if stamp >= end), None)
    if begin is None or closing is None:
        return False
    window = [stamp for stamp in ordered if begin <= stamp <= closing]
    return all(right - left < limit for left, right in zip(window, window[1:]))


def judge_normal(ledger, queries, health_stamps, score_start, score_end):
    summary = ledger.summary("score")
    reasons = []
    if summary["sent"] != 240 or summary["success"] != 240 or summary["rejected"] or summary["pending"] \
            or summary["duplicate_receipts"] or not summary["balanced"]:
        reasons.append("input_conservation")
    if len(queries) != 120 or any(item.get("status") != "OK" for item in queries):
        reasons.append("query_conservation")
    queue_values = [item["queue_ms"] for input_id, item in _first_receipts(ledger, "score")]
    integrate_values = [item["total_ms"] for input_id, item in _first_receipts(ledger, "score")]
    integrate = fusion_report.exact_stats(integrate_values, 240)
    if not fusion_report.within_gate(integrate, INTEGRATE_30K["p95_ms"], INTEGRATE_30K["max_ms"]):
        reasons.append("integrate")
    if not queue_values or max(queue_values) > QUEUE_LIMIT_MS:
        reasons.append("queue_age")
    for name in ("acquire_ms", "roi_ms", "surface_ms"):
        values = [item.get(name) for item in queries]
        if any(not isinstance(value, (int, float)) or isinstance(value, bool) for value in values):
            reasons.append(name)
            continue
        stats = fusion_report.exact_stats(values, 120)
        if not fusion_report.within_gate(stats, QUERY_P95, QUERY_MAX):
            reasons.append(name)
    if not health_covers(health_stamps, score_start, score_end):
        reasons.append("health_gap")
    client = [item["received_at"] - ledger.sent[input_id]["actual"]
              for input_id, item in _first_receipts(ledger, "score")]
    return {
        "pass": not reasons,
        "reasons": reasons,
        "conservation": summary,
        "integrate": integrate,
        "queue_max_ms": max(queue_values) if queue_values else None,
        "client_latency_ms": fusion_report.exact_stats([value * 1000.0 for value in client], 240) if len(client) == 240 else {"status": "incomplete"},
    }


def _first_receipts(ledger, phase):
    pairs = []
    for input_id, sent in ledger.sent.items():
        if sent["phase"] != phase:
            continue
        items = ledger.receipts.get(input_id) or []
        if items:
            pairs.append((input_id, items[0]))
    return pairs


def judge_overload(ledger, recovery_queries, observed_queue_or_reject):
    overload = ledger.summary("overload")
    recovery = ledger.summary("recovery")
    if not observed_queue_or_reject:
        return {"status": "not_evaluated", "reason": "overload did not observe a queue or an explicit rejection",
                "overload": overload, "recovery": recovery}
    reasons = []
    if not overload["balanced"] or overload["pending"]:
        reasons.append("overload_accounting")
    if recovery["sent"] != 60 or recovery["success"] != 60 or recovery["pending"] or recovery["rejected"] \
            or recovery["duplicate_receipts"]:
        reasons.append("recovery_inputs")
    if len(recovery_queries) != 30 or any(item.get("status") != "OK" for item in recovery_queries):
        reasons.append("recovery_queries")
    return {"status": "pass" if not reasons else "fail", "reasons": reasons,
            "overload": overload, "recovery": recovery}


class SustainedRun:
    """ROS subscriber-mode run. Simulated tests do not construct this."""

    def __init__(self, node_path, install_setup, geometry, config, points, out, domain_id=17):
        self.node_path = str(node_path)
        self.install_setup = str(install_setup)
        self.geometry = str(geometry)
        self.config = str(config)
        self.points = points
        self.out = Path(out)
        self.domain_id = domain_id

    def run_normal(self):
        return self._run("normal", normal_schedule())

    def run_overload(self):
        return self._run("overload", overload_schedule())

    def rehearse(self, seconds=2.0):
        spec = {"inputs": schedule(seconds, 2.0), "queries": schedule(seconds, 1.0),
                "duration_s": seconds, "rehearsal": True}
        result = self._run("rehearsal", spec)
        result["product_scored"] = False
        if result.get("status") != "startup_failed":
            result["status"] = "rehearsal"
        return result

    def _run(self, kind, spec):
        self.out.mkdir(parents=True, exist_ok=True)
        env = _ros_env(self.install_setup, self.domain_id)
        log = self.out / "observer.log"
        handle = log.open("w")
        proc = subprocess.Popen(
            [self.node_path, "--ros-args",
             "-p", "geometry_file:=" + self.geometry,
             "-p", "config_file:=" + self.config,
             "-p", "input_mode:=subscription",
             "-p", "optical_frame:=" + OPTICAL_FRAME,
             "-p", "cloud_topic:=optical_points",
             "-p", "sensor_id:=cam0"],
            stdout=handle, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        handle.close()
        try:
            return _ros_session(self.points, spec, kind, self.out, env)
        finally:
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()


def _ros_env(setup, domain_id):
    command = "set +u; source /opt/ros/humble/setup.bash; source %s; python3 -c 'import os,json; print(json.dumps(dict(os.environ)))'" % setup
    base = os.environ.copy()
    for key in ("LD_LIBRARY_PATH", "AMENT_PREFIX_PATH", "PYTHONPATH", "CMAKE_PREFIX_PATH", "ROS_PACKAGE_PATH"):
        base.pop(key, None)
    result = subprocess.run(["bash", "-lc", command], capture_output=True, text=True, check=False, env=base)
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "unable to source ROS install")
    env = json.loads(result.stdout.splitlines()[-1])
    env["ROS_DOMAIN_ID"] = str(domain_id)
    return env


def _ros_session(points, spec, kind, out, env):
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import String
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from uld_occ_observer.msg import UpdateReceipt
    from uld_occ_observer.srv import GetSnapshot, ReadCells, ReadSurface, ReleaseSnapshot
    import yaml

    os.environ.update({key: env[key] for key in ("ROS_DOMAIN_ID",) if key in env})
    rclpy.init()
    node = Node("o7_harness")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    ledger = Ledger()
    health_times = []
    latest_health = {}
    queries = []
    recovery_queries = []
    observed = {"queue": False}
    phase_box = {"name": "startup"}
    os.environ.update(env)
    try:
        broadcaster = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.frame_id = "container_link"
        transform.child_frame_id = OPTICAL_FRAME
        qx, qy, qz, qw = rotation_to_quat(SENSOR_ROTATION)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        transform.transform.translation.x = SENSOR_TRANSLATION[0]
        transform.transform.translation.y = SENSOR_TRANSLATION[1]
        transform.transform.translation.z = SENSOR_TRANSLATION[2]
        broadcaster.sendTransform(transform)
        publisher = node.create_publisher(PointCloud2, "/uld_occ_observer/optical_points", 10)
        node.create_subscription(UpdateReceipt, "/uld_occ_observer/receipts",
                                 lambda msg: _on_receipt(ledger, observed, phase_box, msg), 100)
        node.create_subscription(String, "/uld_occ_observer/health",
                                 lambda msg: _on_health(health_times, latest_health, observed, phase_box, msg, yaml), 10)
        clients = {
            "acquire": node.create_client(GetSnapshot, "/uld_occ_observer/get_snapshot"),
            "roi": node.create_client(ReadCells, "/uld_occ_observer/read_cells"),
            "surface": node.create_client(ReadSurface, "/uld_occ_observer/read_surface"),
            "release": node.create_client(ReleaseSnapshot, "/uld_occ_observer/release_snapshot"),
        }
        if not _wait_ready(clients, health_times, 30.0):
            return {"status": "startup_failed", "kind": kind}
        time.sleep(0.3)
        payload = _pack_xyz(points, PointCloud2, PointField)
        _publish_phase(node, publisher, payload, [{"planned": index * 0.5} for index in range(3)],
                       ledger, "warmup", PointCloud2)
        if not _wait_queue_clear(ledger, latest_health, 30.0):
            return {"status": "startup_failed", "kind": kind, "reason": "warmup queue did not clear"}
        score_start = time.monotonic()
        if kind == "overload":
            phase_box["name"] = "overload"
            _run_phase(node, publisher, payload, clients, spec["overload_inputs"], spec["overload_queries"],
                       ledger, queries, "overload", PointCloud2, GetSnapshot, ReadCells, ReadSurface, ReleaseSnapshot)
            _drain(ledger, queries, DRAIN_S)
            phase_box["name"] = "recovery"
            _run_phase(node, publisher, payload, clients, spec["recovery_inputs"], spec["recovery_queries"],
                       ledger, recovery_queries, "recovery", PointCloud2, GetSnapshot, ReadCells, ReadSurface, ReleaseSnapshot)
            _drain(ledger, recovery_queries, DRAIN_S)
            verdict = judge_overload(ledger, recovery_queries, observed["queue"])
        else:
            phase_box["name"] = "score"
            _run_phase(node, publisher, payload, clients, spec["inputs"], spec["queries"],
                       ledger, queries, "score", PointCloud2, GetSnapshot, ReadCells, ReadSurface, ReleaseSnapshot)
            _drain(ledger, queries, DRAIN_S)
            score_end = score_start + spec["duration_s"]
            # Health is published every 500 ms. Wait for one sample at or after
            # the scoring endpoint so the final gap is observed, not assumed.
            cover_deadline = time.monotonic() + 1.2
            while time.monotonic() < cover_deadline and (not health_times or health_times[-1] < score_end):
                time.sleep(0.05)
            verdict = judge_normal(ledger, queries, health_times, score_start, score_end)
            if spec.get("rehearsal"):
                verdict = {"status": "rehearsal", "product_scored": False, "queries": len(queries),
                           "inputs": ledger.summary("score")}
        (out / "ledger.json").write_text(json.dumps({
            "sent": ledger.sent, "receipts": ledger.receipts, "queries": queries,
            "recovery_queries": recovery_queries, "health_times": health_times}, indent=2) + "\n")
        verdict["kind"] = kind
        return verdict
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _on_receipt(ledger, observed, phase_box, msg):
    ledger.receipt(msg.observation_id, msg.status, float(msg.queue_ms), float(msg.total_ms), time.monotonic())
    sent = ledger.sent.get(str(msg.observation_id))
    if phase_box.get("name") == "overload" and sent and sent["phase"] == "overload":
        if msg.status != "INTEGRATED" or float(msg.queue_ms) > 0:
            observed["queue"] = True


def _on_health(health_times, latest, observed, phase_box, msg, yaml):
    health_times.append(time.monotonic())
    try:
        latest.clear()
        latest.update(yaml.safe_load(msg.data) or {})
    except yaml.YAMLError:
        latest["parse_error"] = True
        return
    if phase_box.get("name") == "overload" and (
            int(latest.get("waiting") or 0) > 0 or int(latest.get("in_flight") or 0) > 0 or
            int(latest.get("queue_rejected") or 0) > 0):
        observed["queue"] = True


def _wait_ready(clients, health_times, limit_s):
    deadline = time.monotonic() + limit_s
    while time.monotonic() < deadline:
        if health_times and all(client.service_is_ready() for client in clients.values()):
            return True
        for client in clients.values():
            client.wait_for_service(timeout_sec=0.1)
    return False


def _wait_queue_clear(ledger, latest, limit_s):
    deadline = time.monotonic() + limit_s
    while time.monotonic() < deadline:
        summary = ledger.summary("warmup")
        integrated = []
        for input_id, sent in ledger.sent.items():
            if sent["phase"] != "warmup":
                continue
            items = ledger.receipts.get(input_id) or []
            integrated.append(bool(items) and items[0]["status"] == "INTEGRATED")
        if summary["sent"] == 3 and integrated.count(True) == 3 and int(latest.get("waiting") or 0) == 0 \
                and int(latest.get("in_flight") or 0) == 0:
            return True
        time.sleep(0.05)
    return False


def _pack_xyz(points, point_cloud, point_field):
    import struct
    buffer = bytearray()
    for point in points:
        buffer += struct.pack("<fff", float(point[0]), float(point[1]), float(point[2]))
    return bytes(buffer), point_cloud, point_field


def _publish_phase(node, publisher, payload, items, ledger, phase, point_cloud, origin=None):
    raw, cloud_type, field_type = payload
    if origin is None:
        origin = time.monotonic()
    for index, item in enumerate(items):
        target = origin + item["planned"]
        delay = target - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        actual = time.monotonic()
        stamp = node.get_clock().now().to_msg()
        message = cloud_type()
        message.header.stamp = stamp
        message.header.frame_id = OPTICAL_FRAME
        message.height = 1
        message.width = len(raw) // 12
        message.fields = [
            field_type(name="x", offset=0, datatype=field_type.FLOAT32, count=1),
            field_type(name="y", offset=4, datatype=field_type.FLOAT32, count=1),
            field_type(name="z", offset=8, datatype=field_type.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = len(raw)
        message.data = raw
        stamp_ns = int(stamp.sec) * 1000000000 + int(stamp.nanosec)
        while str(stamp_ns) in ledger.sent:
            stamp_ns += 1
        message.header.stamp.sec = stamp_ns // 1000000000
        message.header.stamp.nanosec = stamp_ns % 1000000000
        publisher.publish(message)
        ledger.send(str(stamp_ns), item["planned"], actual, phase)


def _run_phase(node, publisher, payload, clients, inputs, queries, ledger, sink, phase,
               point_cloud, get_snapshot, read_cells, read_surface, release_snapshot):
    done = threading.Event()
    origin = time.monotonic()

    def send():
        try:
            _publish_phase(node, publisher, payload, inputs, ledger, phase, point_cloud, origin)
        finally:
            done.set()

    thread = threading.Thread(target=send)
    thread.start()
    for item in queries:
        target = origin + item["planned"]
        delay = target - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        sink.append(_query_chain(clients, get_snapshot, read_cells, read_surface, release_snapshot))
    done.wait()
    thread.join()


def _query_chain(clients, get_snapshot, read_cells, read_surface, release_snapshot):
    started = time.monotonic()
    acquire_req = get_snapshot.Request()
    acquire_req.expected_epoch = 0
    acquire_req.exact_revision = False
    acquire_req.expected_revision = 0
    acquire_req.max_age_ms = 2000
    acquire_req.allow_stale = False
    t0 = time.monotonic()
    acquired = _call(clients["acquire"], acquire_req, 5.0)
    t1 = time.monotonic()
    record = {"status": "ERROR", "acquire_ms": (t1 - t0) * 1000.0, "roi_ms": None, "surface_ms": None,
              "identity": None, "started": started}
    if acquired is None or acquired.status != "OK":
        record["status"] = None if acquired is None else acquired.status
        return record
    handle = acquired.handle
    identity = _identity(acquired.identity)
    try:
        roi_req = read_cells.Request()
        roi_req.handle = handle
        roi_req.begin = [0, 0, 0]
        roi_req.end = [16, 16, 16]
        roi_req.cursor = 0
        roi_req.limit = 4096
        t2 = time.monotonic()
        roi = _call(clients["roi"], roi_req, 5.0)
        t3 = time.monotonic()
        record["roi_ms"] = (t3 - t2) * 1000.0
        surface_req = read_surface.Request()
        surface_req.handle = handle
        surface_req.cursor = 0
        surface_req.limit = 65536
        t4 = time.monotonic()
        surface = _call(clients["surface"], surface_req, 5.0)
        t5 = time.monotonic()
        record["surface_ms"] = (t5 - t4) * 1000.0
        if roi is None or surface is None or roi.status != "OK" or surface.status != "OK" or not surface.done:
            record["status"] = "INCOMPLETE"
            return record
        if _identity(roi.identity) != identity or _identity(surface.identity) != identity:
            record["status"] = "IDENTITY_MISMATCH"
            return record
        record["identity"] = identity
        record["status"] = "OK"
        return record
    finally:
        release_req = release_snapshot.Request()
        release_req.handle = handle
        _call(clients["release"], release_req, 5.0)


def _identity(message):
    return (int(message.epoch), int(message.map_revision), int(message.observed_revision),
            int(message.last_integrated_stamp_ns), message.geometry_hash, message.config_digest,
            message.clock_domain)


def _call(client, request, timeout_s):
    future = client.call_async(request)
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.002)
    if not future.done():
        return None
    return future.result()


def _drain(ledger, queries, limit_s):
    del queries
    deadline = time.monotonic() + limit_s
    while time.monotonic() < deadline:
        if ledger.summary()["pending"] == 0:
            return
        time.sleep(0.05)


def _points_from_frame(path):
    points = []
    for line in Path(path).read_text().splitlines():
        if not line.startswith("- ["):
            continue
        body = line[line.find("[") + 1:line.rfind("]")]
        points.append([float(item) for item in body.split(",")])
    return points


def ensure_points(native, geometry, config, boxes, dest):
    dest = Path(dest)
    if dest.exists():
        return json.loads(dest.read_text())
    folder = dest.with_suffix(".emit")
    if folder.exists():
        import shutil
        shutil.rmtree(folder)
    result = subprocess.run(
        [str(native), "--geometry", str(geometry), "--config", str(config), "--gate", "emit-window",
         "--out", str(folder), "--group", "o7_b%02d" % boxes, "--boxes", str(boxes),
         "--cloud", "30000", "--outside", "0.5", "--cross", "0", "--warmup", "0", "--repeat", "1"],
        capture_output=True, text=True, check=False)
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "O7 point generation failed")
    points = _points_from_frame(folder / "frames" / "f000.yaml")
    if len(points) != 30000:
        raise RuntimeError("O7 cloud does not contain 30000 points")
    dest.write_text(json.dumps(points))
    return points


def _scene(fixtures, boxes):
    manifest = json.loads((Path(fixtures) / "TEST_MANIFEST.json").read_text())
    matches = [scene for scene in manifest["scenes"]
               if scene["resolution"] == 0.01 and scene["hull"] == "chamfer" and scene["boxes"] == boxes]
    if len(matches) != 1:
        raise RuntimeError("O7 fixture scene is missing")
    return matches[0]


def run_campaign(args):
    import fcntl
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    lock_file = open("/tmp/elfin-occ-backend-perf.lock", "a")
    fcntl.flock(lock_file, fcntl.LOCK_EX)
    try:
        (out / "normal_schedule.json").write_text(json.dumps(normal_schedule(), indent=2) + "\n")
        (out / "overload_schedule.json").write_text(json.dumps(overload_schedule(), indent=2) + "\n")
        startup_failures = []
        if args.rehearsal:
            scene = _scene(args.fixtures, 0)
            points = out / "points_b00.json"
            ensure_points(args.eval_native, scene["geometry_file"], scene["config_file"], 0, points)
            code = _window_process(args, "rehearsal", scene, points, out / "rehearsal", startup_failures)
            (out / "startup_failures.json").write_text(json.dumps(startup_failures, indent=2) + "\n")
            return 0 if code == 0 else 3
        windows = []
        for boxes in (0, 5, 20):
            scene = _scene(args.fixtures, boxes)
            points = out / ("points_b%02d.json" % boxes)
            ensure_points(args.eval_native, scene["geometry_file"], scene["config_file"], boxes, points)
            for repetition in range(3):
                folder = out / ("normal_b%02d_r%d" % (boxes, repetition))
                code = _window_process(args, "normal", scene, points, folder, startup_failures)
                windows.append({"boxes": boxes, "repetition": repetition, "code": code})
        overload = _window_process(args, "overload", _scene(args.fixtures, 20),
                                   out / "points_b20.json", out / "overload", startup_failures)
        (out / "startup_failures.json").write_text(json.dumps(startup_failures, indent=2) + "\n")
        (out / "O7_VERDICT.json").write_text(json.dumps({
            "windows": windows, "overload_code": overload, "startup_failures": len(startup_failures),
            "pooled_latency": "not_computed"}, indent=2) + "\n")
        if any(item["code"] == 3 for item in windows) or overload == 3:
            return 3
        if any(item["code"] != 0 for item in windows) or overload != 0:
            return 4
        return 0
    finally:
        fcntl.flock(lock_file, fcntl.LOCK_UN)
        lock_file.close()


def _window_process(args, kind, scene, points, folder, startup_failures):
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    command = [sys.executable, str(Path(__file__).resolve()),
               "--kind", kind, "--node", args.node, "--setup", args.setup,
               "--geometry", scene["geometry_file"], "--config", scene["config_file"],
               "--points", str(points), "--out", str(folder), "--domain-id", str(args.domain_id)]
    last = 4
    for attempt in range(3):
        result = subprocess.run(command, check=False)
        last = result.returncode
        if last != 4:
            return last
        startup_failures.append({"kind": kind, "out": str(folder), "attempt": attempt})
    return last


def main(argv):
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--kind", required=True, choices=("normal", "overload", "rehearsal"))
    parser.add_argument("--node", required=True)
    parser.add_argument("--setup", required=True)
    parser.add_argument("--geometry", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--points", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--domain-id", type=int, default=17)
    args = parser.parse_args(argv)
    points = json.loads(Path(args.points).read_text())
    runner = SustainedRun(args.node, args.setup, args.geometry, args.config, points, args.out, args.domain_id)
    if args.kind == "rehearsal":
        result = runner.rehearse(2.0)
    elif args.kind == "overload":
        result = runner.run_overload()
    else:
        result = runner.run_normal()
    (Path(args.out) / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    status = result.get("status") or ("pass" if result.get("pass") else "fail")
    if status == "startup_failed":
        return 4
    if status in ("pass", "rehearsal"):
        return 0
    if status == "not_evaluated":
        return 4
    return 3


if __name__ == "__main__":
    raise SystemExit(main(__import__("sys").argv[1:]))
