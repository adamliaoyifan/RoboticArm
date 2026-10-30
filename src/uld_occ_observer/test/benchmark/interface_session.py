"""One not-scored ROS window for the interface collection.

The product gates are not applied here. The process records client and server
boundaries and leaves accounting to the report.
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import sustained_run

DRAIN_S = 30.0


class EventLog:
    def __init__(self, run_id, pid):
        self.run_id = run_id
        self.pid = pid
        self._seq = 0
        self._lock = threading.Lock()
        self.rows = []

    def add(self, phase, event, request_id, status, chain_id=None, input_id=None, mono_ns=None):
        with self._lock:
            self._seq += 1
            row = {
                "schema_version": 1,
                "run_id": self.run_id,
                "phase": phase,
                "event_seq": self._seq,
                "event": event,
                "mono_ns": time.monotonic_ns() if mono_ns is None else mono_ns,
                "pid": self.pid,
                "tid": threading.get_native_id(),
                "thread_cpu_ns": time.thread_time_ns(),
                "request_id": request_id,
                "chain_id": chain_id,
                "input_id": input_id,
                "status": status,
            }
            self.rows.append(row)
            return row


def proc_cpu_ns(pid):
    text = Path("/proc/%d/stat" % pid).read_text()
    fields = text[text.rfind(")") + 2:].split()
    ticks = os.sysconf("SC_CLK_TCK")
    return int((int(fields[11]) + int(fields[12])) * 1_000_000_000 / ticks)


def _stop_process(proc):
    if proc.poll() is not None:
        return proc.returncode
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        return proc.poll()
    try:
        return proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return proc.poll()
    try:
        return proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        return proc.wait()


def _sleep_until(target_ns):
    while True:
        now = time.monotonic_ns()
        if now >= target_ns:
            return now
        time.sleep(min(0.05, (target_ns - now) / 1_000_000_000))


def _publish(node, publisher, payload, items, ledger, phase, point_cloud, events, origin_ns):
    raw, cloud_type, field_type = payload
    for item in items:
        _sleep_until(origin_ns + item["planned_ns"])
        actual = time.monotonic()
        stamp = node.get_clock().now().to_msg()
        message = cloud_type()
        message.header.stamp = stamp
        message.header.frame_id = sustained_run.OPTICAL_FRAME
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
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        while str(stamp_ns) in ledger.sent:
            stamp_ns += 1
        message.header.stamp.sec = stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = stamp_ns % 1_000_000_000
        input_id = str(stamp_ns)
        events.add("input", "publish_begin", input_id, "", input_id=input_id)
        publisher.publish(message)
        events.add("input", "publish_end", input_id, "published_not_accepted", input_id=input_id)
        ledger.send(input_id, item["planned_ns"], actual, phase)
        item["input_id"] = input_id


def _call(client, request, timeout_s, events, phase, name, chain_id):
    cpu_before = time.thread_time_ns()
    send = time.monotonic_ns()
    events.add(phase, "rpc_send", name, "", chain_id=chain_id, mono_ns=send)
    future = client.call_async(request)
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.002)
    result_mono = time.monotonic_ns()
    late = not future.done()
    result = None if late else future.result()
    status = "timeout" if late else (getattr(result, "status", None) or "")
    events.add(phase, "rpc_result", name, status, chain_id=chain_id, mono_ns=result_mono)
    return {
        "send_mono_ns": send,
        "result_mono_ns": result_mono,
        "late": late,
        "result": result,
        "status": None if result is None else result.status,
        "client_thread_cpu_ns": time.thread_time_ns() - cpu_before,
        "future": future,
    }


def _identity(message):
    if message is None:
        return None
    return [int(message.epoch), int(message.map_revision), int(message.observed_revision),
            int(message.last_integrated_stamp_ns), message.geometry_hash, message.config_digest,
            message.clock_domain]


def _query(clients, services, events, chain_id, max_age_ms, node_pid, second_surface):
    import interface_events
    started = time.monotonic_ns()
    planned = started
    process_before = proc_cpu_ns(node_pid)
    acquire_req = services["get"].Request()
    acquire_req.expected_epoch = 0
    acquire_req.exact_revision = False
    acquire_req.expected_revision = 0
    acquire_req.max_age_ms = max_age_ms
    acquire_req.allow_stale = False
    acquired = _call(clients["acquire"], acquire_req, 5.0, events, "query", "get_snapshot", chain_id)
    record = {
        "chain_id": chain_id,
        "planned_mono_ns": planned,
        "start_mono_ns": started,
        "lateness_ns": 0,
        "max_age_ms": max_age_ms,
        "allow_stale": False,
        "acquire": {key: acquired[key] for key in ("send_mono_ns", "result_mono_ns", "late", "status", "client_thread_cpu_ns")},
        "process_cpu_before_ns": process_before,
    }
    result = acquired["result"]
    if result is None or result.status != "OK":
        record["status"] = record["acquire"]["status"]
        record["process_cpu_after_ns"] = proc_cpu_ns(node_pid)
        record["end_mono_ns"] = time.monotonic_ns()
        return record
    handle = result.handle
    record["handle"] = handle
    record["age_ns"] = int(result.age_ns)
    record["identity"] = _identity(result.identity)
    record["stale_flag"] = bool(result.stale)
    roi_req = services["roi"].Request()
    roi_req.handle = handle
    roi_req.begin = [0, 0, 0]
    roi_req.end = [16, 16, 16]
    roi_req.cursor = 0
    roi_req.limit = 4096
    roi = _call(clients["roi"], roi_req, 5.0, events, "query", "read_cells", chain_id)
    roi_result = roi["result"]
    roi_record = {key: roi[key] for key in ("send_mono_ns", "result_mono_ns", "late", "status", "client_thread_cpu_ns")}
    if roi_result is not None and roi_result.status == "OK":
        roi_record["length"] = len(roi_result.states)
        roi_record["done"] = bool(roi_result.done)
        roi_record["next_cursor"] = int(roi_result.next_cursor)
        roi_record["content_sha256"] = interface_events.content_sha256(
            interface_events.roi_content_bytes(roi_result.states, roi_result.log_odds))
        roi_record["identity"] = _identity(roi_result.identity)
    record["roi"] = roi_record
    surface_req = services["surface"].Request()
    surface_req.handle = handle
    surface_req.cursor = 0
    surface_req.limit = 65536
    surface = _call(clients["surface"], surface_req, 5.0, events, "query", "read_surface", chain_id)
    surface_result = surface["result"]
    surface_record = {key: surface[key] for key in ("send_mono_ns", "result_mono_ns", "late", "status", "client_thread_cpu_ns")}
    if surface_result is not None and surface_result.status == "OK":
        columns = []
        for index in range(len(surface_result.x)):
            columns.append({
                "x": int(surface_result.x[index]),
                "y": int(surface_result.y[index]),
                "active_count": int(surface_result.active_count[index]),
                "known_count": int(surface_result.known_count[index]),
                "has_occupied": bool(surface_result.has_occupied[index]),
                "top_z": float(surface_result.top_z[index]) if surface_result.has_occupied[index] else None,
            })
        surface_record["length"] = len(columns)
        surface_record["done"] = bool(surface_result.done)
        surface_record["next_cursor"] = int(surface_result.next_cursor)
        surface_record["content_sha256"] = interface_events.content_sha256(
            interface_events.surface_content_bytes(columns))
        surface_record["identity"] = _identity(surface_result.identity)
    record["surface"] = surface_record
    if second_surface and surface_result is not None and surface_result.status == "OK":
        again = _call(clients["surface"], surface_req, 5.0, events, "query", "read_surface_repeat", chain_id)
        record["surface_repeat_status"] = again["status"]
    release_req = services["release"].Request()
    release_req.handle = handle
    released = _call(clients["release"], release_req, 5.0, events, "query", "release_snapshot", chain_id)
    record["release"] = {key: released[key] for key in ("send_mono_ns", "result_mono_ns", "late", "status", "client_thread_cpu_ns")}
    record["status"] = "OK" if (roi_record.get("status") == "OK" and surface_record.get("status") == "OK"
                                and record["release"]["status"] == "OK") else "INCOMPLETE"
    record["process_cpu_after_ns"] = proc_cpu_ns(node_pid)
    record["end_mono_ns"] = time.monotonic_ns()
    if result is not None:
        record["acquire"]["future_checked_late"] = False
    return record


def _run_queries(clients, services, events, queries, origin_ns, max_age_ms, node_pid, second_surface, chain_offset):
    records = []
    for index, item in enumerate(queries):
        planned_ns = origin_ns + item["planned_ns"]
        _sleep_until(planned_ns)
        chain_id = "q%04d" % (chain_offset + index)
        actual = time.monotonic_ns()
        events.add("query", "query_planned", chain_id, "", chain_id=chain_id, mono_ns=planned_ns)
        record = _query(clients, services, events, chain_id, max_age_ms, node_pid, second_surface)
        record["planned_mono_ns"] = planned_ns
        record["lateness_ns"] = actual - planned_ns
        record["role"] = item.get("role", "measure")
        records.append(record)
    return records


def session(args):
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

    def _stop_requested(signum, frame):
        del signum, frame
        raise SystemExit(4)

    signal.signal(signal.SIGTERM, _stop_requested)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    points = json.loads(Path(args.points).read_text())
    env = sustained_run._ros_env(args.setup, args.domain_id)
    env["ULD_OCC_INTERFACE_TRACE"] = "1"
    env["ULD_OCC_TRACE_PATH"] = str(out / "server_events.jsonl")
    env["ULD_OCC_TRACE_RUN_ID"] = args.run_id
    log = (out / "observer.log").open("w")
    proc = subprocess.Popen(
        [args.node, "--ros-args",
         "-p", "geometry_file:=" + args.geometry,
         "-p", "config_file:=" + args.config,
         "-p", "input_mode:=subscription",
         "-p", "optical_frame:=" + sustained_run.OPTICAL_FRAME,
         "-p", "cloud_topic:=optical_points",
         "-p", "sensor_id:=cam0"],
        stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    events = EventLog(args.run_id, os.getpid())
    status = "error"
    queries = []
    health_rows = []
    node = None
    executor = None
    try:
        os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
        rclpy.init()
        node = Node("interface_collection_harness")
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        threading.Thread(target=executor.spin, daemon=True).start()
        ledger = sustained_run.Ledger()
        latest_health = {}
        broadcaster = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.frame_id = "container_link"
        transform.child_frame_id = sustained_run.OPTICAL_FRAME
        qx, qy, qz, qw = sustained_run.rotation_to_quat(sustained_run.SENSOR_ROTATION)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        transform.transform.translation.x = sustained_run.SENSOR_TRANSLATION[0]
        transform.transform.translation.y = sustained_run.SENSOR_TRANSLATION[1]
        transform.transform.translation.z = sustained_run.SENSOR_TRANSLATION[2]
        broadcaster.sendTransform(transform)
        publisher = node.create_publisher(PointCloud2, "/uld_occ_observer/optical_points", 10)

        def on_receipt(msg):
            ledger.receipt(msg.observation_id, msg.status, float(msg.queue_ms), float(msg.total_ms), time.monotonic())
            events.add("receipt", "client_receipt", str(msg.observation_id), msg.status, input_id=str(msg.observation_id))

        def on_health(msg):
            mono = time.monotonic_ns()
            parsed = True
            try:
                latest_health.clear()
                latest_health.update(yaml.safe_load(msg.data) or {})
            except yaml.YAMLError:
                parsed = False
                latest_health["parse_error"] = True
            health_rows.append({"mono_ns": mono, "text": msg.data, "parse_ok": parsed})
            events.add("health", "health_receive", "", "ok" if parsed else "parse_error", mono_ns=mono)

        node.create_subscription(UpdateReceipt, "/uld_occ_observer/receipts", on_receipt, 100)
        node.create_subscription(String, "/uld_occ_observer/health", on_health, 10)
        clients = {
            "acquire": node.create_client(GetSnapshot, "/uld_occ_observer/get_snapshot"),
            "roi": node.create_client(ReadCells, "/uld_occ_observer/read_cells"),
            "surface": node.create_client(ReadSurface, "/uld_occ_observer/read_surface"),
            "release": node.create_client(ReleaseSnapshot, "/uld_occ_observer/release_snapshot"),
        }
        services = {"get": GetSnapshot, "roi": ReadCells, "surface": ReadSurface, "release": ReleaseSnapshot}
        ready_deadline = time.monotonic() + 30.0
        ready = False
        while time.monotonic() < ready_deadline:
            if health_rows and all(client.service_is_ready() for client in clients.values()):
                ready = True
                break
            for client in clients.values():
                client.wait_for_service(timeout_sec=0.1)
        if not ready:
            status = "startup_failed"
            return status
        payload = sustained_run._pack_xyz(points, PointCloud2, PointField)
        warmup_origin = time.monotonic_ns()
        _publish(node, publisher, payload, [{"planned_ns": index * 500_000_000} for index in range(3)],
                 ledger, "warmup", PointCloud2, events, warmup_origin)
        if not sustained_run._wait_queue_clear(ledger, latest_health, 30.0):
            status = "startup_failed"
            return status
        spec = json.loads(Path(args.spec).read_text())
        if spec["kind"] == "recovery":
            time.sleep(spec["idle_s"])
        origin_ns = time.monotonic_ns()
        inputs = spec["inputs"]
        query_items = spec["queries"]
        done = threading.Event()

        def send():
            try:
                _publish(node, publisher, payload, inputs, ledger, spec["phase"], PointCloud2, events, origin_ns)
            finally:
                done.set()

        if spec["kind"] == "recovery" and spec["arm"] == "R1":
            threading.Thread(target=send).start()
            wait_deadline = time.monotonic() + 10.0
            while time.monotonic() < wait_deadline and not (inputs and inputs[0].get("input_id")):
                time.sleep(0.005)
            input_id = inputs[0].get("input_id") if inputs else None
            integrated = False
            while time.monotonic() < wait_deadline and input_id:
                items = ledger.receipts.get(input_id) or []
                if items and items[0]["status"] == "INTEGRATED":
                    integrated = True
                    break
                time.sleep(0.005)
            query_origin = time.monotonic_ns()
            queries = _run_queries(clients, services, events, query_items, query_origin, spec["max_age_ms"],
                                   proc.pid, spec.get("second_surface", False), 0)
            if queries:
                queries[0]["recovery_wait"] = "integrated" if integrated else "timeout"
                queries[0]["recovery_input_id"] = input_id
        else:
            threading.Thread(target=send).start()
            queries = _run_queries(clients, services, events, query_items, origin_ns, spec["max_age_ms"],
                                   proc.pid, spec.get("second_surface", False), 0)
        done.wait()
        sustained_run._drain(ledger, queries, spec.get("drain_s", DRAIN_S))
        if spec.get("recovery"):
            recovery = spec["recovery"]
            recovery_origin = time.monotonic_ns()
            recovery_done = threading.Event()

            def send_recovery():
                try:
                    _publish(node, publisher, payload, recovery["inputs"], ledger, "recovery",
                             PointCloud2, events, recovery_origin)
                finally:
                    recovery_done.set()

            threading.Thread(target=send_recovery).start()
            queries.extend(_run_queries(clients, services, events, recovery["queries"], recovery_origin,
                                        spec["max_age_ms"], proc.pid, False, len(queries)))
            recovery_done.wait()
            sustained_run._drain(ledger, queries, recovery.get("drain_s", DRAIN_S))
        status = "complete"
        (out / "ledger.json").write_text(json.dumps({"sent": ledger.sent, "receipts": ledger.receipts}, indent=2) + "\n")
        return status
    finally:
        code = _stop_process(proc)
        log.close()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        (out / "client_events.jsonl").write_text("".join(json.dumps(row) + "\n" for row in events.rows))
        (out / "health.jsonl").write_text("".join(json.dumps(row) + "\n" for row in health_rows))
        (out / "queries.json").write_text(json.dumps(queries, indent=2) + "\n")
        (out / "result.json").write_text(json.dumps({
            "status": status, "kind": args.kind, "run_id": args.run_id, "node_pid": proc.pid,
            "node_exit": code, "not_scored": True, "health_count": len(health_rows),
            "query_count": len(queries), "client_events": len(events.rows),
        }, indent=2) + "\n")
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def main(argv):
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--kind", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--node", required=True)
    parser.add_argument("--setup", required=True)
    parser.add_argument("--geometry", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--points", required=True)
    parser.add_argument("--spec", required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--domain-id", type=int, default=23)
    args = parser.parse_args(argv)
    status = session(args)
    print(args.kind, status)
    return 0 if status == "complete" else 4


if __name__ == "__main__":
    raise SystemExit(main(__import__("sys").argv[1:]))
