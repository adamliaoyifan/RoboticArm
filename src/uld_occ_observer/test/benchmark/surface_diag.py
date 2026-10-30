"""Not-scored Surface timing. Client wall time is not split into product stages.

O6's in-process surface() and O7's ROS ReadSurface are different operations.
Unobserved stages stay unknown. The residual is not named as integrate cost.
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import time
from pathlib import Path

import sustained_run

UNKNOWN = "unknown"
SURFACE_PATHS = """# Surface call paths

O6 `observer_profile_cli` in Off mode calls `Snapshot::surface(nullptr)` inside the
benchmark process, twice, on the snapshot just acquired. `external_ms` is only the
steady clock around that call. The second call sees the in-process surface cache
when the snapshot object is unchanged. There is no ROS service and no copy into a
response message. Off mode passes a null profile, so build, lock, and cache-hit
fields are not recorded.

O7 `ReadSurface` is a different operation. The client `surface_ms` is the wall time
from sending the service request to receiving the response. The server loads the
handle, calls `surface()` with no profile, then copies every returned column into
the response arrays. Scheduling, executor delay, the surface-cache lock, a build or
a cache hit, the column copy, and middleware serialization all sit inside that one
client interval. The response has no fields for those stages. Historical O7 ledgers
therefore cannot assign the time. The O6-to-O7 difference is not a concurrency
regression.

This diagnostic uses the ROS ReadSurface path for both arms. D0 does not publish
during the queries. D1 publishes the same cloud at 2 Hz during the queries. Both
use acquire, ROI, surface, release, with `max_age_ms` 120000 so an unchanged
snapshot stays readable for the 33 s window. That wider age is not the historical
O7 2000 ms request. Samples are `not_scored`. First read, same-revision repeat, and
revision-changed read stay in separate lists.
"""


def stage_record(surface_ms):
    """Client wall time around ReadSurface. Internal stages were not instrumented."""
    return {
        "surface_client_wall_ms": surface_ms,
        "lock_wait_ms": UNKNOWN,
        "surface_build_ms": UNKNOWN,
        "copy_convert_ms": UNKNOWN,
        "serialization_ms": UNKNOWN,
        "unknown_note": "client wall is not divided; the undivided part is not an integrate cost",
    }


def classify_reads(samples):
    """Keep first, same-revision, and changed-revision reads apart."""
    classes = {"first": [], "same_revision": [], "revision_changed": []}
    previous = None
    for sample in samples:
        revision = sample.get("map_revision")
        if previous is None:
            kind = "first"
        elif revision == previous:
            kind = "same_revision"
        else:
            kind = "revision_changed"
        previous = revision
        classes[kind].append(sample)
    return classes


def _proc_cpu_ms(pid):
    text = Path("/proc/%d/stat" % pid).read_text()
    fields = text[text.rfind(")") + 2:].split()
    ticks = os.sysconf("SC_CLK_TCK")
    return 1000.0 * (int(fields[11]) + int(fields[12])) / ticks


def _timed_surface(clients, services, pid):
    import sustained_run as harness
    started = time.monotonic()
    scheduled = started
    acquire_req = services["get"].Request()
    acquire_req.expected_epoch = 0
    acquire_req.exact_revision = False
    acquire_req.expected_revision = 0
    acquire_req.max_age_ms = 120000
    acquire_req.allow_stale = False
    send = time.monotonic()
    acquired = harness._call(clients["acquire"], acquire_req, 5.0)
    if acquired is None or acquired.status != "OK":
        return {"status": None if acquired is None else acquired.status, "scheduled": scheduled, "send": send,
                "result": time.monotonic(), **stage_record(None)}
    handle = acquired.handle
    identity = harness._identity(acquired.identity)
    try:
        roi_req = services["roi"].Request()
        roi_req.handle = handle
        roi_req.begin = [0, 0, 0]
        roi_req.end = [16, 16, 16]
        roi_req.cursor = 0
        roi_req.limit = 4096
        roi = harness._call(clients["roi"], roi_req, 5.0)
        cpu_before = _proc_cpu_ms(pid)
        thread_before = time.thread_time()
        surface_req = services["surface"].Request()
        surface_req.handle = handle
        surface_req.cursor = 0
        surface_req.limit = 65536
        surface_send = time.monotonic()
        surface = harness._call(clients["surface"], surface_req, 5.0)
        surface_done = time.monotonic()
        cpu_after = _proc_cpu_ms(pid)
        record = stage_record((surface_done - surface_send) * 1000.0)
        record.update({
            "status": None if surface is None else surface.status,
            "scheduled": scheduled,
            "send": surface_send,
            "result": surface_done,
            "age_ns": int(acquired.age_ns),
            "stale": bool(acquired.stale),
            "columns": None if surface is None else len(surface.x),
            "done": None if surface is None else bool(surface.done),
            "map_revision": identity[1],
            "identity": list(identity),
            "cache_state": UNKNOWN,
            "observer_cpu_ms_during_client_wall": cpu_after - cpu_before,
            "client_thread_cpu_ms": (time.thread_time() - thread_before) * 1000.0,
            "roi_status": None if roi is None else roi.status,
        })
        return record
    finally:
        release_req = services["release"].Request()
        release_req.handle = handle
        harness._call(clients["release"], release_req, 5.0)


def run_arm(node, setup, geometry, config, points, mode, out, domain_id):
    """One observer process. mode is D0 (no writes during queries) or D1 (2 Hz writes)."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    env = sustained_run._ros_env(setup, domain_id)
    log = out / "observer.log"
    handle = log.open("w")
    proc = subprocess.Popen(
        [node, "--ros-args", "-p", "geometry_file:=" + geometry, "-p", "config_file:=" + config,
         "-p", "input_mode:=subscription", "-p", "optical_frame:=" + sustained_run.OPTICAL_FRAME,
         "-p", "cloud_topic:=optical_points", "-p", "sensor_id:=cam0"],
        stdout=handle, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    handle.close()
    deadline = time.monotonic() + 600.0
    try:
        return _arm_session(points, mode, out, env, proc.pid, deadline)
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


def _arm_session(points, mode, out, env, pid, deadline):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import String
    from geometry_msgs.msg import TransformStamped
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from uld_occ_observer.msg import UpdateReceipt
    from uld_occ_observer.srv import GetSnapshot, ReadCells, ReadSurface, ReleaseSnapshot

    os.environ.update({key: env[key] for key in ("ROS_DOMAIN_ID",) if key in env})
    rclpy.init()
    node = Node("surface_diag")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    import threading
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    ledger = sustained_run.Ledger()
    health_times = []
    latest = {}
    phase_box = {"name": "warmup"}
    observed = {"queue": False}
    try:
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
        node.create_subscription(UpdateReceipt, "/uld_occ_observer/receipts",
                                 lambda msg: sustained_run._on_receipt(ledger, observed, phase_box, msg), 100)
        node.create_subscription(String, "/uld_occ_observer/health",
                                 lambda msg: sustained_run._on_health(health_times, latest, observed, phase_box, msg, __import__("yaml")), 10)
        clients = {
            "acquire": node.create_client(GetSnapshot, "/uld_occ_observer/get_snapshot"),
            "roi": node.create_client(ReadCells, "/uld_occ_observer/read_cells"),
            "surface": node.create_client(ReadSurface, "/uld_occ_observer/read_surface"),
            "release": node.create_client(ReleaseSnapshot, "/uld_occ_observer/release_snapshot"),
        }
        services = {"get": GetSnapshot, "roi": ReadCells, "surface": ReadSurface, "release": ReleaseSnapshot}
        if not sustained_run._wait_ready(clients, health_times, 30.0):
            result = {"status": "startup_failed", "product_scored": False, "mode": mode}
            (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            return result
        payload = sustained_run._pack_xyz(points, PointCloud2, PointField)
        sustained_run._publish_phase(node, publisher, payload, [{"planned": index * 0.5} for index in range(3)],
                                     ledger, "warmup", PointCloud2)
        if not sustained_run._wait_queue_clear(ledger, latest, 30.0):
            result = {"status": "startup_failed", "reason": "warmup queue did not clear", "product_scored": False, "mode": mode}
            (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            return result
        if time.monotonic() > deadline:
            raise TimeoutError("arm budget")
        stop_writes = threading.Event()
        if mode == "D1":
            phase_box["name"] = "writes"

            def write():
                origin = time.monotonic()
                items = [{"planned": index * 0.5} for index in range(66)]
                sustained_run._publish_phase(node, publisher, payload, items, ledger, "writes", PointCloud2, origin)
                stop_writes.set()

            threading.Thread(target=write, daemon=True).start()
        else:
            stop_writes.set()
        samples = []
        origin = time.monotonic()
        for index in range(33):
            if time.monotonic() > deadline:
                raise TimeoutError("query budget")
            target = origin + index * 1.0
            delay = target - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            sample = _timed_surface(clients, services, pid)
            sample["index"] = index
            sample["role"] = "warmup" if index < 3 else "scored"
            sample["mode"] = mode
            samples.append(sample)
        stop_writes.wait(timeout=5)
        result = {
            "status": "complete",
            "product_scored": False,
            "not_scored": True,
            "mode": mode,
            "max_age_ms": 120000,
            "samples": samples,
            "reads_by_revision": {
                key: [{"index": item["index"], "role": item["role"]} for item in value]
                for key, value in classify_reads(samples).items()},
            "integrate_receipts": ledger.summary("writes") if mode == "D1" else ledger.summary("warmup"),
        }
        (out / "result.json").write_text(json.dumps(result) + "\n")
        return result
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv):
    import argparse
    import fcntl
    parser = argparse.ArgumentParser(description="Not-scored D0/D1 surface diagnostic")
    parser.add_argument("--out", required=True)
    parser.add_argument("--node", required=True)
    parser.add_argument("--setup", required=True)
    parser.add_argument("--fixtures", required=True)
    parser.add_argument("--points-b00", required=True)
    parser.add_argument("--points-b20", required=True)
    parser.add_argument("--domain-id", type=int, default=19)
    args = parser.parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "SURFACE_PATHS.md").write_text(SURFACE_PATHS)
    lock_path = Path("/tmp/elfin-occ-backend-perf.lock")
    lock = open(lock_path, "a")
    wait_started = time.monotonic()
    fcntl.flock(lock, fcntl.LOCK_EX)
    lock_wait = time.monotonic() - wait_started
    started = time.monotonic()
    budget = 90 * 60
    order = [("b00", "D0", args.points_b00, 0), ("b00", "D1", args.points_b00, 0),
             ("b20", "D1", args.points_b20, 20), ("b20", "D0", args.points_b20, 20)]
    runs = []
    try:
        for name, mode, points_path, boxes in order:
            if time.monotonic() - started > budget:
                runs.append({"name": name, "mode": mode, "status": "not_run", "reason": "90 minute budget"})
                continue
            scene = sustained_run._scene(args.fixtures, boxes)
            points = json.loads(Path(points_path).read_text())
            folder = out / ("%s_%s" % (name, mode))
            outcome = run_arm(args.node, args.setup, scene["geometry_file"], scene["config_file"],
                              points, mode, folder, args.domain_id)
            runs.append({"name": name, "mode": mode, "status": outcome.get("status"), "not_scored": True})
    finally:
        fcntl.flock(lock, fcntl.LOCK_UN)
        lock.close()
    (out / "DIAG_INDEX.json").write_text(json.dumps({
        "product_scored": False,
        "lock_wait_s": lock_wait,
        "elapsed_s": time.monotonic() - started,
        "runs": runs,
    }, indent=2) + "\n")
    print("surface_diag", json.dumps(runs))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(__import__("sys").argv[1:]))
