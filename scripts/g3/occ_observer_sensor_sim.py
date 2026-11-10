#!/usr/bin/env python3
"""Sensor-only Gazebo S1-S3 driver. Owns PIDs; never pkill ign gazebo."""
from __future__ import annotations

import fcntl
import hashlib
import json
import math
import os
import signal
import struct
import subprocess
import sys
import time
from pathlib import Path

PASS, FAIL, NOT_EVAL = 0, 3, 4
ROOT = Path(__file__).resolve().parents[2]
LOCK = Path("/tmp/elfin-occ-backend-perf.lock")
PIDFILE = Path("/tmp/elfin_occ_observer_sim.pid")
OPTICAL = "d555_color_optical_frame"
POSE_A = (-1.15, 0.0, 1.20)
POSE_B = (-1.15, 0.45, 1.20)
OCCLUDE = (-0.35, 0.0, 1.15)
R_ULD = (0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0)


def _die(code: int, msg: str) -> int:
    print(msg, file=sys.stderr)
    return code


def rotation_to_quat(m):
    r00, r01, r02, r10, r11, r12, r20, r21, r22 = m
    trace = r00 + r11 + r22
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        return ((r21 - r12) * s, (r02 - r20) * s, (r10 - r01) * s, 0.25 / s)
    if r00 > r11 and r00 > r22:
        s = 2.0 * math.sqrt(1.0 + r00 - r11 - r22)
        return (0.25 * s, (r01 + r10) / s, (r02 + r20) / s, (r21 - r12) / s)
    if r11 > r22:
        s = 2.0 * math.sqrt(1.0 + r11 - r00 - r22)
        return ((r01 + r10) / s, 0.25 * s, (r12 + r21) / s, (r02 - r20) / s)
    s = 2.0 * math.sqrt(1.0 + r22 - r00 - r11)
    return ((r02 + r20) / s, (r12 + r21) / s, 0.25 * s, (r10 - r01) / s)


def residual_sim() -> list[str]:
    r = subprocess.run(
        ["pgrep", "-af", "ros2 launch luggage_gazebo|ign gazebo|gz sim|clock_bridge|camera_bridge|observer_clock_bridge"],
        capture_output=True,
        text=True,
        check=False,
    )
    lines = [ln for ln in (r.stdout or "").splitlines() if ln.strip() and "pgrep" not in ln]
    return lines


def acquire_lock():
    LOCK.touch()
    handle = open(LOCK, "a", encoding="utf-8")
    deadline = time.time() + 1800
    while True:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            handle.write("occ_octomap_cursor_g1 %s\n" % os.getpid())
            handle.flush()
            return handle
        except BlockingIOError:
            if time.time() > deadline:
                handle.close()
                raise TimeoutError("perf lock busy")
            time.sleep(5)


def depth_to_xyz(img, info):
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    step = img.step
    pts = []
    for v in range(img.height):
        row = v * step
        for u in range(img.width):
            z = struct.unpack_from("<f", img.data, row + u * 4)[0]
            if not math.isfinite(z) or z <= 0.0:
                pts.append((float("nan"), float("nan"), float("nan")))
                continue
            pts.append(((u - cx) * z / fx, (v - cy) * z / fy, z))
    return pts


def pack_cloud(pts, width, height, stamp, frame):
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header

    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id=frame)
    msg.height = height
    msg.width = width
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * width
    buf = bytearray(msg.row_step * height)
    for i, (x, y, z) in enumerate(pts):
        struct.pack_into("<fff", buf, i * 12, float(x), float(y), float(z))
    msg.data = bytes(buf)
    msg.is_dense = False
    return msg


def set_model_pose(name, xyz):
    req = "name: \"%s\" position: {x: %s, y: %s, z: %s}" % (name, xyz[0], xyz[1], xyz[2])
    return subprocess.run(
        [
            "ign",
            "service",
            "-s",
            "/world/uld_occ_observer_sensor_only/set_pose",
            "--reqtype",
            "ignition.msgs.Pose",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            "2000",
            "--req",
            req,
        ],
        capture_output=True,
        text=True,
        check=False,
    )


def start_stack(geometry: Path, config: Path, world: Path, log_dir: Path, domain: str):
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = domain
    env["RCUTILS_LOGGING_BUFFERED_STREAM"] = "1"
    setup = (
        "set +u; source /opt/ros/humble/setup.bash; "
        "source /tmp/elfin-occ-octomap-cursor-g1/install_observer/setup.bash; "
        "exec \"$@\""
    )
    log_dir.mkdir(parents=True, exist_ok=True)
    gz_log = open(log_dir / "ign.log", "w", encoding="utf-8")
    launch_log = open(log_dir / "launch.log", "w", encoding="utf-8")
    gz = subprocess.Popen(
        ["ign", "gazebo", "-s", "-r", "--headless-rendering", str(world)],
        stdout=gz_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        env=env,
    )
    time.sleep(3)
    launch = subprocess.Popen(
        [
            "bash",
            "-c",
            setup,
            "_",
            "ros2",
            "launch",
            "uld_occ_observer",
            "sensor_sim.launch.py",
            "geometry_file:=%s" % geometry,
            "config_file:=%s" % config,
        ],
        stdout=launch_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        env=env,
        cwd=str(ROOT),
    )
    PIDFILE.write_text("%s\n%s\n" % (gz.pid, launch.pid))
    return {"gz": gz, "launch": launch, "logs": (gz_log, launch_log), "env": env}


def stop_stack(procs: dict) -> None:
    for key in ("launch", "gz"):
        proc = procs.get(key)
        if proc is None or proc.poll() is not None:
            continue
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except OSError:
            proc.terminate()
        try:
            proc.wait(timeout=12)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except OSError:
                proc.kill()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except OSError:
                    proc.kill()
    for log in procs.get("logs") or ():
        try:
            log.close()
        except Exception:
            pass
    if PIDFILE.exists():
        PIDFILE.unlink()


def run_episode(args, case: str, episode: int, out: Path, geometry: Path, config: Path, world: Path) -> dict:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster
    from uld_occ_observer.msg import UpdateReceipt

    ep_dir = out / case / ("episode_%02d" % episode)
    ep_dir.mkdir(parents=True, exist_ok=True)
    domain = str(171 + episode)
    procs = start_stack(geometry, config, world, ep_dir, domain)
    os.environ["ROS_DOMAIN_ID"] = domain
    rclpy.init()
    from rclpy.parameter import Parameter

    node = Node(
        "occ_observer_sensor_adapter",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
    )
    qos = QoSProfile(depth=4, reliability=ReliabilityPolicy.RELIABLE)
    sensor_qos = qos_profile_sensor_data
    pub = node.create_publisher(PointCloud2, "/uld_occ_observer/optical_points", qos)
    br = TransformBroadcaster(node)
    state = {
        "info": None,
        "accepted": 0,
        "dropped": 0,
        "last_bin": None,
        "pose": POSE_A,
        "publish": True,
        "receipts": [],
        "health": [],
        "hits": set(),
        "frees": set(),
        "t0": time.monotonic(),
    }

    def tf_for(stamp, pose):
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "container_link"
        msg.child_frame_id = OPTICAL
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = pose
        q = rotation_to_quat(R_ULD)
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = q
        return msg

    def on_info(msg):
        state["info"] = msg

    def on_depth(msg):
        info = state["info"]
        if info is None or not state["publish"]:
            return
        elapsed = time.monotonic() - state["t0"]
        if elapsed < 30:
            return
        bin_id = int(msg.header.stamp.sec * 2 + msg.header.stamp.nanosec * 2e-9)
        if state["last_bin"] == bin_id:
            state["dropped"] += 1
            return
        state["last_bin"] = bin_id
        try:
            pts = depth_to_xyz(msg, info)
        except Exception:
            state["dropped"] += 1
            return
        cloud = pack_cloud(pts, msg.width, msg.height, msg.header.stamp, OPTICAL)
        br.sendTransform(tf_for(msg.header.stamp, state["pose"]))
        pub.publish(cloud)
        state["accepted"] += 1

    def on_receipt(msg):
        rec = {"status": msg.status, "stamp": int(msg.input_stamp_ns), "hit": int(msg.hit_keys), "free": int(msg.free_keys)}
        state["receipts"].append(rec)
        if msg.status == "INTEGRATED":
            state["hits"].add(msg.hit_keys)
            state["frees"].add(msg.free_keys)

    def on_health(msg):
        state["health"].append(msg.data[:400])

    node.create_subscription(CameraInfo, "/camera_info", on_info, sensor_qos)
    node.create_subscription(Image, "/observer_depth", on_depth, sensor_qos)
    node.create_subscription(UpdateReceipt, "/uld_occ_observer/receipts", on_receipt, qos)
    node.create_subscription(String, "/uld_occ_observer/health", on_health, qos)

    ready_deadline = time.monotonic() + 30
    ready = False
    while time.monotonic() < ready_deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if state["info"] is not None:
            ready = True
            break
    result = {
        "case": case,
        "episode": episode,
        "ready": ready,
        "pass": False,
        "reason": "not_scored",
    }
    if not ready:
        result["reason"] = "ready_timeout"
        (ep_dir / "episode.json").write_text(json.dumps(result, indent=2) + "\n")
        node.destroy_node()
        rclpy.shutdown()
        stop_stack(procs)
        return result

    end = time.monotonic() + 120
    phase = "ready"
    while time.monotonic() < end:
        elapsed = time.monotonic() - state["t0"]
        if case == "static":
            state["pose"] = POSE_A
            state["publish"] = 30 <= elapsed < 90
        elif case == "moving-camera":
            if elapsed < 30:
                state["publish"] = False
            elif elapsed < 60:
                state["pose"] = POSE_A
                state["publish"] = True
            elif elapsed < 90:
                if phase != "b":
                    set_model_pose("depth_cam", POSE_B)
                    phase = "b"
                state["pose"] = POSE_B
                state["publish"] = True
            else:
                state["publish"] = False
        elif case == "occluded":
            if elapsed < 30:
                state["publish"] = False
            elif elapsed < 60:
                state["pose"] = POSE_A
                state["publish"] = True
            elif elapsed < 70:
                state["publish"] = False
            elif elapsed < 100:
                if phase != "occ":
                    set_model_pose("occluder", OCCLUDE)
                    phase = "occ"
                state["publish"] = True
            else:
                state["publish"] = False
        rclpy.spin_once(node, timeout_sec=0.05)

    integrated = [r for r in state["receipts"] if r["status"] == "INTEGRATED"]
    rejected = [r for r in state["receipts"] if r["status"] not in ("INTEGRATED", "ALREADY_PROCESSED", "NO_VALID_POINTS")]
    hit_sum = sum(r["hit"] for r in integrated)
    free_sum = sum(r["free"] for r in integrated)
    qualified = hit_sum >= 100 and free_sum >= 100
    wanted = 120 if case == "static" else 60
    # static 30-90s at 2Hz => 120; moving two 30s windows => 60+60
    if case == "static":
        ok_count = state["accepted"] >= 120 and len(integrated) >= 120
    elif case == "moving-camera":
        ok_count = state["accepted"] >= 120 and len(integrated) >= 100
    else:
        ok_count = state["accepted"] >= 100
    result.update(
        {
            "ready_s": 30 if ready else None,
            "accepted": state["accepted"],
            "selection_drops": state["dropped"],
            "integrated": len(integrated),
            "rejected": len(rejected),
            "hit_keys_sum": hit_sum,
            "free_keys_sum": free_sum,
            "qualified": qualified,
            "health_samples": len(state["health"]),
            "wanted_frames": wanted,
        }
    )
    if not qualified:
        result["reason"] = "visibility_not_qualified"
        result["pass"] = False
        result["outcome"] = "not_evaluated" if hit_sum == 0 and free_sum == 0 else "fail"
    elif rejected:
        result["reason"] = "observer_reject:" + rejected[0]["status"]
        result["pass"] = False
    elif not ok_count:
        result["reason"] = "frame_count"
        result["pass"] = False
    else:
        result["reason"] = "ok"
        result["pass"] = True
    # Drain: pin a snapshot and count distinct known occupied/free cells in a bounded ROI.
    try:
        from uld_occ_observer.srv import GetSnapshot, ReadCells, ReleaseSnapshot

        now = node.get_clock().now()
        get_cli = node.create_client(GetSnapshot, "/uld_occ_observer/get_snapshot")
        cells_cli = node.create_client(ReadCells, "/uld_occ_observer/read_cells")
        rel_cli = node.create_client(ReleaseSnapshot, "/uld_occ_observer/release_snapshot")
        if get_cli.wait_for_service(timeout_sec=5.0):
            req = GetSnapshot.Request()
            req.max_age_ms = 60000
            req.allow_stale = True
            req.expected_epoch = 0
            req.exact_revision = False
            future = get_cli.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            snap = future.result()
            result["snapshot_status"] = snap.status if snap else "no_result"
            occ = 0
            free = 0
            unknown = 0
            if snap and snap.status == "OK":
                cells_req = ReadCells.Request()
                cells_req.handle = snap.handle
                cells_req.begin = [0, 0, 0]
                cells_req.end = [30, 40, 30]
                cells_req.cursor = 0
                cells_req.limit = 65536
                while True:
                    cf = cells_cli.call_async(cells_req)
                    rclpy.spin_until_future_complete(node, cf, timeout_sec=5.0)
                    page = cf.result()
                    if page is None or page.status != "OK":
                        result["cells_status"] = None if page is None else page.status
                        break
                    for st in page.states:
                        if st == 2:
                            occ += 1
                        elif st == 1:
                            free += 1
                        elif st == 0:
                            unknown += 1
                    if page.done:
                        break
                    cells_req.cursor = page.next_cursor
                rel = ReleaseSnapshot.Request()
                rel.handle = snap.handle
                rf = rel_cli.call_async(rel)
                rclpy.spin_until_future_complete(node, rf, timeout_sec=2.0)
            result["distinct_occupied"] = occ
            result["distinct_free"] = free
            result["distinct_unknown"] = unknown
            qualified = occ >= 100 and free >= 100
            result["qualified"] = qualified
            if result.get("reason") == "ok" and not qualified:
                result["reason"] = "visibility_not_qualified"
                result["pass"] = False
                if occ == 0 and free == 0:
                    result["outcome"] = "not_evaluated"
    except Exception as exc:  # noqa: BLE001
        result["snapshot_error"] = str(exc)
    (ep_dir / "episode.json").write_text(json.dumps(result, indent=2) + "\n")
    (ep_dir / "receipts.json").write_text(json.dumps(state["receipts"][-200:], indent=2) + "\n")
    node.destroy_node()
    rclpy.shutdown()
    stop_stack(procs)
    leftover = residual_sim()
    result["leftover"] = leftover[:8]
    (ep_dir / "teardown.json").write_text(json.dumps({"leftover": leftover[:12]}, indent=2) + "\n")
    return result


def main(argv=None) -> int:
    import argparse

    p = argparse.ArgumentParser()
    p.add_argument("--install", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--fixtures", required=True)
    p.add_argument("--cases", default="static,moving-camera,occluded")
    p.add_argument("--repeat", type=int, default=3)
    p.add_argument("--duration-wall", type=int, default=120)
    p.add_argument("--rehearsal", action="store_true")
    args = p.parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    live = residual_sim()
    if live:
        (out / "blocked.json").write_text(json.dumps({"live": live}, indent=2))
        return _die(NOT_EVAL, "sim slot busy: %s" % live[:3])
    world = ROOT / "src/uld_occ_observer/worlds/sensor_only.sdf"
    fixtures = Path(args.fixtures)
    man = json.loads((fixtures / "TEST_MANIFEST.json").read_text())
    scene = next(s for s in man["scenes"] if s["resolution"] == 0.05 and s["hull"] == "chamfer" and s["boxes"] == 5)
    geometry = Path(scene["geometry_file"])
    config = Path(scene["config_file"])
    world_hash = hashlib.sha256(world.read_bytes()).hexdigest()
    (out / "SIM_MANIFEST.json").write_text(
        json.dumps(
            {
                "world": str(world),
                "world_sha256": world_hash,
                "optical_frame": OPTICAL,
                "pose_a": POSE_A,
                "pose_b": POSE_B,
                "rate_hz": 2,
                "image": [160, 120],
                "geometry": str(geometry),
                "config": str(config),
            },
            indent=2,
        )
        + "\n"
    )
    try:
        lock = acquire_lock()
    except TimeoutError as exc:
        return _die(NOT_EVAL, str(exc))
    cases = [c for c in args.cases.split(",") if c]
    if args.rehearsal:
        cases = cases[:1]
        args.repeat = 1
    rows = []
    overall = True
    uneval = False
    try:
        for case in cases:
            for ep in range(args.repeat):
                row = run_episode(args, case, ep, out, geometry, config, world)
                rows.append(row)
                if row.get("outcome") == "not_evaluated":
                    uneval = True
                    overall = False
                overall = overall and bool(row.get("pass"))
                live = residual_sim()
                if live:
                    row["reason"] = "residual_after_teardown"
                    overall = False
                    (out / "residual.json").write_text(json.dumps(live, indent=2))
    finally:
        fcntl.flock(lock, fcntl.LOCK_UN)
        lock.close()
    (out / "SIMULATION.md").write_text("# Simulation\n\n%s\n" % json.dumps(rows, indent=2)[:20000])
    (out / "summary.json").write_text(json.dumps({"pass": overall, "not_evaluated": uneval, "rows": rows}, indent=2))
    if uneval and not any(r.get("pass") for r in rows):
        return NOT_EVAL
    return PASS if overall else FAIL


if __name__ == "__main__":
    sys.exit(main())
