#!/usr/bin/env python3
"""Exclusive non-scored ST09 Gazebo rehearsal.

One session, domain 7, no GUI and no RViz, 20 minute wall budget.
Startup failures before the first trial write ``startup_failure.json`` and
do not consume a case. The driver writes rehearsal evidence only; this
process never writes a scoring suite. Bag capture stops before 1 GiB and
failed data is left in place.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src" / "luggage_gazebo"))
sys.path.insert(0, str(REPO / "scripts"))

from luggage_gazebo.rehearsal_evidence import (  # noqa: E402
    RUN_BUDGET_BYTES, TOTAL_BUDGET_BYTES, budget_allows,
)
from luggage_gazebo.startup_probe import (  # noqa: E402
    DEFAULT_STAGE_DEADLINE, STAGES, marker_order_ok, plugin_received_urdf,
    robot_spawn_started, run_stages, spawn_marker_count,
)
import sim_startup_rehearsal as startup  # noqa: E402

SESSION_BUDGET_S = 20 * 60
LAUNCH_PARAMS = (
    "gui:=false use_rviz:=false "
    "use_semantic:=true use_motion:=true use_vacuum:=true "
    "use_cargo_map:=true use_packing:=true "
    "visual_kind:=mesh size_mode:=catalog "
    "sequence_ids:=carryon,standard "
    "observe_pose_name:=pickup_observe semantic_require_backend:=bbox_fill "
    "frame_runtime:=true frame_driver_peer:=motion_occ_driver "
    "atlas_enabled:=false"
)
BAG_TOPICS = (
    "/clock", "/tf", "/tf_static", "/joint_states",
    "/monitored_planning_scene",
    "/move_group/monitored_planning_scene",
    "/move_group/display_planned_path",
    "/luggage/place/state",
    "/motion_planner/last_boundary",
    "/luggage/cargo_map/surface_2d",
    "/luggage/cargo_map/surface_v2",
    "/frame_contracts/v2/runtime_identity",
    "/luggage/scene/identity",
)


def directory_bytes(path):
    total = 0
    root = Path(path)
    if not root.exists():
        return 0
    for item in root.rglob("*"):
        if item.is_file():
            total += item.stat().st_size
    return total


def remaining(started):
    return SESSION_BUDGET_S - (time.monotonic() - started)


def write_json(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    text = json.dumps(payload, indent=2, sort_keys=True, default=str) + "\n"
    temporary.write_text(text, encoding="utf-8")
    os.replace(temporary, path)


def probe_launch(attempt, env, deadlines):
    import rclpy
    from sim_startup_rehearsal import RosStartupProbe

    rclpy.init(args=None)
    node = rclpy.create_node("st09_rehearsal_probe")
    probe = RosStartupProbe(node)

    def abort():
        if not attempt.alive():
            return "launch_died"
        return None

    checks = {
        "robot_description": probe.check_robot_description,
        "robot_spawn": lambda: (robot_spawn_started(attempt.log_text()), {}),
        "plugin_urdf": lambda: (plugin_received_urdf(attempt.log_text()), {}),
        "controller_manager": probe.check_controller_manager,
        "controllers_active": probe.check_controllers_active,
        "joint_states": probe.check_joint_states,
    }
    try:
        result = run_stages(checks, deadlines, abort=abort)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    record = result.as_dict()
    record["n_clock_publishers"] = startup.count_clock_publishers(env)
    record["marker_order_ok"] = marker_order_ok(attempt.log_text())
    record["spawn_marker_count"] = spawn_marker_count(attempt.log_text())
    record["plugin_received_urdf"] = plugin_received_urdf(attempt.log_text())
    return record


def start_bag(out, env):
    bag_dir = out / "rosbag"
    bag_dir.mkdir(parents=True, exist_ok=True)
    command = [
        "ros2", "bag", "record", "-o", str(bag_dir / "rehearsal"),
        "--max-bag-size", str(RUN_BUDGET_BYTES - 64 * 1024 * 1024),
        *BAG_TOPICS,
    ]
    log = open(out / "bag_record.log", "w", encoding="utf-8")
    proc = subprocess.Popen(
        command, stdout=log, stderr=subprocess.STDOUT, env=env,
        start_new_session=True)
    (out / "bag_record.pid").write_text("%s\n" % proc.pid)
    return proc


def stop_bag(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except OSError:
        proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        proc.kill()


def watch_budget(proc, out, stop):
    while not stop["flag"] and proc.poll() is None:
        if not budget_allows(directory_bytes(out / "rosbag"), directory_bytes(out)):
            stop["reason"] = "budget"
            stop_bag(proc)
            return
        time.sleep(2)


def run_driver(args, env, out, deadline_s):
    command = [
        "ros2", "run", "luggage_gazebo", "motion_occ_eval_driver.py",
        "--frame-runtime", "--frame-peer", "motion_occ_driver",
        "--payload", "vacuum", "--goto-timeout", "60",
        "--out", str(out),
    ]
    if args.fault:
        command.extend(["--rehearsal-fault", args.fault,
                        "--rehearsal", str(out)])
    else:
        command.extend(["--rehearsal", str(out), "--n", "2"])
    log_path = out / "driver.log"
    with log_path.open("w", encoding="utf-8") as handle:
        completed = subprocess.run(
            command, stdout=handle, stderr=subprocess.STDOUT, env=env,
            timeout=max(1.0, deadline_s), check=False)
    return completed.returncode


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--install", required=True, type=Path)
    parser.add_argument("--ros-domain-id", type=int, default=7)
    parser.add_argument("--fault", choices=("", "pre_pick", "pre_place"), default="")
    parser.add_argument("--pidfile", default="/tmp/elfin_humble_sim.pid")
    args = parser.parse_args(argv)
    if args.ros_domain_id != 7:
        parser.error("the live rehearsal is domain 7; replay uses domain 42")
    if args.out.exists() and any(args.out.iterdir()):
        parser.error("refusing to reuse a non-empty output directory")
    if not (args.install / "setup.bash").is_file():
        parser.error("install prefix has no setup.bash: %s" % args.install)
    if startup.residual_pids():
        print("sim slot busy; refusing to start a second world", file=sys.stderr)
        return 2
    args.out.mkdir(parents=True)
    started = time.monotonic()
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "7"
    env["YOLO_OFFLINE"] = "1"
    env["ULTRALYTICS_OFFLINE"] = "1"
    env["YOLO_AUTOINSTALL"] = "false"
    deadlines = dict(DEFAULT_STAGE_DEADLINE)
    attempt = None
    bag = None
    record = {"scored": False, "namespace": "rehearsal", "cases_attempted": 0,
              "stages": list(STAGES)}
    try:
        if startup.count_clock_publishers(env) != 0:
            record["reason"] = "clock_publishers_before_launch"
            write_json(args.out / "startup_failure.json", record)
            return 2
        while remaining(started) > 60:
            attempt_dir = args.out / "startup"
            attempt_dir.mkdir(exist_ok=True)
            attempt = startup.LaunchAttempt(
                attempt_dir, env, args.pidfile,
                LAUNCH_PARAMS)
            attempt.start()
            probed = probe_launch(attempt, env, deadlines)
            write_json(args.out / "startup_probe.json", probed)
            if probed.get("ok") and probed.get("n_clock_publishers") == 1:
                record["startup"] = "ready"
                break
            record["startup_failure"] = probed.get("reason")
            startup.stop_stack(REPO, env, args.out / "stop_sim.log")
            startup.wait_residuals_clear(60)
            try:
                os.unlink(args.pidfile)
            except OSError:
                pass
            attempt = None
            if remaining(started) <= 60:
                break
        else:
            record["reason"] = record.get("startup_failure") or "startup_budget"
        if attempt is None or record.get("startup") != "ready":
            record["reason"] = record.get("startup_failure") or record.get("reason")
            write_json(args.out / "startup_failure.json", record)
            return 2
        if not budget_allows(0, directory_bytes(args.out),
                             total_limit=TOTAL_BUDGET_BYTES):
            record["reason"] = "total_budget"
            write_json(args.out / "startup_failure.json", record)
            return 2
        bag = start_bag(args.out, env)
        stop = {"flag": False}
        import threading
        watcher = threading.Thread(
            target=watch_budget, args=(bag, args.out, stop), daemon=True)
        watcher.start()
        code = run_driver(args, env, args.out, remaining(started) - 30)
        stop["flag"] = True
        record["driver_exit"] = code
        record["bag_stopped_for_budget"] = stop.get("reason") == "budget"
        record["cases_attempted"] = 1 if (args.out / "trials.jsonl").exists() else 0
        evidence = args.out / (
            "rehearsal-fault.json" if args.fault else "rehearsal-evidence.json")
        record["evidence"] = str(evidence) if evidence.is_file() else ""
        record["run_bytes"] = directory_bytes(args.out)
        write_json(args.out / "rehearsal_session.json", record)
        os.sync()
        if code == 2 and record["cases_attempted"] == 0:
            return 2
        return 0 if code == 0 and evidence.is_file() else 1
    except subprocess.TimeoutExpired:
        record["reason"] = "session_budget"
        write_json(args.out / "rehearsal_session.json", record)
        return 1 if record.get("cases_attempted") else 2
    finally:
        stop_bag(bag)
        if attempt is not None:
            startup.stop_stack(REPO, env, args.out / "stop_sim.log")
            startup.wait_residuals_clear(90)
        try:
            os.unlink(args.pidfile)
        except OSError:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
