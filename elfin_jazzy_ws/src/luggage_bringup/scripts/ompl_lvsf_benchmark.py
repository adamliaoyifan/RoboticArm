#!/usr/bin/env python3
"""Benchmark OMPL solve/simplify time vs. collision-check precision for the
S20 elfin_arm across different ``longest_valid_segment_fraction`` (lvsf)
values.

The S20 uses ``lvsf: 0.002`` (see ``ompl_planning.yaml``), 5-25x denser than
MoveIt's usual 0.5% default, so a free-space plan reports a loud "state
invalid" instead of silently swinging through a box between checked
waypoints. That safety margin is not free: OMPL's path-simplification pass
gets far more collision checks to run per candidate shortcut, which is the
dominant cost on longer free-space plans (observed ~57s on some transit
moves). This script quantifies both sides of that trade-off before deciding
whether to relax it:

  - ``solve_sec`` / ``simplify_sec``, parsed from move_group's own console
    log (``ParallelPlan::solve()`` / ``Path simplification took`` lines)
  - ``waypoints``, ``success``
  - ``invalid_states``: the very thing lvsf is meant to prevent. The accepted
    trajectory is resampled at a step far finer than any tested lvsf's
    effective resolution and every interpolated point is re-checked via
    ``/check_state_validity``; a nonzero count means the coarser stride let a
    collision hide between its own check points.

    CAVEAT: this stack runs MoveIt-only with ``enable_octomap:=false`` and no
    container/box collision objects (no scene sync node, no Gazebo), so
    ``invalid_states`` here only exercises self-collision, not the real
    cargo-box misses lvsf was tightened for. A 0 here across all lvsf values
    means "these three joint-space moves never clip the robot's own links
    more coarsely at 0.01 than at 0.002" -- it does NOT mean 0.01 is safe
    around actual container geometry. Treat solve_sec/simplify_sec as the
    trustworthy half of this benchmark; invalid_states is a sanity floor,
    not proof of box-avoidance safety at relaxed lvsf.

Design (mirrors ``multi_box_gazebo_matrix.py``'s subprocess+JSONL pattern):
each lvsf gets its own MoveIt-only stack (``moveit_with_camera.launch``, no
Gazebo, so software/GPU rendering never competes with the planner for CPU),
started and torn down by this "outer" process via ``roslaunch``. The actual
planning queries run in a separate "inner" subprocess invocation of this same
script (``--inner``) so each stack gets a fresh rospy/moveit_commander client
process -- avoiding any master-reconnect fragility across restarts.

The three queries are fixed joint-space targets relative to the ``observe``
pose (``robot_poses.yaml.example``), sized to approximate the distance
profile of real phase0/phase1/transit moves (short opening-arc hop, moderate
interior reconfiguration, and the large ``observe -> pickup_observe`` swing).
They deliberately do not go through camera-pose IK: this benchmark runs
MoveIt-only with no Gazebo and no live TF (nothing publishes ``/joint_states``
without a robot in the loop), and the real pipeline's strict-down camera
orientation is calibrated from live TF -- reproducing it here would just
mean silently reimplementing (and risking drifting from) that calibration.
Plain joint-space deltas keep this script self-contained and give OMPL a
representative set of short/medium/long moves to plan and simplify.

Usage (inside the Noetic container, after ``catkin build``):
    rosrun luggage_bringup ompl_lvsf_benchmark.py \\
        --lvsf-values 0.002,0.005,0.01 --repeats 5 \\
        --output-dir /catkin_ws/src/luggage_bringup/data/ompl_lvsf_benchmark
"""
from __future__ import division

import argparse
import json
import math
import os
import re
import subprocess
import sys
import time

import rospkg
import yaml

JOINT_NAMES = [
    "elfin_joint1", "elfin_joint2", "elfin_joint3",
    "elfin_joint4", "elfin_joint5", "elfin_joint6",
]
IK_GROUP = "elfin_arm"
DEFAULT_PLANNING_TIME = 30.0  # matches robot_poses.yaml.example defaults
DEFAULT_PLANNING_ATTEMPTS = 3
# Per-joint deltas from "observe", within elfin_arm's joint limits regardless
# of the observe values themselves (J2/J3 stay well clear of their tighter
# bounds). "phase0" mimics a short lateral opening-arc hop (J1 only, like the
# real phase0's near-pure-translation views); "phase1" mimics a moderate
# interior reconfiguration (J2/J3/J5 recompose to look further/lower).
PHASE0_DELTA = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0]
PHASE1_DELTA = [0.0, -0.3, 0.3, 0.0, -0.3, 0.0]
# Interpolation step for the post-hoc safety re-check, well below any tested
# lvsf's effective per-segment resolution -- if the coarser stride hides a
# collision between its own check points, this must catch it.
SAFETY_CHECK_STEP_RAD = 0.01
READY_MARKER = "You can start planning now!"


def _default_robot_poses_path():
    return os.path.join(
        rospkg.RosPack().get_path("luggage_description"),
        "config", "robot_poses.yaml.example",
    )


def _load_yaml(path):
    with open(path, "r") as handle:
        return yaml.safe_load(handle)


def _pose_joint_values(poses_config, name):
    return [float(v) for v in poses_config["poses"][name]["values"]]


# --------------------------------------------------------------------------
# Inner process: one rospy/moveit_commander client per MoveIt-only stack.
# --------------------------------------------------------------------------

def _build_queries(robot_poses_path):
    """Return [(name, start_joints, target_joints), ...] for the fixed suite."""
    poses_config = _load_yaml(robot_poses_path)
    observe = _pose_joint_values(poses_config, "observe")
    pickup_observe = _pose_joint_values(poses_config, "pickup_observe")
    phase0_target = [o + d for o, d in zip(observe, PHASE0_DELTA)]
    phase1_target = [o + d for o, d in zip(observe, PHASE1_DELTA)]
    return [
        ("phase0_view", observe, phase0_target),
        ("phase1_entry", observe, phase1_target),
        ("transit", observe, pickup_observe),
    ]


def _interpolated_states(points, step_rad):
    """Yield joint-value lists finely resampled between consecutive waypoints."""
    for a, b in zip(points[:-1], points[1:]):
        pa = list(a.positions)
        pb = list(b.positions)
        if not pa:
            continue
        max_delta = max(abs(x - y) for x, y in zip(pa, pb))
        steps = max(1, int(math.ceil(max_delta / step_rad)))
        for i in range(1, steps):
            t = float(i) / steps
            yield [pa[j] + t * (pb[j] - pa[j]) for j in range(len(pa))]


def _count_invalid_states(validity_proxy, points):
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetStateValidityRequest
    from sensor_msgs.msg import JointState

    invalid = 0
    checked = 0
    for state in _interpolated_states(points, SAFETY_CHECK_STEP_RAD):
        req = GetStateValidityRequest()
        req.group_name = IK_GROUP
        req.robot_state = RobotState(
            joint_state=JointState(name=list(JOINT_NAMES), position=state))
        resp = validity_proxy(req)
        checked += 1
        if not resp.valid:
            invalid += 1
    return invalid, checked


def _tail_new_content(log_path, offset):
    if not os.path.isfile(log_path):
        return offset, ""
    with open(log_path, "r", errors="ignore") as handle:
        handle.seek(offset)
        content = handle.read()
        new_offset = handle.tell()
    return new_offset, content


def _parse_last(pattern, text):
    matches = re.findall(pattern, text)
    return float(matches[-1]) if matches else None


def run_inner(args):
    import moveit_commander
    import rospy
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetStateValidity
    from sensor_msgs.msg import JointState

    rospy.init_node("ompl_lvsf_benchmark_inner", anonymous=True)
    moveit_commander.roscpp_initialize([])
    group = moveit_commander.MoveGroupCommander(IK_GROUP)
    group.set_planning_time(DEFAULT_PLANNING_TIME)
    group.set_num_planning_attempts(DEFAULT_PLANNING_ATTEMPTS)

    rospy.wait_for_service("/check_state_validity", timeout=30.0)
    validity_proxy = rospy.ServiceProxy("/check_state_validity", GetStateValidity)

    queries = _build_queries(args.robot_poses)

    records = []
    log_offset = os.path.getsize(args.move_group_log) if os.path.isfile(
        args.move_group_log) else 0
    for name, start_joints, target_joints in queries:
        for trial in range(args.repeats):
            group.clear_pose_targets()
            group.set_start_state(RobotState(
                joint_state=JointState(name=list(JOINT_NAMES), position=start_joints)))
            group.set_joint_value_target(list(target_joints))
            t0 = time.time()
            success, plan, _planning_time, error_code = group.plan()
            wall_sec = time.time() - t0
            time.sleep(0.3)  # let move_group's log line land before we tail it
            log_offset, new_log = _tail_new_content(
                args.move_group_log, log_offset)
            solve_sec = _parse_last(
                r"Solution found by one or more threads in ([0-9.]+) seconds",
                new_log)
            simplify_sec = _parse_last(
                r"Path simplification took ([0-9.]+) seconds", new_log)
            points = list(plan.joint_trajectory.points) if success else []
            invalid_states, checked_states = (0, 0)
            if success and len(points) >= 2:
                invalid_states, checked_states = _count_invalid_states(
                    validity_proxy, points)
            records.append({
                "query": name,
                "trial": trial,
                "success": bool(success),
                "error_code": int(getattr(error_code, "val", 0)),
                "wall_sec": round(wall_sec, 4),
                "solve_sec": solve_sec,
                "simplify_sec": simplify_sec,
                "waypoints": len(points),
                "invalid_states": invalid_states,
                "checked_states": checked_states,
            })
    group.set_start_state_to_current_state()
    moveit_commander.roscpp_shutdown()

    with open(args.json_out, "w") as handle:
        json.dump(records, handle, indent=2, sort_keys=True)


# --------------------------------------------------------------------------
# Outer process: owns the roslaunch lifecycle per lvsf value.
# --------------------------------------------------------------------------

def _cleanup_ros():
    for process in ("roslaunch", "move_group", "robot_state_publisher", "rosmaster"):
        subprocess.run(
            ["pkill", "-TERM", process],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    time.sleep(3.0)


def _wait_for_ready(log_path, timeout_sec=60.0):
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if os.path.isfile(log_path):
            with open(log_path, "r", errors="ignore") as handle:
                if READY_MARKER in handle.read():
                    return True
        time.sleep(1.0)
    return False


def _run_one_lvsf(lvsf, args):
    _cleanup_ros()
    log_path = os.path.join(args.output_dir, "move_group_lvsf_%s.log" % lvsf)
    json_out = os.path.join(args.output_dir, "records_lvsf_%s.json" % lvsf)
    # move_group (a C++ node, output="screen") inherits roslaunch's redirected
    # stdout fd directly; since that fd is a regular file rather than a tty,
    # glibc's stdio switches from line- to full-buffering (~4KB), so console
    # lines can sit unflushed for seconds. That breaks the offset-based log
    # tailing below (a solve/simplify line can land several trials late, or
    # not at all before the run ends). `stdbuf -oL` forces line buffering via
    # LD_PRELOAD, which is inherited across roslaunch's fork/exec of
    # move_group, so it fixes both processes at once.
    with open(log_path, "w") as log_stream:
        launch_proc = subprocess.Popen(
            [
                "stdbuf", "-oL", "-eL",
                "roslaunch", "luggage_bringup", "moveit_with_camera.launch",
                "load_urdf:=true", "enable_octomap:=false",
                "lvsf:=%s" % lvsf,
            ],
            stdout=log_stream, stderr=subprocess.STDOUT,
        )
    ready = _wait_for_ready(log_path, timeout_sec=args.ready_timeout_sec)
    if not ready:
        launch_proc.terminate()
        _cleanup_ros()
        return {"lvsf": lvsf, "error": "move_group did not become ready in time"}

    inner_cmd = [
        sys.executable, os.path.abspath(__file__), "--inner",
        "--repeats", str(args.repeats),
        "--robot-poses", args.robot_poses,
        "--move-group-log", log_path,
        "--json-out", json_out,
    ]
    try:
        subprocess.run(inner_cmd, timeout=args.inner_timeout_sec, check=True)
        error = None
    except subprocess.CalledProcessError as exc:
        error = "inner process failed: %s" % exc
    except subprocess.TimeoutExpired:
        error = "inner process timed out"
    finally:
        launch_proc.terminate()
        _cleanup_ros()

    if error is not None:
        return {"lvsf": lvsf, "error": error}
    with open(json_out, "r") as handle:
        records = json.load(handle)
    return {"lvsf": lvsf, "records": records}


def _percentile(values, pct):
    if not values:
        return None
    ordered = sorted(values)
    idx = min(len(ordered) - 1, int(math.ceil(pct / 100.0 * len(ordered))) - 1)
    return ordered[max(0, idx)]


def _summarize(lvsf_result):
    records = lvsf_result.get("records", [])
    solve = [r["solve_sec"] for r in records if r.get("solve_sec") is not None]
    simplify = [
        r["simplify_sec"] for r in records if r.get("simplify_sec") is not None]
    successes = [r for r in records if r["success"]]
    return {
        "lvsf": lvsf_result["lvsf"],
        "error": lvsf_result.get("error"),
        "run_count": len(records),
        "success_rate": (
            len(successes) / len(records) if records else 0.0),
        "solve_sec_median": _percentile(solve, 50),
        "solve_sec_p95": _percentile(solve, 95),
        "simplify_sec_median": _percentile(simplify, 50),
        "simplify_sec_p95": _percentile(simplify, 95),
        "total_invalid_states": sum(r["invalid_states"] for r in records),
        "total_checked_states": sum(r["checked_states"] for r in records),
    }


def run_outer(args):
    os.makedirs(args.output_dir, exist_ok=True)
    lvsf_values = [v.strip() for v in args.lvsf_values.split(",") if v.strip()]
    results = []
    for lvsf in lvsf_values:
        result = _run_one_lvsf(lvsf, args)
        results.append(result)
        print(json.dumps(result if "error" in result and result["error"]
                          else _summarize(result), sort_keys=True), flush=True)

    summary = {
        "lvsf_values": lvsf_values,
        "repeats": args.repeats,
        "results": [_summarize(r) for r in results],
    }
    summary_path = os.path.join(args.output_dir, "summary.json")
    with open(summary_path, "w") as handle:
        json.dump(summary, handle, indent=2, sort_keys=True)
        handle.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inner", action="store_true",
                        help=argparse.SUPPRESS)  # internal re-invocation
    parser.add_argument("--lvsf-values", default="0.002,0.005,0.01")
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument(
        "--robot-poses", default=_default_robot_poses_path())
    parser.add_argument(
        "--output-dir",
        default=os.path.join(
            rospkg.RosPack().get_path("luggage_bringup"),
            "data", "ompl_lvsf_benchmark"))
    parser.add_argument("--ready-timeout-sec", type=float, default=60.0)
    parser.add_argument("--inner-timeout-sec", type=float, default=600.0)
    # Internal-only args, forwarded by the outer process to --inner.
    parser.add_argument("--move-group-log", default=None)
    parser.add_argument("--json-out", default=None)
    args = parser.parse_args()

    if args.inner:
        run_inner(args)
    else:
        run_outer(args)


if __name__ == "__main__":
    main()
