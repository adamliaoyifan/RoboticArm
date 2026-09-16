#!/usr/bin/env python3
"""Non-scored sim_world startup rehearsal: launch, stage, teardown, repeat.

Purpose is diagnosis, not acceptance. It never spawns a box, never scores a
case, and never writes a campaign verdict. It answers one question that
PF-R7 generations 7, 8 and 9 each burned a scored live slot without ever
answering: how often does the ``gz_ros2_control`` URDF fetch actually win its
race, and where does the startup chain stop when it loses?

Every stage is observed by one persistent rclpy node under one clock (see
``luggage_gazebo.startup_probe``). Each iteration tears the stack down with
``scripts/stop_sim.sh`` and refuses to start the next one until residuals are
zero, per ``.cursor/rules/sim-lifecycle.mdc``.

Usage::

    scripts/sim_startup_rehearsal.sh --iterations 10 --out <dir>
"""
from __future__ import annotations

import argparse
import json
import os
import re
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src" / "luggage_gazebo"))

from luggage_gazebo.startup_probe import (  # noqa: E402
    DEFAULT_STAGE_DEADLINE,
    EXPECTED_JOINT_COUNT,
    REQUIRED_CONTROLLERS,
    STAGES,
    ProbeResult,
    marker_order_ok,
    plugin_received_urdf,
    robot_spawn_started,
    run_stages,
    spawn_marker_count,
)

# Same graph shape the PF-R7 campaign launches, so the rehearsal races the same
# node count. Kept in sync with scripts/pf_r7_live_backend.py LAUNCH_PARAMS.
LAUNCH_PARAMS = (
    "gui:=false use_rviz:=false use_semantic:=true use_motion:=true "
    "use_vacuum:=true use_cargo_map:=false use_packing:=false "
    "visual_kind:=mesh size_mode:=catalog "
    "sequence_ids:=carryon,standard,large "
    "xy_jitter_range:=0.12,0.12 yaw_range:=-0.6,0.6 "
    "observe_pose_name:=pickup_observe semantic_require_backend:=bbox_fill"
)

RESIDUAL_PATTERN = (
    "ros2 launch luggage_gazebo|ign gazeb[o]|gz sim|ros_gz_bridge|"
    "parameter_bridge|clock_bridge|camera_bridge"
)


def residual_pids():
    proc = subprocess.run(["pgrep", "-af", RESIDUAL_PATTERN],
                          capture_output=True, text=True, check=False)
    out = []
    for line in (proc.stdout or "").splitlines():
        if not line.strip():
            continue
        if "sim_startup_rehearsal" in line or "pgrep" in line:
            continue
        out.append(line.strip())
    return out


def count_clock_publishers(env):
    proc = subprocess.run(["ros2", "topic", "info", "/clock"],
                          capture_output=True, text=True, timeout=15,
                          check=False, env=env)
    match = re.search(r"Publisher count:\s*(\d+)", proc.stdout or "")
    return int(match.group(1)) if match else 0


class RosStartupProbe:
    """Persistent-node implementation of the stage checks.

    Built once per launch attempt. Service clients and the ``/joint_states``
    subscription are created up front and reused, so discovery happens once.
    """

    def __init__(self, node, rsp_node_name="robot_state_publisher",
                 param_name="robot_description",
                 controller_manager="/controller_manager",
                 required_controllers=REQUIRED_CONTROLLERS,
                 expected_joint_count=EXPECTED_JOINT_COUNT):
        from controller_manager_msgs.srv import ListControllers
        from rcl_interfaces.srv import GetParameters
        from sensor_msgs.msg import JointState

        self._node = node
        self._param_name = param_name
        self._controller_manager = controller_manager
        self._required = tuple(required_controllers)
        self._expected_joint_count = int(expected_joint_count)
        self._list_srv = "%s/list_controllers" % controller_manager.rstrip("/")

        self._param_client = node.create_client(
            GetParameters, "/%s/get_parameters" % rsp_node_name.strip("/"))
        self._cm_client = node.create_client(ListControllers, self._list_srv)
        self._ListControllers = ListControllers
        self._GetParameters = GetParameters

        self._joint_state = None
        self._joint_sub = node.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10)

        self.robot_description = ""
        self.robot_description_bytes = 0

    def _on_joint_state(self, msg):
        self._joint_state = msg

    def spin(self, timeout_sec=0.1):
        import rclpy
        rclpy.spin_once(self._node, timeout_sec=timeout_sec)

    def check_robot_description(self):
        """One bounded async ``get_parameters`` on a persistent client.

        Unlike ``ros2 param get`` this reuses one already-discovered client, so
        a caller polling this cannot be blocked behind node construction.
        """
        import rclpy
        if not self._param_client.service_is_ready():
            self.spin(0.1)
            return False, {"service_ready": False}
        request = self._GetParameters.Request()
        request.names = [self._param_name]
        future = self._param_client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=3.0)
        if not future.done():
            future.cancel()
            return False, {"service_ready": True, "response": False}
        response = future.result()
        if response is None or not response.values:
            return False, {"service_ready": True, "response": False}
        value = response.values[0].string_value or ""
        if not value.lstrip().startswith("<"):
            return False, {"service_ready": True, "bytes": len(value)}
        self.robot_description = value
        self.robot_description_bytes = len(value)
        return True, {"bytes": len(value)}

    def check_controller_manager(self):
        self.spin(0.1)
        ready = self._cm_client.service_is_ready()
        return ready, {"service": self._list_srv, "ready": bool(ready)}

    def check_controllers_active(self):
        import rclpy
        if not self._cm_client.service_is_ready():
            self.spin(0.1)
            return False, {"service_ready": False}
        future = self._cm_client.call_async(self._ListControllers.Request())
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        if not future.done():
            future.cancel()
            return False, {"service_ready": True, "response": False}
        response = future.result()
        if response is None:
            return False, {"service_ready": True, "response": False}
        states = {c.name: c.state for c in response.controller}
        ok = all(states.get(name) == "active" for name in self._required)
        return ok, {"controllers": states}

    def check_joint_states(self):
        self.spin(0.2)
        msg = self._joint_state
        if msg is None:
            return False, {"received": False}
        import math
        positions = list(msg.position)
        finite = [p for p in positions if math.isfinite(p)]
        ok = (len(positions) >= self._expected_joint_count and
              len(finite) == len(positions))
        return ok, {
            "received": True,
            "name_count": len(msg.name),
            "position_count": len(positions),
            "all_finite": len(finite) == len(positions),
        }


class LaunchAttempt:
    def __init__(self, out_dir, env, pidfile, launch_params):
        self.out_dir = Path(out_dir)
        self.env = env
        self.pidfile = pidfile
        self.launch_params = launch_params
        self.proc = None
        self.log_path = self.out_dir / "launch.log"

    def start(self):
        self.out_dir.mkdir(parents=True, exist_ok=True)
        handle = open(self.log_path, "w", encoding="utf-8")
        command = ("ros2 launch luggage_gazebo sim_world.launch.py %s"
                   % self.launch_params)
        self.proc = subprocess.Popen(
            ["bash", "-lc", command], stdout=handle, stderr=subprocess.STDOUT,
            env=self.env, start_new_session=True)
        Path(self.pidfile).write_text("%s\n" % self.proc.pid)
        return self.proc.pid

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def log_text(self):
        try:
            return self.log_path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return ""


def stop_stack(repo, env, log_path):
    with open(log_path, "w", encoding="utf-8") as handle:
        subprocess.run([str(repo / "scripts" / "stop_sim.sh")],
                       stdout=handle, stderr=subprocess.STDOUT, env=env,
                       check=False, timeout=180)


def wait_residuals_clear(timeout_sec=60.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        left = residual_pids()
        if not left:
            return []
        time.sleep(1.0)
    return residual_pids()


def run_iteration(index, args, repo, env, deadlines):
    """One rehearsal slot: up to ``--max-relaunch`` bounded startup attempts.

    A startup that fails before any case is attempted costs nothing but wall
    clock: no scoring cursor moved, no seed was consumed, the revision and
    config hashes are unchanged. Charging such a failure against a scarce
    budget is what turned a ~10% launch race into three dead PF-R7
    generations, so the rehearsal measures both the raw per-launch rate and
    the effective rate under relaunch.
    """
    attempts = []
    for attempt_index in range(1, args.max_relaunch + 2):
        record = run_launch_attempt(index, attempt_index, args, repo, env,
                                    deadlines)
        attempts.append(record)
        if record.get("ok"):
            break

    final = dict(attempts[-1])
    final["launch_attempts"] = len(attempts)
    final["attempt_reasons"] = [a.get("reason") for a in attempts]
    return final


def run_launch_attempt(index, attempt_index, args, repo, env, deadlines):
    out_dir = (Path(args.out) / ("iter_%02d" % index)
               / ("attempt_%02d" % attempt_index))
    out_dir.mkdir(parents=True, exist_ok=True)

    before = residual_pids()
    if before:
        record = {
            "iteration": index, "attempt": attempt_index, "ok": False,
            "reason": "residuals_before_launch",
            "residuals_before": before,
        }
        (out_dir / "ready.json").write_text(json.dumps(record, indent=2))
        return record

    clocks_before = count_clock_publishers(env)
    if clocks_before != 0:
        record = {
            "iteration": index, "attempt": attempt_index, "ok": False,
            "reason": "clock_publishers_before_launch",
            "n_clock_publishers_before": clocks_before,
        }
        (out_dir / "ready.json").write_text(json.dumps(record, indent=2))
        return record

    attempt = LaunchAttempt(out_dir, env, args.pidfile, args.launch_params)
    launch_pid = attempt.start()

    import rclpy
    rclpy.init(args=None)
    node = rclpy.create_node("sim_startup_rehearsal_probe")
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
        result = run_stages(checks, deadlines,
                            global_deadline_sec=args.global_timeout_sec,
                            abort=abort)
    except Exception as exc:  # pragma: no cover - defensive
        result = ProbeResult(False, "probe_exception: %s" % exc, [], 0.0)
    finally:
        clocks_after = count_clock_publishers(env)
        log_text = attempt.log_text()
        try:
            node.destroy_node()
        finally:
            rclpy.shutdown()

    record = dict(result.as_dict())
    record.update({
        "iteration": index,
        "attempt": attempt_index,
        "launch_pid": launch_pid,
        "n_clock_publishers": clocks_after,
        "robot_description_bytes": probe.robot_description_bytes,
        "spawn_marker_count": spawn_marker_count(log_text),
        "marker_order_ok": marker_order_ok(log_text),
        "plugin_received_urdf": plugin_received_urdf(log_text),
        "launch_log": str(attempt.log_path.relative_to(Path(args.out))),
        "watchdog_fired": "startup_failed: plugin_urdf_not_received" in log_text,
    })

    stop_stack(repo, env, out_dir / "stop_sim.log")
    left = wait_residuals_clear(args.teardown_timeout_sec)
    record["residuals_after"] = left
    record["teardown_residual_count"] = len(left)
    if left and record["ok"]:
        record["ok"] = False
        record["reason"] = "teardown_residuals"
    try:
        os.unlink(args.pidfile)
    except OSError:
        pass
    if attempt.alive():
        try:
            os.killpg(attempt.proc.pid, signal.SIGTERM)
        except OSError:
            pass

    (out_dir / "ready.json").write_text(json.dumps(record, indent=2))
    return record


def summarize(records, args, repo):
    ok = [r for r in records if r.get("ok")]
    reasons = {}
    launch_attempts = 0
    launch_failures = 0
    for record in records:
        reasons[record.get("reason", "unknown")] = \
            reasons.get(record.get("reason", "unknown"), 0) + 1
        attempt_reasons = record.get("attempt_reasons") or [record.get("reason")]
        launch_attempts += len(attempt_reasons)
        launch_failures += len([r for r in attempt_reasons if r != "ok"])

    durations = {stage: [] for stage in STAGES}
    for record in records:
        for stage, value in (record.get("stage_durations_sec") or {}).items():
            durations.setdefault(stage, []).append(value)

    stage_stats = {}
    for stage, values in durations.items():
        if not values:
            continue
        stage_stats[stage] = {
            "n": len(values),
            "min_sec": round(min(values), 3),
            "median_sec": round(statistics.median(values), 3),
            "max_sec": round(max(values), 3),
        }

    head = subprocess.run(["git", "-C", str(repo), "rev-parse", "HEAD"],
                          capture_output=True, text=True, check=False)
    dirty = subprocess.run(
        ["git", "-C", str(repo), "status", "--porcelain",
         "--untracked-files=all"], capture_output=True, text=True, check=False)

    return {
        "tool": "sim_startup_rehearsal",
        "scored": False,
        "commit": (head.stdout or "").strip(),
        "dirty_count": len([x for x in (dirty.stdout or "").splitlines() if x]),
        "launch_params": args.launch_params,
        "ros_domain_id": args.ros_domain_id,
        "iterations": len(records),
        "successes": len(ok),
        "success_rate": round(len(ok) / len(records), 4) if records else 0.0,
        "max_relaunch": args.max_relaunch,
        # Raw rate counts every launch; effective rate counts a slot as good
        # if any bounded pre-scoring relaunch reached a ready stack.
        "launch_attempts_total": launch_attempts,
        "launch_failures_total": launch_failures,
        "raw_launch_success_rate": (
            round((launch_attempts - launch_failures) / launch_attempts, 4)
            if launch_attempts else 0.0),
        "reasons": reasons,
        "stage_durations_sec": stage_stats,
        "first_failing_stage": {
            r["iteration"]: r.get("last_stage")
            for r in records if not r.get("ok")
        },
        "iteration_records": [
            {"iteration": r["iteration"], "ok": r.get("ok"),
             "reason": r.get("reason"), "elapsed_sec": r.get("elapsed_sec"),
             "last_stage": r.get("last_stage"),
             "launch_attempts": r.get("launch_attempts", 1),
             "attempt_reasons": r.get("attempt_reasons"),
             "watchdog_fired": r.get("watchdog_fired")}
            for r in records
        ],
    }


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iterations", type=int, default=10)
    parser.add_argument("--out", required=True)
    parser.add_argument("--ros-domain-id", type=int, default=7)
    parser.add_argument("--pidfile", default="/tmp/elfin_humble_sim.pid")
    parser.add_argument("--launch-params", default=LAUNCH_PARAMS)
    parser.add_argument("--global-timeout-sec", type=float, default=180.0)
    parser.add_argument("--teardown-timeout-sec", type=float, default=90.0)
    parser.add_argument("--max-relaunch", type=int, default=0,
                        help="bounded pre-scoring relaunches per slot")
    parser.add_argument("--settle-sec", type=float, default=5.0,
                        help="quiet time between teardown and the next launch")
    for stage, value in DEFAULT_STAGE_DEADLINE.items():
        parser.add_argument("--deadline-%s" % stage.replace("_", "-"),
                            type=float, default=value, dest="dl_%s" % stage)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    repo = REPO
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    env["ELFIN_SIM_PIDFILE"] = args.pidfile
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)

    deadlines = {stage: getattr(args, "dl_%s" % stage) for stage in STAGES}

    records = []
    for index in range(1, args.iterations + 1):
        print("[rehearsal] iteration %d/%d" % (index, args.iterations),
              flush=True)
        record = run_iteration(index, args, repo, env, deadlines)
        print("[rehearsal] iteration %d: ok=%s reason=%s last_stage=%s"
              % (index, record.get("ok"), record.get("reason"),
                 record.get("last_stage")), flush=True)
        records.append(record)
        (out / "rehearsal_records.jsonl").open("a", encoding="utf-8").write(
            json.dumps(record) + "\n")
        if index < args.iterations:
            time.sleep(args.settle_sec)

    summary = summarize(records, args, repo)
    (out / "rehearsal_summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    return 0 if summary["successes"] == summary["iterations"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
