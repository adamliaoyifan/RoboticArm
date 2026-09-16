"""Isolated-domain MoveIt plan-only helper. Eval only; never executes.

Starts ``isolated_moveit.launch.py`` under a ROS domain other than the live
sim (7), with ``ROS_LOCALHOST_ONLY=1`` and ``allow_trajectory_execution``
false. Plans from recorded start joints to the estimated attach pose and
returns the joint trajectory for overlay. The parent process must not have
``rclpy`` initialized on domain 7.
"""
from __future__ import division

import json
import os
import subprocess
import sys
import tempfile

from luggage_perception.eval.isolated_domain import (
    DEFAULT_REPLAY_DOMAIN_ID,
    isolated_replay_env,
)

JOINTS = [
    "elfin_joint1", "elfin_joint2", "elfin_joint3",
    "elfin_joint4", "elfin_joint5", "elfin_joint6",
]


def plan_pick_segments(start_joints, waypoints, domain_id=None,
                       timeout_sec=45.0, environ=None):
    """Plan attach (or last cartesian) from *start_joints*. Never execute.

    Returns a dict with ``t_sec``, ``q``, ``xyz``, ``message``. *xyz* is
    empty unless the planner wrote FK samples.
    """
    env = isolated_replay_env(domain_id, environ=environ)
    if not start_joints or len(start_joints) < 6:
        return {"message": "MoveIt skipped: no start joints",
                "t_sec": [], "q": [], "xyz": []}
    attach = None
    for wp in waypoints or []:
        if wp.get("name") == "attach":
            attach = wp
    if attach is None and waypoints:
        attach = waypoints[-1]
    if attach is None or not attach.get("xyz"):
        return {"message": "MoveIt skipped: no attach waypoint",
                "t_sec": [], "q": [], "xyz": []}
    script = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "isolated_moveit_worker.py")
    if not os.path.isfile(script):
        return {"message": "MoveIt skipped: worker missing",
                "t_sec": [], "q": [], "xyz": []}
    request = {
        "start_joints": [float(v) for v in start_joints[:6]],
        "xyz": [float(v) for v in attach["xyz"]],
        "timeout_sec": float(timeout_sec),
        "joint_names": list(JOINTS),
    }
    with tempfile.TemporaryDirectory(prefix="elfin_replay_moveit_") as tmp:
        req_path = os.path.join(tmp, "request.json")
        out_path = os.path.join(tmp, "result.json")
        with open(req_path, "w", encoding="utf-8") as handle:
            json.dump(request, handle)
        cmd = [sys.executable, script, "--request", req_path, "--out", out_path]
        try:
            proc = subprocess.run(
                cmd, env=env, timeout=float(timeout_sec) + 30.0,
                capture_output=True, text=True, check=False)
        except subprocess.TimeoutExpired:
            return {"message": "MoveIt skipped: worker timeout",
                    "t_sec": [], "q": [], "xyz": []}
        if not os.path.isfile(out_path):
            err = (proc.stderr or proc.stdout or "")[-500:]
            return {"message": "MoveIt skipped: worker failed (%s) %s"
                    % (proc.returncode, err),
                    "t_sec": [], "q": [], "xyz": []}
        with open(out_path, encoding="utf-8") as handle:
            return json.load(handle)
