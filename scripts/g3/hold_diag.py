#!/usr/bin/env python3
"""Single-leg hold diagnosis (bounded; user-authorized 2026-10-20).

Stage 1 — localize the r10/r11 transit-end hold failure WITHOUT the
product flow: no box, no vacuum, no prior, no MoveIt. Sequence:

  1. short positioning move to the frozen leg's FIRST knot;
  2. PRE baseline capture at the start posture (is the drift already
     there before any long motion?);
  3. replay ONE frozen leg verbatim (r11 cert_request leg 0: 323
     knots, 165.9 s — the exact trajectory that ended in
     HANDOFF_SETTLE_TIMEOUT);
  4. HOLD capture at the end posture (default 60 s vs the production
     6 s window — separates "window too short" from "divergence");
  5. summary.json: per-joint error slope (linear fit) over hold,
     split-window slopes (accelerating/decelerating/constant),
     goal result, contact counts, capture health.

Captures: /joint_states (position+velocity+effort if provided) and
/elfin_arm_controller/controller_state (reference/feedback/output),
both JSONL. gz contacts: /world/airport_loading/physics/contacts via
`ign topic -e` started BEFORE the goal (publisher exists from boot if
the Physics system exposes it; absence is recorded, not assumed).
"""
import json
import math
import os
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _pt(row):
    return {
        "positions": list(row["positions"]),
        "velocities": list(row.get("velocities") or []),
        "accelerations": list(row.get("accelerations") or []),
        "time_from_start_ns": int(row["time_from_start_ns"]),
    }


def _dur(ns):
    sec, rem = divmod(int(ns), 10**9)
    from builtin_interfaces.msg import Duration
    d = Duration()
    d.sec = sec
    d.nanosec = rem
    return d


def _fit_slope(samples):
    """[(t_s, value)] -> least-squares slope (value/s)."""
    n = len(samples)
    if n < 3:
        return None
    ts = [s[0] for s in samples]
    vs = [s[1] for s in samples]
    t0 = ts[0]
    tbar = sum(t - t0 for t in ts) / n
    vbar = sum(vs) / n
    num = sum((t - t0 - tbar) * (v - vbar) for t, v in zip(ts, vs))
    den = sum((t - t0 - tbar) ** 2 for t in ts)
    return num / den if den else None


class Diag(Node):
    def __init__(self, out_dir):
        super().__init__("hold_diag")
        self.out = out_dir
        self.js = open(os.path.join(out_dir, "joint_states.jsonl"), "a")
        self.cs = open(os.path.join(out_dir, "controller_state.jsonl"), "a")
        self.js_rows = 0
        self.cs_rows = 0
        self.effort_present = None
        self.create_subscription(
            JointState, "/joint_states", self._on_js, 200)
        self.create_subscription(
            JointTrajectoryControllerState,
            "/elfin_arm_controller/controller_state", self._on_cs, 200)
        self.ac = ActionClient(
            self, FollowJointTrajectory,
            "/elfin_arm_controller/follow_joint_trajectory")

    def _on_js(self, msg):
        self.js_rows += 1
        effort = list(msg.effort or [])
        if self.effort_present is None and effort:
            self.effort_present = True
        row = {
            "t": time.time(),
            "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": effort,
        }
        self.js.write(json.dumps(row) + "\n")

    def _on_cs(self, msg):
        self.cs_rows += 1
        ref = getattr(msg, "reference", None) or getattr(msg, "desired", None)
        fb = getattr(msg, "feedback", None) or getattr(msg, "actual", None)
        out = getattr(msg, "output", None)
        row = {
            "t": time.time(),
            "reference.positions": list(ref.positions) if ref else None,
            "feedback.positions": list(fb.positions) if fb else None,
            "output.positions": list(out.positions) if out else None,
        }
        self.cs.write(json.dumps(row) + "\n")

    def flush(self):
        self.js.flush()
        self.cs.flush()


def _goal(names, points):
    traj = JointTrajectory()
    traj.joint_names = list(names)
    for p in points:
        tp = JointTrajectoryPoint()
        tp.positions = list(p["positions"])
        if p.get("velocities"):
            tp.velocities = list(p["velocities"])
        if p.get("accelerations"):
            tp.accelerations = list(p["accelerations"])
        tp.time_from_start = _dur(p["time_from_start_ns"])
        traj.points.append(tp)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    return goal


def _send_wait(diag, goal, timeout_s, label):
    diag.get_logger().info("%s: sending goal (%d points)"
                           % (label, len(goal.trajectory.points)))
    t0 = time.time()
    fut = diag.ac.send_goal_async(goal)
    fut.add_done_callback(lambda _f: None)
    handle = None
    while time.time() - t0 < 30.0:
        rclpy.spin_once(diag, timeout_sec=0.2)
        if fut.done():
            handle = fut.result()
            break
    if handle is None or not handle.accepted:
        return {"label": label, "accepted": False}
    res_fut = handle.get_result_async()
    while time.time() - t0 < timeout_s:
        rclpy.spin_once(diag, timeout_sec=0.2)
        if res_fut.done():
            res = res_fut.result().result
            return {"label": label, "accepted": True,
                    "error_code": int(res.error_code),
                    "error_string": str(res.error_string),
                    "wall_s": time.time() - t0}
    return {"label": label, "accepted": True, "error_code": None,
            "error_string": "diag wait timeout", "wall_s": time.time() - t0}


def _spin_seconds(diag, seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(diag, timeout_sec=0.2)
    diag.flush()


def main():
    out_dir, traj_path = sys.argv[1], sys.argv[2]
    hold_s = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0
    pre_s = 10.0
    doc = json.load(open(traj_path))
    names = doc["joint_names"]
    points = doc["points"]
    start = points[0]["positions"]
    end = points[-1]["positions"]
    span_s = points[-1]["time_from_start_ns"] / 1e9

    rclpy.init()
    diag = Diag(out_dir)
    if not diag.ac.wait_for_server(timeout_sec=60.0):
        print(json.dumps({"fatal": "action server unavailable"}))
        return 4
    # controller_state must be flowing before we trust captures
    _spin_seconds(diag, 3.0)
    if diag.cs_rows == 0 or diag.js_rows == 0:
        print(json.dumps({"fatal": "captures not flowing",
                          "cs_rows": diag.cs_rows, "js_rows": diag.js_rows}))
        return 5

    # gz contact probe + capture (started BEFORE the goal; recorded
    # availability, never assumed)
    topics = subprocess.run(
        ["ign", "topic", "-l"], capture_output=True, text=True,
        timeout=10).stdout.split()
    contact_topics = [t for t in topics if "contact" in t]
    contact_procs = {}
    for t in contact_topics:
        safe = t.strip("/").replace("/", "_")
        fn = os.path.join(out_dir, "gz_%s.txt" % safe)
        contact_procs[t] = subprocess.Popen(
            ["ign", "topic", "-e", "-t", t],
            stdout=open(fn, "w"), stderr=subprocess.DEVNULL)

    events = [{"event": "diag_start", "t": time.time()}]

    # 1. positioning move to the frozen start posture
    pos_points = [{
        "positions": start,
        "velocities": [], "accelerations": [],
        "time_from_start_ns": 6_000_000_000,
    }]
    r1 = _send_wait(diag, _goal(names, pos_points), 60.0, "positioning")
    events.append({"event": "positioning", "t": time.time(), "result": r1})
    _spin_seconds(diag, 5.0)  # settle

    # 2. baseline at the START posture
    events.append({"event": "pre_baseline_begin", "t": time.time()})
    pre_t0 = time.time()
    _spin_seconds(diag, pre_s)
    events.append({"event": "pre_baseline_end", "t": time.time()})

    # 3. frozen leg, verbatim
    r2 = _send_wait(diag, _goal(names, points), span_s + 180.0, "frozen_leg")
    events.append({"event": "frozen_leg", "t": time.time(), "result": r2})

    # 4. HOLD at the end posture
    events.append({"event": "hold_begin", "t": time.time()})
    hold_t0 = time.time()
    _spin_seconds(diag, hold_s)
    events.append({"event": "hold_end", "t": time.time()})

    for p in contact_procs.values():
        p.terminate()
    diag.flush()

    # 5. summary from the captured rows
    js_rows = [json.loads(line) for line in open(
        os.path.join(out_dir, "joint_states.jsonl"))]
    order = None
    for r in js_rows:
        if r.get("name"):
            order = r["name"]
            break
    idx = {n: i for i, n in enumerate(order or names)}

    def window(t0, t1):
        return [r for r in js_rows if t0 <= r["t"] <= t1]

    pre = window(pre_t0 - pre_s, pre_t0)
    hold = window(hold_t0, hold_t0 + hold_s)
    summary = {
        "frozen_leg": os.path.basename(traj_path),
        "n_points": len(points), "span_s": span_s,
        "start": start, "end": end,
        "goal_positioning": r1, "goal_frozen_leg": r2,
        "effort_present": diag.effort_present is True,
        "rows_joint_states": diag.js_rows,
        "rows_controller_state": diag.cs_rows,
        "contact_topics_seen": contact_topics,
        "per_joint": {},
        "events": events,
    }
    for jn, i in idx.items():
        pre_samples = [(r["t"] - pre_t0, r["position"][i]) for r in pre]
        hold_err = [(r["t"] - hold_t0, end[i] - r["position"][i])
                    for r in hold]
        half = len(hold_err) // 2 or 1
        summary["per_joint"][jn] = {
            "pre_slope_rad_s": _fit_slope(pre_samples),
            "pre_total_rad": (
                pre_samples[-1][1] - pre_samples[0][1]
                if len(pre_samples) > 1 else None),
            "hold_err_first_rad": hold_err[0][1] if hold_err else None,
            "hold_err_last_rad": hold_err[-1][1] if hold_err else None,
            "hold_slope_rad_s": _fit_slope(hold_err),
            "hold_slope_first_half": _fit_slope(hold_err[:half]),
            "hold_slope_second_half": _fit_slope(hold_err[half:]),
        }
    with open(os.path.join(out_dir, "summary.json"), "w") as fh:
        json.dump(summary, fh, indent=1)
    print(json.dumps({k: summary[k] for k in
                      ("goal_frozen_leg", "per_joint",
                       "contact_topics_seen", "effort_present")}, indent=1))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
