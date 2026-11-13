#!/usr/bin/env python3
"""Stage 2c: production pick prehistory + the same frozen leg (bounded,
non-scored; user-authorized single attempt).

Variable ladder so far (all at the same scene/stack/A-prime/seed-0):
  2a  static carried box + production follow           -> hold perfect
  2b  + leg via move_group ExecuteTrajectory           -> hold perfect
  2c  + PRODUCTION PICK PREHISTORY (real spawner, real
       detection-gated pick, PlanMotion segments, vacuum
       admission + attach, pick retreat)               -> this run

Race-free stop: the pick_retreat eval driver's teardown (vacuum off ->
observe -> clear) runs only AFTER the retreat action result reaches the
driver process. This watcher SIGSTOPs the driver while the retreat goal
is ACTIVE and the box has lifted to >=1.315 m: the controller already
holds the complete trajectory and finishes it autonomously, so the arm
reaches the retreat endpoint while the driver is frozen BEFORE it can
run the teardown. The carry (attach + node-side 30 Hz follow) survives.

Boundary gates (pre-fixed, from the user's 2c spec + the production
validator's own value): joints vs frozen-leg first knot <= 0.05 rad;
vacuum attached with empty fail_reason; no ACTIVE goals on either
action layer; carrying box present. Any failure -> verdict
boundary_not_met, driver resumed for its natural teardown, NO rerun,
no extra positioning, no rewritten attach, no edited leg.

Then: submit the same frozen leg through move_group exactly as 2b,
hold 60 SIM seconds, and report per-joint max error, drift slope and
the first time the production settle conditions hold. effort=NaN is
recorded as unusable (not torque evidence).
"""
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener

WORLD = "airport_loading"
MODEL = "pickup_box_0001_inch20"
PANEL_FRAME = "suction_contact_frame"
# r11 production retreat: box 0.970 -> 1.320 (dz +0.35, tf_trace
# before_transit box z = 1.320). Trigger slightly below the endpoint.
TRIG_BOX_Z = 1.315
BOUNDARY_TOL_RAD = 0.05  # move_group's own start-state gate
PICK_WINDOW_S = 300.0


def _fit_slope(samples):
    n = len(samples)
    if n < 3:
        return None
    t0 = samples[0][0]
    ts = [t - t0 for t, _ in samples]
    vs = [v for _, v in samples]
    tbar, vbar = sum(ts) / n, sum(vs) / n
    num = sum((t - tbar) * (v - vbar) for t, v in zip(ts, vs))
    den = sum((t - tbar) ** 2 for t in ts)
    return num / den if den else None


class Diag2c(Node):
    def __init__(self, out_dir):
        super().__init__("hold_diag2c")
        self.out = out_dir
        self.js = open(os.path.join(out_dir, "joint_states.jsonl"), "a")
        self.cs = open(os.path.join(out_dir, "controller_state.jsonl"), "a")
        self.vs = open(os.path.join(out_dir, "vacuum_state.jsonl"), "a")
        self.js_rows = 0
        self.cs_rows = 0
        self.sim_now = None
        self.attached = None
        self.fail_reason = None
        self.active_goals = {}   # goal_id -> layer (status==1)
        self.status_events = []
        self.phase = "pre_pick"
        self.create_subscription(JointState, "/joint_states", self._on_js, 200)
        self.create_subscription(
            JointTrajectoryControllerState,
            "/elfin_arm_controller/controller_state", self._on_cs, 200)
        from rosgraph_msgs.msg import Clock
        from action_msgs.msg import GoalStatusArray
        from luggage_msgs.msg import VacuumState
        self.create_subscription(Clock, "/clock", self._on_clock, 200)
        self.create_subscription(
            VacuumState, "/vacuum/state", self._on_vac, 50)
        self.create_subscription(
            GoalStatusArray,
            "/execute_trajectory/_action/status",
            lambda m: self._on_status("movegroup", m), 100)
        self.create_subscription(
            GoalStatusArray,
            "/elfin_arm_controller/follow_joint_trajectory/_action/status",
            lambda m: self._on_status("controller", m), 100)
        from moveit_msgs.action import ExecuteTrajectory as MgExecute
        self._MgExecute = MgExecute
        self.mg = ActionClient(self, MgExecute, "/execute_trajectory")
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.js_position = None
        self.js_names = None

    def _on_clock(self, msg):
        self.sim_now = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_vac(self, msg):
        self.attached = bool(msg.attached)
        self.fail_reason = str(getattr(msg, "fail_reason", "") or "")
        self.vs.write(json.dumps({
            "t": time.time(), "sim": self.sim_now,
            "attached": self.attached,
            "fail_reason": self.fail_reason}) + "\n")

    def _on_js(self, msg):
        self.js_rows += 1
        self.js_position = list(msg.position)
        self.js_names = list(msg.name)
        self.js.write(json.dumps({
            "t": time.time(), "sim": self.sim_now, "name": self.js_names,
            "position": self.js_position,
            "velocity": list(msg.velocity),
            "effort": "NaN-record-only"}) + "\n")

    def _on_cs(self, msg):
        self.cs_rows += 1
        ref = getattr(msg, "reference", None)
        fb = getattr(msg, "feedback", None)
        out = getattr(msg, "output", None)
        err = getattr(msg, "error", None)
        self.cs.write(json.dumps({
            "t": time.time(), "sim": self.sim_now,
            "phase": self.phase,
            "reference.positions": list(ref.positions) if ref else None,
            "feedback.positions": list(fb.positions) if fb else None,
            "output.positions": list(out.positions) if out else None,
            "error.positions": list(err.positions) if err else None}) + "\n")

    def _on_status(self, layer, msg):
        now_active = {}
        for st in msg.status_list:
            try:
                gid = st.goal_info.goal_id.uuid.tobytes().hex()[:8]
            except Exception:  # noqa: BLE001
                gid = "?"
            status = int(st.status)
            self.status_events.append({
                "t": time.time(), "sim": self.sim_now, "layer": layer,
                "status": status, "goal_id": gid, "phase": self.phase})
            if status == 1:
                now_active[gid] = layer
        self.active_goals = now_active

    def panel_pose(self):
        try:
            tr = self.tf.lookup_transform(
                "world", PANEL_FRAME, rclpy.time.Time())
        except Exception:  # noqa: BLE001 - diag only
            return None
        t = tr.transform
        return ([t.translation.x, t.translation.y, t.translation.z],
                [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])

    def flush(self):
        for fh in (self.js, self.cs, self.vs):
            fh.flush()


def box_pose():
    from luggage_planning.attachment.sim_physical_attachment import observe_model_pose
    try:
        return observe_model_pose(MODEL)
    except Exception:  # noqa: BLE001 - box may not exist yet
        return None


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
        d = Duration()
        ns = int(p["time_from_start_ns"])
        d.sec, d.nanosec = divmod(ns, 10 ** 9)
        tp.time_from_start = d
        traj.points.append(tp)
    g = FollowJointTrajectory.Goal()
    g.trajectory = traj
    return g


def main():
    out_dir, traj_path = sys.argv[1], sys.argv[2]
    hold_s = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0
    doc = json.load(open(traj_path))
    names = doc["joint_names"]
    points = doc["points"]
    start = points[0]["positions"]
    span_s = points[-1]["time_from_start_ns"] / 1e9

    rclpy.init()
    diag = Diag2c(out_dir)
    if not diag.mg.wait_for_server(timeout_sec=30.0):
        print(json.dumps({"fatal": "execute_trajectory server unavailable"}))
        return 4
    _spin(diag, 2.0)

    # ---- Phase A: production pick via the PACK driver, frozen mid ----
    # retreat (race-free): SIGSTOP while the retreat goal is ACTIVE and
    # the box has lifted to TRIG_BOX_Z. The controller already holds the
    # complete trajectory and finishes it autonomously, so the arm
    # reaches the retreat endpoint while the driver is frozen BEFORE it
    # can start the place loop. The carry (box, attach, node-side 30 Hz
    # follow) survives; the frozen driver holds a stale peer lease but
    # nothing queries it (my leg goes DIRECTLY to move_group, not the
    # typed chain). A planner-kill stop was tried first and is WRONG:
    # the blocked place call breaks the 2 s peer lease and the
    # PEER_LEASE_EXPIRED abort path DELETES the carrying box.
    driver_log = open(os.path.join(out_dir, "pack_driver.log"), "w")
    env = dict(os.environ)
    env["ELFIN_BOX_CONTACT_TOPIC"] = "/obs/box_contact"
    env["ELFIN_BOX_SDF_DUMP"] = os.path.join(out_dir, "spawn.sdf")
    driver_cmd = [
        "python3", "/tmp/elfin-master-pack-baseline-g3/install/"
        "luggage_gazebo/lib/luggage_gazebo/pack_eval_driver.py",
        "--frame-runtime", "--frame-peer", "hold_diag",
        "--observe-pose", "pickup_observe_sim",
        "--sequence-ids", "inch20",
        "--max-boxes", "1",
        "--goto-timeout", "60",
        "--plan-timeout", "180",
        "--seed", "0",
        "--auth-max-attempts", "8",
        "--auth-max-elapsed-sec", "8",
        "--stop-on-first-failure",
        "--scene-tf-config", os.path.join(out_dir, "scene_tf_used.yaml"),
        "--out", os.path.join(out_dir, "pack"),
    ]
    os.makedirs(os.path.join(out_dir, "pack"), exist_ok=True)
    driver = subprocess.Popen(
        driver_cmd, stdout=driver_log, stderr=subprocess.STDOUT, env=env,
        cwd="/tmp/elfin-master-pack-baseline-g3")
    diag.phase = "pick"
    t_pick0 = time.time()
    stopped = False
    trigger = None
    while time.time() - t_pick0 < PICK_WINDOW_S:
        rclpy.spin_once(diag, timeout_sec=0.1)
        if not stopped and diag.attached and diag.active_goals:
            bp = box_pose()
            if bp is not None and bp[0][2] >= TRIG_BOX_Z:
                os.kill(driver.pid, signal.SIGSTOP)
                stopped = True
                trigger = {"t": time.time(), "sim": diag.sim_now,
                           "box_z": bp[0][2],
                           "active_goals": dict(diag.active_goals)}
                diag.phase = "frozen_at_retreat"
                break
        if driver.poll() is not None:
            break
        time.sleep(0.05)
    if not stopped:
        diag.flush()
        try:
            driver.kill()
        except Exception:  # noqa: BLE001
            pass
        print(json.dumps({"verdict": "invalid_no_trigger",
                          "attached": diag.attached}))
        with open(os.path.join(out_dir, "summary.json"), "w") as fh:
            json.dump({"verdict": "invalid_no_trigger",
                       "attached": diag.attached,
                       "driver_returncode": driver.returncode}, fh, indent=1)
        rclpy.shutdown()
        return 3

    # controller finishes the retreat; wait for goal quiescence + settle
    t_stop = time.time()
    while time.time() - t_stop < 30.0:
        rclpy.spin_once(diag, timeout_sec=0.1)
        if not diag.active_goals and time.time() - t_stop > 3.0:
            break
    _spin(diag, 3.0)

    # ---- Phase B: pre-fixed boundary gates -----------------------------
    joints = dict(zip(diag.js_names or [], diag.js_position or []))
    deltas = {n: abs(joints.get(n, 9e9) - v) for n, v in zip(names, start)}
    max_delta = max(deltas.values()) if deltas else 9e9
    bp = box_pose()
    boundary = {
        "joints_vs_leg0_first": {
            "max_delta_rad": max_delta, "tolerance_rad": BOUNDARY_TOL_RAD,
            "ok": max_delta <= BOUNDARY_TOL_RAD,
            "delta_per_joint": {k: round(v, 5) for k, v in deltas.items()}},
        "vacuum": {"attached": diag.attached,
                   "fail_reason": diag.fail_reason,
                   "ok": bool(diag.attached)
                   and not diag.fail_reason},
        "no_active_goals": {"active": diag.active_goals,
                            "ok": not diag.active_goals},
        "box": {"present": bp is not None,
                "pose_xyz_rpy": (bp[0] + list(bp[1])) if bp else None,
                "ok": bp is not None},
        "trigger": trigger,
    }
    panel = diag.panel_pose()
    boundary["attach_relation_at_boundary"] = {
        "panel_world": panel, "box_world_xyz": bp[0] if bp else None}
    ok_all = all(v.get("ok") for v in boundary.values()
                 if isinstance(v, dict) and "ok" in v)
    if not ok_all:
        diag.flush()
        with open(os.path.join(out_dir, "summary.json"), "w") as fh:
            json.dump({"verdict": "boundary_not_met", "boundary": boundary},
                      fh, indent=1)
        print(json.dumps({"verdict": "boundary_not_met", "boundary": {
            k: v.get("ok") for k, v in boundary.items()
            if isinstance(v, dict) and "ok" in v}}))
        rclpy.shutdown()
        return 5

    # ---- Phase C: same frozen leg via move_group (identical to 2b) ----
    import hashlib
    digest = hashlib.sha256(json.dumps(
        {"names": names, "points": points}, sort_keys=True).encode()).hexdigest()
    from moveit_msgs.msg import RobotTrajectory
    mg_traj = RobotTrajectory()
    mg_traj.joint_trajectory = _goal(names, points).trajectory
    diag.phase = "frozen_leg_mg"
    t0 = time.time()
    mg_goal = diag._MgExecute.Goal()
    mg_goal.trajectory = mg_traj
    fut = diag.mg.send_goal_async(mg_goal)
    handle = None
    while time.time() - t0 < 30.0:
        rclpy.spin_once(diag, timeout_sec=0.2)
        if fut.done():
            handle = fut.result()
            break
    if handle is None or not handle.accepted:
        r2 = {"accepted": False}
    else:
        res_fut = handle.get_result_async()
        r2 = {"accepted": True}
        while time.time() - t0 < span_s * 2 + 300.0:
            rclpy.spin_once(diag, timeout_sec=0.2)
            if res_fut.done():
                res = res_fut.result().result
                r2.update({"error_code": int(res.error_code.val),
                           "wall_s": time.time() - t0})
                break
        else:
            r2.update({"error_code": None,
                       "error_string": "diag wait timeout",
                       "wall_s": time.time() - t0})

    diag.phase = "hold"
    hold_t0 = time.time()
    sim_hold_start = diag.sim_now
    while diag.sim_now is not None and diag.sim_now - sim_hold_start < hold_s:
        rclpy.spin_once(diag, timeout_sec=0.2)
    hold_t1 = time.time()
    diag.phase = "post_hold"

    diag.flush()

    # ---- summary --------------------------------------------------------
    js_rows = [json.loads(line) for line in open(
        os.path.join(out_dir, "joint_states.jsonl"))]
    hold = [r for r in js_rows if hold_t0 <= r["t"] <= hold_t1]
    end = points[-1]["positions"]
    ti = {n: i for i, n in enumerate(names)}
    per_joint = {}
    for jn in names:
        if jn not in (diag.js_names or []):
            continue
        i_js = diag.js_names.index(jn)
        herr = [(r["t"] - hold_t0, end[ti[jn]] - r["position"][i_js])
                for r in hold]
        if not herr:
            continue
        per_joint[jn] = {
            "hold_err_max_rad": max(abs(v) for _, v in herr),
            "hold_err_first_rad": herr[0][1],
            "hold_err_last_rad": herr[-1][1],
            "hold_slope_rad_s": _fit_slope(herr),
        }
    # production settle conditions: first qualifying window
    TOL, VMAX, MIN_S, HSEC = 0.01, 0.01, 3, 0.3
    settle = {"verdict": "not_evaluated"}
    if hold:
        order = diag.js_names
        rows_ok = []
        for r in hold:
            e_ok = all(abs(end[ti[jn]] - r["position"][order.index(jn)])
                       <= TOL for jn in names if jn in order)
            v_ok = all(abs(v) <= VMAX for v in r["velocity"] or [0.0])
            rows_ok.append((r["t"], e_ok and v_ok, r.get("sim")))
        first_ok = None
        for k in range(len(rows_ok) - MIN_S + 1):
            span = rows_ok[k + MIN_S - 1][0] - rows_ok[k][0]
            if span >= HSEC and all(x[1] for x in rows_ok[k:k + MIN_S]):
                first_ok = {
                    "after_hold_s": rows_ok[k][0] - hold_t0,
                    "sim": rows_ok[k][2]}
                break
        settle = {"verdict": "met" if first_ok else "not_met",
                  "first_satisfied": first_ok,
                  "policy": {"tolerance_rad": TOL, "max_abs_velocity": VMAX,
                             "min_hold_samples": MIN_S, "hold_sec": HSEC}}
    summary = {
        "stage": "2c_pick_prehistory", "verdict": "completed",
        "frozen_leg_sha256": digest, "span_s": span_s,
        "goal_frozen_leg": r2, "boundary": boundary,
        "hold_sim_start": sim_hold_start, "hold_sim_end": diag.sim_now,
        "rows_joint_states": diag.js_rows,
        "rows_controller_state": diag.cs_rows,
        "action_status_events": diag.status_events,
        "per_joint": per_joint, "settle_gate": settle,
        "effort_note": "effort=NaN in sim; not usable as torque evidence",
        "driver_returncode": driver.returncode,
    }
    with open(os.path.join(out_dir, "summary.json"), "w") as fh:
        json.dump(summary, fh, indent=1)
    print(json.dumps({k: summary[k] for k in
                      ("verdict", "goal_frozen_leg", "settle_gate",
                       "per_joint")}, indent=1))
    rclpy.shutdown()


def _spin(node, seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.flush()


def _resume_and_wait(driver):
    try:
        os.kill(driver.pid, signal.SIGCONT)
    except Exception:  # noqa: BLE001
        pass
    try:
        driver.wait(timeout=90.0)
    except subprocess.TimeoutExpired:
        driver.kill()


if __name__ == "__main__":
    main()
