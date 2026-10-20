#!/usr/bin/env python3
"""Stage 2a hold diagnosis: carried box + production follow loop.

Adds to stage 1 exactly ONE mechanism layer: a recess-collision box
(production SDF builder, carry mask 0xFE) spawned at the cup with the
frozen leg's start posture, driven by the PRODUCTION SimVacuumBackend
follow (verbatim class, TF panel pose at 30 Hz via the bridged
/world/<w>/set_pose service). No vacuum gate, no MoveIt scene attach,
no driver: isolates "kinematically followed physical box" as the
mechanism variable. If the r10/r11 drift reproduces here, the source
is the follow/box system, not the planning stack.

Captures: /joint_states (incl. effort), controller_state, box contact
sensor (/obs/box_contact via `ign topic -e`, started AFTER spawn),
follow tick outcomes, goal results, 60 s hold fit per joint.
"""
import json
import math
import os
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
SIZE_WDH = (0.35, 0.53, 0.22)
MASS_KG = 5.67
LID_OFFSET = 0.011033
PRESS_IN_M = 0.01  # yaml header: attach sits ~1 cm into the cube


def _quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def _quat_conj(q):
    return [-q[0], -q[1], -q[2], q[3]]


def _quat_to_matrix(quat):
    x, y, z, w = quat
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


class GzSetPoseClient(object):
    """Verbatim adaptation of vacuum_controller_node.GzSetPoseClient."""

    def __init__(self, node):
        from ros_gz_interfaces.msg import Entity
        from ros_gz_interfaces.srv import SetEntityPose
        self._Entity = Entity
        self._SetEntityPose = SetEntityPose
        self._client = node.create_client(
            SetEntityPose, "/world/%s/set_pose" % WORLD)
        self._ready = False
        self._probed = False

    def set_model_pose(self, model_name, xyz, quat, timeout_sec=0.05):
        if not self._ready:
            budget = 5.0 if not self._probed else 0.0
            if not self._client.wait_for_service(timeout_sec=budget):
                self._probed = True
                return False, "set_pose service unavailable"
            self._ready = True
        elif not self._client.service_is_ready():
            return False, "set_pose service unavailable"
        event = threading.Event()
        request = self._SetEntityPose.Request()
        request.entity.name = model_name
        request.entity.type = self._Entity.MODEL
        request.pose.position.x = float(xyz[0])
        request.pose.position.y = float(xyz[1])
        request.pose.position.z = float(xyz[2])
        request.pose.orientation.x = float(quat[0])
        request.pose.orientation.y = float(quat[1])
        request.pose.orientation.z = float(quat[2])
        request.pose.orientation.w = float(quat[3])
        future = self._client.call_async(request)
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(float(timeout_sec)):
            return False, "set_pose timeout"
        response = future.result()
        if response is None or not response.success:
            return False, "set_pose rejected"
        return True, ""


class NoopScene(object):
    """Planning-scene attach is a MoveIt-side concern; stage 2a has no
    MoveIt interaction. The backend's scene calls are accepted no-op."""

    def attach_pickup_box(self, *_a, **_k):
        return True, ""

    def detach_and_remove(self, *_a, **_k):
        return True, ""


def _fit_slope(samples):
    n = len(samples)
    if n < 3:
        return None
    t0 = samples[0][0]
    ts = [t - t0 for t, _ in samples]
    vs = [v for _, v in samples]
    tbar = sum(ts) / n
    vbar = sum(vs) / n
    num = sum((t - tbar) * (v - vbar) for t, v in zip(ts, vs))
    den = sum((t - tbar) ** 2 for t in ts)
    return num / den if den else None


class Diag2(Node):
    def __init__(self, out_dir):
        super().__init__("hold_diag2b")
        self.out = out_dir
        self.js = open(os.path.join(out_dir, "joint_states.jsonl"), "a")
        self.cs = open(os.path.join(out_dir, "controller_state.jsonl"), "a")
        self.js_rows = 0
        self.cs_rows = 0
        self.effort_present = None
        self.clock_rows = 0
        self.sim_now = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 200)
        self.create_subscription(
            JointTrajectoryControllerState,
            "/elfin_arm_controller/controller_state", self._on_cs, 200)
        from rosgraph_msgs.msg import Clock
        from action_msgs.msg import GoalStatusArray
        self.create_subscription(Clock, "/clock", self._on_clock, 200)
        # Both action layers' status streams (plain-topic view of the
        # actions) — upper move_group layer and controller FJT layer.
        self.status_events = []
        self.create_subscription(
            GoalStatusArray,
            "/execute_trajectory/_action/status",
            lambda m: self._on_status("movegroup", m), 100)
        self.create_subscription(
            GoalStatusArray,
            "/elfin_arm_controller/follow_joint_trajectory/_action/status",
            lambda m: self._on_status("controller", m), 100)
        self.ac = ActionClient(
            self, FollowJointTrajectory,
            "/elfin_arm_controller/follow_joint_trajectory")
        from moveit_msgs.action import ExecuteTrajectory as MgExecute
        self._MgExecute = MgExecute
        self.mg = ActionClient(self, MgExecute, "/execute_trajectory")
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.gz = GzSetPoseClient(self)

    def _on_clock(self, msg):
        self.clock_rows += 1
        self.sim_now = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_status(self, layer, msg):
        for st in msg.status_list:
            try:
                gid = st.goal_info.goal_id.uuid.tobytes().hex()[:8]
            except Exception:  # noqa: BLE001 - diag only
                gid = "?"
            self.status_events.append({
                "t": time.time(), "sim": self.sim_now, "layer": layer,
                "status": int(st.status), "goal_id": gid,
            })

    def _on_js(self, msg):
        self.js_rows += 1
        effort = list(msg.effort or [])
        if self.effort_present is None and effort:
            self.effort_present = True
        self.js.write(json.dumps({
            "t": time.time(), "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity), "effort": effort}) + "\n")

    def _on_cs(self, msg):
        self.cs_rows += 1
        ref = getattr(msg, "reference", None)
        fb = getattr(msg, "feedback", None)
        out = getattr(msg, "output", None)
        self.cs.write(json.dumps({
            "t": time.time(),
            "reference.positions": list(ref.positions) if ref else None,
            "feedback.positions": list(fb.positions) if fb else None,
            "output.positions": list(out.positions) if out else None}) + "\n")

    def panel_pose(self):
        try:
            tr = self.tf.lookup_transform(
                "world", PANEL_FRAME, rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001 - diag only
            return None, repr(exc)[:80]
        t = tr.transform
        return ([t.translation.x, t.translation.y, t.translation.z],
                [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]), None

    def flush(self):
        self.js.flush()
        self.cs.flush()


def _spin_seconds(node, seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.flush()


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
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    return goal


def _send_wait(node, goal, timeout_s, label):
    t0 = time.time()
    fut = node.ac.send_goal_async(goal)
    handle = None
    while time.time() - t0 < 30.0:
        rclpy.spin_once(node, timeout_sec=0.2)
        if fut.done():
            handle = fut.result()
            break
    if handle is None or not handle.accepted:
        return {"label": label, "accepted": False}
    res_fut = handle.get_result_async()
    while time.time() - t0 < timeout_s:
        rclpy.spin_once(node, timeout_sec=0.2)
        if res_fut.done():
            res = res_fut.result().result
            return {"label": label, "accepted": True,
                    "error_code": int(res.error_code),
                    "error_string": str(res.error_string),
                    "wall_s": time.time() - t0}
    return {"label": label, "accepted": True, "error_code": None,
            "error_string": "diag wait timeout", "wall_s": time.time() - t0}


def main():
    out_dir, traj_path = sys.argv[1], sys.argv[2]
    hold_s = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0
    doc = json.load(open(traj_path))
    names = doc["joint_names"]
    points = doc["points"]
    start = points[0]["positions"]
    span_s = points[-1]["time_from_start_ns"] / 1e9

    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "luggage_gazebo", "scripts"))
    rclpy.init()
    diag = Diag2(out_dir)
    if not diag.ac.wait_for_server(timeout_sec=60.0):
        print(json.dumps({"fatal": "action server unavailable"}))
        return 4
    _spin_seconds(diag, 3.0)
    if diag.cs_rows == 0 or diag.js_rows == 0:
        print(json.dumps({"fatal": "captures not flowing"}))
        return 5

    # 1. positioning to the frozen start posture (harness step; 20 s so
    # the move is feasible from ANY prior posture — the frozen leg stays
    # the only execution-path variable)
    r1 = _send_wait(diag, _goal(names, [{
        "positions": start, "velocities": [], "accelerations": [],
        "time_from_start_ns": 20_000_000_000}]), 90.0, "positioning")
    _spin_seconds(diag, 5.0)

    # 2. production SDF + spawn at the cup
    from luggage_description.suitcase_visual import pickup_visual_sdf
    sdf = pickup_visual_sdf(
        MODEL, SIZE_WDH, MASS_KG, "suitcase_loafbrr",
        static=True, contact_topic="/obs/box_contact",
        collide_bitmask=0xFE, collision_kind="box_recess",
        lid_offset=LID_OFFSET)
    with open(os.path.join(out_dir, "carry_box.sdf"), "w") as fh:
        fh.write(sdf)
    panel, err = diag.panel_pose()
    if panel is None:
        print(json.dumps({"fatal": "panel pose unavailable: %s" % err}))
        return 6
    px, pq = panel
    # TRUE production relation, measured from r11 evidence (no FK of the
    # robot base placement is needed): tf_trace before_transit cup at
    # [0.565, 0.592, 1.427], pose probe box at [0.559, 0.600, 1.320]
    # RPY [0, 0, pi/2] at the same instant. Same launch => same base
    # placement => the world-frame offset/yaw transfer exactly.
    BOX_OFFSET_WORLD = [-0.00605, 0.00764, -0.10749]
    half = math.sin(math.pi / 4)
    spawn_xyz = [px[i] + BOX_OFFSET_WORLD[i] for i in range(3)]
    spawn_quat = [0.0, 0.0, half, half]  # world yaw pi/2, level

    from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
    from std_srvs.srv import SetBool
    spawn_cli = diag.create_client(SpawnEntity, "/world/%s/create" % WORLD)
    spawn_cli.wait_for_service(timeout_sec=10.0)
    req = SpawnEntity.Request()
    req.entity_factory.name = MODEL
    req.entity_factory.sdf = sdf
    req.entity_factory.pose.position.x = spawn_xyz[0]
    req.entity_factory.pose.position.y = spawn_xyz[1]
    req.entity_factory.pose.position.z = spawn_xyz[2]
    req.entity_factory.pose.orientation.x = spawn_quat[0]
    req.entity_factory.pose.orientation.y = spawn_quat[1]
    req.entity_factory.pose.orientation.z = spawn_quat[2]
    req.entity_factory.pose.orientation.w = spawn_quat[3]
    fut = spawn_cli.call_async(req)
    while not fut.done():
        rclpy.spin_once(diag, timeout_sec=0.2)
    print(json.dumps({"spawned": bool(fut.result() and fut.result().success)}))
    time.sleep(0.5)

    # p04 carry filter (production set at attach)
    filt = diag.create_client(
        SetBool, "/pickup_box_spawner/set_p04_carry_filter")
    filt.wait_for_service(timeout_sec=10.0)
    fr = filt.call_async(SetBool.Request(data=True))
    while not fr.done():
        rclpy.spin_once(diag, timeout_sec=0.2)

    # contact capture: BOUNDED samples only. The unbounded echo flooded
    # 9.6M protobuf lines in minutes and collapsed RTF (invalidating the
    # first 2a attempt) — the sensor fires nearly every physics step.
    # Phase A: short partner-ID sample right after spawn; the full-rate
    # HOLD-window capture starts after the leg (the decisive window).
    contact_file = os.path.join(out_dir, "box_contact_legphase.txt")
    contact_proc = subprocess.Popen(
        ["ign", "topic", "-e", "-t", "/obs/box_contact", "--num", "300"],
        stdout=open(contact_file, "w"), stderr=subprocess.DEVNULL)

    # 3. freeze the physical relation (production module, observed pose)
    from luggage_planning.sim_physical_attachment import (
        freeze_physical_relation)
    relation = freeze_physical_relation(
        MODEL, 0, {"position": px, "quaternion_xyzw": pq},
        read_identity=lambda: (MODEL, 0))
    with open(os.path.join(out_dir, "physical_relation.json"), "w") as fh:
        json.dump(relation, fh, indent=1)

    # 4. production backend + 30 Hz follow thread
    from luggage_planning.vacuum_backend import SimVacuumBackend
    backend = SimVacuumBackend(
        diag.gz, NoopScene(), follow_rate_hz=30.0,
        carry_filter=lambda on: True)
    ok, msg = backend.attach({
        "attach_record": {
            "generation": 0,
            "box_pose": relation["physical_pose"],
            "size_wdh": list(SIZE_WDH),
        },
        "physical_relation": relation,
        "model_name": MODEL,
        "tool_pose": {"position": px, "quaternion_xyzw": pq},
        "attach_link": PANEL_FRAME,
    })
    print(json.dumps({"backend_attach": [ok, msg]}))
    follow_stats = {"ticks": 0, "ok": 0, "fail": 0, "first_fail": None}
    follow_stop = threading.Event()

    def follow_loop():
        period = 1.0 / 30.0
        while not follow_stop.is_set():
            panel2, _err = diag.panel_pose()
            if panel2 is not None:
                ok2, msg2 = backend.follow_step(panel2[0], panel2[1])
                follow_stats["ticks"] += 1
                if ok2:
                    follow_stats["ok"] += 1
                else:
                    follow_stats["fail"] += 1
                    if follow_stats["first_fail"] is None:
                        follow_stats["first_fail"] = msg2
            follow_stop.wait(period)

    follow_thread = threading.Thread(target=follow_loop, daemon=True)
    follow_thread.start()
    _spin_seconds(diag, 2.0)

    # 5. baseline at start (box carried)
    pre_t0 = time.time()
    _spin_seconds(diag, 10.0)
    pre_t1 = time.time()

    # 6. frozen leg verbatim — THE SINGLE VARIABLE: submitted through
    # move_group /execute_trajectory, goal construction copied from the
    # production motion_executor "timed joint path" branch (RobotTrajectory
    # with joint_trajectory = the 323 certified knots verbatim).
    import hashlib
    digest = hashlib.sha256(json.dumps(
        {"names": names, "points": points}, sort_keys=True).encode()).hexdigest()
    from moveit_msgs.msg import RobotTrajectory
    mg_traj = RobotTrajectory()
    mg_traj.joint_trajectory = _goal(names, points).trajectory
    from luggage_planning.timed_execution_budget import timed_execution_wait
    from luggage_planning.motion_executor import execute_wait_timeout_sec
    prod_wait = min(
        timed_execution_wait(mg_traj.joint_trajectory.points, 60.0),
        execute_wait_timeout_sec(60.0, mg_traj.joint_trajectory.points))
    if not diag.mg.wait_for_server(timeout_sec=15.0):
        print(json.dumps({"fatal": "execute_trajectory server unavailable"}))
        return 7
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
        r2 = {"label": "frozen_leg_mg", "accepted": False}
    else:
        res_fut = handle.get_result_async()
        r2 = {"label": "frozen_leg_mg", "accepted": True}
        while time.time() - t0 < span_s * 2 + 300.0:
            rclpy.spin_once(diag, timeout_sec=0.2)
            if res_fut.done():
                res = res_fut.result().result
                r2.update({
                    "error_code": int(res.error_code.val),
                    "wall_s": time.time() - t0})
                break
        else:
            r2.update({"error_code": None, "error_string": "diag wait timeout",
                       "wall_s": time.time() - t0})

    # 7. hold with follow still running — 60 SIM seconds via /clock;
    # HOLD-window contact capture (bounded)
    hold_contact_file = os.path.join(out_dir, "box_contact_hold.txt")
    hold_contact_proc = subprocess.Popen(
        ["ign", "topic", "-e", "-t", "/obs/box_contact", "--num", "300"],
        stdout=open(hold_contact_file, "w"), stderr=subprocess.DEVNULL)
    hold_t0 = time.time()
    sim_hold_start = diag.sim_now
    while diag.sim_now is not None and diag.sim_now - sim_hold_start < hold_s:
        rclpy.spin_once(diag, timeout_sec=0.2)
    hold_t1 = time.time()

    follow_stop.set()
    follow_thread.join(timeout=2.0)
    contact_proc.terminate()
    hold_contact_proc.terminate()
    backend.detach({})
    filt2 = filt.call_async(SetBool.Request(data=False))
    while not filt2.done():
        rclpy.spin_once(diag, timeout_sec=0.2)
    del_cli = diag.create_client(DeleteEntity, "/world/%s/delete" % WORLD)
    del_cli.wait_for_service(timeout_sec=10.0)
    from ros_gz_interfaces.msg import Entity as GzEntity
    dreq = DeleteEntity.Request()
    dreq.entity.name = MODEL
    dreq.entity.type = GzEntity.MODEL
    del_cli.call_async(dreq)
    time.sleep(1.0)
    diag.flush()

    # 8. summary
    js_rows = [json.loads(line) for line in open(
        os.path.join(out_dir, "joint_states.jsonl"))]
    order = None
    for r in js_rows:
        if r.get("name"):
            order = r["name"]
            break
    traj_index = {n: i for i, n in enumerate(names)}  # trajectory order
    end_traj = points[-1]["positions"]

    def win(t0, t1):
        return [r for r in js_rows if t0 <= r["t"] <= t1]

    pre = win(pre_t0, pre_t1)
    hold = win(hold_t0, hold_t1)
    n_contact = 0
    n_contact_hold = 0
    for path, target in ((contact_file, "leg"), (hold_contact_file, "hold")):
        try:
            with open(path) as fh:
                count = sum(1 for line in fh if "collision1" in line)
            if target == "leg":
                n_contact = count
            else:
                n_contact_hold = count
        except FileNotFoundError:
            pass
    summary = {
        "stage": "replay_captured_fjt", "frozen_leg": os.path.basename(traj_path),
        "input_trajectory_sha256": digest,
        "production_wait_sec": prod_wait,
        "span_s": span_s, "goal_positioning": r1, "goal_frozen_leg": r2,
        "effort_present": diag.effort_present is True,
        "effort_note": "field presence only; physical meaning unvalidated",
        "rows_joint_states": diag.js_rows,
        "rows_controller_state": diag.cs_rows,
        "clock_rows": diag.clock_rows,
        "action_status_events": diag.status_events[-200:],
        "follow_stats": follow_stats,
        "contact_lines_matched": n_contact,
        "contact_lines_hold_window": n_contact_hold,
        "hold_sim_start": sim_hold_start,
        "hold_sim_end": diag.sim_now,
        "per_joint": {},
    }
    for jn in names:
        i_js = order.index(jn) if jn in order else None
        i_tr = traj_index[jn]
        if i_js is None:
            continue
        pre_s = [(r["t"] - pre_t0, r["position"][i_js]) for r in pre]
        hold_err = [(r["t"] - hold_t0, end_traj[i_tr] - r["position"][i_js])
                    for r in hold]
        half = len(hold_err) // 2 or 1
        summary["per_joint"][jn] = {
            "pre_slope_rad_s": _fit_slope(pre_s),
            "hold_err_first_rad": hold_err[0][1] if hold_err else None,
            "hold_err_last_rad": hold_err[-1][1] if hold_err else None,
            "hold_slope_rad_s": _fit_slope(hold_err),
            "hold_slope_first_half": _fit_slope(hold_err[:half]),
            "hold_slope_second_half": _fit_slope(hold_err[half:]),
        }
        summary["per_joint"][jn]["hold_err_max_rad"] = (
            max(abs(v) for _, v in hold_err) if hold_err else None)

    # Production settle gate (handoff policy, verbatim constants from
    # the r10/r11 handoff.json): tolerance 0.01 rad vs the next leg's
    # first point (continuity perfect -> the end target), |velocity|
    # <= 0.01 rad/s, >=3 samples sustained over 0.3 s. Evaluated on the
    # /joint_states stream (position+velocity) inside the hold window.
    TOL, VMAX, MIN_SAMPLES, HOLD_SEC = 0.01, 0.01, 3, 0.3
    if not hold or not all("velocity" in r for r in hold):
        summary["settle_gate"] = {"verdict": "not_evaluated",
                                  "reason": "no valid feedback rows"}
    else:
        rows_ok = []
        for r in hold:
            err_ok = all(
                abs(end_traj[traj_index[jn]] - r["position"][order.index(jn)])
                <= TOL for jn in names)
            vel_ok = all(abs(v) <= VMAX for v in r["velocity"] or [0.0])
            rows_ok.append((r["t"], err_ok and vel_ok))
        first_ok = None
        for k in range(len(rows_ok) - MIN_SAMPLES + 1):
            span = rows_ok[k + MIN_SAMPLES - 1][0] - rows_ok[k][0]
            if span >= HOLD_SEC and all(ok for _, ok in rows_ok[k:k + MIN_SAMPLES]):
                first_ok = rows_ok[k][0] - hold_t0
                break
        summary["settle_gate"] = {
            "verdict": "met" if first_ok is not None else "not_met",
            "satisfied_after_hold_s": first_ok,
            "policy": {"tolerance_rad": TOL, "max_abs_velocity": VMAX,
                       "min_hold_samples": MIN_SAMPLES,
                       "hold_sec": HOLD_SEC},
        }
    with open(os.path.join(out_dir, "summary.json"), "w") as fh:
        json.dump(summary, fh, indent=1)
    print(json.dumps({k: summary[k] for k in
                      ("goal_frozen_leg", "follow_stats",
                       "contact_lines_matched", "effort_present",
                       "per_joint")}, indent=1))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
