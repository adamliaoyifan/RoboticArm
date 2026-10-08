#!/usr/bin/env python3
"""Align controller reference, controller feedback and independent joint
states for every executed leg, and answer three questions per leg.

Q1  During motion, does controller ``feedback`` stay ~= ``reference`` while the
    independent ``/joint_states`` lag?
Q2  Does action success coincide with the END OF THE REFERENCE trajectory, or
    with the ACTUAL joints arriving at the last knot?
Q3  Peak actual-vs-reference error, error at the success instant, and time
    from success to convergence.

Inputs from one run directory: ``bag/`` (``/clock``, ``/joint_states``,
``/elfin_arm_controller/controller_state``), ``launch.log`` (controller
``Received new action goal`` / ``Goal reached`` lines, wall clock) and
``trials.jsonl`` (``extras.submitted_legs``: exact submitted trajectory and
submit/result wall times). Wall clock is mapped to sim time through the
``/clock`` messages' bag receive times; the mapping error is reported.

Convergence is decided from positions only, never from the velocity field: all
six joints within ``--tolerance`` of the last knot AND the per-joint span
(max-min) over a ``--hold`` window below ``--span-limit``. Span, not net
displacement, so back-and-forth motion with zero net change is not accepted.

  handoff_reference_analysis.py --run DIR [--out FILE]
"""
import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

JOINTS = tuple("elfin_joint%d" % i for i in range(1, 7))
TOLERANCE_RAD = 0.01
HOLD_SEC = 0.3
SPAN_LIMIT_RAD = 0.003  # 0.01 rad/s over the 0.3 s hold
REFERENCE_REACHED_RAD = 1e-3
CONTROLLER_STATE_TOPIC = "/elfin_arm_controller/controller_state"


def _order(names, values):
    index = {n: i for i, n in enumerate(names)}
    return [values[index[n]] for n in JOINTS]


def interp_rows(t_query, t_src, q_src):
    """Column-wise linear interpolation of an (N, 6) array."""
    t_src = np.asarray(t_src, dtype=float)
    q_src = np.asarray(q_src, dtype=float)
    return np.column_stack([
        np.interp(t_query, t_src, q_src[:, j]) for j in range(q_src.shape[1])])


def converged_after(t, q, start, last_knot, tolerance=TOLERANCE_RAD,
                    hold=HOLD_SEC, span_limit=SPAN_LIMIT_RAD, until=None):
    """First time >= ``start`` the arm is at ``last_knot`` and still.

    Returns ``(t_converged_or_None, detail)``. ``q`` is (N, 6) actual.
    """
    t = np.asarray(t, dtype=float)
    q = np.asarray(q, dtype=float)
    last_knot = np.asarray(last_knot, dtype=float)
    stop = t[-1] if until is None else min(until, t[-1])
    idx = np.where((t >= start) & (t <= stop))[0]
    for k in idx:
        window = np.where((t >= t[k]) & (t <= t[k] + hold))[0]
        if t[k] + hold > stop or len(window) < 3:
            break
        seg = q[window]
        near = np.abs(seg - last_knot).max()
        span = (seg.max(axis=0) - seg.min(axis=0)).max()
        if near <= tolerance and span <= span_limit:
            return float(t[k]), {"max_dist_to_last_knot": float(near),
                                 "max_span": float(span)}
    return None, {"searched_until": float(stop)}


def analyze_leg(ctrl_t, ref, fb, err, js_t, js_q, accept_t, success_t,
                traj_t_last, last_knot, next_accept_t=None):
    """Metrics for one executed leg. All arrays in sim seconds / radians."""
    ctrl_t = np.asarray(ctrl_t, dtype=float)
    ref = np.asarray(ref, dtype=float)
    fb = np.asarray(fb, dtype=float)
    err = np.asarray(err, dtype=float)
    js_t = np.asarray(js_t, dtype=float)
    js_q = np.asarray(js_q, dtype=float)
    last_knot = np.asarray(last_knot, dtype=float)
    ref_end_t = accept_t + traj_t_last

    in_motion = (ctrl_t >= accept_t) & (ctrl_t <= success_t)
    fb_ref = np.abs(fb - ref)[in_motion].max() if in_motion.any() else None
    err_reported = np.abs(err)[in_motion].max() if in_motion.any() else None

    js_in = (js_t >= accept_t) & (js_t <= success_t)
    act_at_js = js_q[js_in]
    ref_at_js = interp_rows(js_t[js_in], ctrl_t, ref)
    diff = np.abs(act_at_js - ref_at_js)
    if len(diff):
        flat = int(np.argmax(diff.max(axis=1)))
        act_ref_max = float(diff.max())
        act_ref_max_t = float(js_t[js_in][flat])
        act_ref_max_joint = JOINTS[int(np.argmax(diff[flat]))]
    else:
        act_ref_max = act_ref_max_t = act_ref_max_joint = None

    ref_at_success = interp_rows([success_t], ctrl_t, ref)[0]
    act_at_success = interp_rows([success_t], js_t, js_q)[0]
    reached = np.where(
        (ctrl_t >= accept_t)
        & (np.abs(ref - last_knot).max(axis=1) <= REFERENCE_REACHED_RAD))[0]
    ref_reached_t = float(ctrl_t[reached[0]]) if len(reached) else None

    until = next_accept_t
    t_conv, conv_detail = converged_after(
        js_t, js_q, success_t, last_knot, until=until)
    return {
        "accept_sim": accept_t,
        "success_sim": success_t,
        "duration_sec": round(success_t - accept_t, 3),
        "trajectory_t_last": traj_t_last,
        "reference_end_sim": ref_end_t,
        "q1_max_feedback_minus_reference_rad": _r(fb_ref),
        "q1_max_reported_error_rad": _r(err_reported),
        "q1_max_actual_minus_reference_rad": _r(act_ref_max),
        "q1_max_actual_minus_reference_at_sim": _r(act_ref_max_t),
        "q1_max_actual_minus_reference_joint": act_ref_max_joint,
        "q2_success_minus_reference_end_sec": _r(success_t - ref_end_t),
        "q2_reference_first_at_last_knot_sim": _r(ref_reached_t),
        "q2_reference_at_success_minus_last_knot_rad": _r(
            np.abs(ref_at_success - last_knot).max()),
        "q2_actual_at_success_minus_last_knot_rad": _r(
            np.abs(act_at_success - last_knot).max()),
        "q3_actual_minus_reference_at_success_rad": _r(
            np.abs(act_at_success - ref_at_success).max()),
        "q3_converged_sim": _r(t_conv),
        "q3_success_to_converged_sec": _r(
            None if t_conv is None else t_conv - success_t),
        "q3_convergence_detail": conv_detail,
        "q3_worst_joint_at_success": JOINTS[int(np.argmax(
            np.abs(act_at_success - last_knot)))],
    }


def velocity_reliability(js_t, js_q, js_v, start, end, baseline=0.2):
    """Reported joint velocity vs the position finite difference.

    ``false_still``: reported |v| <= 0.01 rad/s while the arm moves faster than
    0.02 rad/s by position. ``false_moving``: reported |v| > 0.01 rad/s while
    positions move less than 0.003 rad/s (a wrongful rejection under the
    velocity criterion). Recorded only; the criteria are not changed here.
    """
    js_t = np.asarray(js_t, dtype=float)
    js_q = np.asarray(js_q, dtype=float)
    js_v = np.asarray(js_v, dtype=float)
    idx = np.where((js_t >= start) & (js_t <= end))[0]
    n = 0
    false_still = false_moving = 0
    worst_reported = worst_fd = 0.0
    for k in idx:
        j = np.searchsorted(js_t, js_t[k] + baseline)
        if j >= len(js_t) or j == k:
            continue
        fd = np.abs(js_q[j] - js_q[k]).max() / (js_t[j] - js_t[k])
        rv = np.abs(js_v[k]).max()
        n += 1
        worst_reported = max(worst_reported, rv)
        worst_fd = max(worst_fd, fd)
        if rv <= 0.01 and fd > 0.02:
            false_still += 1
        if rv > 0.01 and fd < 0.003:
            false_moving += 1
    return {
        "samples": n,
        "max_reported_abs_velocity": _r(worst_reported),
        "max_position_finite_difference": _r(worst_fd),
        "false_still_fraction": _r(false_still / n) if n else None,
        "false_moving_fraction": _r(false_moving / n) if n else None,
    }


def _r(value, digits=5):
    return None if value is None else round(float(value), digits)


# --------------------------------------------------------------------- I/O

def read_bag(run):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from control_msgs.msg import JointTrajectoryControllerState
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(run / "bag"), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"))
    clock_wall, clock_sim = [], []
    js_t, js_q, js_v = [], [], []
    ct, ref, fb, err = [], [], [], []
    bad_names = 0
    while reader.has_next():
        topic, data, wall_ns = reader.read_next()
        if topic == "/clock":
            msg = deserialize_message(data, Clock)
            clock_wall.append(wall_ns * 1e-9)
            clock_sim.append(msg.clock.sec + msg.clock.nanosec * 1e-9)
        elif topic == "/joint_states":
            msg = deserialize_message(data, JointState)
            if set(JOINTS) <= set(msg.name):
                js_t.append(msg.header.stamp.sec
                            + msg.header.stamp.nanosec * 1e-9)
                js_q.append(_order(list(msg.name), list(msg.position)))
                js_v.append(_order(list(msg.name), list(msg.velocity))
                            if len(msg.velocity) == len(msg.name)
                            else [float("nan")] * 6)
        elif topic == CONTROLLER_STATE_TOPIC:
            msg = deserialize_message(data, JointTrajectoryControllerState)
            names = list(msg.joint_names)
            if not set(JOINTS) <= set(names):
                bad_names += 1
                continue
            ct.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            ref.append(_order(names, list(msg.reference.positions)))
            fb.append(_order(names, list(msg.feedback.positions)))
            err.append(_order(names, list(msg.error.positions)))
    return {
        "clock_wall": np.array(clock_wall), "clock_sim": np.array(clock_sim),
        "js_t": np.array(js_t), "js_q": np.array(js_q), "js_v": np.array(js_v),
        "ct": np.array(ct), "ref": np.array(ref), "fb": np.array(fb),
        "err": np.array(err), "controller_bad_name_messages": bad_names,
    }


def read_controller_log(run):
    """``[(wall, kind)]`` with kind in {"accept", "success"}."""
    pattern = re.compile(
        r"\[ign gazebo-\d+\] \[INFO\] \[(\d+\.\d+)\] \[elfin_arm_controller\]: "
        r"(Received new action goal|Goal reached, success)")
    events = []
    with open(run / "launch.log", errors="ignore") as handle:
        for line in handle:
            match = pattern.search(line)
            if match:
                kind = "accept" if match.group(2).startswith(
                    "Received") else "success"
                events.append((float(match.group(1)), kind))
    return events


def read_submitted_legs(run):
    legs = []
    with open(run / "trials.jsonl") as handle:
        for line in handle:
            row = json.loads(line)
            legs.extend((row.get("extras") or {}).get("submitted_legs") or [])
    return legs


def wall_to_sim(bag, wall):
    return float(np.interp(wall, bag["clock_wall"], bag["clock_sim"]))


def analyse_run(run):
    bag = read_bag(run)
    events = read_controller_log(run)
    legs = read_submitted_legs(run)
    out = {"run": str(run), "legs": [], "notes": []}
    if not len(bag["ct"]):
        out["notes"].append("controller_state absent from the bag")
        return out
    # A leg's controller goal is the accept inside [submit_wall, result_wall];
    # PlanMotionV2 may hand the controller more than one goal per leg only if
    # the executor splits, which is reported rather than assumed away.
    accepts = [w for w, k in events if k == "accept"]
    successes = [w for w, k in events if k == "success"]
    for leg in legs:
        traj = leg.get("trajectory")
        inside = [a for a in accepts
                  if leg["submit_wall"] - 0.5 <= a <= leg["result_wall"] + 0.5]
        if not traj or not traj.get("points") or len(inside) != 1:
            out["legs"].append({
                "name": leg["name"], "skipped": True,
                "reason": "no trajectory" if not traj else
                "%d controller goals inside the leg window" % len(inside)})
            continue
        accept_wall = inside[0]
        later = [s for s in successes if s >= accept_wall]
        if not later:
            out["legs"].append({"name": leg["name"], "skipped": True,
                                "reason": "no success event after accept"})
            continue
        success_wall = later[0]
        next_accepts = [a for a in accepts if a > success_wall]
        names = traj["joint_names"]
        last_q = _order(names, traj["points"][-1]["q"])
        t_last = float(traj["points"][-1]["t"])
        accept_t = wall_to_sim(bag, accept_wall)
        success_t = wall_to_sim(bag, success_wall)
        next_t = wall_to_sim(bag, next_accepts[0]) if next_accepts else None
        row = analyze_leg(
            bag["ct"], bag["ref"], bag["fb"], bag["err"],
            bag["js_t"], bag["js_q"], accept_t, success_t, t_last, last_q,
            next_accept_t=next_t)
        row["name"] = leg["name"]
        end_t = row["q3_converged_sim"] if row["q3_converged_sim"] else (
            next_t if next_t else success_t + 5.0)
        row["velocity_field_vs_position"] = velocity_reliability(
            bag["js_t"], bag["js_q"], bag["js_v"], success_t, end_t)
        row["n_trajectory_points"] = traj["n_points"]
        row["next_goal_after_success_sec"] = _r(
            None if next_t is None else next_t - success_t)
        out["legs"].append(row)
    out["clock_map_points"] = int(len(bag["clock_wall"]))
    out["controller_state_messages"] = int(len(bag["ct"]))
    out["joint_states_messages"] = int(len(bag["js_t"]))
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True)
    parser.add_argument("--out", default="")
    args = parser.parse_args(argv)
    result = analyse_run(Path(args.run))
    text = json.dumps(result, indent=2, sort_keys=True)
    if args.out:
        Path(args.out).write_text(text + "\n")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
