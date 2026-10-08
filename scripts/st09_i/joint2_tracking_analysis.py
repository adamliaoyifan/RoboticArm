#!/usr/bin/env python3
"""Offline joint-tracking analysis of one capture run (no simulation).

Built on ``handoff_reference_analysis`` (same bag / launch.log / trials.jsonl
inputs). For a chosen leg (default: the second ``transit`` leg) it separates

  stage 1  submitted trajectory -> controller ``reference``
  stage 2  controller ``reference`` -> actual joint position

then splits the leg into acceleration / near-constant speed / deceleration /
end hold, fits a per-joint delay, and reports the RESIDUAL error after delay
compensation (RMS, max, linear trend) per phase and per joint. It also checks
whether the shortfall depends on pose, direction or reference speed across all
legs, and how the end-hold error decays.

Labels: every number is a direct measurement from the bag unless the key says
``model``. The servo law ``v = K * (reference - position)`` is a *fitted
description*, not read from source.

  joint2_tracking_analysis.py --run DIR --out-dir DIR [--leg-index N]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicHermiteSpline

sys.path.insert(0, str(Path(__file__).resolve().parent))
import handoff_reference_analysis as base  # noqa: E402

JOINTS = base.JOINTS
SPEED_FRACTION_CRUISE = 0.9
TAU_GRID = np.arange(0.0, 2.0001, 0.005)
ERROR_FLOOR_RAD = 0.01          # |reference - position| above which a sample
                                # counts as "following demand"
LOW_SIGNAL_RAD = 0.01           # a joint whose reference moves less is flagged
SERVO_FIT_JOINTS = (0, 3, 5)    # joints 1, 4, 6: fit K on these only
PHASES = ("accel", "cruise", "decel", "hold")


# ----------------------------------------------------------------- helpers
def hermite_from_leg(leg):
    traj = leg["trajectory"]
    names = traj["joint_names"]
    order = [names.index(n) for n in JOINTS]
    t = np.array([p["t"] for p in traj["points"]], dtype=float)
    q = np.array([p["q"] for p in traj["points"]], dtype=float)[:, order]
    v = np.array([p["v"] for p in traj["points"]], dtype=float)[:, order]
    return t, q, v, [CubicHermiteSpline(t, q[:, j], v[:, j]) for j in range(6)]


def reference_at(spline_set, t_knots, q_knots, t_rel):
    """Hermite reference, held at the first/last knot outside the trajectory."""
    t_rel = np.asarray(t_rel, dtype=float)
    inside = np.clip(t_rel, t_knots[0], t_knots[-1])
    return np.column_stack([s(inside) for s in spline_set])


def velocity_centered(t, q, half=5):
    out = np.full_like(q, np.nan)
    out[half:-half] = (q[2 * half:] - q[:-2 * half]) / (
        t[2 * half:] - t[:-2 * half])[:, None]
    return out


def linear_slope(t, y):
    if len(t) < 3:
        return None
    return float(np.polyfit(t - t[0], y, 1)[0])


def rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else None


def r5(x):
    return None if x is None else round(float(x), 5)


# ------------------------------------------------------- leg identification
def leg_windows(run, bag):
    """One dict per submitted leg with sim-time anchors (goal-unique or not)."""
    events = base.read_controller_log(run)
    legs = base.read_submitted_legs(run)
    accepts = [w for w, k in events if k == "accept"]
    successes = [w for w, k in events if k == "success"]
    rows = []
    for index, leg in enumerate(legs):
        inside = [a for a in accepts
                  if leg["submit_wall"] - 0.5 <= a <= leg["result_wall"] + 0.5]
        row = {"index": index, "name": leg["name"], "leg": leg,
               "submit_sim": leg["submit_sim_stamp"],
               "result_sim": leg["result_sim_stamp"],
               "goals_in_window": len(inside)}
        if len(inside) == 1:
            later = [s for s in successes if s >= inside[0]]
            row["accept_sim"] = base.wall_to_sim(bag, inside[0])
            if later:
                row["success_sim"] = base.wall_to_sim(bag, later[0])
        rows.append(row)
    for a, b in zip(rows, rows[1:]):
        a["window_end"] = b["submit_sim"]
    rows[-1]["window_end"] = rows[-1]["result_sim"] + 1.5
    return rows


# ------------------------------------------------------------------ stage 1
def stage1(bag, row):
    leg = row["leg"]
    t_knots, q_knots, _, splines = hermite_from_leg(leg)
    ct, ref = bag["ct"], bag["ref"]
    lo = row["accept_sim"] - 0.05
    best = None
    for t0 in np.arange(lo, lo + 0.35, 0.002):
        m = (ct >= t0) & (ct <= t0 + t_knots[-1])
        pred = np.column_stack([s(ct[m] - t0) for s in splines])
        e = float(np.sqrt(np.mean((pred - ref[m]) ** 2)))
        if best is None or e < best[0]:
            best = (e, float(t0))
    t0 = best[1]
    m = (ct >= t0) & (ct <= t0 + t_knots[-1])
    pred = np.column_stack([s(ct[m] - t0) for s in splines])
    first = float(ct[ct >= row["accept_sim"]][0])
    return {
        "submit_to_accept_sec": r5(row["accept_sim"] - row["submit_sim"]),
        "accept_to_first_reference_sample_sec": r5(first - row["accept_sim"]),
        "reference_start_fit_minus_accept_sec": r5(t0 - row["accept_sim"]),
        "max_abs_reference_minus_hermite_rad": [
            r5(x) for x in np.abs(pred - ref[m]).max(axis=0)],
        "interpolation": "cubic Hermite on submitted q and v; linear would "
                         "differ by up to %.4f rad" % float(np.abs(
                             np.column_stack([
                                 np.interp(ct[m] - t0, t_knots, q_knots[:, j])
                                 for j in range(6)]) - ref[m]).max()),
        "t0_fit_sim": r5(t0),
        "trajectory_duration_sec": r5(t_knots[-1]),
    }, t0


# ------------------------------------------------------------------ phases
def phase_bounds(t_knots, splines, t0):
    grid = np.linspace(0.0, t_knots[-1], 4001)
    speed = np.linalg.norm(np.column_stack([s(grid, 1) for s in splines]),
                           axis=1)
    peak = speed.max()
    fast = np.where(speed >= SPEED_FRACTION_CRUISE * peak)[0]
    a, b = grid[fast[0]], grid[fast[-1]]
    return {
        "accel": (t0, t0 + a), "cruise": (t0 + a, t0 + b),
        "decel": (t0 + b, t0 + t_knots[-1]),
        "peak_path_speed_rad_s": float(peak),
    }


# ------------------------------------------------------------ delay fitting
def fit_delay(ct, fb, ref_fn, t_start, t_end):
    """Per-joint tau minimising rms(fb(t) - ref(t - tau)) over [start, end]."""
    m = (ct >= t_start) & (ct <= t_end)
    t = ct[m]
    out = []
    for j in range(6):
        errs = [rms(fb[m][:, j] - ref_fn(t - tau, j)) for tau in TAU_GRID]
        k = int(np.argmin(errs))
        out.append((float(TAU_GRID[k]), float(errs[k]),
                    bool(k == len(TAU_GRID) - 1)))
    return out


# --------------------------------------------- first-order lag vs pure delay
def simulate_first_order(t, ref_series, gain, x0, step=0.01):
    """Euler integration of x' = gain*(ref - x) sampled back onto ``t``."""
    grid = np.arange(t[0], t[-1] + step, step)
    ref_grid = np.interp(grid, t, ref_series)
    x = np.empty_like(grid)
    x[0] = x0
    for k in range(len(grid) - 1):
        x[k + 1] = x[k] + step * gain * (ref_grid[k] - x[k])
    return np.interp(t, grid, x)


def lag_model_comparison(bag, t0, t_end, ref_fn, taus):
    """Per joint: rms of the best pure delay vs rms of a first-order lag.

    A first-order lag (speed proportional to remaining error) and a pure
    transport delay both give "about 0.1 s" from a delay fit on a ramp; they
    differ in shape at motion start and end, which this comparison scores.
    """
    ct, fb = bag["ct"], bag["fb"]
    m = (ct >= t0) & (ct <= t_end)
    t = ct[m]
    out = {}
    for j in range(6):
        ref_series = ref_fn(t, j)
        best = None
        for gain in np.arange(0.2, 40.0, 0.05):
            sim = simulate_first_order(t, ref_series, gain, fb[m][0, j])
            e = rms(fb[m][:, j] - sim)
            if best is None or e < best[1]:
                best = (float(gain), float(e))
        out[JOINTS[j]] = {
            "pure_delay_tau_sec": r5(taus[j][0]),
            "pure_delay_rms_rad": r5(taus[j][1]),
            "first_order_gain_per_sec": r5(best[0]),
            "first_order_time_constant_sec": r5(1.0 / best[0]),
            "first_order_rms_rad": r5(best[1]),
            "better_model": "first_order" if best[1] < taus[j][1] else "delay",
        }
    return out


# ---------------------------------------------------------- phase residuals
def phase_table(bag, t0, bounds, hold_end, taus, ref_fn, common_tau):
    ct, fb = bag["ct"], bag["fb"]
    ranges = dict(bounds)
    ranges["hold"] = (bounds["decel"][1], hold_end)
    table = {}
    for phase in PHASES:
        a, b = ranges[phase]
        m = (ct >= a) & (ct < b)
        t = ct[m]
        row = {"t_start": r5(a), "t_end": r5(b), "samples": int(m.sum()),
               "joints": {}}
        for j in range(6):
            if not m.any():
                continue
            raw = fb[m][:, j] - ref_fn(t, j)
            fit = fb[m][:, j] - ref_fn(t - taus[j][0], j)
            com = fb[m][:, j] - ref_fn(t - common_tau, j)
            row["joints"][JOINTS[j]] = {
                "raw_rms": r5(rms(raw)), "raw_max": r5(np.abs(raw).max()),
                "fit_tau_rms": r5(rms(fit)), "fit_tau_max": r5(np.abs(fit).max()),
                "fit_tau_trend_rad_s": r5(linear_slope(t, fit)),
                "common_tau_rms": r5(rms(com)),
                "common_tau_max": r5(np.abs(com).max()),
                "common_tau_trend_rad_s": r5(linear_slope(t, com)),
                "common_tau_mean": r5(float(np.mean(com))),
            }
        table[phase] = row
    return table


# ----------------------------------------------- servo-law / dependence part
def servo_fit(bag, windows):
    """K in v = K*e fitted on joints 1,4,6 inside all analysed windows."""
    ct, fb, ref = bag["ct"], bag["fb"], bag["ref"]
    vel = velocity_centered(ct, fb)
    err = ref - fb
    num = den = 0.0
    for lo, hi in windows:
        m = (ct > lo) & (ct < hi) & np.isfinite(vel[:, 0])
        for j in SERVO_FIT_JOINTS:
            big = m & (np.abs(err[:, j]) > ERROR_FLOOR_RAD)
            num += float(np.sum(err[big, j] * vel[big, j]))
            den += float(np.sum(err[big, j] ** 2))
    return num / den if den else None


def delivered_ratio(bag, lo, hi, joint, gain):
    """Median/min of v*sign(e) / (K*|e|) where the joint is following demand."""
    ct, fb, ref = bag["ct"], bag["fb"], bag["ref"]
    vel = velocity_centered(ct, fb)
    err = (ref - fb)[:, joint]
    m = (ct > lo) & (ct < hi) & np.isfinite(vel[:, joint]) & (
        np.abs(err) > ERROR_FLOOR_RAD)
    if m.sum() < 5:
        return None
    ratio = vel[m, joint] * np.sign(err[m]) / (gain * np.abs(err[m]))
    return {"samples": int(m.sum()), "median": r5(np.median(ratio)),
            "p10": r5(np.percentile(ratio, 10)), "min": r5(ratio.min()),
            "max_abs_error_rad": r5(np.abs(err[m]).max()),
            "median_delivered_speed_rad_s": r5(np.median(
                np.abs(vel[m, joint]))),
            "median_model_demand_rad_s": r5(np.median(gain * np.abs(err[m]))),
            "q_start": r5(fb[np.where(m)[0][0], joint]),
            "q_end": r5(fb[np.where(m)[0][-1], joint])}


def per_leg_dependence(bag, rows, gain):
    out = []
    for row in rows:
        lo, hi = row["submit_sim"], row["window_end"]
        t_knots, q_knots, _, splines = hermite_from_leg(row["leg"])
        entry = {"index": row["index"], "name": row["name"],
                 "_t_lo": lo, "_t_hi": hi,
                 "duration_sec": r5(t_knots[-1]),
                 "ref_peak_speed_rad_s": [
                     r5(np.abs(s(np.linspace(0, t_knots[-1], 400), 1)).max())
                     for s in splines],
                 "ref_excursion_rad": [r5(q_knots[-1, j] - q_knots[0, j])
                                       for j in range(6)],
                 "delivered_vs_model_demand": {}}
        for j in range(6):
            d = delivered_ratio(bag, lo, hi, j, gain)
            if d:
                entry["delivered_vs_model_demand"][JOINTS[j]] = d
        out.append(entry)
    return out


def phase_speed_summary(bag, t0, bounds, hold_end, ref_fn, gain, joint=1):
    """Delivered speed against reference speed and model demand per phase."""
    ct, fb, ref = bag["ct"], bag["fb"], bag["ref"]
    vel = velocity_centered(ct, fb)
    ref_vel = velocity_centered(ct, ref)
    ranges = dict(bounds)
    ranges["hold"] = (bounds["decel"][1], hold_end)
    out = {}
    for phase in PHASES:
        a, b = ranges[phase]
        m = (ct >= a) & (ct < b) & np.isfinite(vel[:, joint])
        if m.sum() < 3:
            continue
        v = np.abs(vel[m, joint])
        out[phase] = {
            "delivered_speed_median": r5(np.median(v)),
            "delivered_speed_max": r5(v.max()),
            "reference_speed_median": r5(np.median(np.abs(ref_vel[m, joint]))),
            "model_demand_median": r5(np.median(
                gain * np.abs((ref - fb)[m, joint]))),
            "model_demand_max": r5((gain * np.abs((ref - fb)[m, joint])).max()),
            "position_start": r5(fb[m][0, joint]),
            "position_end": r5(fb[m][-1, joint]),
        }
    m = (ct >= bounds["accel"][0] + 0.5) & (ct < hold_end - 0.15) & np.isfinite(
        vel[:, joint])
    v = np.abs(vel[m, joint])
    demand = gain * np.abs((ref - fb)[m, joint])
    out["association_over_motion_and_hold"] = {
        "note": "Pearson r of delivered |speed| with each candidate driver; "
                "|r| near 0 = delivered speed does not scale with it",
        "with_model_demand": r5(np.corrcoef(v, demand)[0, 1]),
        "with_reference_speed": r5(np.corrcoef(
            v, np.abs(ref_vel[m, joint]))[0, 1]),
        "with_position": r5(np.corrcoef(v, fb[m, joint])[0, 1]),
        "delivered_speed_p10_p90": [r5(np.percentile(v, 10)),
                                    r5(np.percentile(v, 90))],
        "model_demand_p10_p90": [r5(np.percentile(demand, 10)),
                                 r5(np.percentile(demand, 90))],
    }
    return out


def hold_decay(bag, t_from, t_to, joint):
    """Error decay after the reference ended: linear vs exponential."""
    ct, fb, ref = bag["ct"], bag["fb"], bag["ref"]
    m = (ct >= t_from) & (ct <= t_to)
    t = ct[m] - ct[m][0]
    e = (ref - fb)[m, joint]
    sign = np.sign(e[0]) or 1.0
    y = np.abs(e) * 1.0
    usable = y > 0.002
    if usable.sum() < 6:
        return None
    tt, yy = t[usable], y[usable]
    lin = np.polyfit(tt, yy, 1)
    lin_rms = rms(yy - np.polyval(lin, tt))
    expo = np.polyfit(tt, np.log(yy), 1)
    exp_rms = rms(yy - np.exp(np.polyval(expo, tt)))
    return {"joint": JOINTS[joint], "sign": float(sign),
            "e_first_rad": r5(e[0]), "samples_used": int(usable.sum()),
            "linear_rate_rad_s": r5(-lin[0]), "linear_fit_rms_rad": r5(lin_rms),
            "exponential_tau_sec": r5(-1.0 / expo[0]) if expo[0] < 0 else None,
            "exponential_fit_rms_rad": r5(exp_rms),
            "t": [r5(x) for x in (t + ct[m][0])[::5]],
            "abs_error": [r5(x) for x in y[::5]]}


# ---------------------------------------------------------- stream latency
def stream_latency(run, bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from control_msgs.msg import JointTrajectoryControllerState
    from sensor_msgs.msg import JointState
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(run / "bag"), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"))
    out = {"controller_state": [], "joint_states": []}
    while reader.has_next():
        topic, data, wall_ns = reader.read_next()
        if topic == "/joint_states":
            m = deserialize_message(data, JointState)
            key = "joint_states"
        elif topic == base.CONTROLLER_STATE_TOPIC:
            m = deserialize_message(data, JointTrajectoryControllerState)
            key = "controller_state"
        else:
            continue
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        out[key].append(base.wall_to_sim(bag, wall_ns * 1e-9) - stamp)
    summary = {}
    for key, values in out.items():
        a = np.array(values)
        summary[key] = {"n": int(len(a)),
                        "receive_minus_stamp_sec": {
                            "median": r5(np.median(a)),
                            "p95": r5(np.percentile(a, 95)),
                            "max": r5(a.max())},
                        "note": "wall->sim via /clock bag times; resolution "
                                "limited by /clock cadence"}
    ct, jt, jq, fb = bag["ct"], bag["js_t"], bag["js_q"], bag["fb"]
    same = np.isin(np.round(ct, 4), np.round(jt, 4))
    idx = np.searchsorted(jt, ct[same])
    summary["feedback_equals_joint_states_at_same_stamp"] = {
        "controller_samples_with_matching_joint_state_stamp": int(same.sum()),
        "of": int(len(ct)),
        "max_abs_delta_rad": r5(np.abs(jq[idx] - fb[same]).max()),
    }
    return summary


# ------------------------------------------------------------------- main
def analyse(run, leg_index):
    bag = base.read_bag(run)
    rows = leg_windows(run, bag)
    if leg_index < 0:
        transit = [r for r in rows if r["name"] == "transit"]
        row = transit[1]
    else:
        row = rows[leg_index]
    assert "accept_sim" in row, "chosen leg has no unique controller goal"
    s1, t0 = stage1(bag, row)
    t_knots, q_knots, _, splines = hermite_from_leg(row["leg"])
    ref_end = t0 + t_knots[-1]

    def ref_fn(t_abs, j):
        return splines[j](np.clip(np.asarray(t_abs) - t0, 0.0, t_knots[-1]))

    bounds = phase_bounds(t_knots, splines, t0)
    next_accept = row["window_end"]
    last_q = q_knots[-1]
    t_conv, conv_detail = base.converged_after(
        bag["js_t"], bag["js_q"], ref_end, last_q, until=next_accept)
    hold_end = t_conv if t_conv else next_accept
    taus = fit_delay(bag["ct"], bag["fb"], ref_fn, t0, ref_end)
    healthy = [taus[j][0] for j in SERVO_FIT_JOINTS]
    common_tau = float(np.median(healthy))
    gain = servo_fit(bag, [(r["submit_sim"], r["window_end"]) for r in rows])
    phases = phase_table(bag, t0, bounds, hold_end, taus, ref_fn, common_tau)
    excursion = [float(q_knots[-1, j] - q_knots[0, j]) for j in range(6)]
    result = {
        "run": str(run), "leg_index": row["index"], "leg_name": row["name"],
        "note_on_naming": "second 'transit' leg of THIS run (%.2f s); the "
                          "Run08 ~26 s transit-2 is a different trajectory "
                          "and is not in this run." % t_knots[-1],
        "anchors_sim": {
            "submit": r5(row["submit_sim"]), "accept": r5(row["accept_sim"]),
            "reference_start_fit": r5(t0), "reference_end": r5(ref_end),
            "success": r5(row.get("success_sim")),
            "converged_positions_only": r5(t_conv),
            "hold_window_end": r5(hold_end), "converge_detail": conv_detail},
        "stage1_submitted_to_reference": s1,
        "phase_bounds_sim": {k: [r5(v[0]), r5(v[1])] for k, v in
                             bounds.items() if k != "peak_path_speed_rad_s"},
        "peak_path_speed_rad_s": r5(bounds["peak_path_speed_rad_s"]),
        "reference_excursion_rad": {JOINTS[j]: r5(excursion[j])
                                    for j in range(6)},
        "low_signal_joints": [JOINTS[j] for j in range(6)
                              if abs(excursion[j]) < LOW_SIGNAL_RAD],
        "stage2_delay_fit": {
            JOINTS[j]: {"tau_sec": r5(taus[j][0]), "rms_rad": r5(taus[j][1]),
                        "at_search_limit": taus[j][2]} for j in range(6)},
        "common_tau_sec": r5(common_tau),
        "common_tau_source": "median of per-joint fits on joints 1,4,6",
        "phase_table": phases,
        "lag_model_comparison": lag_model_comparison(
            bag, t0, hold_end, ref_fn, taus),
        "servo_law_model": {
            "K_per_sec_fitted_on_joints_1_4_6_all_legs": r5(gain),
            "meaning": "model: commanded speed = K*(reference - position); "
                       "fitted description, not read from source"},
        "per_leg_dependence": per_leg_dependence(bag, rows, gain),
        "joint2_phase_speed": phase_speed_summary(
            bag, t0, bounds, hold_end, ref_fn, gain),
        "hold_decay_joint2": hold_decay(bag, ref_end, hold_end, 1),
        "stream_latency_and_alignment": stream_latency(run, bag),
    }
    return result, bag, dict(row=row, t0=t0, ref_end=ref_end, ref_fn=ref_fn,
                             taus=taus, common_tau=common_tau,
                             bounds=bounds, hold_end=hold_end, gain=gain)


def markdown_table(result):
    lines = ["| phase | t (sim s) | joint | raw rms | fit-tau rms | fit-tau max "
             "| fit-tau trend rad/s | common-tau rms | common-tau max |",
             "|---|---|---|---|---|---|---|---|---|"]
    for phase in PHASES:
        row = result["phase_table"][phase]
        for joint in JOINTS:
            if joint in result["low_signal_joints"]:
                continue
            j = row["joints"].get(joint)
            if not j:
                continue
            lines.append("| %s | %.2f-%.2f | %s | %s | %s | %s | %s | %s | %s |"
                         % (phase, row["t_start"], row["t_end"],
                            joint.replace("elfin_", ""), j["raw_rms"],
                            j["fit_tau_rms"], j["fit_tau_max"],
                            j["fit_tau_trend_rad_s"], j["common_tau_rms"],
                            j["common_tau_max"]))
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--leg-index", type=int, default=-1,
                        help="submitted-leg index; -1 = second transit leg")
    args = parser.parse_args(argv)
    run, out = Path(args.run), Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    result, bag, ctx = analyse(run, args.leg_index)
    (out / "joint2_tracking.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n")
    (out / "joint2_tracking_phase_table.md").write_text(
        markdown_table(result) + "\n")
    try:
        import joint2_tracking_figure as figure
        figure.render(result, bag, ctx, out / "joint2_tracking.png")
    except ImportError as exc:  # figure module is optional
        print("figure skipped:", exc)
    print(markdown_table(result))
    return 0


if __name__ == "__main__":
    sys.exit(main())
