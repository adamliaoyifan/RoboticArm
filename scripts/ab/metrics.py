#!/usr/bin/env python3
"""Frozen metric computation for the A0/A'/B control runs.

Primary: XY offset at t = 38 SIM seconds from dynamic spawn (never wall
clock). Secondary: max offset, per-second speed, stop time, final
attitude, contact-partner changes. Pure offline: consumes the per-run
capture (dyn pose stream parsed by analyze_release_window) and emits
run_metrics.json. Pre-registered; changing any threshold or definition
after the nine runs start invalidates the set.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "g3")))
from analyze_release_window import parse_pose_stream, parse_contacts  # noqa

T_PRIMARY_S = 38.0
STOP_V_MPS = 0.0005          # "stopped" = below 0.5 mm/s for >= 1 s
CONTACT_PARTNER_MIN_COUNT = 5
#: Validity gate (frozen): the capture must span >= 44 of the 45 sim
#: seconds from spawn, be strictly time-monotone, and leave no sample
#: gap > MAX_GAP_S anywhere up to the primary; the 38 s primary is
#: LINEARLY INTERPOLATED between the bracketing samples (an early last
#: frame cannot stand in for 38 s).
MIN_SPAN_S = 44.0
MAX_GAP_S = 0.5


def compute_metrics(samples, contacts, model, t_spawn_s):
    """samples: [(t_ns, x, y, z, quat)] for `model`; contacts: parsed."""
    if not samples:
        return {"valid": False, "reason": "no_dynamic_samples"}
    # Validity gate: span, monotonicity, max gap up to the primary.
    ts = [s[0] for s in samples]
    if any(b <= a for a, b in zip(ts, ts[1:])):
        return {"valid": False, "reason": "time_not_monotone"}
    t0 = ts[0] / 1e9
    span = (ts[-1] - ts[0]) / 1e9
    if span < MIN_SPAN_S:
        return {"valid": False, "reason": "span_short",
                "span_s": round(span, 3)}
    primary_ts = [t for t in ts if (t - ts[0]) / 1e9 <= T_PRIMARY_S]
    gaps = [(b - a) / 1e9 for a, b in zip(primary_ts, primary_ts[1:])]
    if gaps and max(gaps) > MAX_GAP_S:
        return {"valid": False, "reason": "sample_gap",
                "max_gap_s": round(max(gaps), 3)}
    x0, y0 = samples[0][1], samples[0][2]
    rows = []
    prev = None
    for s in samples:
        t = s[0] / 1e9 - t0
        row = {"t_rel_s": round(t, 4),
               "xy_offset_m": round(math.hypot(s[1] - x0, s[2] - y0), 6)}
        if prev is not None:
            dt = (s[0] - prev[0]) / 1e9
            if dt > 0:
                row["v_xy_mps"] = round(
                    math.hypot(s[1] - prev[1], s[2] - prev[2]) / dt, 6)
        rows.append(row)
        prev = s

    # Primary at 38 sim s: linear interpolation between the samples
    # bracketing T_PRIMARY (fixed rule; early-last-frame cannot stand in).
    primary = None
    for i in range(len(samples) - 1):
        ta = (samples[i][0] - samples[0][0]) / 1e9
        tb = (samples[i + 1][0] - samples[0][0]) / 1e9
        if ta <= T_PRIMARY_S <= tb and tb > ta:
            a = (T_PRIMARY_S - ta) / (tb - ta)
            offa = math.hypot(samples[i][1] - x0, samples[i][2] - y0)
            offb = math.hypot(samples[i + 1][1] - x0,
                              samples[i + 1][2] - y0)
            primary = (1 - a) * offa + a * offb
            break
    if primary is None and rows and rows[-1]["t_rel_s"] >= T_PRIMARY_S:
        primary = rows[-1]["xy_offset_m"]

    stop_time = None
    for i in range(1, len(rows)):
        if "v_xy_mps" in rows[i] and rows[i]["v_xy_mps"] < STOP_V_MPS:
            window = [r for r in rows
                      if 0.0 <= r["t_rel_s"] - rows[i]["t_rel_s"] <= 1.0
                      and "v_xy_mps" in r]
            if window and all(r["v_xy_mps"] < STOP_V_MPS for r in window):
                stop_time = rows[i]["t_rel_s"]
                break

    partners = {}
    for t_ns, bodies in contacts:
        for name in bodies:
            if model in name:
                continue
            key = name.split("::")[0]
            partners[key] = partners.get(key, 0) + 1
    partner_changes = sorted(
        k for k, v in partners.items() if v >= CONTACT_PARTNER_MIN_COUNT)

    last = samples[-1]
    return {
        "valid": True,
        "model": model,
        "t_span_s": [rows[0]["t_rel_s"], rows[-1]["t_rel_s"]],
        "primary_xy_offset_at_38sim_m": primary,
        "max_xy_offset_m": max(r["xy_offset_m"] for r in rows),
        "stop_time_s": stop_time,
        "final_attitude_quat": list(last[4]) if last[4] else None,
        "final_xy_m": [round(last[1], 5), round(last[2], 5)],
        "contact_partners": partner_changes,
        "series": rows,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--model", default="pickup_box_0001_inch20")
    parser.add_argument("--t-spawn-sim", type=float, required=True)
    args = parser.parse_args()
    dyn = parse_pose_stream(
        os.path.join(args.run_dir, "dyn_pose_stream.txt"), args.model)
    contacts = parse_contacts(
        os.path.join(args.run_dir, "contact_stream.txt"))
    metrics = compute_metrics(dyn, contacts, args.model,
                              args.t_spawn_sim)
    out = os.path.join(args.run_dir, "run_metrics.json")
    with open(out, "w") as handle:
        json.dump(metrics, handle, indent=2)
    summary = {k: v for k, v in metrics.items() if k != "series"}
    print(json.dumps(summary, indent=1))
    raise SystemExit(0 if metrics.get("valid") else 1)


if __name__ == "__main__":
    main()
