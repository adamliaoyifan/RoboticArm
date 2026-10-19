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


def compute_metrics(samples, contacts, model, t_spawn_s):
    """samples: [(t_ns, x, y, z, quat)] for `model`; contacts: parsed."""
    if not samples:
        return {"valid": False, "reason": "no_dynamic_samples"}
    t0 = samples[0][0] / 1e9
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

    at_primary = [r for r in rows if r["t_rel_s"] <= T_PRIMARY_S]
    primary = at_primary[-1]["xy_offset_m"] if at_primary else None

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
