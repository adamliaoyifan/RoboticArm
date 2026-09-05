#!/usr/bin/env python3
"""Offline comparator evaluation on captured simulation frames (gate 8).

For every captured (raw world cloud, detector top estimate) pair, runs
each comparator through the exact committed estimate_local_support
validation and reports, per suitcase size, the support-valid rate
relative to the exact committed baseline, plus measured-height error vs
the spawn GT (eval-only).
"""

from __future__ import division

import argparse
import json
import os

import numpy as np

from luggage_perception.top_support_estimator import (
    TopSupportConfig, TopSurfaceEstimate)
from pfr6bench import harness
from pfr6bench.methods import COMPARATORS


def size_of(box_id):
    bid = (box_id or "").lower()
    for tag in ("carryon", "standard", "large"):
        if tag in bid:
            return tag
    return "unknown"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--capture-dir", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    rows = json.load(open(os.path.join(args.capture_dir,
                                       "capture_rows.json")))
    usable = [r for r in rows if r.get("fid")]
    config = TopSupportConfig()
    # The live detector's non-privileged static workspace: scene_tf
    # pickup_source XY (platform top center) with the ROI margin as half
    # extents. Derived from scene_tf.yaml, not from any GT channel.
    workspace = ((-1.0, 0.0), (0.5, 0.5))
    per_frame = []
    for r in usable:
        data = np.load(os.path.join(args.capture_dir,
                                    r["fid"] + ".npz"))
        raw = data["raw_world"].astype(np.float64)
        t = r["top"]
        top = TopSurfaceEstimate(
            center_xy=np.asarray(t["center_xy"]), top_z=t["top_z"],
            yaw=t["yaw"], width=t["width"], depth=t["depth"],
            confidence=1.0)
        for name, fitter in sorted(COMPARATORS.items()):
            est = harness.run_support(raw, top, workspace, fitter, config)
            valid = est is not None and est.reason == "ok" and np.isfinite(
                est.support_z)
            measured_h = (
                t["top_z"] - est.support_z) if valid else None
            per_frame.append({
                "fid": r["fid"], "trial": r["trial"],
                "size": size_of(r.get("gt_box_id")),
                "gt_box_id": r.get("gt_box_id"),
                "method": name,
                "valid": bool(valid),
                "reason": est.reason,
                "support_z": None if est is None else float(est.support_z),
                "measured_height_m": measured_h,
                "gt_height_m": r.get("gt_height_m"),
                "height_err_mm": (
                    None if measured_h is None
                    or r.get("gt_height_m") is None
                    else round(1000.0 * (
                        measured_h - r["gt_height_m"]), 2)),
                "candidate_note": r.get("raw_points"),
            })
    # Aggregate per size per method.
    agg = {}
    for size in sorted({p["size"] for p in per_frame}):
        agg[size] = {}
        base_valid = None
        for name in sorted(COMPARATORS):
            sel = [p for p in per_frame
                   if p["size"] == size and p["method"] == name]
            n = len(sel)
            valid = sum(1 for p in sel if p["valid"])
            errs = [p["height_err_mm"] for p in sel
                    if p["height_err_mm"] is not None]
            e = np.asarray(errs) if errs else np.array([])
            if name == "baseline_ransac":
                base_valid = valid / max(1, n)
            agg[size][name] = {
                "frames": n,
                "support_valid_rate": round(valid / max(1, n), 4),
                "height_err_mm": {
                    "median": None if not len(e)
                    else round(float(np.median(e)), 2),
                    "p95": None if not len(e)
                    else round(float(np.percentile(e, 95)), 2),
                    "max": None if not len(e)
                    else round(float(e.max()), 2),
                },
            }
        for name in agg[size]:
            delta = (agg[size][name]["support_valid_rate"]
                     - base_valid) * 100.0
            agg[size][name]["valid_rate_delta_pp_vs_baseline"] = round(
                delta, 2)
    gates = {}
    for name in sorted(COMPARATORS):
        if name == "baseline_ransac":
            continue
        deltas = [agg[s][name]["valid_rate_delta_pp_vs_baseline"]
                  for s in agg]
        gates[name] = {
            "8_three_size_valid_rate_regression": (
                "pass" if deltas and max(deltas) <= 1.0 else "fail"),
            "max_valid_rate_delta_pp": max(deltas) if deltas else None,
        }
    out = {"per_frame": per_frame, "aggregate": agg, "gates": gates,
           "n_usable_frames": len(usable)}
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w") as fh:
        json.dump(out, fh, indent=2, sort_keys=True)
    print(json.dumps(agg, indent=2, sort_keys=True))
    print(json.dumps(gates, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
