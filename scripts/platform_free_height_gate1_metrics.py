#!/usr/bin/env python3
"""Gate 1 metric sweep for platform-free height. Writes JSONL + summary.

Usage:
  PYTHONPATH=src/luggage_perception python3 scripts/platform_free_height_gate1_metrics.py \
    --out docs/status/evidence/platform_free_height/<run_id>/gate1
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/luggage_perception"))
sys.path.insert(0, str(ROOT / "src/luggage_perception/test"))

from luggage_perception.top_support_estimator import (  # noqa: E402
    GEOMETRY_FULL_3D,
    HEIGHT_SOURCE_CATALOG_PRIOR,
    HEIGHT_SOURCE_MEASURED_SUPPORT,
    compose_box_geometry,
    estimate_local_support,
    estimate_top_surface,
)
from test_top_support_estimator import (  # noqa: E402
    CONFIG,
    SIZES,
    SUPPORT_ZS,
    WORKSPACE,
    YAWS,
    _full_scene,
)

LIMITS = {
    "top_z_abs_mm": 5.0,
    "support_z_abs_mm": 5.0,
    "height_abs_mm": 10.0,
    "width_depth_abs_mm": 20.0,
}


def percentile(values, p):
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), p))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for i, (w, d, h) in enumerate(SIZES):
        for j, yaw in enumerate(YAWS):
            for k, support_z in enumerate(SUPPORT_ZS):
                seed = 10 * i + j + k
                cargo, raw, top_z = _full_scene(
                    w, d, h, yaw, support_z, seed=seed, top_noise=0.003,
                    outlier_fraction=0.10)
                top = estimate_top_surface(cargo, WORKSPACE, CONFIG)
                support = estimate_local_support(raw, top, WORKSPACE, CONFIG)
                box = compose_box_geometry(top, support)
                row = {
                    "seed": seed,
                    "width_m": w,
                    "depth_m": d,
                    "height_m": h,
                    "yaw_rad": yaw,
                    "support_z_ref": support_z,
                    "top_z_ref": top_z,
                    "top_valid": top is not None,
                    "support_reason": None if support is None else support.reason,
                    "height_valid": bool(box.height_valid),
                    "height_source": int(box.height_source),
                    "geometry_level": int(box.geometry_level),
                    "top_z_err_mm": None,
                    "support_z_err_mm": None,
                    "height_err_mm": None,
                    "width_err_mm": None,
                    "depth_err_mm": None,
                    "floor_wrong": False,
                    "nonfinite": False,
                }
                if top is None or box.center_xyz is None:
                    row["nonfinite"] = True
                else:
                    row["top_z_err_mm"] = abs(top.top_z - top_z) * 1000.0
                    row["width_err_mm"] = abs(max(top.width, top.depth) - max(w, d)) * 1000.0
                    row["depth_err_mm"] = abs(min(top.width, top.depth) - min(w, d)) * 1000.0
                    if support is not None and np.isfinite(support.support_z):
                        row["support_z_err_mm"] = abs(support.support_z - support_z) * 1000.0
                        if abs(support.support_z - (support_z - 1.5)) < 0.05:
                            row["floor_wrong"] = True
                    if box.height_valid:
                        row["height_err_mm"] = abs(box.height - h) * 1000.0
                    if not all(np.isfinite(v) for v in (
                            top.top_z, top.width, top.depth)):
                        row["nonfinite"] = True
                rows.append(row)

    jsonl = out / "frames.jsonl"
    with jsonl.open("w") as fh:
        for row in rows:
            fh.write(json.dumps(row) + "\n")

    full = [r for r in rows if r["height_valid"]
            and r["height_source"] == HEIGHT_SOURCE_MEASURED_SUPPORT]
    top_errs = [r["top_z_err_mm"] for r in rows if r["top_z_err_mm"] is not None]
    support_errs = [r["support_z_err_mm"] for r in full
                    if r["support_z_err_mm"] is not None]
    height_errs = [r["height_err_mm"] for r in full if r["height_err_mm"] is not None]
    wd_errs = []
    for r in rows:
        if r["width_err_mm"] is not None:
            wd_errs.append(r["width_err_mm"])
        if r["depth_err_mm"] is not None:
            wd_errs.append(r["depth_err_mm"])

    def stats(vals):
        return {
            "count": len(vals),
            "p50": percentile(vals, 50),
            "p95": percentile(vals, 95),
            "max": max(vals) if vals else None,
        }

    summary = {
        "n_frames": len(rows),
        "n_full3d": len(full),
        "success_rate_full3d": (len(full) / len(rows)) if rows else 0.0,
        "top_z_mm": stats(top_errs),
        "support_z_mm": stats(support_errs),
        "height_mm": stats(height_errs),
        "width_depth_mm": stats(wd_errs),
        "wrong_floor": sum(1 for r in rows if r["floor_wrong"]),
        "nonfinite": sum(1 for r in rows if r["nonfinite"]),
        "limits": LIMITS,
        "pass": {
            "top_z": (max(top_errs) if top_errs else 0) <= LIMITS["top_z_abs_mm"],
            "support_z": (max(support_errs) if support_errs else 0) <= LIMITS["support_z_abs_mm"],
            "height": (max(height_errs) if height_errs else 0) <= LIMITS["height_abs_mm"],
            "width_depth": (max(wd_errs) if wd_errs else 0) <= LIMITS["width_depth_abs_mm"],
            "wrong_floor": sum(1 for r in rows if r["floor_wrong"]) == 0,
            "nonfinite": sum(1 for r in rows if r["nonfinite"]) == 0,
        },
    }
    summary["gate1_pass"] = all(summary["pass"].values())
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0 if summary["gate1_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
