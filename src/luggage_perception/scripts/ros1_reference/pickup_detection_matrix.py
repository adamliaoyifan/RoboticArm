#!/usr/bin/env python3
"""Deterministic 30-sample RGBD estimator acceptance matrix."""
from __future__ import division

import argparse
import json
import math
import os
import sys

import numpy as np

SCRIPTS = os.path.dirname(os.path.abspath(__file__))
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from luggage_box_estimator import estimate_box  # noqa: E402


CATALOG = [
    {"id": "carryon", "size": [0.55, 0.40, 0.25]},
    {"id": "standard", "size": [0.70, 0.45, 0.28]},
    {"id": "large", "size": [0.80, 0.50, 0.32]},
]


def _yaw_error_mod_pi(actual, expected):
    delta = actual - expected
    return abs(math.atan2(math.sin(2.0 * delta), math.cos(2.0 * delta))) * 0.5


def _cloud(rng, center, size, yaw, platform_z=0.86):
    width, depth, height = size
    n = 500
    u = rng.uniform(-width * 0.5, width * 0.5, n)
    v = rng.uniform(-depth * 0.5, depth * 0.5, n)
    c, s = math.cos(yaw), math.sin(yaw)
    top = np.column_stack([
        center[0] + c * u - s * v + rng.normal(0, 0.003, n),
        center[1] + s * u + c * v + rng.normal(0, 0.003, n),
        np.full(n, platform_z + height) + rng.normal(0, 0.003, n),
    ])
    platform = np.column_stack([
        rng.uniform(-1.5, -0.5, 1800),
        rng.uniform(-0.5, 0.5, 1800),
        np.full(1800, platform_z) + rng.normal(0, 0.001, 1800),
    ])
    return np.vstack([platform, top])


def run_matrix(samples=30, seed=42):
    rng = np.random.RandomState(seed)
    rows = []
    for index in range(samples):
        entry = CATALOG[index % len(CATALOG)]
        x = -1.0 + rng.uniform(-0.10, 0.10)
        y = rng.uniform(-0.10, 0.10)
        yaw = rng.uniform(-math.pi, math.pi)
        est = estimate_box(
            _cloud(rng, (x, y), entry["size"], yaw),
            roi_center_xy=(-1.0, 0.0),
            roi_margin=0.55,
            platform_z=0.86,
            catalog_entries=CATALOG,
            catalog_tolerance=0.08,
            min_points=50,
        )
        if est is None:
            rows.append({"index": index, "success": False})
            continue
        est_yaw = math.atan2(
            2.0 * est.quaternion_xyzw[3] * est.quaternion_xyzw[2],
            1.0 - 2.0 * est.quaternion_xyzw[2] ** 2,
        )
        rows.append({
            "index": index,
            "success": True,
            "expected_id": entry["id"],
            "detected_id": est.matched_catalog_id,
            "xy_error_m": math.hypot(
                est.center_xyz[0] - x, est.center_xyz[1] - y),
            "z_error_m": abs(
                est.center_xyz[2] - (0.86 + entry["size"][2] * 0.5)),
            "yaw_error_rad_mod_pi": _yaw_error_mod_pi(est_yaw, yaw),
            "confidence": est.confidence,
        })
    successful = [row for row in rows if row["success"]]
    def percentile(name, q):
        values = [row[name] for row in successful]
        return float(np.percentile(values, q)) if values else None
    metrics = {
        "sample_count": samples,
        "success_count": len(successful),
        "success_rate": float(len(successful)) / max(1, samples),
        "catalog_match_rate": (
            float(sum(
                row["detected_id"] == row["expected_id"]
                for row in successful)) / max(1, len(successful))),
        "xy_error_p95_m": percentile("xy_error_m", 95),
        "z_error_p95_m": percentile("z_error_m", 95),
        "yaw_error_p95_rad_mod_pi": percentile(
            "yaw_error_rad_mod_pi", 95),
    }
    gates = {
        "success_rate": metrics["success_rate"] >= 0.95,
        "catalog_match": metrics["catalog_match_rate"] >= 1.0,
        "xy_error": metrics["xy_error_p95_m"] <= 0.03,
        "z_error": metrics["z_error_p95_m"] <= 0.02,
        "yaw_error": (
            metrics["yaw_error_p95_rad_mod_pi"]
            <= math.radians(10.0)),
    }
    return {
        "schema_version": 1,
        "seed": seed,
        "metrics": metrics,
        "gates": gates,
        "passed": all(gates.values()),
        "samples": rows,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=30)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    result = run_matrix(args.samples, args.seed)
    text = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w") as stream:
            stream.write(text + "\n")
    print(text)
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
