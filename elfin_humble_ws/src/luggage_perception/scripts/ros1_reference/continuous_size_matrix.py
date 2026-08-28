#!/usr/bin/env python3
"""Phase 6a acceptance: size estimation on continuously sized boxes.

The old matrix scored ``catalog_match_rate`` -- whether the estimate snapped
back to one of three known shapes. With boxes sampled continuously there is
nothing to snap to, so the questions change:

  * per-axis error, with height held to a tighter bar because the platform
    height is a known prior and the top plane is measured directly;
  * conservatism, which matters more than raw accuracy. Underestimating a
    footprint is the dangerous direction: the map believes the box is smaller
    than it is and the next box gets planned into the overlap. The committed
    size (estimate + margin) must cover the true size.

Pure numpy, no ROS. Usage::

    python3 continuous_size_matrix.py --samples 50 --output metrics.yaml
"""

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

PLATFORM_Z = 0.86
SIZE_RANGE = ((0.55, 0.80), (0.40, 0.50), (0.25, 0.32))


def _yaw_error_mod_pi(actual, expected):
    delta = actual - expected
    return abs(math.atan2(math.sin(2.0 * delta), math.cos(2.0 * delta))) * 0.5


def _cloud(rng, center, size, yaw, occlusion=0.0):
    """Top face over a platform, optionally with one edge band missing.

    ``occlusion`` trims that fraction of the top face along the principal axis,
    reproducing the partially visible face that made E17 underestimate width by
    up to 0.10 m.
    """
    width, depth, height = size
    n = 500
    u_low = -width * 0.5 + occlusion * width
    u = rng.uniform(u_low, width * 0.5, n)
    v = rng.uniform(-depth * 0.5, depth * 0.5, n)
    c, s = math.cos(yaw), math.sin(yaw)
    top = np.column_stack([
        center[0] + c * u - s * v + rng.normal(0, 0.003, n),
        center[1] + s * u + c * v + rng.normal(0, 0.003, n),
        np.full(n, PLATFORM_Z + height) + rng.normal(0, 0.003, n),
    ])
    platform = np.column_stack([
        rng.uniform(-1.5, -0.5, 1800),
        rng.uniform(-0.5, 0.5, 1800),
        np.full(1800, PLATFORM_Z) + rng.normal(0, 0.001, 1800),
    ])
    return np.vstack([platform, top])


def _sample_size(rng):
    return [float(rng.uniform(low, high)) for low, high in SIZE_RANGE]


def run_matrix(samples=50, seed=42, xy_margin=0.02, z_margin=0.01,
               max_occlusion=0.10):
    rng = np.random.RandomState(seed)
    rows = []
    for index in range(samples):
        size = _sample_size(rng)
        x = -1.0 + rng.uniform(-0.10, 0.10)
        y = rng.uniform(-0.10, 0.10)
        yaw = rng.uniform(-math.pi, math.pi)
        occlusion = float(rng.uniform(0.0, max_occlusion))
        est = estimate_box(
            _cloud(rng, (x, y), size, yaw, occlusion),
            roi_center_xy=(-1.0, 0.0),
            roi_margin=0.55,
            platform_z=PLATFORM_Z,
            # No catalog snapping: that is the behaviour under test.
            catalog_entries=None,
            min_points=50,
        )
        if est is None:
            rows.append({"index": index, "success": False})
            continue
        est_yaw = math.atan2(
            2.0 * est.quaternion_xyzw[3] * est.quaternion_xyzw[2],
            1.0 - 2.0 * est.quaternion_xyzw[2] ** 2,
        )
        # Footprint axes may swap with yaw; compare as an unordered pair.
        true_major, true_minor = max(size[0], size[1]), min(size[0], size[1])
        est_major = max(est.width, est.depth)
        est_minor = min(est.width, est.depth)
        committed = (
            est_major + 2.0 * xy_margin,
            est_minor + 2.0 * xy_margin,
            est.height + z_margin,
        )
        rows.append({
            "index": index,
            "success": True,
            "true_size": [round(v, 4) for v in size],
            "estimated": [round(est_major, 4), round(est_minor, 4),
                          round(est.height, 4)],
            "occlusion": round(occlusion, 3),
            "major_error_m": est_major - true_major,
            "minor_error_m": est_minor - true_minor,
            "height_error_m": est.height - size[2],
            "xy_error_m": math.hypot(
                est.center_xyz[0] - x, est.center_xyz[1] - y),
            "yaw_error_rad_mod_pi": _yaw_error_mod_pi(est_yaw, yaw),
            "committed_covers_major": committed[0] >= true_major,
            "committed_covers_minor": committed[1] >= true_minor,
            "committed_covers_height": committed[2] >= size[2],
            "in_envelope": all(
                SIZE_RANGE[axis][0] - 0.06 <= value <= SIZE_RANGE[axis][1] + 0.06
                for axis, value in enumerate(
                    (est_major, est_minor, est.height))),
            "matched_catalog_id": est.matched_catalog_id,
            "confidence": est.confidence,
        })

    ok = [row for row in rows if row["success"]]

    def p95_abs(name):
        values = [abs(row[name]) for row in ok]
        return float(np.percentile(values, 95)) if values else None

    def p95(name):
        values = [row[name] for row in ok]
        return float(np.percentile(values, 95)) if values else None

    def rate(name):
        return (float(sum(bool(row[name]) for row in ok)) / len(ok)
                if ok else 0.0)

    conservative = (
        float(sum(
            row["committed_covers_major"] and row["committed_covers_minor"]
            and row["committed_covers_height"] for row in ok)) / len(ok)
        if ok else 0.0)
    metrics = {
        "sample_count": samples,
        "success_count": len(ok),
        "success_rate": float(len(ok)) / max(1, samples),
        "major_error_p95_m": p95_abs("major_error_m"),
        "minor_error_p95_m": p95_abs("minor_error_m"),
        "height_error_p95_m": p95_abs("height_error_m"),
        "major_error_mean_m": (
            float(np.mean([row["major_error_m"] for row in ok])) if ok else None),
        "center_xy_error_p95_m": p95("xy_error_m"),
        "yaw_error_p95_rad_mod_pi": p95("yaw_error_rad_mod_pi"),
        "committed_covers_true_rate": conservative,
        "in_envelope_rate": rate("in_envelope"),
        "catalog_snap_count": sum(
            1 for row in ok if row["matched_catalog_id"]),
        "xy_margin_m": xy_margin,
        "z_margin_m": z_margin,
    }
    gates = {
        "success_rate": metrics["success_rate"] >= 0.95,
        # Height has a platform prior, so it is held tighter than the footprint.
        "height_error": metrics["height_error_p95_m"] <= 0.010,
        "footprint_error": max(
            metrics["major_error_p95_m"],
            metrics["minor_error_p95_m"]) <= 0.030,
        "center_xy_error": metrics["center_xy_error_p95_m"] <= 0.020,
        "yaw_error": (
            metrics["yaw_error_p95_rad_mod_pi"] <= math.radians(5.0)),
        # The one that actually protects the next placement.
        "conservative_commit": metrics["committed_covers_true_rate"] >= 0.99,
        "plausibility": metrics["in_envelope_rate"] >= 1.0,
        "no_catalog_snapping": metrics["catalog_snap_count"] == 0,
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
    parser.add_argument("--samples", type=int, default=50)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--xy-margin", type=float, default=0.02)
    parser.add_argument("--z-margin", type=float, default=0.01)
    parser.add_argument("--max-occlusion", type=float, default=0.10)
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    result = run_matrix(
        args.samples, args.seed, args.xy_margin, args.z_margin,
        args.max_occlusion)
    text = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w") as stream:
            stream.write(text + "\n")
    print(json.dumps(
        {"metrics": result["metrics"], "gates": result["gates"],
         "passed": result["passed"]}, indent=2, sort_keys=True))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
