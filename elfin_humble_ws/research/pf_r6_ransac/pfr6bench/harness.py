#!/usr/bin/env python3
"""Harness: run each comparator through the exact committed validation.

Patches the committed ``estimate_local_support``'s plane-fitter symbol so
every comparator gets the identical band/annulus candidate set, the same
side-coverage rule, and the same reason-code semantics as production.
"""

from __future__ import division

import time

import numpy as np

import luggage_perception.top_support_estimator as tse
from luggage_perception.top_support_estimator import TopSupportConfig


def run_support(raw, top, workspace, fitter, config=None):
    """estimate_local_support with an injected plane fitter (no production edit)."""
    config = config or TopSupportConfig()
    orig = tse._ransac_horizontal_plane
    tse._ransac_horizontal_plane = fitter
    try:
        est = tse.estimate_local_support(raw, top, workspace, config)
    finally:
        tse._ransac_horizontal_plane = orig
    return est


def sample_metrics(est, gt_support_z, expect):
    valid = (
        est is not None and est.reason == "ok"
        and np.isfinite(est.support_z))
    err = abs(est.support_z - gt_support_z) if valid else None
    return {
        "valid": bool(valid),
        "reason": est.reason if est is not None else "none",
        "support_z": float(est.support_z) if est is not None else None,
        "abs_err_mm": (None if err is None else round(1000.0 * err, 3)),
        "residual_mm": (
            None if est is None or not np.isfinite(est.residual)
            else round(1000.0 * est.residual, 3)),
        "inlier_count": 0 if est is None else int(est.inlier_count),
        "side_coverage": 0.0 if est is None else round(
            float(est.side_coverage), 4),
        # A negative-control "false measured support" = valid height at a
        # wrong plane (> 25 mm, the gate-1 maximum).
        "false_measured_support": bool(
            valid and expect == "reject" and err is not None
            and err > 0.025),
    }


def time_fitter(fitter, points, repeats=30, warmup=5, **kw):
    """Wall-clock latency of the plane-fit stage alone (ms percentiles)."""
    args = dict(max_iter=200, dist_thresh=0.008, normal_thresh=0.15,
                min_inliers=80)
    args.update(kw)
    for _ in range(warmup):
        fitter(points, **args)
    times = []
    for _ in range(int(repeats)):
        t0 = time.perf_counter()
        fitter(points, **args)
        times.append((time.perf_counter() - t0) * 1000.0)
    a = np.asarray(times)
    return {
        "n": int(len(a)),
        "p50_ms": round(float(np.percentile(a, 50)), 3),
        "p95_ms": round(float(np.percentile(a, 95)), 3),
        "max_ms": round(float(a.max()), 3),
    }


def time_full(raw, top, workspace, fitter, repeats=10, warmup=2,
              config=None):
    """Wall-clock of full estimate_local_support with the injected fitter."""
    for _ in range(int(warmup)):
        run_support(raw, top, workspace, fitter, config)
    times = []
    for _ in range(int(repeats)):
        t0 = time.perf_counter()
        run_support(raw, top, workspace, fitter, config)
        times.append((time.perf_counter() - t0) * 1000.0)
    a = np.asarray(times)
    return {
        "n": int(len(a)),
        "p50_ms": round(float(np.percentile(a, 50)), 3),
        "p95_ms": round(float(np.percentile(a, 95)), 3),
        "max_ms": round(float(a.max()), 3),
    }
