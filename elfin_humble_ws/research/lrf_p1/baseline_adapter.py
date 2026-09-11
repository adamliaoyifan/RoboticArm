"""Wrap the production model-based estimator. Read-only import."""

from __future__ import division

import math
import time

import numpy as np

from luggage_perception.luggage_box_estimator import estimate_box

from research.lrf_p1.contracts import (
    REASON_BASELINE_FAILED,
    REASON_EMPTY,
    REASON_MALFORMED,
    REASON_NONFINITE,
    REASON_OK,
    REASON_TOO_FEW,
    assert_inference_clean,
)


def _finite_points(points):
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] < 3:
        return None, REASON_MALFORMED
    if pts.shape[0] == 0:
        return None, REASON_EMPTY
    xyz = pts[:, :3]
    if not np.isfinite(xyz).all():
        return None, REASON_NONFINITE
    return xyz, REASON_OK


def estimate_from_observation(observation, min_points=40):
    """Run the production estimator on a hardware-observable observation."""
    assert_inference_clean(observation)
    t0 = time.perf_counter()
    xyz, reason = _finite_points(observation.get("points"))
    if xyz is None:
        return {
            "ok": False,
            "reason": reason,
            "box": None,
            "latency_ms": (time.perf_counter() - t0) * 1000.0,
        }
    if len(xyz) < int(min_points):
        return {
            "ok": False,
            "reason": REASON_TOO_FEW,
            "box": None,
            "latency_ms": (time.perf_counter() - t0) * 1000.0,
        }
    roi = observation.get("roi_center_xy")
    est = estimate_box(
        xyz,
        roi_center_xy=roi,
        roi_margin=0.55,
        platform_z=None,
        catalog_entries=None,
        min_points=min_points,
        voxel_size=0.015,
    )
    latency = (time.perf_counter() - t0) * 1000.0
    if est is None:
        return {
            "ok": False,
            "reason": REASON_BASELINE_FAILED,
            "box": None,
            "latency_ms": latency,
        }
    yaw = math.atan2(
        2.0 * (est.quaternion_xyzw[3] * est.quaternion_xyzw[2]),
        1.0 - 2.0 * (est.quaternion_xyzw[2] ** 2),
    )
    return {
        "ok": True,
        "reason": REASON_OK,
        "box": {
            "width": float(est.width),
            "depth": float(est.depth),
            "height": float(est.height),
            "center_xyz": [float(v) for v in est.center_xyz],
            "yaw": float(yaw),
            "confidence": float(est.confidence),
            "yaw_valid": bool(est.yaw_valid),
        },
        "latency_ms": latency,
        "source": "observed",
    }


def fuse_views(observations):
    """Measured multi-view fusion: concatenate clouds, then the baseline."""
    clouds = []
    roi = None
    for obs in observations:
        pts = np.asarray(obs.get("points"), dtype=np.float64)
        if pts.ndim == 2 and pts.shape[1] >= 3 and len(pts):
            clouds.append(pts[:, :3])
            roi = obs.get("roi_center_xy", roi)
    if not clouds:
        return estimate_from_observation({"points": np.zeros((0, 3))})
    merged = np.concatenate(clouds, axis=0)
    return estimate_from_observation({
        "points": merged,
        "roi_center_xy": roi,
        "frame_id": "world",
        "stamp": 0.0,
    })
