"""Observation-only geometric features. No mesh, GT, or occlusion labels."""

from __future__ import division

import numpy as np

from research.lrf_p1.contracts import assert_inference_clean


FEATURE_NAMES = (
    "n_log", "cx", "cy", "cz", "sx", "sy", "sz",
    "aabb_x", "aabb_y", "aabb_z", "p05_z", "p95_z",
    "xy_eval0", "xy_eval1",
    "b_ok", "b_w", "b_d", "b_h", "b_cx", "b_cy", "b_cz", "b_yaw",
    "b_conf", "b_yaw_valid",
)


def cloud_features(observation, baseline_box):
    assert_inference_clean(observation)
    pts = np.asarray(observation["points"], dtype=np.float64)
    xyz = pts[:, :3] if pts.ndim == 2 and pts.shape[1] >= 3 else np.zeros((0, 3))
    n = float(len(xyz))
    if n < 1:
        geom = [0.0] * 14
    else:
        centroid = xyz.mean(axis=0)
        std = xyz.std(axis=0)
        aabb = xyz.max(axis=0) - xyz.min(axis=0)
        p05 = float(np.percentile(xyz[:, 2], 5))
        p95 = float(np.percentile(xyz[:, 2], 95))
        cov = np.cov(xyz[:, :2], rowvar=False) if n > 3 else np.eye(2)
        eig = np.sort(np.linalg.eigvalsh(np.atleast_2d(cov)))[::-1]
        if eig.size < 2:
            eig = np.array([0.0, 0.0])
        geom = [
            math_log_n(n),
            float(centroid[0]), float(centroid[1]), float(centroid[2]),
            float(std[0]), float(std[1]), float(std[2]),
            float(aabb[0]), float(aabb[1]), float(aabb[2]),
            p05, p95,
            float(eig[0]), float(eig[1] if eig.size > 1 else 0.0),
        ]
    if baseline_box is None:
        base = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    else:
        c = baseline_box["center_xyz"]
        base = [
            1.0,
            float(baseline_box["width"]),
            float(baseline_box["depth"]),
            float(baseline_box["height"]),
            float(c[0]), float(c[1]), float(c[2]),
            float(baseline_box["yaw"]),
            float(baseline_box.get("confidence", 0.0)),
            1.0 if baseline_box.get("yaw_valid") else 0.0,
        ]
    return np.asarray(geom + base, dtype=np.float64)


def math_log_n(n):
    import math
    return math.log(max(1.0, float(n)))
