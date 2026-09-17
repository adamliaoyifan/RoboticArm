"""Eval-only three-pose world oracle from 16UC1 deprojection + stamped TF.

Does not use a Gazebo camera PointCloud2. Pixel selection uses eval-side
GetCurrentBox / gz truth projected with the same-stamp extrinsics.
"""
from __future__ import division

import math

import numpy as np

from luggage_perception.depth_deprojection import deproject_selected
from luggage_perception.detect_overlay import project_detection
from luggage_perception.eval.gate4_scoring import GATE4_LIMITS, stats
from luggage_perception.top_support_estimator import (
    TopSupportConfig,
    estimate_local_support,
    estimate_top_surface,
)


class _K(object):
    def __init__(self, fx, fy, cx, cy):
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)


def quat_xyzw_to_R(qx, qy, qz, qw):
    qx, qy, qz, qw = [float(v) for v in (qx, qy, qz, qw)]
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def world_from_optical(points_optical, rot_wo, trans_wo):
    pts = np.asarray(points_optical, dtype=np.float64).reshape(-1, 3)
    rot = np.asarray(rot_wo, dtype=np.float64).reshape(3, 3)
    trans = np.asarray(trans_wo, dtype=np.float64).reshape(3)
    return pts.dot(rot.T) + trans


def optical_from_world_rt(rot_wo, trans_wo):
    rot = np.asarray(rot_wo, dtype=np.float64).reshape(3, 3)
    trans = np.asarray(trans_wo, dtype=np.float64).reshape(3)
    rot_ow = rot.T
    trans_ow = -rot_ow.dot(trans)
    return rot_ow, trans_ow


def gt_planes(gt_xyz, gt_size):
    x, y, z = [float(v) for v in gt_xyz]
    width, depth, height = [float(v) for v in gt_size]
    return {
        "xy": [x, y],
        "top_z": z + 0.5 * height,
        "support_z": z - 0.5 * height,
        "width": width,
        "depth": depth,
        "height": height,
    }


def measure_depth_world(depth_mm, width, height, k_tuple, rot_wo, trans_wo,
                        gt_xyz, gt_quat_xyzw, gt_size, stride=1):
    """Deproject 16UC1 mm depth and estimate top/support/XY vs eval GT."""
    mm = np.asarray(depth_mm)
    if mm.ndim != 2:
        mm = mm.reshape(int(height), int(width))
    rot_ow, trans_ow = optical_from_world_rt(rot_wo, trans_wo)
    centre, corners, corner_valid, _ = project_detection(
        gt_xyz, gt_quat_xyzw, gt_size, rot_ow, trans_ow, k_tuple)
    k = _K(*k_tuple)
    if not np.any(corner_valid):
        return {
            "ok": False,
            "reason": "gt_not_in_image",
            "bbox": None,
            "n_cargo": 0,
        }
    valid = corners[corner_valid]
    u0, v0 = valid.min(axis=0)
    u1, v1 = valid.max(axis=0)
    bbox = [float(u0), float(v0), float(u1), float(v1)]
    h, w = int(mm.shape[0]), int(mm.shape[1])

    def _roi(pad):
        ua = max(0, min(w - 1, int(math.floor(u0)) - pad))
        ub = max(0, min(w - 1, int(math.ceil(u1)) + pad))
        va = max(0, min(h - 1, int(math.floor(v0)) - pad))
        vb = max(0, min(h - 1, int(math.ceil(v1)) + pad))
        uu, vu = np.meshgrid(
            np.arange(ua, ub + 1, stride),
            np.arange(va, vb + 1, stride),
            indexing="xy",
        )
        optical, n = deproject_selected(mm, uu.reshape(-1), vu.reshape(-1), k)
        if n == 0:
            return np.zeros((0, 3), dtype=np.float64)
        return world_from_optical(optical, rot_wo, trans_wo)

    cargo = _roi(0)
    fx = max(1e-6, float(k.fx))
    pad = int(math.ceil(fx * 0.22 / 0.60)) + 4
    raw = _roi(pad)
    cfg = TopSupportConfig(min_support_points=80, crop_to_workspace=False)
    timing = {}
    top = estimate_top_surface(cargo, None, config=cfg, timing=timing)
    if top is None:
        return {
            "ok": False,
            "reason": "top_unobservable",
            "bbox": bbox,
            "n_cargo": int(len(cargo)),
            "timing": timing,
        }
    support = estimate_local_support(raw, top, None, config=cfg, timing=timing)
    gt = gt_planes(gt_xyz, gt_size)
    est_xy = [float(top.center_xy[0]), float(top.center_xy[1])]
    support_z = None if support is None else float(support.support_z)
    rec = {
        "ok": support is not None and support.reason == "ok",
        "reason": "ok" if support is not None and support.reason == "ok"
        else (support.reason if support is not None else "no_support"),
        "bbox": bbox,
        "n_cargo": int(len(cargo)),
        "n_raw": int(len(raw)),
        "est_xy": est_xy,
        "est_top_z": float(top.top_z),
        "est_support_z": support_z,
        "est_width": float(top.width),
        "est_depth": float(top.depth),
        "support_inliers": 0 if support is None else int(support.inlier_count),
        "gt": gt,
        "err_xy_m": math.hypot(est_xy[0] - gt["xy"][0], est_xy[1] - gt["xy"][1]),
        "err_top_m": abs(float(top.top_z) - gt["top_z"]),
        "err_support_m": (
            None if support_z is None else abs(support_z - gt["support_z"])),
        "used_camera_cloud": False,
        "tf_source": "stamp",
    }
    return rec


def score_three_pose(samples):
    """Apply Gate-4 XY/top/support limits across >=3 pose samples."""
    failures = []
    pose_names = sorted({str(s.get("pose_name") or "") for s in samples})
    if len([n for n in pose_names if n]) < 3:
        failures.append("need >=3 distinct arm poses, got %s" % pose_names)
    if any(bool(s.get("used_camera_cloud")) for s in samples):
        failures.append("Gazebo camera cloud is forbidden as oracle input")
    if any(str(s.get("tf_source") or "") != "stamp" for s in samples):
        failures.append("TF must be looked up at the image acquisition stamp")
    ok_rows = [s for s in samples if s.get("ok")]
    if len(ok_rows) < 3:
        failures.append("fewer than 3 successful pose samples")
    lim = GATE4_LIMITS
    top_stats = stats([s.get("err_top_m") for s in ok_rows])
    support_stats = stats([s.get("err_support_m") for s in ok_rows])
    xy_stats = stats([s.get("err_xy_m") for s in ok_rows])
    checks = (
        ("top_z_err_m", top_stats, (("p95", lim["top_z_p95_m"]),
                                    ("max", lim["top_z_max_m"]))),
        ("support_z_err_m", support_stats, (("p95", lim["support_z_p95_m"]),
                                            ("max", lim["support_z_max_m"]))),
        ("xy_err_m", xy_stats, (("p95", lim["xy_p95_m"]),)),
    )
    for name, stat, pairs in checks:
        for key, limit in pairs:
            value = stat.get(key)
            if value is None:
                failures.append("%s %s has no samples" % (name, key))
            elif value > limit:
                failures.append("%s %s %.4f > %.4f" % (name, key, value, limit))
    return {
        "pass": not failures,
        "failures": failures,
        "n_samples": len(samples),
        "n_ok": len(ok_rows),
        "pose_names": pose_names,
        "top_z_err_m": top_stats,
        "support_z_err_m": support_stats,
        "xy_err_m": xy_stats,
        "limits": {
            "top_z_p95_m": lim["top_z_p95_m"],
            "top_z_max_m": lim["top_z_max_m"],
            "support_z_p95_m": lim["support_z_p95_m"],
            "support_z_max_m": lim["support_z_max_m"],
            "xy_p95_m": lim["xy_p95_m"],
        },
    }
