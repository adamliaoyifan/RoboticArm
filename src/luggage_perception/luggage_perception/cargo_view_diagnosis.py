#!/usr/bin/env python3
"""Boundary reasons for one post-place cargo view. No ROS."""

from __future__ import division

import math

REASON_TARGET_NOT_IN_FRAME = "TARGET_NOT_IN_FRAME"
REASON_TF_TIME_MISMATCH = "TF_TIME_MISMATCH"
REASON_DEPTH_INVALID = "DEPTH_INVALID"
REASON_BOX_CROP_EMPTY = "BOX_CROP_EMPTY"
REASON_MAP_FILTERED = "MAP_FILTERED"

EMPTY_REASONS = (
    REASON_TARGET_NOT_IN_FRAME,
    REASON_TF_TIME_MISMATCH,
    REASON_DEPTH_INVALID,
    REASON_BOX_CROP_EMPTY,
    REASON_MAP_FILTERED,
)

MIN_VALID_DEPTH = 30
MIN_BOX_POINTS = 30
TF_FRESHNESS_SEC = 0.20


def project_box_corners(corners_cam, fx, fy, cx, cy, width, height):
    """Project camera-frame corners. Returns pixels, in-frame count, behind flag.

    A corner with non-positive z is behind the camera and is not in frame.
    ``behind_camera`` is true only when every corner is behind.
    """
    pixels = []
    in_frame = 0
    behind = 0
    n = 0
    for corner in corners_cam or ():
        n += 1
        x, y, z = (float(v) for v in corner[:3])
        if (not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z))
                or z <= 1e-6):
            behind += 1
            pixels.append(None)
            continue
        u = float(fx) * x / z + float(cx)
        v = float(fy) * y / z + float(cy)
        pixels.append((u, v))
        if 0.0 <= u < float(width) and 0.0 <= v < float(height):
            in_frame += 1
    return {
        "pixels": pixels,
        "in_frame": int(in_frame),
        "n": int(n),
        "in_frame_fraction": (float(in_frame) / float(n)) if n else 0.0,
        "behind_camera": bool(n and behind == n),
    }


def _polygon_mask(height, width, polygon):
    """Boolean mask of pixels inside ``polygon`` (list of (u, v))."""
    import numpy as np
    pts = [
        (float(u), float(v)) for u, v in (polygon or ())
        if u is not None and math.isfinite(float(u)) and math.isfinite(float(v))]
    mask = np.zeros((int(height), int(width)), dtype=bool)
    if len(pts) < 3 or int(height) <= 0 or int(width) <= 0:
        return mask
    us = [p[0] for p in pts]
    vs = [p[1] for p in pts]
    u0 = max(0, int(math.floor(min(us))))
    u1 = min(int(width) - 1, int(math.ceil(max(us))))
    v0 = max(0, int(math.floor(min(vs))))
    v1 = min(int(height) - 1, int(math.ceil(max(vs))))
    if u1 < u0 or v1 < v0:
        return mask
    uu = np.arange(u0, u1 + 1, dtype=np.float64)
    vv = np.arange(v0, v1 + 1, dtype=np.float64)
    grid_u, grid_v = np.meshgrid(uu, vv)
    inside = np.zeros(grid_u.shape, dtype=bool)
    j = len(pts) - 1
    for i in range(len(pts)):
        ui, vi = pts[i]
        uj, vj = pts[j]
        crosses = (vi > grid_v) != (vj > grid_v)
        denom = (vj - vi) if abs(vj - vi) > 1e-12 else 1e-12
        x_cross = (uj - ui) * (grid_v - vi) / denom + ui
        inside ^= crosses & (grid_u < x_cross)
        j = i
    mask[v0:v1 + 1, u0:u1 + 1] = inside
    return mask


def region_depth_stats(depth_mm, uv_polygon):
    """Valid, invalid, and zero counts plus range inside ``uv_polygon``.

    ``depth_mm`` is a 2D image. Valid pixels are finite and strictly positive.
    Range values are metres.
    """
    import numpy as np
    depth = np.asarray(depth_mm)
    empty = {
        "n_valid": 0, "n_invalid": 0, "n_zero": 0,
        "min_m": None, "median_m": None, "max_m": None,
    }
    if depth.ndim != 2 or depth.size == 0:
        return empty
    mask = _polygon_mask(depth.shape[0], depth.shape[1], uv_polygon)
    if not bool(mask.any()):
        return empty
    values = np.asarray(depth[mask], dtype=np.float64)
    finite = np.isfinite(values)
    positive = finite & (values > 0.0)
    zero = finite & (values == 0.0)
    valid = values[positive] * 0.001
    stats = {
        "n_valid": int(positive.sum()),
        "n_invalid": int((~finite).sum()),
        "n_zero": int(zero.sum()),
        "min_m": None, "median_m": None, "max_m": None,
    }
    if valid.size:
        stats["min_m"] = float(valid.min())
        stats["max_m"] = float(valid.max())
        stats["median_m"] = float(np.median(valid))
    return stats


def rim_occlusion_fraction(region_depth_m, expected_lid_range_m, margin):
    """Share of valid ranges nearer than the lid by more than ``margin``."""
    import numpy as np
    if region_depth_m is None:
        ranges = np.zeros((0,), dtype=np.float64)
    else:
        ranges = np.asarray(region_depth_m, dtype=np.float64).reshape(-1)
    ranges = ranges[np.isfinite(ranges) & (ranges > 0.0)]
    if ranges.size == 0:
        return None
    try:
        limit = float(expected_lid_range_m) - float(margin)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(limit):
        return None
    return float(np.mean(ranges < limit))


def classify_empty_view(diag, min_valid_depth=MIN_VALID_DEPTH,
                        min_box_points=MIN_BOX_POINTS,
                        tf_freshness_sec=TF_FRESHNESS_SEC):
    """One ordered reason for a view that put no sensor cell in the footprint.

    ``semantic_mask_empty`` is recorded by the caller and is not a reason:
    the integrate cloud is the box-cropped depth, not the mask.
    """
    info = dict(diag or {})
    if (float(info.get("in_frame_fraction") or 0.0) <= 0.0
            or bool(info.get("behind_camera"))):
        return REASON_TARGET_NOT_IN_FRAME
    stamped_ok = bool(info.get("tf_stamped_ok"))
    used_latest = bool(info.get("tf_used_latest"))
    try:
        age = float(info.get("tf_fallback_age_sec") or 0.0)
    except (TypeError, ValueError):
        age = 0.0
    if (not stamped_ok) and (not used_latest or age > float(tf_freshness_sec)):
        return REASON_TF_TIME_MISMATCH
    try:
        n_valid = int(info.get("n_valid_depth_in_footprint") or 0)
    except (TypeError, ValueError):
        n_valid = 0
    if n_valid < int(min_valid_depth):
        return REASON_DEPTH_INVALID
    try:
        n_kept = int(info.get("n_kept") or 0)
    except (TypeError, ValueError):
        n_kept = 0
    if n_kept < int(min_box_points):
        return REASON_BOX_CROP_EMPTY
    return REASON_MAP_FILTERED
