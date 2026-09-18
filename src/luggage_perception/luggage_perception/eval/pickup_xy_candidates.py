"""Pickup XY strategy candidates from a site-pick replay. ROS-free.

Emits the per-frame world-XY candidate file consumed by
``pickup_xy_benchmark`` (the real-deployment gate: median <= 30 mm,
P95 <= 60 mm, >= 80% improvement over the PCA baseline). The four
strategies reuse what one replay frame already produced — the detector's
top-plane estimate, the kept YOLO bbox, the cargo cloud in the world
frame — so emitting candidates costs nothing beyond the replay itself.

A strategy that cannot be computed honestly for a frame is ``null``:
a missing point is reportable, a guessed point is not.
"""
from __future__ import division

import json
import os

import numpy as np

STRATEGY_PCA = "pca_center"
STRATEGY_BBOX = "yolo_bbox_center_top_plane"
STRATEGY_LID = "lid_inlier_center"
STRATEGY_BLEND = "robust_blended_center"
STRATEGIES = (STRATEGY_PCA, STRATEGY_BBOX, STRATEGY_LID, STRATEGY_BLEND)

# Robust blend needs at least two independent opinions; a blend of one
# value is just that value wearing a different name.
_MIN_BLEND_SOURCES = 2


def bbox_center_pixel(kept_detection):
    """(u, v) centre of the kept YOLO bbox, or None."""
    if not kept_detection:
        return None
    x1, y1, x2, y2 = kept_detection["bbox"]
    return (float(x1) + float(x2)) / 2.0, (float(y1) + float(y2)) / 2.0


def median_depth_window(depth, u, v, half=2):
    """Median non-zero depth (metres) in the (2*half+1)^2 window, or None.

    The window median replaces the single-pixel depth: one bad pixel
    would displace the candidate by centimetres.
    """
    if depth is None:
        return None
    arr = np.asarray(depth)
    if arr.ndim != 2:
        return None
    height, width = arr.shape
    u = int(round(float(u)))
    v = int(round(float(v)))
    u0, u1 = max(0, u - half), min(width, u + half + 1)
    v0, v1 = max(0, v - half), min(height, v + half + 1)
    if u0 >= u1 or v0 >= v1:
        return None
    window = arr[v0:v1, u0:u1]
    nonzero = window[window > 0]
    if nonzero.size == 0 or not np.isfinite(nonzero).all():
        return None
    return float(np.median(nonzero)) * 0.001


def deproject_pinhole(u, v, z_m, intrinsics):
    """Optical-frame point for pixel (u, v) at depth z_m.

    Same convention as depth_deprojection.deproject_selected (plain
    pinhole, no distortion term — the colour-aligned depth grid).
    """
    z = float(z_m)
    return np.array([
        (float(u) - float(intrinsics.cx)) * z / float(intrinsics.fx),
        (float(v) - float(intrinsics.cy)) * z / float(intrinsics.fy),
        z], dtype=np.float64)


def lid_inlier_center(pts_world, top_z, dist_thresh, min_points=10):
    """World-XY mean of the top-plane inliers, or None.

    The detector's internal RANSAC inliers are not exported; threshold
    ``|z - top_z| <= dist_thresh`` around the RETURNED top_z is
    deterministic and avoids re-running RANSAC.
    """
    if pts_world is None:
        return None
    pts = np.asarray(pts_world, dtype=np.float64)
    if pts.ndim != 2 or not len(pts):
        return None
    inliers = np.abs(pts[:, 2] - float(top_z)) <= float(dist_thresh)
    if int(inliers.sum()) < int(min_points):
        return None
    xy = pts[inliers][:, :2].mean(axis=0)
    return [float(xy[0]), float(xy[1])]


def strategies_for_frame(kept_detection, depth, intrinsics, pts_world,
                         detector_result, ransac_dist_thresh,
                         tf_points_fn=None):
    """{strategy: [x, y] | None} in the world frame for one replay frame.

    ``tf_points_fn`` maps an (N,3) optical-frame array to world points
    (the replay's recorded-TF transform with the frame's interpolation
    settings) or None on a TF miss.
    """
    out = {name: None for name in STRATEGIES}
    top = None
    box = getattr(detector_result, "box", None) if (
        detector_result is not None) else None
    top = getattr(box, "top", None) if box is not None else None

    if top is not None and top.center_xy is not None:
        # The live pipeline's current output — the benchmark baseline.
        out[STRATEGY_PCA] = [float(top.center_xy[0]),
                             float(top.center_xy[1])]

    if (kept_detection is not None and depth is not None
            and intrinsics is not None and tf_points_fn is not None):
        uv = bbox_center_pixel(kept_detection)
        z_m = median_depth_window(depth, uv[0], uv[1]) if uv else None
        if uv and z_m is not None and z_m > 0.05:
            optical = deproject_pinhole(uv[0], uv[1], z_m, intrinsics)
            world = tf_points_fn(optical.reshape(1, 3))
            if world is not None and len(world):
                out[STRATEGY_BBOX] = [float(world[0][0]),
                                       float(world[0][1])]

    if top is not None:
        out[STRATEGY_LID] = lid_inlier_center(
            pts_world, float(top.top_z), ransac_dist_thresh)

    available = [value for name, value in out.items()
                 if name != STRATEGY_BLEND and value is not None]
    if len(available) >= _MIN_BLEND_SOURCES:
        out[STRATEGY_BLEND] = [
            float(np.median([point[0] for point in available])),
            float(np.median([point[1] for point in available])),
        ]
    return out


def candidates_frames(bag_name, bag_path, frame_rows):
    """Benchmark-shaped frame rows from replay frame rows.

    ``frame_id`` is ``<bag>@<stamp>`` so labels keyed either by explicit
    frame_id or by bag+stamp join the same row.
    """
    frames = []
    for row in frame_rows:
        strategies = row.get("pickup_candidates")
        if not strategies or not any(
                value is not None for value in strategies.values()):
            continue
        stamp = float(row["stamp_sec"])
        frames.append({
            "frame_id": "%s@%.9f" % (bag_name, stamp),
            "bag_path": str(bag_path),
            "stamp": stamp,
            "strategies": strategies,
        })
    return frames


def write_candidates_file(path, bag_name, bag_path, frame_rows,
                          provenance):
    """Write the candidates JSON; returns the frame count."""
    frames = candidates_frames(bag_name, bag_path, frame_rows)
    payload = {
        "generator": dict(provenance or {}),
        "bag_path": str(bag_path),
        "frames": frames,
    }
    path = os.path.abspath(os.path.expanduser(str(path)))
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return len(frames)
