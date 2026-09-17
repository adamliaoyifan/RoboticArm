"""Eval-only Livox monitoring metrics. Not a join member of RGB-D.

Independent of D555 depth. Sim raster is configured 360x32 at 10 Hz with
``deskewed=false``. Real Mid-360 bag numbers are diagnostic only.
"""
from __future__ import division

import math

import numpy as np


CONFIGURED_H_SAMPLES = 360
CONFIGURED_V_SAMPLES = 32
CONFIGURED_HZ = 10.0
DESKEWED = False
NN_SAMPLE = 2000
PLANE_SAMPLE = 4000
PLANE_RANSAC_ITERS = 40
PLANE_DIST_THRESH_M = 0.04


def xyz_from_structured(data, point_step, n_points, x_off=0, y_off=4, z_off=8):
    """Decode XYZ float32 fields from a packed PointCloud2 payload."""
    n_points = int(n_points)
    point_step = int(point_step)
    if n_points <= 0 or point_step <= 0:
        return np.zeros((0, 3), dtype=np.float64)
    raw = np.ascontiguousarray(np.frombuffer(data, dtype=np.uint8))
    need = n_points * point_step
    if raw.size < need:
        n_points = int(raw.size // point_step)
        need = n_points * point_step
        raw = raw[:need]
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": ["<f4", "<f4", "<f4"],
        "offsets": [int(x_off), int(y_off), int(z_off)],
        "itemsize": point_step,
    })
    arr = np.frombuffer(raw, dtype=dtype, count=n_points)
    return np.stack(
        [arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64)


def intensity_from_structured(data, point_step, n_points, offset, field_size=4):
    if offset is None or n_points <= 0:
        return None
    raw = np.frombuffer(data, dtype=np.uint8)
    need = int(n_points) * int(point_step)
    if raw.size < need:
        n_points = int(raw.size // point_step)
        raw = raw[:n_points * int(point_step)]
    if int(field_size) == 1:
        fmt = "u1"
    elif int(field_size) == 2:
        fmt = "<u2"
    else:
        fmt = "<f4"
    dtype = np.dtype({
        "names": ["i"],
        "formats": [fmt],
        "offsets": [int(offset)],
        "itemsize": int(point_step),
    })
    arr = np.frombuffer(np.ascontiguousarray(raw), dtype=dtype, count=int(n_points))
    return arr["i"].astype(np.float64)


def finite_xyz(xyz):
    pts = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    if pts.size == 0:
        return pts
    mask = np.isfinite(pts).all(axis=1)
    return pts[mask]


def range_stats(xyz):
    pts = finite_xyz(xyz)
    if len(pts) == 0:
        return {"count": 0, "p50": None, "p95": None, "max": None}
    rng = np.linalg.norm(pts, axis=1)
    ordered = np.sort(rng)
    return {
        "count": int(len(ordered)),
        "p50": float(ordered[int(0.50 * (len(ordered) - 1))]),
        "p95": float(ordered[int(0.95 * (len(ordered) - 1))]),
        "max": float(ordered[-1]),
    }


def nearest_neighbor_spacing(xyz, max_points=NN_SAMPLE, rng=None):
    pts = finite_xyz(xyz)
    if len(pts) < 2:
        return {"n": int(len(pts)), "mean": None, "p50": None, "p95": None}
    if len(pts) > int(max_points):
        rng = np.random.RandomState(0) if rng is None else rng
        idx = rng.choice(len(pts), int(max_points), replace=False)
        pts = pts[idx]
    n = len(pts)
    mins = np.empty(n, dtype=np.float64)
    for i in range(n):
        delta = pts - pts[i]
        dist = np.sqrt(np.einsum("ij,ij->i", delta, delta))
        dist[i] = np.inf
        mins[i] = dist.min()
    ordered = np.sort(mins)
    return {
        "n": int(n),
        "mean": float(ordered.mean()),
        "p50": float(ordered[int(0.50 * (n - 1))]),
        "p95": float(ordered[int(0.95 * (n - 1))]),
    }


def _fit_plane(pts):
    centroid = pts.mean(axis=0)
    _, _, vh = np.linalg.svd(pts - centroid, full_matrices=False)
    normal = vh[-1]
    norm = np.linalg.norm(normal)
    if not np.isfinite(norm) or norm < 1e-9:
        return None, None
    normal = normal / norm
    offset = float(np.dot(normal, centroid))
    return normal, offset


def dominant_plane_residuals(xyz, max_points=PLANE_SAMPLE, max_iter=PLANE_RANSAC_ITERS,
                             dist_thresh=PLANE_DIST_THRESH_M, rng=None):
    pts = finite_xyz(xyz)
    if len(pts) < 10:
        return {
            "n": int(len(pts)), "inliers": 0, "inlier_ratio": None,
            "residual_mean": None, "residual_std": None,
        }
    rng = np.random.RandomState(1) if rng is None else rng
    if len(pts) > int(max_points):
        pts = pts[rng.choice(len(pts), int(max_points), replace=False)]
    best_mask = None
    n = len(pts)
    for _ in range(int(max_iter)):
        idx = rng.choice(n, 3, replace=False)
        sample = pts[idx]
        normal, offset = _fit_plane(sample)
        if normal is None:
            continue
        resid = np.abs(pts.dot(normal) - offset)
        mask = resid <= float(dist_thresh)
        if best_mask is None or mask.sum() > best_mask.sum():
            best_mask = mask
    if best_mask is None or best_mask.sum() < 3:
        return {
            "n": int(n), "inliers": 0, "inlier_ratio": 0.0,
            "residual_mean": None, "residual_std": None,
        }
    inliers = pts[best_mask]
    normal, offset = _fit_plane(inliers)
    if normal is None:
        resid = np.zeros(len(inliers))
    else:
        resid = np.abs(inliers.dot(normal) - offset)
    return {
        "n": int(n),
        "inliers": int(best_mask.sum()),
        "inlier_ratio": float(best_mask.mean()),
        "residual_mean": float(resid.mean()),
        "residual_std": float(resid.std()),
    }


def intensity_stats(values):
    if values is None:
        return {"present": False}
    vals = np.asarray(values, dtype=np.float64).reshape(-1)
    vals = vals[np.isfinite(vals)]
    if len(vals) == 0:
        return {"present": True, "count": 0, "mean": None, "p50": None}
    ordered = np.sort(vals)
    return {
        "present": True,
        "count": int(len(ordered)),
        "mean": float(ordered.mean()),
        "p50": float(ordered[int(0.50 * (len(ordered) - 1))]),
        "min": float(ordered[0]),
        "max": float(ordered[-1]),
    }


def summarize_scan(xyz, intensity=None, frame_id="", n_raw=None, heavy=True):
    pts = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    n_raw = int(n_raw if n_raw is not None else len(pts))
    finite = finite_xyz(pts)
    rec = {
        "frame_id": str(frame_id or ""),
        "n_raw": n_raw,
        "n_finite": int(len(finite)),
        "finite_ratio": (float(len(finite)) / n_raw) if n_raw else None,
        "range": range_stats(finite),
        "nn_spacing_m": None,
        "dominant_plane": None,
        "intensity": intensity_stats(intensity),
        "configured_grid": {
            "h_samples": CONFIGURED_H_SAMPLES,
            "v_samples": CONFIGURED_V_SAMPLES,
            "points_expected": CONFIGURED_H_SAMPLES * CONFIGURED_V_SAMPLES,
            "update_hz": CONFIGURED_HZ,
        },
        "deskewed": DESKEWED,
        "heavy": bool(heavy),
    }
    if heavy:
        rec["nn_spacing_m"] = nearest_neighbor_spacing(finite)
        rec["dominant_plane"] = dominant_plane_residuals(finite)
    return rec


def summarize_window(scan_summaries, stamps_sec):
    """Aggregate per-scan summaries over a scored window. Not RGB-D joined."""
    n = len(scan_summaries)
    stamps = [float(s) for s in stamps_sec]
    rate = None
    if len(stamps) >= 2:
        dt = max(1e-6, stamps[-1] - stamps[0])
        rate = (len(stamps) - 1) / dt
    def _col(path):
        out = []
        for rec in scan_summaries:
            cur = rec
            for key in path:
                if not isinstance(cur, dict):
                    cur = None
                    break
                cur = cur.get(key)
            if cur is not None and math.isfinite(float(cur)):
                out.append(float(cur))
        return out

    def _med(vals):
        if not vals:
            return None
        s = sorted(vals)
        return s[len(s) // 2]

    return {
        "n_scans": n,
        "rate_hz": rate,
        "points_per_scan_median": _med(_col(("n_raw",))),
        "finite_ratio_median": _med(_col(("finite_ratio",))),
        "range_p50_median": _med(_col(("range", "p50"))),
        "range_p95_median": _med(_col(("range", "p95"))),
        "range_max_median": _med(_col(("range", "max"))),
        "nn_mean_median": _med(_col(("nn_spacing_m", "mean"))),
        "plane_inlier_ratio_median": _med(_col(("dominant_plane", "inlier_ratio"))),
        "plane_residual_mean_median": _med(_col(("dominant_plane", "residual_mean"))),
        "plane_residual_std_median": _med(_col(("dominant_plane", "residual_std"))),
        "intensity_present": any(
            (rec.get("intensity") or {}).get("present") for rec in scan_summaries),
        "configured_grid": {
            "h_samples": CONFIGURED_H_SAMPLES,
            "v_samples": CONFIGURED_V_SAMPLES,
            "points_expected": CONFIGURED_H_SAMPLES * CONFIGURED_V_SAMPLES,
            "update_hz": CONFIGURED_HZ,
        },
        "deskewed": DESKEWED,
        "note": "independent of RGB-D; not exact-paired; raster sim not Mid-360 20k",
    }
