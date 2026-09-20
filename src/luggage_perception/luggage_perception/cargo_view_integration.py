#!/usr/bin/env python3
"""ROS-free cargo-view integrate checks, downsample, and occupancy scoring.

Humble ``cargo_volume_mapper_node`` uses this for ``IntegrateCargoView``
fail-closed selection. No rclpy/tf2 imports.
"""

from __future__ import division

import time

from luggage_perception.luggage_box_estimator import voxel_downsample

SCHEMA_VERSION = 1

REASON_NOT_SETTLED = "NOT_SETTLED"
REASON_SCHEMA_MISMATCH = "SCHEMA_MISMATCH"
REASON_GEOMETRY_HASH_MISMATCH = "GEOMETRY_HASH_MISMATCH"
REASON_REVISION_MISMATCH = "REVISION_MISMATCH"
REASON_STAMP_STALE = "STAMP_STALE"
REASON_STAMP_ALREADY_INTEGRATED = "STAMP_ALREADY_INTEGRATED"
REASON_VIEW_EMPTY = "VIEW_EMPTY"
REASON_TF_MISSING = "TF_MISSING"

DEFAULT_FRESHNESS_WINDOW_SEC = 0.20


def stamp_to_sec(stamp):
    """``(sec, nanosec)``, mapping, or object with those fields -> float seconds."""
    if stamp is None:
        return 0.0
    if isinstance(stamp, (tuple, list)) and len(stamp) >= 2:
        return float(stamp[0]) + float(stamp[1]) * 1e-9
    if isinstance(stamp, dict):
        return float(stamp.get("sec", 0)) + float(stamp.get("nanosec", 0)) * 1e-9
    sec = getattr(stamp, "sec", 0)
    nanosec = getattr(stamp, "nanosec", 0)
    return float(sec) + float(nanosec) * 1e-9


def stamp_key(stamp):
    if stamp is None:
        return (0, 0)
    if isinstance(stamp, (tuple, list)) and len(stamp) >= 2:
        return (int(stamp[0]), int(stamp[1]))
    if isinstance(stamp, dict):
        return (int(stamp.get("sec", 0)), int(stamp.get("nanosec", 0)))
    return (int(getattr(stamp, "sec", 0)), int(getattr(stamp, "nanosec", 0)))


def validate_integrate_request(
        settled, schema_version, geometry_hash, expected_revision,
        current_hash, current_revision):
    """Return a reason code or ``None`` when the request may proceed."""
    try:
        version = int(schema_version)
    except (TypeError, ValueError):
        return REASON_SCHEMA_MISMATCH
    if version not in (0, SCHEMA_VERSION):
        return REASON_SCHEMA_MISMATCH
    if not bool(settled):
        return REASON_NOT_SETTLED
    want = str(geometry_hash or "")
    have = str(current_hash or "")
    if not want or want != have:
        return REASON_GEOMETRY_HASH_MISMATCH
    try:
        expected = int(expected_revision)
        current = int(current_revision)
    except (TypeError, ValueError):
        return REASON_REVISION_MISMATCH
    if expected != current:
        return REASON_REVISION_MISMATCH
    return None


def select_fresh_record(
        records, request_stamp_sec, window_sec, integrated_keys):
    """Pick the buffered cloud nearest ``request_stamp_sec`` inside the window.

    ``records`` are dicts with ``stamp_sec`` and ``stamp_key``. Returns
    ``(record, reason_code)``; ``record`` is None on failure.
    """
    window = max(0.0, float(window_sec))
    request_stamp_sec = float(request_stamp_sec)
    integrated = set(integrated_keys or ())
    best = None
    best_delta = None
    for record in records or ():
        delta = abs(float(record["stamp_sec"]) - request_stamp_sec)
        if delta > window:
            continue
        if best is None or delta < best_delta:
            best = record
            best_delta = delta
    if best is None:
        return None, REASON_STAMP_STALE
    key = tuple(best["stamp_key"])
    if key in integrated:
        return None, REASON_STAMP_ALREADY_INTEGRATED
    return best, None


def finite_points(points):
    """Nx3 float array with non-finite rows dropped. Empty input -> (0, 3)."""
    import numpy as np
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3) if (
        points is not None and len(points)) else np.zeros((0, 3), dtype=np.float64)
    if pts.shape[0] == 0:
        return pts
    return pts[np.isfinite(pts).all(axis=1)]


def prepare_points(points, voxel_size):
    """Finite-filter then voxel-downsample. Returns (kept Nx3, n_finite)."""
    finite = finite_points(points)
    n_finite = int(finite.shape[0])
    if n_finite == 0:
        return finite, 0
    kept = voxel_downsample(finite, float(voxel_size or 0.0))
    return kept, n_finite


def apply_integrate(mapper, points, origin, voxel_size=None):
    """Downsample ``points`` and call ``mapper.integrate_points``.

    Returns a dict with counts, latency, and revisions. Does not publish.
    """
    resolution = float(
        voxel_size if voxel_size is not None else mapper.resolution)
    t0 = time.monotonic()
    kept, n_finite = prepare_points(points, resolution)
    if n_finite == 0 or kept.shape[0] == 0:
        return {
            "ok": False,
            "reason_code": REASON_VIEW_EMPTY,
            "n_finite": n_finite,
            "n_kept": int(kept.shape[0]),
            "latency_sec": time.monotonic() - t0,
            "revision_before": int(mapper.stats()["map_revision"]),
            "revision_after": int(mapper.stats()["map_revision"]),
        }
    before = int(mapper.stats()["map_revision"])
    origin_tuple = None if origin is None else tuple(float(v) for v in origin)
    mapper.integrate_points(
        kept.tolist(), origin=origin_tuple)
    after = int(mapper.stats()["map_revision"])
    return {
        "ok": after > before,
        "reason_code": "" if after > before else REASON_VIEW_EMPTY,
        "n_finite": n_finite,
        "n_kept": int(kept.shape[0]),
        "latency_sec": time.monotonic() - t0,
        "revision_before": before,
        "revision_after": after,
    }


def cell_xy_container(surface, ix, iy):
    """Cell-center XY in the mapper base frame (Humble: container_link)."""
    origin = surface.get("origin_local") or [0.0, 0.0]
    res = float(surface.get("resolution") or 0.05)
    return (
        float(origin[0]) + (int(ix) + 0.5) * res,
        float(origin[1]) + (int(iy) + 0.5) * res,
    )


def footprint_sensor_coverage(surface, aabb_xy):
    """Fraction of AABB columns that are sensor-occupied.

    ``aabb_xy`` is ``(xmin, ymin, xmax, ymax)`` in the mapper base frame.
    """
    xmin, ymin, xmax, ymax = (float(v) for v in aabb_xy)
    nx = int(surface.get("nx") or 0)
    ny = int(surface.get("ny") or 0)
    state = surface.get("state") or []
    confidence = surface.get("confidence") or []
    total = 0
    hit = 0
    for ix in range(nx):
        for iy in range(ny):
            x, y = cell_xy_container(surface, ix, iy)
            if x < xmin or x > xmax or y < ymin or y > ymax:
                continue
            total += 1
            if (state[ix][iy] == "occupied"
                    and confidence[ix][iy] == "sensor"):
                hit += 1
    if total == 0:
        return 0.0, 0, 0
    return float(hit) / float(total), hit, total


def occupied_sensor_aabb(surface):
    """Axis-aligned XY bounds of sensor-occupied cells, or None."""
    nx = int(surface.get("nx") or 0)
    ny = int(surface.get("ny") or 0)
    state = surface.get("state") or []
    confidence = surface.get("confidence") or []
    xs = []
    ys = []
    res = float(surface.get("resolution") or 0.05)
    for ix in range(nx):
        for iy in range(ny):
            if (state[ix][iy] == "occupied"
                    and confidence[ix][iy] == "sensor"):
                x, y = cell_xy_container(surface, ix, iy)
                xs.append(x)
                ys.append(y)
    if not xs:
        return None
    half = 0.5 * res
    return (
        min(xs) - half, min(ys) - half,
        max(xs) + half, max(ys) + half,
    )


def _aabb_xy_overlap(a, b, tolerance=1e-9):
    return (
        a[0] < b[2] - tolerance and a[2] > b[0] + tolerance
        and a[1] < b[3] - tolerance and a[3] > b[1] + tolerance
    )


def floor_through_count(surface, box_size, occupied_xy, peak_tol=0.05):
    """Count feasible floor candidates whose footprint overlaps ``occupied_xy``.

    ``occupied_xy`` is ``(xmin, ymin, xmax, ymax)``. Uses live
    ``solve_placement`` so the occupancy-used proof does not call the
    production service with synthetic GT.
    """
    from luggage_packing.placement_solver import solve_placement

    if occupied_xy is None:
        return 0, {"message": "no occupied patch"}
    inner_h = float((surface.get("inner_size") or [0, 0, 1.5])[2])
    result = solve_placement(
        surface, box_size, allowed_yaws=[0.0],
        params={"top_n": 400, "keep_rejected": 80})
    count = 0
    for cand in result.get("candidates") or ():
        if not cand.get("feasible"):
            continue
        local = cand.get("center_local") or [0.0, 0.0, 0.0]
        peak = float(local[2]) + 0.5 * inner_h - 0.5 * float(box_size[2])
        if peak > peak_tol:
            continue
        footprint = cand.get("footprint") or cand["size"][:2]
        half_l = float(footprint[0]) * 0.5
        half_w = float(footprint[1]) * 0.5
        cand_xy = (
            float(local[0]) - half_l, float(local[1]) - half_w,
            float(local[0]) + half_l, float(local[1]) + half_w,
        )
        if _aabb_xy_overlap(cand_xy, occupied_xy):
            count += 1
    return count, {
        "success": bool(result.get("success")),
        "n_feasible": sum(
            1 for c in (result.get("candidates") or ()) if c.get("feasible")),
        "n_floor_through": count,
        "reason_code": result.get("reason_code") or "",
        "message": result.get("message") or "",
        "synthetic_plan_only": True,
    }


def untracked_xyz(cargo, obstacle, include_obstacle):
    """Concatenate label-filtered cargo with optional obstacle. No tracker."""
    import numpy as np
    cargo_pts = finite_points(cargo)
    if not include_obstacle:
        return cargo_pts
    obs = finite_points(obstacle)
    if cargo_pts.shape[0] == 0:
        return obs
    if obs.shape[0] == 0:
        return cargo_pts
    return np.concatenate([cargo_pts, obs], axis=0)
