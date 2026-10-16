#!/usr/bin/env python3
"""ROS-free cargo-view integrate checks, downsample, and occupancy scoring.

Humble ``cargo_volume_mapper_node`` uses this for ``IntegrateCargoView``
fail-closed selection. No rclpy/tf2 imports.
"""

from __future__ import division

import math
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
# B1 occupancy-map store reasons (apply_update transaction outcomes).
# Premise mismatches reuse the specific existing codes above (revision) plus
# EPOCH_MISMATCH / CONTENT_DIGEST_MISMATCH below; there is deliberately no
# generic VERSION_CONFLICT — callers must see WHICH premise leg failed.
REASON_OPERATION_ID_CONFLICT = "OPERATION_ID_CONFLICT"
REASON_EPOCH_MISMATCH = "EPOCH_MISMATCH"
REASON_CONTENT_DIGEST_MISMATCH = "CONTENT_DIGEST_MISMATCH"
REASON_EVIDENCE_MISSING = "EVIDENCE_MISSING"
REASON_EVIDENCE_NOT_FINITE = "EVIDENCE_NOT_FINITE"
REASON_EVIDENCE_ANCHOR_MISMATCH = "EVIDENCE_ANCHOR_MISMATCH"
REASON_LATE_FACT_RECORDED_PENDING = "LATE_FACT_RECORDED_PENDING"
REASON_NO_MATCHING_BOX = "NO_MATCHING_BOX"
REASON_INVALID_PAYLOAD = "INVALID_PAYLOAD"

DEFAULT_FRESHNESS_WINDOW_SEC = 0.20

#: Map identity the selector admits. ``static_empty`` is the configured
#: empty container (no placed boxes, no observation). A settled view is
#: ``measured``. A placed-box commit is ``release_integrated``.
MAP_STATIC_EMPTY = "static_empty"
MAP_MEASURED = "measured"
MAP_RELEASE_INTEGRATED = "release_integrated"
MAP_SOURCES = (MAP_STATIC_EMPTY, MAP_MEASURED, MAP_RELEASE_INTEGRATED)


def measured_identity(sensor_stamp_ns):
    """A settled view. Stamp 0 stays 0; it is not filled from the clock."""
    return int(sensor_stamp_ns or 0), MAP_MEASURED


def release_identity(node_now_ns):
    """A placed-box commit. The stamp is the node clock, or 0 if it has
    not started. A zero stamp does not pass the acquisition-age gate."""
    now = int(node_now_ns or 0)
    return (now if now > 0 else 0), MAP_RELEASE_INTEGRATED


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


OCCUPANCY_CHANGE_EVENTS = (
    "box_placed",
    "box_removed",
    "box_disturbed",
    "cargo_edited",
    "container_pose_changed",
)
#: Motion around the container does not edit the cargo that was integrated.
NON_INVALIDATING_EVENTS = (
    "return",
    "pickup_spawn",
    "grasp",
    "planning",
)


def footprint_sensor_hits(surface, center, size, yaw):
    """Sensor-occupied cells whose centers lie in the planned box footprint.

    Z is the box center, so the test is the oriented XY footprint. Cells of
    an earlier box outside this footprint are not counted.

    This reads the accumulated map, so an old sensor cell inside the
    footprint counts. It is a diagnostic for the evidence bundle only;
    confirmation uses ``new_box_evidence_points`` on the view's own points.
    """
    if not surface:
        return 0
    nx = int(surface.get("nx") or 0)
    ny = int(surface.get("ny") or 0)
    state = surface.get("state") or []
    confidence = surface.get("confidence") or []
    points = []
    cz = float(center[2])
    for ix in range(nx):
        if ix >= len(state):
            continue
        for iy in range(ny):
            if iy >= len(state[ix]):
                continue
            if (state[ix][iy] != "occupied"
                    or iy >= len(confidence[ix])
                    or confidence[ix][iy] != "sensor"):
                continue
            x, y = cell_xy_container(surface, ix, iy)
            points.append((x, y, cz))
    if not points:
        return 0
    mask = box_point_mask(points, center, size, yaw=yaw, margin=0.0)
    return int(sum(1 for flag in mask if flag))


#: A placed box is evidenced by points of the integrating view that lie on
#: its upper half. XY slack covers the 0.05 m commit tolerance's inner
#: half; Z slack covers top-face noise. Points of a box underneath sit
#: below the new box's centre plane, so stacking cannot borrow them.
EVIDENCE_XY_MARGIN_M = 0.03
EVIDENCE_Z_MARGIN_M = 0.03
MIN_EVIDENCE_POINTS = 3


def new_box_evidence_points(points, center, size, yaw):
    """Count of this view's points on the upper half of a placed box.

    ``points`` are the points the integration itself observed (container
    frame), never the accumulated map. This is evidence that the new box
    is present at its committed pose, not a measurement of its full
    geometry; geometry stays the committed ledger record.
    """
    # Z range [centre, top + margin]: the upper half only.
    upper = 0.5 * float(size[2]) + EVIDENCE_Z_MARGIN_M
    sub_center = (float(center[0]), float(center[1]),
                  float(center[2]) + 0.5 * upper)
    sub_size = (float(size[0]) + 2.0 * EVIDENCE_XY_MARGIN_M,
                float(size[1]) + 2.0 * EVIDENCE_XY_MARGIN_M,
                upper)
    mask = box_point_mask(points, sub_center, sub_size, yaw=yaw, margin=0.0)
    return int(sum(1 for flag in mask if flag))


def container_pose_moved(before, after, tol_m=0.01, tol_rad=0.02):
    """True when the container pose differs enough to need reconfirmation."""
    if not before or not after:
        return False
    translation = [
        abs(float(a) - float(b))
        for a, b in zip(before.get("translation") or [],
                        after.get("translation") or [])]
    rotation = [
        abs(float(a) - float(b))
        for a, b in zip(before.get("rotation_rpy") or [],
                        after.get("rotation_rpy") or [])]
    return any(delta > float(tol_m) for delta in translation) or any(
        delta > float(tol_rad) for delta in rotation)


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
        # What this view itself observed; confirmation reads this, never
        # the accumulated map.
        "observed_points": kept,
    }


def cell_xy_container(surface, ix, iy):
    """Cell-center XY in the mapper base frame (Humble: container_link)."""
    origin = surface.get("origin_local") or [0.0, 0.0]
    res = float(surface.get("resolution") or 0.05)
    return (
        float(origin[0]) + (int(ix) + 0.5) * res,
        float(origin[1]) + (int(iy) + 0.5) * res,
    )


def _column_height_grid(surface):
    """Floor-relative column tops. v2 ``height_above_floor`` wins over v1 ``height``."""
    if "height_above_floor" in (surface or {}):
        return surface.get("height_above_floor") or []
    return (surface or {}).get("height") or []


def _column_height_digest(surface, cells_i, cells_j, cell_states):
    """Digest of occupied column tops. Empty when the grid has no heights.

    A partial grid is also empty: a digest that silently drops cells
    would make two different maps look identical.
    """
    from st09_contracts._base import digest_of

    heights = _column_height_grid(surface)
    if not heights:
        return ""
    triples = []
    for index, state in enumerate(cell_states):
        if str(state) != "occupied":
            continue
        row = int(cells_i[index])
        column = int(cells_j[index])
        try:
            value = float(heights[row][column])
        except (IndexError, TypeError, ValueError):
            return ""
        triples.append((row, column, round(value, 6)))
    if not triples:
        return ""
    return digest_of(tuple(triples))


def footprint_sensor_height(surface, aabb_xy):
    """Median floor-relative height of sensor-occupied cells inside ``aabb_xy``.

    Returns ``None`` when the footprint has no sensor-occupied cell. The
    value is metres above the inner floor (the same datum as a suitcase
    resting on that floor).
    """
    xmin, ymin, xmax, ymax = (float(v) for v in aabb_xy)
    nx = int(surface.get("nx") or 0)
    ny = int(surface.get("ny") or 0)
    state = surface.get("state") or []
    confidence = surface.get("confidence") or []
    heights = _column_height_grid(surface)
    kept = []
    for ix in range(nx):
        if ix >= len(state) or ix >= len(heights):
            continue
        for iy in range(ny):
            if iy >= len(state[ix]) or iy >= len(heights[ix]):
                continue
            x, y = cell_xy_container(surface, ix, iy)
            if x < xmin or x > xmax or y < ymin or y > ymax:
                continue
            if (state[ix][iy] != "occupied"
                    or confidence[ix][iy] != "sensor"):
                continue
            value = heights[ix][iy]
            if value is None:
                continue
            try:
                number = float(value)
            except (TypeError, ValueError):
                continue
            if math.isfinite(number):
                kept.append(number)
    if not kept:
        return None
    ordered = sorted(kept)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return float(ordered[mid])
    return 0.5 * (float(ordered[mid - 1]) + float(ordered[mid]))


def optical_axis_angle_rad(cam_xyz, quat_xyzw, target_xyz):
    """Angle between camera optical +Z and the vector from the camera to target.

    Returns ``(angle_rad, distance_m)``. ``angle_rad`` is ``None`` when the
    target sits on the camera. Identity quaternion looks along world +Z.
    """
    qx, qy, qz, qw = [float(v) for v in quat_xyzw]
    axis = (
        2.0 * (qx * qz + qy * qw),
        2.0 * (qy * qz - qx * qw),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )
    delta = [float(target_xyz[i]) - float(cam_xyz[i]) for i in range(3)]
    distance = math.sqrt(sum(v * v for v in delta))
    if distance < 1e-9:
        return None, 0.0
    dot = sum(axis[i] * delta[i] for i in range(3)) / distance
    dot = max(-1.0, min(1.0, dot))
    return math.acos(dot), distance


def box_point_mask(points, center, size, yaw=0.0, margin=0.0):
    """Mask of ``points`` that lie inside an oriented box. Never invents points.

    ``center`` and ``points`` share one frame. ``size`` is width, depth, height
    along the box axes; ``yaw`` rotates that box about +Z. ``margin`` expands
    every half-extent. A cloud that misses the box yields an all-false mask.
    """
    import numpy as np
    raw = [] if points is None else points
    pts = np.asarray(raw, dtype=np.float64)
    if pts.size == 0:
        return np.zeros((0,), dtype=bool)
    pts = pts.reshape(-1, 3)
    try:
        origin = [float(v) for v in center]
        extents = [float(v) for v in size]
        turn = float(yaw)
        pad = max(0.0, float(margin))
    except (TypeError, ValueError):
        return np.zeros((pts.shape[0],), dtype=bool)
    if (len(origin) != 3 or len(extents) != 3
            or not all(math.isfinite(v) for v in origin + extents + [turn, pad])
            or any(v <= 0.0 for v in extents)):
        return np.zeros((pts.shape[0],), dtype=bool)
    delta_x = pts[:, 0] - origin[0]
    delta_y = pts[:, 1] - origin[1]
    delta_z = pts[:, 2] - origin[2]
    cos_y = math.cos(turn)
    sin_y = math.sin(turn)
    local_x = cos_y * delta_x + sin_y * delta_y
    local_y = -sin_y * delta_x + cos_y * delta_y
    half_x = 0.5 * extents[0] + pad
    half_y = 0.5 * extents[1] + pad
    half_z = 0.5 * extents[2] + pad
    finite = np.isfinite(pts).all(axis=1)
    return (
        finite
        & (np.abs(local_x) <= half_x)
        & (np.abs(local_y) <= half_y)
        & (np.abs(delta_z) <= half_z)
    )


def occupancy_height_matches(measured, box_height, resolution):
    """True when the occupied column is within one voxel of the suitcase height."""
    if measured is None:
        return False
    try:
        delta = abs(float(measured) - float(box_height))
        limit = float(resolution)
    except (TypeError, ValueError):
        return False
    if not math.isfinite(delta) or not math.isfinite(limit):
        return False
    return delta <= limit + 1e-9


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


def rotated_rect_aabb(center_xy, width, depth, yaw):
    """XY AABB ``(xmin, ymin, xmax, ymax)`` of a ``width`` x ``depth`` box.

    ``width`` lies along the box's own x axis and ``yaw`` is the box's
    heading in the SAME frame as ``center_xy``. A box turned a quarter
    turn swaps the extents; ignoring ``yaw`` mis-sizes the reference
    footprint of every box that is not axis-aligned to the container.
    """
    import math

    cos_yaw, sin_yaw = abs(math.cos(yaw)), abs(math.sin(yaw))
    half_x = 0.5 * (cos_yaw * float(width) + sin_yaw * float(depth))
    half_y = 0.5 * (sin_yaw * float(width) + cos_yaw * float(depth))
    return (
        float(center_xy[0]) - half_x, float(center_xy[1]) - half_y,
        float(center_xy[0]) + half_x, float(center_xy[1]) + half_y,
    )


def _aabb_xy_overlap_depth(first, second):
    """Smaller of the x and y overlap depths (metres); <= 0 means disjoint."""
    return min(
        min(first[2], second[2]) - max(first[0], second[0]),
        min(first[3], second[3]) - max(first[1], second[1]))


def overlap_contains_full_cell(candidate_xy, occupied_xy, origin_xy,
                               resolution_m):
    """True when the overlap covers one complete occupancy cell.

    Candidate edges lie on the occupancy grid. The reference edge is a
    measured position, so a candidate that stops on the grid line just
    outside that edge overlaps by the off-grid fraction. That fraction is
    shorter than one cell and contains no cell. At the live 0.05 m grid one
    cell is 50 mm and half a cell is 25 mm; 25 mm is not this bound, because
    a 30 mm fraction still contains no cell while a one-cell step does.
    """
    import math

    resolution = float(resolution_m)
    if resolution <= 0.0:
        return False
    lo_x = max(float(candidate_xy[0]), float(occupied_xy[0]))
    hi_x = min(float(candidate_xy[2]), float(occupied_xy[2]))
    lo_y = max(float(candidate_xy[1]), float(occupied_xy[1]))
    hi_y = min(float(candidate_xy[3]), float(occupied_xy[3]))
    if hi_x - lo_x < resolution - 1e-9 or hi_y - lo_y < resolution - 1e-9:
        return False
    origin_x, origin_y = float(origin_xy[0]), float(origin_xy[1])
    i0 = math.ceil((lo_x - origin_x) / resolution - 1e-9)
    i1 = math.floor((hi_x - origin_x) / resolution + 1e-9) - 1
    j0 = math.ceil((lo_y - origin_y) / resolution - 1e-9)
    j1 = math.floor((hi_y - origin_y) / resolution + 1e-9) - 1
    return i0 <= i1 and j0 <= j1


def floor_through_count(surface, box_size, occupied_xy, peak_tol=0.05):
    """Count feasible floor candidates that cover a full cell of ``occupied_xy``.

    ``occupied_xy`` is ``(xmin, ymin, xmax, ymax)``. A metric sliver is
    recorded in ``max_overlap_m`` and ``n_boundary_slivers`` and is not
    counted: see ``overlap_contains_full_cell``. Without a grid origin the
    sliver exception is unavailable, so any metric overlap counts. Uses
    live ``solve_placement`` so the occupancy-used proof does not call the
    production service with synthetic GT.
    """
    from luggage_packing.placement_solver import solve_placement

    if occupied_xy is None:
        return 0, {"message": "no occupied patch"}
    resolution = float(surface.get("resolution") or 0.0)
    origin = surface.get("origin_local")
    has_grid = (
        resolution > 0.0
        and isinstance(origin, (list, tuple)) and len(origin) >= 2)
    inner_h = float((surface.get("inner_size") or [0, 0, 1.5])[2])
    result = solve_placement(
        surface, box_size, allowed_yaws=[0.0],
        params={"top_n": 400, "keep_rejected": 80})
    count = 0
    slivers = 0
    max_overlap = 0.0
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
        depth = _aabb_xy_overlap_depth(cand_xy, occupied_xy)
        max_overlap = max(max_overlap, depth)
        if depth <= 1e-9 or not _aabb_xy_overlap(cand_xy, occupied_xy):
            continue
        if has_grid and not overlap_contains_full_cell(
                cand_xy, occupied_xy, origin, resolution):
            slivers += 1
            continue
        count += 1
    return count, {
        "rule": "full_occupancy_cell" if has_grid else "metric_overlap_no_grid",
        "resolution_m": resolution,
        "full_cell_m": resolution,
        "half_cell_m": 0.5 * resolution,
        "max_overlap_m": round(max_overlap, 6),
        "n_boundary_slivers": slivers,
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


# --------------------------------------------------------------------------
# ST09-D: frozen ViewIntegrator + PlanningSnapshot producer (additive; every
# function above is unchanged — the mapper node, drivers and existing tests
# keep their behavior).
# --------------------------------------------------------------------------

# Frozen join window (resolved_planning_contract.map_freshness):
# buffered sensor data must be within this window of the request stamp to
# join (STAMP_STALE / VIEW_TF_STALE otherwise).
SNAPSHOT_JOIN_WINDOW_SEC = DEFAULT_FRESHNESS_WINDOW_SEC


def _contract_violation(reason_value, message):
    from st09_contracts._base import ContractViolation
    return ContractViolation("%s: %s" % (reason_value, message))


def planning_snapshot_from_surface(surface_map, scene_revision="",
                                   request_id=""):
    """Build the frozen ``PlanningSnapshot`` from a ``surface_map_2d`` dict.

    Fills the producer gap: the contract snapshot is what route planning
    and map revalidation consume. Cells are sparse (only free/occupied
    listed, row-major sorted) exactly like the C0 fixtures; heights and
    confidences stay in the surface domain — the contract grid is a
    cell-state view.
    """
    from st09_contracts._base import digest_of
    from st09_contracts.planning import PlanningSnapshot

    resolution = float(surface_map["resolution"])
    origin_x = float(surface_map["origin_local"][0])
    origin_y = float(surface_map["origin_local"][1])
    states = surface_map["state"]
    cells_i = []
    cells_j = []
    cell_states = []
    for ix in range(int(surface_map["nx"])):
        for iy in range(int(surface_map["ny"])):
            state = str(states[ix][iy])
            # Unknown stays unknown: it is omitted, not converted to free.
            # The snapshot therefore does not declare a complete occupancy
            # domain, and an empty listing is not a full container.
            if state == "unknown":
                continue
            cells_i.append(ix)
            cells_j.append(iy)
            cell_states.append(state)
    return PlanningSnapshot(
        geometry_id=str(surface_map.get("frame", "container_local")),
        geometry_hash=str(surface_map.get("geometry_hash") or
                          digest_of(surface_map["inner_size"])),
        # [ST09-I S5.7-6] the identity gates (verify_selected /
        # execution_admission / observation+named-pose producers) compare
        # the snapshot's scene binding — the param was accepted but never
        # wired, so every snapshot read as "" and a real request scene
        # always failed IDENTITY_CHANGED.
        scene_revision=str(scene_revision or ""),
        map_revision=int(surface_map.get("map_revision", 0)),
        acquisition_stamp_ns=int(surface_map.get("acquisition_stamp_ns", 0)),
        map_source=str(surface_map.get("map_source") or ""),
        placed_count=int(surface_map.get("placed_count") or 0),
        cargo_observed=bool(surface_map.get("cargo_observed") or False),
        commit_revision=int(surface_map.get("commit_revision", 1)),
        occupancy_digest=digest_of(tuple(cells_i), tuple(cells_j),
                                   tuple(cell_states)),
        column_height_digest=_column_height_digest(
            surface_map, cells_i, cells_j, cell_states),
        resolution_m=resolution,
        origin_xy=(origin_x, origin_y),
        cells_i=tuple(cells_i),
        cells_j=tuple(cells_j),
        cell_states=tuple(cell_states),
        occupancy_confirmed=bool(
            surface_map.get("occupancy_confirmed", True)),
        change_event=str(surface_map.get("change_event") or ""),
        epoch=str(surface_map.get("epoch") or ""),
        content_digest=str(surface_map.get("content_digest") or ""),
    )


def _cell_of_point(point, origin, resolution):
    return (int(math.floor((float(point[0]) - origin[0]) / resolution)),
            int(math.floor((float(point[1]) - origin[1]) / resolution)))


def integrate_view_with_report(snapshot, requirement, view_payload):
    """Integrate one saved sensor view; pure and immutable.

    Returns ``(new_snapshot_or_None, invalidation_or_None, report)`` where
    ``report`` carries the named outcome (``integrated``, ``duplicate``,
    ``partial_coverage``). Hard failures raise ``ContractViolation`` with
    the frozen ``Reason`` prefix: ``MAP_GEOMETRY_HASH_MISMATCH``,
    ``VIEW_TF_STALE``, ``VIEW_DEPTH_MISSING``, ``VIEW_EMPTY``.

    ``view_payload`` keys:
      - ``acquisition_stamp_ns``: the view's sensor stamp (int ns);
      - ``tf_stamp_ns``: stamp of the TF that expressed the points in the
        container frame (join window applies);
      - ``geometry_hash``: hull identity the view was taken against;
      - ``points_container``: sensor hits in container frame (Nx3);
      - ``observed_cells``: cells the camera actually covered — miss
        testimony; only these may turn UNKNOWN into FREE;
      - ``depth_present``: False marks a view whose depth stream is missing.

    Idempotency is stateless: a view whose acquisition stamp equals the
    snapshot's own acquisition stamp is the same acquisition already
    integrated — the snapshot returns unchanged and the report says
    ``duplicate`` (revision must not advance twice for one observation).
    Missing sensors never promote cells to free or support.
    """
    from st09_contracts.planning import PlanningSnapshot
    from st09_contracts.reasons import Reason
    from st09_contracts._base import digest_of

    acquisition = int(view_payload.get("acquisition_stamp_ns", 0))
    tf_stamp = int(view_payload.get("tf_stamp_ns", acquisition))
    geometry_hash = str(view_payload.get("geometry_hash") or "")
    if geometry_hash and geometry_hash != snapshot.geometry_hash:
        raise _contract_violation(
            Reason.MAP_GEOMETRY_HASH_MISMATCH.value,
            "view geometry %s does not match snapshot %s"
            % (geometry_hash, snapshot.geometry_hash))
    skew_ns = abs(acquisition - tf_stamp)
    if skew_ns > int(SNAPSHOT_JOIN_WINDOW_SEC * 1e9):
        raise _contract_violation(
            Reason.VIEW_TF_STALE.value,
            "TF stamp %d is %.3f s from the view stamp %d"
            % (tf_stamp, skew_ns * 1e-9, acquisition))
    if view_payload.get("depth_present", True) is False:
        raise _contract_violation(
            Reason.VIEW_DEPTH_MISSING.value,
            "the depth stream for this view is missing")
    points = view_payload.get("points_container") or []
    observed_cells = [tuple(cell) for cell in
                      (view_payload.get("observed_cells") or [])]
    if not len(points) and not observed_cells:
        raise _contract_violation(
            Reason.VIEW_EMPTY.value,
            "view carries neither hits nor covered cells")

    if acquisition == snapshot.acquisition_stamp_ns:
        report = {"outcome": "duplicate",
                  "reason": Reason.VIEW_DUPLICATE.value,
                  "map_revision": snapshot.map_revision}
        return snapshot, None, report

    resolution = float(snapshot.resolution_m)
    origin = (float(snapshot.origin_xy[0]), float(snapshot.origin_xy[1]))
    known = {}
    for i, j, state in zip(snapshot.cells_i, snapshot.cells_j,
                           snapshot.cell_states):
        known[(int(i), int(j))] = str(state)
    changed = {}
    for cell in observed_cells:
        if known.get(cell, "unknown") == "unknown":
            changed[cell] = "free"   # sensor miss testimony
    hits = 0
    for point in points:
        cell = _cell_of_point(point, origin, resolution)
        changed[cell] = "occupied"  # sensor hit
        hits += 1

    merged = dict(known)
    merged.update(changed)
    cells = sorted(merged)
    cells_i = tuple(cell[0] for cell in cells)
    cells_j = tuple(cell[1] for cell in cells)
    cell_states = tuple(merged[cell] for cell in cells)
    new_snapshot = PlanningSnapshot(
        geometry_id=snapshot.geometry_id,
        geometry_hash=snapshot.geometry_hash,
        scene_revision=snapshot.scene_revision,
        map_revision=snapshot.map_revision + 1,
        acquisition_stamp_ns=acquisition,
        map_source=MAP_MEASURED,
        placed_count=int(getattr(snapshot, "placed_count", 0) or 0),
        cargo_observed=True,
        commit_revision=snapshot.commit_revision + 1,
        occupancy_digest=digest_of(cells_i, cells_j, cell_states),
        resolution_m=resolution,
        origin_xy=snapshot.origin_xy,
        cells_i=cells_i,
        cells_j=cells_j,
        cell_states=cell_states,
        occupancy_confirmed=True,
        change_event="",
    )

    required = set(requirement.missing_cells) if requirement is not None \
        else set()
    if requirement is not None:
        required |= set(requirement.footprint_cells)
        required |= set(requirement.corridor_cells)
    still_unknown = [
        cell for cell in sorted(required)
        if new_snapshot.state_at(cell[0], cell[1]).value == "unknown"
    ] if required else []
    outcome = ("partial_coverage" if still_unknown else "integrated")
    report = {
        "outcome": outcome,
        "reason": (Reason.VIEW_PARTIAL_COVERAGE.value if still_unknown
                   else None),
        "map_revision": new_snapshot.map_revision,
        "n_hits": hits,
        "n_cells_freed": sum(1 for v in changed.values() if v == "free"),
        "n_cells_occupied": sum(1 for v in changed.values()
                                if v == "occupied"),
        "required_cells": len(required),
        "still_unknown_after": still_unknown,
    }
    return new_snapshot, None, report


class SnapshotViewIntegrator:
    """Frozen ``ViewIntegrator`` protocol implementation.

    Thin wrapper over :func:`integrate_view_with_report`: the protocol
    signature returns ``(snapshot, invalidation)``; the report (named
    outcome, coverage accounting) stays available through the function.
    The integrator is a pure function of its inputs — asynchronous by
    construction: callers (ST09-I adapter) queue ``(snapshot, view)``
    pairs and never hold a mapper lock.
    """

    def integrate_view(self, snapshot, requirement, view_payload):
        new_snapshot, invalidation, _report = integrate_view_with_report(
            snapshot, requirement, view_payload)
        return new_snapshot, invalidation
