#!/usr/bin/env python3
"""Rule-based placement candidate generation on a 2.5D surface map.

Pure-Python (no ROS) so it can be unit tested directly. The solver consumes the
``surface_map_2d`` contract published by ``cargo_volume_mapper`` and, for a given
box size, slides an axis-aligned footprint over the container-local height map to
produce scored candidate center poses.

First-version rules (matching the MVP plan):
  - box is placed horizontally (no tilt)
  - yaw restricted to the requested set, snapped to 0 / 90 deg footprints
  - z = max(height under footprint) + box_height / 2
  - unobserved columns are allowed only when the box lands on the a-priori
    configured floor (peak ≈ floor_z, "floor_prior"); stacking on an unobserved surface
    (peak > 0) is rejected as "unknown_above_floor" (see design §4.2.2)
  - top clearance must exceed box height margin
"""

from __future__ import division

import math


REASON_OVERLAP = "overlap"
REASON_OUTSIDE_APERTURE = "outside_aperture"
REASON_OUTSIDE_HULL = "outside_hull"
REASON_CORRIDOR_BLOCKED = "corridor_blocked"
REASON_UNKNOWN_ABOVE_FLOOR = "unknown_above_floor"
REASON_INSUFFICIENT_CLEARANCE = "insufficient_clearance"
REASON_INSUFFICIENT_SUPPORT = "insufficient_support"

# A capacity gate answers "does this box physically fit here, given the
# container hull and what is already inside?". Everything else (aperture,
# insertion corridor, unobserved support) is a policy or observation gate: it
# rejects a slot the container may still have room for. The split matches the
# independent eval enumerator in ``place_only_fixture.enumerate_footprints``
# so the product's own claim and the arbiter's verdict are comparable.
CAPACITY_REASONS = frozenset((
    REASON_OVERLAP, REASON_OUTSIDE_HULL, REASON_INSUFFICIENT_CLEARANCE))
POLICY_REASONS = frozenset((
    REASON_OUTSIDE_APERTURE, REASON_CORRIDOR_BLOCKED,
    REASON_UNKNOWN_ABOVE_FLOOR, REASON_INSUFFICIENT_SUPPORT))

# Stable placement failure taxonomy (docs/architecture/container_geometry.md).
# The solver emits the first four; the node adds the last two at its own
# boundary. They share this module so every consumer reads one taxonomy.
CODE_INVALID_BOX_SIZE = "INVALID_BOX_SIZE"
CODE_BOX_EXCEEDS_CONTAINER = "BOX_EXCEEDS_CONTAINER"
CODE_BIN_FULL = "BIN_FULL"
CODE_PLACE_CANDIDATE_EXHAUSTED = "PLACE_CANDIDATE_EXHAUSTED"
CODE_CARGO_MAP_GEOMETRY_MISMATCH = "CARGO_MAP_GEOMETRY_MISMATCH"
CODE_DETECT_FULL_GEOMETRY_REQUIRED = "DETECT_FULL_GEOMETRY_REQUIRED"

PLACEMENT_REASON_CODES = frozenset((
    CODE_INVALID_BOX_SIZE, CODE_BOX_EXCEEDS_CONTAINER, CODE_BIN_FULL,
    CODE_PLACE_CANDIDATE_EXHAUSTED, CODE_CARGO_MAP_GEOMETRY_MISMATCH,
    CODE_DETECT_FULL_GEOMETRY_REQUIRED))


def candidate_aabb(candidate, inner_h):
    """Return a candidate AABB in floor-relative container coordinates."""
    local = candidate.get("center_local") or [0.0, 0.0, 0.0]
    footprint = candidate.get("footprint") or candidate["size"][:2]
    height = float(candidate["size"][2])
    center_z = float(local[2]) + float(inner_h) * 0.5
    half_l = float(footprint[0]) * 0.5
    half_w = float(footprint[1]) * 0.5
    half_h = height * 0.5
    return (
        float(local[0]) - half_l, float(local[1]) - half_w, center_z - half_h,
        float(local[0]) + half_l, float(local[1]) + half_w, center_z + half_h,
    )


def aabb_overlap(a, b, tolerance=1e-9):
    """True for positive-volume overlap; touching support faces are allowed."""
    return (
        a[0] < b[3] - tolerance and a[3] > b[0] + tolerance
        and a[1] < b[4] - tolerance and a[4] > b[1] + tolerance
        and a[2] < b[5] - tolerance and a[5] > b[2] + tolerance
    )


def placement_constraint_verdict(
        candidate, inner_h, placed_aabbs=None, aperture_y=None,
        hull_contains=None, inner_size=None, smallest_size=None, hull=None):
    """Return ``(reason, capacity_ok)`` for one candidate.

    ``reason`` is ``None`` when every gate passes, otherwise the first reject
    reason in the historical report order (aperture, hull, overlap, corridor).
    ``capacity_ok`` is independent of that order: it is False only when a
    capacity gate rejected the box, so a slot lost to aperture or corridor
    alone still counts as physical room inside the container.

    ``hull_contains`` receives floor-relative XYZ corners. Keeping it as an
    injected geometry-kernel callback makes this function ROS-free without
    duplicating the authoritative seven-face hull implementation.
    """
    box = candidate_aabb(candidate, inner_h)
    outside_aperture = (
        aperture_y is not None
        and (box[1] < float(aperture_y[0]) - 1e-6
             or box[4] > float(aperture_y[1]) + 1e-6))
    outside_hull = False
    if hull_contains is not None:
        for x in (box[0], box[3]):
            for y in (box[1], box[4]):
                for z in (box[2], box[5]):
                    if not hull_contains((x, y, z)):
                        outside_hull = True
                        break
                if outside_hull:
                    break
            if outside_hull:
                break
    overlaps = any(aabb_overlap(box, placed) for placed in (placed_aabbs or []))
    capacity_ok = not (outside_hull or overlaps)

    if outside_aperture:
        return REASON_OUTSIDE_APERTURE, capacity_ok
    if outside_hull:
        return REASON_OUTSIDE_HULL, capacity_ok
    if overlaps:
        return REASON_OVERLAP, capacity_ok
    # The corridor sweep is the expensive gate; it only matters once the box
    # already fits, so it stays behind the cheap gates as before.
    if (inner_size is not None and smallest_size is not None):
        from luggage_packing.insertion_corridor import corridor_blocked
        if corridor_blocked(
                box, placed_aabbs or [], inner_size, smallest_size,
                hull=hull):
            return REASON_CORRIDOR_BLOCKED, capacity_ok
    return None, capacity_ok


def placement_constraint_reason(
        candidate, inner_h, placed_aabbs=None, aperture_y=None,
        hull_contains=None, inner_size=None, smallest_size=None, hull=None):
    """Return the first hard-constraint reject reason, or ``None``."""
    reason, _capacity_ok = placement_constraint_verdict(
        candidate, inner_h, placed_aabbs=placed_aabbs, aperture_y=aperture_y,
        hull_contains=hull_contains, inner_size=inner_size,
        smallest_size=smallest_size, hull=hull)
    return reason


def _default_params():
    return {
        "clearance_margin": 0.03,   # required free space above the placed box [m]
        "support_tol": 0.05,        # cells within this of the peak count as support [m]
        "min_support_ratio": 0.6,   # required contact ratio of the footprint
        "stride_cells": 1,          # grid stride for the sliding window
        "w_support": 0.5,
        "w_clearance": 0.3,
        "w_compactness": 0.2,       # prefer low placements (good stacking)
        "w_confidence": 0.15,       # penalizes geometry-only or unknown support
        "top_n": 8,
        "keep_rejected": 6,         # rejected candidates kept for visualization
    }


def _snap_yaws(allowed_yaws):
    """Snap requested yaws to {0, pi/2} footprint orientations.

    The grid is axis-aligned in the container-local frame, so only 0 / 90 deg
    footprints stay axis-aligned. Other yaws collapse onto the nearest of these
    two for footprint sizing while preserving the requested world yaw.
    """
    snapped = []
    seen = set()
    for yaw in allowed_yaws or [0.0]:
        # Reduce to [0, pi) since a box footprint is symmetric under pi.
        reduced = yaw % math.pi
        is_rotated = abs(reduced - math.pi / 2.0) < abs(reduced - 0.0)
        key = 1 if is_rotated else 0
        if key in seen:
            continue
        seen.add(key)
        snapped.append((yaw, is_rotated))
    return snapped


def _footprint_cells(footprint_l, footprint_w, resolution):
    cells_x = max(1, int(math.ceil(footprint_l / resolution - 1e-9)))
    cells_y = max(1, int(math.ceil(footprint_w / resolution - 1e-9)))
    return cells_x, cells_y


def _window_stats(surface_map, ix0, iy0, cells_x, cells_y):
    heights = surface_map["height"]
    states = surface_map["state"]
    confidences = surface_map.get("confidence")
    max_h = 0.0
    has_unknown = False
    cell_heights = []
    sensor_cells = 0
    total_cells = 0
    for ix in range(ix0, ix0 + cells_x):
        for iy in range(iy0, iy0 + cells_y):
            total_cells += 1
            if states[ix][iy] == "unknown":
                has_unknown = True
            h = heights[ix][iy]
            cell_heights.append(h)
            if h > max_h:
                max_h = h
            if confidences and confidences[ix][iy] == "sensor":
                sensor_cells += 1
    confidence_ratio = float(sensor_cells) / float(total_cells) if total_cells else 0.0
    return max_h, has_unknown, cell_heights, confidence_ratio


def _support_ratio(cell_heights, peak, tol):
    if not cell_heights:
        return 0.0
    contact = sum(1 for h in cell_heights if h >= peak - tol)
    return float(contact) / float(len(cell_heights))


def _local_to_base(center_base, yaw, lx, ly, lz):
    bx = center_base[0] + math.cos(yaw) * lx - math.sin(yaw) * ly
    by = center_base[1] + math.sin(yaw) * lx + math.cos(yaw) * ly
    bz = center_base[2] + lz
    return [bx, by, bz]


def _validator_verdict(candidate_validator, candidate):
    """Normalize a validator result to ``(reason, capacity_ok)``.

    Validators may return a plain reason string (legacy callers) or the
    ``(reason, capacity_ok)`` pair from ``placement_constraint_verdict``.
    """
    if candidate_validator is None:
        return None, True
    result = candidate_validator(candidate)
    if isinstance(result, tuple):
        reason, capacity_ok = result
        return (str(reason) if reason else None), bool(capacity_ok)
    reason = str(result) if result else None
    # A string-only validator cannot say which class rejected the candidate;
    # assume the conservative answer (no proven capacity) for capacity gates.
    return reason, reason not in CAPACITY_REASONS


def generate_candidates(surface_map, box_size, allowed_yaws=None, params=None,
                        candidate_validator=None, totals=None):
    """Return scored placement candidates sorted feasible-first by score.

    ``surface_map`` is the ``surface_map_2d`` dict. ``box_size`` is
    ``[length, width, height]`` in meters.

    Each candidate also carries ``capacity_feasible``: True when no capacity
    gate rejected it, so a caller can tell "the container is full" from
    "every candidate was lost to a policy gate".

    The returned list is truncated by ``top_n`` / ``keep_rejected``. Pass a
    dict as ``totals`` to receive ``n_enumerated`` and ``n_capacity_feasible``
    over every candidate evaluated, which is what a capacity claim must be
    based on.
    """
    cfg = _default_params()
    if params:
        cfg.update(params)

    box_l, box_w, box_h = [float(v) for v in box_size]
    resolution = float(surface_map["resolution"])
    nx = int(surface_map["nx"])
    ny = int(surface_map["ny"])
    inner_l, inner_w, inner_h = [float(v) for v in surface_map["inner_size"]]
    floor_z = float(surface_map.get("floor_z", 0.0))
    center_base = surface_map["center_base"]
    map_yaw = float(surface_map["yaw"])
    half_l = inner_l * 0.5
    half_w = inner_w * 0.5
    half_h = inner_h * 0.5
    stride = max(1, int(cfg["stride_cells"]))

    candidates = []
    for box_yaw, rotated in _snap_yaws(allowed_yaws):
        footprint_l, footprint_w = (box_w, box_l) if rotated else (box_l, box_w)
        if footprint_l > inner_l + 1e-6 or footprint_w > inner_w + 1e-6:
            continue
        cells_x, cells_y = _footprint_cells(footprint_l, footprint_w, resolution)
        if cells_x > nx or cells_y > ny:
            continue

        for ix0 in range(0, nx - cells_x + 1, stride):
            for iy0 in range(0, ny - cells_y + 1, stride):
                peak, has_unknown, cell_heights, confidence_ratio = _window_stats(
                    surface_map, ix0, iy0, cells_x, cells_y
                )
                support_ratio = _support_ratio(cell_heights, peak, cfg["support_tol"])
                top_of_box = peak + box_h
                clearance_top = inner_h - top_of_box

                lx = -half_l + (ix0 + cells_x * 0.5) * resolution
                ly = -half_w + (iy0 + cells_y * 0.5) * resolution
                lz = -half_h + peak + box_h * 0.5
                center_base_xyz = _local_to_base(center_base, map_yaw, lx, ly, lz)

                feasible = True
                reason = "ok"
                support_source = "sensor"
                # Clearance is a capacity gate, so it is evaluated for every
                # candidate rather than only for the ones that survive the
                # observation gates above it.
                clearance_ok = clearance_top >= cfg["clearance_margin"]
                if has_unknown:
                    if abs(peak - floor_z) > cfg["support_tol"]:
                        # Stacking on an unobserved support surface -> reject.
                        # "floor exists" is geometric prior, but "something
                        # unseen holds the box at peak>0" is not trustworthy.
                        feasible = False
                        reason = REASON_UNKNOWN_ABOVE_FLOOR
                    else:
                        # peak ≈ floor_z: lands on the a-priori container floor.
                        # The floor's *existence* is geometric prior (scene_tf),
                        # not a perception claim, so allow even though the
                        # column is unobserved. confidence_ratio == 0 makes
                        # these score below observed positions. See §4.2.2.
                        support_source = "floor_prior"
                if feasible and not clearance_ok:
                    feasible = False
                    reason = REASON_INSUFFICIENT_CLEARANCE
                elif feasible and support_ratio < cfg["min_support_ratio"]:
                    feasible = False
                    reason = REASON_INSUFFICIENT_SUPPORT

                clearance_score = max(0.0, min(1.0, clearance_top / max(box_h, 1e-6)))
                compactness = 1.0 - min(1.0, peak / max(inner_h, 1e-6))
                w_conf = float(cfg.get("w_confidence", 0.0))
                score = (
                    cfg["w_support"] * support_ratio
                    + cfg["w_clearance"] * clearance_score
                    + cfg["w_compactness"] * compactness
                    + w_conf * confidence_ratio
                )

                candidate = {
                    "center_base": center_base_xyz,
                    "center_local": [lx, ly, lz],
                    "yaw": map_yaw + box_yaw,
                    "box_yaw": box_yaw,
                    "footprint": [footprint_l, footprint_w],
                    "size": [box_l, box_w, box_h],
                    "support_score": round(support_ratio, 4),
                    "clearance_score": round(clearance_score, 4),
                    "clearance_top": round(clearance_top, 4),
                    "collision_margin": round(clearance_top, 4),
                    "confidence_ratio": round(confidence_ratio, 4),
                    "support_source": support_source,
                    "reachability_score": -1.0,
                    "score": round(score, 4),
                    "feasible": feasible,
                    "reason": reason,
                    "capacity_feasible": clearance_ok,
                }
                if clearance_ok:
                    # Run the geometry gates even when an observation gate
                    # already rejected the slot: capacity_feasible must not
                    # depend on which gate happened to fire first.
                    constraint_reason, capacity_ok = _validator_verdict(
                        candidate_validator, candidate)
                    candidate["capacity_feasible"] = bool(capacity_ok)
                    if feasible and constraint_reason:
                        candidate["feasible"] = False
                        candidate["reason"] = constraint_reason
                candidates.append(candidate)

    if totals is not None:
        totals["n_enumerated"] = len(candidates)
        totals["n_capacity_feasible"] = sum(
            1 for c in candidates if c["capacity_feasible"])

    candidates.sort(key=lambda c: (not c["feasible"], -c["score"]))
    feasible = [c for c in candidates if c["feasible"]][: int(cfg["top_n"])]
    rejected = [c for c in candidates if not c["feasible"]][: int(cfg["keep_rejected"])]
    return feasible + rejected


def solve_placement(surface_map, box_size, allowed_yaws=None, params=None,
                    candidate_validator=None):
    """ROS-free ComputePlacement core with a stable failure taxonomy.

    On failure ``reason_code`` is one of ``INVALID_BOX_SIZE``,
    ``BOX_EXCEEDS_CONTAINER``, ``BIN_FULL`` or ``PLACE_CANDIDATE_EXHAUSTED``.
    ``BIN_FULL`` is only returned when no enumerated candidate was
    capacity-feasible, so a run that lost every slot to the aperture, the
    insertion corridor, or unobserved support is never reported as a full
    container.
    """
    size = [float(v) for v in box_size]
    if len(size) != 3 or not all(math.isfinite(v) and v > 0.0 for v in size):
        return {
            "success": False,
            "selected": None,
            "candidates": [],
            "reject_histogram": {"invalid_size": 1},
            "reason_code": CODE_INVALID_BOX_SIZE,
            "capacity_feasible_count": 0,
            "message": "%s invalid_size" % CODE_INVALID_BOX_SIZE,
            "map_revision": surface_map.get("map_revision"),
        }
    totals = {}
    candidates = generate_candidates(
        surface_map, size, allowed_yaws=allowed_yaws, params=params,
        candidate_validator=candidate_validator, totals=totals)
    feasible = [candidate for candidate in candidates
                if candidate.get("feasible", False)]
    # Count over every enumerated candidate, not the retained subset: a
    # capacity claim must not depend on keep_rejected.
    capacity_feasible_count = int(totals.get("n_capacity_feasible", 0))
    histogram = {}
    for candidate in candidates:
        if candidate.get("feasible", False):
            continue
        reason = str(candidate.get("reason") or "rejected")
        histogram[reason] = histogram.get(reason, 0) + 1
    if not feasible:
        if not totals.get("n_enumerated"):
            # No sliding window was even enumerated: the footprint or height
            # exceeds the container at every allowed yaw.
            reason_code = CODE_BOX_EXCEEDS_CONTAINER
        elif capacity_feasible_count:
            reason_code = CODE_PLACE_CANDIDATE_EXHAUSTED
        else:
            reason_code = CODE_BIN_FULL
        parts = " ".join("%s=%d" % (key, value)
                         for key, value in sorted(histogram.items()))
        message = "%s no_candidate%s%s" % (
            reason_code, ": " if parts else "", parts)
        selected = None
    else:
        reason_code = ""
        selected = max(feasible, key=lambda item: item.get("score", 0.0))
        message = "slot score=%.3f feasible=%d rejected=%s" % (
            selected.get("score", 0.0), len(feasible),
            histogram or {})
    return {
        "success": bool(feasible),
        "selected": selected,
        "candidates": candidates,
        "reject_histogram": histogram,
        "reason_code": reason_code,
        "capacity_feasible_count": capacity_feasible_count,
        "candidates_enumerated": int(totals.get("n_enumerated", 0)),
        "message": message,
        "map_revision": surface_map.get("map_revision"),
    }


def best_candidate(candidates):
    for cand in candidates:
        if cand["feasible"]:
            return cand
    return None


def ranked_feasible_candidates(result, max_candidates=5):
    """Score-sorted feasible prefix for ComputePlacement.candidates.

    ``candidates[0]`` matches ``result['selected']`` when any slot is
    feasible. ``max_candidates <= 0`` uses the default of 5.
    """
    feasible = [
        candidate for candidate in (result.get("candidates") or [])
        if candidate.get("feasible")]
    limit = int(max_candidates or 0)
    if limit <= 0:
        limit = 5
    return feasible[:limit]


if __name__ == "__main__":
    # Minimal self-test on a synthetic empty container.
    sm = {
        "resolution": 0.1,
        "nx": 20,
        "ny": 16,
        "inner_size": [2.0, 1.6, 1.5],
        "center_base": [2.0, 0.0, 0.75],
        "yaw": 0.0,
        "height": [[0.0] * 16 for _ in range(20)],
        "state": [["free"] * 16 for _ in range(20)],
        "clearance": [[1.5] * 16 for _ in range(20)],
        "known_ratio": [[1.0] * 16 for _ in range(20)],
        "confidence": [["sensor"] * 16 for _ in range(20)],
    }
    cands = generate_candidates(sm, [0.7, 0.45, 0.28], allowed_yaws=[0.0, math.pi / 2.0])
    print("candidates:", len(cands))
    top = best_candidate(cands)
    print("best:", top["center_base"], "score", top["score"], "reason", top["reason"])
