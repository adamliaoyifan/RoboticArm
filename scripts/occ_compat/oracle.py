"""oracle — independent verification of returned placement candidates.

Contract I4: the oracle is INDEPENDENT of the solver's success flag. For
every returned candidate it recomputes, on the admitted surface map and
the frozen hull descriptor, the full-orientation volume check: hull
containment (with the frozen margin, chamfer per the CANONICAL
descriptor ramp), occupied overlap at every cell the oriented box
overlaps with POSITIVE AREA, unknown-interior classification, support
ratio and clearance. Policy values are frozen from the C1 code, not
negotiated.

Geometry rules (reviews 1622 B1):
- Coverage is VOLUME-level: a cell counts when its square intersects
  the oriented footprint with positive area — center-inside alone
  misses edge overlaps.
- Contact tolerance (support) NEVER swallows a positive-volume occupied
  overlap: the tolerance governs the GAP side only (an occupied top may
  sit up to support_contact_tol BELOW the box bottom and still count as
  support). An occupied top above the box bottom by more than the
  numeric tolerance is a collision at ANY depth (1cm penetration
  rejects — reviews 2026-11-08 penetration control).
- Numeric validity is checked FIRST: finite values, positive size,
  bottom at/above the floor, top at/below the ceiling, yaw within the
  frozen legal set.
- Chamfer follows the canonical container_geometry.y_max_at_z: slope
  (wall_y - floor_y) / (wall_z - floor_z) with floor_z as origin — at
  floor_z the limit is floor_y, at wall_z it is wall_y.
"""
from __future__ import annotations

import math

#: Frozen policy (C1 placement code values; changing these here is a
#: contract change, not a tuning knob).
POLICY = {
    "float_tolerance_m": 1e-9,
    "same_layer_clearance_m": 0.02,
    "min_support_ratio": 0.55,
    # None = AUTO: one map cell (the surface's resolution) — mirrors the
    # C1 default since 2026-11-08 (user directive margin=resolution; was
    # 0.0 exact-face). One cell guarantees the footprint cannot overlap
    # the map's conservative boundary column, which reaches up to one
    # cell inside the exact wall face (K01 run04/07 anchoring miss).
    "hull_margin_default_m": None,
    "support_contact_tol_m": 0.05,
}

_ORIENTATIONS = (0.0, math.pi / 2)   # box_axis_pi yaw semantics


def _oriented_footprint(size, yaw):
    """Axis-aligned footprint of a box with dims (w, d) rotated by yaw."""
    w, d, _h = size
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    return (w * c + d * s, w * s + d * c)


def _yaw_is_legal(yaw):
    """The frozen orientation set (0/90°); other angles are illegal —
    an AABB approximation may not masquerade as legality proof. The
    tolerance covers the solver's 4-decimal representation (1.5708 vs
    pi/2 differs by 3.7e-6) while still rejecting mid angles (0.7)."""
    norm = yaw % math.pi
    return any(abs(norm - o) <= 1e-4 for o in _ORIENTATIONS)


#: Overlap slivers below this width per axis are yaw-REPRESENTATION
#: artifacts (the solver's 4-decimal 1.5708 vs pi/2 gives a footprint
#: error of ~size*1e-4 m) — not real positive-area coverage.
_OVERLAP_MIN_M = 1e-4


def _cells_overlapping(footprint, center_xy, resolution, origin_xy,
                       nx, ny):
    """Cells whose SQUARE intersects the footprint with POSITIVE AREA
    (half-open), regardless of whether the cell center is inside;
    representation slivers below _OVERLAP_MIN_M per axis are excluded."""
    fw, fd = footprint
    x0, x1 = center_xy[0] - fw / 2.0, center_xy[0] + fw / 2.0
    y0, y1 = center_xy[1] - fd / 2.0, center_xy[1] + fd / 2.0
    tol = _OVERLAP_MIN_M
    cells = []
    ix_lo = max(0, int((x0 - origin_xy[0]) / resolution))
    ix_hi = min(nx - 1, int((x1 - origin_xy[0]) / resolution))
    iy_lo = max(0, int((y0 - origin_xy[1]) / resolution))
    iy_hi = min(ny - 1, int((y1 - origin_xy[1]) / resolution))
    for ix in range(ix_lo, ix_hi + 1):
        cx0 = origin_xy[0] + ix * resolution
        cx1 = cx0 + resolution
        if cx1 <= x0 + tol or cx0 >= x1 - tol:
            continue  # no positive-area overlap in x
        for iy in range(iy_lo, iy_hi + 1):
            cy0 = origin_xy[1] + iy * resolution
            cy1 = cy0 + resolution
            if cy1 <= y0 + tol or cy0 >= y1 - tol:
                continue
            cells.append((ix, iy))
    return cells


def _cells_center_inside(footprint, center_xy, resolution, origin_xy,
                         nx, ny):
    """Cells whose CENTER lies inside the footprint — the solver's own
    placement convention, used ONLY to cross-check the candidate's
    self-reported indices (the claim), never as collision truth."""
    fw, fd = footprint
    x0, x1 = center_xy[0] - fw / 2.0, center_xy[0] + fw / 2.0
    y0, y1 = center_xy[1] - fd / 2.0, center_xy[1] + fd / 2.0
    tol = POLICY["float_tolerance_m"]
    cells = []
    for ix in range(nx):
        cx = origin_xy[0] + (ix + 0.5) * resolution
        if cx < x0 - tol or cx > x1 + tol:
            continue
        for iy in range(ny):
            cy = origin_xy[1] + (iy + 0.5) * resolution
            if y0 - tol <= cy <= y1 + tol:
                cells.append((ix, iy))
    return cells


def _column_height(surface, ix, iy):
    """Floor-relative height of one column from a v1 surface dict."""
    height = surface["height"][ix][iy]
    state = surface["state"][ix][iy]
    return float(height or 0.0), str(state)


def _occupied_rectangles(surface, hull, ix, iy, origin_xy, resolution):
    """The occupied region of one occupied cell = THE WHOLE CELL.

    Final ruling (reviews 2026-11-08 D1): a ring-only occupied
    distribution does NOT prove a boundary cell's interior sliver is
    cargo-free — cargo could occupy only boundary cells while every
    full-interior cell reads free. Without a reliable occupancy-source
    fact in the map, the oracle KEEPS boundary-cell occupancy: the
    consumer's own map says occupied, so a footprint overlapping it is
    illegal. Wall clipping requires a native wall declaration that the
    current maps do not carry; the SOLVER avoids these cells instead
    (candidates must not overlap conservatively-occupied wall edges)."""
    gx0 = origin_xy[0] + ix * resolution
    gx1 = gx0 + resolution
    gy0 = origin_xy[1] + iy * resolution
    gy1 = gy0 + resolution
    return [(gx0, gx1, gy0, gy1)]


def verify_candidate(candidate, *, surface, box_size, hull,
                     margin=POLICY["hull_margin_default_m"]):
    """Full-orientation verification of ONE returned candidate.

    ``candidate`` carries at least the center (container frame), yaw and
    the source's own claim. Returns a verdict dict — never an exception:
    every malformed field is a failed leg, not a crash."""
    result = {"legs": {}, "pass": False}
    try:
        center = [float(v) for v in candidate["center_container"]]
        yaw = float(candidate.get("yaw", 0.0))
        size = [float(v) for v in box_size]
    except (KeyError, TypeError, ValueError) as error:
        result["legs"]["decode"] = "malformed candidate: %s" % error
        return result
    if margin is None:
        # auto: one map cell (the C1 default since 2026-11-08)
        try:
            margin = float(surface["resolution"])
        except (KeyError, TypeError, ValueError):
            result["legs"]["hull"] = "margin_auto_no_resolution"
            return result

    # ---- numeric validity FIRST (B1)
    tol = POLICY["float_tolerance_m"]
    if (not all(math.isfinite(v) for v in center)
            or not all(math.isfinite(v) for v in size)
            or not math.isfinite(yaw)):
        result["legs"]["decode"] = "non-finite candidate values"
        return result
    if any(v <= 0 for v in size):
        result["legs"]["decode"] = "non-positive box size"
        return result
    floor_container = float(hull.get("floor_z_container", 0.53))
    ceiling_z = float(hull.get("inner_size", (0, 0, 1.48))[2])
    z0 = center[2] - size[2] / 2.0
    z1 = center[2] + size[2] / 2.0
    if z0 < floor_container - tol:
        result["legs"]["support"] = "FAIL_below_floor"
        return result
    if z1 > ceiling_z + tol:
        result["legs"]["clearance"] = "FAIL_above_ceiling"
        return result
    if not _yaw_is_legal(yaw):
        result["legs"]["hull"] = "FAIL_illegal_yaw"
        return result

    fw, fd = _oriented_footprint(size, yaw)
    inner = hull["inner_size"]

    # ---- hull containment + CANONICAL chamfer ramp (B1)
    half_l, half_w = inner[0] / 2.0, inner[1] / 2.0
    chamfer = hull.get("chamfer")
    if chamfer:
        floor_z = float(chamfer.get("floor_z", floor_container))
        wall_z = float(chamfer["wall_z"])
        wall_y = float(chamfer["wall_y"])
        floor_y = float(chamfer["floor_y"])
        # the limit at the box TOP corner (worst case), floored at the
        # canonical ramp: slope*(z - floor_z) + floor_y up to wall_z
        z_check = min(z1, wall_z)
        if wall_z > floor_z:
            slope = (wall_y - floor_y) / (wall_z - floor_z)
            y_limit = slope * (z_check - floor_z) + floor_y
        else:
            y_limit = wall_y
        y_limit = min(half_w, max(y_limit, 0.0))
        side = str(chamfer.get("side", "positive_y"))
        if side.endswith("positive_y"):
            worst = center[1] + fd / 2.0
        else:
            worst = -(center[1] - fd / 2.0)
        if worst > y_limit + tol + margin:
            result["legs"]["hull"] = "FAIL_chamfer_ramp"
            return result
    result["legs"]["hull"] = "PASS" if (
        abs(center[0]) + fw / 2.0 <= half_l - margin + tol and
        abs(center[1]) + fd / 2.0 <= half_w - margin + tol
    ) else "FAIL_outside_hull"
    if result["legs"]["hull"] != "PASS":
        return result

    resolution = float(surface["resolution"])
    nx, ny = int(surface["nx"]), int(surface["ny"])
    origin_xy = (-(nx / 2.0) * resolution, -(ny / 2.0) * resolution)

    # ---- VOLUME-LEVEL coverage (B1): positive-area intersection; an
    # out-of-grid footprint is a named failure, never clipped.
    grid_x0, grid_x1 = origin_xy[0], origin_xy[0] + nx * resolution
    grid_y0, grid_y1 = origin_xy[1], origin_xy[1] + ny * resolution
    if (center[0] - fw / 2 < grid_x0 - tol
            or center[0] + fw / 2 > grid_x1 + tol
            or center[1] - fd / 2 < grid_y0 - tol
            or center[1] + fd / 2 > grid_y1 + tol):
        result["legs"]["coverage"] = "FAIL_footprint_outside_grid"
        return result
    cells = _cells_overlapping(
        (fw, fd), (center[0], center[1]), resolution, origin_xy, nx, ny)
    if not cells:
        result["legs"]["coverage"] = "FAIL_no_cells"
        return result
    solver_cells = None
    if all(k in candidate for k in ("ix0", "iy0", "cells_x", "cells_y")):
        try:
            solver_cells = [
                (int(candidate["ix0"]) + dx, int(candidate["iy0"]) + dy)
                for dx in range(int(candidate["cells_x"]))
                for dy in range(int(candidate["cells_y"]))]
        except (TypeError, ValueError):
            solver_cells = None
    if solver_cells is not None:
        # the claim is cross-checked against the CENTER-INSIDE set (the
        # solver's own convention); the positive-area set above remains
        # the collision/coverage truth
        center_cells = _cells_center_inside(
            (fw, fd), (center[0], center[1]), resolution, origin_xy,
            nx, ny)
        if sorted(set(solver_cells)) != sorted(set(center_cells)):
            result["legs"]["claim"] = "FAIL_reported_cells_mismatch"
            return result
    result["n_cells"] = len(cells)
    x0r, x1r = center[0] - fw / 2.0, center[0] + fw / 2.0
    y0r, y1r = center[1] - fd / 2.0, center[1] + fd / 2.0

    # ---- frame discipline: surface heights are FLOOR-RELATIVE
    floor_rel = float(surface.get("floor_z", 0.0) or 0.0)
    z0_rel = z0 - floor_container
    occupied_conflict = unknown_interior = supported = 0
    contact_tol = POLICY["support_contact_tol_m"]
    for ix, iy in cells:
        height, state = _column_height(surface, ix, iy)
        top = floor_rel + height
        if state == "occupied":
            # Occupied cell = the whole cell (final D1 ruling). Contact
            # semantics (reviews 2026-11-08 penetration control): the
            # support tolerance governs the GAP side only — an occupied
            # top ABOVE the box bottom by more than the numeric
            # tolerance is a positive-volume penetration no matter how
            # small (1cm must reject); a top within [bottom - tol_band,
            # bottom + numeric] is support contact.
            box_rects = _occupied_rectangles(
                surface, hull, ix, iy, origin_xy, resolution)
            real_overlap = False
            contact_overlap = False
            for (rx0, rx1, ry0, ry1) in box_rects:
                ix_ov = min(x1r, rx1) - max(x0r, rx0)
                iy_ov = min(y1r, ry1) - max(y0r, ry0)
                if ix_ov <= tol or iy_ov <= tol:
                    continue
                if top > z0_rel + tol:
                    real_overlap = True  # positive-volume penetration
                elif top >= z0_rel - contact_tol:
                    contact_overlap = True  # support contact band
                # occupied top below the bottom beyond the gap band:
                # no interaction (overhang)
            if real_overlap:
                occupied_conflict += 1
            elif contact_overlap:
                supported += 1
        elif state == "unknown":
            unknown_interior += 1
        else:  # free column
            if abs(z0_rel - (floor_rel + height)) <= tol:
                supported += 1
    result["legs"]["occupied_overlap"] = (
        "PASS" if occupied_conflict == 0 else
        "FAIL_%d_cells" % occupied_conflict)
    support_ratio = supported / float(len(cells))
    result["support_ratio"] = round(support_ratio, 4)
    claim = float(candidate.get("confidence_ratio", 0.0) or 0.0)
    result["claim_confidence"] = claim
    if claim >= 0.5:
        result["legs"]["unknown_interior"] = (
            "PASS" if unknown_interior == 0 else
            "FAIL_%d_cells" % unknown_interior)
        result["legs"]["support"] = (
            "PASS" if support_ratio >= POLICY["min_support_ratio"] - tol
            else "FAIL_ratio_%.3f" % support_ratio)
        result["candidate_class"] = "known_free"
    else:
        result["legs"]["unknown_interior"] = (
            "PASS" if abs(claim) <= 0.5 else "FAIL_claim_mismatch")
        result["legs"]["support"] = (
            "PASS" if abs(claim) <= 0.5 else "FAIL_claim_mismatch")
        result["candidate_class"] = "floor_prior"
    # ---- clearance: box top under the ceiling (floor-relative)
    result["legs"]["clearance"] = (
        "PASS" if (z0_rel + size[2]) <= ceiling_z - floor_container + tol
        else "FAIL_above_ceiling")
    result["pass"] = all(v == "PASS" for v in result["legs"].values())
    return result


def verify_response(response, *, surface, box_size, hull):
    """Verify EVERY candidate a placement response returned (I4: full
    payloads, not counts), plus the no-solution taxonomy.

    NOTE (reviews 2026-11-08 D1): the response's candidates are
    ALTERNATIVES for one placement — mutually exclusive options, not
    co-placed boxes. The same_layer_clearance policy governs a NEW box
    vs ALREADY-COMMITTED boxes at commit time; applying it pairwise
    among alternatives would reject every ranked list by construction.
    Commit-time clearance is checked by the product's placement ledger
    (placed_aabbs), not by this oracle."""
    candidates = list(getattr(response, "candidates", None) or [])
    verdicts = [verify_candidate(c, surface=surface, box_size=box_size,
                                 hull=hull) for c in candidates]
    return {
        "n_candidates": len(candidates),
        "verdicts": verdicts,
        "all_pass": bool(verdicts) and all(v["pass"] for v in verdicts),
        "reason_code": getattr(response, "reason_code", ""),
    }


def classify_no_solution(response):
    """A no-solution verdict is legal only with a real reason taxonomy —
    never a swallowed exception (forbidden-path check)."""
    reason = str(getattr(response, "reason_code", "") or "")
    legal = {"BIN_FULL", "NO_KNOWN_FREE", "OUTSIDE_HULL",
             "OUTSIDE_APERTURE", "NO_SUPPORT"}
    return reason in legal, reason
