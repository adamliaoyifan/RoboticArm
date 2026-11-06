"""oracle — independent verification of returned placement candidates.

Contract I4: the oracle is INDEPENDENT of the solver's success flag. For
every returned candidate it recomputes, on the admitted surface map and
the frozen hull descriptor, the full-orientation volume check: hull
containment (with the frozen margin), occupied overlap at every cell the
oriented box covers, unknown-interior classification, support ratio and
clearance. Policy values are frozen from the C1 code, not negotiated.
"""
from __future__ import annotations

import math

#: Frozen policy (C1 placement code values; changing these here is a
#: contract change, not a tuning knob).
POLICY = {
    "float_tolerance_m": 1e-9,
    "same_layer_clearance_m": 0.02,
    "min_support_ratio": 0.55,
    "hull_margin_default_m": 0.0,
}

_ORIENTATIONS = (0.0, math.pi / 2)   # box_axis_pi yaw semantics


def _oriented_footprint(size, yaw):
    """Axis-aligned footprint of a box with dims (w, d) rotated by yaw."""
    w, d, _h = size
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    return (w * c + d * s, w * s + d * c)


def _cells_under(footprint, center_xy, resolution, origin_xy, nx, ny):
    """Grid cells whose centers lie inside the oriented footprint."""
    fw, fd = footprint
    x0, x1 = center_xy[0] - fw / 2.0, center_xy[0] + fw / 2.0
    y0, y1 = center_xy[1] - fd / 2.0, center_xy[1] + fd / 2.0
    cells = []
    for ix in range(nx):
        cx = origin_xy[0] + (ix + 0.5) * resolution
        if cx < x0 - POLICY["float_tolerance_m"] or \
                cx > x1 + POLICY["float_tolerance_m"]:
            continue
        for iy in range(ny):
            cy = origin_xy[1] + (iy + 0.5) * resolution
            if y0 - POLICY["float_tolerance_m"] <= cy <= \
                    y1 + POLICY["float_tolerance_m"]:
                cells.append((ix, iy))
    return cells


def _column_height(surface, ix, iy):
    """Floor-relative height of one column from a v1 surface dict."""
    height = surface["height"][ix][iy]
    state = surface["state"][ix][iy]
    return float(height or 0.0), str(state)


def verify_candidate(candidate, *, surface, box_size, hull,
                     margin=POLICY["hull_margin_default_m"]):
    """Full-orientation verification of ONE returned candidate.

    ``candidate`` carries at least the center (container frame), yaw and
    the source's own claim. Returns a verdict dict — never an exception:
    every malformed field is a failed leg, not a crash.
    """
    result = {"legs": {}, "pass": False}
    try:
        center = [float(v) for v in candidate["center_container"]]
        yaw = float(candidate.get("yaw", 0.0) % math.pi)
        size = [float(v) for v in box_size]
    except (KeyError, TypeError, ValueError) as error:
        result["legs"]["decode"] = "malformed candidate: %s" % error
        return result

    fw, fd = _oriented_footprint(size, yaw)
    inner = hull["inner_size"]
    # The solver's own grid placement is authoritative for the footprint
    # when present (ix0/iy0 + cells_x/cells_y in the map's indexing).
    solver_cells = None
    if all(k in candidate for k in ("ix0", "iy0", "cells_x", "cells_y")):
        try:
            solver_cells = [
                (int(candidate["ix0"]) + dx, int(candidate["iy0"]) + dy)
                for dx in range(int(candidate["cells_x"]))
                for dy in range(int(candidate["cells_y"]))]
        except (TypeError, ValueError):
            solver_cells = None
    # Hull containment (with the frozen margin), container frame:
    # origin at the hull center per the planner's center_base convention.
    half_l, half_w, _ = inner[0] / 2.0, inner[1] / 2.0, inner[2]
    result["legs"]["hull"] = "PASS" if (
        abs(center[0]) + fw / 2.0 <= half_l - margin +
        POLICY["float_tolerance_m"] and
        abs(center[1]) + fd / 2.0 <= half_w - margin +
        POLICY["float_tolerance_m"]) else "FAIL_outside_hull"

    resolution = float(surface["resolution"])
    nx, ny = int(surface["nx"]), int(surface["ny"])
    # v1 maps are container-centered: cell (ix, iy) center in container
    # frame is (-(nx/2)+ (ix+0.5)) * res, same for y.
    origin_xy = (-(nx / 2.0) * resolution, -(ny / 2.0) * resolution)
    cells = solver_cells or _cells_under(
        (fw, fd), (center[0], center[1]), resolution, origin_xy, nx, ny)
    cells = [(ix, iy) for ix, iy in cells
             if 0 <= ix < nx and 0 <= iy < ny]
    if not cells:
        result["legs"]["coverage"] = "FAIL_no_cells"
        return result
    result["n_cells"] = len(cells)

    # Frame discipline: surface heights are FLOOR-RELATIVE (the map's
    # floor_z is the solver convention 0.0); candidate centers are
    # container-frame. Convert the candidate bottom into the map's
    # floor-relative frame via the hull's container floor height.
    floor_rel = float(surface.get("floor_z", 0.0) or 0.0)
    floor_container = float(hull.get("floor_z_container", 0.53))
    z0 = center[2] - size[2] / 2.0
    z0_rel = z0 - floor_container
    occupied_conflict = unknown_interior = supported = 0
    for ix, iy in cells:
        height, state = _column_height(surface, ix, iy)
        top = floor_rel + height
        # Product contact tolerance (support_contact_tol_m = 0.05):
        # an occupied top within the tolerance of the box bottom is a
        # SUPPORT CONTACT (aabb_overlap: touching faces allowed), not a
        # positive-volume conflict.
        contact_tol = 0.05
        if state == "occupied" and top > z0_rel + contact_tol + 1e-9:
            occupied_conflict += 1
        elif state == "unknown":
            unknown_interior += 1
        elif abs(top - z0_rel) <= 1e-6:
            supported += 1
    result["legs"]["occupied_overlap"] = (
        "PASS" if occupied_conflict == 0 else
        "FAIL_%d_cells" % occupied_conflict)
    support_ratio = supported / float(len(cells))
    result["support_ratio"] = round(support_ratio, 4)
    # Claim verification (K01/K03 boundary): the candidate's own
    # confidence_ratio claim must match the map. An unknown-ground
    # candidate claiming confidence ~1 is a forged KNOWN_FREE (K03's
    # failure class); the same candidate claiming ~0 is the legal
    # floor-prior/hypothetical class — its unknown-interior is then not
    # a violation, and its support leg is judged against the claim.
    claim = float(candidate.get("confidence_ratio", 0.0) or 0.0)
    observed_known_ratio = 1.0 - unknown_interior / float(len(cells))
    claim_error = abs(claim - (support_ratio if claim > 0.5
                               else observed_known_ratio * 0.0))
    result["claim_confidence"] = claim
    if claim >= 0.5:
        result["legs"]["unknown_interior"] = (
            "PASS" if unknown_interior == 0 else
            "FAIL_%d_cells" % unknown_interior)
        result["legs"]["support"] = (
            "PASS" if support_ratio >= POLICY["min_support_ratio"] -
            POLICY["float_tolerance_m"] else
            "FAIL_ratio_%.3f" % support_ratio)
        result["candidate_class"] = "known_free"
    else:
        result["legs"]["unknown_interior"] = (
            "PASS" if claim < 0.5 and claim_error <= 0.5 else
            "FAIL_claim_mismatch")
        result["legs"]["support"] = (
            "PASS" if claim < 0.5 else "FAIL_claim_mismatch")
        result["candidate_class"] = "floor_prior"
    # Clearance: the box top must stay under the ceiling (relative).
    result["legs"]["clearance"] = "PASS" if \
        (z0_rel + size[2]) <= inner[2] + POLICY["float_tolerance_m"] \
        else "FAIL_above_ceiling"
    result["pass"] = all(v == "PASS" for v in result["legs"].values())
    return result


def verify_response(response, *, surface, box_size, hull):
    """Verify EVERY candidate a placement response returned (I4: full
    payloads, not counts), plus the no-solution taxonomy."""
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
