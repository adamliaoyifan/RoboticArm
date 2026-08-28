#!/usr/bin/env python3
"""Insertion corridor + "do not block deep EMS" hard constraint + proxy score.

P2 of the online-packing redesign (design §5.5, §5.7). Pure Python, no ROS.

  - ``corridor_blocked``: is an EMS unreachable from the opening for a box of
    the smallest catalog size? (conservative AABB sweep along the opening axis)
  - ``blocks_deep_space``: volume of useful EMS that becomes unreachable after
    placing a candidate box. > tol -> reject (the classic greedy failure mode
    of walling off the deep interior).
  - ``proxy_score``: the §5.7 degraded weighted heuristic (used when the
    rollout V̂ budget is exhausted). ``ems_regularity`` carries the highest
    weight because it is the best closed-form proxy for V̂.

Coordinates are floor-relative (X/Y in [-inner/2, +inner/2], Z in [0, inner_h]).
"""

from __future__ import division

import math

from luggage_packing.ems import volume


def _aabb_overlap(a, b):
    return (a[0] < b[3] and a[3] > b[0] and
            a[1] < b[4] and a[4] > b[1] and
            a[2] < b[5] and a[5] > b[2])


def _corridor_to(ems, inner_size, smallest_size, opening_side="negative_x"):
    """AABB of the horizontal corridor from the opening to the EMS near face."""
    inner_l, inner_w, inner_h = inner_size
    ex0, ey0, ez0, ex1, ey1, ez1 = ems
    sw, sd = smallest_size[0], smallest_size[1]
    if opening_side == "negative_x":
        cx0, cx1 = -inner_l * 0.5, ex0
        cy0, cy1 = ey0 - sd * 0.5, ey1 + sd * 0.5
        cz0, cz1 = ez0, ez1
    else:  # future: positive_x / side openings
        cx0, cx1 = ex1, inner_l * 0.5
        cy0, cy1 = ey0 - sd * 0.5, ey1 + sd * 0.5
        cz0, cz1 = ez0, ez1
    return (cx0, cy0, cz0, cx1, cy1, cz1)


def corridor_blocked(ems, boxes, inner_size, smallest_size,
                      opening_side="negative_x"):
    """Is the EMS's opening corridor walled off by a placed box?

    Conservative "wall" detection: a single box blocks the corridor only if it
    intersects the corridor's x-range AND spans the *full container width* in Y
    (so a box cannot pass around it in Y). Partial-width boxes do not wall off
    the deep interior. Multi-box walls are not detected (P2 simplification).
    """
    corridor = _corridor_to(ems, inner_size, smallest_size, opening_side)
    cx0, _cy0, cz0, cx1, _cy1, cz1 = corridor
    inner_w = inner_size[1]
    full_y_min, full_y_max = -inner_w * 0.5, inner_w * 0.5
    for b in boxes:
        # Box must be in the corridor's x-range and overlap its z-range.
        if not (b[0] < cx1 and b[3] > cx0 and b[2] < cz1 and b[5] > cz0):
            continue
        # Wall: spans the full container Y width at this x/z.
        if b[1] <= full_y_min + 1e-9 and b[4] >= full_y_max - 1e-9:
            return True
    return False


def blocks_deep_space(cand_box, ems, boxes, inner_size, smallest_size,
                      v_min, blocked_tol=0.02, opening_side="negative_x"):
    """Volume of useful EMS blocked from the opening by placing ``cand_box``.

    Computes the EMS list *after* placing the candidate (non-mutating), then
    checks each useful EMS (volume >= v_min) for corridor reachability.
    Returns (blocked_volume, is_blocked) where is_blocked = blocked_volume > tol.
    """
    ems_after = ems.ems_after(cand_box)
    all_boxes = list(boxes) + [cand_box]
    blocked = 0.0
    for space in ems_after.spaces:
        if volume(space) < v_min:
            continue
        if corridor_blocked(space, all_boxes, inner_size, smallest_size, opening_side):
            blocked += volume(space)
    return blocked, blocked > blocked_tol


# --------------------------------------------------------------------------- #
# Proxy score (§5.7 degraded path)
# --------------------------------------------------------------------------- #

_PROXY_WEIGHTS = {
    "ems_regularity": 0.35,
    "compactness": 0.20,
    "reachability": 0.15,
    "support_quality": 0.10,
    "insertion_clearance": 0.10,
    "observation_confidence": 0.10,
    "blocked_deep_ems": -0.30,
    "cog_height": -0.10,
}


def proxy_score(cand, model, ems, inner_size, smallest_size,
                reachability_prior=0.5, opening_side="negative_x"):
    """§5.7 weighted proxy for V̂. Returns (score, breakdown dict)."""
    inner_l, inner_w, inner_h = inner_size
    peak = cand.get("peak", 0.0)
    box_l, box_w, box_h = cand["size"]
    v_min = smallest_size[0] * smallest_size[1] * smallest_size[2]

    # Build the candidate AABB (floor-relative) for the corridor check.
    # ``peak`` is already floor-relative, so the box center is peak + h/2. Do
    # not add inner_h/2: that is the volume-center offset and would lift a
    # floor placement to mid-container, making blocks_deep_space score the
    # wrong corridor.
    lx, ly = cand["center_local"][0], cand["center_local"][1]
    lz_floor = peak + box_h * 0.5
    cand_box = (lx - box_l * 0.5, ly - box_w * 0.5, lz_floor - box_h * 0.5,
                lx + box_l * 0.5, ly + box_w * 0.5, lz_floor + box_h * 0.5)
    boxes = [(b["x0"], b["y0"], b["z0"], b["x1"], b["y1"], b["z1"]) for b in model.boxes]
    blocked_v, _ = blocks_deep_space(
        cand_box, ems, boxes, inner_size, smallest_size, v_min,
        opening_side=opening_side)

    ems_reg = ems.regularity()
    compactness = 1.0 - min(1.0, peak / max(inner_h, 1e-9))
    clearance = max(0.0, min(1.0, cand.get("clearance_top", 0.0) / max(box_h, 1e-9)))
    conf = cand.get("confidence_ratio", 0.0)
    support_q = 1.0 if cand.get("support_source", "sensor") != "floor_prior" else 0.5
    cog_height = peak / max(inner_h, 1e-9)
    v_ref = inner_l * inner_w * inner_h
    blocked_ratio = blocked_v / max(v_ref, 1e-9)

    w = _PROXY_WEIGHTS
    score = (
        w["ems_regularity"] * ems_reg
        + w["compactness"] * compactness
        + w["reachability"] * reachability_prior
        + w["support_quality"] * support_q
        + w["insertion_clearance"] * clearance
        + w["observation_confidence"] * conf
        + w["blocked_deep_ems"] * blocked_ratio
        + w["cog_height"] * cog_height
    )
    breakdown = {
        "ems_regularity": round(ems_reg, 4),
        "compactness": round(compactness, 4),
        "reachability": round(reachability_prior, 4),
        "support_quality": round(support_q, 4),
        "insertion_clearance": round(clearance, 4),
        "observation_confidence": round(conf, 4),
        "blocked_deep_ems_ratio": round(blocked_ratio, 4),
        "cog_height": round(cog_height, 4),
    }
    return score, breakdown
