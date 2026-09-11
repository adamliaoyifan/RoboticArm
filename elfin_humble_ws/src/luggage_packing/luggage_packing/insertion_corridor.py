#!/usr/bin/env python3
"""Insertion corridor + "do not block deep EMS" hard constraint + proxy score.

P2 of the online-packing redesign (design §5.5, §5.7). Pure Python, no ROS.

  - ``corridor_blocked``: is an EMS unreachable from the opening for a box of
    the smallest catalog size? Broad-phase AABB plus hull-eroded Y coverage.
  - ``blocks_deep_space``: hull-clipped volume of useful EMS that becomes
    unreachable after placing a candidate box. > tol -> reject.
  - ``proxy_score``: the §5.7 degraded weighted heuristic.

Coordinates are floor-relative (X/Y in [-inner/2, +inner/2], Z in [0, inner_h])
when ``geometry`` is omitted. Callers that pass a TCIG-1 descriptor must use
the same frame as that descriptor. Corridor AABBs are broad-phase only.
"""

from __future__ import division

from luggage_description.container_geometry import (
    ContainerGeometry,
    aabb_intersection_volume,
    contains_swept_box,
    normalize_descriptor,
    payload_center_y_interval,
    volume as hull_volume,
)


SUPPORTED_OPENING_SIDE = "negative_x"
WALL_Y_SLOP = 1e-3
HULL_VOLUME_EPS = 1e-9


def require_opening_side(opening_side):
    """Fail closed unless the opening is the supported -X portal."""
    if opening_side != SUPPORTED_OPENING_SIDE:
        raise ValueError(
            "unsupported opening_side: %r (supported: %s)"
            % (opening_side, SUPPORTED_OPENING_SIDE)
        )


def geometry_from_inner_size(inner_size):
    """Cuboid fallback: omission of chamfer is a valid rectangular hull."""
    inner_l, inner_w, inner_h = [float(value) for value in inner_size]
    if inner_l <= 0.0 or inner_w <= 0.0 or inner_h <= 0.0:
        raise ValueError("inner_size values must be positive")
    return normalize_descriptor(
        {
            "schema_version": 1,
            "frame_id": "container_link",
            "length": inner_l,
            "width": inner_w,
            "floor_z": 0.0,
            "ceiling_z": inner_h,
        }
    )


def resolve_geometry(geometry, inner_size):
    """Return a TCIG-1 descriptor; reject inner_size / descriptor mismatches."""
    inner_l, inner_w, inner_h = [float(value) for value in inner_size]
    if geometry is None:
        return geometry_from_inner_size(inner_size)
    if isinstance(geometry, dict):
        geom = normalize_descriptor(geometry)
    elif isinstance(geometry, ContainerGeometry):
        geom = geometry
    else:
        raise TypeError("geometry must be ContainerGeometry, dict, or None")
    if abs(geom.length - inner_l) > 1e-6 or abs(geom.width - inner_w) > 1e-6:
        raise ValueError("geometry descriptor does not match inner_size XY")
    height = geom.ceiling_z - geom.floor_z
    if abs(height - inner_h) > 1e-6:
        raise ValueError("geometry descriptor does not match inner_size height")
    return geom


def corridor_aabb_from_target(
    target_aabb, inner_size, smallest_size, opening_side=SUPPORTED_OPENING_SIDE
):
    """Broad-phase AABB from the -X opening plane to the target near face."""
    require_opening_side(opening_side)
    inner_l = float(inner_size[0])
    ex0, ey0, ez0, ex1, ey1, ez1 = [float(value) for value in target_aabb]
    del ex1
    sd = float(smallest_size[1])
    return (
        -inner_l * 0.5,
        ey0 - sd * 0.5,
        ez0,
        ex0,
        ey1 + sd * 0.5,
        ez1,
    )


def _corridor_to(ems, inner_size, smallest_size, opening_side=SUPPORTED_OPENING_SIDE):
    return corridor_aabb_from_target(ems, inner_size, smallest_size, opening_side)


def box_overlaps_usable_hull(geometry, box):
    """True when the AABB occupies a positive hull-clipped volume."""
    return (
        aabb_intersection_volume(geometry, box[:3], box[3:]) > HULL_VOLUME_EPS
    )


def swept_payload_inside_hull(
    geometry, start_center, end_center, size, yaw=0.0, margin=0.0
):
    """Narrow-phase swept-box hull check (TCIG-1 primitive)."""
    return contains_swept_box(
        geometry,
        start_center,
        end_center,
        size,
        yaw=yaw,
        margin=margin,
    )


def corridor_blocked(
    ems,
    boxes,
    inner_size,
    smallest_size,
    opening_side=SUPPORTED_OPENING_SIDE,
    geometry=None,
    payload_yaw=0.0,
):
    """Is the EMS opening corridor walled off by a placed box?

    Conservative single-box wall: the box intersects the corridor X/Z band,
    occupies usable hull (wedge-only AABB overlap is ignored), and spans the
    payload-center Y interval eroded from the hull at the carry Z range.
    """
    require_opening_side(opening_side)
    geom = resolve_geometry(geometry, inner_size)
    corridor = _corridor_to(ems, inner_size, smallest_size, opening_side)
    cx0, _cy0, cz0, cx1, _cy1, cz1 = corridor
    y_lo, y_hi = payload_center_y_interval(
        geom, smallest_size, cz0, cz1, yaw=float(payload_yaw)
    )
    # Empty eroded interval: the payload cannot occupy this carry Z even with
    # no placed boxes. Fail closed before the ledger loop so an empty map
    # cannot look feasible.
    if y_hi + WALL_Y_SLOP < y_lo:
        return True
    for box in boxes:
        if not (box[0] < cx1 and box[3] > cx0 and box[2] < cz1 and box[5] > cz0):
            continue
        if not box_overlaps_usable_hull(geom, box):
            continue
        if box[1] <= y_lo + WALL_Y_SLOP and box[4] >= y_hi - WALL_Y_SLOP:
            return True
    return False


def blocks_deep_space(
    cand_box,
    ems,
    boxes,
    inner_size,
    smallest_size,
    v_min,
    blocked_tol=0.02,
    opening_side=SUPPORTED_OPENING_SIDE,
    geometry=None,
    payload_yaw=0.0,
):
    """Hull-clipped volume of useful EMS blocked from the opening by cand_box."""
    require_opening_side(opening_side)
    geom = resolve_geometry(geometry, inner_size)
    ems_after = ems.ems_after(cand_box)
    all_boxes = list(boxes) + [cand_box]
    blocked = 0.0
    for space in ems_after.spaces:
        hull_v = aabb_intersection_volume(geom, space[:3], space[3:])
        if hull_v < v_min:
            continue
        if corridor_blocked(
            space,
            all_boxes,
            inner_size,
            smallest_size,
            opening_side,
            geometry=geom,
            payload_yaw=payload_yaw,
        ):
            blocked += hull_v
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


def proxy_score(
    cand,
    model,
    ems,
    inner_size,
    smallest_size,
    reachability_prior=0.5,
    opening_side=SUPPORTED_OPENING_SIDE,
    geometry=None,
    payload_yaw=0.0,
):
    """§5.7 weighted proxy for V̂. Returns (score, breakdown dict)."""
    require_opening_side(opening_side)
    geom = resolve_geometry(geometry, inner_size)
    inner_h = float(inner_size[2])
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
    cand_box = (
        lx - box_l * 0.5,
        ly - box_w * 0.5,
        lz_floor - box_h * 0.5,
        lx + box_l * 0.5,
        ly + box_w * 0.5,
        lz_floor + box_h * 0.5,
    )
    boxes = [
        (b["x0"], b["y0"], b["z0"], b["x1"], b["y1"], b["z1"])
        for b in model.boxes
    ]
    blocked_v, _ = blocks_deep_space(
        cand_box,
        ems,
        boxes,
        inner_size,
        smallest_size,
        v_min,
        opening_side=opening_side,
        geometry=geom,
        payload_yaw=payload_yaw,
    )

    ems_reg = ems.regularity()
    compactness = 1.0 - min(1.0, peak / max(inner_h, 1e-9))
    clearance = max(
        0.0, min(1.0, cand.get("clearance_top", 0.0) / max(box_h, 1e-9))
    )
    conf = cand.get("confidence_ratio", 0.0)
    support_q = 1.0 if cand.get("support_source", "sensor") != "floor_prior" else 0.5
    cog_height = peak / max(inner_h, 1e-9)
    v_ref = hull_volume(geom)
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
