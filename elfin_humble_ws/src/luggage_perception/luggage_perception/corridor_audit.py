#!/usr/bin/env python3
"""Corridor audit helpers for place-path wiring (G4 + E1, no ROS).

Given the commit ledger (from cargo_volume_mapper_node) and the chosen slot,
compute the corridor AABB, the surface max along it, the required carry
height, and whether the swept corridor is free / occupied / unknown.

Shares insertion_corridor broad-phase AABBs and TCIG-1 hull operations with
the placement gate (docs/plans/corridor_constraints.md layer 2).
"""

from __future__ import division

from luggage_packing.insertion_corridor import (
    SUPPORTED_OPENING_SIDE,
    box_overlaps_usable_hull,
    corridor_aabb_from_target,
    require_opening_side,
    resolve_geometry,
    swept_payload_inside_hull,
)

# E1 verdicts for the swept-corridor occupancy query.
CORRIDOR_FREE = "free"
CORRIDOR_OCCUPIED = "occupied"     # reject slot (fail-closed on unknown too)
CORRIDOR_UNKNOWN = "unknown"       # fail-closed: treat as not placeable now
CORRIDOR_EMPTY_MAP = "empty_map"   # no committed geometry: trivially free


def _box_aabb(center, size):
    cx, cy, cz = [float(value) for value in center]
    width, depth, height = [float(value) for value in size]
    return (
        cx - width * 0.5,
        cy - depth * 0.5,
        cz - height * 0.5,
        cx + width * 0.5,
        cy + depth * 0.5,
        cz + height * 0.5,
    )


def corridor_aabb(
    slot_center_local,
    slot_size,
    inner_size,
    smallest_size,
    opening_side=SUPPORTED_OPENING_SIDE,
):
    """AABB from the opening plane to the slot near face (container-local).

    Same convention as insertion_corridor.corridor_aabb_from_target.
    """
    target = _box_aabb(slot_center_local, slot_size)
    return corridor_aabb_from_target(
        target, inner_size, smallest_size, opening_side
    )


def corridor_surface_max(ledger_boxes, corridor, geometry=None):
    """Highest committed-box top inside the corridor AABB.

    ``ledger_boxes``: iterable of (center_local, size) from the commit
    ledger. Returns ``None`` when the corridor contains no committed box.
    Broad-phase overlap is not enough: a wedge-only AABB that occupies no
    usable hull volume does not raise carry height.
    """
    x0, y0, _z0, x1, y1, _z1 = corridor
    top = None
    for center, size in ledger_boxes or []:
        aabb = _box_aabb(center, size)
        if (
            aabb[3] <= x0
            or aabb[0] >= x1
            or aabb[4] <= y0
            or aabb[1] >= y1
        ):
            continue
        if geometry is not None and not box_overlaps_usable_hull(geometry, aabb):
            continue
        candidate = aabb[5]
        top = candidate if top is None else max(top, candidate)
    return top


def required_carry_z(corridor_surface_max_z, box_height, margin=0.05):
    """Suction-frame height that keeps the payload bottom above the surface.

    The payload hangs a full box height below the suction frame during
    tool-down carry, so the requirement is surface + box_height + margin
    (not half - see waypoint_generator.corridor_clearance).
    """
    if corridor_surface_max_z is None:
        return None
    return (
        float(corridor_surface_max_z)
        + max(0.0, float(box_height))
        + float(margin)
    )


def audit_corridor(
    slot_center_local,
    slot_size,
    ledger_boxes,
    inner_size,
    smallest_size,
    opening_side=SUPPORTED_OPENING_SIDE,
    geometry=None,
    payload_yaw=0.0,
):
    """One-shot corridor audit for a candidate slot.

    Returns a dict with the corridor AABB, the surface max, the required
    carry height and the E1-relevant verdict. ``verdict`` is one of
    CORRIDOR_FREE / CORRIDOR_OCCUPIED / CORRIDOR_UNKNOWN / CORRIDOR_EMPTY_MAP.
    """
    require_opening_side(opening_side)
    geom = resolve_geometry(geometry, inner_size)
    corridor = corridor_aabb(
        slot_center_local, slot_size, inner_size, smallest_size, opening_side
    )
    boxes_in = []
    x0, y0, z0, x1, y1, z1 = corridor
    for center, size in ledger_boxes or []:
        aabb = _box_aabb(center, size)
        overlaps = not (
            aabb[3] <= x0
            or aabb[0] >= x1
            or aabb[4] <= y0
            or aabb[1] >= y1
            or aabb[5] <= z0
            or aabb[2] >= z1
        )
        if overlaps and box_overlaps_usable_hull(geom, aabb):
            boxes_in.append((center, size, aabb))
    surface_max = corridor_surface_max(ledger_boxes, corridor, geometry=geom)
    slot_center = [float(value) for value in slot_center_local]
    opening_center = (
        -float(inner_size[0]) * 0.5 + float(slot_size[0]) * 0.5,
        float(slot_center_local[1]),
        float(slot_center_local[2]),
    )
    sweep_ok = swept_payload_inside_hull(
        geom,
        opening_center,
        slot_center,
        slot_size,
        yaw=float(payload_yaw),
    )
    if not sweep_ok:
        verdict = CORRIDOR_OCCUPIED
    elif not boxes_in:
        verdict = CORRIDOR_EMPTY_MAP if not ledger_boxes else CORRIDOR_FREE
    else:
        clips = any(aabb[5] > z0 and aabb[2] < z1 for _c, _s, aabb in boxes_in)
        verdict = CORRIDOR_OCCUPIED if clips else CORRIDOR_FREE
    return {
        "corridor_aabb": [round(value, 4) for value in corridor],
        "surface_max": None if surface_max is None else round(surface_max, 4),
        "required_carry_z": (
            None
            if surface_max is None
            else round(required_carry_z(surface_max, slot_size[2]), 4)
        ),
        "boxes_in_corridor": len(boxes_in),
        "verdict": verdict,
    }
