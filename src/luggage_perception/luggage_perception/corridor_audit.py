#!/usr/bin/env python3
"""Corridor audit helpers for place-path wiring (G4 + E1, no ROS).

Given the commit ledger (from cargo_volume_mapper_node) and the chosen slot,
compute the corridor AABB, the surface max along it, the required carry
height, and whether the swept corridor is free / occupied / unknown.

Pure module so waypoint_generator_node, the pack driver and unit tests all
share one definition (docs/plans/corridor_constraints.md layer 2).
"""

from __future__ import division

import math

# E1 verdicts for the swept-corridor occupancy query.
CORRIDOR_FREE = "free"
CORRIDOR_OCCUPIED = "occupied"     # reject slot (fail-closed on unknown too)
CORRIDOR_UNKNOWN = "unknown"       # fail-closed: treat as not placeable now
CORRIDOR_EMPTY_MAP = "empty_map"   # no committed geometry: trivially free


def corridor_aabb(slot_center_local, slot_size, inner_size, smallest_size,
                  opening_side="negative_x"):
    """AABB from the opening plane to the slot near face (container-local).

    Same convention as insertion_corridor._corridor_to, factored out so the
    audit and the placement gate cannot drift apart.
    """
    inner_l = float(inner_size[0])
    inner_w = float(inner_size[1])
    cx, cy, cz = [float(v) for v in slot_center_local]
    w, d, h = [float(v) for v in slot_size]
    sw, sd = float(smallest_size[0]), float(smallest_size[1])
    ex0, ex1 = cx - w * 0.5, cx + w * 0.5
    ey0, ey1 = cy - d * 0.5, cy + d * 0.5
    ez0, ez1 = cz - h * 0.5, cz + h * 0.5
    if opening_side == "negative_x":
        x0, x1 = -inner_l * 0.5, ex0
    else:
        x0, x1 = ex1, inner_l * 0.5
    return (x0, ey0 - sd * 0.5, ez0, x1, ey1 + sd * 0.5, ez1)


def corridor_surface_max(ledger_boxes, corridor):
    """Highest committed-box top inside the corridor AABB.

    ``ledger_boxes``: iterable of (center_local, size) from the commit
    ledger. Returns ``None`` when the corridor contains no committed box.
    """
    x0, y0, _z0, x1, y1, _z1 = corridor
    top = None
    for center, size in ledger_boxes or []:
        cx, cy, _cz = [float(v) for v in center]
        w, d, h = [float(v) for v in size]
        if (cx + w * 0.5 <= x0 or cx - w * 0.5 >= x1
                or cy + d * 0.5 <= y0 or cy - d * 0.5 >= y1):
            continue
        candidate = _cz + h * 0.5
        top = candidate if top is None else max(top, candidate)
    return top


def required_carry_z(corridor_surface_max_z, box_height,
                      margin=0.05):
    """Suction-frame height that keeps the payload bottom above the surface.

    The payload hangs a full box height below the suction frame during
    tool-down carry, so the requirement is surface + box_height + margin
    (not half - see waypoint_generator.corridor_clearance).
    """
    if corridor_surface_max_z is None:
        return None
    return (float(corridor_surface_max_z) + max(0.0, float(box_height))
            + float(margin))


def audit_corridor(slot_center_local, slot_size, ledger_boxes, inner_size,
                   smallest_size, opening_side="negative_x"):
    """One-shot corridor audit for a candidate slot.

    Returns a dict with the corridor AABB, the surface max, the required
    carry height and the E1-relevant verdict. ``verdict`` is one of
    CORRIDOR_FREE / CORRIDOR_OCCUPIED / CORRIDOR_UNKNOWN / CORRIDOR_EMPTY_MAP.
    """
    corridor = corridor_aabb(
        slot_center_local, slot_size, inner_size, smallest_size, opening_side)
    boxes_in = []
    x0, y0, z0, x1, y1, z1 = corridor
    for center, size in ledger_boxes or []:
        cx, cy, cz = [float(v) for v in center]
        w, d, h = [float(v) for v in size]
        overlaps = not (cx + w * 0.5 <= x0 or cx - w * 0.5 >= x1
                        or cy + d * 0.5 <= y0 or cy - d * 0.5 >= y1
                        or cz + h * 0.5 <= z0 or cz - h * 0.5 >= z1)
        if overlaps:
            boxes_in.append((center, size))
    surface_max = corridor_surface_max(ledger_boxes, corridor)
    if not boxes_in:
        verdict = CORRIDOR_EMPTY_MAP if not ledger_boxes else CORRIDOR_FREE
    else:
        # The swept AABB overlaps at least one committed box in the entry
        # band. A committed box whose top lies below the corridor z-band
        # does not clip the carry path; one inside the band does.
        clips = any(
            float(c[2]) + float(s[2]) * 0.5 > z0
            and float(c[2]) - float(s[2]) * 0.5 < z1
            for c, s in boxes_in)
        verdict = CORRIDOR_OCCUPIED if clips else CORRIDOR_FREE
    return {
        "corridor_aabb": [round(v, 4) for v in corridor],
        "surface_max": None if surface_max is None else round(surface_max, 4),
        "required_carry_z": (
            None if surface_max is None
            else round(required_carry_z(
                surface_max, slot_size[2]), 4)),
        "boxes_in_corridor": len(boxes_in),
        "verdict": verdict,
    }
