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

import numpy as np

from luggage_packing.placement_legacy_v1 import v1_candidate_view


REASON_OVERLAP = "overlap"
REASON_SAME_LAYER_CLEARANCE = "same_layer_clearance"
REASON_OUTSIDE_APERTURE = "outside_aperture"
REASON_OUTSIDE_HULL = "outside_hull"
REASON_CORRIDOR_BLOCKED = "corridor_blocked"
REASON_UNKNOWN_ABOVE_FLOOR = "unknown_above_floor"
REASON_INSUFFICIENT_CLEARANCE = "insufficient_clearance"
REASON_INSUFFICIENT_SUPPORT = "insufficient_support"
# Adjudication 2026-11-08 (edge-occupancy miss): the candidate's ACTUAL
# footprint (continuous extent, grid-anchored) overlaps an occupied cell
# whose top stands above the support-contact band — e.g. the 5mm sliver
# of a near-wall box over the conservatively-occupied wall column that
# the ceil-sized window cells never read.
REASON_EDGE_OCCUPIED = "edge_occupied_overlap"
# ST09-A S1: a support surface whose contact heights ramp across the
# footprint (gentle slope within ``support_tol`` per cell, so support_ratio
# alone cannot see it) does not hold a placed box flat.
REASON_SUPPORT_TILT_UNSUPPORTED = "support_tilt_unsupported"
# Reviews 2026-11-08_1751 (same-snapshot volume authorization): a
# sensor-supported candidate may only be authorized when its OWN box
# volume is observed known-free on the SAME snapshot the surface was
# built from — an observed support face does not authorize unobserved
# space above it. The volume_verifier hook in generate_candidates is
# the product-side gate; these reasons record its verdicts.
REASON_VOLUME_UNKNOWN = "volume_unknown"
REASON_VOLUME_OCCUPIED = "volume_occupied"
REASON_VOLUME_UNVERIFIED = "volume_evidence_unavailable"

# A capacity gate answers "does this box physically fit here, given the
# container hull and what is already inside?". Everything else (aperture,
# insertion corridor, unobserved support) is a policy or observation gate: it
# rejects a slot the container may still have room for. The split matches the
# independent eval enumerator in ``place_only_fixture.enumerate_footprints``
# so the product's own claim and the arbiter's verdict are comparable.
CAPACITY_REASONS = frozenset((
    REASON_OVERLAP, REASON_SAME_LAYER_CLEARANCE, REASON_OUTSIDE_HULL,
    REASON_INSUFFICIENT_CLEARANCE))
POLICY_REASONS = frozenset((
    REASON_OUTSIDE_APERTURE, REASON_CORRIDOR_BLOCKED,
    REASON_UNKNOWN_ABOVE_FLOOR, REASON_INSUFFICIENT_SUPPORT,
    REASON_SUPPORT_TILT_UNSUPPORTED, REASON_VOLUME_UNKNOWN,
    REASON_VOLUME_OCCUPIED, REASON_VOLUME_UNVERIFIED))

# Frozen st09_contracts GeometryVerdict values, mirrored as strings so the
# plain-dict candidate contract stays ROS-free and import-cheap.
VERDICT_LEGAL = "legal"
VERDICT_HULL_VIOLATION = "hull_violation"
VERDICT_APERTURE_VIOLATION = "aperture_violation"
VERDICT_SUPPORT_MISSING = "support_missing"
VERDICT_SUPPORT_TILT_UNSUPPORTED = "support_tilt_unsupported"
VERDICT_OVERLAP = "overlap"
VERDICT_SAME_LAYER_CLEARANCE = "same_layer_clearance"
VERDICT_TOP_CLEARANCE = "top_clearance"

_REASON_TO_VERDICT = {
    None: VERDICT_LEGAL,
    "ok": VERDICT_LEGAL,
    REASON_OUTSIDE_HULL: VERDICT_HULL_VIOLATION,
    REASON_OUTSIDE_APERTURE: VERDICT_APERTURE_VIOLATION,
    REASON_UNKNOWN_ABOVE_FLOOR: VERDICT_SUPPORT_MISSING,
    REASON_INSUFFICIENT_SUPPORT: VERDICT_SUPPORT_MISSING,
    REASON_SUPPORT_TILT_UNSUPPORTED: VERDICT_SUPPORT_TILT_UNSUPPORTED,
    REASON_OVERLAP: VERDICT_OVERLAP,
    REASON_SAME_LAYER_CLEARANCE: VERDICT_SAME_LAYER_CLEARANCE,
    REASON_INSUFFICIENT_CLEARANCE: VERDICT_TOP_CLEARANCE,
    # Volume-verifier verdicts (reviews 2026-11-08_1751): occupied
    # volume is a collision; unknown/unavailable evidence is the same
    # "cannot claim this placement" family as unknown_above_floor.
    REASON_VOLUME_OCCUPIED: VERDICT_OVERLAP,
    REASON_VOLUME_UNKNOWN: VERDICT_SUPPORT_MISSING,
    REASON_VOLUME_UNVERIFIED: VERDICT_SUPPORT_MISSING,
}

#: Frozen geometric tolerances for the adjudication-2026-11-08 checks.
#: MUST equal the independent oracle (scripts/occ_compat/oracle.py:
#: ``POLICY["float_tolerance_m"]`` and ``_OVERLAP_MIN_M``) — unification
#: ruling reviews 2026-11-08_1751 finding 3. ``PENETRATION_EPS_M`` judges
#: the z-penetration side of the support-contact band (the support
#: tolerance governs the GAP side only; 1cm positive-volume penetration
#: rejects at ANY depth). ``OVERLAP_MIN_M`` is the per-axis sliver floor
#: for footprint cell selection — yaw-REPRESENTATION artifacts (~1e-4 m
#: from the 4-decimal yaw encoding) are not positive-area coverage; the
#: discrete-yaw error itself is handled at decode, never by tolerating
#: real volume.
PENETRATION_EPS_M = 1e-9
OVERLAP_MIN_M = 1e-4
#: OCC occupied-overlap tolerance (user ruling 2026-11-08 night): the
#: placement box may clip up to ONE MAP CELL into OCC occupied cells —
#: cell-granularity conservatism (a cell marked occupied may be only
#: partially occupied in reality; boxes may touch/settle). Default
#: None = AUTO = the map's resolution; an explicit float overrides.
#: The EXACT hull geometry (chamfer faces, walls) keeps ZERO tolerance
#: (the hull gate), and UNKNOWN space is not loosened at all — this
#: tolerance governs collisions with OBSERVED occupancy only.
OCC_OVERLAP_TOL_M = None

# Stable placement failure taxonomy (docs/architecture/container_geometry.md).
# The solver emits the first four. The node adds cargo-map identity,
# measured-height, and atlas-identity failures at its own boundary.
# They share this module so every consumer reads one taxonomy.
CODE_INVALID_BOX_SIZE = "INVALID_BOX_SIZE"
CODE_BOX_EXCEEDS_CONTAINER = "BOX_EXCEEDS_CONTAINER"
CODE_BIN_FULL = "BIN_FULL"
CODE_PLACE_CANDIDATE_EXHAUSTED = "PLACE_CANDIDATE_EXHAUSTED"
# ST09-A S1 4.3: zero legal candidates on a map whose unknown cells could
# host a legal placement once observed. Distinct from BIN_FULL (capacity)
# and from corridor exhaustion (route policy).
CODE_OBSERVATION_REQUIRED = "OBSERVATION_REQUIRED"
CODE_CARGO_MAP_GEOMETRY_MISMATCH = "CARGO_MAP_GEOMETRY_MISMATCH"
CODE_DETECT_FULL_GEOMETRY_REQUIRED = "DETECT_FULL_GEOMETRY_REQUIRED"
# Node boundary, same string as atlas_runtime.ATLAS_IDENTITY_UNVERIFIED.
# Strict atlas identity failed: do not hard-reject or re-rank.
CODE_ATLAS_IDENTITY_UNVERIFIED = "ATLAS_IDENTITY_UNVERIFIED"

PLACEMENT_REASON_CODES = frozenset((
    CODE_INVALID_BOX_SIZE, CODE_BOX_EXCEEDS_CONTAINER, CODE_BIN_FULL,
    CODE_PLACE_CANDIDATE_EXHAUSTED, CODE_OBSERVATION_REQUIRED,
    CODE_CARGO_MAP_GEOMETRY_MISMATCH,
    CODE_DETECT_FULL_GEOMETRY_REQUIRED, CODE_ATLAS_IDENTITY_UNVERIFIED))


def candidate_aabb(candidate, inner_h, floor_z=0.0):
    """Candidate AABB in floor-relative container coordinates (ST05).

    Derived from the container box center: ``z_rel = center_container.z -
    floor_z`` (z=0 at the container floor). ``floor_z`` is the floor's
    elevation in the same container frame as ``center_container`` — the
    internal constraint chain (hull/overlap/corridor) stays floor-relative
    by design; support-relative quantities are derived locally, never
    re-embedded. The legacy ``center_local`` path (volume-centered,
    pre-ST05 replay rows) is honored when a candidate carries no
    container center.
    """
    center = candidate.get("center_container")
    if center is not None:
        cx, cy = float(center[0]), float(center[1])
        cz = float(center[2]) - float(floor_z)
    else:
        local = candidate.get("center_local") or [0.0, 0.0, 0.0]
        cx, cy = float(local[0]), float(local[1])
        cz = float(local[2]) + float(inner_h) * 0.5
    footprint = candidate.get("footprint") or candidate["size"][:2]
    height = float(candidate["size"][2])
    half_l = float(footprint[0]) * 0.5
    half_w = float(footprint[1]) * 0.5
    half_h = height * 0.5
    return (
        cx - half_l, cy - half_w, cz - half_h,
        cx + half_l, cy + half_w, cz + half_h,
    )


def aabb_overlap(a, b, tolerance=1e-9):
    """True for positive-volume overlap; touching support faces are allowed."""
    return (
        a[0] < b[3] - tolerance and a[3] > b[0] + tolerance
        and a[1] < b[4] - tolerance and a[4] > b[1] + tolerance
        and a[2] < b[5] - tolerance and a[5] > b[2] + tolerance
    )


# Frozen for this generation (2026-10-05 quick track / ST09-I v2).
# Equality passes; a smaller oriented-footprint gap fails. A later
# change needs a new generation. Wall and corridor margins are separate.
SAME_LAYER_CLEARANCE_M = 0.05


def _rect_corners(cx, cy, yaw, length, width):
    """Corners of a yawed footprint in CONVEX order.

    G3 HULL-GATES (evidence 2026-10-25): the historical crossed order
    ((hl,hw),(hl,-hw),(-hl,hw),(-hl,-hw)) walks a self-intersecting
    bowtie, so the edge normals this feeds to ``_polygon_overlaps`` lose
    the footprint's width axis -- diagonally separated rectangles read as
    overlapping (sampled false positives at 0/5/45/90 deg, evidence
    ``placement_hull_gates_g3_20261024``). Convex order restores the
    exact rectangle axis set; ``_polygon_overlaps`` itself is unchanged
    and stays exact for convex inputs. Only false-positive overlaps are
    removed (true overlaps project identically), so same-layer clearance
    can only gain legal gaps, never lose a detected penetration.
    """
    cosine = math.cos(float(yaw))
    sine = math.sin(float(yaw))
    half_l = 0.5 * float(length)
    half_w = 0.5 * float(width)
    corners = []
    for local_x, local_y in (
            (half_l, half_w), (half_l, -half_w),
            (-half_l, -half_w), (-half_l, half_w)):
        corners.append((
            float(cx) + cosine * local_x - sine * local_y,
            float(cy) + sine * local_x + cosine * local_y,
        ))
    return corners


def _polygon_overlaps(left, right, tolerance=1e-9):
    """True when two convex polygons have positive area in common."""
    def axes(corners):
        out = []
        count = len(corners)
        for index in range(count):
            x1, y1 = corners[index]
            x2, y2 = corners[(index + 1) % count]
            edge_x = x2 - x1
            edge_y = y2 - y1
            norm = math.hypot(edge_x, edge_y) or 1.0
            out.append((-edge_y / norm, edge_x / norm))
        return out

    def project(corners, axis):
        dots = [point[0] * axis[0] + point[1] * axis[1] for point in corners]
        return min(dots), max(dots)

    for axis in axes(left) + axes(right):
        left_min, left_max = project(left, axis)
        right_min, right_max = project(right, axis)
        if left_max <= right_min + tolerance or right_max <= left_min + tolerance:
            return False
    return True


def _candidate_footprint(candidate, floor_z):
    """Yawed landed rectangle and vertical extent of one candidate."""
    center = candidate.get("center_container")
    if center is None:
        box = candidate_aabb(candidate, 0.0, floor_z=floor_z)
        return {
            "cx": 0.5 * (box[0] + box[3]),
            "cy": 0.5 * (box[1] + box[4]),
            "yaw": 0.0,
            "length": box[3] - box[0],
            "width": box[4] - box[1],
            "z_bottom": box[2],
            "z_top": box[5],
        }
    size = candidate.get("size") or (0.0, 0.0, 0.0)
    length = float(size[0])
    width = float(size[1])
    height = float(size[2]) if len(size) > 2 else 0.0
    if "box_yaw" not in candidate:
        footprint = candidate.get("footprint") or size[:2]
        length = float(footprint[0])
        width = float(footprint[1])
        yaw = 0.0
    else:
        yaw = float(candidate.get("box_yaw") or 0.0)
    cz = float(center[2]) - float(floor_z)
    return {
        "cx": float(center[0]),
        "cy": float(center[1]),
        "yaw": yaw,
        "length": length,
        "width": width,
        "z_bottom": cz - 0.5 * height,
        "z_top": cz + 0.5 * height,
    }


def _placed_footprint(placed, floor_z):
    """A placed body is either an AABB tuple or an oriented record."""
    if isinstance(placed, dict):
        center = placed.get("center_xy") or placed.get("center") or (0.0, 0.0)
        size = placed.get("size_lw") or placed.get("footprint") or (0.0, 0.0)
        return {
            "cx": float(center[0]),
            "cy": float(center[1]),
            "yaw": float(placed.get("yaw") or 0.0),
            "length": float(size[0]),
            "width": float(size[1]),
            "z_bottom": float(placed.get("z_bottom", 0.0)) - float(floor_z),
            "z_top": float(placed.get("z_top", 0.0)) - float(floor_z),
        }
    box = tuple(float(v) for v in placed)
    return {
        "cx": 0.5 * (box[0] + box[3]),
        "cy": 0.5 * (box[1] + box[4]),
        "yaw": 0.0,
        "length": box[3] - box[0],
        "width": box[4] - box[1],
        "z_bottom": box[2],
        "z_top": box[5],
    }


def _placed_aabb(placed, floor_z):
    """Axis-aligned bound used only for the solid-penetration gate."""
    if not isinstance(placed, dict):
        return placed
    foot = _placed_footprint(placed, floor_z)
    corners = _rect_corners(
        foot["cx"], foot["cy"], foot["yaw"], foot["length"], foot["width"])
    xs = [point[0] for point in corners]
    ys = [point[1] for point in corners]
    return (min(xs), min(ys), foot["z_bottom"],
            max(xs), max(ys), foot["z_top"])


def _same_layer_reason(candidate, placed_aabbs, floor_z, clearance_m,
                       support_tol):
    """Boundary gap between footprints that share a support surface.

    A box stacked on a placed support parent may cover that parent's
    footprint. It still keeps this gap against every other body on its
    own layer. Solid penetration stays the overlap gate.
    """
    if not placed_aabbs or clearance_m is None or float(clearance_m) <= 0.0:
        return None
    here = _candidate_footprint(candidate, floor_z)
    here_rect = _rect_corners(
        here["cx"], here["cy"], here["yaw"], here["length"], here["width"])
    grown = _rect_corners(
        here["cx"], here["cy"], here["yaw"],
        here["length"] + float(clearance_m),
        here["width"] + float(clearance_m))
    tol = float(support_tol)
    for placed in placed_aabbs:
        other = _placed_footprint(placed, floor_z)
        other_rect = _rect_corners(
            other["cx"], other["cy"], other["yaw"],
            other["length"], other["width"])
        on_parent = (
            abs(here["z_bottom"] - other["z_top"]) <= tol
            and _polygon_overlaps(here_rect, other_rect))
        if on_parent:
            continue
        same_layer = abs(here["z_bottom"] - other["z_bottom"]) <= tol
        if not same_layer:
            continue
        other_grown = _rect_corners(
            other["cx"], other["cy"], other["yaw"],
            other["length"] + float(clearance_m),
            other["width"] + float(clearance_m))
        if _polygon_overlaps(grown, other_grown):
            return REASON_SAME_LAYER_CLEARANCE
    return None


def placement_constraint_verdict(
        candidate, inner_h, placed_aabbs=None, aperture_y=None,
        hull_contains=None, inner_size=None, smallest_size=None, hull=None,
        floor_z=0.0, same_layer_clearance_m=None, support_tol=0.05,
        corridor_as_ranking=False):
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
    box = candidate_aabb(candidate, inner_h, floor_z=floor_z)
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
    overlaps = any(
        aabb_overlap(box, _placed_aabb(placed, floor_z))
        for placed in (placed_aabbs or []))
    if same_layer_clearance_m is None:
        same_layer_clearance_m = SAME_LAYER_CLEARANCE_M
    layer_reason = _same_layer_reason(
        candidate, placed_aabbs, floor_z, same_layer_clearance_m, support_tol)
    capacity_ok = not (outside_hull or overlaps or layer_reason)

    if outside_aperture:
        return REASON_OUTSIDE_APERTURE, capacity_ok
    if outside_hull:
        return REASON_OUTSIDE_HULL, capacity_ok
    if overlaps:
        return REASON_OVERLAP, capacity_ok
    if layer_reason:
        return layer_reason, capacity_ok
    # The production query uses this coarse straight sweep only as a hint.
    # It cannot prove that a full robot+payload trajectory cannot go around.
    if (inner_size is not None and smallest_size is not None):
        from luggage_packing.insertion_corridor import corridor_blocked
        blocked = corridor_blocked(
            box, [_placed_aabb(placed, floor_z) for placed in (placed_aabbs or [])],
            inner_size, smallest_size, hull=hull)
        if corridor_as_ranking:
            candidate.setdefault("detail", {})["straight_corridor_blocked"] = blocked
        elif blocked:
            return REASON_CORRIDOR_BLOCKED, capacity_ok
    return None, capacity_ok


def placement_constraint_reason(
        candidate, inner_h, placed_aabbs=None, aperture_y=None,
        hull_contains=None, inner_size=None, smallest_size=None, hull=None,
        floor_z=0.0, same_layer_clearance_m=None, support_tol=0.05):
    """Return the first hard-constraint reject reason, or ``None``."""
    reason, _capacity_ok = placement_constraint_verdict(
        candidate, inner_h, placed_aabbs=placed_aabbs, aperture_y=aperture_y,
        hull_contains=hull_contains, inner_size=inner_size,
        smallest_size=smallest_size, hull=hull, floor_z=floor_z,
        same_layer_clearance_m=same_layer_clearance_m,
        support_tol=support_tol)
    return reason


def _resolved_hull_margin(cfg, resolution):
    """Resolve the hull margin: None (the default) = one map cell.

    The map's conservative boundary column reaches up to one cell inside
    the exact wall face, so a margin of one resolution guarantees the
    footprint never overlaps a map-occupied ring column regardless of
    the hull-vs-grid anchoring offset. An explicit float (0.0 = exact
    face) overrides."""
    margin = cfg.get("hull_margin")
    if margin is None:
        return float(resolution)
    return float(margin)


def _default_params():
    return {
        "clearance_margin": 0.03,   # required free space above the placed box [m]
        # Same-layer footprint gap. See SAME_LAYER_CLEARANCE_M.
        "same_layer_clearance_m": None,
        "support_tol": 0.05,        # cells within this of the peak count as support [m]
        "min_support_ratio": 0.6,   # required contact ratio of the footprint
        "stride_cells": 1,          # grid stride for the sliding window
        "w_support": 0.5,
        "w_clearance": 0.3,
        "w_compactness": 0.2,       # prefer low placements (good stacking)
        "w_confidence": 0.15,       # penalizes geometry-only or unknown support
        "top_n": 8,
        "keep_rejected": 6,         # rejected candidates kept for visualization
        # --- ST09-A (S1) additions -------------------------------------------
        # Wall margin handed to the hull kernel's closed faces. Default
        # None = AUTO: one map cell (the surface's resolution) — the
        # map's conservative boundary column reaches up to one cell
        # INSIDE the exact wall face, so an exact-face (0.0) margin can
        # still slide a footprint into map-occupied ring cells (the 5 mm
        # anchoring-offset miss, K01 run04/07). Default changed 2026-11-08
        # (user directive margin=resolution); 0.0 restores exact-face.
        "hull_margin": None,
        # OCC occupied-overlap tolerance; None = AUTO = one map cell
        # (see OCC_OVERLAP_TOL_M). An explicit float overrides (0.0
        # restores the strict positive-volume rule).
        "occ_overlap_tolerance": None,
        # Gentle support ramps: a contact plane that ramps across the
        # footprint keeps every cell within support_tol of the peak, so
        # support_ratio alone reads it as full contact — yet the box would
        # rest on the slope's highest cells and tilt. Reject when the
        # contact-cell height span over the longest footprint axis exceeds
        # this ratio (0.05 ~ 2.9 deg; discrete voxel steps of one
        # resolution never fire because only one level stays within tol).
        # defined_by_A: no frozen contract value; provenance recorded in
        # candidate_replay.json. Not widened after live failures.
        "max_support_tilt_ratio": 0.05,
        # Payload symmetry declaration ("none"|"pi"). Only a declared pi
        # symmetry expands theta and theta+pi headings (convergence 5.1).
        "payload_symmetry": "none",
    }


def _surface_hull(surface_map):
    """Rehydrate the chamfered hull kernel from the map's descriptor.

    Returns ``None`` only when the map carries no descriptor at all
    (legacy cuboid fixtures; hull legality then stays with the injected
    validator, exactly as before ST09-A). A map that CARRIES a descriptor
    which fails normalization is foreign, not legacy: silently answering
    from rectangle semantics would be a fail-open degrade, so G3
    HULL-GATES makes it a named error (the node path already fails
    closed on geometry-hash mismatch before reaching here).
    """
    descriptor = surface_map.get("geometry_descriptor")
    if not descriptor:
        return None
    try:
        from luggage_description.container_geometry import normalize_descriptor
        return normalize_descriptor(descriptor)
    except Exception as error:
        raise ValueError(
            "HULL_DESCRIPTOR_INVALID: the surface map carries a geometry "
            "descriptor that does not normalize (%s); refusing rectangle "
            "fallback" % error) from error


def _depth_datum(surface_map, hull):
    """Opening-plane point + inward unit normal for the depth metric.

    Prefers the hull kernel's aperture-derived datum (no hardcoded axis);
    falls back to the map grid's opening-side face when no hull exists.
    """
    if hull is not None:
        from luggage_description.container_geometry import opening_inward_datum
        return opening_inward_datum(hull)
    half_l = 0.5 * float(surface_map["inner_size"][0])
    return (-half_l, 0.0, 0.0), (1.0, 0.0, 0.0)


def _depth_m(center, datum_point, datum_normal):
    """depth = dot(box_center - opening_center, inward_normal) (S1 4.2)."""
    return sum(
        (float(center[i]) - float(datum_point[i])) * float(datum_normal[i])
        for i in range(3))


def _support_layer_z(peak, resolution):
    """Support height quantised into surface-resolution layers (S1 4.2)."""
    if resolution <= 0.0:
        return float(peak)
    return round(math.floor(float(peak) / resolution + 0.5) * resolution, 9)


def _candidate_sort_key(candidate):
    """Deterministic deep-first selection key (frozen PlacementCandidate
    .sorting_key order): support layer low-to-high, depth deep-to-shallow,
    route clearance large-to-small, score high-to-low, stable id last.

    ``route_clearance_m`` stays 0.0 (route_not_evaluated) until ST09-B
    annotates it; equal-zero keeps the key total and deterministic.
    """
    detail = candidate.get("detail") or {}
    return (
        float(candidate.get("support_layer_z", 0.0)),
        -float(candidate.get("depth_m", 0.0)),
        -float(detail.get("route_clearance_m", 0.0) or 0.0),
        bool(detail.get("straight_corridor_blocked", False)),
        -float(candidate.get("score", 0.0)),
        str(candidate.get("candidate_id", "")),
    )


def _support_tilt_ratio(cell_heights, peak, tol):
    """Height span across the *support* cells over the footprint diagonal.

    Cells within ``tol`` of the peak are contact cells; a ramp that rises
    less than ``tol`` per cell registers as full support by ratio, so the
    span across those same cells is what exposes it.
    """
    contact = [h for h in cell_heights if h >= peak - tol]
    if len(contact) < 2:
        return 0.0
    span = max(contact) - min(contact)
    return float(span)


def _hull_legal(center_container, footprint, box_h, hull, margin):
    """Exact seven-face oriented-box legality for a FINAL placement.

    G3 HULL-GATES (evidence 2026-10-25): final containment is full
    closed-hull containment -- the -X aperture face included -- with the
    caller's ``hull_margin`` charged to walls, ceiling and the chamfer
    only. The floor keeps its by-design contact (``floor_margin`` 0): a
    floor-resting box touches the floor, and charging the margin there
    rejects every floor slot (an empty container answered BIN_FULL under
    ``hull_margin=0.01`` -- counterexample saved under evidence
    ``placement_hull_gates_g3_20261024``). Route transit through the
    aperture keeps its own predicate
    (``contains_oriented_box_through_aperture``) at the corridor and
    waypoint call sites; it never comes through here.

    Returns ``(legal, margin_detail)``; ``margin_detail`` names each margin
    class separately (static walls incl. the chamfered face share the hull
    kernel's inset, top clearance and support contact keep their own
    parameters) so consumers can audit which bound moved.
    """
    from luggage_description.container_geometry import contains_oriented_box
    size = (float(footprint[0]), float(footprint[1]), float(box_h))
    legal = contains_oriented_box(
        hull, center_container, size, yaw=0.0, margin=float(margin),
        floor_margin=0.0)
    return bool(legal), {
        "hull_static_wall_and_chamfer_m": float(margin),
    }


def _cell_center_local(surface_map, ix, iy):
    resolution = float(surface_map["resolution"])
    half_l = 0.5 * float(surface_map["inner_size"][0])
    half_w = 0.5 * float(surface_map["inner_size"][1])
    return (
        -half_l + (ix + 0.5) * resolution,
        -half_w + (iy + 0.5) * resolution,
    )


def _hypothetical_floor_windows(surface_map, box_size, allowed_yaws, cfg, hull,
                                candidate_validator=None):
    """Windows that would be legal if the unknown region were observed.

    Occupied testimony stays occupied (this is the S1 4.3 starvation test:
    a corrected hull may reject candidates but must never conjure free space
    out of nothing). Each window is re-evaluated as a flat observed floor
    landing: no occupied column under the footprint, exact hull legality at
    floor level, clearance ok, and — when the caller injected one — the same
    constraint validator the live enumeration used (placed boxes, aperture,
    corridor), so overlap against known cargo cannot be observed away.
    Returns the list of (ix0, iy0, cells_x, cells_y, yaw, depth) tuples,
    best (deepest) first.
    """
    box_l, box_w, box_h = [float(v) for v in box_size]
    resolution = float(surface_map["resolution"])
    cfg["hull_margin"] = _resolved_hull_margin(cfg, resolution)
    if cfg.get("occ_overlap_tolerance") is None:
        cfg["occ_overlap_tolerance"] = resolution
    nx = int(surface_map["nx"])
    ny = int(surface_map["ny"])
    inner_l, inner_w, inner_h = [float(v) for v in surface_map["inner_size"]]
    heights = surface_map["height"]
    states = surface_map["state"]
    floor_container_z = float(surface_map["center_base"][2]) - 0.5 * inner_h
    datum_point, datum_normal = _depth_datum(surface_map, hull)
    out = []
    for box_yaw, rotated in _snap_yaws(allowed_yaws):
        footprint_l, footprint_w = (box_w, box_l) if rotated else (box_l, box_w)
        if footprint_l > inner_l + 1e-6 or footprint_w > inner_w + 1e-6:
            continue
        cells_x, cells_y = _footprint_cells(footprint_l, footprint_w, resolution)
        if cells_x > nx or cells_y > ny:
            continue
        for ix0 in range(0, nx - cells_x + 1):
            for iy0 in range(0, ny - cells_y + 1):
                occupied_or_tall = False
                for ix in range(ix0, ix0 + cells_x):
                    for iy in range(iy0, iy0 + cells_y):
                        if states[ix][iy] == "occupied" and (
                                heights[ix][iy] > cfg["support_tol"]):
                            occupied_or_tall = True
                            break
                    if occupied_or_tall:
                        break
                if occupied_or_tall:
                    continue
                top_of_box = box_h
                if inner_h - top_of_box < cfg["clearance_margin"]:
                    continue
                lx, ly = _cell_center_local(
                    surface_map, ix0 + cells_x / 2.0, iy0 + cells_y / 2.0)
                center = (lx, ly, floor_container_z + box_h * 0.5)
                if hull is not None:
                    legal, _ = _hull_legal(
                        center, (footprint_l, footprint_w), box_h, hull,
                        cfg["hull_margin"])
                    if not legal:
                        continue
                if candidate_validator is not None:
                    probe = {
                        "center_container": list(center),
                        "footprint": [footprint_l, footprint_w],
                        "size": [box_l, box_w, box_h],
                    }
                    verdict = candidate_validator(probe)
                    if isinstance(verdict, tuple):
                        verdict = verdict[0]
                    if verdict:
                        continue
                depth = _depth_m(center, datum_point, datum_normal)
                out.append((ix0, iy0, cells_x, cells_y, box_yaw, depth))
    out.sort(key=lambda row: -row[5])
    return out


def _observation_requirement(surface_map, candidates, hypothetical=None):
    """Build the ObservationRequirement record for the current selection.

    ``footprint_cells``/``corridor_cells`` cover the candidate footprint and
    the aperture-to-slot strip along the inward normal; ``missing_cells``
    lists the still-unknown cells among them with per-cell reasons. No GT,
    no catalog geometry — map states only.
    """
    nx = int(surface_map["nx"])
    states = surface_map["state"]
    target = None
    for candidate in candidates:
        if candidate.get("feasible"):
            target = candidate
            break
    if target is None and hypothetical:
        ix0, iy0, cells_x, cells_y, _yaw, _depth = hypothetical[0]
        target = {
            "ix0": ix0, "iy0": iy0, "cells_x": cells_x, "cells_y": cells_y,
        }
    if target is None:
        return None
    ix0 = int(target["ix0"])
    iy0 = int(target["iy0"])
    cells_x = int(target["cells_x"])
    cells_y = int(target["cells_y"])
    footprint_cells = [
        (ix, iy)
        for ix in range(ix0, ix0 + cells_x)
        for iy in range(iy0, iy0 + cells_y)
    ]
    # Corridor: opening edge (ix=0) to the footprint's far edge, across the
    # footprint's y band — the strip an insert/retreat must traverse.
    corridor_cells = [
        (ix, iy)
        for ix in range(0, ix0 + cells_x)
        for iy in range(iy0, iy0 + cells_y)
        if (ix, iy) not in set(footprint_cells)
    ]
    required = sorted(set(footprint_cells) | set(corridor_cells))
    missing = []
    reasons = []
    for ix, iy in required:
        if 0 <= ix < nx and 0 <= iy < len(states[ix]):
            if states[ix][iy] == "unknown":
                missing.append((ix, iy))
                reasons.append("unknown_%d_%d" % (ix, iy))
    if not missing:
        return None
    return {
        "requirement_id": "obs-req-%s" % (surface_map.get("map_revision"),),
        "candidate_ids": (target.get("candidate_id", ""),),
        "footprint_cells": footprint_cells,
        "corridor_cells": corridor_cells,
        "missing_cells": missing,
        "missing_reasons": reasons,
        "note": (
            "unobserved cells in the target footprint / entry corridor; "
            "bounded observation (<=2 prevalidated views) then re-solve"),
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
        is_rotated = bool(int(round(float(yaw) / (math.pi / 2))) % 2)
        key = 1 if is_rotated else 0
        if key in seen:
            continue
        seen.add(key)
        snapped.append((yaw, is_rotated))
    return snapped


def _headings_for_footprint(allowed_yaws, rotated, expand_half_turn):
    """Keep each explicitly requested directed quarter-turn in its footprint.

    The legacy symmetry switch may add the opposite heading; production passes
    all four explicitly and does not infer headings from box symmetry.
    """
    result = []
    for requested in allowed_yaws or [0.0]:
        quadrant = int(round(float(requested) / (math.pi / 2))) % 4
        if quadrant % 2 != int(rotated):
            continue
        if quadrant not in result:
            result.append(quadrant)
        if expand_half_turn and (quadrant + 2) % 4 not in result:
            result.append((quadrant + 2) % 4)
    return [quadrant * math.pi / 2 for quadrant in result]


def retain_directed_candidates(candidates, limit):
    """Reserve one preferred and one shallow fallback per admitted heading.

    Then retain the remaining packing order. A limit smaller than the number
    of headings cannot preserve them all; callers must expose that budget.
    No new target or heading is invented here.
    """
    limit = max(0, int(limit))
    groups = {}
    for candidate in candidates:
        heading = round(float(candidate['heading_theta']) % (2 * math.pi), 8)
        groups.setdefault(heading, []).append(candidate)
    selected, identities = [], set()

    def retain(candidate):
        identity = candidate['candidate_id']
        if identity not in identities and len(selected) < limit:
            selected.append(candidate)
            identities.add(identity)

    for group in groups.values():
        retain(group[0])
    for group in groups.values():
        retain(min(group, key=lambda row: (float(row.get('support_layer_z', 0)),
                                          float(row.get('depth_m', 0)),
                                          row['candidate_id'])))
    for candidate in candidates:
        retain(candidate)
        if len(selected) == limit:
            break
    return selected


def _footprint_cells(footprint_l, footprint_w, resolution):
    cells_x = max(1, int(math.ceil(footprint_l / resolution - 1e-9)))
    cells_y = max(1, int(math.ceil(footprint_w / resolution - 1e-9)))
    return cells_x, cells_y


def _grid_arrays(surface_map):
    """Numpy views of one surface map (built once per enumeration).

    Exact-parity contract with the per-window Python scans: heights are
    float64 copies of the map's lists; the state masks are boolean and
    the one-cell False/0 padding reproduces the scans' bounds clamping
    (cells outside the grid were skipped, never read)."""
    heights = np.array(surface_map["height"], dtype=float)
    states = surface_map["state"]
    unknown = np.array(
        [[c == "unknown" for c in row] for row in states], dtype=bool)
    occupied = np.array(
        [[c == "occupied" for c in row] for row in states], dtype=bool)
    confidences = surface_map.get("confidence")
    if confidences:
        sensor = np.array(
            [[c == "sensor" for c in row] for row in confidences],
            dtype=bool)
    else:
        sensor = np.zeros_like(unknown)
    pad_f = lambda a: np.pad(a, 1, mode="constant", constant_values=False)  # noqa: E731
    return {
        "heights": heights,
        "heights_p": np.pad(heights, 1, mode="constant",
                            constant_values=0.0),
        "unknown_p": pad_f(unknown),
        "occupied_p": pad_f(occupied),
        "sensor": sensor, "unknown": unknown, "occupied": occupied,
    }


def _sliding(window, shape, stride):
    """Stride-sliced sliding-window view of `window` with shape `shape`."""
    from numpy.lib.stride_tricks import sliding_window_view
    return sliding_window_view(window, shape)[::stride, ::stride]


def _rel_cells(edge, span, cells, resolution):
    """Relative window-cell indices [lo, hi) whose squares overlap
    [edge, edge + span] with more than the sliver floor.

    Evaluates the SAME min/max overlap expression the scalar scan used,
    over the tiny candidate range — exact float parity, no off-by-one
    derivations. The kept set is contiguous (unimodal overlap)."""
    keep = [r for r in range(-1, cells + 2)
            if min(edge + span, (r + 1) * resolution)
            - max(edge, r * resolution) > OVERLAP_MIN_M]
    if not keep:
        return 0, 0
    return keep[0], keep[-1] + 1


def _edge_scan_grid(arrays, peak, nx, ny, resolution, half_l, half_w,
                    footprint_l, footprint_w, cells_x, cells_y, stride,
                    occ_tol):
    """Vectorized _footprint_edge_state over ALL window positions of one
    yaw — exact parity with the per-window scalar scan.

    Geometry identity: lx = -half_l + (ix0 + cells_x/2)*res, so as ix0
    advances by one the footprint shifts by exactly one cell — its
    position relative to the grid lines is CONSTANT per yaw. The cells
    the continuous footprint overlaps (unknown judgment) and the cells
    its tolerance-eroded core reaches (occupied judgment) are therefore
    two constant relative rectangles around the window block, computed
    once with the scalar overlap expression. Inside the window block
    every height <= the window's peak by definition, so occupied
    PENETRATION can only trigger where the eroded core reaches; the
    padding stands in for the clamped out-of-grid skip.

    ``peak`` is the per-window peak array (strided window grid). Returns
    (pen_any, unknown_any) boolean arrays on the same grid."""
    from numpy.lib.stride_tricks import sliding_window_view
    ox, oy = -nx * resolution * 0.5, -ny * resolution * 0.5
    # footprint left edges relative to the window's first grid line
    a_x = (-half_l - ox) + (cells_x * resolution - footprint_l) * 0.5
    a_y = (-half_w - oy) + (cells_y * resolution - footprint_w) * 0.5
    tol_x = min(occ_tol, footprint_l * 0.5)
    tol_y = min(occ_tol, footprint_w * 0.5)

    ulo_x, uhi_x = _rel_cells(a_x, footprint_l, cells_x, resolution)
    ulo_y, uhi_y = _rel_cells(a_y, footprint_w, cells_y, resolution)
    elo_x, ehi_x = _rel_cells(a_x + tol_x, footprint_l - 2.0 * tol_x,
                              cells_x, resolution)
    elo_y, ehi_y = _rel_cells(a_y + tol_y, footprint_w - 2.0 * tol_y,
                              cells_y, resolution)

    def windows(mask, lo_x, hi_x, lo_y, hi_y):
        """Sliding windows of `mask` whose top-left sits at the window
        origin plus the constant relative offset (padded indexing). The
        base is cropped so the LAST window origin equals the
        enumeration's (nx - cells_x, ny - cells_y) — without the crop
        the padding lets the view slide one position past it."""
        if hi_x <= lo_x or hi_y <= lo_y:
            return None
        wx, wy = hi_x - lo_x, hi_y - lo_y
        base = mask[1 + lo_x: 1 + lo_x + (nx - cells_x) + wx,
                    1 + lo_y: 1 + lo_y + (ny - cells_y) + wy]
        return sliding_window_view(base, (wx, wy))[::stride, ::stride]

    shape = (max(0, (nx - cells_x) // stride + 1),
             max(0, (ny - cells_y) // stride + 1))
    unknown_win = windows(arrays["unknown_p"], ulo_x, uhi_x, ulo_y, uhi_y)
    unknown_any = (unknown_win.any(axis=(2, 3))
                   if unknown_win is not None
                   else np.zeros(shape, dtype=bool))

    pen_any = np.zeros(shape, dtype=bool)
    occ_win = windows(arrays["occupied_p"], elo_x, ehi_x, elo_y, ehi_y)
    if occ_win is not None:
        h_win = windows(arrays["heights_p"], elo_x, ehi_x, elo_y, ehi_y)
        pen_any = (occ_win & (h_win > peak[:, :, None, None]
                              + occ_tol)).any(axis=(2, 3))
    return pen_any, unknown_any


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


def _footprint_edge_state(surface_map, cx, cy, footprint_l, footprint_w,
                          box_bottom, overlap_tol=None):
    """Footprint-coverage occupancy/unknown scan (adjudication
    2026-11-08).

    The ceil-sized window cells are anchored to the HULL edge while the
    map grid is anchored at -(nx/2)*resolution — on a grid wider than
    the hull (75 cells * 0.02 = 1.50 m vs inner 1.49 m) the box's real
    extent spills past the last window cell into the adjacent column.
    This scans every grid cell the CONTINUOUS footprint overlaps with
    positive area:

    - occupied cell whose top stands above the box bottom by more than
      the numeric tolerance is a collision at ANY depth (reviews
      2026-11-08 penetration control: the support tolerance governs the
      GAP side only — 1cm positive-volume penetration must reject);
      an occupied top AT/BELOW the box bottom is support contact and
      stays legal;
    - unknown cell -> the footprint holds unobserved volume; the caller
      routes it through the same unknown/floor-prior policy as unknown
      window cells (unknown is never free).
    """
    resolution = float(surface_map["resolution"])
    nx = int(surface_map["nx"])
    ny = int(surface_map["ny"])
    heights = surface_map["height"]
    states = surface_map["state"]
    origin_x = -nx * resolution * 0.5
    origin_y = -ny * resolution * 0.5
    x0, x1 = cx - footprint_l * 0.5, cx + footprint_l * 0.5
    y0, y1 = cy - footprint_w * 0.5, cy + footprint_w * 0.5
    tol = resolution if overlap_tol is None else float(overlap_tol)
    # OCC overlap tolerance (user ruling 2026-11-08 night): the occupied
    # scan runs on the footprint ERODED by one tolerance — a clip up to
    # tol deep into an occupied cell is acceptable; deeper is a
    # collision. UNKNOWN stays on the FULL footprint: missing evidence
    # is an authorization question, never loosened by a collision
    # tolerance.
    ex0 = x0 + min(tol, footprint_l * 0.5)
    ex1 = x1 - min(tol, footprint_l * 0.5)
    ey0 = y0 + min(tol, footprint_w * 0.5)
    ey1 = y1 - min(tol, footprint_w * 0.5)
    overlaps_occupied = False
    overlaps_unknown = False
    ix_lo = max(0, int(math.floor((x0 - origin_x) / resolution)))
    ix_hi = min(nx - 1, int(math.floor(
        (x1 - origin_x - OVERLAP_MIN_M) / resolution)))
    iy_lo = max(0, int(math.floor((y0 - origin_y) / resolution)))
    iy_hi = min(ny - 1, int(math.floor(
        (y1 - origin_y - OVERLAP_MIN_M) / resolution)))
    for ix in range(ix_lo, ix_hi + 1):
        cx0 = origin_x + ix * resolution
        if min(x1, cx0 + resolution) - max(x0, cx0) <= OVERLAP_MIN_M:
            continue
        for iy in range(iy_lo, iy_hi + 1):
            cy0 = origin_y + iy * resolution
            if min(y1, cy0 + resolution) - max(y0, cy0) <= OVERLAP_MIN_M:
                continue
            state = states[ix][iy]
            if state == "occupied":
                # eroded footprint must actually reach this cell
                if (min(ex1, cx0 + resolution) - max(ex0, cx0)
                        > OVERLAP_MIN_M
                        and min(ey1, cy0 + resolution) - max(ey0, cy0)
                        > OVERLAP_MIN_M
                        and float(heights[ix][iy]) > box_bottom + tol):
                    overlaps_occupied = True
            elif state == "unknown":
                overlaps_unknown = True
    return overlaps_occupied, overlaps_unknown


def volume_state_in_evidence(evidence, center, footprint, box_bottom,
                             box_top):
    """Complete-XYZ known-free check of one candidate's OWN box volume
    against same-snapshot voxel evidence (reviews 2026-11-08_1751).

    ``evidence`` is the backend-normalized same-snapshot voxel grid::

        {"resolution": r, "nx": nx, "ny": ny, "nz": nz,
         "origin_xy": [ox, oy],   # container coords of cell (0, 0)
         "floor_z": fz,           # container z of voxel layer 0
         "states": flat list, x-fastest: idx = ix + nx*(iy + ny*iz)}

    State 1 = observed free. 0 (unknown) / 2 (occupied) / 3
    (out-of-domain) / absent are NOT known-free. Every voxel whose cell
    has POSITIVE intersection with the continuous box is checked —
    complete ranges, no boundary shrink: a box at floor-relative
    [0.20, 0.40) on a 2cm grid reads layers 10..19, both boundary
    layers included (each carries 2cm of real thickness; face contact
    with the support below is NOT volume overlap and layer 9 stays
    unchecked). The frozen sliver/penetration floors
    (``OVERLAP_MIN_M`` / ``PENETRATION_EPS_M``) apply per axis. An
    empty range, a range outside the evidence, or short evidence
    rejects fail-closed. Returns None (known-free) or a
    ``REASON_VOLUME_*`` string.
    """
    resolution = float(evidence["resolution"])
    nx = int(evidence["nx"])
    ny = int(evidence["ny"])
    nz = int(evidence["nz"])
    ox, oy = [float(v) for v in evidence["origin_xy"]]
    floor_z = float(evidence["floor_z"])
    states = evidence["states"]
    fw, fd = float(footprint[0]), float(footprint[1])
    x0, x1 = float(center[0]) - fw * 0.5, float(center[0]) + fw * 0.5
    y0, y1 = float(center[1]) - fd * 0.5, float(center[1]) + fd * 0.5
    z0 = float(box_bottom) - floor_z
    z1 = float(box_top) - floor_z
    if (x0 < ox - OVERLAP_MIN_M or x1 > ox + nx * resolution + OVERLAP_MIN_M
            or y0 < oy - OVERLAP_MIN_M
            or y1 > oy + ny * resolution + OVERLAP_MIN_M):
        return REASON_VOLUME_UNVERIFIED   # volume not covered by evidence
    iz_lo = int(math.floor(z0 / resolution + PENETRATION_EPS_M))
    iz_hi = int(math.floor(z1 / resolution - PENETRATION_EPS_M))
    if iz_lo > iz_hi:
        return REASON_VOLUME_UNVERIFIED   # empty range is not a proof
    if iz_lo < 0 or iz_hi >= nz:
        return REASON_VOLUME_UNVERIFIED   # volume not covered by evidence
    ix_lo = max(0, int(math.floor((x0 - ox) / resolution)))
    ix_hi = min(nx - 1, int(math.floor(
        (x1 - ox - OVERLAP_MIN_M) / resolution)))
    iy_lo = max(0, int(math.floor((y0 - oy) / resolution)))
    iy_hi = min(ny - 1, int(math.floor(
        (y1 - oy - OVERLAP_MIN_M) / resolution)))
    expected = nx * ny * nz
    if len(states) < expected:
        return REASON_VOLUME_UNVERIFIED   # short evidence
    # OCC occupied-overlap tolerance: OCCUPIED voxels are judged on the
    # box volume eroded by one tolerance per axis (a clip up to tol deep
    # is acceptable — user ruling 2026-11-08 night). UNKNOWN is judged
    # on the COMPLETE volume: missing evidence is an authorization
    # question and is never loosened by a collision tolerance.
    tol = resolution if evidence.get("overlap_tol") is None else float(
        evidence["overlap_tol"])
    ex0, ex1 = x0 + min(tol, fw * 0.5), x1 - min(tol, fw * 0.5)
    ey0, ey1 = y0 + min(tol, fd * 0.5), y1 - min(tol, fd * 0.5)
    ez0, ez1 = z0 + min(tol, (z1 - z0) * 0.5), z1 - min(
        tol, (z1 - z0) * 0.5)
    for iz in range(iz_lo, iz_hi + 1):
        z_base = iz * resolution
        if min(z1, z_base + resolution) - max(z0, z_base) <= (
                PENETRATION_EPS_M):
            continue
        z_touch = min(ez1, z_base + resolution) - max(ez0, z_base) > (
            PENETRATION_EPS_M)
        for ix in range(ix_lo, ix_hi + 1):
            cx0 = ox + ix * resolution
            if min(x1, cx0 + resolution) - max(x0, cx0) <= OVERLAP_MIN_M:
                continue
            x_touch = (min(ex1, cx0 + resolution) - max(ex0, cx0)
                       > OVERLAP_MIN_M)
            for iy in range(iy_lo, iy_hi + 1):
                cy0 = oy + iy * resolution
                if min(y1, cy0 + resolution) - max(y0, cy0) <= OVERLAP_MIN_M:
                    continue
                state = int(states[ix + nx * (iy + ny * iz)])
                if state == 2:
                    if z_touch and x_touch and (
                            min(ey1, cy0 + resolution) - max(ey0, cy0)
                            > OVERLAP_MIN_M):
                        return REASON_VOLUME_OCCUPIED
                    continue
                if state != 1:
                    return REASON_VOLUME_UNKNOWN
    return None


def volume_evidence_from_pages(pages, *, resolution, nx, ny, nz,
                               origin_xy, floor_z, identity):
    """Assemble paged same-snapshot cell reads into volume evidence.

    ``pages``: ordered raw page mappings with at least
    ``{"status", "states", "identity"}`` (a ReadCells response view; the
    caller owns the transport). ``identity`` is the snapshot identity the
    surface was built from — every page must carry the SAME identity
    (same-snapshot binding: geometry_hash, map_revision, epoch, and
    observed_revision when the backend declares it). Fail-closed: a
    non-OK status, an identity mismatch, or an assembled length !=
    ``nx * ny * nz`` returns ``{"error": ...}`` instead of evidence.
    """
    assembled = []
    for page in pages:
        if str(page.get("status")) != "OK":
            return {"error": "cells_status_%s" % page.get("status")}
        page_identity = page.get("identity") or {}
        for field in ("geometry_hash", "map_revision", "epoch",
                      "observed_revision"):
            if field in identity and field in page_identity:
                if str(page_identity[field]) != str(identity[field]):
                    return {"error": "cells_identity_mismatch_%s" % field}
        assembled.extend(int(state) for state in page.get("states") or ())
    if len(assembled) != nx * ny * nz:
        return {"error": "cells_short_%d_of_%d" % (
            len(assembled), nx * ny * nz)}
    return {
        "resolution": float(resolution),
        "nx": int(nx), "ny": int(ny), "nz": int(nz),
        "origin_xy": [float(origin_xy[0]), float(origin_xy[1])],
        "floor_z": float(floor_z),
        "states": assembled,
    }


def make_volume_verifier(evidence):
    """volume_verifier adapter over volume_state_in_evidence.

    Returns a ``generate_candidates(volume_verifier=...)`` callable; a
    missing/short evidence dict yields a fail-closed rejector instead
    (no evidence, no known-free authorization)."""
    if not evidence or "error" in evidence or "states" not in evidence:
        reason = (evidence or {}).get("error", "volume_evidence_missing")

        def broken(candidate):
            return "%s:%s" % (REASON_VOLUME_UNVERIFIED, reason)
        return broken

    def verifier(candidate):
        center = candidate["center_container"]
        height = float(candidate["size"][2])
        return volume_state_in_evidence(
            evidence, center, candidate["footprint"],
            float(center[2]) - height * 0.5,
            float(center[2]) + height * 0.5)
    return verifier


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
                        candidate_validator=None, totals=None,
                        volume_verifier=None):
    """Return scored placement candidates sorted feasible-first by score.

    ``surface_map`` is the ``surface_map_2d`` dict. ``box_size`` is
    ``[length, width, height]`` in meters.

    ``volume_verifier`` (reviews 2026-11-08_1751) is the same-snapshot
    volume authorization gate: a callable ``(candidate) -> None |
    reason`` consulted for every sensor-supported candidate that passed
    all other gates. None (or a falsy return) authorizes — the candidate
    then carries ``volume_verified: True``; a reason string rejects the
    candidate with that reason. A raising verifier rejects fail-closed.
    ``None`` (default) keeps the legacy surface-only path.

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
    cfg["hull_margin"] = _resolved_hull_margin(cfg, resolution)
    if cfg.get("occ_overlap_tolerance") is None:
        cfg["occ_overlap_tolerance"] = resolution
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
    hull = _surface_hull(surface_map)
    datum_point, datum_normal = _depth_datum(surface_map, hull)
    expand_headings = str(cfg.get("payload_symmetry", "none")).lower() == "pi"
    arrays = _grid_arrays(surface_map)
    for box_yaw, rotated in _snap_yaws(allowed_yaws):
        footprint_l, footprint_w = (box_w, box_l) if rotated else (box_l, box_w)
        if footprint_l > inner_l + 1e-6 or footprint_w > inner_w + 1e-6:
            continue
        cells_x, cells_y = _footprint_cells(footprint_l, footprint_w, resolution)
        if cells_x > nx or cells_y > ny:
            continue
        footprint_class = 1 if rotated else 0
        headings = _headings_for_footprint(allowed_yaws, rotated, expand_headings)

        # Vectorized aggregates for EVERY window of this yaw (exact
        # parity with _window_stats/_support_ratio/_footprint_edge_state;
        # the scalar helpers remain for the legacy enumeration path).
        hw = _sliding(arrays["heights"], (cells_x, cells_y), stride)
        peak_g = np.maximum(hw.max(axis=(2, 3)), 0.0)
        unknown_g = _sliding(
            arrays["unknown"], (cells_x, cells_y), stride).any(axis=(2, 3))
        total_cells = cells_x * cells_y
        sensor_g = _sliding(
            arrays["sensor"], (cells_x, cells_y), stride).sum(axis=(2, 3))
        confidence_g = sensor_g / float(total_cells)
        contact_g = (hw >= peak_g[:, :, None, None]
                     - cfg["support_tol"]).sum(axis=(2, 3))
        support_g = contact_g / float(total_cells)
        pen_g, spill_unknown_g = _edge_scan_grid(
            arrays, peak_g, nx, ny, resolution, half_l, half_w,
            footprint_l, footprint_w, cells_x, cells_y, stride,
            float(cfg["occ_overlap_tolerance"]))
        for i in range(peak_g.shape[0]):
            ix0 = i * stride
            for j in range(peak_g.shape[1]):
                iy0 = j * stride
                peak = float(peak_g[i, j])
                has_unknown = bool(unknown_g[i, j])
                confidence_ratio = float(confidence_g[i, j])
                support_ratio = float(support_g[i, j])
                top_of_box = peak + box_h
                clearance_top = inner_h - top_of_box

                lx = -half_l + (ix0 + cells_x * 0.5) * resolution
                ly = -half_w + (iy0 + cells_y * 0.5) * resolution
                # ST05 F07: the slot truth is the container box center with
                # z = floor + support + h/2; the historical volume-centered
                # fields are derived by the single legacy decoder below.
                floor_container_z = center_base[2] - half_h
                center_container = [
                    lx, ly, floor_container_z + peak + box_h * 0.5]
                depth = _depth_m(center_container, datum_point, datum_normal)
                support_layer = _support_layer_z(peak, resolution)

                feasible = True
                reason = "ok"
                support_source = "sensor"
                # Footprint-coverage scan (adjudication 2026-11-08):
                # the continuous footprint is checked against the grid
                # cells it actually overlaps — the ceil-sized window can
                # miss a 5mm sliver over an adjacent occupied column.
                edge_occupied = bool(pen_g[i, j])
                edge_unknown = bool(spill_unknown_g[i, j])
                if edge_occupied:
                    feasible = False
                    reason = REASON_EDGE_OCCUPIED
                if edge_unknown:
                    has_unknown = True
                if has_unknown:
                    # The footprint holds unobserved volume (window cell
                    # or edge sliver): the claim must not present it as
                    # known — confidence 0 keeps the floor-prior answer
                    # below observed placements (§4.2.2) and matches the
                    # oracle's class check (claim >= 0.5 requires zero
                    # unknown under the footprint).
                    confidence_ratio = 0.0
                # Clearance is a capacity gate, so it is evaluated for every
                # candidate rather than only for the ones that survive the
                # observation gates above it.
                clearance_ok = clearance_top >= cfg["clearance_margin"]
                if feasible and has_unknown:
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
                tilt_record = {
                    "support_span_m": 0.0,
                    "support_tilt_ratio": 0.0,
                    "max_support_tilt_ratio": float(
                        cfg["max_support_tilt_ratio"]),
                }
                if feasible and not clearance_ok:
                    feasible = False
                    reason = REASON_INSUFFICIENT_CLEARANCE
                elif feasible and support_ratio < cfg["min_support_ratio"]:
                    feasible = False
                    reason = REASON_INSUFFICIENT_SUPPORT
                elif feasible:
                    # ST09-A S1: a support ramp within support_tol per cell
                    # reads as full support by ratio; the span across the
                    # contact cells exposes the tilt. (lazy: only windows
                    # that reach this gate pay for the slice)
                    tilt_span = _support_tilt_ratio(
                        arrays["heights"][ix0:ix0 + cells_x,
                                          iy0:iy0 + cells_y].ravel().tolist(),
                        peak, cfg["support_tol"])
                    tilt_ratio = tilt_span / max(footprint_l, footprint_w, 1e-6)
                    if tilt_ratio > float(cfg["max_support_tilt_ratio"]):
                        feasible = False
                        reason = REASON_SUPPORT_TILT_UNSUPPORTED
                    tilt_record = {
                        "support_span_m": round(tilt_span, 6),
                        "support_tilt_ratio": round(tilt_ratio, 6),
                        "max_support_tilt_ratio": float(
                            cfg["max_support_tilt_ratio"]),
                    }

                clearance_score = max(0.0, min(1.0, clearance_top / max(box_h, 1e-6)))
                compactness = 1.0 - min(1.0, peak / max(inner_h, 1e-6))
                w_conf = float(cfg.get("w_confidence", 0.0))
                score = (
                    cfg["w_support"] * support_ratio
                    + cfg["w_clearance"] * clearance_score
                    + cfg["w_compactness"] * compactness
                    + w_conf * confidence_ratio
                )

                # ST09-A S1: exact seven-face hull legality before any
                # truncation, evaluated on every window so the capacity claim
                # never rests on the flat inner_size rectangle. When an
                # injected validator exists (node path) its verdict keeps
                # historical precedence; the internal gate closes the
                # standalone/replay path and both must agree.
                hull_ok = True
                hull_margin_detail = {}
                if hull is not None:
                    hull_ok, hull_margin_detail = _hull_legal(
                        center_container, (footprint_l, footprint_w), box_h,
                        hull, cfg["hull_margin"])

                for heading_index, heading_theta in enumerate(headings):
                    candidate = {
                        "candidate_id": "c%03d_%03d_f%d_h%d" % (
                            ix0, iy0, footprint_class, heading_index),
                        "center_container": center_container,
                        "box_yaw": heading_theta,
                        "heading_theta": heading_theta,
                        "footprint_class": footprint_class,
                        "support_layer_z": support_layer,
                        "depth_m": round(depth, 6),
                        "ix0": ix0,
                        "iy0": iy0,
                        "cells_x": cells_x,
                        "cells_y": cells_y,
                        "footprint": [footprint_l, footprint_w],
                        "size": [box_l, box_w, box_h],
                        "support_score": round(support_ratio, 4),
                        "clearance_score": round(clearance_score, 4),
                        "clearance_top": round(clearance_top, 4),
                        "collision_margin": round(clearance_top, 4),
                        "confidence_ratio": round(confidence_ratio, 4),
                        "support_source": support_source,
                        "volume_verified": False,
                        "reachability_score": -1.0,
                        "score": round(score, 4),
                        "feasible": feasible,
                        "reason": reason,
                        "capacity_feasible": bool(clearance_ok and hull_ok),
                        "detail": {
                            "route_clearance_m": 0.0,
                            "route_not_evaluated": True,
                            "tilt": tilt_record,
                            "margins": {
                                "hull_static_wall_and_chamfer_m": float(
                                    cfg["hull_margin"]),
                                "top_clearance_m": float(cfg["clearance_margin"]),
                                "support_contact_tol_m": float(cfg["support_tol"]),
                                **hull_margin_detail,
                            },
                            "hull_from_descriptor": hull is not None,
                        },
                    }
                    v1_candidate_view(candidate, surface_map)
                    if clearance_ok:
                        # Run the geometry gates even when an observation gate
                        # already rejected the slot: capacity_feasible must not
                        # depend on which gate happened to fire first.
                        constraint_reason, capacity_ok = _validator_verdict(
                            candidate_validator, candidate)
                        candidate["capacity_feasible"] = bool(
                            capacity_ok and hull_ok)
                        if feasible and constraint_reason:
                            candidate["feasible"] = False
                            candidate["reason"] = constraint_reason
                    if feasible and not hull_ok:
                        # No injected validator fired (or none was provided)
                        # and the exact hull rejects the pose.
                        candidate["feasible"] = False
                        candidate["reason"] = REASON_OUTSIDE_HULL
                    if (feasible and candidate["feasible"]
                            and (support_source == "sensor"
                                 or cfg.get("verify_floor_prior_volume", False))
                            and volume_verifier is not None):
                        # Reviews 2026-11-08_1751: the authorization gate.
                        # The candidate's OWN box volume must be observed
                        # known-free on the SAME snapshot the surface was
                        # built from — an observed support face never
                        # authorizes unobserved space above it. A verifier
                        # error is fail-closed: no evidence, no claim.
                        try:
                            volume_reason = volume_verifier(candidate)
                        except Exception:  # noqa: BLE001 - fail closed
                            volume_reason = REASON_VOLUME_UNVERIFIED
                        if volume_reason:
                            candidate["feasible"] = False
                            candidate["reason"] = str(volume_reason)
                        else:
                            candidate["volume_verified"] = True
                    candidate["verdict"] = _REASON_TO_VERDICT.get(
                        candidate["reason"]
                        if candidate["reason"] != "ok" else None,
                        VERDICT_LEGAL,
                    )
                    candidates.append(candidate)

    if totals is not None:
        totals["n_enumerated"] = len(candidates)
        totals["n_capacity_feasible"] = sum(
            1 for c in candidates if c["capacity_feasible"])

    # ST09-A S1 4.2 deterministic deep-first order (frozen sorting_key):
    # support layer low-to-high, depth deep-to-shallow, route clearance,
    # score, stable id. Feasible-first, then rejected in the same order.
    candidates.sort(key=lambda c: (not c["feasible"], _candidate_sort_key(c)))
    feasible = [c for c in candidates if c["feasible"]]
    if cfg.get("retain_heading_diversity", False):
        feasible = retain_directed_candidates(feasible, cfg["top_n"])
    else:
        feasible = feasible[:int(cfg["top_n"])]
    rejected = [c for c in candidates if not c["feasible"]][: int(cfg["keep_rejected"])]
    return feasible + rejected


def solve_placement(surface_map, box_size, allowed_yaws=None, params=None,
                    candidate_validator=None, atlas_annotate=None,
                    volume_verifier=None):
    """ROS-free ComputePlacement core with a stable failure taxonomy.

    On failure ``reason_code`` is one of ``INVALID_BOX_SIZE``,
    ``BOX_EXCEEDS_CONTAINER``, ``BIN_FULL`` or ``PLACE_CANDIDATE_EXHAUSTED``.
    ``BIN_FULL`` is only returned when no enumerated candidate was
    capacity-feasible, so a run that lost every slot to the aperture, the
    insertion corridor, or unobserved support is never reported as a full
    container.

    ``atlas_annotate`` (see placement_reachability.make_atlas_annotate) runs
    after enumeration and before selection: it may flip candidates
    infeasible (atlas_unreachable) and adjust ``score`` (reachability
    prior), so both the reject histogram and the ``max(score)`` selection
    reflect the atlas. Its report is returned as ``result["atlas"]``.
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
        candidate_validator=candidate_validator, totals=totals,
        volume_verifier=volume_verifier)
    atlas_report = None
    if atlas_annotate is not None:
        candidates, atlas_report = atlas_annotate(candidates)
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

    # ST09-A S1 4.2: deterministic deep-first selection over the feasible
    # set (the frozen PlacementCandidate.sorting_key order), replacing the
    # historical max-score pick. All hard gates already ran in enumeration.
    cfg = _default_params()
    if params:
        cfg.update(params)
    cfg["hull_margin"] = _resolved_hull_margin(
        cfg, float(surface_map["resolution"]))
    hull = _surface_hull(surface_map)
    selected = None
    observation_requirement = None
    hypothetical = []
    if feasible:
        selected = min(feasible, key=_candidate_sort_key)
        message = (
            "slot id=%s score=%.3f depth=%.3f layer=%.3f feasible=%d "
            "rejected=%s" % (
                selected.get("candidate_id", ""),
                selected.get("score", 0.0),
                selected.get("depth_m", 0.0),
                selected.get("support_layer_z", 0.0),
                len(feasible), histogram or {}))
        reason_code = ""
    else:
        if not totals.get("n_enumerated"):
            # No sliding window was even enumerated: the footprint or height
            # exceeds the container at every allowed yaw.
            reason_code = CODE_BOX_EXCEEDS_CONTAINER
        else:
            # S1 4.3 starvation test: would ANY window become legal if the
            # unknown region were observed (occupied testimony kept, and the
            # same validator gates — placed boxes cannot be observed away)?
            # If yes this is an observation deficit, not a full container.
            # Evidence-absent rejections (unknown_above_floor) outrank the
            # exhausted-policy claim: observe first, then re-solve — the
            # route/corridor policy may still fail afterwards.
            hypothetical = _hypothetical_floor_windows(
                surface_map, size, allowed_yaws, cfg, hull,
                candidate_validator=candidate_validator)
            starved = bool(histogram.get(REASON_UNKNOWN_ABOVE_FLOOR))
            if (starved or not capacity_feasible_count) and hypothetical:
                reason_code = CODE_OBSERVATION_REQUIRED
            elif capacity_feasible_count:
                reason_code = CODE_PLACE_CANDIDATE_EXHAUSTED
            else:
                reason_code = CODE_BIN_FULL
        parts = " ".join("%s=%d" % (key, value)
                         for key, value in sorted(histogram.items()))
        message = "%s no_candidate%s%s" % (
            reason_code, ": " if parts else "", parts)
        selected = None

    # ObservationRequirement (S1 4.3 / frozen ObservationRequirement): the
    # still-unknown cells in the selected candidate's footprint and entry
    # corridor, for the bounded observation stage (ST09-D) — or, on the
    # starved-map failure path, of the best hypothetical legal window.
    observation_requirement = _observation_requirement(
        surface_map, candidates if feasible else [], hypothetical=hypothetical)

    return {
        "success": bool(feasible),
        # preferred_candidate is the packing preference. It is not a
        # certified selected slot; SelectPlacePathV2 owns that.
        "preferred_candidate": selected,
        "selected": selected,
        "candidates": candidates,
        "reject_histogram": histogram,
        "reason_code": reason_code,
        "capacity_feasible_count": capacity_feasible_count,
        "candidates_enumerated": int(totals.get("n_enumerated", 0)),
        "message": message,
        "map_revision": surface_map.get("map_revision"),
        "atlas": atlas_report,
        "observation_requirement": observation_requirement,
        "hypothetical_legal_windows": [
            {"ix0": row[0], "iy0": row[1], "depth_m": round(row[5], 6)}
            for row in hypothetical[:8]
        ],
    }


def best_candidate(candidates):
    for cand in candidates:
        if cand["feasible"]:
            return cand
    return None


def _reachability_band(candidate):
    """0 reachable, 1 marginal, 2 unknown or no atlas.

    Unknown and no-atlas slots stay in band 2 so a deep unsampled cell
    does not outrank a reachable or marginal one. Reachable and
    marginal slots then prefer the opening side. Unknown slots keep
    the frozen deep-first key.
    """
    status = candidate.get("atlas_status")
    if status == 3:
        return 0
    if status == 2:
        return 1
    return 2


def candidate_prefix_note(returned, feasible_count):
    """One line describing which feasible slots the prefix kept."""
    bands = {}
    depths = []
    for candidate in returned:
        if candidate.get("atlas_status") == 3:
            band = "reachable"
        elif candidate.get("atlas_status") == 2:
            band = "marginal"
        elif str(candidate.get("atlas_status_name") or "") == "no_atlas":
            band = "no_atlas"
        else:
            band = "unknown"
        bands[band] = bands.get(band, 0) + 1
        depths.append(float(candidate.get("depth_m") or 0.0))
    band_text = ",".join(
        "%s:%d" % item for item in sorted(bands.items())) or "none"
    if depths:
        span = "%.3f..%.3f" % (min(depths), max(depths))
    else:
        span = "none"
    return "returned=%d/%d bands=%s depth=%s" % (
        len(returned), int(feasible_count), band_text, span)


def _motion_rank_key(candidate):
    band = _reachability_band(candidate)
    frozen = _candidate_sort_key(candidate)
    if band in (0, 1):
        return (band, float(candidate.get("depth_m") or 0.0),
                frozen[0], -float(candidate.get("score") or 0.0))
    return (2,) + frozen


def ranked_feasible_candidates(result, max_candidates=5):
    """Feasible prefix for ComputePlacement.candidates.

    Groups are reachable, then marginal, then unknown. Inside the
    reachable and marginal groups the shallower slot comes first.
    Unknown slots keep the frozen deep-first key, so ``candidates[0]``
    still matches ``result['preferred_candidate']`` when the atlas did
    not annotate anyone. ``max_candidates <= 0`` uses 5.
    """
    feasible = [
        candidate for candidate in (result.get("candidates") or [])
        if candidate.get("feasible")]
    shared_admission = result.get('admission_policy', {}).get('name') == 'shared_observed_volume'
    feasible.sort(key=_candidate_sort_key if shared_admission else _motion_rank_key)
    limit = int(max_candidates or 0)
    if limit <= 0:
        limit = 5
    if shared_admission:
        return retain_directed_candidates(feasible, limit)
    return feasible[:limit]


class SnapshotCandidateGenerator:
    """Frozen-protocol adapter over ``solve_placement`` (st09_contracts
    ``CandidateGenerator``): pure, no motion requests, never BIN_FULL from a
    single failed insertion corridor.

    The frozen ``PlanningSnapshot`` is a coarse cell-state grid (no height
    field, no hull descriptor), so the adapter takes the missing geometry
    out-of-band at construction: ``geometry_descriptor`` (preferred —
    rehydrates the exact chamfered hull and the aperture depth datum) or a
    plain ``inner_size`` fallback, plus the payload (dimensions + symmetry)
    the snapshot itself cannot carry. ``generate_candidates`` then only has
    to match the digest, keeping the frozen call signature intact.
    """

    def __init__(self, payload, geometry_descriptor=None, inner_size=None,
                 allowed_yaws=None, params=None):
        self._payload = payload
        self._geometry_descriptor = geometry_descriptor
        self._inner_size = inner_size
        self._allowed_yaws = allowed_yaws
        self._params = dict(params or {})

    # -- internals -------------------------------------------------------

    def _surface_from_snapshot(self, snapshot):
        from st09_contracts.planning import OccupancyCellState
        resolution = float(snapshot.resolution_m)
        cells_i = tuple(snapshot.cells_i)
        cells_j = tuple(snapshot.cells_j)
        cell_states = tuple(snapshot.cell_states)
        nx = (max(cells_i) + 1) if cells_i else 1
        ny = (max(cells_j) + 1) if cells_j else 1
        state = [["unknown"] * ny for _ in range(nx)]
        height = [[0.0] * ny for _ in range(nx)]
        confidence = [["none"] * ny for _ in range(nx)]
        for i, j, cell_state in zip(cells_i, cells_j, cell_states):
            i, j = int(i), int(j)
            if i >= nx or j >= ny:
                raise ValueError("snapshot cell outside derived grid")
            label = str(OccupancyCellState(cell_state).value)
            state[i][j] = label
            if label == "occupied":
                # One-voxel marker height: conservative (forces stacking z,
                # keeps occupied columns out of floor landings).
                height[i][j] = resolution
                confidence[i][j] = "sensor"
            elif label == "free":
                confidence[i][j] = "sensor"
        if self._geometry_descriptor is not None:
            inner_size = list(self._geometry_descriptor.get(
                "inner_size",
                [float(self._geometry_descriptor["length"]),
                 float(self._geometry_descriptor["width"]),
                 float(self._geometry_descriptor["ceiling_z"])
                 - float(self._geometry_descriptor["floor_z"])]))
            center_z = 0.5 * (
                float(self._geometry_descriptor["ceiling_z"])
                + float(self._geometry_descriptor["floor_z"]))
        else:
            if self._inner_size is None:
                raise ValueError(
                    "SnapshotCandidateGenerator needs geometry_descriptor "
                    "or inner_size (the frozen snapshot carries neither)")
            inner_size = [float(v) for v in self._inner_size]
            center_z = 0.5 * float(inner_size[2])
        return {
            "resolution": resolution,
            "nx": nx,
            "ny": ny,
            "inner_size": inner_size,
            "center_base": [0.0, 0.0, center_z],
            "yaw": 0.0,
            "height": height,
            "state": state,
            "confidence": confidence,
            "map_revision": int(snapshot.map_revision),
            "geometry_descriptor": self._geometry_descriptor,
        }

    def _to_contract(self, candidate, hull_present):
        from st09_contracts.planning import GeometryVerdict, PlacementCandidate
        yaw = float(candidate.get("heading_theta", 0.0) or 0.0)
        half = 0.5 * yaw
        quaternion = (0.0, 0.0, math.sin(half), math.cos(half))
        detail = dict(candidate.get("detail") or {})
        detail["box_yaw_requested"] = candidate.get("box_yaw")
        detail["support_source"] = candidate.get("support_source")
        detail["hull_from_descriptor"] = hull_present
        return PlacementCandidate(
            candidate_id=str(candidate["candidate_id"]),
            footprint_class=int(candidate["footprint_class"]),
            heading_theta=float(candidate["heading_theta"]),
            position_container=tuple(
                float(v) for v in candidate["center_container"]),
            quaternion_xyzw_container=quaternion,
            support_layer_z=float(candidate["support_layer_z"]),
            depth_m=float(candidate["depth_m"]),
            verdict=GeometryVerdict(str(candidate["verdict"])),
            score=float(candidate["score"]),
            detail=detail,
        )

    # -- frozen protocol -------------------------------------------------

    def generate_candidates(self, snapshot, payload_digest):
        from st09_contracts.planning import ObservationRequirement
        payload = self._payload
        expected = getattr(payload, "digest", None)
        if expected is not None and payload_digest is not None:
            if str(payload_digest) != str(expected()):
                raise ValueError(
                    "payload_digest does not match the constructed payload")
        params = dict(self._params)
        params.setdefault(
            "payload_symmetry",
            str(getattr(payload, "symmetry", "none") or "none").lower(),
        )
        dimensions = tuple(
            float(v) for v in payload.dimensions_wdh)
        surface = self._surface_from_snapshot(snapshot)
        result = solve_placement(
            surface, list(dimensions), allowed_yaws=self._allowed_yaws,
            params=params)
        hull_present = surface.get("geometry_descriptor") is not None
        contract_candidates = [
            self._to_contract(candidate, hull_present)
            for candidate in result["candidates"]
        ]
        requirement = None
        raw = result.get("observation_requirement")
        if raw:
            requirement = ObservationRequirement(
                requirement_id=str(raw["requirement_id"]),
                candidate_ids=tuple(str(c) for c in raw["candidate_ids"]),
                footprint_cells=tuple(
                    tuple(int(v) for v in cell)
                    for cell in raw["footprint_cells"]),
                corridor_cells=tuple(
                    tuple(int(v) for v in cell)
                    for cell in raw["corridor_cells"]),
                missing_cells=tuple(
                    tuple(int(v) for v in cell)
                    for cell in raw["missing_cells"]),
                missing_reasons=tuple(
                    str(reason) for reason in raw["missing_reasons"]),
                note=str(raw.get("note", "")),
            )
        return contract_candidates, requirement


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
