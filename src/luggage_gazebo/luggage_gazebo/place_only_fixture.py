#!/usr/bin/env python3
"""Eval-only perfect-geometry fixtures for the place-only simulation (POS-1).

Pure Python: no ROS, TF, or Gazebo imports. The container hull comes from the
authoritative ``luggage_description.container_geometry`` kernel (itself
ROS-free); everything else (fixture layouts, occupancy diff scoring, swept-path
validation, capacity tests, failure classification) is implemented here
independently from the production placement solver and cargo mapper so it can
cross-check them (docs/plans/place_only_perfect_geometry_sim.md).

Coordinate contract used throughout this module:

- container_link X/Y with the origin at the container center;
- Z is floor-relative (0 at the usable cargo floor, ``inner_h`` at the
  ceiling). The kernel hull helpers take absolute container Z, so this module
  adds ``floor_z`` before calling them.

Sizes are ``(width, depth, height)`` matching SlotSpec/DetectedLuggage.
"""

from __future__ import division

import hashlib
import json
import math
import os
from dataclasses import dataclass, field

from luggage_description.container_geometry import (
    contains_oriented_box,
    contains_swept_box,
    descriptor_from_scene_config,
    y_max_at_z,
)

# Catalog reference sizes (box_catalog.yaml.example). The driver passes the
# live catalog sizes in; these defaults keep unit tests ROS/config free.
CATALOG_SIZES = {
    "carryon": (0.55, 0.40, 0.25),
    "standard": (0.70, 0.45, 0.28),
    "large": (0.80, 0.50, 0.32),
}

INPUT_SOURCE = "eval_perfect_geometry"

REASON_BIN_FULL = "BIN_FULL"
REASON_NO_CANDIDATE = "NO_FEASIBLE_CANDIDATE"

FIXTURE_SETUP_FAILED = "FIXTURE_SETUP_FAILED"

# Aperture side of the calibrated container (portal at container -X).
OPENING_SIDE = "negative_x"


@dataclass(frozen=True)
class FixtureBox(object):
    """One declared fixture box, container_link, floor-relative Z.

    ``in_map`` False declares a planner-blind obstacle: it exists
    physically and in the MoveIt scene, but is NOT committed to the cargo
    map, so placement planning does not know to route around it (the P4
    trap). Everything else is committed through the production map path.
    """

    center: tuple  # (x, y, z) floor-relative
    size: tuple    # (width, depth, height)
    yaw: float = 0.0
    role: str = "fixture"
    in_map: bool = True

    def aabb(self):
        """Floor-relative axis-aligned bounds (yaw must be k*pi/2)."""
        w, d, h = (float(v) for v in self.size)
        x, y, z = (float(v) for v in self.center)
        return (x - w * 0.5, y - d * 0.5, z - h * 0.5,
                x + w * 0.5, y + d * 0.5, z + h * 0.5)

    def record(self):
        return {
            "center": [float(v) for v in self.center],
            "size": [float(v) for v in self.size],
            "yaw": float(self.yaw),
            "role": str(self.role),
            "in_map": bool(self.in_map),
        }


@dataclass(frozen=True)
class CaseSpec(object):
    """One P0-P4 matrix case (docs/plans/place_only_perfect_geometry_sim.md)."""

    case_id: str
    cargo_id: str
    initial: str            # empty | carry | saturated | obstacle
    expect_place: bool
    expect_committed_after: int = 0   # relative to the case fixture baseline
    fail_closed_codes: tuple = ()     # accepted BIN_FULL classifications
    expected_reject_contains: str = ""  # substring in reject message (P3/P4)
    note: str = ""

    def manifest(self, cargo_size):
        return {
            "case_id": self.case_id,
            "cargo_id": self.cargo_id,
            "cargo_size_wdh": [round(float(v), 4) for v in cargo_size],
            "initial_map": self.initial,
            "expect_place": self.expect_place,
            "expect_committed_after": self.expect_committed_after,
            "fail_closed_codes": list(self.fail_closed_codes),
            "input_source": INPUT_SOURCE,
            "note": self.note,
        }


def case_matrix(sizes=None):
    """The exact P0-P4 workload matrix, in streak order."""
    sizes = sizes or CATALOG_SIZES
    return [
        CaseSpec("P0", "carryon", "empty", True, 1,
                 note="successful floor placement and first commit"),
        CaseSpec("P1", "standard", "carry", True, 2,
                 note="non-overlapping placement, second commit"),
        CaseSpec("P2", "large", "carry", True, 3,
                 note="non-overlapping placement, third commit"),
        CaseSpec("P3", "carryon", "saturated", False, 0,
                 fail_closed_codes=(REASON_BIN_FULL, REASON_NO_CANDIDATE),
                 expected_reject_contains="no_candidate",
                 note="fail closed, zero motion, map digest preserved"),
        CaseSpec("P4", "standard", "obstacle", True, 1,
                 note="corridor-blocked tempting slot rejected before "
                      "execution, different collision-free candidate "
                      "succeeds"),
    ]


def hull_from_scene_config(config):
    """Authoritative kernel geometry from a loaded scene config dict."""
    return descriptor_from_scene_config(config)


def hull_context(hull, floor_z):
    """Floor-relative wrappers around the kernel hull helpers."""
    ctx = {
        "hull": hull,
        "floor_z": float(floor_z),
        "inner_l": 2.0 * float(hull.half_x),
        "inner_w": 2.0 * float(hull.half_y),
        "inner_h": float(hull.height),
        "geometry_hash": str(getattr(hull, "geometry_hash", "") or ""),
    }

    def y_max_floor(z_rel, margin=0.0):
        return y_max_at_z(hull, float(floor_z) + float(z_rel), margin=margin)

    def contains_floor_box(center, size, yaw, margin=0.0):
        """True when the whole oriented box is inside the seven-face hull."""
        return contains_oriented_box(
            hull,
            (float(center[0]), float(center[1]),
             float(floor_z) + float(center[2])),
            (float(size[0]), float(size[1]), float(size[2])),
            float(yaw), margin=margin)

    def contains_floor_sweep(center_a, center_b, size, yaw, margin=0.0):
        return contains_swept_box(
            hull,
            (float(center_a[0]), float(center_a[1]),
             float(floor_z) + float(center_a[2])),
            (float(center_b[0]), float(center_b[1]),
             float(floor_z) + float(center_b[2])),
            (float(size[0]), float(size[1]), float(size[2])),
            float(yaw), margin=margin)

    def contains_floor_box_lateral(center, size, yaw, margin=0.010):
        """Hull containment with LATERAL clearance only.

        A floor-resting box touches the floor by design (contact, not
        clearance), so the margin applies to the x walls, the -y wall, the
        chamfer plane, and the ceiling — never the floor. The kernel's
        isotropic margin would reject every legal floor placement.
        """
        from luggage_description.container_geometry import oriented_box_corners
        corners = oriented_box_corners(
            (float(center[0]), float(center[1]),
             float(floor_z) + float(center[2])),
            (float(size[0]), float(size[1]), float(size[2])),
            float(yaw))
        for corner in corners:
            x, y, z = corner
            if not (float(hull.floor_z) - 1e-9 <= z
                    <= float(hull.ceiling_z) - margin):
                return False
            if not (-hull.half_x + margin <= x <= hull.half_x - margin):
                return False
            if y < -hull.half_y + margin:
                return False
            if y > y_max_at_z(hull, z, margin=margin) + 1e-9:
                return False
        return True

    ctx["y_max_at_z"] = y_max_floor
    ctx["contains_floor_box"] = contains_floor_box
    ctx["contains_floor_box_lateral"] = contains_floor_box_lateral
    ctx["contains_floor_sweep"] = contains_floor_sweep
    # Lateral clearance the independent enumeration must agree on with the
    # production planner's hull_margin (0 = strict, matching the kernel).
    ctx["lateral_margin"] = 0.0
    return ctx


def _footprint_for(size, yaw):
    """Axis-aligned footprint (container x, y) after yaw snapping."""
    rotated = abs((float(yaw) % math.pi) - math.pi * 0.5) < (
        math.pi * 0.25)
    w, d, _h = (float(v) for v in size)
    return (d, w) if rotated else (w, d)


def aabb_overlap(a, b, tolerance=1e-9):
    return (a[0] < b[3] - tolerance and a[3] > b[0] + tolerance
            and a[1] < b[4] - tolerance and a[4] > b[1] + tolerance
            and a[2] < b[5] - tolerance and a[5] > b[2] + tolerance)


# ---------------------------------------------------------------------------
# Fixture layouts
# ---------------------------------------------------------------------------

def saturated_fixture_boxes(ctx, cargo_size, clearance_margin=0.03,
                            edge_margin=0.01, stacking_slop=0.02):
    """Two slabs covering the whole usable floor so no footprint fits.

    The slabs respect the seven-face hull: their +Y edge follows the floor
    chamfer bound ``y_max_at_z(0)``. Their height also removes stacking
    (top clearance below ``clearance_margin`` for ``cargo_size``).
    """
    inner_l = ctx["inner_l"]
    inner_w = ctx["inner_w"]
    inner_h = ctx["inner_h"]
    y_max_floor = ctx["y_max_at_z"]

    y_hi = min(y_max_floor(0.0, margin=edge_margin), inner_w * 0.5 - edge_margin)
    y_lo = -inner_w * 0.5 + edge_margin
    cargo_h = float(cargo_size[2])
    slab_h = max(0.0, inner_h - clearance_margin - cargo_h + stacking_slop)
    x0 = -inner_l * 0.5 + edge_margin
    x1 = inner_l * 0.5 - edge_margin
    mid = 0.5 * (x0 + x1)
    slab_w = (mid - x0)
    boxes = [
        FixtureBox(
            center=(x0 + slab_w * 0.5, 0.5 * (y_lo + y_hi), slab_h * 0.5),
            size=(slab_w, y_hi - y_lo, slab_h), role="saturate"),
        FixtureBox(
            center=(x1 - slab_w * 0.5, 0.5 * (y_lo + y_hi), slab_h * 0.5),
            size=(slab_w, y_hi - y_lo, slab_h), role="saturate"),
    ]
    for box in boxes:
        if not ctx["contains_floor_box"](box.center, box.size, box.yaw):
            raise ValueError("saturated slab outside hull: %r" % (box,))
    return boxes


def obstacle_fixture_boxes(ctx, cargo_size, edge_margin=0.01,
                           wall_thickness=0.20, wall_height=0.55,
                           wall_x_center=0.15, low_height=0.20,
                           low_size=(0.25, 1.00, 0.20),
                           low_center=(-0.475, -0.30)):
    """P4: a planner-blind path wall plus a committed low front box.

    - ``path_obstacle`` (in_map=False): spans the full usable floor width
      at mid-length. It is physically present and in the MoveIt scene, but
      not in the cargo map, so the corridor-aware traverse height is NOT
      raised for it and the swept path to any slot beyond it is blocked.
    - ``front_low_box`` (in_map=True): lowers the score of front-floor
      slots (stacking) so the geometrically tempting deep floor slots
      beyond the wall rank first — the candidate the system must not
      blindly execute into.
    """
    inner_w = ctx["inner_w"]
    y_hi = min(ctx["y_max_at_z"](0.0, margin=edge_margin),
               inner_w * 0.5 - edge_margin)
    y_lo = -inner_w * 0.5 + edge_margin
    wall = FixtureBox(
        center=(float(wall_x_center), 0.5 * (y_lo + y_hi), wall_height * 0.5),
        size=(float(wall_thickness), y_hi - y_lo, float(wall_height)),
        role="path_obstacle", in_map=False)
    low = FixtureBox(
        center=(float(low_center[0]), float(low_center[1]),
                float(low_height) * 0.5),
        size=(float(low_size[0]), float(low_size[1]), float(low_height)),
        role="front_low_box", in_map=True)
    for box in (wall, low):
        if not ctx["contains_floor_box"](box.center, box.size, box.yaw):
            raise ValueError("obstacle fixture outside hull: %r" % (box,))
    if wall.aabb()[0] - low.aabb()[3] < 0.05:
        raise ValueError("front box and wall must leave a clear gap")
    return [wall, low]


# ---------------------------------------------------------------------------
# Independent placement-capacity test + candidate replay
# ---------------------------------------------------------------------------

def enumerate_footprints(ctx, size, placed_aabbs, resolution=0.05,
                         clearance_margin=0.03, yaws=(0.0, math.pi / 2.0)):
    """Independent sliding-window enumeration of floor-level candidates.

    Returns a list of deterministic records ``{"cell": (ix, iy, yaw_key,
    peak), "center": (x, y, z), "feasible": bool, "reason": str}``. This
    mirrors the production solver's *contract* (grid sliding window, hull
    corners, placed overlap, top clearance) but not its code, so it can
    arbitrate BIN_FULL. Support peaks are the floor plus committed box tops.
    """
    inner_l, inner_w, inner_h = ctx["inner_l"], ctx["inner_w"], ctx["inner_h"]
    box_h = float(size[2])
    peaks = sorted({0.0} | {float(a[5]) for a in (placed_aabbs or [])})
    out = []
    for yaw in yaws:
        foot_l, foot_w = _footprint_for(size, yaw)
        cells_x = max(1, int(math.ceil(foot_l / resolution - 1e-9)))
        cells_y = max(1, int(math.ceil(foot_w / resolution - 1e-9)))
        nx = max(1, int(round(inner_l / resolution)))
        ny = max(1, int(round(inner_w / resolution)))
        if cells_x > nx or cells_y > ny:
            continue
        for ix0 in range(0, nx - cells_x + 1):
            for iy0 in range(0, ny - cells_y + 1):
                cx = -inner_l * 0.5 + (ix0 + cells_x * 0.5) * resolution
                cy = -inner_w * 0.5 + (iy0 + cells_y * 0.5) * resolution
                for peak in peaks:
                    center = (cx, cy, peak + box_h * 0.5)
                    cand = {
                        "cell": (ix0, iy0, round(float(yaw), 4),
                                 round(peak, 4)),
                        "center": center,
                        "footprint": (foot_l, foot_w),
                        "yaw": float(yaw),
                        "feasible": False,
                        "reason": "",
                    }
                    out.append(cand)
                    if inner_h - (peak + box_h) < clearance_margin:
                        cand["reason"] = "insufficient_clearance"
                        continue
                    if not ctx["contains_floor_box_lateral"](
                            center, (foot_l, foot_w, box_h), yaw,
                            margin=float(ctx.get("lateral_margin", 0.0))):
                        cand["reason"] = "outside_hull"
                        continue
                    box = (cx - foot_l * 0.5, cy - foot_w * 0.5, peak,
                           cx + foot_l * 0.5, cy + foot_w * 0.5,
                           peak + box_h)
                    if any(aabb_overlap(box, placed) for placed in placed_aabbs):
                        cand["reason"] = "overlap"
                        continue
                    cand["feasible"] = True
                    cand["reason"] = "ok"
    return out


def geometric_capacity(ctx, size, placed_aabbs, **kwargs):
    """(fits_any, first_fit) — the independent capacity test for BIN_FULL."""
    for cand in enumerate_footprints(ctx, size, placed_aabbs, **kwargs):
        if cand["feasible"]:
            return True, cand
    return False, None


def replay_candidates(surface, size, placed_aabbs, ctx, resolution=None):
    """Deterministic replay of a saved surface map through the independent
    enumerator, keyed by stable cell ids (revision and score independent)."""
    resolution = float(resolution or surface.get("resolution", 0.05))
    inner = surface.get("inner_size") or [
        ctx["inner_l"], ctx["inner_w"], ctx["inner_h"]]
    peaks = sorted({0.0} | {float(a[5]) for a in (placed_aabbs or [])})
    replay_ctx = dict(ctx)
    replay_ctx["inner_l"] = float(inner[0])
    replay_ctx["inner_w"] = float(inner[1])
    replay_ctx["inner_h"] = float(inner[2])
    cands = enumerate_footprints(
        replay_ctx, size, placed_aabbs, resolution=resolution)
    return {
        "%s" % (cand["cell"],): {
            "feasible": cand["feasible"], "reason": cand["reason"],
        } for cand in cands
    }


# ---------------------------------------------------------------------------
# Swept-path validation (selected candidate vs obstacles / placed boxes)
# ---------------------------------------------------------------------------

def _oriented_corners(center, size, yaw):
    w, d, h = (float(v) for v in size)
    cx, cy, cz = (float(v) for v in center)
    cos_y, sin_y = math.cos(float(yaw)), math.sin(float(yaw))
    corners = []
    for lx in (-w * 0.5, w * 0.5):
        for ly in (-d * 0.5, d * 0.5):
            for lz in (-h * 0.5, h * 0.5):
                corners.append((
                    cx + cos_y * lx - sin_y * ly,
                    cy + sin_y * lx + cos_y * ly,
                    cz + lz))
    return corners


def _box_edge_axes(corners):
    """Three independent edge directions of an oriented box corner set."""
    origin = corners[0]
    axes = []
    for corner in corners[1:]:
        vec = [corner[k] - origin[k] for k in range(3)]
        if sum(abs(v) for v in vec) > 1e-9:
            axes.append(vec)
        if len(axes) == 3:
            break
    return axes


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0])


def _corners_overlap(corners_a, corners_b):
    """SAT overlap test between two oriented (possibly rotated) boxes.

    Face contact is NOT overlap: an axis whose projections merely touch
    (gap within epsilon) separates the boxes, matching the placement
    solver's positive-volume overlap semantics so support-touching
    neighbours are legal placements.
    """
    axes = []
    for corners in (corners_a, corners_b):
        edges = _box_edge_axes(corners)
        axes.extend(edges)
        for i in range(len(edges)):
            for j in range(i + 1, len(edges)):
                cross = _cross(edges[i], edges[j])
                if sum(abs(v) for v in cross) > 1e-9:
                    axes.append(cross)
    for axis in axes:
        pa = [sum(c[k] * axis[k] for k in range(3)) for c in corners_a]
        pb = [sum(c[k] * axis[k] for k in range(3)) for c in corners_b]
        if (max(pa) <= min(pb) + 1e-9 or max(pb) <= min(pa) + 1e-9):
            return False
    return True


def box_overlaps_aabb(center, size, yaw, aabb):
    """Positive-measure overlap between an oriented box and an AABB."""
    corners = _oriented_corners(center, size, yaw)
    lo = [min(c[k] for c in corners) for k in range(3)]
    hi = [max(c[k] for c in corners) for k in range(3)]
    other_lo = [aabb[0], aabb[1], aabb[2]]
    other_hi = [aabb[3], aabb[4], aabb[5]]
    if any(hi[k] <= other_lo[k] + 1e-9 or lo[k] >= other_hi[k] - 1e-9
           for k in range(3)):
        return False
    aabb_corners = [
        (x, y, z)
        for x in (aabb[0], aabb[3])
        for y in (aabb[1], aabb[4])
        for z in (aabb[2], aabb[5])]
    return _corners_overlap(corners, aabb_corners)


def swept_path_blocked(ctx, slot_center, size, yaw, obstacle_aabbs,
                       traverse_contact_z, portal_x=None,
                       sample_step=0.05):
    """Independent swept-geometry check for the selected candidate.

    The place motion is approximated by the production segment geometry:
    a horizontal payload sweep at the traverse/carry height from the portal
    plane to above the slot, then a vertical descent onto the slot. Returns
    ``(blocked, hits)`` where each hit names the colliding obstacle AABB.
    """
    inner_l = ctx["inner_l"]
    portal_x = float(portal_x if portal_x is not None else -inner_l * 0.5)
    box_h = float(size[2])
    sx, sy = float(slot_center[0]), float(slot_center[1])
    # traverse_contact_z is the suction contact height = payload TOP
    # (waypoint generator contract); the payload centre hangs half a box
    # below it.
    carry_z = float(traverse_contact_z) - box_h * 0.5
    final_z = float(slot_center[2])

    hits = []
    x = portal_x
    while x <= sx + 1e-9:
        center = (x, sy, carry_z)
        for index, aabb in enumerate(obstacle_aabbs):
            if box_overlaps_aabb(center, size, yaw, aabb):
                hits.append({
                    "stage": "traverse", "x": round(x, 4), "obstacle": index,
                    "aabb": [round(float(v), 4) for v in aabb]})
                break
        x += sample_step
    # Vertical descent above the slot.
    z = carry_z
    while z >= final_z - 1e-9:
        center = (sx, sy, z)
        for index, aabb in enumerate(obstacle_aabbs):
            if box_overlaps_aabb(center, size, yaw, aabb):
                hits.append({
                    "stage": "descend", "z": round(z, 4), "obstacle": index,
                    "aabb": [round(float(v), 4) for v in aabb]})
                break
        z -= sample_step
    return bool(hits), hits


def corridor_traverse_z(ctx, slot_center, size, placed_boxes,
                        place_clearance_z=0.15, corridor_margin=0.05,
                        portal_x=None):
    """Traverse contact height following the production corridor rule.

    Mirrors ``luggage_planning.waypoint_generator.corridor_clearance``: the
    payload bottom must clear the tallest committed surface between the
    portal and the slot by ``corridor_margin``.
    """
    from luggage_planning.waypoint_generator import corridor_clearance
    inner_l = ctx["inner_l"]
    portal_x = float(portal_x if portal_x is not None else -inner_l * 0.5)
    box_h = float(size[2])
    contact_z = float(slot_center[2]) + box_h * 0.5
    footprint_l, footprint_w = _footprint_for(size, 0.0)
    surface_max = None
    for box in placed_boxes or []:
        aabb = box.aabb() if isinstance(box, FixtureBox) else tuple(box)
        if aabb[3] < portal_x or aabb[0] > float(slot_center[0]):
            continue  # not between portal and slot
        if aabb[4] < float(slot_center[1]) - footprint_w * 0.5 \
                or aabb[1] > float(slot_center[1]) + footprint_w * 0.5:
            continue  # laterally clear of the swept column
        top = float(aabb[5])
        surface_max = top if surface_max is None else max(surface_max, top)
    clearance = corridor_clearance(
        surface_max, box_h, contact_z, float(place_clearance_z),
        margin=float(corridor_margin))
    return contact_z + clearance


# ---------------------------------------------------------------------------
# Occupancy diff scoring
# ---------------------------------------------------------------------------

def surface_digest(surface):
    """Stable digest of the surface grid (includes revision)."""
    payload = {
        "resolution": surface.get("resolution"),
        "nx": surface.get("nx"),
        "ny": surface.get("ny"),
        "map_revision": surface.get("map_revision"),
        "height": [[round(float(v), 4) for v in row]
                   for row in surface.get("height", [])],
        "state": [list(row) for row in surface.get("state", [])],
    }
    blob = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()


def _cell_grid(surface):
    res = float(surface["resolution"])
    nx, ny = int(surface["nx"]), int(surface["ny"])
    inner_l, inner_w = float(surface["inner_size"][0]), float(
        surface["inner_size"][1])
    return res, nx, ny, inner_l, inner_w


def gt_footprint_cells(surface, center_local, size, yaw):
    """Independent rasterization of the oriented footprint at map resolution.

    Re-implements the cargo mapper's rasterization *contract* (not its
    code): an interior sample grid (``ceil(extent/res)+1`` strata, samples
    at stratum centers, spacing below one cell), each sample marking the
    half-open cell that contains it. This marks exactly the cells the box
    covers with positive measure, including for boundary-aligned and
    rotated footprints.
    """
    res, nx, ny, inner_l, inner_w = _cell_grid(surface)
    half_l = inner_l * 0.5
    half_w = inner_w * 0.5
    w, d = float(size[0]), float(size[1])
    sx = max(2, int(math.ceil(w / res)) + 1)
    sy = max(2, int(math.ceil(d / res)) + 1)
    cos_y, sin_y = math.cos(float(yaw)), math.sin(float(yaw))
    cx, cy = float(center_local[0]), float(center_local[1])
    cells = set()
    for ix in range(sx):
        lx = ((ix + 0.5) / float(sx) - 0.5) * w
        for iy in range(sy):
            ly = ((iy + 0.5) / float(sy) - 0.5) * d
            px = cx + cos_y * lx - sin_y * ly
            py = cy + sin_y * lx + cos_y * ly
            gx = int(math.floor((px + half_l) / res))
            gy = int(math.floor((py + half_w) / res))
            if 0 <= gx < nx and 0 <= gy < ny:
                cells.add((gx, gy))
    return cells


def occupancy_diff(pre_surface, post_surface, center_local, size, yaw):
    """Changed-cell diff plus independent GT footprint scoring.

    Returns a dict with the changed cell set, IoU against the independently
    rasterized oriented GT footprint, max height error vs the verified box
    top, and a flag for mutations outside the footprint.
    """
    res, nx, ny, inner_l, inner_w = _cell_grid(post_surface)
    pre = pre_surface.get("height") or []
    post = post_surface.get("height") or []
    changed = set()
    outside = 0
    gt = gt_footprint_cells(post_surface, center_local, size, yaw)
    for ix in range(min(nx, len(post), len(pre))):
        for iy in range(min(ny, len(post[ix]), len(pre[ix]))):
            if abs(float(post[ix][iy]) - float(pre[ix][iy])) > 1e-6:
                changed.add((ix, iy))
                if (ix, iy) not in gt:
                    outside += 1
    intersection = len(changed & gt)
    union = len(changed | gt)
    iou = float(intersection) / float(union) if union else 1.0
    box_top = float(center_local[2]) + 0.5 * float(size[2])
    max_height_error = 0.0
    for ix, iy in changed & gt:
        max_height_error = max(
            max_height_error, abs(float(post[ix][iy]) - box_top))
    return {
        "changed_cells": sorted(changed),
        "n_changed": len(changed),
        "n_gt": len(gt),
        "iou": round(iou, 4),
        "max_height_error_m": round(max_height_error, 4),
        "resolution_m": res,
        "outside_footprint_changes": outside,
        "height_within_one_cell": bool(max_height_error <= res + 1e-9),
        "iou_ok": bool(iou >= 0.90),
    }


# ---------------------------------------------------------------------------
# Classification / ordering / idempotency guards
# ---------------------------------------------------------------------------

def classify_placement_failure(message, capacity_fits, reject_histogram=None):
    """Fail-closed classification guard (debug-evidence rule).

    A ``BIN_FULL``/``no_candidate`` message may only be accepted as genuine
    packing capacity exhaustion when the independent geometric capacity test
    agrees. Otherwise the case is ``PLACE_CANDIDATE_EXHAUSTED`` — candidates
    were lost to other gates and must not be relabelled BIN_FULL.
    """
    text = str(message or "")
    histogram = dict(reject_histogram or {})
    exhaustion = "no_candidate" in text or "invalid_size" in text
    if not exhaustion:
        return {
            "code": text.split(":")[0][:64] or "PLACEMENT_FAILED",
            "bin_full_accepted": False, "capacity_confirmed": False,
            "ok": False,
            "reason": "unexpected placement failure shape: %s" % text,
        }
    code = REASON_BIN_FULL if "BIN_FULL" in text else REASON_NO_CANDIDATE
    accepted = bool(capacity_fits is False)
    return {
        "code": code if accepted else "PLACE_CANDIDATE_EXHAUSTED",
        "bin_full_accepted": accepted,
        "capacity_confirmed": accepted,
        "ok": accepted,
        "reason": (
            "independent capacity test agrees: no footprint fits"
            if accepted else
            "capacity test found a fitting footprint; candidate exhaustion "
            "must not be relabelled BIN_FULL (histogram=%s)" % (histogram,)),
    }


def validate_commit_order(events):
    """events: [(name, t_ros)] with names release|retreat|verify|commit.

    Commit is legal only after release, retreat, and physical verification,
    in that sequence. Equal timestamps are tolerated (probe/dry-run chains
    can transition within one sim tick); only a true reversal fails.
    Returns (ok, reason).
    """
    order = {"release": 0, "retreat": 1, "verify": 2, "commit": 3}
    seen = {}
    for name, t in events:
        if name not in order:
            continue
        if name in seen:
            return False, "duplicate event %s" % name
        seen[name] = float(t)
    for name in order:
        if name not in seen:
            return False, "missing event %s" % name
    seq = [seen["release"], seen["retreat"], seen["verify"], seen["commit"]]
    if any(seq[i] > seq[i + 1] for i in range(len(seq) - 1)):
        return False, "out-of-order commit chain: %s" % seq
    return True, "commit after release->retreat->verify"


def idempotency_check(base_stats, repeat_stats, base_digest, repeat_digest):
    """Repeating an identical commit must not change revision/count/digest."""
    diffs = {}
    for key in ("map_revision", "committed_box_count"):
        a, b = base_stats.get(key), repeat_stats.get(key)
        if a != b:
            diffs[key] = [a, b]
    if base_digest != repeat_digest:
        diffs["digest"] = [base_digest, repeat_digest]
    return {"ok": not diffs, "diffs": diffs}


# ---------------------------------------------------------------------------
# Segment-name guard (shared by the driver's isolation checks)
# ---------------------------------------------------------------------------

PLACE_SEGMENT_NAMES = (
    "transit", "traverse", "insert", "descend", "retreat",
    "stage", "stage_mid", "stage_late",
)
SETUP_SEGMENT_NAMES = ("setup_pre_over_box", "setup_attach")


def unexpected_segment_names(names):
    """Names that are neither scored place-chain nor fixture-setup segments.

    Pick-phase names (pre_grasp/approach/attach/pick_retreat) and anything
    unknown show up here, so a non-empty result is an isolation violation.
    """
    allowed = set(PLACE_SEGMENT_NAMES) | set(SETUP_SEGMENT_NAMES)
    return sorted({str(n) for n in names if str(n) not in allowed})


# ---------------------------------------------------------------------------
# Descriptor + dump contracts
# ---------------------------------------------------------------------------

def perfect_descriptor_fields(cargo_id, size, pose_position, yaw):
    """Fields for the exact DetectedLuggage built from declared geometry."""
    w, d, h = (float(v) for v in size)
    return {
        "id": "eval_%s" % cargo_id,
        "catalog_id": cargo_id,
        "size_wdh": [w, d, h],
        "pose_position": [float(v) for v in pose_position],
        "yaw": float(yaw),
        "yaw_valid": True,
        "height_valid": True,
        "height_confidence": 1.0,
        "height_source": 2,  # HEIGHT_SOURCE_CONFIGURED_SUPPORT (declared)
        "top_surface_valid": True,
        "top_surface_confidence": 1.0,
        "top_surface_z": float(pose_position[2]) + h * 0.5,
        "aspect_ratio": max(w, d) / max(1e-9, min(w, d)),
        "input_source": INPUT_SOURCE,
    }


CASE_REQUIRED_ARTIFACTS = (
    "case_manifest.json",
    "t1_trace.jsonl",
    "surface_pre.json",
    "placement_last_result.json",
)


def dump_artifacts_complete(case_dir, required=None):
    """T0/T1 dump completeness gate (debug-evidence rule)."""
    required = required or CASE_REQUIRED_ARTIFACTS
    missing = [name for name in required
               if not os.path.isfile(os.path.join(case_dir, name))]
    empty = []
    for name in required:
        path = os.path.join(case_dir, name)
        if os.path.isfile(path) and os.path.getsize(path) == 0:
            empty.append(name)
    return {
        "capture_complete": not missing and not empty,
        "missing": missing,
        "empty": empty,
        "replay_possible": not missing,
    }
