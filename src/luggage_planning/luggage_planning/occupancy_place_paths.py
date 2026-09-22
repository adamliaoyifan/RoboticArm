#!/usr/bin/env python3
"""ROS-free occupancy sweep, place-path variants, and trajectory selector.

Algorithm module: do not import ``rclpy`` or ``moveit_msgs``. Nodes convert
frames and call MoveIt; this file only reasons on ``surface_2d`` plus
polylines in the map frame (the same frame as ``center_base``).
"""

from __future__ import division

import math
import os
from dataclasses import dataclass, field

from luggage_description.container_geometry import (
    contains_oriented_box,
    contains_oriented_box_through_aperture,
    contains_point,
    contains_point_through_aperture,
    normalize_descriptor,
)

PLACE_PATH_INFEASIBLE = "PLACE_PATH_INFEASIBLE"

PAYLOAD_SOURCE_MEASURED = "current_box_measured"
PAYLOAD_SOURCE_DEFAULT = "static_default"


def resolve_payload_wdh(measured, defaults):
    """Pick the occupancy-sweep payload box, measured first.

    ``measured`` is the /luggage/current_box measured record (or None);
    ``defaults`` is the fallback [w, d, h] envelope. Returns
    ``(wdh, source)`` where source is ``current_box_measured`` or
    ``static_default``. The measured record is the perception estimate
    (RANSAC top + PCA footprint + platform-free height); the default is a
    conservative envelope used only when no measurement has been synced.
    """
    if isinstance(measured, dict):
        try:
            wdh = [
                float(measured["width"]),
                float(measured["depth"]),
                float(measured["height"]),
            ]
        except (KeyError, TypeError, ValueError):
            wdh = []
        if len(wdh) == 3 and all(v > 0.0 and math.isfinite(v) for v in wdh):
            return wdh, PAYLOAD_SOURCE_MEASURED
    return [float(v) for v in defaults], PAYLOAD_SOURCE_DEFAULT


DEFAULT_WEIGHTS = {
    "w_placement": 1.0,
    "w_dimension": 1.0,
    "w_safety": 1.5,
    "w_efficiency": 0.5,
    "safety_ref_m": 0.10,
    "inflate_m": 0.05,
    "arm_radius_m": 0.08,
    "max_candidates": 5,
    "max_candidates_per_slot": 4,
    "cartesian_min_fraction": 0.95,
    "max_occ_objects": 200,
    "occupancy_freshness_s": 0.0,
    "support_tol": 0.05,
    # Payload-bottom gap above occupied box tops (also covers ~10 mm FCL pad).
    "carry_margin_m": 0.05,
    "collision_pad_m": 0.02,
    # Wrist + camera stack above suction in tool-down; MoveIt still owns
    # mesh-vs-ceiling, this only keeps the geometric hover under the lid.
    "arm_overhead_m": 0.20,
}

OFFSET_DELTAS = (0.10, 0.20, -0.10, -0.20)


class OccupancyMismatch(ValueError):
    """geometry_hash / map_revision pin failed. Always fail closed."""

    reason = PLACE_PATH_INFEASIBLE


def stamp_geometry_descriptor(surface, hull=None):
    """Copy ``surface`` and fill ``geometry_descriptor`` when a hull is known.

    Online ``from_surface_2d`` still fails closed if the descriptor is
    missing. Replay tools must join a dump with the current scene_tf hull;
    this helper never invents a cuboid from ``inner_size``.
    """
    out = dict(surface or {})
    if isinstance(out.get("geometry_descriptor"), dict) and out["geometry_descriptor"]:
        return out
    if hull is None:
        return out
    desc = hull.descriptor()
    out["geometry_descriptor"] = desc
    if not out.get("geometry_hash"):
        out["geometry_hash"] = desc["geometry_hash"]
    return out


def clip01(value):
    if value <= 0.0:
        return 0.0
    if value >= 1.0:
        return 1.0
    return float(value)


def _unit_xy(dx, dy):
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return (1.0, 0.0)
    return (dx / length, dy / length)


def _aabb_distance(a, b):
    """Min XY distance between two AABBs ``(xmin, ymin, xmax, ymax)``.

    Overlap returns 0.
    """
    dx = max(0.0, a[0] - b[2], b[0] - a[2])
    dy = max(0.0, a[1] - b[3], b[1] - a[3])
    if dx == 0.0 and dy == 0.0:
        return 0.0
    return math.hypot(dx, dy)


def _rotated_footprint(cx, cy, width, depth, yaw):
    """Axis-aligned AABB of a yaw-rotated rectangle (conservative)."""
    hw, hd = 0.5 * abs(width), 0.5 * abs(depth)
    c, s = math.cos(yaw), math.sin(yaw)
    xs, ys = [], []
    for sx, sy in ((-hw, -hd), (-hw, hd), (hw, -hd), (hw, hd)):
        xs.append(cx + c * sx - s * sy)
        ys.append(cy + s * sx + c * sy)
    return (min(xs), min(ys), max(xs), max(ys))


def selector_weights_path(module_file=None):
    """Resolve ``place_path_selector.yaml``: src tree first, then the
    installed share directory.

    ``module_file`` defaults to this module's ``__file__``. With
    ``--symlink-install`` the module IS the src file, so the src-relative
    lookup hits. With a copy install the module lives in
    ``site-packages`` and the src-relative path does not exist — the yaml
    is installed at ``share/luggage_planning/config/``, so fall back to
    the ament share directory. Returns ``None`` when neither resolves.
    """
    here = os.path.dirname(os.path.abspath(
        module_file or __file__))
    src_path = os.path.normpath(os.path.join(
        here, "..", "config", "place_path_selector.yaml"))
    if os.path.isfile(src_path):
        return src_path
    try:
        from ament_index_python.packages import (
            get_package_share_directory)
        share_path = os.path.join(
            get_package_share_directory("luggage_planning"),
            "config", "place_path_selector.yaml")
    except Exception:  # noqa: BLE001 - package not installed / no ament
        return None
    if os.path.isfile(share_path):
        return share_path
    return None


def load_selector_weights(path=None):
    """Load selector YAML. Missing file falls back to ``DEFAULT_WEIGHTS``.

    The fallback is never silent: when no yaml can be resolved at all, a
    one-line warning goes to stderr (a copy install that silently ran on
    DEFAULT_WEIGHTS is how selector tuning "disappears").
    """
    weights = dict(DEFAULT_WEIGHTS)
    if path is None:
        path = selector_weights_path()
        if path is None:
            import sys
            sys.stderr.write(
                "luggage_planning: place_path_selector.yaml not found "
                "(src tree or installed share); using DEFAULT_WEIGHTS\n")
            return weights
    if not path or not os.path.isfile(path):
        return weights
    try:
        import yaml
    except ImportError:
        return weights
    with open(path, "r", encoding="utf-8") as handle:
        loaded = yaml.safe_load(handle) or {}
    if not isinstance(loaded, dict):
        return weights
    for key, default in DEFAULT_WEIGHTS.items():
        if key in loaded:
            weights[key] = type(default)(loaded[key])
    return weights


@dataclass
class SweepResult:
    collides: bool
    min_clearance: float
    first_hit_cell: tuple = None
    min_aperture: float = float("inf")
    samples: int = 0
    reason: str = ""


@dataclass
class PathVariant:
    method: str
    waypoints: list  # list of (x, y, z); last is the carry target


@dataclass
class OccupancySnapshot:
    geometry_hash: str
    map_revision: int
    resolution: float
    nx: int
    ny: int
    inner_size: list
    floor_z: float
    center_base: list
    yaw: float
    height: list
    state: list
    confidence: list
    obstacle: list = field(default_factory=list)
    column_top: list = field(default_factory=list)
    inflate_m: float = 0.05
    support_tol: float = 0.05
    hull: object = None

    @classmethod
    def from_surface_2d(cls, surface, expected_hash=None,
                        expected_revision=None, inflate_m=0.05,
                        support_tol=0.05):
        if not isinstance(surface, dict) or "height" not in surface:
            raise OccupancyMismatch(
                "%s: surface_2d missing height" % PLACE_PATH_INFEASIBLE)
        raw = surface.get("geometry_descriptor")
        if not isinstance(raw, dict) or not raw:
            raise OccupancyMismatch(
                "%s: surface_2d missing geometry_descriptor"
                % PLACE_PATH_INFEASIBLE)
        try:
            hull = normalize_descriptor(raw)
        except (TypeError, ValueError, KeyError) as exc:
            raise OccupancyMismatch(
                "%s: invalid geometry_descriptor: %s"
                % (PLACE_PATH_INFEASIBLE, exc))
        incoming_hash = str(surface.get("geometry_hash") or "")
        if incoming_hash and incoming_hash != hull.geometry_hash:
            raise OccupancyMismatch(
                "%s: geometry_hash %s != descriptor %s"
                % (PLACE_PATH_INFEASIBLE, incoming_hash, hull.geometry_hash))
        incoming_hash = hull.geometry_hash
        want_hash = "" if expected_hash is None else str(expected_hash)
        if want_hash and incoming_hash != want_hash:
            raise OccupancyMismatch(
                "%s: geometry_hash %s != %s"
                % (PLACE_PATH_INFEASIBLE, incoming_hash or "<missing>",
                   want_hash))
        incoming_rev = surface.get("map_revision")
        if expected_revision is not None:
            try:
                have = int(incoming_rev)
                want = int(expected_revision)
            except (TypeError, ValueError):
                raise OccupancyMismatch(
                    "%s: map_revision %s != %s"
                    % (PLACE_PATH_INFEASIBLE, incoming_rev, expected_revision))
            if have != want:
                raise OccupancyMismatch(
                    "%s: map_revision %s != %s"
                    % (PLACE_PATH_INFEASIBLE, have, want))
        nx = int(surface["nx"])
        ny = int(surface["ny"])
        inner = [float(v) for v in surface["inner_size"]]
        snapshot = cls(
            geometry_hash=incoming_hash,
            map_revision=int(incoming_rev or 0),
            resolution=float(surface["resolution"]),
            nx=nx,
            ny=ny,
            inner_size=inner,
            floor_z=float(surface.get("floor_z", 0.0)),
            center_base=[float(v) for v in surface["center_base"]],
            yaw=float(surface.get("yaw", 0.0)),
            height=surface["height"],
            state=surface["state"],
            confidence=surface.get("confidence") or [
                ["none"] * ny for _ in range(nx)],
            inflate_m=float(inflate_m),
            support_tol=float(support_tol),
            hull=hull,
        )
        snapshot._build_obstacles()
        return snapshot

    def _build_obstacles(self):
        nx, ny = self.nx, self.ny
        inner_h = float(self.inner_size[2])
        raw = [[False] * ny for _ in range(nx)]
        top = [[0.0] * ny for _ in range(nx)]
        for ix in range(nx):
            for iy in range(ny):
                state = str(self.state[ix][iy])
                peak = float(self.height[ix][iy])
                if state == "occupied":
                    raw[ix][iy] = True
                    top[ix][iy] = peak
                elif (state == "unknown"
                      and abs(peak - self.floor_z) > self.support_tol):
                    raw[ix][iy] = True
                    top[ix][iy] = inner_h
        radius = int(math.ceil(self.inflate_m / max(self.resolution, 1e-6)))
        if radius <= 0:
            self.obstacle = raw
            self.column_top = top
            return
        inflated = [[False] * ny for _ in range(nx)]
        inf_top = [[0.0] * ny for _ in range(nx)]
        for ix in range(nx):
            for iy in range(ny):
                peak = 0.0
                hit = False
                for jx in range(max(0, ix - radius), min(nx, ix + radius + 1)):
                    for jy in range(max(0, iy - radius), min(ny, iy + radius + 1)):
                        if raw[jx][jy]:
                            hit = True
                            if top[jx][jy] > peak:
                                peak = top[jx][jy]
                inflated[ix][iy] = hit
                inf_top[ix][iy] = peak
        self.obstacle = inflated
        self.column_top = inf_top

    def occupied_count(self):
        return sum(1 for row in self.obstacle for cell in row if cell)

    def local_to_map(self, lx, ly, lz):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return [
            self.center_base[0] + c * lx - s * ly,
            self.center_base[1] + s * lx + c * ly,
            self.center_base[2] + lz,
        ]

    def map_to_local(self, xyz):
        dx = float(xyz[0]) - self.center_base[0]
        dy = float(xyz[1]) - self.center_base[1]
        dz = float(xyz[2]) - self.center_base[2]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return [c * dx + s * dy, -s * dx + c * dy, dz]

    def xyz_to_cell(self, xyz):
        local = self.map_to_local(xyz)
        half_l = 0.5 * self.inner_size[0]
        half_w = 0.5 * self.inner_size[1]
        ix = int(math.floor((local[0] + half_l) / self.resolution))
        iy = int(math.floor((local[1] + half_w) / self.resolution))
        return ix, iy, local

    def cell_aabb_xy(self, ix, iy):
        half_l = 0.5 * self.inner_size[0]
        half_w = 0.5 * self.inner_size[1]
        xmin = -half_l + ix * self.resolution
        ymin = -half_w + iy * self.resolution
        return (xmin, ymin, xmin + self.resolution, ymin + self.resolution)

    def cell_center_map(self, ix, iy, local_z=0.0):
        half_l = 0.5 * self.inner_size[0]
        half_w = 0.5 * self.inner_size[1]
        lx = -half_l + (ix + 0.5) * self.resolution
        ly = -half_w + (iy + 0.5) * self.resolution
        return self.local_to_map(lx, ly, local_z)

    def floor_map_z(self):
        return self.center_base[2] - 0.5 * self.inner_size[2] + self.floor_z

    def column_top_map_z(self, ix, iy):
        return self.floor_map_z() + float(self.column_top[ix][iy])

    def cell_center_container(self, ix, iy, map_z=None):
        """Cell center in the map frame (Humble: ``container_link``)."""
        if map_z is None:
            map_z = self.floor_map_z()
        return self.cell_center_map(
            ix, iy, float(map_z) - self.center_base[2])

    def cell_inside_hull(self, ix, iy):
        if self.hull is None:
            return False
        return contains_point(self.hull, self.cell_center_container(ix, iy))


def exempt_footprint_locals(snapshot, waypoints, payload_wdh, yaw=0.0,
                            arm_radius=None, inflate_m=None, margin_m=0.0):
    """Landing-footprint exemption AABBs (grid-local) for in-hull waypoints.

    Each waypoint whose payload center sits inside the seven-face hull
    (aperture open) anchors one exemption: the yaw-rotated payload footprint
    expanded by ``arm_radius + inflate + margin`` — exactly the artifacts
    ``sweep_polyline`` padding and ``occupancy_collision_boxes`` inflation
    would otherwise add around a landing slot. The payload is destined to
    occupy that column (support top, inflated neighbours), so cells there
    are expected, not obstacles. Waypoints outside the hull (portal,
    staging) anchor nothing, so carry paths keep today's behavior.
    """
    if not waypoints:
        return []
    width, depth, height = [float(v) for v in payload_wdh]
    if width <= 0.0 or depth <= 0.0 or height <= 0.0:
        return []
    pad = (_weight("arm_radius_m", arm_radius)
           + (snapshot.inflate_m if inflate_m is None else float(inflate_m))
           + max(0.0, float(margin_m)))
    out = []
    seen = set()
    for wp in waypoints:
        x, y, z = float(wp[0]), float(wp[1]), float(wp[2])
        if snapshot.hull is not None and not contains_point_through_aperture(
                snapshot.hull, [x, y, z - 0.5 * height]):
            continue
        footprint = _rotated_footprint(
            x, y, width + 2.0 * pad, depth + 2.0 * pad, yaw)
        a = snapshot.map_to_local([footprint[0], footprint[1], z])
        b = snapshot.map_to_local([footprint[2], footprint[3], z])
        aabb = (min(a[0], b[0]), min(a[1], b[1]),
                max(a[0], b[0]), max(a[1], b[1]))
        key = tuple(round(v, 3) for v in aabb)
        if key in seen:
            continue
        seen.add(key)
        out.append(aabb)
    return out


def _exempt_cells(snapshot, exempt_locals):
    """Grid cells whose AABB touches any exemption footprint."""
    cells = set()
    for aabb in (exempt_locals or []):
        half_l = 0.5 * snapshot.inner_size[0]
        half_w = 0.5 * snapshot.inner_size[1]
        ix0 = int(math.floor((aabb[0] + half_l) / snapshot.resolution))
        iy0 = int(math.floor((aabb[1] + half_w) / snapshot.resolution))
        ix1 = int(math.floor((aabb[2] + half_l) / snapshot.resolution))
        iy1 = int(math.floor((aabb[3] + half_w) / snapshot.resolution))
        for ix in range(max(0, ix0), min(snapshot.nx, ix1 + 1)):
            for iy in range(max(0, iy0), min(snapshot.ny, iy1 + 1)):
                if _aabb_distance(aabb, snapshot.cell_aabb_xy(ix, iy)) <= 1e-6:
                    cells.add((ix, iy))
    return cells


def occupancy_collision_boxes(snapshot, max_objects=200, frame="container_link",
                              exempt=None):
    """Merged occupied-column boxes in the map (``center_base``) frame.

    ``exempt`` is a list of grid-local exemption AABBs
    (``exempt_footprint_locals``): cells touching them produce no box, so a
    landing slot's support and its inflated neighbours do not block the
    insertion they exist for.
    """
    del frame
    boxes = []
    nx, ny = snapshot.nx, snapshot.ny
    res = snapshot.resolution
    half_h = 0.5 * snapshot.inner_size[2]
    exempt_cells = _exempt_cells(snapshot, exempt)
    used = [[False] * ny for _ in range(nx)]
    for ix in range(nx):
        iy = 0
        while iy < ny:
            if (not snapshot.obstacle[ix][iy]) or used[ix][iy]:
                iy += 1
                continue
            if (ix, iy) in exempt_cells:
                used[ix][iy] = True
                iy += 1
                continue
            if not snapshot.cell_inside_hull(ix, iy):
                used[ix][iy] = True
                iy += 1
                continue
            peak = snapshot.column_top[ix][iy]
            run = 1
            while (iy + run < ny
                   and snapshot.obstacle[ix][iy + run]
                   and not used[ix][iy + run]
                   and abs(snapshot.column_top[ix][iy + run] - peak) < res):
                run += 1
            for k in range(run):
                used[ix][iy + k] = True
            height = max(peak, res)
            lx_span = res
            ly_span = run * res
            half_l = 0.5 * snapshot.inner_size[0]
            half_w = 0.5 * snapshot.inner_size[1]
            lx = -half_l + (ix + 0.5) * res
            ly = -half_w + (iy + 0.5 * run) * res
            lz = -half_h + 0.5 * height
            xyz = snapshot.local_to_map(lx, ly, lz)
            boxes.append({
                "id": "cargo_occ_%d_%d" % (ix, iy),
                "xyz": xyz,
                "size": [lx_span + 2.0 * snapshot.inflate_m,
                         ly_span + 2.0 * snapshot.inflate_m,
                         height],
                "quat": (0.0, 0.0, 0.0, 1.0),
            })
            if len(boxes) >= int(max_objects):
                return boxes
            iy += run
    return boxes


def _weight(name, override=None):
    if override is not None:
        return type(DEFAULT_WEIGHTS[name])(override)
    return DEFAULT_WEIGHTS[name]


def inner_ceiling_map_z(snapshot):
    """Inner lid Z in the map frame (floor + usable height)."""
    return snapshot.floor_map_z() + float(snapshot.inner_size[2])


def suction_ceiling_z(snapshot, arm_overhead=None, collision_pad=None):
    """Max suction Z so payload top and wrist/camera stay under the lid.

    Waypoints are suction (box top). Payload occupies ``[z - h, z]``. The
    wrist/camera stack occupies ``[z, z + arm_overhead]``. Both must stay
    below the inner ceiling minus ``collision_pad``.
    """
    overhead = _weight("arm_overhead_m", arm_overhead)
    pad = _weight("collision_pad_m", collision_pad)
    return inner_ceiling_map_z(snapshot) - float(overhead) - float(pad)


def required_carry_suction_z(surface_max, payload_height, margin=None,
                             collision_pad=None):
    """Suction Z that keeps the hanging payload off occupied tops.

    ``payload_bottom = suction - height``, so
    ``suction = surface_max + height + gap`` with
    ``gap = max(carry_margin, collision_pad)``.
    """
    gap = max(
        _weight("carry_margin_m", margin),
        _weight("collision_pad_m", collision_pad))
    return float(surface_max) + max(0.0, float(payload_height)) + float(gap)


def _footprint_cells(snapshot, sample, width, depth, yaw, arm_radius):
    """Yield ``(ix, iy, local_fp)`` for cells under the payload XY AABB."""
    pad = float(arm_radius)
    footprint = _rotated_footprint(
        sample[0], sample[1], width + 2.0 * pad, depth + 2.0 * pad, yaw)
    local_min = snapshot.map_to_local(
        [footprint[0], footprint[1], sample[2]])
    local_max = snapshot.map_to_local(
        [footprint[2], footprint[3], sample[2]])
    half_l = 0.5 * snapshot.inner_size[0]
    half_w = 0.5 * snapshot.inner_size[1]
    ix0 = int(math.floor((min(local_min[0], local_max[0]) + half_l)
                         / snapshot.resolution)) - 1
    iy0 = int(math.floor((min(local_min[1], local_max[1]) + half_w)
                         / snapshot.resolution)) - 1
    ix1 = int(math.floor((max(local_min[0], local_max[0]) + half_l)
                         / snapshot.resolution)) + 1
    iy1 = int(math.floor((max(local_min[1], local_max[1]) + half_w)
                         / snapshot.resolution)) + 1
    local_fp = (
        min(local_min[0], local_max[0]),
        min(local_min[1], local_max[1]),
        max(local_min[0], local_max[0]),
        max(local_min[1], local_max[1]),
    )
    for ix in range(max(0, ix0), min(snapshot.nx, ix1 + 1)):
        for iy in range(max(0, iy0), min(snapshot.ny, iy1 + 1)):
            yield ix, iy, local_fp


def occupied_surface_max_along(snapshot, points, payload_wdh, yaw=0.0,
                               arm_radius=None):
    """Highest occupied column top under the swept payload footprint.

    Defaults to the map-frame floor when no occupied cell overlaps. A wide
    payload at the goal must see a neighbor beside the slot, not just the
    center-line cells.
    """
    peak = snapshot.floor_map_z()
    if not points:
        return peak
    radius = _weight("arm_radius_m", arm_radius)
    width, depth, _height = [float(v) for v in payload_wdh]
    samples = _sample_polyline(points, snapshot.resolution)
    use_footprint = width > 1e-6 and depth > 1e-6
    for sample in samples:
        if use_footprint:
            cells = _footprint_cells(
                snapshot, sample, width, depth, yaw, radius)
        else:
            ix, iy, _local = snapshot.xyz_to_cell(sample)
            if 0 <= ix < snapshot.nx and 0 <= iy < snapshot.ny:
                cells = [(ix, iy, None)]
            else:
                cells = []
        for ix, iy, local_fp in cells:
            if not snapshot.obstacle[ix][iy]:
                continue
            if local_fp is not None:
                dist = _aabb_distance(local_fp, snapshot.cell_aabb_xy(ix, iy))
                if dist > 1e-6:
                    continue
            peak = max(peak, snapshot.column_top_map_z(ix, iy))
    return peak


def _sample_polyline(points, step):
    if not points:
        return []
    samples = [tuple(float(v) for v in points[0])]
    for i in range(1, len(points)):
        a = points[i - 1]
        b = points[i]
        dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        n = max(1, int(math.ceil(length / max(step, 1e-6))))
        for k in range(1, n + 1):
            t = float(k) / float(n)
            samples.append((a[0] + t * dx, a[1] + t * dy, a[2] + t * dz))
    return samples


def sweep_polyline(snapshot, points, payload_wdh, yaw=0.0, arm_radius=0.08,
                   collision_pad=None, exempt=None):
    """Sweep the hanging payload AABB (+ arm radius) along ``points``.

    ``points`` are in the map frame (Humble cargo map: ``container_link``).
    Payload hangs below each point (tool-down suction frame = box top).
    Collision is infeasible, not a cost. A gap smaller than ``collision_pad``
    is a collision (MoveIt FCL padding). The oriented payload must also stay
    inside the seven-face hull; leaving it is ``outside_hull``, not cargo
    occupancy. The -X door is exempt: an insertion path enters through it, so
    the payload straddles that face by construction and only the closed faces
    can reject it.

    ``exempt`` is a list of grid-local landing-footprint AABBs
    (``exempt_footprint_locals``): cargo cells inside them do not collide —
    the payload is entering its own landing column. Hull containment still
    applies to every sample.
    """
    width, depth, height = [float(v) for v in payload_wdh]
    if width <= 0.0 or depth <= 0.0 or height <= 0.0:
        return SweepResult(True, 0.0, reason="invalid_payload")
    samples = _sample_polyline(points, snapshot.resolution)
    min_clearance = float("inf")
    first_hit = None
    floor_z = snapshot.floor_map_z()
    pad_z = _weight("collision_pad_m", collision_pad)
    hull = snapshot.hull
    exempt_cells = _exempt_cells(snapshot, exempt)
    for sample in samples:
        if hull is not None:
            center = [sample[0], sample[1], sample[2] - 0.5 * height]
            if not contains_oriented_box_through_aperture(
                    hull, center, [width, depth, height], yaw=yaw):
                return SweepResult(
                    True, 0.0, None, 0.0, len(samples),
                    reason="outside_hull")
        payload_bottom = sample[2] - height
        payload_top = sample[2]
        for ix, iy, local_fp in _footprint_cells(
                snapshot, sample, width, depth, yaw, arm_radius):
            if (ix, iy) in exempt_cells:
                continue
            if not snapshot.obstacle[ix][iy]:
                continue
            col_top = snapshot.column_top_map_z(ix, iy)
            if payload_bottom >= col_top + pad_z - 1e-6:
                continue
            if payload_top <= floor_z + 1e-6:
                continue
            dist = _aabb_distance(local_fp, snapshot.cell_aabb_xy(ix, iy))
            if dist < min_clearance:
                min_clearance = dist
            if dist <= 1e-6:
                return SweepResult(
                    True, 0.0, (ix, iy), 0.0, len(samples),
                    reason="occupancy_collision")
    if min_clearance is float("inf"):
        min_clearance = float(DEFAULT_WEIGHTS["safety_ref_m"])
    return SweepResult(
        False, float(min_clearance), first_hit, float(min_clearance),
        len(samples), reason="")


def _line_cells(snapshot, a, b):
    cells = []
    samples = _sample_polyline([a, b], snapshot.resolution)
    seen = set()
    for sample in samples:
        ix, iy, _local = snapshot.xyz_to_cell(sample)
        if 0 <= ix < snapshot.nx and 0 <= iy < snapshot.ny:
            key = (ix, iy)
            if key not in seen:
                seen.add(key)
                cells.append(key)
    return cells


def generate_path_variants(portal, target, snapshot=None, opening_tangent=None,
                           inner_h=None, max_candidates=4, payload_height=0.0,
                           payload_wdh=None, yaw=0.0, arm_radius=None,
                           carry_margin=None, collision_pad=None,
                           arm_overhead=None):
    """Return at least two geometric variants whenever possible.

    ``portal`` / ``target`` are map-frame XYZ (suction frame). ``target`` is
    the above-slot pose (typically the contact / box-top). Carry Z is not a
    fixed delta: it is ``occupied_top + payload_height + margin``, clamped so
    the wrist/camera stack stays under the inner lid. Offset/via keep the
    same destination XY and use that carry Z.
    """
    del inner_h  # Ceiling comes from snapshot inner height, not this alias.
    portal = tuple(float(v) for v in portal)
    target = tuple(float(v) for v in target)
    cap = max(2, int(max_candidates or 4))
    dx, dy = target[0] - portal[0], target[1] - portal[1]
    along = _unit_xy(dx, dy)
    if opening_tangent is not None:
        tangent = _unit_xy(float(opening_tangent[0]), float(opening_tangent[1]))
    else:
        tangent = (-along[1], along[0])
    height = float(payload_height)
    if payload_wdh is not None:
        wdh = [float(v) for v in payload_wdh]
        height = float(wdh[2])
    else:
        wdh = [0.0, 0.0, height]
    radius = _weight("arm_radius_m", arm_radius)

    carry_z = target[2]
    suction_max = None
    if snapshot is not None:
        surface_max = occupied_surface_max_along(
            snapshot, [portal, target], wdh, yaw=yaw, arm_radius=radius)
        need = required_carry_suction_z(
            surface_max, height, margin=carry_margin,
            collision_pad=collision_pad)
        suction_max = suction_ceiling_z(
            snapshot, arm_overhead=arm_overhead, collision_pad=collision_pad)
        carry_z = min(max(target[2], need), suction_max)

    dest = (target[0], target[1], carry_z)
    entry = (portal[0], portal[1], max(portal[2], dest[2]))

    variants = [PathVariant("direct", [portal, target])]
    if (abs(dest[2] - target[2]) > 1e-4 or abs(entry[2] - portal[2]) > 1e-4):
        variants.append(PathVariant("clear_top", [entry, dest]))
    if (suction_max is not None
            and (suction_max - dest[2]) >= _weight("carry_margin_m",
                                                   carry_margin)):
        headroom = 0.5 * (dest[2] + suction_max)
        hi_entry = (entry[0], entry[1], max(entry[2], headroom))
        hi_dest = (dest[0], dest[1], max(dest[2], headroom))
        if abs(hi_dest[2] - dest[2]) > 1e-4:
            variants.append(PathVariant("clear_headroom", [hi_entry, hi_dest]))

    mid = (
        0.5 * (portal[0] + dest[0]),
        0.5 * (portal[1] + dest[1]),
        max(entry[2], dest[2]),
    )
    for delta in OFFSET_DELTAS:
        via = (mid[0] + tangent[0] * delta, mid[1] + tangent[1] * delta, mid[2])
        variants.append(PathVariant(
            "offset_%+.2f" % delta, [entry, via, dest]))

    if snapshot is not None:
        peak_cell = None
        peak_h = -1.0
        for ix, iy in _line_cells(snapshot, portal, target):
            if snapshot.obstacle[ix][iy] and snapshot.column_top[ix][iy] > peak_h:
                peak_h = snapshot.column_top[ix][iy]
                peak_cell = (ix, iy)
        if peak_cell is not None:
            center = snapshot.cell_center_map(
                peak_cell[0], peak_cell[1],
                snapshot.map_to_local(mid)[2])
            shift = max(0.15, 2.0 * snapshot.resolution + snapshot.inflate_m)
            via_z = max(
                entry[2], dest[2],
                snapshot.column_top_map_z(*peak_cell) + _weight(
                    "carry_margin_m", carry_margin) + height)
            if suction_max is not None:
                via_z = min(via_z, suction_max)
            via = (
                center[0] + tangent[0] * shift,
                center[1] + tangent[1] * shift,
                via_z,
            )
            variants.append(PathVariant("via_clear", [entry, via, dest]))

    chosen = []
    seen_family = set()
    for variant in variants:
        family = variant.method.split("_")[0]
        if family in seen_family and len(chosen) >= 1:
            # Fill remaining slots after one of each family.
            if len(chosen) >= cap:
                break
            if family in seen_family and len(chosen) >= min(4, cap):
                continue
        chosen.append(variant)
        seen_family.add(family)
        if len(chosen) >= cap:
            break
    if len(chosen) < min(2, len(variants)):
        for variant in variants:
            if variant not in chosen:
                chosen.append(variant)
            if len(chosen) >= 2:
                break
    return chosen[:cap]


def path_length(waypoints):
    total = 0.0
    for i in range(1, len(waypoints)):
        a, b = waypoints[i - 1], waypoints[i]
        total += math.sqrt(
            (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)
    return total


def _dimension_cost(min_clearance, safety_ref):
    if min_clearance <= 0.0:
        return 1.0
    return 1.0 - clip01(min_clearance / max(safety_ref, 1e-6))


def _safety_cost(min_clearance, safety_ref):
    return 1.0 - clip01(min_clearance / max(safety_ref, 1e-6))


def _efficiency_cost(waypoints, cartesian_fraction):
    if len(waypoints) < 2:
        straight = 1e-6
        length = 0.0
    else:
        straight = max(1e-6, path_length([waypoints[0], waypoints[-1]]))
        length = path_length(waypoints)
    detour = max(0.0, length / straight - 1.0)
    frac = 0.0 if cartesian_fraction is None else float(cartesian_fraction)
    frac = clip01(frac)
    return 0.5 * detour + 0.5 * (1.0 - frac)


def select_trajectory(rows, weights=None):
    """Pick the minimum-cost feasible row.

    Each row is a dict that already has sweep/probe fields. This function
    writes ``C_place``, ``C_dim``, ``C_safe``, ``C_eff``, ``cost``,
    ``selected``. Returns ``(winner_or_None, rows, reason)``.
    """
    cfg = dict(DEFAULT_WEIGHTS)
    if weights:
        cfg.update(weights)
    feasible_scores = [
        float(row.get("slot_score", 0.0))
        for row in rows if row.get("feasible")]
    score_min = min(feasible_scores) if feasible_scores else 0.0
    score_max = max(feasible_scores) if feasible_scores else 1.0
    span = max(1e-6, score_max - score_min)
    safety_ref = float(cfg["safety_ref_m"])
    winner = None
    winner_cost = float("inf")
    for row in rows:
        row["selected"] = False
        if not row.get("feasible"):
            row["C_place"] = None
            row["C_dim"] = None
            row["C_safe"] = None
            row["C_eff"] = None
            row["cost"] = None
            continue
        if score_max == score_min:
            c_place = 0.0
        else:
            c_place = 1.0 - (float(row.get("slot_score", 0.0)) - score_min) / span
        c_dim = _dimension_cost(float(row.get("min_clearance", 0.0)), safety_ref)
        c_safe = _safety_cost(float(row.get("min_clearance", 0.0)), safety_ref)
        c_eff = _efficiency_cost(
            row.get("waypoints") or [], row.get("cartesian_fraction"))
        cost = (
            float(cfg["w_placement"]) * c_place
            + float(cfg["w_dimension"]) * c_dim
            + float(cfg["w_safety"]) * c_safe
            + float(cfg["w_efficiency"]) * c_eff)
        row["C_place"] = round(c_place, 4)
        row["C_dim"] = round(c_dim, 4)
        row["C_safe"] = round(c_safe, 4)
        row["C_eff"] = round(c_eff, 4)
        row["cost"] = round(cost, 4)
        if cost < winner_cost:
            winner_cost = cost
            winner = row
    if winner is None:
        return None, rows, PLACE_PATH_INFEASIBLE
    winner["selected"] = True
    return winner, rows, ""


def dump_rows(rows):
    """JSON-friendly selector dump (no numpy)."""
    out = []
    for row in rows:
        item = dict(row)
        waypoints = item.get("waypoints") or []
        item["waypoints"] = [
            [round(float(v), 4) for v in wp] for wp in waypoints]
        hit = item.get("first_hit_cell")
        if hit is not None:
            item["first_hit_cell"] = [int(hit[0]), int(hit[1])]
        out.append(item)
    return out
