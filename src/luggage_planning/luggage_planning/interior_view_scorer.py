#!/usr/bin/env python3
"""Deterministic, ROS-free scoring for interior camera candidates.

Occupancy accessors are duck typed.  They must provide ``world_to_index(point)``
and ``occupancy(index)``.  They may provide ``contains_index(index)``,
``cell_center(index)``, ``normal(index)``, and an exact
``ray_indices(origin, direction, max_range)`` iterator.  The included
``SparseOccupancyGrid`` implements the complete contract.
"""

from __future__ import division

import json
import math


FREE = "free"
UNKNOWN = "unknown"
OCCUPIED = "occupied"
_VALID_STATES = frozenset((FREE, UNKNOWN, OCCUPIED))
_EPSILON = 1.0e-12


def _finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("%s must be finite" % name)
    return value


def _nonnegative(value, name):
    value = _finite(value, name)
    if value < 0.0:
        raise ValueError("%s must be nonnegative" % name)
    return value


def _clamp01(value):
    return max(0.0, min(1.0, float(value)))


def _vector3(value, name):
    if value is None or len(value) != 3:
        raise ValueError("%s must contain three values" % name)
    return tuple(_finite(item, name) for item in value)


def _add(a, b):
    return tuple(a[i] + b[i] for i in range(3))


def _subtract(a, b):
    return tuple(a[i] - b[i] for i in range(3))


def _scale(vector, scalar):
    return tuple(item * scalar for item in vector)


def _dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(vector):
    return math.sqrt(_dot(vector, vector))


def _unit(vector, name):
    length = _norm(vector)
    if length <= _EPSILON:
        raise ValueError("%s must be nonzero" % name)
    return _scale(vector, 1.0 / length)


class CameraIntrinsics(object):
    """Pinhole intrinsics with integer image dimensions."""

    def __init__(self, width, height, fx, fy, cx=None, cy=None):
        self.width = int(width)
        self.height = int(height)
        if self.width <= 0 or self.height <= 0:
            raise ValueError("image dimensions must be positive")
        self.fx = _finite(fx, "fx")
        self.fy = _finite(fy, "fy")
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("focal lengths must be positive")
        self.cx = _finite(
            (self.width - 1) * 0.5 if cx is None else cx, "cx")
        self.cy = _finite(
            (self.height - 1) * 0.5 if cy is None else cy, "cy")


class RaycastConfig(object):
    """Controls deterministic pixel sampling and visibility discounts."""

    def __init__(
            self, max_range=2.0, pixel_stride=1, range_decay=0.0,
            grazing_power=1.0, fallback_step=None):
        self.max_range = _nonnegative(max_range, "max_range")
        self.pixel_stride = int(pixel_stride)
        if self.pixel_stride <= 0:
            raise ValueError("pixel_stride must be positive")
        self.range_decay = _nonnegative(range_decay, "range_decay")
        self.grazing_power = _nonnegative(grazing_power, "grazing_power")
        self.fallback_step = (
            None if fallback_step is None
            else _nonnegative(fallback_step, "fallback_step"))
        if self.fallback_step == 0.0:
            raise ValueError("fallback_step must be positive")


DEFAULT_WEIGHTS = {
    "information_gain": 1.0,
    "corridor_confidence": 0.25,
    "depth": 0.15,
    "manipulability": 0.20,
    "joint_margin": 0.20,
    "trajectory": 0.15,
    "risk": 0.50,
}


class SparseOccupancyGrid(object):
    """Small deterministic axis-aligned occupancy grid.

    ``cells`` maps ``(ix, iy, iz)`` to ``FREE``, ``UNKNOWN``, or
    ``OCCUPIED``.  Missing in-bounds cells use ``default_state``.
    """

    def __init__(
            self, origin, shape, resolution, cells=None, normals=None,
            default_state=UNKNOWN):
        self.origin = _vector3(origin, "origin")
        self.shape = tuple(int(item) for item in shape)
        if len(self.shape) != 3 or any(item <= 0 for item in self.shape):
            raise ValueError("shape must contain three positive integers")
        self.resolution = _finite(resolution, "resolution")
        if self.resolution <= 0.0:
            raise ValueError("resolution must be positive")
        if default_state not in _VALID_STATES:
            raise ValueError("invalid default occupancy state")
        self.default_state = default_state
        self._cells = {}
        for index, state in (cells or {}).items():
            index = self._index(index)
            if not self.contains_index(index):
                raise ValueError("cell index outside grid: %r" % (index,))
            if state not in _VALID_STATES:
                raise ValueError("invalid occupancy state: %r" % (state,))
            self._cells[index] = state
        self._normals = {}
        for index, normal in (normals or {}).items():
            index = self._index(index)
            if not self.contains_index(index):
                raise ValueError("normal index outside grid: %r" % (index,))
            self._normals[index] = _unit(
                _vector3(normal, "normal"), "normal")

    @staticmethod
    def _index(index):
        if index is None or len(index) != 3:
            raise ValueError("grid index must contain three values")
        return tuple(int(item) for item in index)

    def contains_index(self, index):
        return all(0 <= index[axis] < self.shape[axis] for axis in range(3))

    def world_to_index(self, point):
        point = _vector3(point, "point")
        return tuple(
            int(math.floor((point[axis] - self.origin[axis])
                           / self.resolution))
            for axis in range(3)
        )

    def cell_center(self, index):
        index = self._index(index)
        return tuple(
            self.origin[axis] + (index[axis] + 0.5) * self.resolution
            for axis in range(3)
        )

    def occupancy(self, index):
        index = self._index(index)
        if not self.contains_index(index):
            return None
        return self._cells.get(index, self.default_state)

    def normal(self, index):
        return self._normals.get(self._index(index))

    def ray_indices(self, origin, direction, max_range):
        """Yield intersected cells in near-to-far order using voxel DDA."""
        point = _vector3(origin, "ray origin")
        direction = _unit(_vector3(direction, "ray direction"), "ray direction")
        max_range = _nonnegative(max_range, "max_range")
        index = self.world_to_index(point)
        step = tuple(
            1 if component > _EPSILON
            else (-1 if component < -_EPSILON else 0)
            for component in direction
        )
        t_max = []
        t_delta = []
        for axis in range(3):
            if step[axis] == 0:
                t_max.append(float("inf"))
                t_delta.append(float("inf"))
                continue
            boundary_index = index[axis] + (1 if step[axis] > 0 else 0)
            boundary = self.origin[axis] + boundary_index * self.resolution
            t_max.append((boundary - point[axis]) / direction[axis])
            t_delta.append(self.resolution / abs(direction[axis]))

        distance = 0.0
        while distance <= max_range + _EPSILON:
            yield index, max(0.0, distance)
            next_distance = min(t_max)
            if next_distance > max_range + _EPSILON:
                break
            # Advance every tied axis so corner/edge crossings are unambiguous.
            for axis in range(3):
                if abs(t_max[axis] - next_distance) <= _EPSILON:
                    index = tuple(
                        index[item] + (step[item] if item == axis else 0)
                        for item in range(3)
                    )
                    t_max[axis] += t_delta[axis]
            distance = next_distance


def _camera_rays(candidate, intrinsics, pixel_stride):
    origin = _vector3(candidate.get("camera_xyz"), "camera_xyz")
    look_at = _vector3(candidate.get("look_at"), "look_at")
    forward = _unit(_subtract(look_at, origin), "camera view direction")
    up_hint = _unit(
        _vector3(candidate.get("camera_up", (0.0, 0.0, 1.0)), "camera_up"),
        "camera_up")
    right_raw = _cross(forward, up_hint)
    if _norm(right_raw) <= _EPSILON:
        alternate = (0.0, 1.0, 0.0)
        if abs(_dot(forward, alternate)) > 0.99:
            alternate = (1.0, 0.0, 0.0)
        right_raw = _cross(forward, alternate)
    right = _unit(right_raw, "camera right axis")
    up = _unit(_cross(right, forward), "camera up axis")

    for v in range(0, intrinsics.height, pixel_stride):
        for u in range(0, intrinsics.width, pixel_stride):
            image_x = (float(u) - intrinsics.cx) / intrinsics.fx
            image_y = (float(v) - intrinsics.cy) / intrinsics.fy
            direction = _add(
                forward,
                _add(_scale(right, image_x), _scale(up, -image_y)),
            )
            yield u, v, origin, _unit(direction, "pixel ray")


def _contains(accessor, index):
    method = getattr(accessor, "contains_index", None)
    if method is not None:
        return bool(method(index))
    return accessor.occupancy(index) is not None


def _center(accessor, index, fallback_point):
    method = getattr(accessor, "cell_center", None)
    return fallback_point if method is None else _vector3(
        method(index), "cell center")


def _normal(accessor, index):
    method = getattr(accessor, "normal", None)
    if method is None:
        return None
    value = method(index)
    if value is None:
        return None
    return _unit(_vector3(value, "normal"), "normal")


def _fallback_ray_indices(accessor, origin, direction, config):
    step = config.fallback_step
    if step is None:
        step = getattr(accessor, "resolution", None)
    if step is None:
        raise ValueError(
            "accessor must provide ray_indices, resolution, or fallback_step")
    step = _nonnegative(step, "ray step") * 0.25
    if step <= 0.0:
        raise ValueError("ray step must be positive")
    previous = None
    distance = 0.0
    while distance <= config.max_range + _EPSILON:
        point = _add(origin, _scale(direction, distance))
        index = tuple(accessor.world_to_index(point))
        if index != previous:
            yield index, distance
            previous = index
        distance += step


def raycast_information_gain(candidate, occupancy, intrinsics, config=None):
    """Return visible unknown-voxel gain and deterministic diagnostics.

    Occupied cells terminate their ray.  Unknown cells contribute at most once
    over the complete frustum and do not stop a ray.  A known surface normal
    applies an incidence factor ``abs(normal dot direction_to_camera)`` raised
    to ``grazing_power``.  Range discount is ``exp(-distance/range_decay)``;
    a zero decay disables that discount.
    """
    config = config or RaycastConfig()
    seen_unknown = set()
    traversed = set()
    gain = 0.0
    raw_unknown = 0
    occluded_rays = 0
    rays_cast = 0
    normal_discounted = 0

    for _u, _v, origin, direction in _camera_rays(
            candidate, intrinsics, config.pixel_stride):
        rays_cast += 1
        iterator_method = getattr(occupancy, "ray_indices", None)
        if iterator_method is None:
            iterator = _fallback_ray_indices(
                occupancy, origin, direction, config)
        else:
            iterator = iterator_method(origin, direction, config.max_range)

        for index, entry_distance in iterator:
            index = tuple(int(item) for item in index)
            if not _contains(occupancy, index):
                continue
            traversed.add(index)
            state = occupancy.occupancy(index)
            if state not in _VALID_STATES:
                raise ValueError(
                    "occupancy accessor returned invalid state %r" % state)
            if state == OCCUPIED:
                occluded_rays += 1
                break
            if state != UNKNOWN or index in seen_unknown:
                continue

            fallback_point = _add(origin, _scale(direction, entry_distance))
            center = _center(occupancy, index, fallback_point)
            distance = _norm(_subtract(center, origin))
            if distance > config.max_range + _EPSILON:
                continue
            range_factor = (
                1.0 if config.range_decay <= 0.0
                else math.exp(-distance / config.range_decay))
            surface_normal = _normal(occupancy, index)
            incidence = 1.0
            if surface_normal is not None:
                incidence = abs(_dot(surface_normal, _scale(direction, -1.0)))
                incidence = max(0.0, min(1.0, incidence))
                incidence = incidence ** config.grazing_power
                if incidence < 1.0 - _EPSILON:
                    normal_discounted += 1
            gain += range_factor * incidence
            raw_unknown += 1
            seen_unknown.add(index)

    return {
        "information_gain": gain,
        "rays_cast": rays_cast,
        "traversed_voxels": len(traversed),
        "visible_unknown_voxels": raw_unknown,
        "occluded_rays": occluded_rays,
        "normal_discounted_voxels": normal_discounted,
    }


class ContainerFloor(object):
    """Fixed container inner-floor plane discretized into XY cells.

    Cell indexing matches ``CargoVolumeMapper._local_to_voxel`` when both are
    built from the same container geometry, so ``(ix, iy)`` here is the same
    column as ``(ix, iy, iz)`` in the occupancy grid.  The plane is the inner
    floor derived from ``inner_size`` and does not move with cargo stacking.
    """

    def __init__(self, center_base, yaw, inner_size, resolution):
        self.center = _vector3(center_base, "center_base")
        self.yaw = _finite(yaw, "yaw")
        inner = _vector3(inner_size, "inner_size")
        if any(value <= 0.0 for value in inner):
            raise ValueError("inner_size must be positive")
        self.inner_l, self.inner_w, self.inner_h = inner
        self.resolution = _finite(resolution, "resolution")
        if self.resolution <= 0.0:
            raise ValueError("resolution must be positive")
        self.nx = max(1, int(math.ceil(self.inner_l / self.resolution)))
        self.ny = max(1, int(math.ceil(self.inner_w / self.resolution)))
        self.plane_z = self.center[2] - self.inner_h * 0.5

    @property
    def cell_count(self):
        return self.nx * self.ny

    def world_to_cell(self, point):
        """Return the ``(ix, iy)`` floor cell or None when outside the box."""
        delta_x = point[0] - self.center[0]
        delta_y = point[1] - self.center[1]
        cos_yaw = math.cos(-self.yaw)
        sin_yaw = math.sin(-self.yaw)
        local_x = cos_yaw * delta_x - sin_yaw * delta_y
        local_y = sin_yaw * delta_x + cos_yaw * delta_y
        half_l = self.inner_l * 0.5
        half_w = self.inner_w * 0.5
        if abs(local_x) > half_l or abs(local_y) > half_w:
            return None
        ix = int((local_x + half_l) / self.resolution)
        iy = int((local_y + half_w) / self.resolution)
        return (
            min(max(ix, 0), self.nx - 1),
            min(max(iy, 0), self.ny - 1),
        )

    def cell_center_base(self, ix, iy, height=0.0):
        """Return the base-frame center of a floor cell, offset upward."""
        half_l = self.inner_l * 0.5
        half_w = self.inner_w * 0.5
        local_x = -half_l + (int(ix) + 0.5) * self.resolution
        local_y = -half_w + (int(iy) + 0.5) * self.resolution
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        return (
            self.center[0] + cos_yaw * local_x - sin_yaw * local_y,
            self.center[1] + sin_yaw * local_x + cos_yaw * local_y,
            self.plane_z + _finite(height, "height"),
        )


def _floor_hit_distance(origin, direction, plane_z):
    """Distance along a ray to a horizontal plane, or None when it never hits."""
    if abs(direction[2]) <= _EPSILON:
        return None
    distance = (plane_z - origin[2]) / direction[2]
    return None if distance < 0.0 else distance


def _column_has_unknown(occupancy, floor, cell):
    """True when the occupancy column above a floor cell still holds unknown."""
    point = floor.cell_center_base(
        cell[0], cell[1], height=floor.resolution * 0.5)
    base_index = occupancy.world_to_index(point)
    if base_index is None:
        return False
    index = tuple(int(item) for item in base_index)
    while _contains(occupancy, index):
        if occupancy.occupancy(index) == UNKNOWN:
            return True
        index = (index[0], index[1], index[2] + 1)
    return False


def floor_coverage_metrics(
        candidate, occupancy, intrinsics, floor, config=None):
    """Ray-vs-inner-floor coverage for one candidate camera pose.

    A floor cell counts as covered when a pixel ray reaches its column without
    being terminated by an occupied voxel belonging to a *different* column.
    Cargo standing inside the cell does not hide it: observing the top of that
    stack still observes the column, which is what placement planning needs.
    """
    config = config or RaycastConfig()
    covered = set()
    blocked = set()
    rays_cast = 0
    rays_inside = 0
    rays_hit_floor = 0
    rays_blocked = 0

    for _u, _v, origin, direction in _camera_rays(
            candidate, intrinsics, config.pixel_stride):
        rays_cast += 1
        iterator_method = getattr(occupancy, "ray_indices", None)
        if iterator_method is None:
            iterator = _fallback_ray_indices(
                occupancy, origin, direction, config)
        else:
            iterator = iterator_method(origin, direction, config.max_range)

        inside = False
        blocking_column = None
        blocking_distance = None
        for index, entry_distance in iterator:
            if entry_distance > config.max_range + _EPSILON:
                break
            index = tuple(int(item) for item in index)
            if not _contains(occupancy, index):
                continue
            inside = True
            state = occupancy.occupancy(index)
            if state not in _VALID_STATES:
                raise ValueError(
                    "occupancy accessor returned invalid state %r" % state)
            if state == OCCUPIED:
                blocking_column = (index[0], index[1])
                blocking_distance = entry_distance
                break
        if inside:
            rays_inside += 1

        distance = _floor_hit_distance(origin, direction, floor.plane_z)
        if distance is None or distance > config.max_range + _EPSILON:
            continue
        cell = floor.world_to_cell(
            _add(origin, _scale(direction, distance)))
        if cell is None:
            continue
        rays_hit_floor += 1
        if (
                blocking_column is not None
                and blocking_column != cell
                and blocking_distance < distance - _EPSILON):
            blocked.add(cell)
            rays_blocked += 1
            continue
        covered.add(cell)

    blocked -= covered
    total = float(floor.cell_count)
    unknown_covered = sum(
        1 for cell in covered if _column_has_unknown(occupancy, floor, cell))
    return {
        "floor_xy_coverage": len(covered) / total if total else 0.0,
        "floor_unknown_gain": unknown_covered / total if total else 0.0,
        "inside_container_fov_ratio": (
            float(rays_inside) / float(rays_cast) if rays_cast else 0.0),
        "outside_container_ratio": (
            1.0 - float(rays_inside) / float(rays_cast) if rays_cast else 0.0),
        "covered_cells": sorted(covered),
        "blocked_cells": sorted(blocked),
        "floor_cells_total": int(floor.cell_count),
        "floor_cells_covered": len(covered),
        "floor_cells_blocked": len(blocked),
        "rays_cast": rays_cast,
        "rays_inside": rays_inside,
        "rays_hit_floor": rays_hit_floor,
        "rays_blocked": rays_blocked,
    }


_HARD_GATES = (
    "hard_feasible",
    "geometry_feasible",
    "ik_feasible",
    "collision_free",
    "trajectory_feasible",
)


def hard_feasibility(candidate):
    """Return ``(feasible, reasons)`` in a fixed diagnostic order."""
    reasons = [
        key for key in _HARD_GATES
        if key in candidate and not bool(candidate[key])
    ]
    return not reasons, reasons


def _component(candidate, name, default=0.0, clamp=False):
    value = _finite(candidate.get(name, default), name)
    return _clamp01(value) if clamp else max(0.0, value)


def score_candidate(
        candidate, occupancy, intrinsics, config=None, weights=None,
        source_index=0):
    """Score one candidate after enforcing all declared hard gates."""
    feasible, reasons = hard_feasibility(candidate)
    candidate_id = str(candidate.get(
        "candidate_id", candidate.get("name", "candidate_%06d" % source_index)))
    base = {
        "candidate_id": candidate_id,
        "source_index": int(source_index),
        "feasible": feasible,
        "score": None,
        "components": {
            "information_gain": 0.0,
            "effective_information_gain": 0.0,
            "corridor_confidence": 0.0,
            "depth_reward": 0.0,
            "manipulability": 0.0,
            "joint_margin": 0.0,
            "trajectory": 0.0,
            "risk": 0.0,
        },
        "diagnostics": {
            "reject_reasons": reasons,
            "raycast": None,
        },
    }
    if not feasible:
        return base

    raycast = raycast_information_gain(
        candidate, occupancy, intrinsics, config=config)
    corridor = _component(candidate, "corridor_confidence", 1.0, clamp=True)
    trajectory = candidate.get("trajectory_quality")
    if trajectory is None:
        trajectory_cost = _component(candidate, "trajectory_cost", 0.0)
        trajectory = 1.0 / (1.0 + trajectory_cost)
    else:
        trajectory = _component(
            candidate, "trajectory_quality", 0.0, clamp=True)
    information_gain = raycast["information_gain"]
    components = {
        "information_gain": information_gain,
        "effective_information_gain": information_gain * corridor,
        "corridor_confidence": corridor,
        "depth_reward": _component(
            candidate, "depth_reward", candidate.get("depth", 0.0)),
        "manipulability": _component(
            candidate, "manipulability", 0.0, clamp=True),
        "joint_margin": _component(
            candidate, "joint_margin", 0.0, clamp=True),
        "trajectory": trajectory,
        "risk": _component(candidate, "risk", 0.0),
    }
    applied_weights = dict(DEFAULT_WEIGHTS)
    if weights:
        unknown = set(weights) - set(DEFAULT_WEIGHTS)
        if unknown:
            raise ValueError("unknown score weights: %s" % sorted(unknown))
        applied_weights.update(
            (name, _finite(value, "weight %s" % name))
            for name, value in weights.items()
        )
    total = (
        applied_weights["information_gain"]
        * components["effective_information_gain"]
        + applied_weights["corridor_confidence"]
        * components["corridor_confidence"]
        + applied_weights["depth"] * components["depth_reward"]
        + applied_weights["manipulability"] * components["manipulability"]
        + applied_weights["joint_margin"] * components["joint_margin"]
        + applied_weights["trajectory"] * components["trajectory"]
        - applied_weights["risk"] * components["risk"]
    )
    base["score"] = total
    base["components"] = components
    base["diagnostics"]["raycast"] = raycast
    return base


def _ranking_key(result):
    if not result["feasible"]:
        return (
            1,
            tuple(result["diagnostics"]["reject_reasons"]),
            result["candidate_id"],
            result["source_index"],
        )
    component = result["components"]
    return (
        0,
        -result["score"],
        -component["effective_information_gain"],
        -component["corridor_confidence"],
        -component["depth_reward"],
        -component["manipulability"],
        -component["joint_margin"],
        -component["trajectory"],
        component["risk"],
        result["candidate_id"],
        result["source_index"],
    )


def rank_candidates(candidates, occupancy, intrinsics, config=None, weights=None):
    """Return all results in a deterministic, documented lexicographic order.

    Feasible candidates precede rejected ones.  Feasible ordering uses total
    score, effective information, corridor confidence, depth, manipulability,
    joint margin, trajectory, lower risk, candidate ID, then input index.
    """
    results = [
        score_candidate(
            candidate, occupancy, intrinsics, config=config, weights=weights,
            source_index=index)
        for index, candidate in enumerate(candidates)
    ]
    return sorted(results, key=_ranking_key)


def stable_diagnostics(results):
    """Serialize ranking diagnostics canonically for replay/golden fixtures."""
    return json.dumps(
        results, sort_keys=True, separators=(",", ":"), allow_nan=False)
