#!/usr/bin/env python3
"""ROS-free container inner-hull geometry primitives."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from typing import Iterable, Sequence


EPS = 1e-9


@dataclass(frozen=True)
class Chamfer:
    side: str
    floor_y: float
    wall_y: float
    wall_z: float


@dataclass(frozen=True)
class ContainerGeometry:
    schema_version: int
    frame_id: str
    length: float
    width: float
    floor_z: float
    ceiling_z: float
    chamfer: Chamfer | None = None
    geometry_hash: str = ""

    @property
    def half_x(self) -> float:
        return 0.5 * self.length

    @property
    def half_y(self) -> float:
        return 0.5 * self.width

    @property
    def height(self) -> float:
        return self.ceiling_z - self.floor_z

    def descriptor(self, include_hash: bool = True) -> dict:
        data: dict[str, object] = {
            "schema_version": int(self.schema_version),
            "frame_id": self.frame_id,
            "length": self.length,
            "width": self.width,
            "floor_z": self.floor_z,
            "ceiling_z": self.ceiling_z,
        }
        if self.chamfer is not None:
            data["chamfer"] = {
                "side": self.chamfer.side,
                "floor_y": self.chamfer.floor_y,
                "wall_y": self.chamfer.wall_y,
                "wall_z": self.chamfer.wall_z,
            }
        if include_hash:
            data["geometry_hash"] = self.geometry_hash or geometry_hash(data)
        return data


def _finite(value: object, name: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("%s must be finite" % name)
    return number


def _round_float(value: float) -> float:
    return round(float(value), 12)


def _canonical_payload(descriptor: dict) -> dict:
    payload: dict[str, object] = {
        "schema_version": int(descriptor.get("schema_version", 1)),
        "frame_id": str(descriptor.get("frame_id", "container_link")),
        "length": _round_float(_finite(descriptor["length"], "length")),
        "width": _round_float(_finite(descriptor["width"], "width")),
        "floor_z": _round_float(_finite(descriptor.get("floor_z", 0.0), "floor_z")),
        "ceiling_z": _round_float(
            _finite(descriptor.get("ceiling_z", descriptor.get("height")), "ceiling_z")
        ),
    }
    chamfer = descriptor.get("chamfer")
    if chamfer:
        raw = dict(chamfer)
        payload["chamfer"] = {
            "side": str(raw.get("side", "positive_y")).strip().lower(),
            "floor_y": _round_float(_finite(raw["floor_y"], "chamfer.floor_y")),
            "wall_y": _round_float(
                _finite(raw.get("wall_y", 0.5 * float(payload["width"])), "chamfer.wall_y")
            ),
            "wall_z": _round_float(_finite(raw["wall_z"], "chamfer.wall_z")),
        }
    return payload


def geometry_hash(descriptor: dict) -> str:
    payload = _canonical_payload({k: v for k, v in descriptor.items() if k != "geometry_hash"})
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def normalize_descriptor(descriptor: dict) -> ContainerGeometry:
    payload = _canonical_payload(descriptor)
    length = float(payload["length"])
    width = float(payload["width"])
    floor_z = float(payload["floor_z"])
    ceiling_z = float(payload["ceiling_z"])
    if length <= 0.0:
        raise ValueError("length must be positive")
    if width <= 0.0:
        raise ValueError("width must be positive")
    if ceiling_z <= floor_z:
        raise ValueError("ceiling_z must be greater than floor_z")
    chamfer = None
    raw_chamfer = payload.get("chamfer")
    if raw_chamfer:
        raw = dict(raw_chamfer)
        side = str(raw["side"])
        if side != "positive_y":
            raise ValueError("unsupported chamfer side: %s" % side)
        floor_y = float(raw["floor_y"])
        wall_y = float(raw["wall_y"])
        wall_z = float(raw["wall_z"])
        half_y = 0.5 * width
        if wall_z <= floor_z or wall_z >= ceiling_z:
            raise ValueError("chamfer.wall_z must be between floor_z and ceiling_z")
        if floor_y < -half_y or floor_y > half_y:
            raise ValueError("chamfer.floor_y must be inside the inner width")
        if wall_y < floor_y or wall_y > half_y:
            raise ValueError("chamfer.wall_y must be inside the positive-y wall")
        chamfer = Chamfer(side=side, floor_y=floor_y, wall_y=wall_y, wall_z=wall_z)
    normalized = {
        "schema_version": int(payload["schema_version"]),
        "frame_id": str(payload["frame_id"]),
        "length": length,
        "width": width,
        "floor_z": floor_z,
        "ceiling_z": ceiling_z,
    }
    if chamfer is not None:
        normalized["chamfer"] = {
            "side": chamfer.side,
            "floor_y": chamfer.floor_y,
            "wall_y": chamfer.wall_y,
            "wall_z": chamfer.wall_z,
        }
    expected_hash = geometry_hash(normalized)
    supplied_hash = descriptor.get("geometry_hash")
    if supplied_hash and str(supplied_hash) != expected_hash:
        raise ValueError("geometry_hash mismatch")
    return ContainerGeometry(
        schema_version=int(payload["schema_version"]),
        frame_id=str(payload["frame_id"]),
        length=length,
        width=width,
        floor_z=floor_z,
        ceiling_z=ceiling_z,
        chamfer=chamfer,
        geometry_hash=expected_hash,
    )


def descriptor_from_scene_config(config: dict) -> ContainerGeometry:
    container = config.get("container", {})
    inner = container.get("inner", {})
    descriptor = {
        "schema_version": int(inner.get("schema_version", 1)),
        "frame_id": str(inner.get("frame_id", "container_link")),
        "length": inner.get("length", 2.3),
        "width": inner.get("width", 1.9),
        "floor_z": inner.get("floor_z", 0.0),
        "ceiling_z": inner.get("ceiling_z", inner.get("height", 2.1)),
    }
    if inner.get("chamfer"):
        descriptor["chamfer"] = dict(inner["chamfer"])
    return normalize_descriptor(descriptor)


def y_max_at_z(geometry: ContainerGeometry, z: float, margin: float = 0.0) -> float:
    clearance = max(0.0, float(margin))
    limit = geometry.half_y - clearance
    chamfer = geometry.chamfer
    if chamfer is None:
        return limit
    z = float(z)
    if z >= chamfer.wall_z - EPS:
        return limit
    slope = (chamfer.wall_y - chamfer.floor_y) / (chamfer.wall_z - geometry.floor_z)
    intercept = chamfer.floor_y - slope * geometry.floor_z
    plane_limit = slope * z + intercept - clearance * math.sqrt(1.0 + slope * slope)
    return min(limit, plane_limit)


def yz_polygon(geometry: ContainerGeometry, margin: float = 0.0) -> list[tuple[float, float]]:
    clearance = max(0.0, float(margin))
    floor_z = geometry.floor_z + clearance
    ceiling_z = geometry.ceiling_z - clearance
    min_y = -geometry.half_y + clearance
    if ceiling_z < floor_z:
        return []
    chamfer = geometry.chamfer
    if chamfer is None:
        max_y = geometry.half_y - clearance
        if max_y < min_y:
            return []
        return [(min_y, floor_z), (max_y, floor_z), (max_y, ceiling_z), (min_y, ceiling_z)]
    floor_y = y_max_at_z(geometry, floor_z, margin=clearance)
    wall_z = min(max(chamfer.wall_z, floor_z), ceiling_z)
    wall_y = y_max_at_z(geometry, wall_z, margin=clearance)
    top_y = geometry.half_y - clearance
    if floor_y < min_y or wall_y < min_y or top_y < min_y:
        return []
    points = [(min_y, floor_z), (floor_y, floor_z)]
    if wall_z > floor_z + EPS:
        points.append((wall_y, wall_z))
    points.append((top_y, ceiling_z))
    points.append((min_y, ceiling_z))
    return _dedupe_polygon(points)


def _dedupe_polygon(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    deduped: list[tuple[float, float]] = []
    for point in points:
        if not deduped or _dist2(deduped[-1], point) > EPS * EPS:
            deduped.append(point)
    if len(deduped) > 1 and _dist2(deduped[0], deduped[-1]) <= EPS * EPS:
        deduped.pop()
    return deduped


def _dist2(a: tuple[float, float], b: tuple[float, float]) -> float:
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


def polygon_area(points: Sequence[tuple[float, float]]) -> float:
    if len(points) < 3:
        return 0.0
    twice = 0.0
    for index, (y0, z0) in enumerate(points):
        y1, z1 = points[(index + 1) % len(points)]
        twice += y0 * z1 - y1 * z0
    return abs(0.5 * twice)


def volume(geometry: ContainerGeometry) -> float:
    return geometry.length * polygon_area(yz_polygon(geometry))


def floor_area(geometry: ContainerGeometry) -> float:
    floor_y_max = y_max_at_z(geometry, geometry.floor_z, margin=0.0)
    return geometry.length * max(0.0, floor_y_max + geometry.half_y)


def contains_point(
    geometry: ContainerGeometry,
    point: Sequence[float],
    margin: float = 0.0,
) -> bool:
    clearance = max(0.0, float(margin))
    x, y, z = [float(value) for value in point]
    if x < -geometry.half_x + clearance - EPS or x > geometry.half_x - clearance + EPS:
        return False
    if y < -geometry.half_y + clearance - EPS or y > geometry.half_y - clearance + EPS:
        return False
    if z < geometry.floor_z + clearance - EPS or z > geometry.ceiling_z - clearance + EPS:
        return False
    return y <= y_max_at_z(geometry, z, margin=clearance) + EPS


def oriented_box_corners(
    center: Sequence[float],
    size: Sequence[float],
    yaw: float = 0.0,
) -> list[tuple[float, float, float]]:
    cx, cy, cz = [float(value) for value in center]
    sx, sy, sz = [0.5 * float(value) for value in size]
    cos_yaw = math.cos(float(yaw))
    sin_yaw = math.sin(float(yaw))
    corners = []
    for dx in (-sx, sx):
        for dy in (-sy, sy):
            for dz in (-sz, sz):
                rx = cos_yaw * dx - sin_yaw * dy
                ry = sin_yaw * dx + cos_yaw * dy
                corners.append((cx + rx, cy + ry, cz + dz))
    return corners


def contains_oriented_box(
    geometry: ContainerGeometry,
    center: Sequence[float],
    size: Sequence[float],
    yaw: float = 0.0,
    margin: float = 0.0,
) -> bool:
    if any(float(value) <= 0.0 for value in size):
        raise ValueError("box size values must be positive")
    return all(
        contains_point(geometry, corner, margin=margin)
        for corner in oriented_box_corners(center, size, yaw)
    )


def contains_swept_box(
    geometry: ContainerGeometry,
    start_center: Sequence[float],
    end_center: Sequence[float],
    size: Sequence[float],
    yaw: float = 0.0,
    margin: float = 0.0,
) -> bool:
    start = [float(value) for value in start_center]
    end = [float(value) for value in end_center]
    candidates = [0.0, 1.0]
    for axis in (2,):
        delta = end[axis] - start[axis]
        if abs(delta) < EPS:
            continue
        for z in (geometry.floor_z, geometry.ceiling_z):
            t = (z - start[axis]) / delta
            if EPS < t < 1.0 - EPS:
                candidates.append(t)
        chamfer = geometry.chamfer
        if chamfer is not None:
            t = (chamfer.wall_z - start[axis]) / delta
            if EPS < t < 1.0 - EPS:
                candidates.append(t)
    for t in sorted(set(round(value, 12) for value in candidates)):
        center = [start[i] + (end[i] - start[i]) * t for i in range(3)]
        if not contains_oriented_box(geometry, center, size, yaw=yaw, margin=margin):
            return False
    return True


def aabb_intersection_volume(
    geometry: ContainerGeometry,
    lower: Sequence[float],
    upper: Sequence[float],
    margin: float = 0.0,
) -> float:
    x0, y0, z0 = [float(value) for value in lower]
    x1, y1, z1 = [float(value) for value in upper]
    if x1 < x0:
        x0, x1 = x1, x0
    if y1 < y0:
        y0, y1 = y1, y0
    if z1 < z0:
        z0, z1 = z1, z0
    x_min = max(x0, -geometry.half_x + max(0.0, float(margin)))
    x_max = min(x1, geometry.half_x - max(0.0, float(margin)))
    if x_max <= x_min:
        return 0.0
    clipped = clip_polygon_rect(yz_polygon(geometry, margin=margin), y0, y1, z0, z1)
    return (x_max - x_min) * polygon_area(clipped)


def clip_polygon_rect(
    points: Sequence[tuple[float, float]],
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
) -> list[tuple[float, float]]:
    clipped = list(points)
    bounds = (
        (0, float(y_min), True),
        (0, float(y_max), False),
        (1, float(z_min), True),
        (1, float(z_max), False),
    )
    for axis, bound, keep_greater in bounds:
        clipped = _clip_halfspace(clipped, axis, bound, keep_greater)
        if not clipped:
            return []
    return _dedupe_polygon(clipped)


def _clip_halfspace(
    points: list[tuple[float, float]],
    axis: int,
    bound: float,
    keep_greater: bool,
) -> list[tuple[float, float]]:
    if not points:
        return []

    def inside(point: tuple[float, float]) -> bool:
        value = point[axis]
        return value >= bound - EPS if keep_greater else value <= bound + EPS

    def intersect(a: tuple[float, float], b: tuple[float, float]) -> tuple[float, float]:
        av = a[axis]
        bv = b[axis]
        if abs(bv - av) < EPS:
            return b
        t = (bound - av) / (bv - av)
        return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)

    output: list[tuple[float, float]] = []
    previous = points[-1]
    previous_inside = inside(previous)
    for current in points:
        current_inside = inside(current)
        if current_inside:
            if not previous_inside:
                output.append(intersect(previous, current))
            output.append(current)
        elif previous_inside:
            output.append(intersect(previous, current))
        previous = current
        previous_inside = current_inside
    return output


def floor_support_area(
    geometry: ContainerGeometry,
    lower_xy: Sequence[float],
    upper_xy: Sequence[float],
    margin: float = 0.0,
) -> float:
    x0, y0 = [float(value) for value in lower_xy]
    x1, y1 = [float(value) for value in upper_xy]
    if x1 < x0:
        x0, x1 = x1, x0
    if y1 < y0:
        y0, y1 = y1, y0
    clearance = max(0.0, float(margin))
    x_min = max(x0, -geometry.half_x + clearance)
    x_max = min(x1, geometry.half_x - clearance)
    y_min = max(y0, -geometry.half_y + clearance)
    y_max = min(y1, y_max_at_z(geometry, geometry.floor_z + clearance, margin=clearance))
    return max(0.0, x_max - x_min) * max(0.0, y_max - y_min)


def payload_footprint_half_extents(size_xy: Sequence[float], yaw: float) -> tuple[float, float]:
    sx, sy = [0.5 * float(value) for value in size_xy]
    if sx <= 0.0 or sy <= 0.0:
        raise ValueError("payload footprint dimensions must be positive")
    cos_yaw = abs(math.cos(float(yaw)))
    sin_yaw = abs(math.sin(float(yaw)))
    return cos_yaw * sx + sin_yaw * sy, sin_yaw * sx + cos_yaw * sy


def payload_center_y_interval(
    geometry: ContainerGeometry,
    payload_size: Sequence[float],
    z_min: float,
    z_max: float,
    yaw: float = 0.0,
    margin: float = 0.0,
) -> tuple[float, float]:
    _half_x, half_y = payload_footprint_half_extents(payload_size[:2], yaw)
    lower_z = float(z_min)
    upper_z = float(z_max)
    if upper_z < lower_z:
        lower_z, upper_z = upper_z, lower_z
    z_samples = [lower_z, upper_z]
    chamfer = geometry.chamfer
    if chamfer is not None and lower_z <= chamfer.wall_z <= upper_z:
        z_samples.append(chamfer.wall_z)
    min_y = -geometry.half_y + max(0.0, float(margin)) + half_y
    max_y = min(y_max_at_z(geometry, z, margin=margin) for z in z_samples) - half_y
    return min_y, max_y


def payload_center_yz_polygon(
    geometry: ContainerGeometry,
    payload_size: Sequence[float],
    z_min: float,
    z_max: float,
    yaw: float = 0.0,
    margin: float = 0.0,
) -> list[tuple[float, float]]:
    y0, y1 = payload_center_y_interval(
        geometry, payload_size, z_min, z_max, yaw=yaw, margin=margin
    )
    z0 = max(float(z_min), geometry.floor_z + max(0.0, float(margin)))
    z1 = min(float(z_max), geometry.ceiling_z - max(0.0, float(margin)))
    if y1 < y0 or z1 < z0:
        return []
    return [(y0, z0), (y1, z0), (y1, z1), (y0, z1)]


def sum_aabb_tiles(
    geometry: ContainerGeometry,
    x_edges: Iterable[float],
    y_edges: Iterable[float],
    z_edges: Iterable[float],
) -> float:
    xs = list(x_edges)
    ys = list(y_edges)
    zs = list(z_edges)
    total = 0.0
    for ix in range(len(xs) - 1):
        for iy in range(len(ys) - 1):
            for iz in range(len(zs) - 1):
                total += aabb_intersection_volume(
                    geometry,
                    (xs[ix], ys[iy], zs[iz]),
                    (xs[ix + 1], ys[iy + 1], zs[iz + 1]),
                )
    return total
