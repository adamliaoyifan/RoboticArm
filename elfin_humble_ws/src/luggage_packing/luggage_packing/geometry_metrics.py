#!/usr/bin/env python3
"""TCIG-5 metric adapter: exact usable volume and floor area from TCIG-1.

This module is the only utilization-metric adapter from a normalized TCIG-1
descriptor to G5 denominators. It never falls back to the legacy rectangular
constants ``4.344`` or ``1.49 * 1.97``.

Floor-contact rule: a box belongs in ``floor_coverage`` only when the bottom
face is within ``FLOOR_CONTACT_TOL_M`` (1 mm) of the real TCIG ``floor_z``.
That tolerance is the same 1 mm peak gate already used by the bringup
placement records; it is deterministic and documented here rather than taken
from eval/Gazebo truth.
"""

from __future__ import annotations

import math
from itertools import combinations
from typing import Iterable, Mapping, Sequence

from luggage_description.container_geometry import (
    ContainerGeometry,
    floor_area as hull_floor_area,
    normalize_descriptor,
    volume as hull_volume,
    y_max_at_z,
)

FLOOR_CONTACT_TOL_M = 1e-3
METRIC_SCHEMA_VERSION = 1
LEGACY_REPLAY_FIELDS = (
    "floor_coverage",
    "V_container",
    "overall_fill_rate",
    "reachable_volume_ratio",
    "reachable_fill_rate",
)

_EPS = 1e-12


class GeometryMetricsError(ValueError):
    """Fail-closed metric error with a stable ``reason`` code."""

    def __init__(self, reason: str, message: str = ""):
        self.reason = str(reason)
        super().__init__(message or self.reason)


def _finite(value: object, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise GeometryMetricsError(
            "NON_FINITE_GEOMETRY", "%s is not a number" % name) from exc
    if not math.isfinite(number):
        raise GeometryMetricsError(
            "NON_FINITE_GEOMETRY", "%s must be finite" % name)
    return number


def _map_normalize_error(exc: Exception) -> GeometryMetricsError:
    message = str(exc)
    lowered = message.lower()
    if "geometry_hash mismatch" in lowered:
        reason = "HASH_MISMATCH"
    elif "must be finite" in lowered:
        reason = "NON_FINITE_GEOMETRY"
    elif "unsupported" in lowered:
        reason = "UNSUPPORTED_GEOMETRY"
    elif isinstance(exc, KeyError):
        reason = "MALFORMED_GEOMETRY"
        message = "missing field %s" % exc
    else:
        reason = "MALFORMED_GEOMETRY"
    return GeometryMetricsError(reason, message)


def denominators(descriptor: Mapping | ContainerGeometry | None) -> dict:
    """Return exact hull denominators plus identity. Never rectangular defaults."""
    if descriptor is None:
        raise GeometryMetricsError("MISSING_GEOMETRY", "geometry descriptor is missing")
    if isinstance(descriptor, ContainerGeometry):
        geometry = descriptor
    elif isinstance(descriptor, Mapping):
        if not descriptor:
            raise GeometryMetricsError(
                "MISSING_GEOMETRY", "geometry descriptor is empty")
        try:
            geometry = normalize_descriptor(dict(descriptor))
        except GeometryMetricsError:
            raise
        except Exception as exc:  # noqa: BLE001 - map kernel errors to G5 reasons
            raise _map_normalize_error(exc) from exc
    else:
        raise GeometryMetricsError(
            "MALFORMED_GEOMETRY", "geometry descriptor must be a mapping")
    usable_volume = hull_volume(geometry)
    usable_floor = hull_floor_area(geometry)
    if not math.isfinite(usable_volume) or not math.isfinite(usable_floor):
        raise GeometryMetricsError(
            "NON_FINITE_GEOMETRY", "exact hull denominators are not finite")
    if usable_volume <= 0.0 or usable_floor <= 0.0:
        raise GeometryMetricsError(
            "MALFORMED_GEOMETRY", "exact hull denominators must be positive")
    return {
        "usable_volume_m3": usable_volume,
        "floor_area_m2": usable_floor,
        "schema_version": int(geometry.schema_version),
        "metric_schema_version": METRIC_SCHEMA_VERSION,
        "geometry_hash": geometry.geometry_hash,
        "frame_id": geometry.frame_id,
        "geometry": geometry,
    }


def denominators_from_scene_config(config: Mapping) -> dict:
    from luggage_description.container_geometry import descriptor_from_scene_config

    if not config:
        raise GeometryMetricsError("MISSING_GEOMETRY", "scene config is missing")
    try:
        geometry = descriptor_from_scene_config(dict(config))
    except Exception as exc:  # noqa: BLE001 - map kernel errors to G5 reasons
        raise _map_normalize_error(exc) from exc
    return denominators(geometry)


def volume_fraction_report(packed_volume_m3: object, denom: Mapping) -> dict:
    packed = _finite(packed_volume_m3, "packed_volume_m3")
    if packed < 0.0:
        raise GeometryMetricsError(
            "INVALID_BOX_RECORD", "packed_volume_m3 must be non-negative")
    usable = _finite(denom["usable_volume_m3"], "usable_volume_m3")
    return {
        "volume_fraction": packed / usable,
        "packed_volume_m3": packed,
        "usable_volume_m3": usable,
        "schema_version": int(denom["schema_version"]),
        "metric_schema_version": int(denom.get(
            "metric_schema_version", METRIC_SCHEMA_VERSION)),
        "geometry_hash": str(denom["geometry_hash"]),
    }


def _shoelace_xy(points: Sequence[tuple[float, float]]) -> float:
    if len(points) < 3:
        return 0.0
    twice = 0.0
    for index, (x0, y0) in enumerate(points):
        x1, y1 = points[(index + 1) % len(points)]
        twice += x0 * y1 - x1 * y0
    return abs(0.5 * twice)


def _dedupe_xy(points: Sequence[tuple[float, float]]) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    for point in points:
        if not out or math.hypot(out[-1][0] - point[0], out[-1][1] - point[1]) > _EPS:
            out.append((float(point[0]), float(point[1])))
    if len(out) > 1 and math.hypot(out[0][0] - out[-1][0], out[0][1] - out[-1][1]) <= _EPS:
        out.pop()
    return out


def _inside_left(
    a: tuple[float, float], b: tuple[float, float], p: tuple[float, float]
) -> bool:
    return ((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])) >= -_EPS


def _intersect_segments(
    a: tuple[float, float],
    b: tuple[float, float],
    c: tuple[float, float],
    d: tuple[float, float],
) -> tuple[float, float]:
    x1, y1 = a
    x2, y2 = b
    x3, y3 = c
    x4, y4 = d
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(den) < _EPS:
        return b
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / den
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / den
    return (px, py)


def _clip_convex(
    subject: Sequence[tuple[float, float]],
    clip: Sequence[tuple[float, float]],
) -> list[tuple[float, float]]:
    output = list(subject)
    if len(clip) < 3:
        return []
    for index, start in enumerate(clip):
        end = clip[(index + 1) % len(clip)]
        input_pts = output
        output = []
        if not input_pts:
            return []
        previous = input_pts[-1]
        for current in input_pts:
            current_in = _inside_left(start, end, current)
            previous_in = _inside_left(start, end, previous)
            if current_in:
                if not previous_in:
                    output.append(_intersect_segments(previous, current, start, end))
                output.append(current)
            elif previous_in:
                output.append(_intersect_segments(previous, current, start, end))
            previous = current
    return _dedupe_xy(output)


def floor_polygon(geometry: ContainerGeometry) -> list[tuple[float, float]]:
    half_x = geometry.half_x
    y_min = -geometry.half_y
    y_max = y_max_at_z(geometry, geometry.floor_z, margin=0.0)
    return [
        (-half_x, y_min),
        (half_x, y_min),
        (half_x, y_max),
        (-half_x, y_max),
    ]


def _oriented_footprint(
    center_xy: Sequence[float], size_xy: Sequence[float], yaw: float
) -> list[tuple[float, float]]:
    cx, cy = float(center_xy[0]), float(center_xy[1])
    hx, hy = 0.5 * float(size_xy[0]), 0.5 * float(size_xy[1])
    cos_yaw = math.cos(float(yaw))
    sin_yaw = math.sin(float(yaw))
    local = ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))
    corners = []
    for dx, dy in local:
        corners.append((
            cx + cos_yaw * dx - sin_yaw * dy,
            cy + sin_yaw * dx + cos_yaw * dy,
        ))
    return corners


def _union_area(polygons: Sequence[Sequence[tuple[float, float]]]) -> float:
    convex = [list(poly) for poly in polygons if len(poly) >= 3]
    n = len(convex)
    if n == 0:
        return 0.0
    if n > 16:
        raise GeometryMetricsError(
            "OVERLAPPING_FLOOR_FOOTPRINTS",
            "too many floor footprints (%d) for exact union" % n)
    total = 0.0
    for rank in range(1, n + 1):
        sign = 1.0 if rank % 2 == 1 else -1.0
        for combo in combinations(range(n), rank):
            inter = list(convex[combo[0]])
            empty = False
            for index in combo[1:]:
                inter = _clip_convex(inter, convex[index])
                if len(inter) < 3:
                    empty = True
                    break
            if not empty:
                total += sign * _shoelace_xy(inter)
    return max(0.0, total)


def _box_center(record: Mapping, geometry: ContainerGeometry) -> list[float]:
    if record.get("center") is not None:
        center = record["center"]
        return [
            _finite(center[0], "center.x"),
            _finite(center[1], "center.y"),
            _finite(center[2], "center.z") if len(center) > 2 else geometry.floor_z,
        ]
    if record.get("center_container_link") is not None:
        center = record["center_container_link"]
        return [
            _finite(center[0], "center_container_link.x"),
            _finite(center[1], "center_container_link.y"),
            _finite(center[2], "center_container_link.z") if len(center) > 2
            else geometry.floor_z,
        ]
    if "container_x" in record and "container_y" in record:
        z_value = record.get("center_z")
        if z_value is None and record.get("peak") is not None and record.get("size"):
            height = _finite(record["size"][2], "size.height")
            peak = _finite(record["peak"], "peak")
            z_value = geometry.floor_z + peak + 0.5 * height
        return [
            _finite(record["container_x"], "container_x"),
            _finite(record["container_y"], "container_y"),
            geometry.floor_z if z_value is None else _finite(z_value, "center_z"),
        ]
    raise GeometryMetricsError(
        "INVALID_BOX_RECORD", "box record is missing container-frame XY")


def _box_size(record: Mapping) -> list[float]:
    size = record.get("size") or record.get("size_wdh")
    if not size or len(size) < 3:
        raise GeometryMetricsError(
            "INVALID_BOX_RECORD", "box record is missing size")
    values = [
        _finite(size[0], "size.width"),
        _finite(size[1], "size.depth"),
        _finite(size[2], "size.height"),
    ]
    if any(value <= 0.0 for value in values):
        raise GeometryMetricsError(
            "INVALID_BOX_RECORD", "box size values must be positive")
    return values


def _box_yaw(record: Mapping) -> float:
    if "yaw" in record:
        return _finite(record["yaw"], "yaw")
    if "yaw_container_link" in record:
        return _finite(record["yaw_container_link"], "yaw_container_link")
    return 0.0


def _bottom_z(record: Mapping, center: Sequence[float], size: Sequence[float],
              geometry: ContainerGeometry) -> float:
    if record.get("bottom_z") is not None:
        return _finite(record["bottom_z"], "bottom_z")
    if record.get("peak") is not None:
        return geometry.floor_z + _finite(record["peak"], "peak")
    return float(center[2]) - 0.5 * float(size[2])


def _is_floor_contact(record: Mapping, geometry: ContainerGeometry) -> bool:
    size = _box_size(record)
    center = _box_center(record, geometry)
    bottom = _bottom_z(record, center, size, geometry)
    return abs(bottom - geometry.floor_z) <= FLOOR_CONTACT_TOL_M


def floor_coverage_report(
    boxes: Iterable[Mapping], denom: Mapping
) -> dict:
    geometry = denom.get("geometry")
    if not isinstance(geometry, ContainerGeometry):
        raise GeometryMetricsError(
            "MALFORMED_GEOMETRY", "denominators are missing normalized geometry")
    floor_poly = floor_polygon(geometry)
    clipped = []
    floor_count = 0
    for record in boxes:
        if not _is_floor_contact(record, geometry):
            continue
        floor_count += 1
        center = _box_center(record, geometry)
        size = _box_size(record)
        yaw = _box_yaw(record)
        footprint = _oriented_footprint(center[:2], size[:2], yaw)
        clipped_poly = _clip_convex(footprint, floor_poly)
        if len(clipped_poly) >= 3:
            clipped.append(clipped_poly)
    packed_floor = _union_area(clipped)
    usable_floor = _finite(denom["floor_area_m2"], "floor_area_m2")
    return {
        "floor_coverage": packed_floor / usable_floor,
        "packed_floor_area_m2": packed_floor,
        "floor_area_m2": usable_floor,
        "floor_item_count": floor_count,
        "schema_version": int(denom["schema_version"]),
        "metric_schema_version": int(denom.get(
            "metric_schema_version", METRIC_SCHEMA_VERSION)),
        "geometry_hash": str(denom["geometry_hash"]),
        "floor_contact_tol_m": FLOOR_CONTACT_TOL_M,
    }


def capacity_report(
    packed_volume_m3: object,
    boxes: Iterable[Mapping],
    descriptor: Mapping | ContainerGeometry,
) -> dict:
    """Authoritative G5 volume and floor report used by online/eval entry points."""
    denom = denominators(descriptor)
    volume = volume_fraction_report(packed_volume_m3, denom)
    floor = floor_coverage_report(boxes, denom)
    return {
        "volume_fraction": volume["volume_fraction"],
        "packed_volume_m3": volume["packed_volume_m3"],
        "usable_volume_m3": volume["usable_volume_m3"],
        "floor_coverage": floor["floor_coverage"],
        "packed_floor_area_m2": floor["packed_floor_area_m2"],
        "floor_area_m2": floor["floor_area_m2"],
        "floor_item_count": floor["floor_item_count"],
        "schema_version": volume["schema_version"],
        "metric_schema_version": volume["metric_schema_version"],
        "geometry_hash": volume["geometry_hash"],
        "frame_id": denom["frame_id"],
        "floor_contact_tol_m": FLOOR_CONTACT_TOL_M,
    }


def packed_volume_from_boxes(boxes: Iterable[Mapping]) -> float:
    total = 0.0
    for record in boxes:
        size = _box_size(record)
        total += size[0] * size[1] * size[2]
    return total


def annotate_replay_runs(runs: Sequence[Mapping], descriptor) -> dict:
    """Recompute G5 volume_fraction; mark TCIG-6-owned replay fields legacy."""
    denom = denominators(descriptor)
    v_placed = [_finite(run.get("V_placed", 0.0), "V_placed") for run in runs]
    numerator = (sum(v_placed) / len(v_placed)) if v_placed else 0.0
    volume = volume_fraction_report(numerator, denom)
    legacy = {}
    if runs:
        first = runs[0]
        overall = [
            _finite(run.get("overall_fill_rate", 0.0), "overall_fill_rate")
            for run in runs]
        floor_cov = [
            _finite(run.get("floor_coverage", 0.0), "floor_coverage")
            for run in runs]
        rfr = [
            _finite(run.get("reachable_fill_rate", 0.0), "reachable_fill_rate")
            for run in runs]
        legacy = {
            "status": "legacy_non_authoritative",
            "owner": "TCIG-6",
            "note": (
                "Replay floor_coverage, V_container, overall_fill_rate, "
                "reachable_volume_ratio, and reachable_fill_rate originate in "
                "packing_replay.py and cannot satisfy a G5 floor or G7 "
                "reachability claim. Authoritative replay floor coverage is "
                "deferred to TCIG-6."
            ),
            "floor_coverage": (
                sum(floor_cov) / len(floor_cov) if floor_cov else 0.0),
            "V_container": first.get("V_container"),
            "overall_fill_rate": (
                sum(overall) / len(overall) if overall else 0.0),
            "reachable_volume_ratio": first.get("reachable_volume_ratio"),
            "reachable_fill_rate": (
                sum(rfr) / len(rfr) if rfr else 0.0),
        }
    else:
        legacy = {
            "status": "legacy_non_authoritative",
            "owner": "TCIG-6",
            "note": "no replay runs",
            "floor_coverage": 0.0,
            "V_container": None,
            "overall_fill_rate": 0.0,
            "reachable_volume_ratio": None,
            "reachable_fill_rate": 0.0,
        }
    for field in LEGACY_REPLAY_FIELDS:
        legacy.setdefault(field, None)
    return {
        "volume_fraction": volume,
        "legacy_non_authoritative": legacy,
        "schema_version": volume["schema_version"],
        "geometry_hash": volume["geometry_hash"],
        "usable_volume_m3": volume["usable_volume_m3"],
    }
