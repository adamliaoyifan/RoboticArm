#!/usr/bin/env python3
"""ROS-free validation for the geometry-bearing cargo surface contract."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

from luggage_description.container_geometry import normalize_descriptor, volume


SURFACE_SCHEMA_VERSION = 1


class SurfaceMapValidationError(ValueError):
    """Stable fail-closed validation error for a cargo surface map."""

    def __init__(self, reason, detail=""):
        self.reason = str(reason)
        self.detail = str(detail)
        message = self.reason
        if self.detail:
            message += ": " + self.detail
        super().__init__(message)


def _require(mapping, key):
    if key not in mapping:
        raise SurfaceMapValidationError(
            "SURFACE_FIELD_MISSING", str(key))
    return mapping[key]


def _integer(mapping, key, minimum=0):
    value = _require(mapping, key)
    if isinstance(value, bool):
        raise SurfaceMapValidationError("SURFACE_FIELD_MALFORMED", key)
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise SurfaceMapValidationError(
            "SURFACE_FIELD_MALFORMED", key) from exc
    if parsed != value or parsed < minimum:
        raise SurfaceMapValidationError("SURFACE_FIELD_MALFORMED", key)
    return parsed


def _finite(mapping, key, positive=False):
    try:
        value = float(_require(mapping, key))
    except (TypeError, ValueError) as exc:
        raise SurfaceMapValidationError(
            "SURFACE_FIELD_MALFORMED", key) from exc
    if not math.isfinite(value) or (positive and value <= 0.0):
        raise SurfaceMapValidationError("SURFACE_FIELD_MALFORMED", key)
    return value


def _grid(surface, key, nx, ny, numeric=False, unit_interval=False):
    value = _require(surface, key)
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise SurfaceMapValidationError("SURFACE_GRID_MALFORMED", key)
    if len(value) != nx:
        raise SurfaceMapValidationError("SURFACE_GRID_SHAPE_MISMATCH", key)
    checked = []
    for row in value:
        if not isinstance(row, Sequence) or isinstance(row, (str, bytes)):
            raise SurfaceMapValidationError("SURFACE_GRID_MALFORMED", key)
        if len(row) != ny:
            raise SurfaceMapValidationError("SURFACE_GRID_SHAPE_MISMATCH", key)
        if not numeric:
            continue
        for cell in row:
            try:
                number = float(cell)
            except (TypeError, ValueError) as exc:
                raise SurfaceMapValidationError(
                    "SURFACE_GRID_MALFORMED", key) from exc
            if not math.isfinite(number) or number < 0.0:
                raise SurfaceMapValidationError("SURFACE_GRID_MALFORMED", key)
            if unit_interval and number > 1.0 + 1e-12:
                raise SurfaceMapValidationError("SURFACE_GRID_MALFORMED", key)
            checked.append(number)
    return checked if numeric else value


def validate_surface_map(surface, expected_geometry_hash=None):
    """Validate identity, dimensions, semantics, and column geometry.

    The function has no ROS dependency and does not mutate ``surface``.  It
    returns the canonical TCIG-1 geometry descriptor when the contract is
    valid, otherwise it raises :class:`SurfaceMapValidationError` with a stable
    reason.  Wiring this validator into placement consumption is owned by
    TCIG-3, not by this module.
    """
    if not isinstance(surface, Mapping):
        raise SurfaceMapValidationError("SURFACE_MAP_MALFORMED")

    surface_version = _integer(surface, "surface_schema_version", minimum=1)
    if surface_version != SURFACE_SCHEMA_VERSION:
        raise SurfaceMapValidationError(
            "SURFACE_SCHEMA_UNSUPPORTED", str(surface_version))

    descriptor = _require(surface, "geometry_descriptor")
    if not isinstance(descriptor, Mapping):
        raise SurfaceMapValidationError("GEOMETRY_DESCRIPTOR_MALFORMED")
    try:
        geometry = normalize_descriptor(dict(descriptor))
    except (KeyError, TypeError, ValueError) as exc:
        reason = (
            "GEOMETRY_HASH_MISMATCH"
            if "geometry_hash mismatch" in str(exc)
            else "GEOMETRY_DESCRIPTOR_INVALID")
        raise SurfaceMapValidationError(reason, str(exc)) from exc

    surface_hash = str(_require(surface, "geometry_hash"))
    if surface_hash != geometry.geometry_hash:
        raise SurfaceMapValidationError("GEOMETRY_HASH_MISMATCH")
    if expected_geometry_hash is not None and (
            surface_hash != str(expected_geometry_hash)):
        raise SurfaceMapValidationError("GEOMETRY_HASH_MISMATCH")

    geometry_version = _integer(surface, "geometry_schema_version", minimum=1)
    schema_version = _integer(surface, "schema_version", minimum=1)
    if geometry_version != geometry.schema_version or schema_version != geometry.schema_version:
        raise SurfaceMapValidationError("GEOMETRY_SCHEMA_MISMATCH")
    if str(_require(surface, "frame_id")) != geometry.frame_id:
        raise SurfaceMapValidationError("GEOMETRY_FRAME_MISMATCH")
    if str(_require(surface, "height_semantics")) != "meters_above_geometry_floor":
        raise SurfaceMapValidationError("SURFACE_HEIGHT_SEMANTICS_INVALID")
    if str(_require(surface, "floor_semantics")) != "horizontal_hull_floor_intersection":
        raise SurfaceMapValidationError("SURFACE_FLOOR_SEMANTICS_INVALID")
    if not math.isclose(
            _finite(surface, "geometry_floor_z"), geometry.floor_z,
            rel_tol=0.0, abs_tol=1e-12):
        raise SurfaceMapValidationError("SURFACE_FLOOR_MISMATCH")
    if not math.isclose(
            _finite(surface, "floor_z"), 0.0, rel_tol=0.0, abs_tol=1e-12):
        raise SurfaceMapValidationError("SURFACE_HEIGHT_REFERENCE_INVALID")

    _finite(surface, "resolution", positive=True)
    nx = _integer(surface, "nx", minimum=1)
    ny = _integer(surface, "ny", minimum=1)
    for key in (
            "height", "clearance", "known_ratio", "confidence", "state",
            "semantic_label"):
        _grid(surface, key, nx, ny)
    support = _grid(surface, "floor_support_area", nx, ny, numeric=True)
    floor_supported = _grid(surface, "floor_support", nx, ny)
    support_fraction = _grid(
        surface, "floor_support_fraction", nx, ny,
        numeric=True, unit_interval=True)
    column_volume = _grid(
        surface, "column_usable_volume", nx, ny, numeric=True)
    if any(value < -1e-15 for value in support + support_fraction + column_volume):
        raise SurfaceMapValidationError("SURFACE_COLUMN_GEOMETRY_INVALID")
    for ix, row in enumerate(floor_supported):
        for iy, cell in enumerate(row):
            if not isinstance(cell, bool) or cell != (surface["floor_support_area"][ix][iy] > 1e-15):
                raise SurfaceMapValidationError("SURFACE_COLUMN_GEOMETRY_INVALID")
    if not math.isclose(
            math.fsum(column_volume), volume(geometry),
            rel_tol=0.0, abs_tol=1e-8):
        raise SurfaceMapValidationError("SURFACE_COLUMN_VOLUME_MISMATCH")

    return geometry.descriptor()


def validate_surface_map_2d(surface, expected_geometry_hash=None):
    """Descriptive alias retained for consumers naming the 2-D contract."""
    return validate_surface_map(surface, expected_geometry_hash)
