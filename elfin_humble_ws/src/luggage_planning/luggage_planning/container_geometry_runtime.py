#!/usr/bin/env python3
"""ROS-free runtime container-geometry envelopes and identity invalidation."""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Callable, Mapping

from luggage_description.container_geometry import (
    ContainerGeometry,
    normalize_descriptor,
    yz_polygon,
)


RUNTIME_GEOMETRY_ENCODING = "tcig.container_geometry.canonical_json.v1"
IDENTITY_OK = "ok"
IDENTITY_MISSING = "geometry_identity_missing"
IDENTITY_MISMATCH = "geometry_identity_mismatch"
IDENTITY_CHANGED = "geometry_identity_changed"
VERSION_REUSED = "geometry_version_reused"

INVALIDATION_SCOPES = frozenset((
    "atlas_queries", "candidates", "exploration_snapshots"))


@dataclass(frozen=True)
class RuntimeGeometryEnvelope:
    descriptor: dict
    schema_version: int
    geometry_hash: str
    geometry_version: int
    encoding: str = RUNTIME_GEOMETRY_ENCODING


@dataclass(frozen=True)
class IdentityCheck:
    ok: bool
    reason: str


def encode_runtime_geometry(
        descriptor: Mapping[str, object], geometry_version: int) -> dict:
    geometry = normalize_descriptor(dict(descriptor))
    version = int(geometry_version)
    if version <= 0:
        raise ValueError("geometry_version must be positive")
    normalized = geometry.descriptor()
    return {
        "geometry_encoding": RUNTIME_GEOMETRY_ENCODING,
        "geometry_schema_version": geometry.schema_version,
        "geometry_descriptor_json": json.dumps(
            normalized, sort_keys=True, separators=(",", ":"),
            allow_nan=False),
        "geometry_hash": geometry.geometry_hash,
        "geometry_version": version,
    }


def decode_runtime_geometry(fields: Mapping[str, object]) -> RuntimeGeometryEnvelope:
    if str(fields.get("geometry_encoding", "")) != RUNTIME_GEOMETRY_ENCODING:
        raise ValueError(IDENTITY_MISSING)
    raw_json = str(fields.get("geometry_descriptor_json", ""))
    if not raw_json:
        raise ValueError(IDENTITY_MISSING)
    try:
        descriptor = json.loads(raw_json)
    except (TypeError, ValueError) as exc:
        raise ValueError("%s: %s" % (IDENTITY_MISSING, exc))
    if not isinstance(descriptor, dict):
        raise ValueError(IDENTITY_MISSING)
    geometry = normalize_descriptor(descriptor)
    schema_version = int(fields.get("geometry_schema_version", -1))
    supplied_hash = str(fields.get("geometry_hash", ""))
    if (schema_version != geometry.schema_version
            or supplied_hash != geometry.geometry_hash):
        raise ValueError(IDENTITY_MISMATCH)
    version = int(fields.get("geometry_version", 0))
    if version <= 0:
        raise ValueError(IDENTITY_MISSING)
    return RuntimeGeometryEnvelope(
        descriptor=geometry.descriptor(),
        schema_version=geometry.schema_version,
        geometry_hash=geometry.geometry_hash,
        geometry_version=version,
    )


class GeometryIdentityValidator:
    """Tracks sensed identity and explicitly invalidates dependent caches."""

    def __init__(self):
        self._current = None
        self._hooks = {scope: {} for scope in INVALIDATION_SCOPES}

    @property
    def current(self):
        return self._current

    def register_invalidator(
            self, scope: str, name: str, callback: Callable[[str], None]):
        if scope not in INVALIDATION_SCOPES:
            raise ValueError("unsupported invalidation scope: %s" % scope)
        label = str(name).strip()
        if not label or not callable(callback):
            raise ValueError("invalidator name and callback are required")
        self._hooks[scope][label] = callback

    def update(self, descriptor: Mapping[str, object], geometry_version: int):
        encoded = encode_runtime_geometry(descriptor, geometry_version)
        envelope = decode_runtime_geometry(encoded)
        previous = self._current
        if previous is not None:
            if (envelope.geometry_hash != previous.geometry_hash
                    and envelope.geometry_version <= previous.geometry_version):
                raise ValueError(VERSION_REUSED)
            changed = (
                envelope.geometry_hash != previous.geometry_hash
                or envelope.geometry_version != previous.geometry_version)
            if changed:
                for scope in sorted(self._hooks):
                    for name in sorted(self._hooks[scope]):
                        self._hooks[scope][name](IDENTITY_CHANGED)
        self._current = envelope
        return envelope

    def validate(self, geometry_hash: str, geometry_version: int | None = None):
        if self._current is None:
            return IdentityCheck(False, IDENTITY_MISSING)
        if str(geometry_hash) != self._current.geometry_hash:
            return IdentityCheck(False, IDENTITY_MISMATCH)
        if (geometry_version is not None
                and int(geometry_version) != self._current.geometry_version):
            return IdentityCheck(False, IDENTITY_MISMATCH)
        return IdentityCheck(True, IDENTITY_OK)


def hull_wireframe_segments(descriptor: Mapping[str, object]):
    """Return exact hull edge segments in the descriptor frame.

    Each segment is ``((x0, y0, z0), (x1, y1, z1))``.  A seven-face hull
    therefore exposes the slanted edge instead of drawing the removed AABB
    wedge as usable space.
    """
    geometry: ContainerGeometry = normalize_descriptor(dict(descriptor))
    cross_section = yz_polygon(geometry)
    segments = []
    for x in (-geometry.half_x, geometry.half_x):
        for index, (y0, z0) in enumerate(cross_section):
            y1, z1 = cross_section[(index + 1) % len(cross_section)]
            segments.append(((x, y0, z0), (x, y1, z1)))
    for y, z in cross_section:
        segments.append(((-geometry.half_x, y, z),
                         (geometry.half_x, y, z)))
    return tuple(segments)


__all__ = [
    "GeometryIdentityValidator", "IdentityCheck", "RuntimeGeometryEnvelope",
    "decode_runtime_geometry", "encode_runtime_geometry",
    "hull_wireframe_segments", "RUNTIME_GEOMETRY_ENCODING",
    "IDENTITY_OK", "IDENTITY_MISSING", "IDENTITY_MISMATCH",
    "IDENTITY_CHANGED", "VERSION_REUSED", "INVALIDATION_SCOPES",
]
