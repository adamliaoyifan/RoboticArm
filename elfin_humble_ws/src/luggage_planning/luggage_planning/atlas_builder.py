#!/usr/bin/env python3
"""ROS-free, hull-aware reachability atlas builder kernel.

The kernel owns grid iteration, geometry masking and artifact semantics.  ROS
adapters inject an IK callback; tests can inject a deterministic plain-Python
callback.  No ROS or message packages are imported here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Mapping, Sequence

import numpy as np

from luggage_description.container_geometry import (
    ContainerGeometry,
    aabb_intersection_volume,
    contains_oriented_box,
    contains_point,
    normalize_descriptor,
    volume,
    y_max_at_z,
)


UNKNOWN = 0
UNREACHABLE = 1
MARGINAL = 2
REACHABLE = 3

BUILDER_REVISION = "tcig7-hull-aware-atlas-v3"
PAYLOAD_YAW_CONVENTION = "container_z_ccw_radians"
PAYLOAD_CENTER_CONVENTION = "contact_plus_container_offset"


@dataclass(frozen=True)
class AtlasGrid:
    resolution: float
    origin: tuple[float, float, float]
    size: tuple[int, int, int]
    yaw_bins: tuple[float, ...]
    frame: str = "container_link"

    @classmethod
    def covering_geometry(
            cls, descriptor: Mapping[str, object], resolution: float,
            yaw_bins: Sequence[float]) -> "AtlasGrid":
        geometry = normalize_descriptor(dict(descriptor))
        step = _positive(resolution, "resolution")
        return cls(
            resolution=step,
            origin=(-geometry.half_x, -geometry.half_y, geometry.floor_z),
            size=(
                int(math.ceil(geometry.length / step)),
                int(math.ceil(geometry.width / step)),
                int(math.ceil(geometry.height / step)),
            ),
            yaw_bins=tuple(float(value) for value in yaw_bins),
            frame=geometry.frame_id,
        ).validated()

    def validated(self) -> "AtlasGrid":
        resolution = _positive(self.resolution, "grid resolution")
        origin = tuple(_finite(value, "grid origin") for value in self.origin)
        if len(origin) != 3:
            raise ValueError("grid origin must contain three values")
        size = tuple(int(value) for value in self.size)
        if len(size) != 3 or any(value <= 0 for value in size):
            raise ValueError("grid size must contain three positive integers")
        yaw_bins = tuple(_finite(value, "yaw bin") for value in self.yaw_bins)
        if not yaw_bins:
            raise ValueError("at least one yaw bin is required")
        frame = str(self.frame).strip()
        if not frame:
            raise ValueError("grid frame is required")
        return AtlasGrid(resolution, origin, size, yaw_bins, frame)


@dataclass(frozen=True)
class PayloadProfile:
    """Nominal payload relative to the sampled suction contact point."""

    enabled: bool = False
    size: tuple[float, float, float] | None = None
    center_offset: tuple[float, float, float] | None = None
    yaw_convention: str = PAYLOAD_YAW_CONVENTION
    center_convention: str = PAYLOAD_CENTER_CONVENTION

    def validated(self) -> "PayloadProfile":
        if not self.enabled:
            return PayloadProfile()
        if self.size is None:
            raise ValueError("enabled payload requires size")
        size = tuple(_positive(value, "payload size") for value in self.size)
        if len(size) != 3:
            raise ValueError("payload size must contain three values")
        raw_offset = self.center_offset
        if raw_offset is None:
            raw_offset = (0.0, 0.0, -0.5 * size[2])
        offset = tuple(_finite(value, "payload center offset") for value in raw_offset)
        if len(offset) != 3:
            raise ValueError("payload center offset must contain three values")
        if self.yaw_convention != PAYLOAD_YAW_CONVENTION:
            raise ValueError("unsupported payload yaw convention")
        if self.center_convention != PAYLOAD_CENTER_CONVENTION:
            raise ValueError("unsupported payload center convention")
        return PayloadProfile(True, size, offset, self.yaw_convention,
                              self.center_convention)

    def metadata(self) -> dict[str, object]:
        profile = self.validated()
        return {
            "enabled": profile.enabled,
            "shape": "box" if profile.enabled else None,
            "size": list(profile.size) if profile.size is not None else None,
            "center_offset": (
                list(profile.center_offset)
                if profile.center_offset is not None else None),
            "yaw_convention": profile.yaw_convention,
            "center_convention": profile.center_convention,
        }


def _finite(value: object, name: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("%s must be finite" % name)
    return number


def _positive(value: object, name: str) -> float:
    number = _finite(value, name)
    if number <= 0.0:
        raise ValueError("%s must be positive" % name)
    return number


def payload_center(
        contact: Sequence[float], yaw: float,
        profile: PayloadProfile) -> tuple[float, float, float]:
    profile = profile.validated()
    if not profile.enabled or profile.center_offset is None:
        return tuple(float(value) for value in contact)
    ox, oy, oz = profile.center_offset
    cosine = math.cos(float(yaw))
    sine = math.sin(float(yaw))
    return (
        float(contact[0]) + cosine * ox - sine * oy,
        float(contact[1]) + sine * ox + cosine * oy,
        float(contact[2]) + oz,
    )


def sample_is_hull_valid(
        geometry: ContainerGeometry, contact: Sequence[float], yaw: float,
        profile: PayloadProfile) -> bool:
    profile = profile.validated()
    if not contains_point(geometry, contact):
        return False
    if not profile.enabled:
        return True
    return contains_oriented_box(
        geometry, payload_center(contact, yaw, profile), profile.size,
        yaw=float(yaw))


def _cell_sample(geometry, lower, resolution):
    """Choose a deterministic point inside a clipped hull cell.

    The allocation's final cells may extend past the geometry AABB, and cells
    crossing the chamfer may have their rectangular center in the removed
    wedge.  Sampling the center of the clipped cross-section keeps every
    positive-volume contact cell represented without invoking IK outside the
    hull.
    """
    upper = (
        min(lower[0] + resolution, geometry.half_x),
        min(lower[1] + resolution, geometry.half_y),
        min(lower[2] + resolution, geometry.ceiling_z),
    )
    x = 0.5 * (max(lower[0], -geometry.half_x) + upper[0])
    z = 0.5 * (max(lower[2], geometry.floor_z) + upper[2])
    usable_y_upper = min(upper[1], y_max_at_z(geometry, z))
    y = 0.5 * (max(lower[1], -geometry.half_y) + usable_y_upper)
    return (x, y, z)


def compute_geometry_masks(
        descriptor: Mapping[str, object], grid: AtlasGrid,
        payload: PayloadProfile | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(active_mask_4d, clipped_cell_volume_3d)``."""
    geometry = normalize_descriptor(dict(descriptor))
    grid = grid.validated()
    if grid.frame != geometry.frame_id:
        raise ValueError("grid frame does not match geometry frame")
    profile = (payload or PayloadProfile()).validated()
    nx, ny, nz = grid.size
    step = grid.resolution
    volumes = np.zeros((nx, ny, nz), dtype=np.float64)
    active = np.zeros((nx, ny, nz, len(grid.yaw_bins)), dtype=np.bool_)
    for ix in range(nx):
        for iy in range(ny):
            for iz in range(nz):
                lower = (
                    grid.origin[0] + ix * step,
                    grid.origin[1] + iy * step,
                    grid.origin[2] + iz * step,
                )
                upper = tuple(value + step for value in lower)
                cell_volume = aabb_intersection_volume(geometry, lower, upper)
                volumes[ix, iy, iz] = cell_volume
                if cell_volume <= 0.0:
                    continue
                contact = _cell_sample(geometry, lower, step)
                for iyaw, yaw in enumerate(grid.yaw_bins):
                    active[ix, iy, iz, iyaw] = sample_is_hull_valid(
                        geometry, contact, yaw, profile)
    return active, volumes


def _normalize_callback_result(result: object) -> dict[str, object]:
    if isinstance(result, (bool, np.bool_)):
        return {"status": REACHABLE if bool(result) else UNREACHABLE}
    if isinstance(result, (int, np.integer)):
        return {"status": int(result)}
    if isinstance(result, Mapping):
        return dict(result)
    raise ValueError("IK callback must return bool, status integer, or mapping")


class AtlasBuilderKernel:
    """Build a schema-v3 atlas while skipping hull-invalid samples."""

    def __init__(
            self, descriptor: Mapping[str, object], grid: AtlasGrid,
            ik_callback: Callable[[float, float, float, float], object],
            payload: PayloadProfile | None = None, max_seeds: int = 1,
            builder_revision: str = BUILDER_REVISION):
        self.geometry = normalize_descriptor(dict(descriptor))
        self.grid = grid.validated()
        if self.grid.frame != self.geometry.frame_id:
            raise ValueError("grid frame does not match geometry frame")
        if not callable(ik_callback):
            raise ValueError("ik_callback must be callable")
        self.ik_callback = ik_callback
        self.payload = (payload or PayloadProfile()).validated()
        self.max_seeds = int(max_seeds)
        if self.max_seeds <= 0:
            raise ValueError("max_seeds must be positive")
        self.builder_revision = str(builder_revision).strip()
        if not self.builder_revision:
            raise ValueError("builder_revision is required")

    def build(self) -> tuple[dict[str, np.ndarray], dict[str, object]]:
        active, cell_volumes = compute_geometry_masks(
            self.geometry.descriptor(), self.grid, self.payload)
        shape = active.shape
        status = np.full(shape, UNKNOWN, dtype=np.uint8)
        opening_connected = np.zeros(shape, dtype=np.bool_)
        contact_ik = np.zeros(shape, dtype=np.bool_)
        transit_ik = np.zeros(shape, dtype=np.bool_)
        contact_seeds = np.zeros(shape + (self.max_seeds, 6), dtype=np.float64)
        transit_seeds = np.zeros_like(contact_seeds)
        solution_count = np.zeros(shape, dtype=np.uint8)
        joint_margin = np.full(shape, np.nan, dtype=np.float32)
        manipulability = np.full(shape, np.nan, dtype=np.float32)
        neighbor_confidence = np.zeros(shape, dtype=np.float32)

        step = self.grid.resolution
        for index in np.ndindex(shape):
            if not active[index]:
                continue
            ix, iy, iz, iyaw = index
            lower = (
                self.grid.origin[0] + ix * step,
                self.grid.origin[1] + iy * step,
                self.grid.origin[2] + iz * step,
            )
            contact = _cell_sample(self.geometry, lower, step)
            decoded = _normalize_callback_result(
                self.ik_callback(*contact, self.grid.yaw_bins[iyaw]))
            cell_status = int(decoded.get("status", UNKNOWN))
            if cell_status not in (UNKNOWN, UNREACHABLE, MARGINAL, REACHABLE):
                raise ValueError("IK callback returned unsupported status")
            status[index] = cell_status
            seeds = list(decoded.get("contact_seeds", decoded.get("seeds", [])))
            transit = list(decoded.get("transit_seeds", seeds))
            used = min(len(seeds), len(transit), self.max_seeds)
            for seed_index in range(used):
                if len(seeds[seed_index]) != 6 or len(transit[seed_index]) != 6:
                    raise ValueError("IK seeds must contain six joints")
                contact_seeds[index + (seed_index,)] = seeds[seed_index]
                transit_seeds[index + (seed_index,)] = transit[seed_index]
            solution_count[index] = used
            reachable = cell_status in (MARGINAL, REACHABLE)
            contact_ik[index] = bool(decoded.get("contact_ik", reachable))
            transit_ik[index] = bool(decoded.get("transit_ik", reachable))
            opening_connected[index] = bool(
                decoded.get("opening_connected", reachable))
            joint_margin[index] = float(decoded.get("joint_margin", np.nan))
            manipulability[index] = float(decoded.get("manipulability", np.nan))
            neighbor_confidence[index] = float(
                decoded.get("neighbor_confidence", 0.0))

        reachable = np.logical_or(status == MARGINAL, status == REACHABLE)
        legacy_seeds = np.zeros(shape + (6,), dtype=np.float64)
        has_seed = solution_count > 0
        legacy_seeds[has_seed] = contact_seeds[..., 0, :][has_seed]
        data = {
            "active_mask": active,
            "cell_volumes": cell_volumes,
            "status": status,
            "opening_connected": opening_connected,
            "reachable": reachable,
            "contact_ik": contact_ik,
            "transit_ik": transit_ik,
            "contact_seeds": contact_seeds,
            "transit_seeds": transit_seeds,
            "solution_count": solution_count,
            "neighbor_confidence": neighbor_confidence,
            "seed_joints": legacy_seeds,
            "joint_margin": joint_margin,
            "manipulability": manipulability,
        }
        active_count = int(np.count_nonzero(active))
        reachable_count = int(np.count_nonzero(np.logical_and(active, reachable)))
        descriptor = self.geometry.descriptor()
        meta = {
            "atlas_version": "3.0",
            "schema_version": 3,
            "builder_revision": self.builder_revision,
            "geometry": {
                "schema_version": self.geometry.schema_version,
                "geometry_hash": self.geometry.geometry_hash,
                "descriptor": descriptor,
            },
            "payload": self.payload.metadata(),
            "grid": {
                "frame": self.grid.frame,
                "resolution_xyz": self.grid.resolution,
                "origin": list(self.grid.origin),
                "size": list(self.grid.size),
                "yaw_bins": list(self.grid.yaw_bins),
            },
            "stats": {
                "total_cells": active_count,
                "allocated_cells": int(active.size),
                "inactive_cells": int(active.size - active_count),
                "active_cells": active_count,
                "reachable_cells": reachable_count,
                "reachability_rate": (
                    float(reachable_count) / active_count if active_count else 0.0),
                "usable_hull_volume_m3": volume(self.geometry),
            },
        }
        return data, meta


__all__ = [
    "AtlasBuilderKernel", "AtlasGrid", "PayloadProfile", "BUILDER_REVISION",
    "PAYLOAD_YAW_CONVENTION", "PAYLOAD_CENTER_CONVENTION",
    "compute_geometry_masks", "payload_center", "sample_is_hull_valid",
]
