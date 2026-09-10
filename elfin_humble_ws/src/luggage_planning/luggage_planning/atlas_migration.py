#!/usr/bin/env python3
"""Deterministic, provenance-checked migration of schema-v2 atlases."""

from __future__ import annotations

import argparse
import copy
import hashlib
import os
from pathlib import Path
from typing import Mapping

import numpy as np
import yaml

from luggage_description.container_geometry import (
    descriptor_from_scene_config,
    normalize_descriptor,
)

from .atlas_builder import (
    AtlasGrid,
    BUILDER_REVISION,
    PayloadProfile,
    compute_geometry_masks,
)
from .atlas_io import save_npz_deterministic


MIGRATION_PROVENANCE_INCOMPLETE = "migration_provenance_incomplete"
MIGRATION_SOURCE_SCHEMA_UNSUPPORTED = "migration_source_schema_unsupported"


def _sha256(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                return digest.hexdigest()
            digest.update(chunk)


def _require(mapping: Mapping[str, object], key: str, context: str):
    if key not in mapping:
        raise ValueError("%s: missing %s.%s" % (
            MIGRATION_PROVENANCE_INCOMPLETE, context, key))
    return mapping[key]


def _legacy_grid(meta: Mapping[str, object]) -> AtlasGrid:
    raw = _require(meta, "grid", "metadata")
    if not isinstance(raw, Mapping):
        raise ValueError("%s: grid" % MIGRATION_PROVENANCE_INCOMPLETE)
    return AtlasGrid(
        resolution=_require(raw, "resolution_xyz", "grid"),
        origin=tuple(_require(raw, "origin", "grid")),
        size=tuple(_require(raw, "size", "grid")),
        yaw_bins=tuple(_require(raw, "yaw_bins", "grid")),
        frame=str(_require(raw, "frame", "grid")),
    ).validated()


def _legacy_payload(meta: Mapping[str, object]) -> tuple[PayloadProfile, str]:
    raw = _require(meta, "payload", "metadata")
    if not isinstance(raw, Mapping) or "enabled" not in raw:
        raise ValueError("%s: payload" % MIGRATION_PROVENANCE_INCOMPLETE)
    if not bool(raw["enabled"]):
        return PayloadProfile(), "contact_only"
    if str(_require(raw, "shape", "payload")) != "box":
        raise ValueError("%s: payload.shape" % MIGRATION_PROVENANCE_INCOMPLETE)
    size = tuple(_require(raw, "size", "payload"))
    attach_offset = tuple(_require(raw, "offset", "payload"))
    if len(attach_offset) != 3:
        raise ValueError("%s: payload.offset" % MIGRATION_PROVENANCE_INCOMPLETE)
    # V2 sampled a tool-down suction frame.  Positive local tool Z therefore
    # pointed down in container coordinates; preserve that physical profile.
    center_offset = (
        float(attach_offset[0]), -float(attach_offset[1]),
        -float(attach_offset[2]))
    return PayloadProfile(
        enabled=True, size=size, center_offset=center_offset).validated(), (
            "tool_down_attach_xyz_to_container_offset_(x,-y,-z)")


def _validate_container_provenance(meta, geometry):
    raw = _require(meta, "container", "metadata")
    if not isinstance(raw, Mapping):
        raise ValueError("%s: container" % MIGRATION_PROVENANCE_INCOMPLETE)
    dimensions = tuple(_require(raw, "inner_dimensions", "container"))
    floor_z = float(_require(raw, "floor_z", "container"))
    ceiling_z = float(_require(raw, "ceiling_z", "container"))
    if len(dimensions) < 2:
        raise ValueError("%s: container.inner_dimensions" %
                         MIGRATION_PROVENANCE_INCOMPLETE)
    if (abs(float(dimensions[0]) - geometry.length) > 1e-9
            or abs(float(dimensions[1]) - geometry.width) > 1e-9
            or abs(floor_z - geometry.floor_z) > 1e-9
            or abs(ceiling_z - geometry.ceiling_z) > 1e-9):
        raise ValueError("legacy_container_geometry_mismatch")


def _masked_arrays(data, active_mask, cell_volumes):
    arrays = {name: np.array(value, copy=True) for name, value in data.items()}
    shape = active_mask.shape
    rules = {
        "status": 0,
        "reachable": False,
        "opening_connected": False,
        "contact_ik": False,
        "transit_ik": False,
        "solution_count": 0,
        "neighbor_confidence": 0.0,
        "joint_margin": np.nan,
        "manipulability": np.nan,
    }
    for name, fill in rules.items():
        if name in arrays:
            if arrays[name].shape != shape:
                raise ValueError("%s array shape mismatch" % name)
            arrays[name][~active_mask] = fill
    for name in ("contact_seeds", "transit_seeds"):
        if name in arrays:
            if arrays[name].shape[:4] != shape:
                raise ValueError("%s array shape mismatch" % name)
            arrays[name][~active_mask] = 0.0
    if "seed_joints" in arrays:
        if arrays["seed_joints"].shape[:4] != shape:
            raise ValueError("seed_joints array shape mismatch")
        arrays["seed_joints"][~active_mask] = 0.0
    arrays["active_mask"] = active_mask
    arrays["cell_volumes"] = cell_volumes
    return arrays


def migrate_atlas_pair(
        npz_path: str, meta_path: str, descriptor: Mapping[str, object],
        output_npz: str | None = None, output_meta: str | None = None):
    """Migrate one v2 pair to v3, refusing incomplete provenance."""
    output_npz = output_npz or npz_path
    output_meta = output_meta or meta_path
    with open(meta_path, "r", encoding="utf-8") as stream:
        source_meta = yaml.safe_load(stream)
    if not isinstance(source_meta, Mapping):
        raise ValueError(MIGRATION_PROVENANCE_INCOMPLETE)
    if int(source_meta.get("schema_version", 0)) != 2:
        raise ValueError(MIGRATION_SOURCE_SCHEMA_UNSUPPORTED)
    geometry = normalize_descriptor(dict(descriptor))
    grid = _legacy_grid(source_meta)
    if grid.frame != geometry.frame_id:
        raise ValueError("legacy_grid_geometry_frame_mismatch")
    _ = _require(source_meta, "builder", "metadata")
    builder = source_meta["builder"]
    if not isinstance(builder, Mapping):
        raise ValueError(MIGRATION_PROVENANCE_INCOMPLETE)
    algorithm = str(_require(builder, "algorithm", "builder"))
    fingerprint = str(_require(
        builder, "deterministic_fingerprint", "builder"))
    if not algorithm or not fingerprint:
        raise ValueError(MIGRATION_PROVENANCE_INCOMPLETE)
    _validate_container_provenance(source_meta, geometry)
    payload, offset_transform = _legacy_payload(source_meta)
    with np.load(npz_path, allow_pickle=False) as archive:
        source_data = {name: archive[name] for name in archive.files}
    status = source_data.get("status")
    expected_shape = grid.size + (len(grid.yaw_bins),)
    if status is None or status.shape != expected_shape:
        raise ValueError("%s: status/grid shape" %
                         MIGRATION_PROVENANCE_INCOMPLETE)

    source_npz_hash = _sha256(npz_path)
    source_meta_hash = _sha256(meta_path)
    active_mask, cell_volumes = compute_geometry_masks(
        geometry.descriptor(), grid, payload)
    arrays = _masked_arrays(source_data, active_mask, cell_volumes)

    meta = copy.deepcopy(dict(source_meta))
    meta["atlas_version"] = "3.0"
    meta["schema_version"] = 3
    meta["builder_revision"] = "migrated:%s:%s" % (
        algorithm, fingerprint)
    meta["geometry"] = {
        "schema_version": geometry.schema_version,
        "geometry_hash": geometry.geometry_hash,
        "descriptor": geometry.descriptor(),
    }
    meta["payload"] = payload.metadata()
    status = arrays["status"]
    active_total = int(np.count_nonzero(active_mask))
    reachable = int(np.count_nonzero(np.logical_and(
        active_mask, status == 3)))
    marginal = int(np.count_nonzero(np.logical_and(
        active_mask, status == 2)))
    meta["stats"] = {
        "total_cells": active_total,
        "allocated_cells": int(active_mask.size),
        "active_cells": active_total,
        "inactive_cells": int(active_mask.size - active_total),
        "reachable_cells": reachable,
        "marginal_cells": marginal,
        "unreachable_cells": int(np.count_nonzero(np.logical_and(
            active_mask, status == 1))),
        "unknown_cells": int(np.count_nonzero(np.logical_and(
            active_mask, status == 0))),
        "reachability_rate": (
            float(reachable + marginal) / active_total if active_total else 0.0),
    }
    meta["migration"] = {
        "tool_revision": BUILDER_REVISION,
        "source_schema_version": 2,
        "source_npz_sha256": source_npz_hash,
        "source_metadata_sha256": source_meta_hash,
        "active_mask_rule": "contact_and_complete_payload_inside_tcig1_hull",
        "payload_offset_transform": offset_transform,
    }
    save_npz_deterministic(output_npz, arrays)
    os.makedirs(os.path.dirname(output_meta) or ".", exist_ok=True)
    temporary = output_meta + ".tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        yaml.safe_dump(meta, stream, default_flow_style=False, sort_keys=False)
    os.replace(temporary, output_meta)
    return meta


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Migrate provenance-complete reachability atlas v2 pairs")
    parser.add_argument("--scene-config", required=True)
    parser.add_argument(
        "prefix", nargs="+",
        help="atlas path prefix; .npz and .yaml are appended")
    args = parser.parse_args(argv)
    with open(args.scene_config, "r", encoding="utf-8") as stream:
        geometry = descriptor_from_scene_config(yaml.safe_load(stream))
    for raw_prefix in args.prefix:
        prefix = str(Path(raw_prefix))
        migrate_atlas_pair(
            prefix + ".npz", prefix + ".yaml", geometry.descriptor())
    return 0


__all__ = [
    "migrate_atlas_pair", "MIGRATION_PROVENANCE_INCOMPLETE",
    "MIGRATION_SOURCE_SCHEMA_UNSUPPORTED",
]


if __name__ == "__main__":
    raise SystemExit(main())
