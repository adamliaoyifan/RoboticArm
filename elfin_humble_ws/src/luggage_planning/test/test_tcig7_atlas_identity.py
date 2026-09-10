#!/usr/bin/env python3
"""Gate G7 tests for hull masking, identity, migration, and runtime geometry."""

import hashlib
import os
import tempfile

import numpy as np
import pytest
import yaml

from luggage_description.container_geometry import (
    contains_point,
    normalize_descriptor,
    volume,
)
from luggage_planning.atlas_builder import (
    AtlasBuilderKernel,
    AtlasGrid,
    PayloadProfile,
)
from luggage_planning.atlas_migration import (
    MIGRATION_PROVENANCE_INCOMPLETE,
    migrate_atlas_pair,
)
from luggage_planning.container_geometry_runtime import (
    GeometryIdentityValidator,
    VERSION_REUSED,
    decode_runtime_geometry,
    encode_runtime_geometry,
    hull_wireframe_segments,
)
from luggage_planning.reachability_atlas import (
    GEOMETRY_IDENTITY_CHANGED,
    GEOMETRY_IDENTITY_MISMATCH,
    GEOMETRY_IDENTITY_REQUIRED,
    HULL_INVALID_PAYLOAD,
    AtlasIdentityError,
    REACHABLE,
    ReachabilityAtlas,
)


def seven_face_descriptor():
    return normalize_descriptor({
        "schema_version": 1,
        "frame_id": "container_link",
        "length": 1.0,
        "width": 1.0,
        "floor_z": 0.0,
        "ceiling_z": 1.0,
        "chamfer": {
            "side": "positive_y",
            "floor_y": 0.0,
            "wall_y": 0.5,
            "wall_z": 0.5,
        },
    }).descriptor()


def cuboid_descriptor():
    return normalize_descriptor({
        "schema_version": 1,
        "frame_id": "container_link",
        "length": 1.0,
        "width": 1.0,
        "floor_z": 0.0,
        "ceiling_z": 1.0,
    }).descriptor()


def _build(descriptor, resolution=0.25, payload=None, callback=None):
    grid = AtlasGrid.covering_geometry(descriptor, resolution, (0.0,))
    calls = []

    def accepting(x, y, z, yaw):
        calls.append((x, y, z, yaw))
        if callback is not None:
            return callback(x, y, z, yaw)
        return {"status": REACHABLE, "seeds": [[0, 1, 2, 3, 4, 5]]}

    data, meta = AtlasBuilderKernel(
        descriptor, grid, accepting, payload=payload).build()
    return data, meta, calls


def test_small_grid_skips_hull_invalid_cells_without_calling_ik():
    data, _meta, calls = _build(seven_face_descriptor())
    assert len(calls) == int(np.count_nonzero(data["active_mask"]))
    assert len(calls) < data["active_mask"].size


def test_complete_payload_is_masked_before_ik():
    descriptor = seven_face_descriptor()
    payload = PayloadProfile(enabled=True, size=(0.2, 0.2, 0.2))
    data, _meta, calls = _build(descriptor, resolution=0.125, payload=payload)
    # This contact is inside, while the payload's lower +Y corner crosses the
    # slanted face.  The callback must never see the sample.
    grid = AtlasGrid.covering_geometry(descriptor, 0.125, (0.0,))
    ix, iy, iz = 4, 4, 2
    contact = tuple(grid.origin[i] + (index + 0.5) * grid.resolution
                    for i, index in enumerate((ix, iy, iz)))
    assert contains_point(normalize_descriptor(descriptor), contact)
    assert not data["active_mask"][ix, iy, iz, 0]
    assert contact + (0.0,) not in calls


def test_query_rejects_runtime_hull_invalid_payload_and_changed_identity():
    descriptor = seven_face_descriptor()
    payload = PayloadProfile(enabled=True, size=(0.2, 0.2, 0.2))
    data, meta, _calls = _build(descriptor, resolution=0.125, payload=payload)
    atlas = ReachabilityAtlas.from_builder(meta=meta, **data)
    result = atlas.query(0.0, 0.08, 0.31, 0.0)
    assert result.reason == HULL_INVALID_PAYLOAD
    atlas.invalidate_geometry()
    assert atlas.query(0.0, -0.2, 0.3, 0.0).reason == GEOMETRY_IDENTITY_CHANGED


def test_load_requires_exact_identity_and_volume_ratio_uses_exact_hull():
    descriptor = seven_face_descriptor()
    data, meta, _calls = _build(descriptor, resolution=0.25)
    atlas = ReachabilityAtlas.from_builder(meta=meta, **data)
    with tempfile.TemporaryDirectory() as tmp:
        npz_path = os.path.join(tmp, "atlas.npz")
        meta_path = os.path.join(tmp, "atlas.yaml")
        atlas.save(npz_path, meta_path)
        with pytest.raises(AtlasIdentityError, match=GEOMETRY_IDENTITY_REQUIRED):
            ReachabilityAtlas.load(npz_path, meta_path)
        with pytest.raises(AtlasIdentityError, match=GEOMETRY_IDENTITY_MISMATCH):
            ReachabilityAtlas.load(
                npz_path, meta_path, geometry_descriptor=cuboid_descriptor())
        loaded = ReachabilityAtlas.load(
            npz_path, meta_path, geometry_descriptor=descriptor)
        assert loaded.stats()["usable_hull_volume"] == pytest.approx(
            volume(normalize_descriptor(descriptor)), abs=1e-12)
        assert loaded.reachable_volume_ratio() == pytest.approx(1.0, abs=1e-12)


def test_resolution_change_is_within_one_clipped_boundary_cell():
    descriptor = seven_face_descriptor()
    ratios = []
    tolerances = []
    geometry = normalize_descriptor(descriptor)
    for resolution in (0.19, 0.27):
        data, meta, _calls = _build(descriptor, resolution=resolution)
        atlas = ReachabilityAtlas.from_builder(meta=meta, **data)
        ratios.append(atlas.reachable_volume_ratio())
        tolerances.append(float(np.max(data["cell_volumes"])) / volume(geometry))
    assert abs(ratios[0] - ratios[1]) <= max(tolerances) + 1e-12


def _legacy_fixture(prefix, descriptor, include_payload=True):
    shape = (2, 2, 2, 1)
    data = {
        "status": np.full(shape, REACHABLE, dtype=np.uint8),
        "reachable": np.ones(shape, dtype=np.bool_),
        "opening_connected": np.ones(shape, dtype=np.bool_),
        "contact_ik": np.ones(shape, dtype=np.bool_),
        "transit_ik": np.ones(shape, dtype=np.bool_),
        "contact_seeds": np.zeros(shape + (1, 6), dtype=np.float64),
        "transit_seeds": np.zeros(shape + (1, 6), dtype=np.float64),
        "seed_joints": np.zeros(shape + (6,), dtype=np.float64),
        "solution_count": np.ones(shape, dtype=np.uint8),
        "joint_margin": np.ones(shape, dtype=np.float32),
        "manipulability": np.ones(shape, dtype=np.float32),
        "neighbor_confidence": np.ones(shape, dtype=np.float32),
    }
    np.savez_compressed(prefix + ".npz", **data)
    geometry = normalize_descriptor(descriptor)
    meta = {
        "atlas_version": "2.0",
        "schema_version": 2,
        "builder": {
            "algorithm": "legacy-test",
            "deterministic_fingerprint": "abc123",
        },
        "grid": {
            "frame": "container_link",
            "resolution_xyz": 0.5,
            "origin": [-0.5, -0.5, 0.0],
            "size": [2, 2, 2],
            "yaw_bins": [0.0],
        },
        "container": {
            "inner_dimensions": [geometry.length, geometry.width, 1.0],
            "floor_z": geometry.floor_z,
            "ceiling_z": geometry.ceiling_z,
        },
    }
    if include_payload:
        meta["payload"] = {
            "enabled": False,
            "shape": None,
            "size": None,
            "offset": None,
        }
    with open(prefix + ".yaml", "w", encoding="utf-8") as stream:
        yaml.safe_dump(meta, stream, sort_keys=False)


def test_migration_is_deterministic_and_records_provenance():
    descriptor = seven_face_descriptor()
    with tempfile.TemporaryDirectory() as tmp:
        source = os.path.join(tmp, "source")
        _legacy_fixture(source, descriptor)
        outputs = [os.path.join(tmp, "migrated-a"), os.path.join(tmp, "migrated-b")]
        metas = []
        for output in outputs:
            metas.append(migrate_atlas_pair(
                source + ".npz", source + ".yaml", descriptor,
                output + ".npz", output + ".yaml"))
        assert metas[0] == metas[1]
        for suffix in (".npz", ".yaml"):
            blobs = []
            for output in outputs:
                with open(output + suffix, "rb") as stream:
                    blobs.append(stream.read())
            assert hashlib.sha256(blobs[0]).digest() == hashlib.sha256(blobs[1]).digest()
        assert metas[0]["schema_version"] == 3
        assert metas[0]["geometry"]["geometry_hash"] == descriptor["geometry_hash"]
        assert metas[0]["migration"]["source_schema_version"] == 2
        loaded = ReachabilityAtlas.load(
            outputs[0] + ".npz", outputs[0] + ".yaml",
            geometry_descriptor=descriptor)
        assert loaded.geometry_hash == descriptor["geometry_hash"]


def test_migration_refuses_missing_payload_provenance():
    descriptor = seven_face_descriptor()
    with tempfile.TemporaryDirectory() as tmp:
        source = os.path.join(tmp, "source")
        _legacy_fixture(source, descriptor, include_payload=False)
        with pytest.raises(ValueError, match=MIGRATION_PROVENANCE_INCOMPLETE):
            migrate_atlas_pair(
                source + ".npz", source + ".yaml", descriptor)


@pytest.mark.parametrize("descriptor", [cuboid_descriptor(), seven_face_descriptor()])
def test_runtime_geometry_round_trip(descriptor):
    encoded = encode_runtime_geometry(descriptor, geometry_version=7)
    decoded = decode_runtime_geometry(encoded)
    assert decoded.descriptor == descriptor
    assert decoded.geometry_hash == descriptor["geometry_hash"]
    assert decoded.geometry_version == 7


def test_geometry_change_invalidates_all_runtime_cache_scopes():
    validator = GeometryIdentityValidator()
    invalidated = []
    for scope in ("atlas_queries", "candidates", "exploration_snapshots"):
        validator.register_invalidator(
            scope, scope,
            lambda reason, scope=scope: invalidated.append((scope, reason)))
    validator.update(cuboid_descriptor(), 1)
    validator.update(seven_face_descriptor(), 2)
    assert {scope for scope, _reason in invalidated} == {
        "atlas_queries", "candidates", "exploration_snapshots"}
    assert all(reason == GEOMETRY_IDENTITY_CHANGED
               for _scope, reason in invalidated)
    with pytest.raises(ValueError, match=VERSION_REUSED):
        validator.update(cuboid_descriptor(), 2)


def test_hull_wireframe_contains_slanted_face_and_not_aabb_wedge():
    cuboid = hull_wireframe_segments(cuboid_descriptor())
    hull = hull_wireframe_segments(seven_face_descriptor())
    assert len(cuboid) == 12
    assert len(hull) == 15
    assert any(
        start[1] != end[1] and start[2] != end[2]
        for start, end in hull)
