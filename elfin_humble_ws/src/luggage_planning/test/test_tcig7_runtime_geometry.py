#!/usr/bin/env python3
"""Gate G7 runtime geometry, floor, and debug-hull contracts."""

import pytest

from luggage_description.container_geometry import floor_area, normalize_descriptor
from luggage_planning.container_geometry_runtime import (
    GeometryIdentityValidator,
    decode_runtime_geometry,
    encode_runtime_geometry,
    hull_wireframe_segments,
)
from luggage_planning.interior_view_scorer import ContainerFloor


def descriptor(chamfer=True):
    result = {
        "schema_version": 1,
        "frame_id": "container_link",
        "length": 1.49,
        "width": 1.97,
        "floor_z": 0.53,
        "ceiling_z": 2.01,
    }
    if chamfer:
        result["chamfer"] = {
            "side": "positive_y",
            "floor_y": 0.55,
            "wall_y": 0.985,
            "wall_z": 0.90,
        }
    return result


@pytest.mark.parametrize("chamfer", [False, True])
def test_runtime_geometry_round_trip_preserves_canonical_identity(chamfer):
    fields = encode_runtime_geometry(descriptor(chamfer), 7)
    envelope = decode_runtime_geometry(fields)
    expected = normalize_descriptor(descriptor(chamfer))
    assert envelope.descriptor == expected.descriptor()
    assert envelope.geometry_hash == expected.geometry_hash
    assert envelope.geometry_version == 7


def test_runtime_geometry_rejects_hash_tampering():
    fields = encode_runtime_geometry(descriptor(), 1)
    fields["geometry_hash"] = "stale"
    with pytest.raises(ValueError, match="geometry_identity_mismatch"):
        decode_runtime_geometry(fields)


def test_real_floor_cells_exclude_removed_chamfer_wedge():
    geometry = normalize_descriptor(descriptor())
    floor = ContainerFloor(
        center_base=(0.0, 0.0, 1.27),
        yaw=0.0,
        inner_size=(geometry.length, geometry.width, geometry.height),
        resolution=0.10,
        geometry_descriptor=geometry.descriptor(),
    )
    assert floor.total_area == pytest.approx(floor_area(geometry), abs=1e-12)
    assert floor.world_to_cell((0.0, 0.90, floor.plane_z)) is None
    assert floor.world_to_cell((0.0, 0.0, floor.plane_z)) is not None


def test_wireframe_contains_slanted_hull_edges_not_only_aabb_edges():
    segments = hull_wireframe_segments(descriptor())
    assert len(segments) == 15
    assert any(
        first[1] != second[1] and first[2] != second[2]
        for first, second in segments
    )


def test_all_three_runtime_cache_scopes_receive_identity_change():
    validator = GeometryIdentityValidator()
    events = []
    for scope in ("atlas_queries", "candidates", "exploration_snapshots"):
        validator.register_invalidator(
            scope, scope, lambda reason, name=scope: events.append((name, reason)))
    validator.update(descriptor(), 1)
    changed = descriptor()
    changed["length"] = 1.48
    validator.update(changed, 2)
    assert {name for name, _reason in events} == {
        "atlas_queries", "candidates", "exploration_snapshots"
    }
