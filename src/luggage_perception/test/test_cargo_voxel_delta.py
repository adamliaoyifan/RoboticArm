"""Evidence capture is independent of the fusion algorithm and primary layout."""
import numpy as np
import pytest

from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.cargo_voxel_delta import VoxelDeltaBuilder


def mapper():
    return CargoVolumeMapper((.12, .12, .12), (0, 0, .06), 0, .02)


def test_first_old_final_new_includes_shadow_and_identity():
    m = mapper()
    d = VoxelDeltaBuilder(m)
    m._voxel_delta = d
    m.mark_occupied_voxel(3, 3, 3, label=4, instance_id=8)
    m.mark_occupied_voxel(3, 3, 3, label=5, instance_id=9)
    delta = d.finalize()
    assert len(delta.indices) == 1
    assert delta.before['state'][0] == 0
    assert delta.after['label'][0] == 5
    assert delta.after['instance'][0] == 9
    assert delta.after['shadow_instance'][0] == 9
    assert delta.after['shadow'][0] == 2 * m._occupancy.hit_increment
    for array in (delta.indices, delta.before, delta.after):
        with pytest.raises(ValueError):
            array.setflags(write=True)


def test_reset_is_explicit_rebuild_and_primary_lists_unchanged():
    m = mapper()
    m._voxel_delta = VoxelDeltaBuilder(m)
    m.mark_occupied_voxel(1, 1, 1)
    m.reset(preserve_placed=True)
    delta = m._voxel_delta.finalize()
    assert delta.rebuild and len(delta.indices) == 0
    assert isinstance(m._occupancy.values, list)
    assert isinstance(m._occupancy.geometry_locked, list)


def test_net_unchanged_cell_is_removed():
    m = mapper()
    d = VoxelDeltaBuilder(m)
    d.record_before((0, 0))
    assert len(d.finalize().indices) == 0


def test_shadow_change_under_geometry_lock_is_not_lost():
    m = mapper()
    m.mark_placed_box((0, 0, .06), (.08, .08, .08))
    m._voxel_delta = VoxelDeltaBuilder(m)
    m.mark_free_world(.01, .01, .07)
    delta = m._voxel_delta.finalize()
    assert len(delta.indices) == 1
    assert delta.before['state'][0] == delta.after['state'][0] == 2
    assert delta.before['shadow'][0] != delta.after['shadow'][0]
