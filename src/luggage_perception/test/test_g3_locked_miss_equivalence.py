"""G4-A gen3 I3 fix pins (review_three_issues): the vectorized miss
kernel must reproduce the scalar geometry-lock semantics exactly --
the lock protects the LIVE occupancy, the sensor shadow ledger still
records every miss, and repeated free-space observations can never
erode a placed box to FREE.
"""

import numpy as np
import pytest

from luggage_perception.cargo_volume_mapper import (
    CargoVolumeMapper, SOURCE_GEOMETRY, SOURCE_SENSOR)


def make_mapper():
    return CargoVolumeMapper((0.3, 0.3, 0.3), (0, 0, 0.15), 0, 0.05)


def scalar_miss(mapper, idx):
    """The per-cell semantics under test (integrate_points lines
    682-688): apply_miss + _record_sensor_miss + conditional reset."""
    mapper._grid[idx] = mapper._occupancy.apply_miss(idx)
    mapper._record_sensor_miss(idx)
    if mapper._grid[idx] == 1 and mapper._source[idx] != SOURCE_GEOMETRY:
        mapper._source[idx] = 0
        mapper._labels[idx] = 0
        mapper._instance_ids[idx] = 0


def primary_state(mapper, idx=0):
    return {'logodds': mapper._occupancy.values[idx],
            'geometry_locked': bool(
                mapper._occupancy.geometry_locked[idx]),
            'grid': mapper._grid[idx],
            'sensor_values': mapper._sensor_values[idx],
            'sensor_seen': bool(mapper._sensor_seen[idx])}


def test_locked_single_miss_exact_signature():
    """The saved numerical signature (locked_miss_result.json): one
    miss on a geometry-locked cell must leave the live value at the
    clamp while the shadow records -0.4054651081..."""
    scalar, batch = make_mapper(), make_mapper()
    for mapper in (scalar, batch):
        mapper.mark_occupied_voxel(0, 0, 0, source=SOURCE_GEOMETRY)
    scalar_miss(scalar, 0)
    batch._batch_miss_update({0})
    for field in ('logodds', 'geometry_locked', 'grid',
                  'sensor_values'):
        assert primary_state(scalar)[field] == \
            primary_state(batch)[field], field
    assert primary_state(batch)['logodds'] == 3.5
    assert abs(primary_state(batch)['sensor_values']
               - (-0.4054651081081643)) < 1e-12


def test_ten_repeated_misses_locked_cell_stays_occupied():
    scalar, batch = make_mapper(), make_mapper()
    for mapper in (scalar, batch):
        mapper.mark_occupied_voxel(0, 0, 0, source=SOURCE_GEOMETRY)
    for _ in range(10):
        scalar_miss(scalar, 0)
        batch._batch_miss_update({0})
    for field in ('logodds', 'grid', 'sensor_values'):
        assert primary_state(scalar)[field] == \
            primary_state(batch)[field], field
    assert primary_state(batch)['logodds'] == 3.5
    assert primary_state(batch)['grid'] == 2        # OCCUPIED, not FREE
    assert primary_state(batch)['sensor_values'] == -2.0  # shadow saw all


def test_unlocked_miss_paths_agree():
    """Unlocked cells (sensor-only occupancy) must behave identically:
    value decrement, grid reclassify, shadow record, label reset."""
    scalar, batch = make_mapper(), make_mapper()
    for mapper in (scalar, batch):
        mapper.mark_occupied_voxel(0, 0, 0, source=SOURCE_SENSOR)
    for _ in range(20):
        scalar_miss(scalar, 0)
        batch._batch_miss_update({0})
    for field in ('logodds', 'grid', 'sensor_values', 'sensor_seen'):
        assert primary_state(scalar)[field] == \
            primary_state(batch)[field], field


def test_mixed_locked_and_unlocked_indices():
    """One batch call carrying both locked and unlocked cells: the mask
    must be per-index, not per-call."""
    scalar, batch = make_mapper(), make_mapper()
    for mapper in (scalar, batch):
        mapper.mark_occupied_voxel(0, 0, 0, source=SOURCE_GEOMETRY)
        mapper.mark_occupied_voxel(0, 0, 1, source=SOURCE_SENSOR)
    for _ in range(8):
        for idx in (0, 1):
            scalar_miss(scalar, idx)
        batch._batch_miss_update({0, 1})
    for idx in (0, 1):
        for field in ('logodds', 'grid', 'sensor_values'):
            assert primary_state(scalar, idx)[field] == \
                primary_state(batch, idx)[field], (idx, field)
    assert primary_state(batch, 0)['grid'] == 2   # locked box stays
    assert primary_state(batch, 1)['grid'] == 1   # sensor cell decays


def test_box_removal_restores_sensor_view():
    """The lock protects live occupancy but the SHADOW keeps recording:
    after retracting the geometry, the recorded sensor evidence is the
    true sensor-only trajectory (identical on both paths)."""
    scalar, batch = make_mapper(), make_mapper()
    for mapper in (scalar, batch):
        mapper.mark_occupied_voxel(0, 0, 0, source=SOURCE_GEOMETRY)
    for _ in range(10):
        scalar_miss(scalar, 0)
        batch._batch_miss_update({0})
    # Retract the geometry: unlock + retract occupancy to the shadow.
    for mapper in (scalar, batch):
        mapper._occupancy.geometry_locked[0] = False
        mapper._occupancy.values[0] = mapper._sensor_values[0]
        mapper._grid[0] = mapper._occupancy.state_at(0)
    assert primary_state(scalar) == primary_state(batch)
    assert primary_state(batch)['grid'] == 1       # FREE per sensor


def test_full_integration_paths_equal_with_boxes_and_misses():
    """Store-level scalar-vs-vectorized integration with PLACED boxes
    (the divergence class was box-dependent): the entire primary array
    set must be identical after several outside-crossing views."""
    from luggage_perception.occupancy_map_store import (
        MapUpdate, OccupancyMapStore, UpdateKind)

    def run(vectorized):
        mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                                   0.05, max_raycast_points=2000)
        store = OccupancyMapStore(mapper, epoch='lock')
        for i, (z, y, x) in enumerate(
                (z, y, x) for z in range(2)
                for y in (-.48, .18) for x in (-.5, 0., .5)):
            if i >= 6:
                break
            assert store.apply_update(MapUpdate(
                operation_id='b%d' % i, kind=UpdateKind.ADD_BOX_V1,
                box=((x, y, mapper._hull.floor_z + .11 + .24 * z),
                     (.35, .53, .22), 0.), box_id='box-%d' % i)).accepted
        rng = np.random.default_rng(20261023)
        original = mapper.integrate_points

        def patched(*args, **kwargs):
            return original(*args, **{**kwargs, '_vectorized': vectorized})
        mapper.integrate_points = patched
        for stamp in range(3):
            n = 8000
            inside_n = int(round(n * 0.7))
            inside = rng.uniform([-.6, -.6, .7], [.6, .4, 1.7],
                                 (inside_n, 3))
            external = rng.uniform([1.1, -.6, .7], [1.6, .4, 1.7],
                                   (n - inside_n, 3))
            points = np.concatenate([inside, external])
            rng.shuffle(points)
            store.apply_update(MapUpdate(
                operation_id='v%d' % stamp,
                kind=UpdateKind.INTEGRATE_VIEW,
                expected_snapshot=store.identity,
                geometry_hash=mapper.geometry_hash, points=points,
                origin=(-1.2, 0., 1.1),
                minimum_z=mapper._hull.floor_z + .05,
                stamp_key=(stamp, 0),
                acquisition_stamp_ns=1_000_000_000 + stamp,
                settled=True, schema_version=1))
        return mapper

    a, b = run(True), run(False)
    for field, dtype in (('_grid', 'u1'), ('_active', 'u1'),
                         ('_source', 'u1'), ('_sensor_seen', 'u1')):
        va = np.frombuffer(getattr(a, field), dtype=dtype)
        vb = np.frombuffer(getattr(b, field), dtype=dtype)
        assert np.array_equal(va, vb), field
    assert np.array_equal(np.asarray(a._occupancy.values),
                          np.asarray(b._occupancy.values))
    assert np.array_equal(np.asarray(a._sensor_values),
                          np.asarray(b._sensor_values))
