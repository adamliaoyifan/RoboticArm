"""Dirty-column projection compared against the unchanged full-grid oracle."""
import json
from pathlib import Path

import numpy as np
import pytest

from luggage_perception.cargo_grid_projection import surface_columns
from luggage_perception.cargo_map_query import QueryError, QueryPage
from luggage_perception.cargo_query_indexes import QueryIndexOwner
from luggage_perception.cargo_surface_projection import (
    FIELD_DTYPES, IncrementalSurfaceProjection,
)
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.cargo_voxel_delta import VoxelDeltaBuilder
from luggage_perception.occupancy_map_store import MapUpdate, OccupancyMapStore, UpdateKind
from luggage_description.container_geometry import ContainerGeometry, Chamfer


def mapper(resolution=.02, size=.34):
    return CargoVolumeMapper((size, size, size), (0, 0, size/2), 0, resolution)


def publish(store):
    store._snapshot = store._build_snapshot()


def assert_projection(m, projection):
    expected = surface_columns(m)
    actual = projection.as_dict()
    assert actual == expected


@pytest.mark.parametrize('resolution', [.01, .02, .05])
def test_random_columns_inactive_and_ties_match_full_oracle(resolution):
    m = mapper(resolution, .2)
    rng = np.random.default_rng(156)
    # Oracle fixture includes inactive cells and all state/source combinations.
    for buffer, high, dtype in ((m._grid, 3, 'u1'), (m._source, 3, 'u1'),
                                (m._labels, 5, '<i8'), (m._active, 2, 'u1')):
        view = np.frombuffer(buffer, dtype)
        view[:] = rng.integers(high, size=len(view))
    owner = IncrementalSurfaceProjection()
    owner.initialize(m)
    assert_projection(m, owner.freeze())
    for _ in range(30):
        columns = np.unique(rng.integers(m.nx*m.ny, size=17))
        indices = columns[None, :] + m.nx*m.ny*np.arange(m.nz)[:, None]
        for buffer, high, dtype in ((m._grid, 3, 'u1'), (m._source, 3, 'u1'),
                                    (m._labels, 5, '<i8')):
            view = np.frombuffer(buffer, dtype)
            view[indices] = rng.integers(high, size=indices.shape)
        owner.update_columns(m, columns)
        assert owner.work['columns_recomputed'] == len(columns)
        assert_projection(m, owner.freeze())


@pytest.mark.parametrize('seed', range(5))
def test_net_transactions_shadow_overlap_reset_match_full(seed):
    rng = np.random.default_rng(seed)
    m = mapper(.02, .12)
    owner = QueryIndexOwner()
    owner.initialize(m)
    for transaction in range(100):
        m._voxel_delta = VoxelDeltaBuilder(m)
        action = int(rng.integers(7))
        voxel = rng.integers(6, size=3)
        point = m._local_to_world(*m._voxel_center_local(*voxel))
        try:
            if action < 2:
                m.mark_occupied_voxel(*voxel, label=int(rng.integers(5)), instance_id=7)
            elif action == 2:
                m.mark_free_world(*point)
            elif action == 3:
                m.integrate_points([point], origin=(0, 0, .2), labels=[2], instance_ids=[4])
            elif action == 4:
                m.mark_placed_box((0, 0, .06), (.06, .06, .06))
            elif action == 5:
                m.unmark_placed_box((0, 0, .06), (.06, .06, .06))
            else:
                m.reset(preserve_placed=bool(transaction % 2))
            delta = m._voxel_delta.finalize()
            owner.apply_delta(m, delta)
            assert_projection(m, owner.surface_projection())
        except Exception:
            root = Path('docs/status/evidence/occ_owner_b1/occ_query_index_g2_20261025/s3/failures')
            root.mkdir(parents=True, exist_ok=True)
            (root / ('seed_%d_txn_%d.json' % (seed, transaction))).write_text(json.dumps(dict(
                seed=seed, transaction=transaction, action=action,
                replay='pytest -q src/luggage_perception/test/test_cargo_surface_projection.py -k net_transactions',
                capture_complete=True, replay_possible=True))+'\n')
            raise


def test_dirty_columns_copy_on_write_and_evidence_only_change():
    s = OccupancyMapStore(mapper(.02, .5), epoch='surface')
    owner = s._query_indexes
    old = owner.surface_projection()
    before = owner.surface_work()['total_columns']
    s.mapper.mark_occupied_voxel(1, 2, 3, label=3, instance_id=7)
    s.mapper.mark_occupied_voxel(1, 2, 4, label=4, instance_id=7)
    publish(s)
    new = owner.surface_projection()
    assert owner.surface_work()['total_columns']-before == 1
    assert new.tiles[0] is not old.tiles[0]
    assert all(a is b for a, b in zip(old.tiles[1:], new.tiles[1:]))
    assert old.as_dict()['height'][1][2] == 0
    assert new.as_dict()['height'][1][2] == .1
    for array in new.tiles[0].fields.values():
        with pytest.raises(ValueError):
            array.setflags(write=True)
    before = owner.surface_work()['total_columns']
    s.mapper.mark_occupied_voxel(1, 2, 4, label=4, instance_id=8)
    publish(s)
    assert owner.surface_work()['total_columns'] == before
    assert owner.surface_projection().tiles[0] is new.tiles[0]


def test_surface_full_roi_patch_metadata_and_cursor_contract():
    s = OccupancyMapStore(mapper(), epoch='surface')
    old = s.acquire_query_snapshot()
    s.mapper.mark_occupied_voxel(1, 2, 3, label=4)
    s.mapper.mark_occupied_voxel(2, 3, 4, label=5)
    publish(s)
    with s.acquire_query_snapshot() as current:
        result = current.surface()
        expected = surface_columns(s.mapper)
        assert {name: result.as_dict()[name] for name in FIELD_DTYPES} == expected
        for name in ('surface_z_container', 'height_above_floor', 'contract_confidence'):
            assert result.columns[name] == getattr(s.snapshot(), name)
        assert result.examined_cells == 0
        assert result.shape_xy == (s.mapper.nx, s.mapper.ny)
        first = current.surface_changes(old.identity, page=QueryPage(1))
        assert not first.complete and first.patches[0].ix == 1
        second = current.surface_changes(old.identity, page=QueryPage(1, first.next_cursor))
        assert second.complete and second.patches[0].ix == 2
        assert second.next_cursor == ''
        repeated = current.surface_changes(old.identity, page=QueryPage(1))
        assert repeated.patches == first.patches and repeated.next_cursor == first.next_cursor
        ox, oy, _ = current.grid.origin
        r = current.grid.resolution
        roi = ((ox+r, oy+2*r), (ox+2*r, oy+3*r))
        cropped = current.surface(roi)
        assert cropped.origin_index_xy == (1, 2) and cropped.shape_xy == (1, 1)
        assert cropped.columns['semantic_label'] == ((4,),)
        assert current.surface_changes(old.identity, roi).returned_count == 1
        with pytest.raises(QueryError, match='INVALID_CURSOR'):
            current.surface_changes(old.identity, roi, QueryPage(1, first.next_cursor))
        assert current.surface(((100, 100), (101, 101))).returned_count == 0
        with pytest.raises(QueryError, match='INVALID_REGION'):
            current.surface(((0, 0), (float('nan'), 1)))
        with pytest.raises(QueryError, match='INVALID_FILTER'):
            current.surface_changes(old.identity, page=QueryPage(0))
    # Metadata-only publication changes identity, not column values.
    previous = s.acquire_query_snapshot()
    s.mapper.note_external_change()
    publish(s)
    with s.acquire_query_snapshot() as current:
        assert current.surface_changes(previous.identity).patches == ()
        with pytest.raises(QueryError, match='SNAPSHOT_EXPIRED'):
            current.surface_changes(old.identity)
    old.close()
    previous.close()


def test_large_patch_is_bounded_and_covers_all_columns_in_flat_order():
    s = OccupancyMapStore(mapper(.01, .65), epoch='surface')
    old_identity = s.acquire_query_snapshot().identity
    # Existing public mutation authority, one occupied slice across all columns.
    for y in range(s.mapper.ny):
        for x in range(s.mapper.nx):
            s.mapper.mark_occupied_voxel(x, y, 0)
    publish(s)
    with s.acquire_query_snapshot() as current:
        first = current.surface_changes(old_identity, page=QueryPage(4096))
        assert first.returned_count == 4096 and not first.complete
        second = current.surface_changes(old_identity, page=QueryPage(4096, first.next_cursor))
        assert second.complete
        ids = [p.ix+s.mapper.nx*p.iy for p in first.patches+second.patches]
        assert ids == list(range(s.mapper.nx*s.mapper.ny))


def test_chamfer_and_old_hot_export_use_current_cached_columns(monkeypatch):
    hull = ContainerGeometry(1, 'container_link', .4, .4, 0., .4,
                             Chamfer('positive_y', .1, .2, .1))
    m = CargoVolumeMapper((.4, .4, .4), (0, 0, .2), 0, .02,
                          geometry_descriptor=hull.descriptor())
    s = OccupancyMapStore(m, epoch='surface')
    m.mark_occupied_voxel(3, 4, 2, label=3)
    expected = surface_columns(m)
    import luggage_perception.cargo_volume_mapper as mapper_module
    monkeypatch.setattr(mapper_module, 'surface_columns',
                        lambda *_: pytest.fail('hot full 3D projection'))
    publish(s)
    assert {name: m.surface_map_2d()[name] for name in FIELD_DTYPES} == expected
    with s.acquire_query_snapshot() as snapshot:
        assert {name: snapshot.surface().as_dict()[name] for name in FIELD_DTYPES} == expected
    # Instance-only mutation cannot change the projected column or its tile.
    prior = s.acquire_query_snapshot().identity
    m.mark_occupied_voxel(3, 4, 2, label=3, instance_id=9)
    publish(s)
    with s.acquire_query_snapshot() as snapshot:
        assert snapshot.surface_changes(prior).patches == ()
    assert s.apply_update(MapUpdate(operation_id='reset', kind=UpdateKind.RESET)).accepted
    with s.acquire_query_snapshot() as snapshot:
        with pytest.raises(QueryError, match='SNAPSHOT_IDENTITY_MISMATCH'):
            snapshot.surface_changes(prior)
