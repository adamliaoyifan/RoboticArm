"""Independent region/instance/pagination and snapshot-lifecycle oracles."""
from dataclasses import FrozenInstanceError
import threading

import numpy as np
import pytest

from luggage_perception.cargo_map_query import (
    CellKey, InstanceKey, QueryError, QueryPage,
)
from luggage_perception.cargo_map_query_adapter import QuerySnapshotRegistry
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import MapUpdate, OccupancyMapStore, UpdateKind


def store(resolution=.02, size=.34):
    return OccupancyMapStore(CargoVolumeMapper((size, size, size), (0, 0, size/2),
                                              0, resolution), epoch='epoch')


def publish_direct(s):
    s._snapshot = s._build_snapshot()


def concat(call, limit=113):
    cursor = ''
    result = []
    while True:
        page = call(QueryPage(limit, cursor))
        repeated = call(QueryPage(limit, cursor))
        assert repeated.cells == page.cells
        assert repeated.next_cursor == page.next_cursor
        assert repeated.complete == page.complete
        result.extend(c.flat_index for c in page.cells)
        if page.complete:
            assert page.next_cursor == ''
            return result
        assert page.next_cursor
        cursor = page.next_cursor


def scalar_region_oracle(m, region, states=(0, 1, 2), instance=None):
    low, high = region
    result = []
    origin = (-m.inner_l/2, -m.inner_w/2, m._hull.floor_z)
    for z in range(m.nz):
        for y in range(m.ny):
            for x in range(m.nx):
                index = m._index(x, y, z)
                if not m._active[index] or m._grid[index] not in states:
                    continue
                if instance is not None and m._instance_ids[index] != instance:
                    continue
                p = (x, y, z)
                if all(origin[a]+(p[a]+1)*m.resolution > low[a]
                       and origin[a]+p[a]*m.resolution < high[a] for a in range(3)):
                    result.append(index)
    return result


@pytest.mark.parametrize('resolution', [.01, .02, .05])
def test_random_roi_pagination_matches_scalar(resolution):
    s = store(resolution)
    m = s.mapper
    rng = np.random.default_rng(20261025)
    for _ in range(70):
        voxel = [int(rng.integers(n)) for n in (m.nx, m.ny, m.nz)]
        m.mark_occupied_voxel(*voxel, label=3, instance_id=17)
    for _ in range(30):
        voxel = [int(rng.integers(n)) for n in (m.nx, m.ny, m.nz)]
        point = m._local_to_world(*m._voxel_center_local(*voxel))
        m.mark_free_world(*point)
    publish_direct(s)
    with s.acquire_query_snapshot() as snapshot:
        for _ in range(10):
            low = np.asarray([-.17, -.17, 0]) + rng.uniform(0, .2, 3)
            high = low + rng.uniform(.02, .16, 3)
            region = (tuple(low), tuple(high))
            expected = scalar_region_oracle(m, region, states=(0, 2))
            actual = concat(lambda page: snapshot.query_cells(region, (0, 2), page=page), 31)
            assert actual == expected
        all_frontier = sorted(m._index(*p) for p in m._frontier_indices_full())
        assert concat(lambda page: snapshot.query_frontiers(page=page), 37) == all_frontier
        assert snapshot.count_frontiers().count == len(all_frontier)
        region = ((-.11, -.12, .08), (.13, .11, .25))
        frontier_set = set(all_frontier)
        expected_count = sum(i in frontier_set for i in scalar_region_oracle(m, region))
        assert snapshot.count_frontiers(region).count == expected_count


def test_instance_reads_only_its_blocks_and_geometry_overlap_restores():
    s = store(.01, .5)
    m = s.mapper
    m.mark_occupied_voxel(1, 1, 1, label=3, instance_id=17)
    m.mark_occupied_voxel(40, 40, 40, label=4, instance_id=18)
    publish_direct(s)
    with s.acquire_query_snapshot() as snapshot:
        result = snapshot.query_instance(InstanceKey('sensor_instance', 17, 'legacy:epoch'))
        assert result.examined_blocks == 1
        assert len(result.cells) == 1 and result.cells[0].instance_id == 17
        with pytest.raises(QueryError, match='INSTANCE_BINDING_MISSING'):
            snapshot.query_instance(InstanceKey('sensor_instance', 17))
        with pytest.raises(QueryError, match='INVALID_FILTER'):
            snapshot.query_instance(InstanceKey('sensor_instance', 0, 'legacy:epoch'))
    boxes = [((-.03, 0, .12), (.12, .12, .12), 0.),
             ((.04, 0, .12), (.12, .12, .12), 0.)]
    for i, box in enumerate(boxes):
        assert s.apply_update(MapUpdate(operation_id='add'+str(i), kind=UpdateKind.ADD_BOX_V1,
                                       box=box, box_id='case'+str(i))).accepted
    with s.acquire_query_snapshot() as snapshot:
        for i, box in enumerate(boxes):
            expected = sorted(m._iter_placed_box_voxels(*box))
            actual = concat(lambda page: snapshot.query_instance(
                InstanceKey('committed_box', 'case'+str(i)), page=page))
            assert actual == expected
        overlap = snapshot.query_instance(InstanceKey('committed_box', 'case1'), page=QueryPage(4096))
        assert any(c.geometry_membership == ('case0', 'case1') for c in overlap.cells)
    assert s.apply_update(MapUpdate(operation_id='remove0', kind=UpdateKind.REMOVE_BOX_V1,
                                   box=boxes[0])).accepted
    with s.acquire_query_snapshot() as snapshot:
        assert snapshot.query_instance(InstanceKey('committed_box', 'case0')).cells == ()
        assert concat(lambda p: snapshot.query_instance(InstanceKey('committed_box', 'case1'), page=p)) == sorted(m._iter_placed_box_voxels(*boxes[1]))


def test_missing_and_ambiguous_box_identity_do_not_change_acceptance():
    s = store(.02, .5)
    box = ((0, 0, .12), (.12, .12, .12), 0.)
    assert s.apply_update(MapUpdate(operation_id='legacy', kind=UpdateKind.ADD_BOX_V1, box=box)).accepted
    with s.acquire_query_snapshot() as snapshot:
        with pytest.raises(QueryError, match='INSTANCE_BINDING_MISSING'):
            snapshot.query_instance(InstanceKey('committed_box', '0'))
    for i, x in enumerate((-.12, .12)):
        update = MapUpdate(operation_id='real'+str(i), kind=UpdateKind.ADD_BOX_V1,
                           box=((x, 0, .3), (.1, .1, .1), 0.), box_id='ambiguous')
        assert s.apply_update(update).accepted
    with s.acquire_query_snapshot() as snapshot:
        with pytest.raises(QueryError, match='INSTANCE_BINDING_MISSING'):
            snapshot.query_instance(InstanceKey('committed_box', 'ambiguous'))


def test_old_snapshot_cursor_and_new_identity_are_separate():
    s = store()
    old = s.acquire_query_snapshot()
    page = old.query_cells(page=QueryPage(5))
    before = old.summary().as_dict()
    s.mapper.mark_occupied_voxel(3, 3, 3)
    s.mapper.note_external_change()
    publish_direct(s)
    with s.acquire_query_snapshot() as new:
        assert new.identity != old.identity
        assert old.summary().as_dict() == before
        assert old.query_cells(page=QueryPage(5, page.next_cursor)).cells[0].flat_index == 5
        with pytest.raises(QueryError, match='INVALID_CURSOR'):
            new.query_cells(page=QueryPage(5, page.next_cursor))
        with pytest.raises(QueryError, match='SNAPSHOT_IDENTITY_MISMATCH'):
            s.acquire_query_snapshot(old.identity)
    s.mapper.note_external_change()
    publish_direct(s)
    with pytest.raises(QueryError, match='SNAPSHOT_EXPIRED'):
        old.summary()


def test_expiry_reset_close_and_zero_scan_summary():
    s = store()
    data = s.acquire_query_snapshot()._data()
    now = [1]
    registry = QuerySnapshotRegistry(clock_ns=lambda: now[0])
    registry.publish(data)
    old = registry.open()
    assert old.summary().examined_cells == 0
    assert old.count_frontiers().examined_cells == 0
    now[0] += 5_000_000_000
    with pytest.raises(QueryError, match='SNAPSHOT_EXPIRED'):
        old.summary()
    old = s.acquire_query_snapshot()
    s.apply_update(MapUpdate(operation_id='reset', kind=UpdateKind.RESET))
    with pytest.raises(QueryError, match='SNAPSHOT_EXPIRED'):
        old.get_cell(CellKey('epoch', 0, 0, 0))
    handle = s.acquire_query_snapshot()
    handle.close()
    with pytest.raises(QueryError, match='SNAPSHOT_EXPIRED'):
        handle.summary()


def test_bad_query_fields_and_deep_immutability():
    s = store()
    with s.acquire_query_snapshot() as snapshot:
        with pytest.raises(FrozenInstanceError):
            snapshot.identity = None
        page = snapshot.query_cells(page=QueryPage(2))
        with pytest.raises(FrozenInstanceError):
            page.cells[0].state = 2
        data = snapshot._data()
        block = data.blocks[0]
        for array in (block.flat_indices, block.records()):
            with pytest.raises(ValueError):
                array.setflags(write=True)
        with pytest.raises(TypeError):
            data.sensor_blocks[1] = ()
        for region in (((0, 0, 0), (0, 1, 1)), ((0, 0, 0), (1, float('nan'), 1)), [1]):
            with pytest.raises(QueryError, match='INVALID_REGION'):
                snapshot.query_cells(region=region)
        for states in ((), (4,), (True,), ('occupied',)):
            with pytest.raises(QueryError, match='INVALID_FILTER'):
                snapshot.query_cells(states=states)
        for limit in (0, 4097, True):
            with pytest.raises(QueryError, match='INVALID_FILTER'):
                snapshot.query_cells(page=QueryPage(limit))
        with pytest.raises(QueryError, match='INVALID_CURSOR'):
            snapshot.query_cells(page=QueryPage(5, page.next_cursor[:-1]+'!'))
        with pytest.raises(QueryError, match='INVALID_CURSOR'):
            snapshot.query_cells(states=(2,), page=QueryPage(2, page.next_cursor))
        with pytest.raises(QueryError, match='OUTSIDE_MAP'):
            snapshot.get_cell(CellKey('epoch', 10000, 0, 0))


def test_block_seams_cow_and_full_fact_digest():
    s = store(.01, .34)
    before = s.acquire_query_snapshot()._data()
    for coordinate in (15, 16, 17):
        s.mapper.mark_occupied_voxel(coordinate, 16, 16, label=2, instance_id=9)
    publish_direct(s)
    with s.acquire_query_snapshot() as snapshot:
        data = snapshot._data()
        assert data.blocks[-1] is before.blocks[-1]
        for coordinate in (15, 16, 17):
            cell = snapshot.get_cell(CellKey('epoch', coordinate, 16, 16))
            assert cell.state == 2 and cell.instance_id == 9
        old_content = snapshot.identity.content_digest
        old_voxel = snapshot.identity.voxel_digest
    s.mapper.mark_occupied_voxel(16, 16, 16, label=3, instance_id=10)
    publish_direct(s)
    with s.acquire_query_snapshot() as snapshot:
        assert snapshot.identity.content_digest == old_content
        assert snapshot.identity.voxel_digest != old_voxel
        page = snapshot.query_instance(InstanceKey('sensor_instance', 10, 'legacy:epoch'))
        assert [c.flat_index for c in page.cells] == [s.mapper._index(16, 16, 16)]
    # Snapshot block evidence and exported arrays remain unchanged after COW.
    assert before.blocks[0].records()['state'].sum() == 0


def test_sensor_instance_index_follows_latest_shadow_after_removal():
    s = store(.02, .34)
    m = s.mapper
    box = ((0, 0, .12), (.12, .12, .12), 0.)
    assert s.apply_update(MapUpdate(operation_id='add', kind=UpdateKind.ADD_BOX_V1,
                                   box=box, box_id='case')).accepted
    m.mark_occupied_voxel(8, 8, 5, label=3, instance_id=19)
    point = m._local_to_world(*m._voxel_center_local(8, 8, 5))
    for _ in range(8):
        m.mark_free_world(*point)
    publish_direct(s)
    with s.acquire_query_snapshot() as snapshot:
        assert snapshot.query_instance(InstanceKey('sensor_instance', 19, 'legacy:epoch')).returned_count == 1
    assert s.apply_update(MapUpdate(operation_id='remove', kind=UpdateKind.REMOVE_BOX_V1, box=box)).accepted
    with s.acquire_query_snapshot() as snapshot:
        assert snapshot.query_instance(InstanceKey('sensor_instance', 19, 'legacy:epoch')).cells == ()
        cell = snapshot.get_cell(CellKey('epoch', 8, 8, 5))
        assert cell.state == 1 and cell.instance_id == 0


def test_session_and_memory_limits_are_explicit(monkeypatch):
    import luggage_perception.cargo_map_query_adapter as adapter
    s = store()
    data = s.acquire_query_snapshot()._data()
    registry = QuerySnapshotRegistry()
    registry.publish(data)
    handles = [registry.open() for _ in range(adapter.MAX_SESSIONS)]
    with pytest.raises(QueryError, match='RESOURCE_LIMIT'):
        registry.open()
    handles[0].close()
    registry.open().close()
    monkeypatch.setattr(adapter, 'MAX_RETAINED_BYTES', 1)
    registry.publish(data)
    with pytest.raises(QueryError, match='RESOURCE_LIMIT'):
        registry.open()
    # Resource refusal rejects new sessions; it does not erase an admitted
    # pinned result to make the memory check appear to pass.
    assert handles[1].summary().snapshot_identity == data.identity


def test_inactive_hole_keeps_legacy_frontier_neighbor_predicate():
    m = CargoVolumeMapper((.08, .08, .08), (0, 0, .04), 0, .02,
                         hull_local_inside=lambda x, y, z: not (x > 0 and y > 0 and z > 0))
    s = OccupancyMapStore(m, epoch='epoch')
    # Legacy exact voxel mutator admits inactive cells. They remain excluded
    # from returned cells but their observed state still affects neighbors.
    m.mark_occupied_voxel(2, 2, 2)
    publish_direct(s)
    with s.acquire_query_snapshot() as snapshot:
        with pytest.raises(QueryError, match='OUTSIDE_HULL'):
            snapshot.get_cell(CellKey('epoch', 2, 2, 2))
        assert concat(lambda p: snapshot.query_frontiers(page=p)) == sorted(m._index(*p) for p in m._frontier_indices_full())


@pytest.mark.parametrize('repeat', range(3))
def test_two_readers_one_writer_1000_transactions(repeat):
    s = store(.02, .12)
    done = threading.Event()
    errors = []
    accepted = {}
    observations = []
    def writer():
        try:
            for transaction in range(1000):
                point = (.01, .01, .07 if transaction % 2 else .03)
                result = s.apply_update(MapUpdate(operation_id=str(transaction), kind=UpdateKind.INTEGRATE_VIEW,
                    points=(point,), settled=True, schema_version=1, origin=(0, 0, .15),
                    geometry_hash=s.mapper.geometry_hash,
                    stamp_key=(repeat, transaction), acquisition_stamp_ns=transaction+1))
                assert result.accepted
                accepted[result.identity] = (result.snapshot.placed_count, result.snapshot)
        except BaseException as error:
            errors.append(error)
        finally:
            done.set()
    def reader():
        while not done.is_set():
            try:
                with s.acquire_query_snapshot() as snapshot:
                    page = snapshot.query_cells(page=QueryPage(4096))
                    states = np.asarray([c.state for c in page.cells])
                    summary = snapshot.summary()
                    assert tuple(summary.snapshot_identity) == tuple(page.snapshot_identity)
                    assert [int((states == v).sum()) for v in (0, 1, 2)] == [summary.values[k] for k in ('unknown_count', 'free_count', 'occupied_count')]
                    observations.append(page.snapshot_identity)
            except QueryError as error:
                if error.reason != 'SNAPSHOT_EXPIRED':
                    errors.append(error)
            except BaseException as error:
                errors.append(error)
    threads = [threading.Thread(target=reader), threading.Thread(target=reader), threading.Thread(target=writer)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=30)
    assert all(not t.is_alive() for t in threads)
    assert not errors
    assert len(accepted) == 1000
    assert observations
    for identity in observations:
        if identity.revision != 1:
            from luggage_perception.occupancy_map_store import SnapshotIdentity
            assert SnapshotIdentity(*identity[:3]) in accepted
