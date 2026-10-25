"""Independent brute-force voxel-volume oracle and bounded failure contracts."""
from dataclasses import FrozenInstanceError

import numpy as np
import pytest

from luggage_description.container_geometry import ContainerGeometry, Chamfer
from luggage_perception.cargo_distance_query import DistanceBudget
from luggage_perception.cargo_map_query import QueryError
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import MapUpdate, OccupancyMapStore, UpdateKind


def store(resolution=.02, size=.34, hull=None):
    return OccupancyMapStore(CargoVolumeMapper((size, size, size), (0, 0, size/2), 0,
        resolution, geometry_descriptor=hull.descriptor() if hull else None), epoch='distance')


def publish(s):
    s._snapshot = s._build_snapshot()


def brute(m, point, state, radius):
    """Independent whole-grid oracle, no block metadata or query helpers."""
    selected = [i for i in range(len(m._grid)) if m._active[i] and m._grid[i] == state]
    if not selected:
        return 'NO_MATCH', None, None
    distances = []
    origin = (-m.inner_l/2, -m.inner_w/2, m._hull.floor_z)
    for i in selected:
        xyz = (i % m.nx, i//m.nx % m.ny, i//(m.nx*m.ny))
        squared = 0.
        for axis in range(3):
            low = origin[axis]+xyz[axis]*m.resolution
            high = origin[axis]+(xyz[axis]+1)*m.resolution
            delta = max(low-point[axis], point[axis]-high, 0.)
            squared += delta*delta
        distances.append(squared)
    best = min(distances)
    if best > radius*radius:
        return 'RADIUS_CLEAR', None, None
    return 'MATCH', best**.5, selected[distances.index(best)]


@pytest.mark.parametrize('resolution', [.01, .02, .05])
def test_fixed_1000_points_match_independent_aabb_oracle(resolution):
    s = store(resolution, .2)
    rng = np.random.default_rng(617)
    # Random facts are written through existing mapper APIs before publication.
    for _ in range(30):
        voxel = [int(rng.integers(n)) for n in (s.mapper.nx, s.mapper.ny, s.mapper.nz)]
        s.mapper.mark_occupied_voxel(*voxel, label=3, instance_id=7)
    publish(s)
    points = rng.uniform([-.099, -.099, .001], [.099, .099, .199], (1000, 3))
    # Occupied-only workload; unknown is exhaustively covered separately below.
    with s.acquire_query_snapshot() as snapshot:
        for start in range(0, len(points), 250):
            batch = snapshot.nearest_obstacles(points[start:start+250], .2, 'occupied_only')
            assert batch.status == 'OK' and batch.complete
            for point, result in zip(points[start:start+250], batch.results):
                expected = brute(s.mapper, point, 2, .2)
                assert result.unknown is None
                assert result.occupied.status == expected[0]
                if expected[0] == 'MATCH':
                    assert result.occupied.exact
                    assert abs(result.occupied.distance_m-expected[1]) <= 1e-9
                    assert result.occupied.cell.flat_index == expected[2]


def test_unknown_separate_free_empty_and_radius_bounds():
    s = store()
    p = (0., 0., .1)
    with s.acquire_query_snapshot() as snapshot:
        result = snapshot.nearest_obstacles([p], .1).results[0]
        assert result.occupied.status == 'NO_MATCH' and result.occupied.exact
        assert result.unknown.status == 'MATCH' and result.unknown.distance_m == 0.
    # Positive free evidence in every cell: empty obstacle and unknown classes.
    for z in range(s.mapper.nz):
        for y in range(s.mapper.ny):
            for x in range(s.mapper.nx):
                s.mapper.mark_free_world(*s.mapper._local_to_world(
                    *s.mapper._voxel_center_local(x, y, z)))
    s.mapper.mark_occupied_voxel(0, 0, 0)
    # One hit after a miss is UNKNOWN under the unchanged logodds thresholds.
    s.mapper.mark_occupied_voxel(0, 0, 0)
    publish(s)
    with s.acquire_query_snapshot() as snapshot:
        result = snapshot.nearest_obstacles([p], .01).results[0]
        assert result.unknown.status == 'NO_MATCH'
        assert result.occupied.status == 'RADIUS_CLEAR'
        assert not result.occupied.exact and result.occupied.distance_m is None
        assert result.occupied.lower_bound_m == .01
        assert snapshot.nearest_obstacles([p], 0).results[0].occupied.status == 'RADIUS_CLEAR'


@pytest.mark.parametrize('resolution', [.01, .02, .05])
def test_1000_mixed_unknown_and_occupied_points_match_volume_oracle(resolution):
    m = CargoVolumeMapper((.12, .12, .12), (0, 0, .06), 0, resolution)
    rng = np.random.default_rng(125)
    for z in range(m.nz):
        for y in range(m.ny):
            for x in range(m.nx):
                state = int(rng.integers(3))
                if state == 1:
                    m.mark_free_world(*m._local_to_world(*m._voxel_center_local(x, y, z)))
                elif state == 2:
                    m.mark_occupied_voxel(x, y, z)
    s = OccupancyMapStore(m, epoch='distance')
    targets = {}
    for state in (0, 2):
        flat = np.asarray([i for i, v in enumerate(m._grid) if v == state and m._active[i]])
        coordinates = np.asarray([(i % m.nx, i//m.nx % m.ny, i//(m.nx*m.ny)) for i in flat])
        lower = np.asarray([-.06, -.06, 0])+coordinates*resolution
        upper = np.asarray([-.06, -.06, 0])+(coordinates+1)*resolution
        targets[state] = flat, lower, upper
    points = rng.uniform([-.059, -.059, .001], [.059, .059, .119], (1000, 3))
    with s.acquire_query_snapshot() as snapshot:
        for start in range(0, 1000, 250):
            result = snapshot.nearest_obstacles(points[start:start+250], .15)
            assert result.complete
            for point, item in zip(points[start:start+250], result.results):
                for state, match in ((0, item.unknown), (2, item.occupied)):
                    flat, lower, upper = targets[state]
                    delta = np.maximum(np.maximum(lower-point, point-upper), 0.)
                    distances = (delta*delta).sum(axis=1)
                    at = int(np.argmin(distances))
                    assert match.status == 'MATCH' and match.exact
                    assert abs(match.distance_m-distances[at]**.5) <= 1e-9
                    assert match.cell.flat_index == flat[at]


def test_tie_across_block_boundary_and_unique_nearest_deletion():
    s = store(.01)
    m = s.mapper
    m.mark_occupied_voxel(15, 2, 3, label=3)
    m.mark_occupied_voxel(16, 2, 3, label=4)
    m.mark_occupied_voxel(30, 2, 3, label=5)
    publish(s)
    p = (-.17+16*.01, -.17+2.5*.01, 3.5*.01)
    old = s.acquire_query_snapshot()
    result = old.nearest_obstacles([p], .3, 'occupied_only').results[0].occupied
    assert result.distance_m == 0. and result.cell.key.ix == 15
    m.mark_free_world(*m._local_to_world(*m._voxel_center_local(15, 2, 3)))
    m.mark_free_world(*m._local_to_world(*m._voxel_center_local(16, 2, 3)))
    publish(s)
    with s.acquire_query_snapshot() as current:
        new = current.nearest_obstacles([p], .3, 'occupied_only').results[0].occupied
        assert new.cell.key.ix == 30 and abs(new.distance_m-.14) < 1e-12
        assert old.nearest_obstacles([p], .3, 'occupied_only').results[0].occupied.cell.key.ix == 15
    old.close()


def test_mixed_outside_points_and_chamfer_are_named():
    hull = ContainerGeometry(1, 'container_link', .4, .4, 0., .4,
                             Chamfer('positive_y', .05, .2, .2))
    s = store(.02, .4, hull)
    with s.acquire_query_snapshot() as snapshot:
        result = snapshot.nearest_obstacles([(0, 0, .1), (.21, 0, .1), (0, .19, .01)], .3)
        assert [p.status for p in result.results] == ['OK', 'OUTSIDE_MAP', 'OUTSIDE_HULL']
        assert not result.results[1].occupied.exact and not result.results[2].unknown.exact


def test_validation_budget_and_immutability():
    s = store()
    s.mapper.mark_occupied_voxel(1, 1, 1)
    publish(s)
    with s.acquire_query_snapshot() as snapshot:
        for points, radius, policy in (([], .1, 'separate'), ([(0, 0)], .1, 'separate'),
                ([(0, 0, float('nan'))], .1, 'separate'), ([(0, 0, .1)]*257, .1, 'separate'),
                ([(0, 0, .1)], -1., 'separate'), ([(0, 0, .1)], 20., 'separate'),
                ([(0, 0, .1)], .1, 'free')):
            with pytest.raises(QueryError, match='INVALID_FILTER'):
                snapshot.nearest_obstacles(points, radius, policy)
        result = snapshot.nearest_obstacles([(0, 0, .1)], .3,
                                           budget=DistanceBudget(max_examined_cells=0))
        assert not result.complete and result.status == 'QUERY_BUDGET_EXHAUSTED'
        assert result.examined_cells == 0
        assert not result.results[0].occupied.exact
        assert result.results[0].occupied.distance_m is None
        with pytest.raises(FrozenInstanceError):
            result.results[0].occupied.exact = True
        with pytest.raises(QueryError, match='INVALID_FILTER'):
            snapshot.nearest_obstacles([(0, 0, .1)], budget=DistanceBudget(timeout_ms=201))
        for array in (snapshot._data().distance_index.lower, snapshot._data().distance_index.upper):
            with pytest.raises(ValueError):
                array.setflags(write=True)


def test_geometry_removal_uses_restored_sensor_shadow():
    s = store()
    box = ((0, 0, .1), (.1, .1, .1), 0.)
    assert s.apply_update(MapUpdate(operation_id='add', kind=UpdateKind.ADD_BOX_V1,
                                   box=box, box_id='actual-box')).accepted
    m = s.mapper
    m.mark_occupied_voxel(8, 8, 4, label=7, instance_id=31)
    publish(s)
    assert s.apply_update(MapUpdate(operation_id='remove', kind=UpdateKind.REMOVE_BOX_V1,
                                   box=box)).accepted
    p = m._local_to_world(*m._voxel_center_local(8, 8, 4))
    with s.acquire_query_snapshot() as snapshot:
        result = snapshot.nearest_obstacles([p], .1, 'occupied_only').results[0].occupied
        assert result.status == 'MATCH' and result.cell.instance_id == 31
        assert result.cell.geometry_membership == ()


def test_containing_unknown_fast_path_and_closed_plane_ties():
    s = store(.01)
    with s.acquire_query_snapshot() as snapshot:
        # Sparse occupied maps must not scan 4096 unknown cells for each
        # point already inside a known UNKNOWN cell volume.
        result = snapshot.nearest_obstacles([(0., 0., .1)]*256, .3)
        assert result.complete and result.examined_cells <= 256*8
    s.mapper.mark_occupied_voxel(15, 2, 3)
    publish(s)
    boundary = -.17+16*.01
    coordinates = [boundary, np.nextafter(boundary, -np.inf), np.nextafter(boundary, np.inf)]
    with s.acquire_query_snapshot() as snapshot:
        for x in coordinates:
            point = (x, -.17+2.5*.01, 3.5*.01)
            result = snapshot.nearest_obstacles([point], .3).results[0]
            for state, match in ((0, result.unknown), (2, result.occupied)):
                expected = brute(s.mapper, point, state, .3)
                assert match.status == expected[0]
                assert abs(match.distance_m-expected[1]) <= 1e-9
                assert match.cell.flat_index == expected[2]


def test_sparse_batch_decodes_each_block_once_without_changing_work_cap():
    s = store(.01, .30)
    # Sparse targets distributed across all eight blocks. Repeated membership
    # scans previously exhausted the default cap before completing 250 points.
    for z in (4, 24):
        for y in (4, 24):
            for x in (4, 24):
                s.mapper.mark_occupied_voxel(x, y, z)
    publish(s)
    points = np.random.default_rng(2).uniform([-.149, -.149, .001], [.149, .149, .299], (250, 3))
    with s.acquire_query_snapshot() as snapshot:
        result = snapshot.nearest_obstacles(points, .3)
        assert result.complete
        assert result.examined_cells < 50_000
        for point, item in zip(points, result.results):
            expected = brute(s.mapper, point, 2, .3)
            assert abs(item.occupied.distance_m-expected[1]) <= 1e-9
            assert item.occupied.cell.flat_index == expected[2]
        bounded = snapshot.nearest_obstacles(points, .3,
            budget=DistanceBudget(max_examined_cells=10))
        assert not bounded.complete and bounded.examined_cells <= 10
