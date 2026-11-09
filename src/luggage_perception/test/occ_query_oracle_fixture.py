"""Deterministic, process-isolated correctness workload; no ROS graph.

Small grids exercise every transaction at three physical resolutions. This
does not replace the separate deployment-size boundary/resource campaigns.
Imports are pinned only to --root, never mixed baseline/candidate modules.
"""
import argparse
from dataclasses import asdict
import hashlib
import itertools
import json
from pathlib import Path
import random
import sys


def canonical_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=True).encode()).hexdigest()


def normalize_epoch(value):
    if isinstance(value, dict):
        return {k: ('normalized-reset-epoch' if k == 'epoch' else normalize_epoch(v))
                for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [normalize_epoch(v) for v in value]
    return value


def primary_arrays(mapper):
    import numpy as np
    arrays = {name: np.frombuffer(getattr(mapper, name), dtype=dtype).copy()
        for name, dtype in (('_grid', 'u1'), ('_active', 'u1'), ('_source', 'u1'),
            ('_labels', '<i8'), ('_instance_ids', '<i8'), ('_sensor_seen', 'u1'),
            ('_sensor_values', '<f8'), ('_sensor_labels', '<i8'), ('_sensor_instance_ids', '<i8'))}
    arrays['logodds'] = np.asarray(mapper._occupancy.values, dtype='<f8')
    arrays['geometry_locked'] = np.asarray(mapper._occupancy.geometry_locked, dtype='u1')
    return arrays


def check_indexes(store, voxel, full):
    import numpy as np
    from luggage_perception.cargo_grid_projection import occupancy_counts, surface_columns
    from luggage_perception.cargo_map_query import CellKey, InstanceKey, QueryError, QueryPage
    m = store.mapper
    active, inactive, unknown, free, occupied, labels = occupancy_counts(m)
    with store.acquire_query_snapshot() as query:
        summary = query.summary()
        assert [summary.values[k] for k in ('total_voxels', 'inactive_count',
            'unknown_count', 'free_count', 'occupied_count')] == [active, inactive, unknown, free, occupied]
        assert summary.values['label_distribution'] == {str(k): v for k, v in labels.items()}
        grid = np.frombuffer(m._grid, 'u1').reshape((m.nz, m.ny, m.nx))
        observed = (grid == 1) | (grid == 2)
        neighbors = np.zeros(grid.shape, dtype=bool)
        for axis in range(3):
            a, b = [slice(None)]*3, [slice(None)]*3
            a[axis], b[axis] = slice(1, None), slice(None, -1)
            neighbors[tuple(a)] |= observed[tuple(b)]
            neighbors[tuple(b)] |= observed[tuple(a)]
        allowed = np.frombuffer(m._active, 'u1').reshape(grid.shape).astype(bool)
        expected_frontiers = np.flatnonzero((grid == 0) & allowed & neighbors).tolist()
        if full:
            assert expected_frontiers == sorted(m._index(*p) for p in m._frontier_indices_full())
        assert query.count_frontiers().count == len(expected_frontiers)
        actual = []
        cursor = ''
        while True:
            result = query.query_frontiers(page=QueryPage(4096, cursor))
            actual.extend(c.flat_index for c in result.cells)
            if result.complete:
                break
            cursor = result.next_cursor
        assert actual == expected_frontiers
        surface = query.surface().as_dict()
        assert {name: surface[name] for name in surface_columns(m)} == surface_columns(m)
        if not full:
            if m._active[m._index(*voxel)]:
                cells = [query.get_cell(CellKey(query.identity.epoch, *voxel))]
            else:
                try:
                    query.get_cell(CellKey(query.identity.epoch, *voxel))
                except QueryError as error:
                    assert error.reason == 'OUTSIDE_HULL'
                else:
                    raise AssertionError('inactive point returned a cell')
                cells = []
        else:
            cells, cursor = [], ''
            while True:
                result = query.query_cells(page=QueryPage(4096, cursor))
                cells.extend(result.cells)
                if result.complete:
                    break
                cursor = result.next_cursor
            assert [c.flat_index for c in cells] == np.flatnonzero(allowed.ravel()).tolist()
        for c in cells:
            i = c.flat_index
            assert (c.state, c.source, c.label, c.instance_id) == (
                m._grid[i], m._source[i], m._labels[i], m._instance_ids[i])
        if not full:
            return
        expected = np.flatnonzero(allowed.ravel()).tolist()
        ids = {m._instance_ids[i] for i in expected if m._grid[i] == 2 and m._instance_ids[i] > 0}
        for identifier in ids:
            result = query.query_instance(InstanceKey('sensor_instance', identifier,
                'legacy:'+query.identity.epoch), page=QueryPage(4096))
            wanted = [i for i in expected if m._grid[i] == 2 and m._instance_ids[i] == identifier]
            assert result.complete and [c.flat_index for c in result.cells] == wanted


def check_distances(store, seed, count, out, case):
    """Whole-map independent volume oracle, never using query block bounds."""
    import numpy as np
    from luggage_description.container_geometry import contains_point
    m = store.mapper
    rng = np.random.default_rng(seed)
    points = []
    while len(points) < count:
        point = tuple(rng.uniform([-.15, -.15, 0.], [.15, .15, .3]))
        if contains_point(m._hull, point):
            points.append(point)
    origin = np.asarray([-.15, -.15, 0.])
    targets = {}
    active = np.frombuffer(m._active, 'u1').astype(bool)
    grid = np.frombuffer(m._grid, 'u1')
    for state in (0, 2):
        flat = np.flatnonzero(active & (grid == state))
        xyz = np.column_stack((flat % m.nx, flat//m.nx % m.ny, flat//(m.nx*m.ny)))
        targets[state] = flat, origin+xyz*m.resolution, origin+(xyz+1)*m.resolution
    batch = None
    try:
        for start in range(0, count, 250):
            # Independent bounded requests; oracle CPU work happens after
            # close and cannot consume/renew the next request's token TTL.
            with store.acquire_query_snapshot() as query:
                batch = query.nearest_obstacles(points[start:start+250], .3)
            assert batch.complete, 'distance query exhausted default budget'
            for point, result in zip(points[start:start+250], batch.results):
                assert result.status == 'OK'
                for state, match in ((0, result.unknown), (2, result.occupied)):
                    flat, lower, upper = targets[state]
                    if not len(flat):
                        assert match.status == 'NO_MATCH' and match.exact
                        continue
                    delta = np.maximum(np.maximum(lower-point, point-upper), 0.)
                    squared = np.sum(delta*delta, axis=1)
                    at = int(np.argmin(squared))
                    if squared[at] > .3**2:
                        assert match.status == 'RADIUS_CLEAR' and not match.exact
                        assert match.distance_m is None and match.lower_bound_m == .3
                    else:
                        assert match.status == 'MATCH' and match.exact
                        assert abs(match.distance_m-squared[at]**.5) <= 1e-9
                        assert match.cell.flat_index == int(flat[at])
    except Exception as error:
        failure = out/'distance_failure';failure.mkdir(exist_ok=True)
        np.savez_compressed(failure/'map.npz', **primary_arrays(m))
        np.save(failure/'points.npy', np.asarray(points))
        (failure/'case.json').write_text(json.dumps(dict(case, error=repr(error),
            response=None if batch is None else asdict(batch), capture_complete=True,
            replay_possible=True), indent=2, default=str)+'\n')
        raise


def run(args):
    import numpy as np
    from luggage_description.container_geometry import ContainerGeometry, Chamfer
    from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
    from luggage_perception.occupancy_map_store import MapUpdate, OccupancyMapStore, UpdateKind
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    manifest = dict(profile='small-grid exhaustive transactions', root=args.root,
        seed_start=args.seed_start, seeds=args.seeds, transactions=args.transactions,
        resolutions=args.resolutions, boxes=args.boxes, shapes=args.shapes,
        check_indexes=args.check_indexes, inner_size=[.30, .30, .30],
        distance_points_per_initial_and_final_map=args.distance_points,
        full_cell_oracle='initial/each 50th/end boundary; facts/counts/frontier/surface every transaction',
        deployment_boundaries='not_evaluated', complete=False)
    manifest['loaded_mapper'] = sys.modules[CargoVolumeMapper.__module__].__file__
    manifest['loaded_store'] = sys.modules[OccupancyMapStore.__module__].__file__
    if any(not Path(manifest[k]).resolve().is_relative_to(Path(args.root).resolve())
           for k in ('loaded_mapper', 'loaded_store')):
        raise RuntimeError('mixed product module roots')
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    with (out/'rows.jsonl').open('w') as rows, (out/'inputs.jsonl').open('w') as journal:
        for r, box_count, shape in itertools.product(args.resolutions, args.boxes, args.shapes):
            for seed in range(args.seed_start, args.seed_start+args.seeds):
                rng = random.Random(seed)
                hull = ContainerGeometry(1, 'container_link', .3, .3, 0., .3,
                    Chamfer('positive_y', .05, .15, .10) if shape == 'chamfer' else None)
                m = CargoVolumeMapper((.3, .3, .3), (0, 0, .15), 0, r,
                    geometry_descriptor=hull.descriptor(), max_raycast_points=4)
                clock = [1_000_000_000]
                s = OccupancyMapStore(m, epoch='matrix', clock_ns=lambda: clock[0],
                                      require_confirmation=True)
                epoch_aliases = {s.epoch: 'epoch_0'}
                xy = [(-.15+(int((value+.15)/r)+.5)*r) for value in (-.125, -.025, .075)]
                zz = [(int(value/r)+.5)*r for value in (.075, .175, .275)]
                boxes = [((x, y, z), (.8*r, .8*r, .8*r), 0.) for z in zz for y in xy for x in xy]
                for i, box in enumerate(boxes[:box_count]):
                    result = s.apply_update(MapUpdate(operation_id='initial-'+str(i),
                        kind=UpdateKind.ADD_BOX_V1, box=box, box_id='fixture-box-'+str(i)))
                    assert result.accepted
                if len(m._placed_boxes) != box_count:
                    (out/'initialization_failure.json').write_text(json.dumps(dict(
                        resolution=r, shape=shape, seed=seed, requested=box_count,
                        actual=len(m._placed_boxes), boxes=boxes[:box_count],
                        capture_complete=True, replay_possible=True), indent=2)+'\n')
                    raise AssertionError('actual initial box count differs from workload')
                if args.check_indexes and args.distance_points:
                    check_distances(s, seed, args.distance_points, out, dict(
                        resolution=r, initial_boxes=box_count, shape=shape, seed=seed, stage='initial'))
                previous = None
                for transaction in range(args.transactions):
                    clock[0] += 1
                    action = transaction % 16 if transaction < 32 else rng.randrange(16)
                    voxel = tuple(rng.randrange(n) for n in (m.nx, m.ny, m.nz))
                    point = m._local_to_world(*m._voxel_center_local(*voxel))
                    slot = rng.randrange(len(boxes))
                    case = dict(resolution=r, initial_boxes=box_count, shape=shape, seed=seed,
                                transaction=transaction, action=action, voxel=voxel, slot=slot)
                    before = primary_arrays(m)
                    if args.capture_final and transaction == args.transactions-1:
                        name = 'r%s_b%d_%s_s%d' % (str(r).replace('.', 'p'), box_count, shape, seed)
                        np.savez_compressed(out/(name+'_before_last.npz'), **before)
                        (out/(name+'_before_last_exports.json')).write_text(json.dumps(dict(
                            mapper=m.surface_map_2d(), v1=s.v1_surface_dict(),
                            v2=s.v2_record('fixture-scene', 'fixture-request')), sort_keys=True)+'\n')
                    update = None
                    try:
                        if action in (0, 1, 2):
                            update = MapUpdate(operation_id=str(transaction), kind=UpdateKind.INTEGRATE_VIEW,
                                points=(point,), origin=(0., 0., .4), schema_version=1,
                                settled=True, geometry_hash=m.geometry_hash, stamp_key=(seed, transaction),
                                acquisition_stamp_ns=clock[0], pose_at_confirm={'position': [0., 0., 0.]})
                        elif action == 3:
                            update = MapUpdate(operation_id=str(transaction), kind=UpdateKind.ADD_BOX_V1,
                                box=boxes[slot], box_id='fixture-box-'+str(slot))
                        elif action == 4:
                            update = MapUpdate(operation_id=str(transaction), kind=UpdateKind.REMOVE_BOX_V1,
                                               box=boxes[slot])
                        elif action == 5 and previous is not None:
                            update = previous
                        elif action == 6:
                            update = MapUpdate(operation_id=str(transaction), kind=UpdateKind.RESET)
                        elif action == 7:
                            update = MapUpdate(operation_id='', kind=UpdateKind.RESET)
                        elif action == 8:
                            update = MapUpdate(operation_id=str(transaction), kind=UpdateKind.NOTE_EVENT,
                                event='box_released', box_id='fixture-box-'+str(slot), event_stamp_ns=clock[0])
                        elif action == 9:
                            update = MapUpdate(operation_id=str(transaction), kind=UpdateKind.ADD_BOX_V2,
                                box=boxes[slot], box_id='fixture-box-'+str(slot), geometry_hash=m.geometry_hash,
                                expected_snapshot=s.identity, measured_source='verify_placed_box', verified_center=boxes[slot][0],
                                verify_tolerance_m=.1*r, verify_source='oracle_fixture')
                        elif action == 12:
                            m.reset(preserve_placed=True)
                            s._snapshot = s._build_snapshot()
                        elif action == 13:
                            center, size, yaw = boxes[slot]
                            overlap = ((center[0]+.1*r, center[1], center[2]), size, yaw)
                            update = MapUpdate(operation_id=str(transaction), kind=UpdateKind.ADD_BOX_V1,
                                box=overlap, box_id='overlap-'+str(slot))
                        elif action == 14:
                            update = MapUpdate(operation_id=str(transaction), kind=UpdateKind.ADD_BOX_V2,
                                expected_snapshot=s.identity._replace(revision=s.map_revision-1),
                                box=boxes[slot], geometry_hash=m.geometry_hash)
                        elif action == 15 and previous is not None:
                            update = MapUpdate(operation_id=previous.operation_id, kind=UpdateKind.NOTE_EVENT,
                                event='unobserved_change', event_stamp_ns=clock[0])
                        else:
                            # Internal mapper-boundary fixture, not a new public writer.
                            m.mark_occupied_voxel(*voxel, label=1+slot % 4, instance_id=1+slot % 5)
                            if action == 11:
                                m.mark_free_world(*point)
                            s._snapshot = s._build_snapshot()
                        payload = None if update is None else asdict(update)
                        if payload is not None and update.expected_snapshot is not None:
                            payload['expected_snapshot'] = dict(epoch=epoch_aliases.get(
                                update.expected_snapshot.epoch, 'foreign_epoch'),
                                revision=update.expected_snapshot.revision,
                                content_digest=update.expected_snapshot.content_digest)
                        journal.write(json.dumps(dict(case, request=payload), sort_keys=True)+'\n')
                        if update is not None:
                            result = s.apply_update(update)
                            previous = update
                            verdict = (result.status.value, result.reason)
                        else:
                            verdict = ('internal_boundary_fixture', '')
                        if s.epoch not in epoch_aliases:
                            epoch_aliases[s.epoch] = 'epoch_'+str(len(epoch_aliases))
                        if args.check_indexes:
                            check_indexes(s, voxel, transaction % 50 == 0
                                          or transaction == args.transactions-1)
                        arrays = primary_arrays(m)
                        fact_hash = hashlib.sha256()
                        for name in sorted(arrays):
                            fact_hash.update(name.encode());fact_hash.update(arrays[name].tobytes())
                        exports = dict(mapper=m.surface_map_2d(), v1=s.v1_surface_dict(),
                                       v2=s.v2_record('fixture-scene', 'fixture-request'))
                        row = dict(case, verdict=verdict, actual_boxes=len(m._placed_boxes),
                            revision=m.map_revision, facts=fact_hash.hexdigest(), stats=m.stats(),
                            exports_sha256=canonical_hash(normalize_epoch(exports)),
                            content_digest=s.identity.content_digest)
                        rows.write(json.dumps(row, sort_keys=True)+'\n')
                    except Exception as error:
                        journal.flush();rows.flush()
                        failure = out/'failure';failure.mkdir(exist_ok=True)
                        np.savez_compressed(failure/'before.npz', **before)
                        np.savez_compressed(failure/'after.npz', **primary_arrays(m))
                        (failure/'case.json').write_text(json.dumps(dict(case, error=repr(error),
                            update=None if update is None else normalize_epoch(asdict(update)),
                            capture_complete=True, replay_possible=True), indent=2, default=str)+'\n')
                        raise
                if args.check_indexes and args.distance_points:
                    check_distances(s, seed, args.distance_points, out, dict(
                        resolution=r, initial_boxes=box_count, shape=shape, seed=seed, stage='final'))
                if args.capture_final:
                    name = 'r%s_b%d_%s_s%d' % (str(r).replace('.', 'p'), box_count, shape, seed)
                    np.savez_compressed(out/(name+'.npz'), **primary_arrays(m))
                    (out/(name+'_exports.json')).write_text(json.dumps(dict(
                        mapper=m.surface_map_2d(), v1=s.v1_surface_dict(),
                        v2=s.v2_record('fixture-scene', 'fixture-request')), sort_keys=True)+'\n')
    manifest.update(complete=True, rows=sum(1 for _ in (out/'rows.jsonl').open()),
                    rows_sha256=hashlib.sha256((out/'rows.jsonl').read_bytes()).hexdigest())
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--seed-start', type=int, default=0)
    p.add_argument('--seeds', type=int, default=20)
    p.add_argument('--transactions', type=int, default=500)
    p.add_argument('--resolutions', type=float, nargs='+', default=[.01, .02, .05], choices=(.01, .02, .05))
    p.add_argument('--boxes', type=int, nargs='+', default=[0, 5, 20], choices=(0, 5, 20))
    p.add_argument('--shapes', nargs='+', choices=('cuboid', 'chamfer'), default=['cuboid', 'chamfer'])
    p.add_argument('--check-indexes', action='store_true')
    p.add_argument('--distance-points', type=int, default=1000)
    p.add_argument('--capture-final', action='store_true')
    args = p.parse_args()
    if args.seeds < 1 or args.transactions < 1 or args.seed_start < 0 or args.distance_points < 0:
        p.error('seed and transaction counts must be positive')
    root = Path(args.root).resolve()
    sys.path[:0] = [str(root/'src/luggage_perception'), str(root/'src/luggage_description'), str(root)]
    run(args)


if __name__ == '__main__':
    main()
