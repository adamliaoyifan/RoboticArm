#!/usr/bin/env python3
"""OCC box-scaling: does more cargo slow the OCC down?

For each box count (default 0,1,3,5,8,12) at one resolution:
- build the store, place N boxes (same layout as the concurrency owner),
- run K warm + K measured IntegrateCargoView writes (30k synthetic
  cloud, the frozen acceptance load) and record:
  write wall, fusion wall (integrate_points), derived wall
  (refresh), snapshot build, plus the point-flow accounting
  (rays_admitted/rays_sampled/hits),
- run R ROI queries through the isolated facade and record latency,
- measure commit->READY visibility once,
- dump per-case work counters (dirty/reused/changed blocks,
  replica decodes).

Answers the user's question directly: per-update and per-query cost as
boxes accumulate. No sim, deterministic, perf flock held.
"""

import argparse
import fcntl
import json
import os
import statistics
import sys
import tempfile
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS_SRC = os.path.normpath(os.path.join(_HERE, "..", ".."))
sys.path.insert(0, os.path.join(_WS_SRC, "src", "luggage_perception"))
sys.path.insert(0, os.path.join(_WS_SRC, "src", "luggage_description"))
sys.path.insert(0, _WS_SRC)

import numpy as np

from luggage_perception.cargo_map_query_mode import wire_store_export
from luggage_perception.cargo_map_query import QueryPage, QueryError
from luggage_perception.cargo_map_read_facade import QueryMapReadFacade
from luggage_perception.cargo_query_replica import (
    QueryReplica, ReplicaState)
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)

PERF_LOCK = '/tmp/elfin-occ-backend-perf.lock'


def build(boxes, resolution):
    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                               resolution, max_raycast_points=2000)
    store = OccupancyMapStore(mapper, epoch='scale')
    for i, (z, y, x) in enumerate(
            (z, y, x) for z in range(4)
            for y in (-.48, .18) for x in (-.5, 0., .5)):
        if i >= boxes:
            break
        r = store.apply_update(MapUpdate(
            operation_id='b%d' % i, kind=UpdateKind.ADD_BOX_V1,
            box=((x, y, mapper._hull.floor_z + .11 + .24 * z),
                 (.35, .53, .22), 0.), box_id='box-%d' % i))
        assert r.accepted
    return store, mapper


def phases_of(store, mapper, cloud, stamp):
    """One write with per-phase timers (fusion/derived/snapshot)."""
    marks = {}

    def wrap(obj, name, key):
        original = getattr(obj, name)
        def timed(*a, **k):
            t0 = time.perf_counter_ns()
            r = original(*a, **k)
            marks[key] = (time.perf_counter_ns() - t0) / 1e6
            return r
        setattr(obj, name, timed)
        return original

    restore = []
    restore.append((store._mapper, 'integrate_points',
                    wrap(store._mapper, 'integrate_points', 'fusion_ms')))
    restore.append((store, '_refresh_query_indexes',
                    wrap(store, '_refresh_query_indexes', 'derived_ms')))
    t0 = time.perf_counter_ns()
    result = store.apply_update(MapUpdate(
        operation_id='s-%d' % stamp, kind=UpdateKind.INTEGRATE_VIEW,
        expected_snapshot=store.identity,
        geometry_hash=mapper.geometry_hash, points=cloud,
        origin=(0., 0., mapper._hull.ceiling_z - 0.1),
        minimum_z=mapper._hull.floor_z + .05,
        stamp_key=(stamp, 0),
        acquisition_stamp_ns=1_000_000_000 + stamp,
        settled=True, schema_version=1))
    wall = (time.perf_counter_ns() - t0) / 1e6
    for obj, name, original in restore:
        setattr(obj, name, original)
    counts = result.details.get('preprocess_counts', {}) \
        if hasattr(result, 'details') else {}
    return {'wall_ms': round(wall, 3),
            'fusion_ms': round(marks.get('fusion_ms', 0), 3),
            'derived_ms': round(marks.get('derived_ms', 0), 3),
            'counts': counts}, result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--boxes', default='0,1,3,5,8,12')
    parser.add_argument('--resolution', type=float, default=0.01)
    parser.add_argument('--samples', type=int, default=20)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    lock_fd = os.open(PERF_LOCK, os.O_CREAT | os.O_RDWR)
    fcntl.flock(lock_fd, fcntl.LOCK_EX)
    report = {'resolution': args.resolution, 'cases': []}

    for boxes in (int(v) for v in args.boxes.split(',')):
        store, mapper = build(boxes, args.resolution)
        root = tempfile.mkdtemp(prefix='scale-')
        hook = wire_store_export(store, root)
        store._snapshot = store._build_snapshot()
        hook.wait_pending(120.0)
        replica = QueryReplica(root)
        facade = QueryMapReadFacade(replica)
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            if replica.poll() == ReplicaState.READY:
                break
            time.sleep(0.05)

        cloud = np.column_stack([
            np.random.default_rng(3).uniform(-0.6, 0.6, 30000),
            np.random.default_rng(4).uniform(-0.8, 0.8, 30000),
            np.random.default_rng(5).uniform(
                mapper._hull.floor_z + .1,
                mapper._hull.floor_z + 1.2, 30000)])

        # warm
        for stamp in range(3):
            assert store.apply_update(MapUpdate(
                operation_id='w%d' % stamp,
                kind=UpdateKind.INTEGRATE_VIEW,
                expected_snapshot=store.identity,
                geometry_hash=mapper.geometry_hash, points=cloud,
                origin=(0., 0., mapper._hull.ceiling_z - 0.1),
                minimum_z=mapper._hull.floor_z + .05,
                stamp_key=(900 + stamp, 0),
                acquisition_stamp_ns=1_000_000_000 + stamp,
                settled=True, schema_version=1)).accepted
            store._snapshot = store._build_snapshot()
        hook.wait_pending(120.0)

        # measured writes
        writes = []
        for stamp in range(args.samples):
            row, _ = phases_of(store, mapper, cloud, 1000 + stamp)
            writes.append(row)
            store._snapshot = store._build_snapshot()
        hook.wait_pending(120.0)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if replica.poll() == ReplicaState.READY:
                break
            time.sleep(0.02)
        work = dict(store._query_indexes.work)

        # measured ROI queries (isolated facade)
        floor = mapper._hull.floor_z
        region = ((-0.15, -0.15, floor), (0.15, 0.15, floor + 0.3))
        roi = []
        for _ in range(args.samples):
            t0 = time.perf_counter_ns()
            try:
                handle = facade.acquire_query_snapshot('CURRENT')
                handle.query_cells(region, (0, 1, 2),
                                   page=QueryPage(4096, ''))
                roi.append((time.perf_counter_ns() - t0) / 1e6)
            except QueryError as error:
                roi.append(None)
                break

        def med(values):
            v = sorted(x for x in values if x is not None)
            return round(statistics.median(v), 3) if v else None

        case = {
            'boxes': boxes,
            'write_wall_ms_med': med([w['wall_ms'] for w in writes]),
            'fusion_ms_med': med([w['fusion_ms'] for w in writes]),
            'derived_ms_med': med([w['derived_ms'] for w in writes]),
            'point_flow_med': {
                'rays_admitted': med([w['counts'].get('rays_admitted')
                                      for w in writes]),
                'rays_sampled': med([w['counts'].get('rays_sampled')
                                     for w in writes]),
                'hits': med([w['counts'].get('hits') for w in writes]),
            },
            'roi_query_ms_med': med(roi),
            'work': {k: work.get(k) for k in
                     ('dirty_blocks', 'public_payload_reused_blocks',
                      'public_payload_changed_blocks')},
            'replica_cache': {'hits': replica.cache_hits,
                              'misses': replica.cache_misses},
            'feasible_cells': sum(
                1 for name in replica.manifest['entries']
                if name.endswith('.meta')) if replica.manifest else None,
        }
        report['cases'].append(case)
        print(json.dumps(case))
        hook.close()
        import shutil
        shutil.rmtree(root, ignore_errors=True)

    os.close(lock_fd)
    with open(args.out, 'w') as handle:
        json.dump(report, handle, indent=1)
    print('saved', args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
