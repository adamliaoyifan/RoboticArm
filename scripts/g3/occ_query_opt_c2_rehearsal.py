#!/usr/bin/env python3
"""C2-form load rehearsal for the under-load distance gate (W2).

Replicates the production topology in one process: a writer committing
30k-point INTEGRATE_VIEWs at a fixed 2 Hz (fresh seed per write -- the
worst-case contention phase, unlike saturated repeats), the replica's
own single poller applying deltas, and the C2 query mix (summary 10 Hz,
ROI 5 Hz, distance-256 2 Hz, surface 1 Hz) against the facade.

Gate (user 2026-10-31): distance P95 ~50 ms, cap 60 ms, no
QUERY_BUDGET_EXHAUSTED.
"""
import os
for _v in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')
import argparse
import json
import statistics
import sys
import threading
import time
from pathlib import Path

_WS = Path(__file__).resolve().parents[2]
for entry in (_WS/'src'/'luggage_perception', _WS/'src'/'luggage_description', _WS):
    sys.path.insert(0, str(entry))

import numpy as np

from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)
from luggage_perception.cargo_map_query_mode import wire_store_export
from luggage_perception.cargo_query_replica import QueryReplica
from luggage_perception.cargo_map_read_facade import QueryMapReadFacade
from luggage_perception.cargo_map_query import QueryPage
from luggage_perception.cargo_distance_query import DistanceBudget

RESOLUTION = 0.02
RATES = {'summary': 10.0, 'roi': 5.0, 'distance': 2.0, 'surface': 1.0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--write-hz', type=float, default=2.0)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    import tempfile
    root = tempfile.mkdtemp(prefix='c2-rehearsal-')
    stop_file = Path(root)/'stop'
    ready_file = Path(root)/'ready'

    def writer_process():
        # Owns the store exclusively (forked before any parent state);
        # the export hook's threads live in this process, as in the
        # production owner process.
        mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                                   RESOLUTION, max_raycast_points=2000)
        store = OccupancyMapStore(mapper, epoch='c2')
        hook = wire_store_export(store, root)

        def cloud(seed, n=30000):
            rng_ = np.random.default_rng(seed)
            return np.column_stack((
                rng_.uniform(-.6, .6, n), rng_.uniform(-.8, .8, n),
                rng_.uniform(mapper._hull.floor_z + .1,
                             mapper._hull.floor_z + 1.2, n)))

        def commit(index, seed):
            store.apply_update(MapUpdate(
                operation_id='w-%d' % index, kind=UpdateKind.INTEGRATE_VIEW,
                points=cloud(seed), origin=(0., 0., mapper._hull.ceiling_z - .1),
                minimum_z=mapper._hull.floor_z + .05, schema_version=1,
                settled=True, geometry_hash=mapper.geometry_hash,
                stamp_key=('w', index), acquisition_stamp_ns=1_000_000 + index,
                pose_at_confirm={'position': [0., 0., 0.]}))
            hook.wait_pending(60.0)

        for i in range(3):
            commit(i, 1000 + i)
        ready_file.touch()
        index, period = 3, 1.0/args.write_hz
        next_t = time.perf_counter()
        wall = []
        while not stop_file.exists():
            begin = time.perf_counter()
            commit(index, index)
            wall.append((time.perf_counter()-begin)*1e3)
            index += 1
            next_t += period
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.perf_counter()
        with open(Path(root)/'writer_stats.json', 'w') as fh:
            json.dump({'done': index-3, 'wall_ms': wall}, fh)

    import multiprocessing as mp
    ctx = mp.get_context('fork')
    wt = ctx.Process(target=writer_process)
    wt.start()
    deadline = time.time() + 60
    while not ready_file.exists() and time.time() < deadline:
        time.sleep(0.05)
    if not ready_file.exists():
        raise RuntimeError('writer warmup timed out')
    replica = QueryReplica(root)
    facade = QueryMapReadFacade(replica)
    replica.poll()
    replica.start_polling(interval_s=0.05)
    writes = {'done': 0, 'wall': []}
    stop = threading.Event()

    # The writer subprocess owns the mapper; the container geometry is
    # fixed (floor_z = 0.74 - 1.48/2 = 0.0), so the serving side uses the
    # constant directly instead of reaching into the child's state.
    floor_z = 0.74 - 1.48/2
    rng = np.random.default_rng(20261031)
    points = rng.uniform((-.6, -.8, floor_z + .05),
                         (.6, .8, floor_z + 1.0), size=(256, 3))
    region = ((-.2, -.2, floor_z + .1), (.2, .2, floor_z + .9))

    stop = threading.Event()
    stop_file = Path(root)/'stop'
    writes = {'done': 0, 'wall': []}

    def writer():  # runs in a SUBPROCESS: the production topology keeps
        # the writer's python-heavy fusion off the serving process's GIL;
        # co-locating it measured 160 ms distance medians as an artifact.
        import pickle
        index, period = 3, 1.0/args.write_hz
        next_t = time.perf_counter()
        wall = []
        while not stop_file.exists():
            begin = time.perf_counter()
            commit(index, index)
            wall.append((time.perf_counter()-begin)*1e3)
            index += 1
            next_t += period
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.perf_counter()
        with open(Path(root)/'writer_stats.json', 'w') as fh:
            json.dump({'done': index-3, 'wall_ms': wall}, fh)

    def query(kind):
        begin = time.perf_counter_ns()
        try:
            with facade.acquire_query_snapshot('CURRENT') as handle:
                if kind == 'distance':
                    batch = handle.nearest_obstacles(points, 0.30, 'separate')
                    ok = batch.status == 'OK'
                elif kind == 'summary':
                    handle.summary(); ok = True
                elif kind == 'roi':
                    page = handle.query_cells(region, (0, 1, 2),
                                              page=QueryPage(1024, ''))
                    ok = True
                else:
                    handle.surface(region)
                    ok = True
            status = 'OK'
        except Exception as error:  # noqa: BLE001
            ok, status = False, str(getattr(error, 'reason', error))[:60]
        return (time.perf_counter_ns()-begin)/1e6, ok, status

    samples = {k: [] for k in RATES}
    startup = {k: [] for k in RATES}
    failures = {k: [] for k in RATES}
    warmed = threading.Event()

    def run_queries():
        next_t = {k: time.perf_counter() for k in RATES}
        while not stop.is_set():
            now = time.perf_counter()
            for kind, rate in RATES.items():
                if now >= next_t[kind]:
                    elapsed, ok, status = query(kind)
                    if not warmed.is_set():
                        # Startup window: until the first successful
                        # acquire, samples are recorded separately (the
                        # replica may not have applied the first FULL
                        # yet) and never gate the verdict.
                        startup[kind].append(elapsed)
                        if ok:
                            warmed.set()
                    else:
                        samples[kind].append(elapsed)
                        if not ok:
                            failures[kind].append(status)
                    next_t[kind] += 1.0/rate
            time.sleep(0.002)

    qt = threading.Thread(target=run_queries, daemon=True)
    qt.start()
    time.sleep(args.duration)
    print('PHASE stop', flush=True)
    stop_file.touch(); stop.set()
    qt.join(timeout=15)
    print('PHASE qt joined', flush=True)
    try:
        wt.join(timeout=20)
        if wt.is_alive():
            wt.terminate(); wt.join(timeout=5)
    except Exception:  # noqa: BLE001 - teardown must never eat the report
        pass
    print('PHASE wt joined', flush=True)
    try:
        replica.stop_polling(timeout_s=5)
    except Exception:  # noqa: BLE001
        pass
    print('PHASE polling stopped', flush=True)
    writer_stats = json.loads(
        (Path(root)/'writer_stats.json').read_text()) \
        if (Path(root)/'writer_stats.json').exists() else {}
    writes['done'] = writer_stats.get('done', 0)
    writes['wall'] = writer_stats.get('wall_ms', [])

    def stats(values):
        if not values:
            return None
        ordered = sorted(values)
        return dict(n=len(ordered),
                    median_ms=round(statistics.median(ordered), 2),
                    p95_ms=round(ordered[max(0, int(0.95*len(ordered))-1)], 2),
                    max_ms=round(ordered[-1], 2))

    report = dict(kind='c2_rehearsal', duration_s=args.duration,
                  startup_window_ms={k: [round(v, 1) for v in vlist[:3]]
                                     for k, vlist in startup.items() if vlist},
                  distance_over_60ms=[round(v, 1) for v in samples['distance'] if v > 60],
                  write_hz=args.write_hz, writes=writes['done'],
                  write_wall=stats(writes['wall']),
                  queries={k: stats(v) for k, v in samples.items()},
                  failures={k: v[:5] for k, v in failures.items() if v})
    print('PHASE report ready', flush=True)
    Path(args.out).write_text(json.dumps(report, indent=2, sort_keys=True)+'\n')
    print('PHASE written', flush=True)
    print('PHASE printing', flush=True)
    d = report['queries']['distance']
    verdict = ('PASS' if d and d['p95_ms'] <= 60 and d['max_ms'] <= 60
               and not failures.get('distance') else 'FAIL')
    print(json.dumps({'distance': d, 'distance_failures': len(failures['distance']),
                      'writes': writes['done'], 'write_wall': report['write_wall'],
                      'verdict': verdict}))


if __name__ == '__main__':
    raise SystemExit(main())
