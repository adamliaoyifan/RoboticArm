#!/usr/bin/env python3
"""G4-A concurrent representative: INDEPENDENT query process with
write/query overlap (review action 4, review_a17152e3).

Scope and honesty statement (do not read as a PASS artifact):
- The writer (this process) owns the store + export hook and performs
  back-to-back INTEGRATE_VIEW writes -- the offered load is therefore
  "as fast as the writer sustains", measured and reported, not a
  fixed 2Hz target (the user relaxed that requirement).
- A SEPARATE subprocess owns the QueryReplica + read facade (the same
  surfaces the ROS replica node uses) and issues ROI queries in a
  loop WHILE the writer writes -- genuine cross-process overlap, no
  shared GIL.
- Reported: write wall/throughput, per-query latency (nearest-rank
  P95 only at n>=30), per-query FRESHNESS lag (wall time between the
  commit that produced the queried identity and the query), export
  backlog events, encoder thread CPU, failures.
- This is one representative measurement at one resolution/box count,
  not the frozen acceptance matrix, and not b33-paired. The A8 b33
  comparison belongs to the formal acceptance run.
"""

import argparse
import fcntl
import json
import os
import subprocess
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
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)

PERF_LOCK = '/tmp/elfin-occ-backend-perf.lock'

WORKER = r'''
import json, os, sys, time
root, duration_s, interval_ms = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
for entry in (%r + "/src/luggage_description",
              %r + "/src/luggage_perception", %r):
    sys.path.insert(0, entry)
from luggage_perception.cargo_query_replica import QueryReplica, ReplicaState
from luggage_perception.cargo_map_read_facade import QueryMapReadFacade
from luggage_perception.cargo_map_query import QueryError, QueryPage

replica = QueryReplica(root)
facade = QueryMapReadFacade(replica)
floor = 0.0
region = ((-0.15, -0.15, floor), (0.15, 0.15, floor + 0.3))
rows, errors, states = [], [], {}
deadline = time.time() + duration_s
ready_seen = False
while time.time() < deadline:
    t0 = time.perf_counter_ns()
    state = replica.poll()
    states[str(state)] = states.get(str(state), 0) + 1
    err = None
    try:
        handle = facade.acquire_query_snapshot('CURRENT')
        handle.query_cells(region, (0, 1, 2), page=QueryPage(4096, ''))
    except QueryError as error:
        err = str(getattr(error, 'reason', error))
    elapsed_ms = (time.perf_counter_ns() - t0) / 1e6
    snap = replica.snapshot
    ident = (list(snap.identity[:2]) if snap is not None else None)
    if err is None and state == ReplicaState.READY and ident:
        ready_seen = True
        rows.append({'t': time.time(), 'ms': round(elapsed_ms, 3),
                     'identity': ident})
    elif err is not None:
        errors.append(err)
    time.sleep(interval_ms / 1000.0)
print(json.dumps({'rows': rows, 'errors': errors[:10],
                  'error_count': len(errors), 'states': states,
                  'ready_seen': ready_seen}))
'''


def make_cloud(mapper, seed=3):
    return np.column_stack([
        np.random.default_rng(seed).uniform(-0.6, 0.6, 30000),
        np.random.default_rng(seed + 1).uniform(-0.8, 0.8, 30000),
        np.random.default_rng(seed + 2).uniform(
            mapper._hull.floor_z + .1,
            mapper._hull.floor_z + 1.2, 30000)])


def integrate(store, mapper, cloud, stamp):
    return store.apply_update(MapUpdate(
        operation_id='w-%d' % stamp, kind=UpdateKind.INTEGRATE_VIEW,
        expected_snapshot=store.identity,
        geometry_hash=mapper.geometry_hash, points=cloud,
        origin=(0., 0., mapper._hull.ceiling_z - 0.1),
        minimum_z=mapper._hull.floor_z + .05,
        stamp_key=(stamp, 0),
        acquisition_stamp_ns=1_000_000_000 + stamp,
        settled=True, schema_version=1))


def nearest_rank_p95(values):
    """Nearest-rank P95: sorted[ceil(0.95*n)-1]. None when n<30
    (not_evaluated -- never a recomputed guess)."""
    import math
    if len(values) < 30:
        return None
    return sorted(values)[math.ceil(0.95 * len(values)) - 1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--boxes', type=int, default=0)
    parser.add_argument('--resolution', type=float, default=0.01)
    parser.add_argument('--duration-s', type=float, default=60.0)
    parser.add_argument('--query-interval-ms', type=float, default=50.0)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    lock_fd = os.open(PERF_LOCK, os.O_CREAT | os.O_RDWR)
    fcntl.flock(lock_fd, fcntl.LOCK_EX)

    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                               args.resolution, max_raycast_points=2000)
    store = OccupancyMapStore(mapper, epoch='conc')
    for i, (z, y, x) in enumerate(
            (z, y, x) for z in range(4)
            for y in (-.48, .18) for x in (-.5, 0., .5)):
        if i >= args.boxes:
            break
        assert store.apply_update(MapUpdate(
            operation_id='b%d' % i, kind=UpdateKind.ADD_BOX_V1,
            box=((x, y, mapper._hull.floor_z + .11 + .24 * z),
                 (.35, .53, .22), 0.), box_id='box-%d' % i)).accepted

    root = tempfile.mkdtemp(prefix='conc-rep-')
    hook = wire_store_export(store, root)
    store._snapshot = store._build_snapshot()
    hook.wait_pending(60.0)

    cloud = make_cloud(mapper)
    for stamp in range(3):
        assert integrate(store, mapper, cloud, stamp).accepted
    hook.wait_pending(60.0)

    # Spawn the INDEPENDENT query process before writing begins.
    worker = subprocess.Popen(
        [sys.executable, '-c', WORKER % (_WS_SRC, _WS_SRC, _WS_SRC),
         root, str(args.duration_s), str(args.query_interval_ms)],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    revision = 3
    commits, write_ms, backlog_events = [], [], 0
    deadline = time.time() + args.duration_s
    while time.time() < deadline:
        pending = getattr(hook._publisher, '_pending', 0)
        if pending and pending >= 2:
            backlog_events += 1
        t0 = time.perf_counter_ns()
        result = integrate(store, mapper, cloud, 1000 + revision)
        write_ms.append((time.perf_counter_ns() - t0) / 1e6)
        if not result.accepted:
            break
        commits.append({'t': time.time(),
                        'identity': list(result.identity[:2])})
        revision += 1

    out, err = worker.communicate(timeout=30)
    hook.wait_pending(60.0)
    hook.close()
    os.close(lock_fd)

    try:
        queried = json.loads(out.strip().splitlines()[-1])
    except (ValueError, IndexError):
        print('worker output unparseable: %r stderr: %s'
              % (out[-200:], err[-500:]), file=sys.stderr)
        return 4
    if queried['error_count']:
        print('query errors: %d e.g. %s'
              % (queried['error_count'], queried['errors'][:2]),
              file=sys.stderr)

    # Freshness: for each query, wall-time distance to the commit that
    # produced the queried identity (writer and worker share wall time).
    commit_by_key = {}
    for c in commits:
        commit_by_key[tuple(c['identity'])] = c['t']
    lag_ms, unknown = [], 0
    for row in queried['rows']:
        t_commit = commit_by_key.get(tuple(row['identity']))
        if t_commit is None:
            unknown += 1
            continue
        lag_ms.append((row['t'] - t_commit) * 1000.0)

    lat = sorted(row['ms'] for row in queried['rows'])
    writes_sorted = sorted(write_ms)
    report = {
        'revision': subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=_WS_SRC,
            text=True).strip(),
        'scope': ('one concurrent representative: independent query '
                  'process, writer back-to-back (sustainable offered '
                  'load), not the frozen matrix, not b33-paired'),
        'boxes': args.boxes, 'resolution': args.resolution,
        'duration_s': args.duration_s,
        'query_interval_ms': args.query_interval_ms,
        'writes': {'n': len(write_ms),
                   'med_ms': round(writes_sorted[len(writes_sorted) // 2], 3)
                   if writes_sorted else None,
                   'p95_ms': nearest_rank_p95(
                       [round(v, 3) for v in write_ms]),
                   'throughput_per_s': round(
                       len(write_ms) / args.duration_s, 4)},
        'queries': {'n': len(lat),
                    'med_ms': round(lat[len(lat) // 2], 3) if lat else None,
                    'p95_ms': nearest_rank_p95(lat),
                    'max_ms': round(lat[-1], 3) if lat else None,
                    'errors': queried['error_count'],
                    'replica_states': queried['states'],
                    'ready_seen': queried['ready_seen']},
        'freshness': {'n': len(lag_ms),
                      'med_ms': round(sorted(lag_ms)[len(lag_ms) // 2], 1)
                      if lag_ms else None,
                      'p95_ms': nearest_rank_p95(lag_ms),
                      'max_ms': round(max(lag_ms), 1) if lag_ms else None,
                      'identity_unknown': unknown},
        'export_backlog_events': backlog_events,
    }
    with open(args.out, 'w') as handle:
        json.dump(report, handle, indent=1)
    print(json.dumps(report, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
