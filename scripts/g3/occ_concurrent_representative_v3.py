#!/usr/bin/env python3
"""G4-A gen3 concurrent representative runner (plan d87c5731 section
6.1/6.2, review_8a372dee P1 measurement holes).

What changed vs the gen2 runner:
- RAW EVENT JSONL, not aggregates: per-commit full 5-leg identities +
  ledger_id, per-fact export ledger terminals, receiver
  received/ready/resync events with full identities, and EVERY query
  request with exactly one terminal state (a success while the
  replica is not READY is recorded as such, never dropped).
- Visibility is judged from commit->READY joins (exact_visible /
  coalesced_into / not_delivered / superseded_reset via the approved
  coalescing ledger), not from query-time version age.
- Gates are evaluated by the PURE judge (occ_metrics_g3): the summary
  is recomputable offline from the raw file; the process exits 0
  (pass) / 3 (fail) / 4 (not_evaluated).
- 1 Hz RSS samples of both processes are part of the raw record.

Honest scope statement: the writer still owns the store (offered load
= whatever the writer sustains, measured); the query side is a REAL
independent subprocess using the same replica + facade surfaces the
ROS node uses.
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
sys.path.insert(0, os.path.join(_WS_SRC, 'scripts', 'g3'))
sys.path.insert(0, os.path.join(_WS_SRC, "src", "luggage_perception"))
sys.path.insert(0, os.path.join(_WS_SRC, "src", "luggage_description"))
sys.path.insert(0, _WS_SRC)

import numpy as np

from luggage_perception.cargo_map_query_mode import wire_store_export
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)
from occ_metrics_g3 import judge

PERF_LOCK = '/tmp/elfin-occ-backend-perf.lock'

WORKER = r'''
import json, os, sys, time
root, events_path, duration_s, interval_ms, typed_flag = (
    sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4]),
    sys.argv[5] == 'typed')
for entry in (%r + "/src/luggage_description",
              %r + "/src/luggage_perception", %r):
    sys.path.insert(0, entry)
from luggage_perception.cargo_query_replica import QueryReplica, ReplicaState
from luggage_perception.cargo_map_read_facade import QueryMapReadFacade
from luggage_perception.cargo_map_query import QueryError, QueryPage

out = open(events_path, 'a', buffering=1)


def emit(record):
    record['t_ns'] = time.monotonic_ns()
    out.write(json.dumps(record) + '\n')


def rss_mb():
    try:
        with open('/proc/self/status') as handle:
            for line in handle:
                if line.startswith('VmRSS'):
                    return int(line.split()[1]) / 1024.0
    except OSError:
        return None


replica = QueryReplica(root)
facade = QueryMapReadFacade(replica)
floor = 0.0
region = ((-0.15, -0.15, floor), (0.15, 0.15, floor + 0.3))
request_id = 0
seen_events = 0
startup = True
next_rss = 0.0

typed_node = None
typed_client = None
if typed_flag:
    import rclpy
    from luggage_msgs.srv import QueryCargoCellsV2
    from luggage_perception.cargo_map_query_service import MapQueryService
    rclpy.init()
    typed_node = rclpy.create_node('occ_g3_typed_worker')
    MapQueryService(typed_node, facade, None)
    typed_client = typed_node.create_client(
        QueryCargoCellsV2, '/cargo_map/query_cells_v2')


def typed_query_roundtrip():
    """REAL typed ROS roundtrip over the graph (serialization + DDS
    transport + callback), recorded as its own measurement path."""
    from luggage_msgs.srv import QueryCargoCellsV2
    request = QueryCargoCellsV2.Request()
    request.query.request_id = 'typed-g3'
    request.query.frame_id = 'container_link'
    request.query.use_current = True
    request.query.release_after_query = True
    future = typed_client.call_async(request)
    deadline = time.monotonic() + 2.0
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(typed_node, timeout_sec=0.001)
    if not future.done():
        return 'timeout', None
    response = future.result()
    status = response.meta.status
    if status != 'OK':
        return 'named_reject', status
    return 'ok', None


deadline = time.time() + duration_s
while time.time() < deadline:
    t0 = time.perf_counter_ns()
    state = replica.poll()
    events = replica.events()
    while seen_events < len(events):
        event = dict(events[seen_events])
        seen_events += 1
        out.write(json.dumps(event) + '\n')   # ready/resync/... history
    if str(state) == 'READY' and replica.snapshot is not None:
        startup = False
    request_id += 1
    err = None
    try:
        handle = facade.acquire_query_snapshot('CURRENT')
        handle.query_cells(region, (0, 1, 2), page=QueryPage(4096, ''))
        handle.close()
    except QueryError as error:
        err = str(getattr(error, 'reason', error))
    except Exception as error:                 # noqa: BLE001 - recorded
        err = 'exception:' + type(error).__name__
    total_ms = (time.perf_counter_ns() - t0) / 1e6
    record = {'kind': 'query', 'request_id': 'q' + str(request_id),
              'expected': 'CURRENT', 'path': 'facade',
              'returned': (list(replica.snapshot.identity[:2])
                          if replica.snapshot is not None else None),
              'status': ('ok' if err is None else
                         ('named_reject' if not err.startswith('exception')
                          else 'exception')),
              'reason': err, 'total_ms': round(total_ms, 3),
              'state_at_request': str(state), 'startup': startup}
    out.write(json.dumps(record) + '\n')
    if typed_client is not None:
        t1 = time.perf_counter_ns()
        status, detail = typed_query_roundtrip()
        typed_ms = (time.perf_counter_ns() - t1) / 1e6
        out.write(json.dumps({
            'kind': 'query', 'request_id': 't' + str(request_id),
            'expected': 'CURRENT', 'path': 'typed', 'returned': None,
            'status': status, 'reason': detail,
            'total_ms': round(typed_ms, 3),
            'state_at_request': str(state), 'startup': startup}) + '\n')
    if time.time() >= next_rss:
        emit({'kind': 'rss', 'process': 'query', 'rss_mb': rss_mb()})
        next_rss = time.time() + 1.0
    time.sleep(interval_ms / 1000.0)
emit({'kind': 'rss', 'process': 'query', 'rss_mb': rss_mb()})
if typed_node is not None:
    typed_node.destroy_node()
    rclpy.shutdown()
out.close()
'''


def rss_mb_pid(pid):
    try:
        with open('/proc/%d/status' % pid) as handle:
            for line in handle:
                if line.startswith('VmRSS'):
                    return int(line.split()[1]) / 1024.0
    except OSError:
        return None
    return None


def rss_mb_self():
    return rss_mb_pid(os.getpid())


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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--boxes', type=int, default=0)
    parser.add_argument('--resolution', type=float, default=0.01)
    parser.add_argument('--duration-s', type=float, default=60.0)
    parser.add_argument('--query-interval-ms', type=float, default=50.0)
    parser.add_argument('--query-p95-ms', type=float, default=100.0)
    parser.add_argument('--query-max-ms', type=float, default=250.0)
    parser.add_argument('--vis-p95-ms', type=float, default=1000.0)
    parser.add_argument('--vis-max-ms', type=float, default=2000.0)
    parser.add_argument('--mode', choices=('d0', 'd1'), default='d1',
                        help='d1: overlapping writes (visibility gated);'
                             ' d0: read-only window (visibility N/A)')
    parser.add_argument('--typed', action='store_true',
                        help='also measure REAL typed ROS roundtrips '
                             '(separate judge path)')
    parser.add_argument('--out', required=True,
                        help='output prefix: <out>.events.jsonl + <out>.json')
    args = parser.parse_args()

    events_path = args.out + '.events.jsonl'
    worker_events = args.out + '.worker.jsonl'
    # Path validation BEFORE taking the perf lock: an invalid run must
    # never queue behind (or block) a legitimate re-timing campaign.
    for path in (events_path, worker_events, args.out + '.json'):
        if os.path.exists(path):
            raise SystemExit('refusing to overwrite %s' % path)

    lock_fd = os.open(PERF_LOCK, os.O_CREAT | os.O_RDWR)
    fcntl.flock(lock_fd, fcntl.LOCK_EX)

    def emit(record):
        record['t_ns'] = time.monotonic_ns()
        with open(events_path, 'a') as handle:
            handle.write(json.dumps(record) + '\n')

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

    root = tempfile.mkdtemp(prefix='conc-rep-g3-')
    hook = wire_store_export(store, root)
    store._snapshot = store._build_snapshot()
    hook.wait_pending(60.0)

    cloud = make_cloud(mapper)
    for stamp in range(3):
        assert integrate(store, mapper, cloud, stamp).accepted
    hook.wait_pending(60.0)

    worker = subprocess.Popen(
        [sys.executable, '-c',
         WORKER % (_WS_SRC, _WS_SRC, _WS_SRC),
         root, worker_events, str(args.duration_s + 3.0),
         str(args.query_interval_ms),
         'typed' if args.typed else 'plain'],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    revision = 3
    emitted_ledgers = set()
    next_rss = 0.0
    writes = []
    if args.mode == 'd0':
        # Read-only window: the writer idles; only the worker queries.
        deadline = time.time() + args.duration_s
        while time.time() < deadline:
            time.sleep(0.5)
            emit({'kind': 'rss', 'process': 'writer',
                  'rss_mb': rss_mb_self(),
                  'worker_rss_mb': rss_mb_pid(worker.pid)})
        worker.communicate(timeout=30)
        hook.wait_pending(60.0)
        emit({'kind': 'rss', 'process': 'writer', 'rss_mb': rss_mb_self(),
              'worker_rss_mb': None})
        hook.close()
        os.close(lock_fd)
        records = []
        for path in (events_path, worker_events):
            with open(path) as handle:
                for line in handle:
                    if line.strip():
                        records.append(json.loads(line))
        records.sort(key=lambda r: r.get('t_ns', 0))
        with open(events_path, 'w') as handle:
            for record in records:
                handle.write(json.dumps(record) + '\n')
        gates = {'query': {'p95_bound_ms': args.query_p95_ms,
                           'max_bound_ms': args.query_max_ms}}
        verdict = judge(records, gates)
        summary = {
            'revision': subprocess.check_output(
                ['git', 'rev-parse', 'HEAD'], cwd=_WS_SRC,
                text=True).strip(),
            'mode': 'd0',
            'scope': ('gen3 D0 read-only window: query gate only '
                      '(visibility N/A per plan 7.2)'),
            'boxes': args.boxes, 'resolution': args.resolution,
            'duration_s': args.duration_s, 'raw_events': events_path,
            'gates': verdict['gates'], 'exit': verdict['exit']}
        with open(args.out + '.json', 'w') as handle:
            json.dump(summary, handle, indent=1)
        print(json.dumps(summary, indent=1))
        return verdict['exit']
    deadline = time.time() + args.duration_s
    while time.time() < deadline:
        t0 = time.perf_counter_ns()
        result = integrate(store, mapper, cloud, 1000 + revision)
        write_ms = (time.perf_counter_ns() - t0) / 1e6
        if not result.accepted:
            emit({'kind': 'commit', 'op_id': 'w-%d' % (1000 + revision),
                  'identity': list(result.identity), 'epoch':
                  result.identity[0], 'accepted': False,
                  'reason': result.reason, 'write_ms': round(write_ms, 3)})
            break
        offer = hook.last_status or {}
        emit({'kind': 'commit', 'op_id': 'w-%d' % (1000 + revision),
              'identity': list(result.identity),
              'epoch': result.identity[0], 'accepted': True,
              'ledger_id': offer.get('ledger_id'),
              'write_ms': round(write_ms, 3)})
        writes.append(write_ms)
        revision += 1
        # ledger terminals since last sample
        for entry in hook._publisher.ledger_snapshot():
            if (entry['status'] not in ('pending',)
                    and entry['ledger_id'] not in emitted_ledgers):
                emitted_ledgers.add(entry['ledger_id'])
                emit({'kind': 'ledger', **entry})
        if time.time() >= next_rss:
            emit({'kind': 'rss', 'process': 'writer',
                  'rss_mb': rss_mb_self(),
                  'worker_rss_mb': rss_mb_pid(worker.pid)})
            next_rss = time.time() + 1.0

    worker.communicate(timeout=30)
    hook.wait_pending(60.0)
    # grace: the worker runs 3 s past the writer's deadline so the
    # trailing generations still get their READY events recorded.
    time.sleep(3.0)
    for entry in hook._publisher.ledger_snapshot():
        if (entry['status'] not in ('pending',)
                and entry['ledger_id'] not in emitted_ledgers):
            emit({'kind': 'ledger', **entry})
    emit({'kind': 'rss', 'process': 'writer', 'rss_mb': rss_mb_self(),
          'worker_rss_mb': None})
    hook.close()
    os.close(lock_fd)

    # Merge worker events into the raw stream (both share monotonic ns
    # on this host); the summary judge runs on the MERGED file.
    records = []
    for path in (events_path, worker_events):
        with open(path) as handle:
            for line in handle:
                if line.strip():
                    records.append(json.loads(line))
    records.sort(key=lambda r: r.get('t_ns', 0))
    with open(events_path, 'w') as handle:
        for record in records:
            handle.write(json.dumps(record) + '\n')

    gates = {'query': {'p95_bound_ms': args.query_p95_ms,
                       'max_bound_ms': args.query_max_ms},
             'visibility': {'p95_bound_ms': args.vis_p95_ms,
                            'max_bound_ms': args.vis_max_ms}}
    verdict = judge(records, gates)
    summary = {
        'revision': subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=_WS_SRC,
            text=True).strip(),
        'scope': ('gen3 concurrent representative: raw events + pure '
                  'judge; summary recomputable via occ_metrics_g3 '
                  'on the .events.jsonl file'),
        'boxes': args.boxes, 'resolution': args.resolution,
        'duration_s': args.duration_s,
        'raw_events': events_path,
        'writes': {'n': len(writes),
                   'med_ms': round(sorted(writes)[len(writes) // 2], 3)
                   if writes else None,
                   'throughput_per_s': round(
                       len(writes) / args.duration_s, 4)},
        'gates': verdict['gates'],
        'exit': verdict['exit'],
    }
    with open(args.out + '.json', 'w') as handle:
        json.dump(summary, handle, indent=1)
    print(json.dumps(summary, indent=1))
    return verdict['exit']


if __name__ == "__main__":
    sys.exit(main())
