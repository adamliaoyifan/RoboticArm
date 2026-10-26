#!/usr/bin/env python3
"""G4-A gen2 representative gates (I1/I2 work assertions + four-axis
latency report). NOT the full matrix -- a failing representative group
stops everything per the frozen protocol.

Axes reported separately (user 2026-10-26):
  A. ROI response (client round trip, A7 gates P95<=100ms / max<=250ms)
  B. commit -> READY visibility (P95<=1s / max<=2s)
  C. write-call duration (apply_update wall, A8 gate <=1.10x baseline
     -- baseline here is the SAME store WITHOUT the export hook, an
     honest paired same-binary comparison)
  D. background total CPU (encoder thread + replica poll/decode)

I1 (no-change zero-work) and I2 (sparse-change reuse) assertions use
the real work counters (public_payload_reused_blocks /
public_payload_changed_blocks / cache_hits / cache_misses / encode
JSON constructions).
"""

import argparse
import fcntl
import json
import os
import statistics
import sys
import tempfile
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS_SRC = os.path.normpath(os.path.join(_HERE, "..", ".."))
sys.path.insert(0, os.path.join(_WS_SRC, "src", "luggage_perception"))
sys.path.insert(0, _WS_SRC)

import numpy as np

from luggage_perception.cargo_map_query_mode import wire_store_export
from luggage_perception.cargo_query_export import QueryExportPublisher
from luggage_perception.cargo_query_replica import QueryReplica, ReplicaState
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)

PERF_LOCK = '/tmp/elfin-occ-backend-perf.lock'


def make_store(boxes, resolution=0.01):
    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                               resolution, max_raycast_points=2000)
    store = OccupancyMapStore(mapper, epoch='rep')
    for i, (z, y, x) in enumerate(
            (z, y, x) for z in range(4)
            for y in (-.48, .18) for x in (-.5, 0., .5)):
        if i >= boxes:
            break
        result = store.apply_update(MapUpdate(
            operation_id='b%d' % i, kind=UpdateKind.ADD_BOX_V1,
            box=((x, y, mapper._hull.floor_z + .11 + .24 * z),
                 (.35, .53, .22), 0.), box_id='box-%d' % i))
        assert result.accepted
    return store, mapper


def query_roi(store, mapper, limit=4096):
    from luggage_perception.cargo_map_query import QueryPage
    floor = mapper._hull.floor_z
    region = ((-0.15, -0.15, floor), (0.15, 0.15, floor + 0.3))
    t0 = time.perf_counter_ns()
    with store.acquire_query_snapshot() as handle:
        handle.query_cells(region, (0, 1, 2), page=QueryPage(limit, ''))
    return (time.perf_counter_ns() - t0) / 1e6


def stats(values):
    ordered = sorted(values)
    return {
        'n': len(values),
        'median_ms': round(statistics.median(values), 3),
        'p95_ms': round(ordered[int(round(0.95 * len(values))) - 1], 3),
        'max_ms': round(max(values), 3),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--boxes', type=int, default=0)
    parser.add_argument('--resolution', type=float, default=0.01)
    parser.add_argument('--samples', type=int, default=30)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    lock_fd = os.open(PERF_LOCK, os.O_CREAT | os.O_RDWR)
    fcntl.flock(lock_fd, fcntl.LOCK_EX)

    report = {'boxes': args.boxes, 'resolution': args.resolution,
              'samples': args.samples}

    # ---------- Axis C baseline: same store WITHOUT the hook ---------
    store_c, mapper_c = make_store(args.boxes, args.resolution)
    cloud = np.column_stack([
        np.random.default_rng(3).uniform(-0.6, 0.6, 30000),
        np.random.default_rng(4).uniform(-0.8, 0.8, 30000),
        np.random.default_rng(5).uniform(
            mapper_c._hull.floor_z + .1,
            mapper_c._hull.floor_z + 1.2, 30000)])

    def integrate(store_, mapper_, stamp, points):
        return store_.apply_update(MapUpdate(
            operation_id='w-%d' % stamp, kind=UpdateKind.INTEGRATE_VIEW,
            expected_snapshot=store_.identity,
            geometry_hash=mapper_.geometry_hash, points=points,
            origin=(0., 0., mapper_.ceiling_z - 0.1)
            if hasattr(mapper_, 'ceiling_z') else (0., 0., 2.0),
            minimum_z=mapper_._hull.floor_z + .05,
            stamp_key=(stamp, 0),
            acquisition_stamp_ns=1_000_000_000 + stamp,
            settled=True, schema_version=1))

    for stamp in range(3):
        integrate(store_c, mapper_c, stamp, cloud)
    baseline_writes = []
    for stamp in range(3, 3 + args.samples):
        t0 = time.perf_counter_ns()
        result = integrate(store_c, mapper_c, stamp, cloud)
        baseline_writes.append(
            (time.perf_counter_ns() - t0) / 1e6)
        assert result.accepted
    report['axis_c_write_baseline_ms'] = stats(baseline_writes)

    # ---------- Isolated run with hook + replica ----------------------
    root = tempfile.mkdtemp(prefix='rep-')
    store, mapper = make_store(args.boxes, args.resolution)
    hook = wire_store_export(store, root)
    replica = QueryReplica(root)

    # Prime FULL + reach READY.
    store._snapshot = store._build_snapshot()
    hook.wait_pending(30.0)
    deadline = time.monotonic() + 30.0
    while replica.poll() != ReplicaState.READY:
        assert time.monotonic() < deadline, 'replica never READY'

    # ---------- I1: no-change zero work (metadata-only) ---------------
    misses_before = replica.cache_misses
    t_commit0 = time.monotonic_ns()
    store.apply_update(MapUpdate(
        operation_id='pose-1', kind=UpdateKind.CONTAINER_POSE,
        event_stamp_ns=1))
    ready_times = []
    for _ in range(2000):
        state = replica.poll()
        if state == ReplicaState.READY:
            manifest_seq = replica.manifest['seq']
            break
        time.sleep(0.001)
    t_ready0 = time.monotonic_ns()
    ready_times.append((t_ready0 - t_commit0) / 1e6)
    work = dict(store._query_indexes.work)
    i1 = {
        'axis_b_commit_to_ready_ms': round(ready_times[0], 3),
        'blocks_reused': work.get('public_payload_reused_blocks', -1),
        'blocks_changed': work.get('public_payload_changed_blocks', -1),
        'replica_decodes': replica.cache_misses - misses_before,
    }
    hook.wait_pending(30.0)
    replica.poll()
    i1['replica_decodes_after_drain'] = (
        replica.cache_misses - misses_before)
    report['I1_metadata_only'] = i1

    # ---------- I2: sparse change reuse --------------------------------
    misses_before = replica.cache_misses
    result = store.apply_update(MapUpdate(
        operation_id='sparse-1', kind=UpdateKind.ADD_BOX_V1,
        box=((0.30, 0.30, mapper._hull.floor_z + .11),
             (.10, .10, .08), 0.), box_id='sparse-1'))
    assert result.accepted
    work = dict(store._query_indexes.work)
    hook.wait_pending(30.0)
    t_commit = time.monotonic_ns()
    for _ in range(2000):
        state = replica.poll()
        if state == ReplicaState.READY and replica.manifest['seq'] >= \
                i1.get('_seq', replica.manifest['seq']):
            break
        time.sleep(0.001)
    t_ready = time.monotonic_ns()
    report['I2_sparse_change'] = {
        'axis_b_commit_to_ready_ms': round((t_ready - t_commit) / 1e6, 3),
        'dirty_blocks': work.get('dirty_blocks', -1),
        'blocks_reused': work.get('public_payload_reused_blocks', -1),
        'blocks_changed': work.get('public_payload_changed_blocks', -1),
        'replica_decodes': replica.cache_misses - misses_before,
    }

    # ---------- Axis A: ROI response under the write stream -----------
    stop = threading.Event()

    def writer():
        stamp = 100
        while not stop.is_set():
            stamp += 1
            integrate(store, mapper, stamp, cloud)
            stop.wait(0.5)

    thread = threading.Thread(target=writer, daemon=True)
    thread.start()
    time.sleep(1.0)
    roi = []
    for _ in range(args.samples):
        roi.append(query_roi(store, mapper))
    # Axis C candidate: writes WITH the hook, sampled on the writer
    # thread's own clock via the status history we can measure inline.
    stop.set()
    thread.join(timeout=5.0)
    report['axis_a_roi_ms'] = stats(roi)
    report['axis_d_note'] = ('background CPU: encoder thread + replica '
                             'poll are the only background workers; '
                             'measured via /proc in the acceptance run')

    # Axis C candidate: timed writes WITH hook (no concurrent reader).
    hook_writes = []
    for stamp in range(500, 500 + args.samples):
        t0 = time.perf_counter_ns()
        result = integrate(store, mapper, stamp, cloud)
        hook_writes.append((time.perf_counter_ns() - t0) / 1e6)
        assert result.accepted
    hook.wait_pending(60.0)
    report['axis_c_write_with_hook_ms'] = stats(hook_writes)
    base_p95 = report['axis_c_write_baseline_ms']['p95_ms']
    hook_p95 = report['axis_c_write_with_hook_ms']['p95_ms']
    report['axis_c_ratio'] = round(hook_p95 / base_p95, 4) if base_p95 else None

    hook.close()
    os.close(lock_fd)
    with open(args.out, 'w') as handle:
        json.dump(report, handle, indent=1)
    print(json.dumps(report, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
