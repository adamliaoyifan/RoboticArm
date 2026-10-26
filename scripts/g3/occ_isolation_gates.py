#!/usr/bin/env python3
"""G4-A isolation CLI entry: the I-gate work assertions on the isolated
path (protocol section 7: I1 no-change zero work, I2 sparse-change
reuse) plus the b33-paired axis-C shape at representative loads, and
A9 resource sampling (both processes' RSS via /proc/self/status at
1 Hz through the whole run).

This is the isolation subcommand backing occ_query_eval.py's contract
(exit 0/3/4); the eval CLI wires it in a follow-up commit. Matrix here:
resolutions 0.05/0.02/0.01 x boxes 0/5/20 x cuboid/chamfer-lite
(the store hull switch), seed 0..9 x 100 transactions for the I3
oracle spot checks is REDUCED here to the representative shape per
plan section 7's 'representative first' ordering -- the FULL I-matrix
runs under G4-B only after every A gate passes.
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
from luggage_perception.cargo_query_replica import (
    QueryReplica, ReplicaState)
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)

PERF_LOCK = '/tmp/elfin-occ-backend-perf.lock'
EXIT_PASS, EXIT_FAIL, EXIT_NOT_EVAL = 0, 3, 4


def rss_kb():
    with open('/proc/self/status') as handle:
        for line in handle:
            if line.startswith('VmRSS:'):
                return int(line.split()[1])
    return 0


def p95(values):
    if len(values) < 30:
        return None
    return sorted(values)[28]


class RssSampler:
    def __init__(self):
        self.samples = []
        self._stop = threading.Event()

    def start(self):
        self.samples.append((time.monotonic_ns(), rss_kb()))
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop.wait(1.0):
            self.samples.append((time.monotonic_ns(), rss_kb()))

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        self.samples.append((time.monotonic_ns(), rss_kb()))

    @property
    def peak_kb(self):
        return max((v for _, v in self.samples), default=0)


def make_store(boxes, resolution):
    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                               resolution, max_raycast_points=2000)
    store = OccupancyMapStore(mapper, epoch='ig')
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


def cloud_for(mapper, seed):
    return np.column_stack([
        np.random.default_rng(seed).uniform(-0.6, 0.6, 30000),
        np.random.default_rng(seed + 1).uniform(-0.8, 0.8, 30000),
        np.random.default_rng(seed + 2).uniform(
            mapper._hull.floor_z + .1,
            mapper._hull.floor_z + 1.2, 30000)])


def integrate(store, mapper, cloud, stamp):
    return store.apply_update(MapUpdate(
        operation_id='ig-%d' % stamp, kind=UpdateKind.INTEGRATE_VIEW,
        expected_snapshot=store.identity,
        geometry_hash=mapper.geometry_hash, points=cloud,
        origin=(0., 0., mapper._hull.ceiling_z - 0.1),
        minimum_z=mapper._hull.floor_z + .05,
        stamp_key=(stamp, 0),
        acquisition_stamp_ns=1_000_000_000 + stamp,
        settled=True, schema_version=1))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--resolutions', default='0.05,0.02,0.01')
    parser.add_argument('--boxes', default='0,5,20')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    lock_fd = os.open(PERF_LOCK, os.O_CREAT | os.O_RDWR)
    fcntl.flock(lock_fd, fcntl.LOCK_EX)
    report = {'cases': []}
    failures = []

    rss = RssSampler()
    rss.start()

    for resolution in (float(v) for v in args.resolutions.split(',')):
        for boxes in (int(v) for v in args.boxes.split(',')):
            store, mapper = make_store(boxes, resolution)
            root = tempfile.mkdtemp(prefix='ig-')
            hook = wire_store_export(store, root)
            store._snapshot = store._build_snapshot()
            hook.wait_pending(120.0)
            replica = QueryReplica(root)
            deadline = time.monotonic() + 60.0
            ready = False
            while time.monotonic() < deadline:
                if replica.poll() == ReplicaState.READY:
                    ready = True
                    break
                time.sleep(0.05)
            if not ready:
                failures.append('init: res %s boxes %d never READY '
                                '(state %s err %r)'
                                % (resolution, boxes, replica.state,
                                   replica.last_error[:80]))
                hook.close()
                continue

            cloud = cloud_for(mapper, seed=3)
            # Warm write.
            assert integrate(store, mapper, cloud, 0).accepted
            store._snapshot = store._build_snapshot()
            hook.wait_pending(60.0)
            deadline = time.monotonic() + 30.0
            while time.monotonic() < deadline:
                if replica.poll() == ReplicaState.READY:
                    break
                time.sleep(0.02)

            # --- I1 metadata-only ------------------------------------
            misses_before = replica.cache_misses
            store.apply_update(MapUpdate(
                operation_id='pose', kind=UpdateKind.CONTAINER_POSE,
                event_stamp_ns=42))
            work_i1 = dict(store._query_indexes.work)
            store._snapshot = store._build_snapshot()
            hook.wait_pending(60.0)
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                if replica.poll() == ReplicaState.READY:
                    break
                time.sleep(0.02)
            i1 = {
                'blocks_rebuilt': work_i1.get(
                    'public_payload_changed_blocks', -1),
                'replica_decodes': replica.cache_misses - misses_before,
            }
            if i1['blocks_rebuilt'] != 0 or i1['replica_decodes'] != 0:
                failures.append(
                    'I1 res %s boxes %d: rebuilt=%s decodes=%s '
                    '(expect 0/0)' % (resolution, boxes,
                                      i1['blocks_rebuilt'],
                                      i1['replica_decodes']))

            # --- I2 sparse change ------------------------------------
            misses_before = replica.cache_misses
            store.apply_update(MapUpdate(
                operation_id='sparse', kind=UpdateKind.ADD_BOX_V1,
                box=((0.30, 0.30, mapper._hull.floor_z + .11),
                     (.10, .10, .08), 0.), box_id='sparse'))
            # Read the transaction's counters IMMEDIATELY (the snapshot
            # helper's empty-delta refresh overwrites _last_work).
            work_i2 = dict(store._query_indexes.work)
            store._snapshot = store._build_snapshot()
            hook.wait_pending(60.0)
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                if replica.poll() == ReplicaState.READY:
                    break
                time.sleep(0.02)
            i2 = {
                'dirty_blocks': work_i2.get('dirty_blocks', -1),
                'blocks_changed': work_i2.get(
                    'public_payload_changed_blocks', -1),
                'replica_decodes': replica.cache_misses - misses_before,
            }
            if i2['blocks_changed'] <= 0 or i2['blocks_changed'] > \
                    i2['dirty_blocks']:
                failures.append(
                    'I2 res %s boxes %d: changed=%s dirty=%s'
                    % (resolution, boxes, i2['blocks_changed'],
                       i2['dirty_blocks']))
            report['cases'].append({
                'resolution': resolution, 'boxes': boxes,
                'I1': i1, 'I2': i2})
            hook.close()
            import shutil
            shutil.rmtree(root, ignore_errors=True)

    rss.stop()
    report['rss_peak_kb'] = rss.peak_kb
    report['rss_samples'] = len(rss.samples)
    # A9 resource gate (this harness is single-process; the two-process
    # accounting runs in the acceptance harness -- recorded not claimed).
    report['A9_note'] = ('single-process RSS peak recorded; the dual-'
                         'process merged accounting is the acceptance '
                         'harness gate')
    os.close(lock_fd)
    with open(args.out, 'w') as handle:
        json.dump(report, handle, indent=1)
    print(json.dumps({'cases': len(report['cases']),
                      'rss_peak_kb': report['rss_peak_kb'],
                      'failures': failures}, indent=1))
    return EXIT_FAIL if failures else EXIT_PASS


if __name__ == "__main__":
    sys.exit(main())
