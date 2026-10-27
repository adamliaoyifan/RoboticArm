#!/usr/bin/env python3
"""G4-A gen3 A9 capture on/off pairing (review_three_issues #3).

Same frozen product, same input world, same query kinds; the ONLY
variable is the isolation/capture stack:
- OFF: bare store, direct snapshot handle (no export hook, no replica)
- ON : wired export hook + QueryReplica + read facade (the isolated
  query path, i.e. the capture/evidence machinery engaged)

Per kind (ROI cells / distance / surface): 3 warmup + 30 samples per
mode, nearest-rank P95, gate ratio = on/off <= 1.10 (per kind).
Negative controls for the judge live in test_g3_metrics_judge-style
pytest (n<30 -> not_evaluated; ratio over bound -> FAIL).

Also records the derived-resource ledger at the ON side: export root
physical bytes by category (objects are content-addressed = unique),
registry conservative retained bytes, replica object-cache stats,
1 Hz RSS. Exit 0 pass / 3 fail / 4 not_evaluated.
"""

import argparse
import fcntl
import json
import math
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

from luggage_perception.cargo_map_query import QueryPage
from luggage_perception.cargo_map_query_mode import wire_store_export
from luggage_perception.cargo_query_replica import (
    QueryReplica, ReplicaState)
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)

PERF_LOCK = '/tmp/elfin-occ-backend-perf.lock'
REGION = ((-0.2, -0.2, 0.5), (0.2, 0.2, 0.9))
PASS, FAIL, NOT_EVALUATED = 0, 3, 4


def p95(values, min_samples=30):
    values = sorted(values)
    if len(values) < min_samples:
        return None
    return values[math.ceil(0.95 * len(values)) - 1]


def make_world(resolution, boxes):
    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                               resolution, max_raycast_points=2000)
    store = OccupancyMapStore(mapper, epoch='a9')
    for i, (z, y, x) in enumerate(
            (z, y, x) for z in range(4)
            for y in (-.48, .18) for x in (-.5, 0., .5)):
        if i >= boxes:
            break
        assert store.apply_update(MapUpdate(
            operation_id='b%d' % i, kind=UpdateKind.ADD_BOX_V1,
            box=((x, y, mapper._hull.floor_z + .11 + .24 * z),
                 (.35, .53, .22), 0.), box_id='box-%d' % i)).accepted
    rng = np.random.default_rng(20261023)
    points = np.column_stack([
        rng.uniform(-0.5, 0.5, 20000),
        rng.uniform(-0.6, 0.6, 20000),
        rng.uniform(mapper._hull.floor_z + .1,
                    mapper._hull.floor_z + 1.2, 20000)])
    assert store.apply_update(MapUpdate(
        operation_id='seed', kind=UpdateKind.INTEGRATE_VIEW,
        expected_snapshot=store.identity,
        geometry_hash=mapper.geometry_hash, points=points,
        origin=(0., 0., mapper._hull.ceiling_z - 0.1),
        minimum_z=mapper._hull.floor_z + .05,
        stamp_key=(1, 0), acquisition_stamp_ns=1_000_000_001,
        settled=True, schema_version=1)).accepted
    return store, mapper


def distance_points():
    rng = np.random.default_rng(11)
    return [(float(x), float(y), float(z))
            for x, y, z in rng.uniform(
                (-0.6, -0.6, 0.55), (0.6, 0.6, 1.35), (60, 3))]


def one_query(handle, kind, points):
    if kind == 'roi':
        handle.query_cells(REGION, (0, 1, 2),
                           page=QueryPage(4096, ''))
    elif kind == 'distance':
        handle.nearest_obstacles(points, 0.5, 'separate')
    else:
        handle.surface(REGION)


def sample_path(acquire, kinds, points, warmup, samples):
    import gc
    out = {}
    for kind in kinds:
        times, gc_events = [], []
        gc_before = sum(gc.get_stats()[gen]['collections']
                        for gen in range(3))
        for i in range(warmup + samples):
            handle = acquire()
            t0 = time.perf_counter_ns()
            one_query(handle, kind, points)
            times.append((time.perf_counter_ns() - t0) / 1e6)
            handle.close()
            gc_now = sum(gc.get_stats()[gen]['collections']
                         for gen in range(3))
            gc_events.append(gc_now - gc_before)
            gc_before = gc_now
        # attach: which samples coincided with a GC collection
        flagged = [i for i, delta in enumerate(gc_events[warmup:])
                   if delta > 0]
        out[kind] = {'times_ms': times[warmup:], 'gc_samples': flagged}
    return out


def tree_bytes(path):
    total = 0
    for root, _, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(root, name))
            except OSError:
                pass
    return total


def rss_mb(pid='self'):
    try:
        with open('/proc/%s/status' % pid) as handle:
            for line in handle:
                if line.startswith('VmRSS'):
                    return int(line.split()[1]) / 1024.0
    except OSError:
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--resolution', type=float, default=0.01)
    parser.add_argument('--boxes', type=int, default=20)
    parser.add_argument('--samples', type=int, default=30)
    parser.add_argument('--warmup', type=int, default=3)
    parser.add_argument('--ratio-bound', type=float, default=1.10)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    if os.path.exists(args.out):
        raise SystemExit('refusing to overwrite %s' % args.out)

    lock_fd = os.open(PERF_LOCK, os.O_CREAT | os.O_RDWR)
    fcntl.flock(lock_fd, fcntl.LOCK_EX)

    kinds = ('roi', 'distance', 'surface')
    points = distance_points()

    # OFF: bare store, direct handle (capture/isolation stack disabled)
    store_off, _ = make_world(args.resolution, args.boxes)
    off = sample_path(lambda: store_off.acquire_query_snapshot(),
                      kinds, points, args.warmup, args.samples)
    # Measurement hygiene: release the OFF world BEFORE timing ON --
    # a live OFF heap would inflate the ON side's GC traversal set and
    # fabricate an allocation-tail difference that is not the isolated
    # path's (harness artifact found while diagnosing the ROI tail).
    del store_off
    import gc
    gc.collect()

    # ON: wired hook + replica + facade (same world construction)
    store_on, _ = make_world(args.resolution, args.boxes)
    root = tempfile.mkdtemp(prefix='a9-pair-')
    hook = wire_store_export(store_on, root)
    store_on._snapshot = store_on._build_snapshot()
    hook.wait_pending(120.0)
    replica = QueryReplica(root)
    deadline = time.monotonic() + 120
    while time.monotonic() < deadline:
        if replica.poll() == ReplicaState.READY:
            break
        time.sleep(0.05)
    assert replica.state == ReplicaState.READY, replica.state
    from luggage_perception.cargo_map_read_facade import (
        QueryMapReadFacade)
    facade = QueryMapReadFacade(replica)
    rss_samples = []
    on = sample_path(lambda: facade.acquire_query_snapshot('CURRENT'),
                     kinds, points, args.warmup, args.samples,
                     )

    # derived-resource ledger (ON side)
    ledger = {
        'export_root_bytes': {
            'objects': tree_bytes(os.path.join(root, 'objects')),
            'generations': tree_bytes(os.path.join(root, 'generations')),
            'staging': tree_bytes(os.path.join(root, 'staging')),
            'control': tree_bytes(os.path.join(root, 'control')),
        },
        'writer_registry': dict(
            store_on._query_registry.resource_summary()),
        'replica': {'cache_hits': replica.cache_hits,
                    'cache_misses': replica.cache_misses,
                    'state': str(replica.state)},
        'rss_mb_final': {'writer': rss_mb(), },
    }
    hook.close()
    os.close(lock_fd)

    report = {
        'revision': subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=_WS_SRC,
            text=True).strip(),
        'scope': ('capture on/off pairing: same world construction, '
                  'OFF=bare store handle, ON=wired export+replica+'
                  'facade; per-kind nearest-rank P95, ratio=on/off'),
        'resolution': args.resolution, 'boxes': args.boxes,
        'samples': args.samples, 'kinds': {},
        'resource_ledger': ledger,
        'exit': None,
    }
    verdict = PASS
    for kind in kinds:
        off_times, on_times = off[kind]['times_ms'], on[kind]['times_ms']
        off_p95 = p95(off_times)
        on_p95 = p95(on_times)
        ratio = (round(on_p95 / off_p95, 4)
                 if off_p95 and on_p95 else None)
        entry = {'off_p95_ms': round(off_p95, 3) if off_p95 else None,
                 'on_p95_ms': round(on_p95, 3) if on_p95 else None,
                 'on_max_ms': round(max(on_times), 3),
                 'off_max_ms': round(max(off_times), 3),
                 'ratio': ratio,
                 'median_ratio': round(
                     (sorted(on_times)[len(on_times) // 2]
                      / sorted(off_times)[len(off_times) // 2]), 4),
                 'gc_flagged_samples': {
                     'off': off[kind]['gc_samples'],
                     'on': on[kind]['gc_samples']},
                 'raw': {'off': [round(v, 3) for v in off_times],
                         'on': [round(v, 3) for v in on_times]}}
        if ratio is None:
            entry['verdict'] = NOT_EVALUATED
            if verdict != FAIL:
                verdict = NOT_EVALUATED
        elif ratio > args.ratio_bound:
            entry['verdict'] = FAIL
            verdict = FAIL
        else:
            entry['verdict'] = PASS
        report['kinds'][kind] = entry
    report['exit'] = verdict
    with open(args.out, 'w') as handle:
        json.dump(report, handle, indent=1)
    print(json.dumps({k: {kk: vv for kk, vv in v.items()
                          if kk != 'raw'}
                      for k, v in report['kinds'].items()}, indent=1))
    print('exit', verdict)
    return verdict


if __name__ == '__main__':
    sys.exit(main())
