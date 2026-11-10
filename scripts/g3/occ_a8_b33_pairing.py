#!/usr/bin/env python3
"""G4-A A8: b33-frozen-pairing write gate.

Runs the IDENTICAL initial state + operation sequence in two subprocess
trees -- b33dda91 (production oracle, pre-derived-index write path) and
the candidate (isolated mode with the export hook) -- and compares the
30-sample write-call P95 (nearest-rank sorted[28]) and max. Gate: P95
ratio <= 1.10, max <= b33max * 1.20 + 20 ms.

Each subprocess prints one JSON line per timed write; the parent
aggregates. Both runs hold the perf flock via the parent.

v2 (review_64f0bdf0 Finding 2): the report embeds each tree's git
revision / commit time / dirty state captured AT RUN TIME and the
full per-sample arrays with the exact nearest-rank index, so the
summary is traceable without backfilling from a later HEAD.
"""

import argparse
import datetime
import fcntl
import json
import math
import os
import subprocess
import sys
import tempfile

PERF_LOCK = '/tmp/elfin-occ-backend-perf.lock'

WORKER = r'''
import json, sys, time
root_tree = sys.argv[1]
boxes = int(sys.argv[2]); resolution = float(sys.argv[3])
samples = int(sys.argv[4]); warm = int(sys.argv[5])
outside = float(sys.argv[6]); crossing = float(sys.argv[7]) > 0
sys.path.insert(0, root_tree + "/src/luggage_perception")
sys.path.insert(0, root_tree + "/src/luggage_description")
sys.path.insert(0, root_tree)
sys.path.insert(0, root_tree + "/src")
import numpy as np
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import MapUpdate, OccupancyMapStore, UpdateKind

mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                           resolution, max_raycast_points=2000)
store = OccupancyMapStore(mapper, epoch='a8')
for i, (z, y, x) in enumerate(
        (z, y, x) for z in range(4)
        for y in (-.48, .18) for x in (-.5, 0., .5)):
    if i >= boxes: break
    r = store.apply_update(MapUpdate(operation_id='b%d' % i,
        kind=UpdateKind.ADD_BOX_V1,
        box=((x, y, mapper._hull.floor_z + .11 + .24 * z),
             (.35, .53, .22), 0.), box_id='box-%d' % i))
    assert r.accepted

CANDIDATE = "candidate" in sys.argv
NOHOOK = "nohook" in sys.argv
if CANDIDATE and not NOHOOK:
    from luggage_perception.cargo_map_query_mode import wire_store_export
    import tempfile as _tf
    hook = wire_store_export(store, _tf.mkdtemp(prefix='a8-'))
    store._snapshot = store._build_snapshot()
    hook.wait_pending(120.0)

# FROZEN workload generator (compare_occ_crop.workload semantics,
# review_8a372dee P2: the A8 gate measures the protocol's
# outside/crossing profiles, not an all-inside uniform cloud):
# inside points span the container; the outside fraction sits either
# far away (crossing 0) or just beyond the wall so rays from the
# external origin CROSS the container (crossing .5).
def frozen_cloud(n, outside_fraction, crossing_capable):
    rng = np.random.default_rng(20261023)
    inside_n = int(round(n * (1 - outside_fraction)))
    inside = rng.uniform([-.60, -.6, .7], [.60, .4, 1.7], (inside_n, 3))
    if crossing_capable:
        external = rng.uniform([1.1, -.6, .7], [1.6, .4, 1.7],
                               (n - inside_n, 3))
    else:
        external = rng.uniform([-2.5, -.6, .7], [-1.5, .4, 1.7],
                               (n - inside_n, 3))
    points = np.concatenate([inside, external])
    rng.shuffle(points)
    return points

cloud = frozen_cloud(30000, outside, crossing)
ORIGIN = (-1.2, 0., 1.1)          # external sensor, as frozen

def integrate(stamp):
    return store.apply_update(MapUpdate(
        operation_id='a8-%d' % stamp, kind=UpdateKind.INTEGRATE_VIEW,
        expected_snapshot=store.identity,
        geometry_hash=mapper.geometry_hash, points=cloud,
        origin=ORIGIN,
        minimum_z=mapper._hull.floor_z + .05,
        stamp_key=(stamp, 0),
        acquisition_stamp_ns=1_000_000_000 + stamp,
        settled=True, schema_version=1))

for stamp in range(warm):
    assert integrate(stamp).accepted
for stamp in range(warm, warm + samples):
    t0 = time.perf_counter_ns()
    assert integrate(stamp).accepted
    print(json.dumps({"ms": (time.perf_counter_ns() - t0) / 1e6}), flush=True)
'''


def run_tree(tree, boxes, resolution, samples, candidate, nohook=False,
             outside=0.5, crossing=0.0):
    extra = ('nohook' if nohook else
             ('candidate' if candidate else 'baseline'))
    result = subprocess.run(
        [sys.executable, '-c', WORKER, tree, str(boxes),
         str(resolution), str(samples), '3', str(outside),
         str(crossing), extra],
        capture_output=True, text=True, timeout=3600)
    if result.returncode != 0:
        raise RuntimeError('%s worker failed: %s'
                           % (tree, result.stderr[-400:]))
    return [json.loads(line)['ms'] for line in
            result.stdout.strip().splitlines() if line.startswith('{')]


def p95(values):
    """Nearest-rank P95: ceil(0.95*n)-1 (sorted[28] only when n=30)."""
    return sorted(values)[math.ceil(0.95 * len(values)) - 1] \
        if len(values) >= 30 else None


def tree_state(tree):
    """Revision binding captured AT RUN TIME (review_64f0bdf0 #2):
    HEAD, commit time, and dirty state of a tree, recorded in the
    report so the summary is self-contained -- no post-hoc HEAD
    backfill. Failure to read git leaves explicit nulls, not blanks."""
    def git(*args):
        return subprocess.run(
            ['git', '-C', tree] + list(args),
            capture_output=True, text=True, timeout=30,
        ).stdout.strip()
    dirty = git('status', '--porcelain')
    return {
        'tree': tree,
        'revision': git('rev-parse', 'HEAD') or None,
        'commit_time': git('log', '-1', '--format=%cI') or None,
        'dirty': bool(dirty),
        'dirty_files': dirty.splitlines()[:10],
        'state_time': datetime.datetime.now().astimezone().isoformat(),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--candidate-tree', required=True)
    parser.add_argument('--baseline-tree', default='/tmp/elfin-b33-pairing')
    parser.add_argument('--boxes', type=int, default=0)
    parser.add_argument('--resolution', type=float, default=0.01)
    parser.add_argument('--samples', type=int, default=30)
    parser.add_argument('--outside', type=float, default=None,
                        help='frozen outside fraction; default: .5 for '
                             '0 boxes, .9 for 20 boxes (plan 7.2)')
    parser.add_argument('--crossing', type=float, default=None,
                        help='frozen crossing profile; default: 0 for '
                             '0 boxes, .5 for 20 boxes')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    outside = (args.outside if args.outside is not None
               else (0.5 if args.boxes == 0 else 0.9))
    crossing = (args.crossing if args.crossing is not None
                else (0.0 if args.boxes == 0 else 0.5))

    lock_fd = os.open(PERF_LOCK, os.O_CREAT | os.O_RDWR)
    fcntl.flock(lock_fd, fcntl.LOCK_EX)
    try:
        trees = {'baseline': tree_state(args.baseline_tree),
                 'candidate': tree_state(args.candidate_tree)}
        baseline = run_tree(args.baseline_tree, args.boxes,
                            args.resolution, args.samples, False,
                            outside, crossing)
        nohook = run_tree(args.candidate_tree, args.boxes,
                          args.resolution, args.samples, False,
                          nohook=True, outside=outside,
                          crossing=crossing)
        candidate = run_tree(args.candidate_tree, args.boxes,
                             args.resolution, args.samples, True,
                             outside, crossing)
    finally:
        os.close(lock_fd)

    b_p95, c_p95 = p95(baseline), p95(candidate)
    n_p95 = p95(nohook)
    b_max, c_max = max(baseline), max(candidate)
    ratio = round(c_p95 / b_p95, 4) if b_p95 else None
    nohook_ratio = round(n_p95 / b_p95, 4) if (b_p95 and n_p95) else None
    max_gate = round(b_max * 1.20 + 20.0, 3)
    passed = (ratio is not None and ratio <= 1.10
              and c_max <= max_gate)
    report = {
        'baseline_tree': args.baseline_tree,
        'candidate_tree': args.candidate_tree,
        'trees': trees,
        'boxes': args.boxes, 'resolution': args.resolution,
        'samples_per_tree': args.samples,
        'nearest_rank': {'rule': 'ceil(0.95*n)-1 on sorted samples',
                         'index_at_n': math.ceil(
                             0.95 * args.samples) - 1},
        'workload': {'points': 30000, 'outside': outside,
                     'crossing': crossing,
                     'generator': 'frozen (compare_occ_crop.workload '
                                  'semantics, rng 20261023, external '
                                  'origin)'},
        'baseline_p95_ms': round(b_p95, 3) if b_p95 else None,
        'candidate_p95_ms': round(c_p95, 3) if c_p95 else None,
        'ratio': ratio,
        'nohook_p95_ms': round(n_p95, 3) if n_p95 else None,
        'nohook_ratio': nohook_ratio,
        'baseline_max_ms': round(b_max, 3),
        'candidate_max_ms': round(c_max, 3),
        'max_gate_ms': max_gate,
        'gate_pass': passed,
        'raw_samples_ms': {
            'warmup_per_tree': 3,
            'baseline': [round(v, 3) for v in baseline],
            'nohook': [round(v, 3) for v in nohook],
            'candidate': [round(v, 3) for v in candidate]},
    }
    with open(args.out, 'w') as handle:
        json.dump(report, handle, indent=1)
    print(json.dumps(report, indent=1))
    return 0 if passed else 3


if __name__ == "__main__":
    sys.exit(main())
