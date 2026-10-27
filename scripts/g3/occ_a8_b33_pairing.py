#!/usr/bin/env python3
"""G4-A A8: b33-frozen-pairing write gate.

Runs the IDENTICAL initial state + operation sequence in two subprocess
trees -- b33dda91 (production oracle, pre-derived-index write path) and
the candidate (isolated mode with the export hook) -- and compares the
30-sample write-call P95 (nearest-rank sorted[28]) and max. Gate: P95
ratio <= 1.10, max <= b33max * 1.20 + 20 ms.

Each subprocess prints one JSON line per timed write; the parent
aggregates. Both runs hold the perf flock via the parent.
"""

import argparse
import fcntl
import json
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

cloud = np.column_stack([
    np.random.default_rng(3).uniform(-0.6, 0.6, 30000),
    np.random.default_rng(4).uniform(-0.8, 0.8, 30000),
    np.random.default_rng(5).uniform(
        mapper._hull.floor_z + .1, mapper._hull.floor_z + 1.2, 30000)])

def integrate(stamp):
    return store.apply_update(MapUpdate(
        operation_id='a8-%d' % stamp, kind=UpdateKind.INTEGRATE_VIEW,
        expected_snapshot=store.identity,
        geometry_hash=mapper.geometry_hash, points=cloud,
        origin=(0., 0., mapper._hull.ceiling_z - 0.1),
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


def run_tree(tree, boxes, resolution, samples, candidate, nohook=False):
    extra = ('nohook' if nohook else
             ('candidate' if candidate else 'baseline'))
    result = subprocess.run(
        [sys.executable, '-c', WORKER, tree, str(boxes),
         str(resolution), str(samples), '3', extra],
        capture_output=True, text=True, timeout=3600)
    if result.returncode != 0:
        raise RuntimeError('%s worker failed: %s'
                           % (tree, result.stderr[-400:]))
    return [json.loads(line)['ms'] for line in
            result.stdout.strip().splitlines() if line.startswith('{')]


def p95(values):
    """Nearest-rank P95: ceil(0.95*n)-1 (sorted[28] only when n=30)."""
    import math
    return sorted(values)[math.ceil(0.95 * len(values)) - 1] \
        if len(values) >= 30 else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--candidate-tree', required=True)
    parser.add_argument('--baseline-tree', default='/tmp/elfin-b33-pairing')
    parser.add_argument('--boxes', type=int, default=0)
    parser.add_argument('--resolution', type=float, default=0.01)
    parser.add_argument('--samples', type=int, default=30)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    lock_fd = os.open(PERF_LOCK, os.O_CREAT | os.O_RDWR)
    fcntl.flock(lock_fd, fcntl.LOCK_EX)
    try:
        baseline = run_tree(args.baseline_tree, args.boxes,
                            args.resolution, args.samples, False)
        nohook = run_tree(args.candidate_tree, args.boxes,
                          args.resolution, args.samples, False, nohook=True)
        candidate = run_tree(args.candidate_tree, args.boxes,
                             args.resolution, args.samples, True)
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
        'boxes': args.boxes, 'resolution': args.resolution,
        'baseline_p95_ms': round(b_p95, 3) if b_p95 else None,
        'candidate_p95_ms': round(c_p95, 3) if c_p95 else None,
        'ratio': ratio,
        'nohook_p95_ms': round(n_p95, 3) if n_p95 else None,
        'nohook_ratio': nohook_ratio,
        'baseline_max_ms': round(b_max, 3),
        'candidate_max_ms': round(c_max, 3),
        'max_gate_ms': max_gate,
        'gate_pass': passed,
    }
    with open(args.out, 'w') as handle:
        json.dump(report, handle, indent=1)
    print(json.dumps(report, indent=1))
    return 0 if passed else 3


if __name__ == "__main__":
    sys.exit(main())
