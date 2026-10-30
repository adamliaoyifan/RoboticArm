#!/usr/bin/env python3
"""Paired distance-query benchmark for the W1 optimization campaign.

Median-of-N wall timing of nearest_obstacles on the same scenarios the
bit-exact oracle covers: empty map (RADIUS_CLEAR drain) and 5-box map
(quick match), 256 deterministic points, radius 0.30 m.
"""
import argparse
import json
import statistics
import sys
import time
from pathlib import Path

_WS = Path(__file__).resolve().parents[2]
for entry in (_WS/'src'/'luggage_perception', _WS/'src'/'luggage_description', _WS):
    sys.path.insert(0, str(entry))

import numpy as np

from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)


def scenario(boxes):
    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                               0.02, max_raycast_points=2000)
    store = OccupancyMapStore(mapper, epoch='dist-bench')
    placed = 0
    for z, y, x in ((z, y, x) for z in range(3)
                    for y in (-.48, .18) for x in (-.5, 0., .5)):
        if placed >= boxes:
            break
        store.apply_update(MapUpdate(
            operation_id='b%d' % placed, kind=UpdateKind.ADD_BOX_V1,
            box=((x, y, mapper._hull.floor_z + .11 + .24*z), (.35, .53, .22), 0.),
            box_id='box-%d' % placed))
        placed += 1
    # Converge the map with full-camera-shaped clouds (as the runtime
    # owner does): 30k points from the ceiling carve free space and
    # build occupied surfaces -- without this the map stays empty and
    # the RADIUS_CLEAR drain is not representative (baseline lesson:
    # 6 single-point observations measured 7-15 ms vs the real 52-57).
    for index in range(12):
        rng = np.random.default_rng(70 + index)
        cloud = np.column_stack((
            rng.uniform(-.6, .6, 30000), rng.uniform(-.8, .8, 30000),
            rng.uniform(mapper._hull.floor_z + .1,
                        mapper._hull.floor_z + 1.2, 30000)))
        store.apply_update(MapUpdate(
            operation_id='cloud%d' % index, kind=UpdateKind.INTEGRATE_VIEW,
            points=cloud, origin=(0., 0., mapper._hull.ceiling_z - .1),
            minimum_z=mapper._hull.floor_z + .05, schema_version=1,
            settled=True, geometry_hash=mapper.geometry_hash,
            stamp_key=('c', index), acquisition_stamp_ns=2_000_000+index,
            pose_at_confirm={'position': [0., 0., 0.]}))
    rng = np.random.default_rng(20261031)
    lo = np.asarray((-.6, -.8, mapper._hull.floor_z + .05))
    hi = np.asarray((.6, .8, mapper._hull.floor_z + 1.0))
    return store, rng.uniform(lo, hi, size=(256, 3))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', required=True)
    parser.add_argument('--repeats', type=int, default=15)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    report = {'label': args.label, 'repeats': args.repeats, 'scenarios': {}}
    for tag, boxes in (('b0_radius_clear', 0), ('b5_quick_match', 5)):
        store, points = scenario(boxes)
        with store.acquire_query_snapshot() as handle:
            times = []
            for _ in range(args.repeats):
                begin = time.perf_counter_ns()
                batch = handle.nearest_obstacles(points, 0.30, 'separate')
                times.append((time.perf_counter_ns()-begin)/1e6)
            report['scenarios'][tag] = dict(
                median_ms=round(statistics.median(times), 3),
                p95_ms=round(sorted(times)[int(0.95*len(times))-1], 3),
                max_ms=round(max(times), 3),
                examined=batch.examined_cells, status=batch.status)
    Path(args.out).write_text(json.dumps(report, indent=2, sort_keys=True)+'\n')
    print(json.dumps(report['scenarios']))


if __name__ == '__main__':
    raise SystemExit(main())
