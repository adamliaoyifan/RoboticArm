"""Bit-exact oracle for distance-query optimization (W1 A/B/C campaign).

The golden file records the complete observable output of
``nearest_obstacles`` -- per-point statuses, distances, matched cell
records, examined counters, batch status -- for deterministic scenarios
that cover every algorithmic branch: RADIUS_CLEAR drain (no cargo),
quick-match (nearby boxes), grid-plane boundary points, both unknown
policies, and a count-based budget-exhaustion boundary. Any query-side
optimization must reproduce these values exactly (float.hex for
distances); only elapsed_ns is exempt. Regen the golden with
DISTANCE_ORACLE_REGEN=1 against the unoptimized product only.
"""
import dataclasses
import json
import os
import unittest
from pathlib import Path

import numpy as np

from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)
from luggage_perception.cargo_distance_query import DistanceBudget

GOLDEN = Path(__file__).with_suffix('.golden.json')
RESOLUTION = 0.02
RADIUS = 0.30


def build_store(boxes):
    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                               RESOLUTION, max_raycast_points=2000)
    store = OccupancyMapStore(mapper, epoch='dist-oracle')
    placed = 0
    for z, y, x in ((z, y, x) for z in range(3)
                    for y in (-.48, .18) for x in (-.5, 0., .5)):
        if placed >= boxes:
            break
        result = store.apply_update(MapUpdate(
            operation_id='b%d' % placed, kind=UpdateKind.ADD_BOX_V1,
            box=((x, y, mapper._hull.floor_z + .11 + .24*z), (.35, .53, .22), 0.),
            box_id='box-%d' % placed))
        assert result.accepted
        placed += 1
    for index in range(6):
        voxel = (5 + 3*index, 11 + 5*index, 3 + 2*index)
        point = mapper._local_to_world(*mapper._voxel_center_local(*voxel))
        store.apply_update(MapUpdate(
            operation_id='v%d' % index, kind=UpdateKind.INTEGRATE_VIEW,
            points=(point,), origin=(0., 0., .4), schema_version=1,
            settled=True, geometry_hash=mapper.geometry_hash,
            stamp_key=('o', index), acquisition_stamp_ns=1_000_000+index,
            pose_at_confirm={'position': [0., 0., 0.]}))
    return store


def query_points(mapper):
    """256 deterministic points: 240 uniform central + boundary specials
    generated through the mapper's own grid helpers so grid-plane cases
    are exactly aligned."""

    def voxel_center(vx):
        return np.asarray(mapper._local_to_world(*mapper._voxel_center_local(*vx)))

    rng = np.random.default_rng(20261031)
    lo = np.asarray((-.6, -.8, mapper._hull.floor_z + .05))
    hi = np.asarray((.6, .8, mapper._hull.floor_z + 1.0))
    points = rng.uniform(lo, hi, size=(240, 3))
    step = mapper.resolution
    base = voxel_center((7, 9, 3))
    specials = np.asarray([
        base,                                             # interior grid-center
        base + np.array([step/2, 0., 0.]),                # exact grid plane
        voxel_center((0, 0, 0)),                          # near-min corner cell
        voxel_center((mapper.nx-1, mapper.ny-1, mapper.nz-1)),  # near-max cell
        np.asarray((0., 0., mapper._hull.floor_z + .30)), # central axis (RADIUS_CLEAR prone)
        np.asarray((-2.0, 0., .5)),                       # far outside map
        np.asarray((0., 0., 5.0)),                        # outside above hull
        base + np.array([1e-7, 0., 0.]),                  # off-center epsilon
        base - np.array([1e-7, 0., 0.]),
        voxel_center((16, 16, 16)),                       # interior grid node
        voxel_center((16, 16, 16)) + np.array([step*.5, step*.5, 0.]),
        voxel_center((16, 16, 16)) - np.array([step*.5, step*.5, 0.]),
        np.asarray((.95, .0, .5)),                        # hull boundary probe
        np.asarray((-.95, .0, .5)),
        np.asarray((0., .8, .4)),
        voxel_center((7, 9, 3)) + np.array([0., step*.5, step*.5]),
    ])
    return np.concatenate((points, specials))[:256]


def project(batch):
    """Comparable projection: everything except elapsed_ns."""

    def match(m):
        if m is None:
            return None
        record = m.cell
        return dict(status=m.status, exact=m.exact,
                    distance_m=None if m.distance_m is None else float(m.distance_m).hex(),
                    lower_bound_m=None if m.lower_bound_m is None else float(m.lower_bound_m).hex(),
                    cell=None if record is None else [
                        None if v is None else (float(v).hex() if isinstance(v, float) else str(v))
                        for v in dataclasses.astuple(record)])

    def point(p):
        return dict(point=[float(v).hex() for v in p.point], status=p.status,
                    occupied=match(p.occupied), unknown=match(p.unknown))

    return dict(status=batch.status, reason=batch.reason,
                complete=batch.complete, returned_count=batch.returned_count,
                examined_cells=batch.examined_cells,
                examined_blocks=batch.examined_blocks,
                unknown_policy=batch.unknown_policy,
                results=[point(p) for p in batch.results])


def run_scenarios():
    out = {}
    for tag, boxes in (('b0_radium_clear', 0), ('b5_quick_match', 5)):
        store = build_store(boxes)
        points = query_points(store.mapper)
        with store.acquire_query_snapshot() as handle:
            for policy in ('separate', 'occupied_only'):
                batch = handle.nearest_obstacles(points, RADIUS, policy)
                out['%s/%s' % (tag, policy)] = project(batch)
            batch = handle.nearest_obstacles(points[:32], RADIUS, 'separate',
                                             budget=DistanceBudget(
                                                 max_examined_cells=3000,
                                                 max_examined_blocks=64))
            out['%s/budget_3k' % tag] = project(batch)
            batch = handle.nearest_obstacles(points[:8], 0.05, 'separate')
            out['%s/radius_5cm' % tag] = project(batch)
    return out


class TestDistanceBatchOracle(unittest.TestCase):

    def test_matches_golden_exactly(self):
        scenarios = run_scenarios()
        if os.environ.get('DISTANCE_ORACLE_REGEN'):
            GOLDEN.write_text(json.dumps(scenarios, indent=1, sort_keys=True)+'\n')
            self.skipTest('golden regenerated')
        golden = json.loads(GOLDEN.read_text())
        self.assertEqual(sorted(golden), sorted(scenarios))
        for name in sorted(golden):
            self.assertEqual(golden[name], scenarios[name],
                             'bit-exact divergence in scenario %s' % name)


if __name__ == '__main__':
    unittest.main(verbosity=2)
