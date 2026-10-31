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
import unittest.mock
from pathlib import Path

import numpy as np

from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)
from luggage_perception import cargo_distance_query
from luggage_perception.cargo_distance_query import DistanceBudget


class _FrozenClock:
    """A module clock that never advances: the golden scenarios verify
    traversal arithmetic and COUNT budgets, so wall-clock expiry (e.g. a
    cold table build landing inside the first query under load) must not
    be able to flip a scenario's status. elapsed_ns is exempt from the
    golden comparison anyway."""

    now = 0

    @classmethod
    def perf_counter_ns(cls):
        return cls.now

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
    from luggage_perception import distance_kernels_jit
    with unittest.mock.patch.object(cargo_distance_query, 'time', _FrozenClock), \
         unittest.mock.patch.object(distance_kernels_jit, '_clock_ns',
                                    lambda: 0):
        out = _run_scenarios_inner()
    return out


def _run_scenarios_inner():
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


class _FakeModuleClock:
    """Stand-in for the time module inside cargo_distance_query: every
    perf_counter_ns read advances the clock by one step, so a budget
    expires after a fixed number of reads regardless of real time."""

    def __init__(self, step_ns):
        self.step_ns = step_ns
        self.now = 0

    def perf_counter_ns(self):
        self.now += self.step_ns
        return self.now


class _JumpingKernelClock:
    """Stand-in for distance_kernels_jit._clock_ns: each call jumps a
    fixed amount, so the kernel's elapsed measurement exceeds any
    remaining budget after its first stride check."""

    def __init__(self, jump_ns):
        self.jump_ns = jump_ns
        self.calls = 0
        self._real = __import__('time').perf_counter_ns()

    def __call__(self):
        self.calls += 1
        return self._real + self.jump_ns * self.calls


class TestTimeBudgetBoundary(unittest.TestCase):
    """P1 regression (reviews Q-20261031-7): the JIT kernel must honor
    the clock half of the budget WITHIN a point's traversal, at the same
    admission sites as the reference path. Pre-fix, only the count half
    was checked per pop; the clock half ran between points only, so an
    in-point expiry could still return OK."""

    def setUp(self):
        from luggage_perception import cargo_distance_query, distance_kernels_jit
        if not distance_kernels_jit.HAS_NUMBA:
            self.skipTest('numba unavailable')
        self.dq = cargo_distance_query
        self.dkj = distance_kernels_jit
        self.store = build_store(0)   # empty map: RADIUS_CLEAR drains most
        self.points = query_points(self.store.mapper)

    def _single_point_query(self, budget=None):
        with self.store.acquire_query_snapshot() as handle:
            kwargs = {'budget': budget} if budget is not None else {}
            return handle.nearest_obstacles(self.points[244:245], RADIUS,
                                            'separate', **kwargs)

    def test_controlled_clock_parity_both_paths(self):
        """The reviews' controlled-clock repro: 1 ms budget, a clock that
        has run 10 ms past the deadline. Both paths must report the
        point as QUERY_BUDGET_EXHAUSTED."""
        fake = _FakeModuleClock(step_ns=10_000_000)
        budget = self.dq.DistanceBudget(timeout_ms=1.0)
        for use_jit in (False, True):
            with unittest.mock.patch.object(self.dq, '_USE_JIT', use_jit), \
                 unittest.mock.patch.object(self.dq, 'time', fake):
                fake.now = 0
                batch = self._single_point_query(budget)
            self.assertEqual(batch.status, 'QUERY_BUDGET_EXHAUSTED', use_jit)
            self.assertFalse(batch.complete, use_jit)
            self.assertEqual(batch.results[0].status,
                             'QUERY_BUDGET_EXHAUSTED', use_jit)
            self.assertIsNotNone(batch.results[0].occupied, use_jit)

    def test_kernel_clock_expires_within_traversal(self):
        """Isolate the KERNEL admission site: the module clock stays
        early (wrapper never pre-fails, zero_match admits pass) while
        the kernel clock jumps past the remaining budget, so exhaustion
        can only come from the kernel's own stride check."""
        static_early = type('E', (), {'perf_counter_ns': staticmethod(
            lambda: 0)})()
        jumping = _JumpingKernelClock(jump_ns=1_000_000_000)
        budget = self.dq.DistanceBudget(timeout_ms=1.0)
        with unittest.mock.patch.object(self.dq, '_USE_JIT', True), \
             unittest.mock.patch.object(self.dq, 'time', static_early), \
             unittest.mock.patch.object(self.dkj, '_clock_ns', jumping):
            batch = self._single_point_query(budget)
        self.assertGreaterEqual(jumping.calls, 1)   # kernel entered AND checked
        self.assertEqual(batch.status, 'QUERY_BUDGET_EXHAUSTED')
        self.assertEqual(batch.results[0].status, 'QUERY_BUDGET_EXHAUSTED')
        point = batch.results[0]
        exhausted = [m for m in (point.occupied, point.unknown)
                     if m is not None and m.status == 'QUERY_BUDGET_EXHAUSTED']
        self.assertEqual(len(exhausted), 1)
        match = exhausted[0]
        # lower_bound_m is only set by an in-kernel admission failure
        # (zero_match and the between-points fast fail carry no bound),
        # so a finite bound proves the kernel's own clock check fired,
        # with the first pop's key exactly like the reference admits.
        self.assertIsNotNone(match.lower_bound_m)
        self.assertGreaterEqual(match.lower_bound_m, 0.0)
        self.assertGreater(batch.examined_blocks, 0)
        # Sanity: same geometry, real clocks, ample budget -> no
        # exhaustion (the tight-budget result above is a clock effect).
        with unittest.mock.patch.object(self.dq, '_USE_JIT', True):
            control = self._single_point_query()
        self.assertNotEqual(control.status, 'QUERY_BUDGET_EXHAUSTED')

    def test_real_clock_mid_traversal_expiry(self):
        """Self-calibrating real-clock check on both paths: a budget at
        60% of the fastest full-budget elapsed must exhaust mid-query."""
        with unittest.mock.patch.object(self.dq, '_USE_JIT', True):
            for _ in range(4):
                warm = self._single_point_query()
        elapsed = [self._single_point_query().elapsed_ns for _ in range(3)]
        tight_ns = 0.6 * min(elapsed)
        budget = self.dq.DistanceBudget(timeout_ms=tight_ns / 1e6)
        for use_jit in (False, True):
            with unittest.mock.patch.object(self.dq, '_USE_JIT', use_jit):
                batch = self._single_point_query(budget)
            self.assertEqual(batch.status, 'QUERY_BUDGET_EXHAUSTED', use_jit)
            self.assertEqual(batch.results[0].status,
                             'QUERY_BUDGET_EXHAUSTED', use_jit)

    def test_pre_kernel_gap_spent_against_budget(self):
        """Reviews' repros (rounds 2 and 3): a 10 ms delay landing
        anywhere between the deadline-setting read and the kernel's
        clock reads must be SPENT against the budget. Round-2 pre-fix,
        the kernel re-based its elapsed at entry so a constant delay
        cancelled; round-3 pre-fix, a re-anchoring read added the
        remaining budget onto a LATER clock, so a delay between the two
        dispatch reads was stolen outright (reproduced: warm kernel,
        static module clock, delayed anchor read, 1 ms budget -> OK).
        The deadline is now passed through UNCHANGED, and every kernel
        read is 10 ms late against it, so the first stride check
        trips."""
        import time as _time

        class _DelayedKernelClock:
            def __call__(self):
                return _time.perf_counter_ns() + 10_000_000

        static_early = type('E', (), {'perf_counter_ns': staticmethod(
            lambda: 0)})()
        budget = self.dq.DistanceBudget(timeout_ms=1.0)
        with unittest.mock.patch.object(self.dq, '_USE_JIT', True), \
             unittest.mock.patch.object(self.dq, 'time', static_early), \
             unittest.mock.patch.object(self.dkj, '_clock_ns',
                                        _DelayedKernelClock()):
            batch = self._single_point_query(budget)
        self.assertEqual(batch.status, 'QUERY_BUDGET_EXHAUSTED')
        point = batch.results[0]
        exhausted = [m for m in (point.occupied, point.unknown)
                     if m is not None and m.status == 'QUERY_BUDGET_EXHAUSTED']
        self.assertTrue(exhausted)
        self.assertIsNotNone(exhausted[0].lower_bound_m)

    def test_pre_expired_bound_is_true_minimum(self):
        """Reviews' second-round repro: the pre-expiry bound must be the
        first POPPED key -- the minimum lower over admitted candidates --
        not cand_lowers[0] (candidates are ordered by block id; the
        measured bug returned squared bound 0.0820857 where the true
        minimum is 0). Kernel-level, with the reviewer's numbers."""
        import numpy as np
        from luggage_perception import distance_kernels_jit as dkj
        point = np.zeros(3)
        cand_blocks = np.array([0, 1], dtype=np.int64)
        cand_lowers = np.array([0.0820857, 0.0])   # block order, NOT min-first
        blk_leaf_base = np.array([0, 1, 2], dtype=np.int64)
        leaf_low = np.zeros((2, 3))
        leaf_high = np.ones((2, 3))
        gstart = np.array([0, 1], dtype=np.int64)
        gend = np.array([1, 2], dtype=np.int64)
        mem_low = np.zeros((2, 3))
        mem_high = np.ones((2, 3))
        mem_flat = np.array([5, 7], dtype=np.int64)
        cells = np.zeros(1, dtype=np.int64)
        blocks = np.zeros(1, dtype=np.int64)
        scratch_bounds = np.zeros((2, 2))
        scratch_order = np.zeros((2, 4), dtype=np.int64)
        status, best_sq, bound_sq, *_ = dkj.kernel_nearest(
            point, 1.0, cand_blocks, cand_lowers, blk_leaf_base,
            leaf_low, leaf_high, gstart, gend, mem_low, mem_high,
            mem_flat, 2_000_000, 50_000,
            0,   # deadline long past in this clock domain
            cells, blocks, scratch_bounds, scratch_order)
        self.assertEqual(status, dkj.STATUS_BUDGET)
        # The first popped key is the TRUE minimum (0.0), not
        # cand_lowers[0] (0.0820857).
        self.assertEqual(bound_sq, 0.0)
        # Reference parity: failed at the first admission, nothing charged.
        self.assertEqual(int(cells[0]), 0)
        self.assertEqual(int(blocks[0]), 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
