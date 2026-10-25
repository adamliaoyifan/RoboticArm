#!/usr/bin/env python3
"""Vectorized frontier-count oracle tests (ADMISSION-DIAG 2026-10-25).

The vectorized ``frontier_count`` must equal the historical pure-Python
``_frontier_indices`` predicate on random and edge grids (unknown AND
active cell with any observed 6-neighbour, boundary neighbours skipped),
and must run in sub-second time on grid shapes whose old loop cost
seconds -- that cost held the process GIL long enough to expire the
readiness lease of identity peers.
"""

import os
import random
import sys
import time
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(PKG_ROOT, "src", "luggage_perception"))
sys.path.insert(0, os.path.join(PKG_ROOT, "src"))

from luggage_perception.cargo_volume_mapper import (  # noqa: E402
    FREE, OCCUPIED, UNKNOWN, CargoVolumeMapper)


class _StubMapper:
    """Minimal field set both the loop predicate and _arrays() need."""

    def __init__(self, nx, ny, nz, grid, active):
        self.nx, self.ny, self.nz = nx, ny, nz
        self._grid = bytearray(grid)
        self._active = bytearray(active)

    def _index(self, ix, iy, iz):
        return ix + self.nx * (iy + self.ny * iz)

    def _has_observed_neighbor(self, ix, iy, iz):
        for dx, dy, dz in (
                (-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0),
                (0, 0, -1), (0, 0, 1)):
            wx, wy, wz = ix + dx, iy + dy, iz + dz
            if (wx < 0 or wy < 0 or wz < 0
                    or wx >= self.nx or wy >= self.ny or wz >= self.nz):
                continue
            if self._grid[self._index(wx, wy, wz)] in (FREE, OCCUPIED):
                return True
        return False


def _loop_frontier_count(mapper):
    """The historical predicate, verbatim (the oracle)."""
    count = 0
    for ix in range(mapper.nx):
        for iy in range(mapper.ny):
            for iz in range(mapper.nz):
                idx = mapper._index(ix, iy, iz)
                if mapper._active is not None and not mapper._active[idx]:
                    continue
                if mapper._grid[idx] != UNKNOWN:
                    continue
                if mapper._has_observed_neighbor(ix, iy, iz):
                    count += 1
    return count


def _vectorized(mapper):
    return CargoVolumeMapper.frontier_count(mapper)


def _random_stub(nx, ny, nz, rng, unknown_bias=0.8, active_bias=0.9):
    total = nx * ny * nz
    grid = [UNKNOWN if rng.random() < unknown_bias
            else rng.choice((FREE, OCCUPIED)) for _ in range(total)]
    active = [1 if rng.random() < active_bias else 0 for _ in range(total)]
    return _StubMapper(nx, ny, nz, grid, active)


class TestFrontierCountVectorized(unittest.TestCase):

    def test_random_grids_match_loop_oracle(self):
        rng = random.Random(20261025)
        for _ in range(40):
            nx, ny, nz = (rng.randint(1, 8) for _ in range(3))
            stub = _random_stub(nx, ny, nz, rng)
            self.assertEqual(
                _vectorized(stub), _loop_frontier_count(stub),
                (nx, ny, nz))

    def test_edge_grids(self):
        # All unknown, all active: no observed neighbour anywhere.
        stub = _StubMapper(3, 3, 3, [UNKNOWN] * 27, [1] * 27)
        self.assertEqual(_vectorized(stub), 0)
        # Single observed core voxel: its 6 unknown neighbours count.
        grid = [UNKNOWN] * 27
        grid[_StubMapper(3, 3, 3, grid, [1] * 27)._index(1, 1, 1)] = FREE
        stub = _StubMapper(3, 3, 3, grid, [1] * 27)
        self.assertEqual(_vectorized(stub), 6)
        # Inactive unknown cells never count, even with neighbours.
        active = [0] * 27
        stub = _StubMapper(3, 3, 3, grid, active)
        self.assertEqual(_vectorized(stub), 0)
        # Boundary cells see only in-grid neighbours (no wrap-around).
        grid = [FREE] * 27
        grid[0] = UNKNOWN  # corner
        stub = _StubMapper(3, 3, 3, grid, [1] * 27)
        self.assertEqual(_vectorized(stub), 1)

    def test_large_grid_speed(self):
        # 149x197x144-shaped small multiple: the old loop cost ~seconds
        # per call at this scale; the vectorized twin must stay fast.
        nx, ny, nz = 150, 99, 72  # ~1.07M voxels
        rng = random.Random(7)
        stub = _random_stub(nx, ny, nz, rng)
        t0 = time.perf_counter()
        _vectorized(stub)
        dt = time.perf_counter() - t0
        self.assertLess(dt, 1.0, "vectorized frontier count must be "
                                 "sub-second at 1cm-grid scale")


if __name__ == "__main__":
    unittest.main()
