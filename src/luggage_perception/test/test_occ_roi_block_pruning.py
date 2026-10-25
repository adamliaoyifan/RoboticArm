#!/usr/bin/env python3
"""ROI block-pruning oracle tests (OCC G3 P1, plan 26339903 §4).

``intersecting_block_ids`` must return exactly the candidate window
from whose blocks the exact ``region_intersects(block_bounds(...))``
test can still pass: a brute-force AABB oracle over every block must
survive only inside the pruned set, on edge shapes (BLOCK_EDGE-1/edge/
+1), nextfloat boundaries, thin/external ROIs, non-zero origins and
the deployment shape. Query-level equivalence runs through the
existing scalar-oracle contract test (unchanged code path).
"""

import math
import os
import sys
import unittest

import numpy as np

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(PKG_ROOT, "src", "luggage_perception"))
sys.path.insert(0, os.path.join(PKG_ROOT, "src"))

from luggage_perception.cargo_map_query import (  # noqa: E402
    BLOCK_EDGE, GridInfo, block_bounds, intersecting_block_ids,
    region_intersects)
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper  # noqa: E402
from luggage_perception.occupancy_map_store import (  # noqa: E402
    OccupancyMapStore)


def _grid_from_mapper(mapper):
    origin = (-mapper.inner_l / 2, -mapper.inner_w / 2, mapper._hull.floor_z)
    return GridInfo((mapper.nx, mapper.ny, mapper.nz), origin,
                    mapper.resolution, 'container_link',
                    mapper.geometry_hash, mapper._hull)


def _block_aabb_survivors(grid, n_blocks, region):
    """Brute force: ids whose block AABB passes the exact region test."""
    survivors = []
    for block_id in range(n_blocks):
        bx = (grid.shape[0] + BLOCK_EDGE - 1) // BLOCK_EDGE
        by = (grid.shape[1] + BLOCK_EDGE - 1) // BLOCK_EDGE
        ix = block_id % bx
        iy = block_id // bx % by
        iz = block_id // (bx * by)
        start = (ix * BLOCK_EDGE, iy * BLOCK_EDGE, iz * BLOCK_EDGE)
        shape = tuple(min(BLOCK_EDGE, grid.shape[a] - start[a])
                      for a in range(3))
        low = np.asarray(grid.origin) + np.asarray(start) * grid.resolution
        high = low + np.asarray(shape) * grid.resolution
        if region_intersects(low, high, region):
            survivors.append(block_id)
    return survivors


def _n_blocks(grid):
    bx = (grid.shape[0] + BLOCK_EDGE - 1) // BLOCK_EDGE
    by = (grid.shape[1] + BLOCK_EDGE - 1) // BLOCK_EDGE
    bz = (grid.shape[2] + BLOCK_EDGE - 1) // BLOCK_EDGE
    return bx * by * bz


class TestIntersectingBlockIds(unittest.TestCase):

    def _check(self, grid, region):
        pruned = intersecting_block_ids(grid, region)
        self.assertIsNotNone(pruned)
        survivors = _block_aabb_survivors(grid, _n_blocks(grid), region)
        pruned_set = set(pruned)
        # Every exact survivor is in the pruned window (no false drop)...
        self.assertTrue(set(survivors) <= pruned_set,
                        (grid.shape, region, sorted(set(survivors) - pruned_set)))
        # ...and the exact test over the pruned window reproduces the
        # survivors exactly (the window is not lossy in the other
        # direction: pruning + exact test == brute force + exact test).
        from luggage_perception.cargo_map_query import VoxelBlock
        # No block content needed: the exact test only reads bounds.
        rechecked = [i for i in pruned
                     if self._block_passes(grid, i, region)]
        self.assertEqual(sorted(rechecked), sorted(survivors),
                         (grid.shape, region))

    @staticmethod
    def _block_passes(grid, block_id, region):
        bx = (grid.shape[0] + BLOCK_EDGE - 1) // BLOCK_EDGE
        by = (grid.shape[1] + BLOCK_EDGE - 1) // BLOCK_EDGE
        ix = block_id % bx
        iy = block_id // bx % by
        iz = block_id // (bx * by)
        start = (ix * BLOCK_EDGE, iy * BLOCK_EDGE, iz * BLOCK_EDGE)
        shape = tuple(min(BLOCK_EDGE, grid.shape[a] - start[a])
                      for a in range(3))
        low = np.asarray(grid.origin) + np.asarray(start) * grid.resolution
        high = low + np.asarray(shape) * grid.resolution
        return region_intersects(low, high, region)

    def test_edge_shapes_and_boundaries(self):
        origin = (-0.745, -0.985, 0.53)
        res = 0.01
        for nx in (1, BLOCK_EDGE - 1, BLOCK_EDGE, BLOCK_EDGE + 1,
                   2 * BLOCK_EDGE + 5, 149):
            for ny, nz in ((17, 13), (16, 33)):
                grid = GridInfo((nx, ny, nz), origin, res, 'container_link',
                                'h', None)
                span = [nx * res, ny * res, nz * res]
                # Interior, touching-lower, touching-upper, thin slab per
                # axis, fully external, and nextafter-above/below boundaries.
                for axis in range(3):
                    a0 = origin[axis]
                    a1 = origin[axis] + span[axis]
                    rois = [
                        tuple(a0 + 0.3 * span[a] if a == axis else origin[a]
                              for a in range(3)),
                    ]
                    hi = tuple(a0 + 0.6 * span[a] if a == axis
                               else origin[a] + span[a] for a in range(3))
                    rois = [(rois[0], hi)]
                    rois.append(((a0, origin[1], origin[2]),
                                 (math.nextafter(a0, a1),
                                  origin[1] + span[1], origin[2] + span[2])))
                    rois.append(((math.nextafter(a1, a0 + 2 * span[0]),
                                  origin[1], origin[2]),
                                 (a1 + span[0], origin[1] + span[1],
                                  origin[2] + span[2])))
                    rois.append(((origin[0], origin[1], origin[2]),
                                 (origin[0] + span[0], origin[1] + span[1],
                                  math.nextafter(origin[2], origin[2] + 1)
                                  if axis == 2 else origin[2] + span[2])))
                    for low, high in rois:
                        # nextafter regions can be degenerate on one axis;
                        # normalize_region rejects a>=b, the selector must
                        # tolerate whatever the caller passes.
                        self._check(grid, (low, high))

    def test_random_rois_seeded(self):
        rng = np.random.default_rng(20261025)
        for _ in range(1000):
            shape = tuple(int(rng.integers(1, 40)) for _ in range(3))
            origin = tuple(float(rng.uniform(-2, 2)) for _ in range(3))
            res = float(rng.choice((0.05, 0.02, 0.01)))
            grid = GridInfo(shape, origin, res, 'container_link', 'h', None)
            span = [s * res for s in shape]
            low = [origin[a] + rng.uniform(-0.2, 1.0) * span[a]
                   for a in range(3)]
            high = [low[a] + rng.uniform(0.001, 0.7) * span[a] + 1e-9
                    for a in range(3)]
            self._check(grid, (tuple(low), tuple(high)))

    def test_external_region_empty_window(self):
        grid = GridInfo((33, 17, 20), (0.0, 0.0, 0.0), 0.02,
                        'container_link', 'h', None)
        self.assertEqual(
            (), intersecting_block_ids(grid, ((5.0, 5.0, 5.0),
                                              (5.1, 5.1, 5.1))))

    def test_none_region_returns_none(self):
        grid = GridInfo((4, 4, 4), (0, 0, 0), 0.05, 'container_link', 'h',
                        None)
        self.assertIsNone(intersecting_block_ids(grid, None))

    def test_instance_intersection(self):
        grid = GridInfo((40, 40, 40), (0.0, 0.0, 0.0), 0.01,
                        'container_link', 'h', None)
        region = ((0.0, 0.0, 0.0), (0.1, 0.1, 0.1))
        full = set(intersecting_block_ids(grid, region))
        pruned = intersecting_block_ids(grid, region, block_ids=(3, 7, 11))
        self.assertEqual(sorted(pruned),
                         sorted(i for i in (3, 7, 11) if i in full))

    def test_deployment_shape_prunes(self):
        # 1cm deployment container: pruning must not return the whole grid.
        grid = GridInfo((149, 197, 144), (-0.745, -0.985, 0.53), 0.01,
                        'container_link', 'h', None)
        region = ((-0.1, -0.1, 0.6), (0.1, 0.1, 0.8))
        pruned = intersecting_block_ids(grid, region)
        total = _n_blocks(grid)
        self.assertLess(len(pruned), total // 4,
                        "pruning must cut the candidate window sharply")
        self._check(grid, region)


if __name__ == "__main__":
    unittest.main()
