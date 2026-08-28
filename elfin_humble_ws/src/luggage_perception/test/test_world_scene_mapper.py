#!/usr/bin/env python3
"""Unit tests for WorldSceneMapper voxel grid logic."""
import math
import os
import sys
import unittest


from luggage_perception.world_scene_mapper import (  # noqa: E402
    FREE,
    OCCUPIED,
    UNKNOWN,
    WorldSceneMapper,
)


class TestWorldSceneMapperBasics(unittest.TestCase):
    def _mapper(self, res=0.1, stale=10.0):
        bounds = [[-0.5, 0.5], [-0.5, 0.5], [-0.5, 0.5]]
        return WorldSceneMapper(bounds, res, stale)

    def test_initial_state_all_unknown(self):
        m = self._mapper()
        stats = m.stats()
        self.assertEqual(stats["unknown_count"], stats["total_voxels"])
        self.assertEqual(stats["occupied_count"], 0)
        self.assertEqual(stats["free_count"], 0)

    def test_integrate_single_point_marks_occupied(self):
        m = self._mapper(res=0.1)
        m.integrate_points([(0.0, 0.0, 0.0)], origin=(-0.4, 0.0, 0.0), now=1.0)
        stats = m.stats()
        self.assertGreater(stats["occupied_count"], 0)

    def test_raycast_marks_free(self):
        m = self._mapper(res=0.1)
        m.integrate_points([(0.3, 0.0, 0.0)], origin=(-0.3, 0.0, 0.0), now=1.0)
        stats = m.stats()
        self.assertGreater(stats["free_count"], 0)

    def test_stale_clearing(self):
        m = self._mapper(res=0.1, stale=5.0)
        m.integrate_points([(0.0, 0.0, 0.0)], origin=(-0.3, 0.0, 0.0), now=1.0)
        self.assertGreater(m.stats()["occupied_count"], 0)
        cleared = m.clear_stale(100.0)
        self.assertGreater(cleared, 0)
        self.assertEqual(m.stats()["occupied_count"], 0)
        self.assertEqual(m.stats()["free_count"], 0)

    def test_stale_not_clearing_fresh(self):
        m = self._mapper(res=0.1, stale=5.0)
        m.integrate_points([(0.0, 0.0, 0.0)], origin=(-0.3, 0.0, 0.0), now=10.0)
        cleared = m.clear_stale(12.0)
        self.assertEqual(cleared, 0)
        self.assertGreater(m.stats()["occupied_count"], 0)

    def test_reset_clears_all(self):
        m = self._mapper(res=0.1)
        m.integrate_points([(0.0, 0.0, 0.0)], origin=(-0.3, 0.0, 0.0), now=1.0)
        m.reset()
        stats = m.stats()
        self.assertEqual(stats["occupied_count"], 0)
        self.assertEqual(stats["free_count"], 0)
        self.assertEqual(stats["unknown_count"], stats["total_voxels"])

    def test_obstacle_clusters(self):
        m = self._mapper(res=0.1)
        m.integrate_points(
            [(0.1, 0.1, 0.1), (0.2, 0.2, 0.2)],
            origin=(-0.3, 0.0, 0.0),
            now=1.0,
        )
        clusters = m.obstacle_clusters()
        self.assertGreater(len(clusters), 0)
        for x, y, z in clusters:
            self.assertTrue(math.isfinite(x))
            self.assertTrue(math.isfinite(y))
            self.assertTrue(math.isfinite(z))

    def test_out_of_bounds_ignored(self):
        m = self._mapper(res=0.1)
        m.integrate_points([(10.0, 10.0, 10.0)], origin=(0.0, 0.0, 0.0), now=1.0)
        stats = m.stats()
        self.assertEqual(stats["occupied_count"], 0)

    def test_grid_dimensions(self):
        bounds = [[-1.0, 1.0], [-0.5, 0.5], [0.0, 1.0]]
        m = WorldSceneMapper(bounds, 0.1, 10.0)
        self.assertEqual(m.nx, 20)
        self.assertEqual(m.ny, 10)
        self.assertEqual(m.nz, 10)


class TestWorldSceneMapperMultiFrame(unittest.TestCase):
    def test_dense_same_frame_counts_as_one_hit(self):
        bounds = [[-0.5, 0.5], [-0.5, 0.5], [-0.5, 0.5]]
        m = WorldSceneMapper(
            bounds,
            0.1,
            30.0,
            occupancy_params={"occupied_threshold": 1.2},
        )
        repeated = [(0.0, 0.0, 0.0)] * 100
        m.integrate_points(repeated, origin=(-0.4, 0.0, 0.0), now=1.0)
        self.assertEqual(m.stats()["occupied_count"], 0)
        m.integrate_points(repeated, origin=(-0.4, 0.0, 0.0), now=2.0)
        self.assertEqual(m.stats()["occupied_count"], 1)

    def test_free_rays_reverse_old_occupied_voxel(self):
        bounds = [[-0.5, 0.5], [-0.5, 0.5], [-0.5, 0.5]]
        m = WorldSceneMapper(
            bounds,
            0.1,
            30.0,
            occupancy_params={"occupied_threshold": 1.2},
        )
        for now in (1.0, 2.0):
            m.integrate_points(
                [(0.0, 0.0, 0.0)],
                origin=(-0.4, 0.0, 0.0),
                now=now,
            )
        old_voxel = m._world_to_voxel(0.0, 0.0, 0.0)
        old_index = m._index(*old_voxel)
        self.assertEqual(m.state_at(old_index), OCCUPIED)
        for now in range(3, 10):
            m.integrate_points(
                [(0.4, 0.0, 0.0)],
                origin=(-0.4, 0.0, 0.0),
                now=float(now),
            )
        self.assertNotEqual(m.state_at(old_index), OCCUPIED)

    def test_multi_frame_fusion(self):
        bounds = [[-1.0, 1.0], [-1.0, 1.0], [-1.0, 1.0]]
        m = WorldSceneMapper(bounds, 0.1, 30.0)
        m.integrate_points(
            [(0.3, 0.0, 0.0)],
            origin=(-0.5, 0.0, 0.0),
            now=1.0,
        )
        occ1 = m.stats()["occupied_count"]
        m.integrate_points(
            [(0.0, 0.3, 0.0)],
            origin=(0.0, -0.5, 0.0),
            now=2.0,
        )
        occ2 = m.stats()["occupied_count"]
        self.assertGreaterEqual(occ2, occ1)


if __name__ == "__main__":
    unittest.main()
