#!/usr/bin/env python3
"""Unit tests for depth_realism_filter (no roscore required)."""

import os
import sys
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from luggage_perception.depth_realism_filter import DepthRealismFilter


class TestDepthRealismFilterDisabled(unittest.TestCase):
    def test_passthrough_when_disabled(self):
        f = DepthRealismFilter(enabled=False)
        pts = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
        result = f.filter_points(pts, origin=(0.0, 0.0, 0.0))
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0], (1.0, 2.0, 3.0))
        stats = f.last_stats
        self.assertEqual(stats["raw_count"], 2)
        self.assertEqual(stats["filtered_count"], 2)
        self.assertEqual(stats["dropped_range"], 0)
        self.assertEqual(stats["dropped_dropout"], 0)


class TestRangeCutoff(unittest.TestCase):
    def test_hard_max_drops_distant_points(self):
        f = DepthRealismFilter(
            enabled=True,
            max_reliable_range=3.0,
            hard_max_range=4.0,
            range_noise_sigma=0.0,
            dropout_rate=0.0,
            random_seed=42,
        )
        pts = [(1.0, 0.0, 0.0), (5.0, 0.0, 0.0)]
        result = f.filter_points(pts, origin=(0.0, 0.0, 0.0))
        self.assertEqual(len(result), 1)
        self.assertAlmostEqual(result[0][0], 1.0)
        stats = f.last_stats
        self.assertEqual(stats["dropped_range"], 1)

    def test_points_within_reliable_range_kept(self):
        f = DepthRealismFilter(
            enabled=True,
            max_reliable_range=3.0,
            hard_max_range=4.0,
            range_noise_sigma=0.0,
            dropout_rate=0.0,
            random_seed=42,
        )
        pts = [(2.0, 0.0, 0.0), (2.5, 0.0, 0.0), (0.5, 0.0, 0.0)]
        result = f.filter_points(pts, origin=(0.0, 0.0, 0.0))
        self.assertEqual(len(result), 3)

    def test_probabilistic_drop_between_reliable_and_hard_max(self):
        f = DepthRealismFilter(
            enabled=True,
            max_reliable_range=2.0,
            hard_max_range=4.0,
            range_noise_sigma=0.0,
            dropout_rate=0.0,
            random_seed=123,
        )
        pts = [(3.5, 0.0, 0.0)] * 100
        result = f.filter_points(pts, origin=(0.0, 0.0, 0.0))
        self.assertGreater(len(result), 0)
        self.assertLess(len(result), 100)


class TestDropout(unittest.TestCase):
    def test_dropout_removes_some_points(self):
        f = DepthRealismFilter(
            enabled=True,
            max_reliable_range=10.0,
            hard_max_range=20.0,
            range_noise_sigma=0.0,
            dropout_rate=0.5,
            random_seed=99,
        )
        pts = [(1.0, 0.0, 0.0)] * 200
        result = f.filter_points(pts, origin=(0.0, 0.0, 0.0))
        self.assertGreater(len(result), 50)
        self.assertLess(len(result), 150)
        stats = f.last_stats
        self.assertGreater(stats["dropped_dropout"], 50)

    def test_zero_dropout_keeps_all(self):
        f = DepthRealismFilter(
            enabled=True,
            max_reliable_range=10.0,
            hard_max_range=20.0,
            range_noise_sigma=0.0,
            dropout_rate=0.0,
            random_seed=42,
        )
        pts = [(1.0, 0.0, 0.0)] * 50
        result = f.filter_points(pts, origin=(0.0, 0.0, 0.0))
        self.assertEqual(len(result), 50)


class TestNoise(unittest.TestCase):
    def test_noise_perturbs_points(self):
        f = DepthRealismFilter(
            enabled=True,
            max_reliable_range=10.0,
            hard_max_range=20.0,
            range_noise_sigma=0.01,
            dropout_rate=0.0,
            random_seed=42,
        )
        pts = [(2.0, 0.0, 0.0)] * 50
        result = f.filter_points(pts, origin=(0.0, 0.0, 0.0))
        self.assertEqual(len(result), 50)
        xs = [p[0] for p in result]
        self.assertFalse(all(x == 2.0 for x in xs))
        for x in xs:
            self.assertAlmostEqual(x, 2.0, delta=0.15)
        stats = f.last_stats
        self.assertEqual(stats["noise_applied"], 50)

    def test_zero_noise_no_perturbation(self):
        f = DepthRealismFilter(
            enabled=True,
            max_reliable_range=10.0,
            hard_max_range=20.0,
            range_noise_sigma=0.0,
            dropout_rate=0.0,
            random_seed=42,
        )
        pts = [(2.0, 1.0, 0.5)]
        result = f.filter_points(pts, origin=(0.0, 0.0, 0.0))
        self.assertEqual(result[0], (2.0, 1.0, 0.5))


class TestDeterminism(unittest.TestCase):
    def test_same_seed_same_result(self):
        pts = [(3.5, 0.0, 0.0)] * 50
        f1 = DepthRealismFilter(
            enabled=True, max_reliable_range=2.0, hard_max_range=5.0,
            range_noise_sigma=0.01, dropout_rate=0.1, random_seed=77,
        )
        f2 = DepthRealismFilter(
            enabled=True, max_reliable_range=2.0, hard_max_range=5.0,
            range_noise_sigma=0.01, dropout_rate=0.1, random_seed=77,
        )
        r1 = f1.filter_points(pts, origin=(0.0, 0.0, 0.0))
        r2 = f2.filter_points(pts, origin=(0.0, 0.0, 0.0))
        self.assertEqual(len(r1), len(r2))
        for p1, p2 in zip(r1, r2):
            self.assertAlmostEqual(p1[0], p2[0], places=10)


if __name__ == "__main__":
    unittest.main()
