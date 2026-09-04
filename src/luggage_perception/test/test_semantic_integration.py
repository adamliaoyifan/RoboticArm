#!/usr/bin/env python3
"""Integration-level tests for the perception pipeline.

These tests verify the box estimator with realistic scene parameters
(platform Z, pickup source, catalog entries matching actual configs)
and XY jitter, without requiring ROS or Gazebo.
"""

import math
import os
import sys
import unittest

import numpy as np


from luggage_perception.luggage_box_estimator import estimate_box, match_catalog  # noqa: E402

# Scene parameters matching scene_tf.yaml.example.
PLATFORM_Z = 0.86
PICKUP_SOURCE_XY = (-1.0, 0.0)

CATALOG = [
    {"id": "carryon", "size": [0.55, 0.40, 0.25]},
    {"id": "standard", "size": [0.70, 0.45, 0.28]},
    {"id": "large", "size": [0.80, 0.50, 0.32]},
]


def _synthetic_box_cloud(cx, cy, width, depth, height, yaw,
                         n_top=600, noise_std=0.003, rng=None):
    """Generate top-surface points of a box sitting on the platform."""
    if rng is None:
        rng = np.random.RandomState(42)
    top_z = PLATFORM_Z + height
    u = rng.uniform(-width / 2, width / 2, n_top)
    v = rng.uniform(-depth / 2, depth / 2, n_top)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    x = cx + cos_y * u - sin_y * v + rng.normal(0, noise_std, n_top)
    y = cy + sin_y * u + cos_y * v + rng.normal(0, noise_std, n_top)
    z = np.full(n_top, top_z) + rng.normal(0, noise_std, n_top)
    return np.column_stack([x, y, z])


class TestSpawnWithJitterDetectAndCompare(unittest.TestCase):
    """Simulate spawn-with-jitter → detect → compare-to-GT cycle."""

    def _run_trial(self, entry, dx, dy, yaw):
        cx = PICKUP_SOURCE_XY[0] + dx
        cy = PICKUP_SOURCE_XY[1] + dy
        w, d, h = entry["size"]

        cloud = _synthetic_box_cloud(cx, cy, w, d, h, yaw, n_top=800, noise_std=0.003)

        est = estimate_box(
            cloud,
            roi_center_xy=PICKUP_SOURCE_XY,
            roi_margin=0.5,
            platform_z=PLATFORM_Z,
            catalog_entries=CATALOG,
            catalog_tolerance=0.05,
            min_points=50,
        )
        return est, cx, cy, w, d, h, yaw

    def test_standard_box_no_jitter(self):
        entry = CATALOG[1]  # standard
        est, cx, cy, w, d, h, yaw = self._run_trial(entry, 0.0, 0.0, 0.0)
        self.assertIsNotNone(est)
        self.assertEqual(est.matched_catalog_id, "standard")
        self.assertAlmostEqual(est.center_xyz[0], cx, delta=0.02)
        self.assertAlmostEqual(est.center_xyz[1], cy, delta=0.02)
        gt_cz = PLATFORM_Z + h / 2
        self.assertAlmostEqual(est.center_xyz[2], gt_cz, delta=0.02)

    def test_standard_box_with_xy_jitter(self):
        entry = CATALOG[1]
        for dx, dy in [(0.05, 0.03), (-0.08, 0.06), (0.10, -0.10)]:
            est, cx, cy, w, d, h, yaw = self._run_trial(entry, dx, dy, 0.2)
            self.assertIsNotNone(est, "Failed for dx=%.2f dy=%.2f" % (dx, dy))
            self.assertAlmostEqual(est.center_xyz[0], cx, delta=0.03)
            self.assertAlmostEqual(est.center_xyz[1], cy, delta=0.03)

    def test_carryon_with_large_yaw(self):
        entry = CATALOG[0]  # carryon
        est, cx, cy, w, d, h, yaw = self._run_trial(entry, -0.05, 0.02, 1.2)
        self.assertIsNotNone(est)
        self.assertEqual(est.matched_catalog_id, "carryon")
        self.assertAlmostEqual(est.center_xyz[0], cx, delta=0.03)

    def test_large_box_with_jitter(self):
        entry = CATALOG[2]  # large
        est, cx, cy, w, d, h, yaw = self._run_trial(entry, 0.07, -0.04, -0.5)
        self.assertIsNotNone(est)
        self.assertEqual(est.matched_catalog_id, "large")
        self.assertAlmostEqual(est.width, 0.80, delta=0.01)
        self.assertAlmostEqual(est.depth, 0.50, delta=0.01)

    def test_all_catalog_entries_detected_correctly(self):
        """Every catalog entry should be detected with correct ID."""
        rng = np.random.RandomState(123)
        for entry in CATALOG:
            dx = rng.uniform(-0.10, 0.10)
            dy = rng.uniform(-0.10, 0.10)
            yaw = rng.uniform(-math.pi, math.pi)
            est, cx, cy, w, d, h, _ = self._run_trial(entry, dx, dy, yaw)
            self.assertIsNotNone(est, "Detection failed for %s" % entry["id"])
            self.assertEqual(
                est.matched_catalog_id, entry["id"],
                "Catalog mismatch for %s: got %s" % (entry["id"], est.matched_catalog_id),
            )

    def test_gt_delta_within_tolerance(self):
        """Perception vs GT position delta should be < 3 cm for all sizes."""
        rng = np.random.RandomState(99)
        for entry in CATALOG:
            dx = rng.uniform(-0.08, 0.08)
            dy = rng.uniform(-0.08, 0.08)
            yaw = rng.uniform(-1.0, 1.0)
            est, cx, cy, w, d, h, _ = self._run_trial(entry, dx, dy, yaw)
            self.assertIsNotNone(est)
            delta = math.sqrt(
                (est.center_xyz[0] - cx) ** 2
                + (est.center_xyz[1] - cy) ** 2
            )
            self.assertLess(
                delta, 0.03,
                "XY delta %.4f too large for %s" % (delta, entry["id"]),
            )


if __name__ == "__main__":
    unittest.main()
