#!/usr/bin/env python3
"""Unit tests for top-down box footprint geometry."""

import math
import unittest

from luggage_perception.box_geometry import (
    MIN_ASPECT_FOR_YAW,
    aspect_ratio,
    fold_yaw_pi,
    footprint_iou,
    is_near_square,
    oriented_footprint_corners,
    wrap_to_pi,
)


class TestYawHelpers(unittest.TestCase):
    def test_wrap_to_pi(self):
        self.assertAlmostEqual(wrap_to_pi(0.0), 0.0)
        self.assertAlmostEqual(wrap_to_pi(math.pi), math.pi)
        self.assertAlmostEqual(wrap_to_pi(-math.pi), math.pi)
        self.assertAlmostEqual(wrap_to_pi(3.0 * math.pi), math.pi)

    def test_fold_yaw_pi_collapses_180(self):
        self.assertAlmostEqual(fold_yaw_pi(math.pi), 0.0, places=9)
        self.assertAlmostEqual(fold_yaw_pi(-math.pi), 0.0, places=9)
        self.assertAlmostEqual(abs(fold_yaw_pi(math.pi / 2.0 + 0.01)),
                               math.pi / 2.0 - 0.01, places=9)


class TestAspect(unittest.TestCase):
    def test_aspect_ratio(self):
        self.assertAlmostEqual(aspect_ratio(0.70, 0.45), 0.70 / 0.45)
        self.assertAlmostEqual(aspect_ratio(0.45, 0.70), 0.70 / 0.45)
        self.assertAlmostEqual(aspect_ratio(0.50, 0.50), 1.0)

    def test_near_square_uses_aspect_not_abs_diff(self):
        self.assertFalse(is_near_square(0.70, 0.45))
        self.assertTrue(is_near_square(0.55, 0.50))  # 1.10 < 1.15
        self.assertGreater(MIN_ASPECT_FOR_YAW, 1.10)
        self.assertFalse(is_near_square(0.70, 0.45, min_aspect=1.15))


class TestFootprintIou(unittest.TestCase):
    def test_identical_boxes_are_one(self):
        iou = footprint_iou(
            0.0, 0.0, 0.0, 0.80, 0.40,
            0.0, 0.0, 0.0, 0.80, 0.40)
        self.assertAlmostEqual(iou, 1.0, places=6)

    def test_180_flip_is_the_same_footprint(self):
        iou = footprint_iou(
            -1.0, 0.1, 0.3, 0.70, 0.45,
            -1.0, 0.1, 0.3 + math.pi, 0.70, 0.45)
        self.assertAlmostEqual(iou, 1.0, places=5)

    def test_90_with_size_swap_is_the_same_footprint(self):
        iou = footprint_iou(
            0.0, 0.0, 0.0, 0.80, 0.40,
            0.0, 0.0, math.pi / 2.0, 0.40, 0.80)
        self.assertAlmostEqual(iou, 1.0, places=5)

    def test_true_90_without_swap_fails_iou(self):
        iou = footprint_iou(
            0.0, 0.0, 0.0, 0.80, 0.40,
            0.0, 0.0, math.pi / 2.0, 0.80, 0.40)
        self.assertLess(iou, 0.45)

    def test_small_center_shift_keeps_high_iou(self):
        iou = footprint_iou(
            0.0, 0.0, 0.0, 0.70, 0.45,
            0.03, 0.0, 0.0, 0.70, 0.45)
        self.assertGreater(iou, 0.75)

    def test_corners_are_ccw_and_centered(self):
        corners = oriented_footprint_corners(1.0, 2.0, 0.0, 0.80, 0.40)
        self.assertEqual(corners.shape, (4, 2))
        self.assertAlmostEqual(corners[:, 0].mean(), 1.0, places=9)
        self.assertAlmostEqual(corners[:, 1].mean(), 2.0, places=9)


if __name__ == "__main__":
    unittest.main()
