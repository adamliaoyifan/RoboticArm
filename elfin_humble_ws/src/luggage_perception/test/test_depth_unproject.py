#!/usr/bin/env python3
"""Unit tests for colour-aligned Z16 unprojection (no ROS)."""

import unittest

import numpy as np

from luggage_perception.depth_unproject import unproject_z16


class TestUnprojectZ16(unittest.TestCase):
    def test_principal_point_is_optical_axis(self):
        depth = np.zeros((3, 3), dtype=np.uint16)
        depth[1, 1] = 1000
        pts = unproject_z16(depth, fx=100.0, fy=100.0, cx=1.0, cy=1.0)
        self.assertEqual(pts.shape, (1, 3))
        np.testing.assert_allclose(pts[0], [0.0, 0.0, 1.0], atol=1e-6)

    def test_offset_pixel(self):
        depth = np.zeros((2, 2), dtype=np.uint16)
        depth[0, 0] = 1000
        pts = unproject_z16(depth, fx=100.0, fy=100.0, cx=1.0, cy=1.0)
        np.testing.assert_allclose(pts[0], [-0.01, -0.01, 1.0], atol=1e-6)

    def test_zero_and_invalid_dropped(self):
        depth = np.array([[0, 500], [0, 0]], dtype=np.uint16)
        pts = unproject_z16(depth, fx=50.0, fy=50.0, cx=0.0, cy=0.0)
        self.assertEqual(len(pts), 1)
        np.testing.assert_allclose(pts[0, 2], 0.5, atol=1e-6)

    def test_stride_samples_grid(self):
        depth = np.full((4, 4), 1000, dtype=np.uint16)
        full = unproject_z16(depth, 100.0, 100.0, 0.0, 0.0, stride=1)
        half = unproject_z16(depth, 100.0, 100.0, 0.0, 0.0, stride=2)
        self.assertEqual(len(full), 16)
        self.assertEqual(len(half), 4)

    def test_bad_intrinsics_empty(self):
        depth = np.full((2, 2), 1000, dtype=np.uint16)
        pts = unproject_z16(depth, fx=0.0, fy=100.0, cx=0.0, cy=0.0)
        self.assertEqual(pts.shape, (0, 3))
