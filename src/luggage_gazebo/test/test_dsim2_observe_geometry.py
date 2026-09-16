#!/usr/bin/env python3
"""DSIM-2 negative controls that do not need Gazebo."""

import unittest

import numpy as np

from luggage_gazebo.depth_image_adapter import convert_depth_metres_to_mm
from luggage_perception.depth_deprojection import deproject_selected


def lying_optical_z(cam_z=1.9, platform_z=0.86, height=0.32):
    return cam_z - (platform_z + height)


class DummyK:
    fx = 323.1775
    fy = 322.8994
    cx = 317.7526
    cy = 178.0294


class TestDsim2Geometry(unittest.TestCase):
    def test_catalog_lying_is_not_the_0p24_standing_case(self):
        z_lying = lying_optical_z()
        z_standing = lying_optical_z(height=0.80)
        self.assertGreater(z_lying, 0.30)
        self.assertAlmostEqual(z_standing, 0.24, places=2)
        self.assertLess(z_standing, 0.30)

    def test_old_documented_pose_already_clears_min_z_for_lying_catalog(self):
        self.assertGreater(lying_optical_z(height=0.25), 0.30)
        self.assertGreater(lying_optical_z(height=0.32), 0.30)

    def test_restoring_standing_0p80_at_old_pose_fails_near_margin(self):
        # DS2-C2: the documented 0.24 m failure is the standing 0.80 m
        # envelope, not the lying catalog. Restoring that geometry at the
        # current pickup_observe pose fails the 0.30 m near-margin gate.
        z_standing = lying_optical_z(height=0.80)
        self.assertLess(z_standing, 0.30)
        self.assertAlmostEqual(z_standing, 0.24, places=2)

    def test_synthetic_inside_near_clip_yields_no_geometry(self):
        metres = np.full((8, 8), 0.20, dtype=np.float32)
        data = metres.tobytes()
        out, reason = convert_depth_metres_to_mm(
            "32FC1", 8, 8, 32, data, near_m=0.26, max_m=3.0)
        self.assertIsNone(reason)
        mm = np.frombuffer(out, dtype="<u2").reshape(8, 8)
        self.assertTrue((mm == 0).all())
        uu, vu = np.meshgrid(np.arange(8), np.arange(8))
        points, n = deproject_selected(mm, uu.reshape(-1), vu.reshape(-1), DummyK())
        self.assertEqual(n, 0)
        self.assertEqual(points.shape[0], 0)
