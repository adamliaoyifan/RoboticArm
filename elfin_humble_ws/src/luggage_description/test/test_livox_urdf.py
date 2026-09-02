#!/usr/bin/env python3
"""URDF Mid-360S frames exist with handbook initial values."""

import os
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MOUNT = os.path.join(ROOT, "urdf", "eef_sensor_mount.urdf.xacro")
ORIGIN = os.path.join(ROOT, "config", "mid360_origin.xacro")


class TestLivoxUrdfFrames(unittest.TestCase):
    def test_origin_handbook_numbers(self):
        with open(ORIGIN, encoding="utf-8") as handle:
            text = handle.read()
        self.assertIn('name="livox_optical_xyz" value="0.000 0.000 0.047"', text)
        self.assertIn('name="livox_imu_xyz" value="0.01100 0.02329 -0.04412"', text)
        self.assertIn(
            'name="mid360_mount_rpy" value="0 1.57079633 1.57079633"', text
        )
        self.assertIn(
            'name="mid360_mount_xyz" value="0.022 0.103 0.038"', text
        )

    def test_mount_declares_livox_chain(self):
        with open(MOUNT, encoding="utf-8") as handle:
            text = handle.read()
        for name in (
            "mid360_mount_frame",
            "livox_frame",
            "livox_imu_frame",
            "livox_optical_joint",
            "livox_imu_joint",
        ):
            self.assertIn(name, text)
        self.assertIn("mid360_origin.xacro", text)


if __name__ == "__main__":
    unittest.main()
