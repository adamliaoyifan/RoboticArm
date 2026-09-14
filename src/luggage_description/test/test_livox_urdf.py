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
            'name="mid360_mount_xyz" value="0.010000 0.130000 0.015000"',
            text,
        )
        self.assertIn(
            'name="mid360_mount_rpy" value="0.16221799 1.63740799 1.77427038"',
            text,
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

    def test_sim_gpu_lidar_on_mount(self):
        with open(MOUNT, encoding="utf-8") as handle:
            text = handle.read()
        self.assertIn('type="gpu_lidar"', text)
        self.assertIn("<topic>livox/scan</topic>", text)
        self.assertIn("<gz_frame_id>livox_frame</gz_frame_id>", text)
        self.assertIn("${livox_optical_xyz}", text)
        self.assertNotIn('type="imu"', text)

    def test_sim_lidar_handbook_fov(self):
        with open(ORIGIN, encoding="utf-8") as handle:
            text = handle.read()
        self.assertIn('name="livox_sim_v_min" value="-0.12217305"', text)
        self.assertIn('name="livox_sim_v_max" value="0.90757121"', text)
        self.assertIn('name="livox_sim_range_min" value="0.1"', text)
        self.assertIn('name="livox_sim_range_max" value="40.0"', text)
        self.assertIn('name="livox_sim_update_hz" value="10.0"', text)


if __name__ == "__main__":
    unittest.main()
