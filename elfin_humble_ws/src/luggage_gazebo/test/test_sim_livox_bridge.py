#!/usr/bin/env python3
"""sim_world bridges the Fortress gpu_lidar cloud onto /livox/lidar."""

import os
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
LAUNCH = os.path.join(ROOT, "launch", "sim_world.launch.py")
RVIZ = os.path.join(ROOT, "rviz", "sim_full.rviz")


class TestSimLivoxBridge(unittest.TestCase):
    def test_camera_bridge_remaps_livox_cloud(self):
        with open(LAUNCH, encoding="utf-8") as handle:
            text = handle.read()
        self.assertIn(
            "/livox/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            text,
        )
        self.assertIn('("/livox/scan/points", "/livox/lidar")', text)

    def test_rviz_shows_livox_cloud(self):
        with open(RVIZ, encoding="utf-8") as handle:
            text = handle.read()
        self.assertIn("Name: Mid360Points", text)
        self.assertIn("Value: /livox/lidar", text)


if __name__ == "__main__":
    unittest.main()
