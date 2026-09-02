#!/usr/bin/env python3
"""D555 body frame reuses the D435 camera_link mount (identity)."""

import os
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
D435 = os.path.join(ROOT, "urdf", "realsense_d435.urdf.xacro")
CAM = os.path.join(ROOT, "config", "camera_mount_origin.xacro")


class TestD555UsesD435Extrinsics(unittest.TestCase):
    def test_identity_on_camera_link(self):
        with open(D435, encoding="utf-8") as handle:
            text = handle.read()
        self.assertIn('name="d555_link"', text)
        self.assertIn('<parent link="camera_link"/>', text)
        self.assertIn('<child link="d555_link"/>', text)
        self.assertIn('<origin xyz="0 0 0" rpy="0 0 0"/>', text)

    def test_mount_origin_unchanged(self):
        with open(CAM, encoding="utf-8") as handle:
            text = handle.read()
        self.assertIn('name="cam_mount_parent" value="eef_mount_adapter"', text)
        self.assertIn('name="cam_mount_xyz" value="0.013000 0.097000 -0.021000"', text)
        self.assertIn(
            'name="cam_mount_rpy" value="0.03769911 1.36345121 1.57079633"',
            text,
        )


if __name__ == "__main__":
    unittest.main()
