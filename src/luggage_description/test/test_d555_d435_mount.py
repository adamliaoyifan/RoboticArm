#!/usr/bin/env python3
"""The D555 driver root is an identity alias of the calibrated camera_link."""

import os
import unittest

import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
D435 = os.path.join(ROOT, "urdf", "realsense_d435.urdf.xacro")
CAM = os.path.join(ROOT, "config", "camera_mount_origin.xacro")
D435_CONFIG = os.path.join(ROOT, "config", "realsense_d435.yaml")
MOUNT_CONFIG = os.path.join(ROOT, "config", "realsense_d435_mount.yaml.example")

LOCKED_XYZ = [-0.023249, 0.099580, -0.052059]
LOCKED_RPY = [0.02901151, 1.32524323, 1.59953586]


class TestD555FrameAlias(unittest.TestCase):
    def test_identity_on_camera_link(self):
        with open(D435, encoding="utf-8") as handle:
            text = handle.read()
        self.assertIn('name="d555_link"', text)
        self.assertIn('<parent link="camera_link"/>', text)
        self.assertIn('<child link="d555_link"/>', text)
        self.assertIn('<origin xyz="0 0 0" rpy="0 0 0"/>', text)

    def test_visual_uses_d555_datasheet_envelope(self):
        with open(D435, encoding="utf-8") as handle:
            text = handle.read()
        self.assertIn('name="d555_cam_depth" value="0.048"', text)
        self.assertIn('name="d555_cam_length" value="0.167"', text)
        self.assertIn('name="d555_cam_height" value="0.042"', text)
        self.assertIn('name="d555_cam_py" value="0.0475"', text)
        self.assertIn(
            'size="${d555_cam_depth} ${d555_cam_length} ${d555_cam_height}"',
            text,
        )
        self.assertNotIn('name="d435_cam_width" value="0.090"', text)
        self.assertNotIn('name="d435_cam_depth" value="0.02505"', text)

    def test_mount_origin_unchanged(self):
        with open(CAM, encoding="utf-8") as handle:
            text = handle.read()
        self.assertIn('name="cam_mount_parent" value="eef_mount_adapter"', text)
        self.assertIn(
            'name="cam_mount_xyz" value="-0.023249 0.099580 -0.052059"',
            text,
        )
        self.assertIn(
            'name="cam_mount_rpy" value="0.02901151 1.32524323 1.59953586"',
            text,
        )

    def test_compatibility_yamls_match_locked_layer3(self):
        for path, root_key in (
            (D435_CONFIG, "camera"),
            (MOUNT_CONFIG, None),
        ):
            with self.subTest(path=path), open(path, encoding="utf-8") as handle:
                data = yaml.safe_load(handle)
                mount = data[root_key]["mount"] if root_key else data["mount"]
                self.assertEqual(mount["fixed"]["xyz"], LOCKED_XYZ)
                self.assertEqual(mount["fixed"]["rpy"], LOCKED_RPY)
                self.assertEqual(mount["xyz"], LOCKED_XYZ)
                self.assertEqual(mount["rpy"], LOCKED_RPY)
                self.assertEqual(
                    [mount["tune_joints"][key] for key in ("tx", "ty", "tz")],
                    LOCKED_XYZ,
                )
                self.assertEqual(
                    [mount["tune_joints"][key] for key in ("rx", "ry", "rz")],
                    LOCKED_RPY,
                )
                self.assertTrue(data["meta"]["locked"])
                self.assertEqual(data["meta"]["hardware"], "d555_poe")


if __name__ == "__main__":
    unittest.main()
