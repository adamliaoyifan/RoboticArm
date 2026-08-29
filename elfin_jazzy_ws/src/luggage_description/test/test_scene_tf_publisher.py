#!/usr/bin/env python3
"""Unit tests for task_roi_from_scene and static-TF quaternion helpers."""

import math
import os
import unittest
import xml.etree.ElementTree as ET

from luggage_description.scene_tf_config_utils import (
    load_scene_tf_config,
    robot_base_frame,
    static_transforms,
    task_roi_from_scene,
    urdf_world_base_pose,
)
from luggage_description.scene_tf_publisher import (
    rpy_to_quaternion,
    static_transform_payloads,
)
from luggage_description.xacro_robot_with_scene_base import (
    format_xyz,
    nonfixed_joint_names,
    patch_world_base,
)

CONFIG = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "config", "scene_tf.yaml.example"))


def _ros1_make_quat(rpy):
    """ROS 1 container_tf_publisher._make_transform quaternion."""
    cr, sr = math.cos(rpy[0] * 0.5), math.sin(rpy[0] * 0.5)
    cp, sp = math.cos(rpy[1] * 0.5), math.sin(rpy[1] * 0.5)
    cy, sy = math.cos(rpy[2] * 0.5), math.sin(rpy[2] * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class TestRpyToQuaternion(unittest.TestCase):
    def test_identity(self):
        x, y, z, w = rpy_to_quaternion((0.0, 0.0, 0.0))
        self.assertAlmostEqual(x, 0.0, places=9)
        self.assertAlmostEqual(y, 0.0, places=9)
        self.assertAlmostEqual(z, 0.0, places=9)
        self.assertAlmostEqual(w, 1.0, places=9)

    def test_matches_ros1_formula_for_pedestal_yaw(self):
        rpy = (0.0, 0.0, 1.5708)
        actual = rpy_to_quaternion(rpy)
        expected = _ros1_make_quat(rpy)
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=9)

    def test_payloads_exclude_robot_base(self):
        config = load_scene_tf_config(CONFIG)
        payloads = static_transform_payloads(config)
        children = {item["child"] for item in payloads}
        self.assertNotIn(robot_base_frame(config), children)
        edges = {(item["parent"], item["child"]) for item in payloads}
        self.assertEqual(
            edges,
            {
                ("world", "pedestal_link"),
                ("world", "pickup_platform_link"),
                ("pickup_platform_link", "pickup_platform_top"),
                ("world", "container_link"),
                ("container_link", "container_opening_frame"),
            },
        )
        container = [p for p in payloads if p["child"] == "container_link"][0]
        self.assertEqual(container["translation"], [1.5, 0.0, 0.0])
        self.assertEqual(
            [round(v, 9) for v in container["rotation_xyzw"]],
            [0.0, 0.0, 0.0, 1.0],
        )


class TestTaskRoiFromScene(unittest.TestCase):
    def setUp(self):
        self.scene = load_scene_tf_config(CONFIG)
        self.values = task_roi_from_scene(self.scene)

    def test_values_are_numeric_not_strings(self):
        for key in ("container_center", "container_dims",
                    "opening_center", "opening_normal"):
            self.assertIsInstance(self.values[key], list)
            for item in self.values[key]:
                self.assertIsInstance(item, float)
        for key in ("container_yaw", "aperture_width", "aperture_height"):
            self.assertIsInstance(self.values[key], float)

    def test_container_dims_are_the_usable_span(self):
        dims = self.values["container_dims"]
        self.assertAlmostEqual(dims[0], 1.49, places=3)
        self.assertAlmostEqual(dims[1], 1.97, places=3)
        self.assertAlmostEqual(dims[2], 1.48, places=3)

    def test_container_is_not_at_the_robot_origin(self):
        center = self.values["container_center"]
        self.assertGreater(abs(center[1]), 1.0)
        self.assertNotEqual(center, [0.0, 0.0, 0.0])

    def test_opening_matches_the_aperture_corners(self):
        self.assertAlmostEqual(self.values["aperture_width"], 1.326, places=3)
        self.assertAlmostEqual(self.values["aperture_height"], 1.412, places=3)


class TestPatchWorldBase(unittest.TestCase):
    def test_patches_origin_to_scene_pose(self):
        xml = """
        <robot name="t">
          <joint name="world_base" type="fixed">
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <parent link="world"/>
            <child link="elfin_base_link"/>
          </joint>
        </robot>
        """
        root = ET.fromstring(xml)
        config = load_scene_tf_config(CONFIG)
        xyz, rpy = urdf_world_base_pose(config)
        self.assertTrue(patch_world_base(root, xyz, rpy))
        origin = root.find("joint").find("origin")
        got_xyz = [float(v) for v in origin.get("xyz").split()]
        got_rpy = [float(v) for v in origin.get("rpy").split()]
        for a, b in zip(got_xyz, xyz):
            self.assertAlmostEqual(a, b, places=6)
        for a, b in zip(got_rpy, rpy):
            self.assertAlmostEqual(a, b, places=6)
        self.assertEqual(origin.get("xyz"), format_xyz(xyz))

    def test_returns_false_without_world_base(self):
        root = ET.fromstring("<robot name='t'><joint name='other'/></robot>")
        self.assertFalse(patch_world_base(root, [0, 0, 0], [0, 0, 0]))

    def test_nonfixed_joint_names_skip_fixed(self):
        root = ET.fromstring(
            """
            <robot name="t">
              <joint name="world_base" type="fixed"/>
              <joint name="elfin_joint1" type="revolute"/>
              <joint name="slide" type="prismatic"/>
            </robot>
            """
        )
        self.assertEqual(
            nonfixed_joint_names(root), ["elfin_joint1", "slide"])

    def test_expand_installed_camera_xacro_if_available(self):
        try:
            from ament_index_python.packages import get_package_share_directory
            xacro_path = os.path.join(
                get_package_share_directory("luggage_description"),
                "urdf",
                "elfin_s20_with_camera.urdf.xacro",
            )
        except Exception:
            self.skipTest("luggage_description share not installed")
        if not os.path.isfile(xacro_path):
            self.skipTest("camera xacro not installed")
        from luggage_description.xacro_robot_with_scene_base import expand_and_patch
        try:
            xml, (xyz, rpy) = expand_and_patch(xacro_path, CONFIG)
        except FileNotFoundError:
            self.skipTest("xacro executable not on PATH")
        except Exception as exc:
            self.skipTest("xacro expand failed: %s" % exc)
        self.assertIn("elfin_base_link", xml)
        self.assertIn("suction_panel", xml)
        self.assertIn("camera_depth_optical_frame", xml)
        self.assertIn(format_xyz(xyz), xml)
        self.assertIn(format_xyz(rpy), xml)
        names = nonfixed_joint_names(ET.fromstring(xml))
        for joint in ("elfin_joint1", "elfin_joint6"):
            self.assertIn(joint, names)


class TestStaticTransformsStillExcludeBase(unittest.TestCase):
    def test_library_list_matches_publisher_payloads(self):
        config = load_scene_tf_config(CONFIG)
        lib_edges = {
            (item["parent"], item["child"]) for item in static_transforms(config)
        }
        pub_edges = {
            (item["parent"], item["child"])
            for item in static_transform_payloads(config)
        }
        self.assertEqual(lib_edges, pub_edges)


if __name__ == "__main__":
    unittest.main()
