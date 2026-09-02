#!/usr/bin/env python3
"""Unit tests for scene_tf_config_utils (no roscore required)."""

import os
import sys
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from luggage_description.scene_tf_config_utils import (  # noqa: E402
    container_in_base_link,
    container_inner_ceiling_z,
    container_inner_chamfer,
    container_inner_dimensions,
    container_inner_floor_z,
    container_inner_hull_edges_in_container,
    container_inner_y_max,
    container_opening_in_container,
    container_opening_aperture_corners,
    container_opening_aperture_corners_in_container,
    container_opening_aperture_lateral_offsets,
    container_opening_axes_in_base_link,
    container_opening_dimensions,
    container_opening_normal_in_world,
    container_opening_target_point,
    container_opening_target_point_in_world,
    container_outer_dimensions,
    container_usable_center_in_base_link,
    container_usable_dimensions,
    load_scene_tf_config,
    origin_in_world,
    pickup_platform_enabled,
    pickup_platform_top_in_world,
    pickup_source_in_world,
    point_inside_container_inner_box,
    point_inside_container_inner_hull_container,
    point_inside_opening_aperture,
    robot_base_in_world,
    robot_base_frame,
    static_transforms,
    urdf_world_base_pose,
    xyz_base_link_to_world,
    xyz_world_to_base_link,
    _local_point_to_base_link,
)


def _example_scene_path():
    return os.path.join(PKG_ROOT, "config", "scene_tf.yaml.example")


def _child_frames(transforms):
    return {item["child"] for item in transforms}


def _edge_set(transforms):
    return {(item["parent"], item["child"]) for item in transforms}


class TestSceneTfConfigUtils(unittest.TestCase):
  def setUp(self):
    self.example_scene = load_scene_tf_config(_example_scene_path())

  def test_pedestal_enabled_static_transforms_exclude_robot_base(self):
    transforms = static_transforms(self.example_scene)
    children = _child_frames(transforms)
    self.assertNotIn(robot_base_frame(self.example_scene), children)
    self.assertEqual(
        _edge_set(transforms),
        {
            ("world", "pedestal_link"),
            ("world", "pickup_platform_link"),
            ("pickup_platform_link", "pickup_platform_top"),
            ("world", "container_link"),
            ("container_link", "container_opening_frame"),
        },
    )

  def test_pedestal_disabled_static_transforms_exclude_robot_base(self):
    config = dict(self.example_scene)
    config["pedestal"] = dict(config["pedestal"])
    config["pedestal"]["enabled"] = False
    transforms = static_transforms(config)
    children = _child_frames(transforms)
    self.assertNotIn(robot_base_frame(config), children)
    self.assertEqual(
        _edge_set(transforms),
        {
            ("world", "pickup_platform_link"),
            ("pickup_platform_link", "pickup_platform_top"),
            ("world", "container_link"),
            ("container_link", "container_opening_frame"),
        },
    )

  def test_pickup_platform_top_height_matches_pedestal(self):
    top_xyz, _ = pickup_platform_top_in_world(self.example_scene)
    self.assertAlmostEqual(top_xyz[2], 0.86, places=4)

  def test_pickup_source_resolves_via_platform(self):
    src_xyz, src_rpy = pickup_source_in_world(self.example_scene)
    top_xyz, _ = pickup_platform_top_in_world(self.example_scene)
    self.assertAlmostEqual(src_xyz[2], top_xyz[2], places=4)

  def test_pickup_source_legacy_when_platform_disabled(self):
    config = dict(self.example_scene)
    config["pickup_platform"] = {"enabled": False}
    config["pickup_source"] = {"translation": [0.3, -0.8, 0.0], "rotation_rpy": [0.0, 0.0, 0.0]}
    src_xyz, _ = pickup_source_in_world(config)
    self.assertAlmostEqual(src_xyz[0], 0.3, places=4)
    self.assertAlmostEqual(src_xyz[1], -0.8, places=4)
    self.assertAlmostEqual(src_xyz[2], 0.0, places=4)

  def test_container_in_base_link_regression(self):
    xyz, rpy = container_in_base_link(self.example_scene)
    self.assertAlmostEqual(xyz[0], 0.0, places=4)
    self.assertAlmostEqual(xyz[1], -1.5, places=4)
    self.assertAlmostEqual(xyz[2], -0.86, places=6)
    self.assertAlmostEqual(rpy[0], 0.0, places=6)
    self.assertAlmostEqual(rpy[1], 0.0, places=6)
    self.assertAlmostEqual(rpy[2], -1.5708, places=4)

  def test_container_semantic_dimensions_live_in_scene_tf(self):
    self.assertEqual(container_outer_dimensions(self.example_scene), (1.57, 2.07, 2.12))
    self.assertEqual(container_inner_dimensions(self.example_scene), (1.49, 1.97, 2.01))
    opening_xyz, opening_rpy = container_opening_in_container(self.example_scene)
    self.assertEqual(opening_xyz, [-0.745, -0.27, 1.30])
    self.assertEqual(opening_rpy, [0.0, 0.0, 0.0])

  def test_container_usable_floor_and_ceiling(self):
    self.assertAlmostEqual(container_inner_floor_z(self.example_scene), 0.53)
    self.assertAlmostEqual(container_inner_ceiling_z(self.example_scene), 2.01)
    usable = container_usable_dimensions(self.example_scene)
    self.assertAlmostEqual(usable[0], 1.49)
    self.assertAlmostEqual(usable[1], 1.97)
    self.assertAlmostEqual(usable[2], 1.48)
    center = container_usable_center_in_base_link(self.example_scene)
    container_xyz, _ = container_in_base_link(self.example_scene)
    self.assertAlmostEqual(
        center[2] - container_xyz[2], (0.53 + 2.01) * 0.5, places=6)

  def test_legacy_container_defaults_floor_to_zero(self):
    config = dict(self.example_scene)
    config["container"] = dict(config["container"])
    config["container"]["inner"] = dict(config["container"]["inner"])
    config["container"]["inner"].pop("floor_z")
    config["container"]["inner"].pop("ceiling_z")
    self.assertEqual(container_inner_floor_z(config), 0.0)
    self.assertEqual(container_inner_ceiling_z(config), 2.01)
    self.assertEqual(container_usable_dimensions(config), (1.49, 1.97, 2.01))

  def test_container_opening_target_point_uses_scene_tf(self):
    xyz = container_opening_target_point(self.example_scene)
    self.assertAlmostEqual(xyz[0], -0.27, places=4)
    self.assertAlmostEqual(xyz[1], -0.755, places=4)
    self.assertAlmostEqual(xyz[2], 0.44, places=4)

  def test_opening_axes_are_orthonormal(self):
    normal, lateral, vertical = container_opening_axes_in_base_link(self.example_scene)
    for axis in (normal, lateral, vertical):
      self.assertAlmostEqual(sum(v * v for v in axis), 1.0, places=6)
    self.assertAlmostEqual(sum(normal[i] * lateral[i] for i in range(3)), 0.0, places=6)
    self.assertAlmostEqual(sum(normal[i] * vertical[i] for i in range(3)), 0.0, places=6)

  def test_opening_aperture_margin_and_point_check(self):
    center = container_opening_target_point(self.example_scene)
    self.assertTrue(point_inside_opening_aperture(center, self.example_scene, margin=0.1))
    full = container_opening_aperture_corners(self.example_scene, margin=0.0)
    safe = container_opening_aperture_corners(self.example_scene, margin=0.1)
    full_span = sum((full[1][i] - full[0][i]) ** 2 for i in range(3))
    safe_span = sum((safe[1][i] - safe[0][i]) ** 2 for i in range(3))
    self.assertLess(safe_span, full_span)

  def test_asymmetric_aperture_corners_derive_dimensions(self):
    width, height = container_opening_dimensions(self.example_scene)
    self.assertAlmostEqual(width, 1.326, places=3)
    self.assertAlmostEqual(height, 1.412, places=3)

  def test_asymmetric_aperture_corners_reject_outside_point(self):
    corners = container_opening_aperture_corners(self.example_scene, margin=0.0)
    outside = _local_point_to_base_link([-0.745, 0.90, 1.30], self.example_scene)
    self.assertFalse(point_inside_opening_aperture(outside, self.example_scene, margin=0.0))
    inside = container_opening_target_point(self.example_scene)
    self.assertTrue(point_inside_opening_aperture(inside, self.example_scene, margin=0.0))

  def test_legacy_opening_without_aperture_corners(self):
    config = dict(self.example_scene)
    config["container"] = dict(config["container"])
    config["container"]["opening"] = dict(config["container"]["opening"])
    config["container"]["opening"].pop("aperture", None)
    config["container"]["opening"]["side"] = "positive_y"
    config["container"]["opening"]["width"] = 1.18
    config["container"]["opening"]["height"] = 1.93
    for item in config["static_transforms"]:
      if item["child"] == "container_opening_frame":
        item["translation"] = [0.0, 0.985, 1.06]
    center = container_opening_target_point(config)
    self.assertTrue(point_inside_opening_aperture(center, config, margin=0.1))
    self.assertEqual(container_opening_dimensions(config), (1.18, 1.93))

  def test_aperture_lateral_offsets_span_inset_window(self):
    offsets = container_opening_aperture_lateral_offsets(
        self.example_scene, num_lateral=3, margin=0.12
    )
    self.assertEqual(len(offsets), 3)
    self.assertLess(offsets[0], offsets[-1])

  def test_urdf_world_base_pose_matches_planning_base(self):
    urdf_xyz, urdf_rpy = urdf_world_base_pose(self.example_scene)
    plan_xyz, plan_rpy = robot_base_in_world(self.example_scene)
    for actual, expected in zip(urdf_xyz, plan_xyz):
        self.assertAlmostEqual(actual, expected, places=6)
    for actual, expected in zip(urdf_rpy, plan_rpy):
        self.assertAlmostEqual(actual, expected, places=6)

  def test_opening_in_world_uses_container_link_pose(self):
    world = container_opening_target_point_in_world(self.example_scene)
    origin, _ = origin_in_world(self.example_scene)
    local, _ = container_opening_in_container(self.example_scene)
    self.assertAlmostEqual(world[0], origin[0] + local[0], places=6)
    self.assertAlmostEqual(world[1], origin[1] + local[1], places=6)
    self.assertAlmostEqual(world[2], origin[2] + local[2], places=6)
    normal = container_opening_normal_in_world(self.example_scene)
    self.assertAlmostEqual(normal[0], -1.0, places=5)
    self.assertAlmostEqual(normal[1], 0.0, places=5)
    self.assertAlmostEqual(normal[2], 0.0, places=5)

  def test_world_base_link_point_roundtrip(self):
    world = [1.5, 0.0, 0.655]
    base = xyz_world_to_base_link(self.example_scene, world)
    back = xyz_base_link_to_world(self.example_scene, base)
    for actual, expected in zip(back, world):
      self.assertAlmostEqual(actual, expected, places=6)

  def test_chamfer_clips_plus_y_floor_corner(self):
    chamfer = container_inner_chamfer(self.example_scene)
    self.assertIsNotNone(chamfer)
    self.assertAlmostEqual(container_inner_y_max(0.53, self.example_scene), 0.55, places=2)
    self.assertAlmostEqual(container_inner_y_max(1.20, self.example_scene), 0.985, places=3)
    self.assertFalse(point_inside_container_inner_hull_container(
        [0.0, 0.90, 0.55], self.example_scene))
    self.assertTrue(point_inside_container_inner_hull_container(
        [0.0, 0.90, 1.20], self.example_scene))
    self.assertTrue(point_inside_container_inner_hull_container(
        [0.0, -0.90, 0.55], self.example_scene))
    corner_base = _local_point_to_base_link([0.0, 0.90, 0.55], self.example_scene)
    self.assertFalse(point_inside_container_inner_box(corner_base, self.example_scene))

  def test_chamfer_absent_keeps_aabb(self):
    config = dict(self.example_scene)
    config["container"] = dict(config["container"])
    config["container"]["inner"] = dict(config["container"]["inner"])
    config["container"]["inner"].pop("chamfer", None)
    self.assertIsNone(container_inner_chamfer(config))
    self.assertTrue(point_inside_container_inner_hull_container(
        [0.0, 0.90, 0.55], config))
    self.assertEqual(len(container_inner_hull_edges_in_container(config)), 12)

  def test_hull_has_pentagon_end_walls(self):
    edges = container_inner_hull_edges_in_container(self.example_scene)
    self.assertGreaterEqual(len(edges), 15)
    opening_face = [e for e in edges if abs(e[0][0] + 0.745) < 1e-3 and abs(e[1][0] + 0.745) < 1e-3]
    self.assertEqual(len(opening_face), 5)


if __name__ == "__main__":
    unittest.main()
