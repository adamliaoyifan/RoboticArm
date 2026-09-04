#!/usr/bin/env python3
"""Unit tests for scene mesh asset resolution (no roscore required)."""

import os
import sys
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from luggage_description.scene_mesh_utils import (  # noqa: E402
    container_collision_mesh_path,
    container_model_name,
    container_visual_mesh_path,
)
from luggage_description.scene_tf_config_utils import load_scene_tf_config  # noqa: E402


def _example_scene_path():
    return os.path.join(PKG_ROOT, "config", "scene_tf.yaml.example")


class TestSceneMeshUtils(unittest.TestCase):
  def setUp(self):
    self.example_scene = load_scene_tf_config(_example_scene_path())

  def test_container_model_name_from_scene_tf(self):
    self.assertEqual(container_model_name(self.example_scene), "airport_container_real")

  def test_container_mesh_paths_exist(self):
    collision = container_collision_mesh_path(self.example_scene)
    visual = container_visual_mesh_path(self.example_scene)
    self.assertTrue(collision.endswith("container_collision.stl"))
    self.assertTrue(visual.endswith("container_visual.stl"))
    self.assertTrue(os.path.exists(collision), collision)
    self.assertTrue(os.path.exists(visual), visual)


if __name__ == "__main__":
    unittest.main()
