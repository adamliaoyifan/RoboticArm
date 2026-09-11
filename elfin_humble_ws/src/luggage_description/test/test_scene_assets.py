#!/usr/bin/env python3
import os
import unittest

from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

from luggage_description.scene_assets import (
    build_scene_asset_markers,
    container_visual_mesh_resource,
)
from luggage_description.scene_tf_config_utils import load_scene_tf_config

CONFIG = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "config", "scene_tf.yaml.example")
)


class TestSceneAssets(unittest.TestCase):
    def setUp(self):
        self.scene = load_scene_tf_config(CONFIG)
        self.stamp = Time(sec=1, nanosec=0)

    def test_markers_cover_container_pedestal_and_platform(self):
        markers = build_scene_asset_markers(self.scene, self.stamp)
        frames = {item.header.frame_id for item in markers}
        namespaces = {item.ns for item in markers}
        self.assertIn("container_link", frames)
        self.assertIn("pedestal_link", frames)
        self.assertIn("pickup_platform_link", frames)
        self.assertIn("container_opening_frame", frames)
        self.assertTrue({"hull", "opening", "pedestal", "platform"} <= namespaces)
        hull = [item for item in markers if item.ns == "hull"][0]
        self.assertEqual(hull.type, Marker.LINE_LIST)
        self.assertGreater(len(hull.points), 8)
        self.assertEqual(hull.header.frame_id, "container_link")

    def test_visual_mesh_resolves_on_this_tree(self):
        uri = container_visual_mesh_resource(self.scene)
        self.assertTrue(uri.startswith("file://") or uri.startswith("package://"))
        self.assertIn("container_visual.stl", uri)
        mesh = [
            item
            for item in build_scene_asset_markers(self.scene, self.stamp)
            if item.type == Marker.MESH_RESOURCE
        ]
        self.assertEqual(len(mesh), 1)
        self.assertEqual(mesh[0].header.frame_id, "container_link")

    def test_scene_hardware_starts_scene_assets(self):
        launch = os.path.join(
            os.path.dirname(__file__), "..", "launch", "scene_hardware.launch.py"
        )
        src = open(launch, encoding="utf-8").read()
        self.assertIn("scene_assets_node", src)
        self.assertIn("/luggage/debug/scene_assets", src)
