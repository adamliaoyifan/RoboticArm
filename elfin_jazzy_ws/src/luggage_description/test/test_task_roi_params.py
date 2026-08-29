#!/usr/bin/env python3
"""Task-ROI geometry must be derived from scene_tf, not transcribed.

The previous launch-file constants had two failure modes at once: they drift
when scene_tf changes, and roslaunch typed them as strings so the C++ node fell
back to a 1 m cube at the robot origin. These tests pin the derivation and the
numeric types.
"""

import os
import unittest

from luggage_description.scene_tf_config_utils import (
    load_scene_tf_config,
    task_roi_from_scene,
)

CONFIG = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "config", "scene_tf.yaml.example"))


def _derive(scene_config):
    return task_roi_from_scene(scene_config)


class TestTaskRoiDerivation(unittest.TestCase):
    def setUp(self):
        self.scene = load_scene_tf_config(CONFIG)
        self.values = _derive(self.scene)

    def test_values_are_numeric_not_strings(self):
        """The exact defect the literal launch params had."""
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
        # ceiling_z - floor_z, not the legacy inner.height of 2.01.
        self.assertAlmostEqual(dims[2], 1.48, places=3)

    def test_container_is_not_at_the_robot_origin(self):
        """The fallback the C++ node was silently using."""
        center = self.values["container_center"]
        self.assertGreater(abs(center[1]), 1.0)
        self.assertNotEqual(center, [0.0, 0.0, 0.0])

    def test_opening_matches_the_aperture_corners(self):
        self.assertAlmostEqual(self.values["aperture_width"], 1.326, places=3)
        self.assertAlmostEqual(self.values["aperture_height"], 1.412, places=3)

    def test_derivation_tracks_a_moved_container(self):
        """Transcribed constants would not have followed this."""
        moved = load_scene_tf_config(CONFIG)
        for transform in moved["static_transforms"]:
            if transform.get("child") == "container_link":
                transform["translation"] = [
                    transform["translation"][0] + 0.5,
                    transform["translation"][1] - 0.25,
                    transform["translation"][2],
                ]
        shifted = _derive(moved)
        self.assertNotAlmostEqual(
            shifted["container_center"][0],
            self.values["container_center"][0], places=3)

    def test_launch_no_longer_transcribes_the_geometry(self):
        launch = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "..",
            "luggage_bringup", "launch", "active_loading.launch"))
        if not os.path.isfile(launch):
            self.skipTest("bringup launch not present")
        text = open(launch).read()
        for name in ("container_center", "container_dims", "container_yaw",
                     "opening_center", "opening_normal", "aperture_width",
                     "aperture_height"):
            self.assertNotIn(
                '<param name="%s"' % name, text,
                "%s is transcribed again in active_loading.launch" % name)


if __name__ == "__main__":
    unittest.main()
