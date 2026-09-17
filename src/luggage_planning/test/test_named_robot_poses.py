#!/usr/bin/env python3
"""Unit tests for configurable observe poses (no ROS graph)."""

import os
import tempfile
import textwrap
import unittest

from luggage_planning.named_robot_poses import (
    CURRENT_POSE_NAMES,
    default_observe_pose_name,
    format_pose_yaml,
    is_current_pose_name,
    load_poses_config,
    named_pose_joints,
    plan_goto_joints,
    resolve_pose_name,
)


SITE = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..",
    "luggage_description", "config", "robot_poses.site.yaml"))
EXAMPLE = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..",
    "luggage_description", "config", "robot_poses.yaml.example"))


class NamedRobotPosesTest(unittest.TestCase):
    def test_current_aliases(self):
        for name in ("current", "here", "CURRENT", " Here "):
            self.assertTrue(is_current_pose_name(name), name)
        self.assertIn("current", CURRENT_POSE_NAMES)
        self.assertFalse(is_current_pose_name("pickup_observe"))
        self.assertFalse(is_current_pose_name(""))

    def test_site_yaml_defaults_to_current_without_sim_pickup(self):
        config = load_poses_config(SITE)
        self.assertEqual(default_observe_pose_name(config), "current")
        self.assertNotIn("pickup_observe", config.get("poses") or {})
        with self.assertRaises(KeyError) as ctx:
            named_pose_joints(config, "pickup_observe")
        self.assertIn("current", str(ctx.exception))

    def test_example_yaml_keeps_sim_pickup_observe(self):
        config = load_poses_config(EXAMPLE)
        joints = named_pose_joints(config, "pickup_observe")
        self.assertEqual(len(joints), 6)
        self.assertAlmostEqual(joints[0], 1.8806, places=4)

    def test_plan_goto_current_stays(self):
        config = load_poses_config(SITE)
        live = [0.1, -1.0, -1.1, 0.2, 1.5, 0.3]
        self.assertIsNone(plan_goto_joints("current", config, live))
        self.assertIsNone(plan_goto_joints("here", config, live))
        with self.assertRaises(RuntimeError):
            plan_goto_joints("current", config, None)

    def test_plan_goto_named_returns_values(self):
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as handle:
            handle.write(textwrap.dedent("""\
                poses:
                  pickup_observe:
                    values: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
                defaults:
                  observe_pose: pickup_observe
            """))
            path = handle.name
        try:
            config = load_poses_config(path)
            self.assertEqual(
                default_observe_pose_name(config), "pickup_observe")
            self.assertEqual(
                plan_goto_joints("pickup_observe", config, [0.0] * 6),
                [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
            self.assertEqual(
                resolve_pose_name("", config), "pickup_observe")
            self.assertEqual(
                resolve_pose_name("", config, param_override="current"),
                "current")
        finally:
            os.unlink(path)

    def test_format_pose_yaml_round_trip_name(self):
        snippet = format_pose_yaml(
            "pickup_observe", [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            observe_default="pickup_observe")
        self.assertIn("pickup_observe:", snippet)
        self.assertIn("values: [1.000000, 2.000000, 3.000000, 4.000000, 5.000000, 6.000000]", snippet)
        self.assertIn("observe_pose: pickup_observe", snippet)


if __name__ == "__main__":
    unittest.main()
