"""Static checks for the site hardware_pick launch (no ROS graph)."""

import unittest
from pathlib import Path

LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "hardware_pick.launch.py"


class HardwarePickLaunchTest(unittest.TestCase):
    def test_profile_b_is_site_yaml_not_replay(self):
        src = LAUNCH.read_text(encoding="utf-8")
        self.assertIn("preprocessor_d555_site.yaml", src)
        self.assertIn("preprocessor_d555_live.yaml", src)
        self.assertIn(
            'site_pp = os.path.join(perc_share, "config", "preprocessor_d555_site.yaml")',
            src,
        )
        self.assertIn("Do not use ", src)
        self.assertIn("preprocessor_d555_replay.yaml", src)
        self.assertNotIn(
            'os.path.join(exec_share, "config", "preprocessor_d555_replay.yaml")',
            src,
        )

    def test_site_launch_enables_semantic_overlay(self):
        src = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('DeclareLaunchArgument(\n                "publish_overlay"', src)
        self.assertIn('default_value="true"', src)
        self.assertIn(
            '"publish_overlay": ParameterValue(',
            src,
        )
        self.assertIn(
            'LaunchConfiguration("publish_overlay")',
            src,
        )

    def test_role_switches_gate_perception_and_planning_nodes(self):
        src = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('"start_perception"', src)
        self.assertIn('"start_planning"', src)
        self.assertIn('start_perception = IfCondition(', src)
        self.assertIn('start_planning = IfCondition(', src)
        for name in (
            "sensor_preprocessor",
            "semantic_segmenter",
            "semantic_point_filter",
            "luggage_detector",
        ):
            self.assertRegex(
                src,
                r'(?s)name="%s".*?condition=start_perception' % name,
            )
        for name in (
            "scene_manager",
            "waypoint_generator",
            "motion_planner",
            "vacuum_controller",
        ):
            self.assertRegex(
                src,
                r'(?s)name="%s".*?condition=start_planning' % name,
            )
        self.assertIn(
            'LaunchConfiguration("start_planning").perform(context)', src)
        self.assertIn("if not _bool_text", src)

    def test_low_level_owners_remain_independent_switches(self):
        src = LAUNCH.read_text(encoding="utf-8")
        for arg in ("start_d555", "start_executor", "start_scene", "use_moveit"):
            self.assertIn('"%s"' % arg, src)
        self.assertIn("condition=start_d555", src)
        self.assertIn("condition=start_executor", src)
        self.assertIn("condition=start_scene", src)


if __name__ == "__main__":
    unittest.main()
