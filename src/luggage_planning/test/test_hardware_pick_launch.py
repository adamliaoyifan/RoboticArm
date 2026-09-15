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


if __name__ == "__main__":
    unittest.main()
