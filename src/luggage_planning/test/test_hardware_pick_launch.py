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


if __name__ == "__main__":
    unittest.main()
