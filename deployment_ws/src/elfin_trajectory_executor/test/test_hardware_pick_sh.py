"""实机代码 (site / real-cell). Not part of the Gazebo simulation stack."""

import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "hardware_pick.sh"


class HardwarePickShTest(unittest.TestCase):
    def test_script_exists(self):
        self.assertTrue(SCRIPT.is_file(), SCRIPT)

    def test_no_gpu_defaults_semantic_device_cpu(self):
        src = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("_has_nvidia_gpu", src)
        self.assertIn('launch_args+=("semantic_device:=cpu")', src)
        self.assertIn("nvidia-smi", src)

    def test_comments_point_at_site_profile_b_not_live_replay(self):
        src = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("preprocessor_d555_site.yaml", src)
        self.assertIn("Do not pass preprocessor_d555_replay.yaml", src)

    def test_driver_flags_are_not_forwarded_to_launch(self):
        src = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("hardware_pick_driver.py flag", src)
        self.assertIn("skip-observe:=*", src)
        self.assertIn("detect-only:=*", src)


if __name__ == "__main__":
    unittest.main()
