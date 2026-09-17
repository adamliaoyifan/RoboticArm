"""Static checks for Orin perception_site launch and script (no ROS graph)."""

import stat
import unittest
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
LAUNCH = PKG / "launch" / "perception_site.launch.py"
SCRIPT = PKG / "scripts" / "perception_site.sh"
HARDWARE_PICK = (
    PKG.parents[1] / "src" / "luggage_planning" / "launch" / "hardware_pick.launch.py"
)

SITE_TOPICS = (
    "/camera/d555/color/image_raw/compressed",
    "/camera/d555/aligned_depth_to_color/image_raw/compressed",
    "/livox/lidar",
    "/luggage/preprocessed/camera/depth/image",
    "/luggage/semantic/yolo_detections",
    "/luggage/semantic/overlay",
    "/luggage_detector/detect_luggage",
)


class PerceptionSiteLaunchTest(unittest.TestCase):
    def test_keeps_hardware_pick_topic_names(self):
        src = LAUNCH.read_text(encoding="utf-8")
        self.assertIn("d555_rgbd.launch.py", src)
        self.assertIn("mid360.launch.py", src)
        self.assertIn("require_backend", src)
        self.assertIn("yolo_world", src)
        self.assertIn("preprocessor_d555_site.yaml", src)
        self.assertIn('name="semantic_segmenter"', src)
        self.assertIn('name="luggage_detector"', src)
        self.assertIn('depth_topic": "/luggage/preprocessed/camera/depth/image"', src)
        self.assertIn("/luggage/semantic/yolo_detections", src)
        self.assertNotIn('executable="trajectory_executor"', src)
        self.assertNotIn("hardware_pick.launch.py", src)
        for topic in SITE_TOPICS:
            self.assertIn(topic, src)

    def test_does_not_own_cps_or_rename_topics(self):
        src = LAUNCH.read_text(encoding="utf-8")
        self.assertNotIn("remappings=", src)
        self.assertNotIn("jazzy_real.launch.py", src)
        self.assertNotIn("motion_planner", src)
        self.assertIn('default_value="true"', src)
        self.assertIn('"start_d555"', src)
        self.assertIn('"start_mid360"', src)


class PerceptionSiteScriptTest(unittest.TestCase):
    def test_script_is_executable_and_lists_laptop_topics(self):
        self.assertTrue(SCRIPT.is_file(), SCRIPT)
        mode = SCRIPT.stat().st_mode
        self.assertTrue(mode & stat.S_IXUSR, SCRIPT)
        src = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("perception_site.launch.py", src)
        self.assertIn("ROS_DOMAIN_ID", src)
        self.assertIn("LUGGAGE_CLIP_VENDOR_DIR", src)
        for topic in SITE_TOPICS:
            self.assertIn(topic, src)

    def test_hardware_pick_topic_contract_still_matches(self):
        pick = HARDWARE_PICK.read_text(encoding="utf-8")
        site = LAUNCH.read_text(encoding="utf-8")
        self.assertIn("/luggage/preprocessed/camera/depth/image", pick)
        self.assertIn("/luggage/preprocessed/camera/depth/image", site)
        self.assertIn("d555_color_optical_frame", pick)
        self.assertIn("d555_color_optical_frame", site)
        self.assertIn('default_value="waypoint"', pick)
        self.assertNotIn('default_value="servo_j"', pick)


if __name__ == "__main__":
    unittest.main()
