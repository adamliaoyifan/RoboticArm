import json
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "launch" / "overlay_livox_d555.launch.py"
CALIB = ROOT / "launch" / "d555_calib.launch.py"
RVIZ = (
    ROOT.parents[2]
    / "elfin_humble_ws"
    / "src"
    / "elfin_description"
    / "rviz"
    / "overlay_livox_d555.rviz"
)


class OverlayLivoxD555Test(unittest.TestCase):
    def test_overlay_launch_wires_livox_and_d555_cloud(self):
        src = LAUNCH.read_text()
        self.assertIn("d555_calib.launch.py", src)
        self.assertIn("mid360.launch.py", src)
        self.assertIn("scene_hardware.launch.py", src)
        self.assertIn('"xfer_format": "0"', src)
        self.assertIn('"pointcloud_enable": "true"', src)
        self.assertIn('"align_depth_enable": "true"', src)
        self.assertIn("overlay_livox_d555.rviz", src)
        self.assertIn("start_rviz", src)
        self.assertIn("elfin_base_link", src)
        self.assertIn("/livox/lidar", src)
        self.assertIn("/camera/d555/depth/color/points", src)
        self.assertIn("/luggage/debug/scene_assets", src)
        self.assertIn("frame_id", src)

    def test_calib_launch_exposes_cloud_args_without_host_stamp(self):
        src = CALIB.read_text()
        self.assertIn("pointcloud_enable", src)
        self.assertIn("align_depth_enable", src)
        self.assertIn('"pointcloud_qos": "DEFAULT"', src)
        self.assertIn("align_depth.enable", src)
        self.assertIn("pointcloud.enable", src)
        self.assertNotIn('executable="d555_host_stamp"', src)
        self.assertNotIn("image_hw", src)

    def test_rviz_subscribes_to_live_driver_topics(self):
        self.assertTrue(RVIZ.is_file(), RVIZ)
        src = RVIZ.read_text()
        self.assertIn("/livox/lidar", src)
        self.assertIn("/camera/d555/depth/color/points", src)
        self.assertIn("/camera/d555/color/image_raw", src)
        self.assertIn("Fixed Frame: elfin_base_link", src)
        self.assertIn("Visual Enabled: true", src)
        self.assertIn("/luggage/debug/scene_assets", src)
        self.assertIn("Color Transformer: Intensity", src)
        self.assertIn("Color Transformer: FlatColor", src)
        self.assertNotIn("/luggage/preprocessed/", src)

    def test_livox_host_ip_from_site_json(self):
        import importlib.util

        spec = importlib.util.spec_from_file_location("overlay_livox_d555", LAUNCH)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        payload = {"Mid360s": {"host_net_info": [{"host_ip": "192.168.1.5"}]}}
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as handle:
            json.dump(payload, handle)
            path = handle.name
        try:
            self.assertEqual(mod._livox_host_ip(path), "192.168.1.5")
        finally:
            Path(path).unlink(missing_ok=True)
