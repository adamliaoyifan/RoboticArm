import unittest

from builtin_interfaces.msg import Time
from sensor_msgs.msg import CompressedImage, Image

from elfin_trajectory_executor.d555_host_stamp_node import (
    CLOCK_MASTER_JSON,
    apply_common_stamp,
)


class HostStampTest(unittest.TestCase):
    def test_apply_common_stamp_writes_one_time(self):
        a = Image()
        b = Image()
        a.header.stamp.sec = 1
        b.header.stamp.sec = 9
        stamp = Time(sec=42, nanosec=7)
        apply_common_stamp((a, b), stamp)
        self.assertEqual(a.header.stamp.sec, 42)
        self.assertEqual(b.header.stamp.sec, 42)
        self.assertEqual(a.header.stamp.nanosec, 7)
        self.assertEqual(b.header.stamp.nanosec, 7)

    def test_apply_common_stamp_on_compressed(self):
        a = CompressedImage()
        b = CompressedImage()
        stamp = Time(sec=11, nanosec=2)
        apply_common_stamp((a, b), stamp)
        self.assertEqual(a.header.stamp.sec, 11)
        self.assertEqual(b.header.stamp.nanosec, 2)

    def test_clock_master_names_host(self):
        self.assertIn("host_ros_system_time", CLOCK_MASTER_JSON)
        self.assertIn("receive_sync_group", CLOCK_MASTER_JSON)


class D555LaunchContractTest(unittest.TestCase):
    def test_driver_stays_on_raw_image_transport(self):
        from pathlib import Path

        launch = (
            Path(__file__).resolve().parents[1]
            / "launch"
            / "d555_rgbd.launch.py"
        )
        src = launch.read_text()
        self.assertNotIn("image_transport/compressed", src)
        self.assertIn('"subscribe_compressed": False', src)
        self.assertIn("image_transport/raw", src)
        self.assertIn("motion/sample", src)
        self.assertIn("motion/sample_hw", src)
        self.assertIn("_imu_remaps", src)
        self.assertIn("_prefixed_remaps", src)

    def test_canonical_camera_info_is_latched_reliable(self):
        from pathlib import Path

        src = (
            Path(__file__).resolve().parents[1]
            / "elfin_trajectory_executor"
            / "d555_host_stamp_node.py"
        ).read_text()
        self.assertIn(
            'CameraInfo, f"{base}/color/camera_info", _INFO', src)
        self.assertIn(
            'CameraInfo, f"{base}/aligned_depth_to_color/camera_info", _INFO',
            src)
        self.assertNotIn(
            'self._on_cinfo, _INFO)',
            src)

    def test_imu_remaps_prefix_camera_name(self):
        import importlib.util
        from pathlib import Path

        path = (
            Path(__file__).resolve().parents[1] / "launch" / "d555_rgbd.launch.py"
        )
        spec = importlib.util.spec_from_file_location("d555_rgbd_launch", path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        remaps = mod._imu_remaps("d555")
        self.assertIn(("motion/sample", "motion/sample_hw"), remaps)
        self.assertIn(("d555/motion/sample", "d555/motion/sample_hw"), remaps)
        self.assertIn(("d555/gyro/sample", "d555/gyro/sample_hw"), remaps)


class RecordRegexTest(unittest.TestCase):
    def test_pendant_regex_includes_d555_imu_names(self):
        from pathlib import Path

        src = (
            Path(__file__).resolve().parents[1]
            / "launch"
            / "record_site.launch.py"
        ).read_text()
        self.assertIn("/camera/d555/imu$", src)
        self.assertIn("/camera/d555/motion/", src)
        self.assertIn("/camera/d555/gyro/", src)
        self.assertIn("/camera/d555/accel/", src)
