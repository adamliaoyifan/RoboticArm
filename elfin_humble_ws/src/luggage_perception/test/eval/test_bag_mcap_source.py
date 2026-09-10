#!/usr/bin/env python3
"""Unit tests for the mcap bag source (synthetic fixture bag)."""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "fixtures"))
from make_tiny_replay_bag import BASE_NS, build_fixture  # noqa: E402

from luggage_perception.eval.bag_mcap_source import (  # noqa: E402
    BagScan,
    decode_color_rgb,
    decode_depth_mm,
    find_mcap_file,
    iter_bag_messages,
    scan_bag,
)

COLOR = "/camera/d555/color/image_raw"
DEPTH = "/camera/d555/aligned_depth_to_color/image_raw"
DEPTH_INFO = "/camera/d555/aligned_depth_to_color/camera_info"


class TestBagMcapSource(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.bag = build_fixture(os.path.join(cls._tmp.name, "tiny.mcap"))

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def test_find_mcap_file_dir_and_direct(self):
        direct = find_mcap_file(self.bag)
        self.assertEqual(direct, os.path.abspath(self.bag))
        self.assertEqual(find_mcap_file(os.path.dirname(self.bag)), direct)
        with self.assertRaises(FileNotFoundError):
            find_mcap_file("/nonexistent/bag")

    def test_scan_counts_match_written_records(self):
        scan = scan_bag(self.bag)
        self.assertIsInstance(scan, BagScan)
        self.assertEqual(scan.topics[COLOR]["message_count"], 4)
        self.assertEqual(scan.topics[DEPTH]["message_count"], 4)
        self.assertEqual(scan.topics["/joint_states"]["message_count"], 1)
        self.assertEqual(
            scan.topics[DEPTH_INFO]["msg_type"], "sensor_msgs/msg/CameraInfo")
        self.assertIn("/livox/imu", scan.skipped_topics)

    def test_iter_decodes_registered_topics_only(self):
        topics = [m.topic for m in iter_bag_messages(self.bag)]
        self.assertNotIn("/livox/imu", topics)
        self.assertEqual(topics.count(COLOR), 4)

    def test_header_and_log_time(self):
        color = [m for m in iter_bag_messages(self.bag, topics=[COLOR])]
        self.assertEqual(color[0].header_stamp_ns, BASE_NS)
        self.assertEqual(
            color[0].log_time_ns - color[0].header_stamp_ns, 12_000_000)

    def test_decode_color_shapes(self):
        for msg in (m.message for m in
                    iter_bag_messages(self.bag, topics=[COLOR])):
            arr = decode_color_rgb(msg)
            self.assertEqual(arr.shape, (12, 16, 3))
            self.assertEqual(str(arr.dtype), "uint8")
            self.assertTrue(arr.flags["C_CONTIGUOUS"])

    def test_decode_depth(self):
        depth = [m.message for m in
                 iter_bag_messages(self.bag, topics=[DEPTH])]
        arr = decode_depth_mm(depth[0])
        self.assertEqual(arr.shape, (12, 16))
        self.assertIn(str(arr.dtype), ("uint16", "<u2", ">u2"))
        self.assertEqual(int(arr[0, 0]), 1500)

    def test_padded_step_decodes(self):
        # The fixture's second color frame carries a 4-byte row pad
        # (step 52 = 16*3 + 4); the adapter must honour msg.step.
        color = [m for m in iter_bag_messages(self.bag, topics=[COLOR])]
        padded = color[1].message
        self.assertEqual(int(padded.step), 16 * 3 + 4)
        arr = decode_color_rgb(padded)
        self.assertIsNotNone(arr)
        self.assertEqual(arr.shape, (12, 16, 3))
        self.assertEqual(arr[0, 0].tolist(), [0, 1, 2])

    def test_bgr8_swapped_to_rgb(self):
        from sensor_msgs.msg import Image

        msg = Image()
        msg.width, msg.height = 2, 1
        msg.encoding = "bgr8"
        msg.step = 6
        msg.data = bytearray([255, 0, 0, 0, 0, 255])  # BGR BGR
        arr = decode_color_rgb(msg)
        self.assertEqual(arr.shape, (1, 2, 3))
        self.assertEqual(arr[0, 0].tolist(), [0, 0, 255])  # -> RGB
        self.assertEqual(arr[0, 1].tolist(), [255, 0, 0])


if __name__ == "__main__":
    unittest.main()
