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
    decode_lidar_scan,
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
        self.assertEqual(scan.topics["/livox/lidar"]["message_count"], 3)
        self.assertEqual(
            scan.topics[DEPTH_INFO]["msg_type"], "sensor_msgs/msg/CameraInfo")
        self.assertIn("/livox/imu", scan.skipped_topics)

    def test_decode_lidar_scan_structured(self):
        lidar = [m.message for m in
                 iter_bag_messages(self.bag, topics=["/livox/lidar"])]
        arr = decode_lidar_scan(lidar[0])
        self.assertEqual(arr.shape, (4,))
        self.assertEqual(
            arr.dtype.names,
            ("x", "y", "z", "intensity", "tag", "line", "timestamp"))
        self.assertAlmostEqual(float(arr[1]["z"]), 1.2, places=5)
        self.assertAlmostEqual(float(arr[3]["intensity"]), 10.0, places=4)
        self.assertEqual(int(arr[0]["tag"]), 1)
        self.assertEqual(int(arr[0]["line"]), 2)
        # Per-point absolute-ns timestamps, 1 ms apart, base at the scan
        # header stamp (the fixture's first scan sits at t0+2 ms) — the
        # deskew input.
        lidar_t0 = float(BASE_NS + 2_000_000)
        self.assertAlmostEqual(
            float(arr[0]["timestamp"]), lidar_t0, delta=1.0)
        self.assertAlmostEqual(
            float(arr[3]["timestamp"]), lidar_t0 + 3e6, delta=1.0)

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


class TestHeaderFastPath(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.bag = build_fixture(os.path.join(cls._tmp.name, "tiny.mcap"))

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def test_fast_path_stamps_equal_full_path(self):
        # Every header-first topic in the fixture: the 8 stamp bytes parse
        # to exactly the stamp the full deserializer reads.
        for topic in (COLOR, DEPTH, DEPTH_INFO, "/joint_states",
                      "/elfin/tcp_pose", "/camera/d555/color/camera_info"):
            full = list(iter_bag_messages(self.bag, topics=[topic]))
            fast = list(iter_bag_messages(self.bag, topics=[topic],
                                          header_only_topics=[topic]))
            self.assertTrue(full, topic)
            self.assertEqual(len(full), len(fast), topic)
            for a, b in zip(full, fast):
                self.assertEqual(a.header_stamp_ns, b.header_stamp_ns,
                                 topic)
                self.assertEqual(a.log_time_ns, b.log_time_ns, topic)
                self.assertIsNone(b.message, topic)
                self.assertIsNotNone(a.message, topic)

    def test_parse_cdr_header_stamp_round_trip(self):
        from builtin_interfaces.msg import Time
        from rclpy.serialization import serialize_message
        from sensor_msgs.msg import Image

        from luggage_perception.eval.bag_mcap_source import (
            parse_cdr_header_stamp,
        )

        msg = Image()
        msg.header.stamp = Time(sec=1_760_000_000, nanosec=123_456_789)
        msg.header.frame_id = "optical"
        self.assertEqual(parse_cdr_header_stamp(serialize_message(msg)),
                         1_760_000_000 * 1_000_000_000 + 123_456_789)

    def test_parse_cdr_header_stamp_rejects_junk(self):
        from luggage_perception.eval.bag_mcap_source import (
            parse_cdr_header_stamp,
        )

        self.assertIsNone(parse_cdr_header_stamp(b"\x00\x00\x00\x00"
                                                 b"\x00\x00\x00\x00"
                                                 b"\x00\x00\x00\x00"))
        self.assertIsNone(parse_cdr_header_stamp(b"\x00\x01"))
        self.assertIsNone(parse_cdr_header_stamp(b""))


if __name__ == "__main__":
    unittest.main()
