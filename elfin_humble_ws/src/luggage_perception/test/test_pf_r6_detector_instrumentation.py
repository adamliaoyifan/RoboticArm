#!/usr/bin/env python3
"""Focused PF-R6 detector instrumentation tests without a ROS node."""

import importlib.util
import os
import threading
import unittest
from collections import OrderedDict
from types import SimpleNamespace
from unittest import mock

import numpy as np


def _load_detector_module():
    script = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "scripts", "luggage_detector_node.py")
    spec = importlib.util.spec_from_file_location(
        "luggage_detector_node_under_test", script)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _stamp(sec, nanosec):
    from builtin_interfaces.msg import Time

    stamp = Time()
    stamp.sec = int(sec)
    stamp.nanosec = int(nanosec)
    return stamp


def _cloud(sec, nanosec, frame_id="camera"):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=_stamp(sec, nanosec), frame_id=frame_id))


class TestPfR6LazyRawLookup(unittest.TestCase):

    def setUp(self):
        self.mod = _load_detector_module()
        self.node = object.__new__(self.mod.LuggageDetector)
        self.node._raw_buffer = OrderedDict()
        self.node._raw_buffer_maxlen = 4
        self.node._raw_lock = threading.Lock()
        self.node._raw_counts = {
            "raw_received": 0,
            "raw_evicted": 0,
            "raw_lookup_hit": 0,
            "raw_lookup_miss": 0,
            "raw_lookup_empty": 0,
            "raw_lookup_decode_fail": 0,
            "raw_lookup_tf_fail": 0,
        }
        self.node._last_raw_lookup = {}
        self.node._cloud_data_frame = ""
        self.node._world_frame = "world"
        self.node._tf_buffer = object()
        # PF-R9 g2: the lazy lookup deprojects aligned depth locally.
        self.node._support_stride = 1
        self.node._support_info_lock = threading.Lock()
        from luggage_perception.semantic_point_filter import CameraIntrinsics
        self.node._support_intrinsics = CameraIntrinsics(
            fx=1.0, fy=1.0, cx=0.0, cy=0.0, width=2, height=2)
        self.node._scratch_lock = threading.Lock()
        self.node._scratch_buffers = {}

    def test_raw_lookup_rejects_neighbor_stamp(self):
        self.node._raw_buffer[(10, 0)] = _cloud(10, 0)

        result = self.node._pop_raw_world_with_retry(
            (10, 1), attempts=1, period_sec=0.0)

        self.assertIsNone(result)
        self.assertEqual(self.node._raw_counts["raw_lookup_miss"], 1)
        self.assertEqual(
            self.node._last_raw_lookup["raw_lookup_status"], "miss")
        self.assertEqual(
            self.node._last_raw_lookup["raw_lookup_key"], [10, 1])

    def test_raw_lookup_retries_same_stamp_before_miss(self):
        key = (11, 123)
        depth = np.array([[1000, 1000], [1000, 1000]], dtype=np.uint16)
        # stride 1, fx=fy=1, c=0: deprojection of z=1 m pixels (0,0)/(1,1)
        # -> (0,0,1) and (1,1,1).

        def decode(_msg):
            return depth

        def transform(_tf_buffer, pts, source, target, stamp_time,
                      wall_timeout_sec=0.5, poll_sec=0.02, **_kw):
            self.assertEqual((source, target), ("camera", "world"))
            return pts + 1.0, None

        def sleep(_period):
            self.node._raw_buffer[key] = _cloud(*key)

        with mock.patch.object(self.mod.adapters, "depth_array_from_msg",
                               side_effect=decode):
            with mock.patch.object(self.mod, "_transform_points_to_world",
                                   side_effect=transform):
                with mock.patch.object(self.mod.time, "sleep",
                                       side_effect=sleep):
                    result = self.node._pop_raw_world_with_retry(
                        key, attempts=2, period_sec=0.0)

        expected = np.array(
            [[0.0, 0.0, 1.0], [1.0, 0.0, 1.0],
             [0.0, 1.0, 1.0], [1.0, 1.0, 1.0]],
            dtype=np.float32) + 1.0
        self.assertTrue(np.allclose(result, expected))
        self.assertEqual(self.node._raw_counts["raw_lookup_hit"], 1)
        self.assertEqual(
            self.node._last_raw_lookup["raw_lookup_attempts"], 2)
        self.assertEqual(
            self.node._last_raw_lookup["raw_lookup_status"], "hit")

    def test_raw_lookup_records_tf_failure(self):
        key = (12, 0)
        self.node._raw_buffer[key] = _cloud(*key)
        with mock.patch.object(
                self.mod.adapters, "depth_array_from_msg",
                return_value=np.array(
                    [[1000, 1000], [1000, 1000]], dtype=np.uint16)):
            with mock.patch.object(self.mod, "_transform_points_to_world",
                                   return_value=(None, "missing tf")):
                result = self.node._pop_raw_world_with_retry(
                    key, attempts=1, period_sec=0.0)

        self.assertIsNone(result)
        self.assertEqual(self.node._raw_counts["raw_lookup_tf_fail"], 1)
        self.assertEqual(
            self.node._last_raw_lookup["raw_lookup_status"], "tf_fail")


if __name__ == "__main__":
    unittest.main()
