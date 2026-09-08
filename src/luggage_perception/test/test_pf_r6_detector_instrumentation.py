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
        points = np.array([[1.0, 2.0, 3.0]], dtype=np.float64)

        def decode(_msg):
            return points

        def transform(_tf_buffer, pts, source, target, stamp_time,
                      wall_timeout_sec=0.5, poll_sec=0.02):
            self.assertEqual((source, target), ("camera", "world"))
            return pts + 1.0, None

        def sleep(_period):
            self.node._raw_buffer[key] = _cloud(*key)

        with mock.patch.object(self.mod.adapters, "cloud_points_from_msg",
                               side_effect=decode):
            with mock.patch.object(self.mod, "_transform_points_to_world",
                                   side_effect=transform):
                with mock.patch.object(self.mod.time, "sleep",
                                       side_effect=sleep):
                    result = self.node._pop_raw_world_with_retry(
                        key, attempts=2, period_sec=0.0)

        self.assertTrue(np.array_equal(result, points + 1.0))
        self.assertEqual(self.node._raw_counts["raw_lookup_hit"], 1)
        self.assertEqual(
            self.node._last_raw_lookup["raw_lookup_attempts"], 2)
        self.assertEqual(
            self.node._last_raw_lookup["raw_lookup_status"], "hit")

    def test_raw_lookup_records_tf_failure(self):
        key = (12, 0)
        self.node._raw_buffer[key] = _cloud(*key)
        with mock.patch.object(
                self.mod.adapters, "cloud_points_from_msg",
                return_value=np.array([[1.0, 2.0, 3.0]], dtype=np.float64)):
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
