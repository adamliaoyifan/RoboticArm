#!/usr/bin/env python3
"""Untracked cargo publisher is independent of CargoInstanceTracker."""

import importlib.util
import os
import threading
import unittest

import numpy as np
from builtin_interfaces.msg import Time


_HERE = os.path.dirname(os.path.abspath(__file__))
_NODE = os.path.normpath(os.path.join(
    _HERE, "..", "scripts", "semantic_point_filter_node.py"))


def _load():
    spec = importlib.util.spec_from_file_location(
        "semantic_point_filter_node", _NODE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _Pub(object):
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class TestUntrackedPublish(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        try:
            cls.mod = _load()
        except ImportError as exc:
            raise unittest.SkipTest("filter node deps unavailable: %s" % exc)

    def _node(self, include_obstacle=False):
        node = self.mod.SemanticPointFilterNode.__new__(
            self.mod.SemanticPointFilterNode)
        node._untracked_pub = _Pub()
        node._untracked_include_obstacle = include_obstacle
        node._last_depth_frame = "camera_depth_optical_frame"
        node._lock = threading.Lock()
        node._counts = {
            "untracked_publish_count": 0, "untracked_point_count": 0}
        return node

    def test_disabled_is_noop(self):
        node = self._node()
        node._untracked_pub = None
        node._publish_untracked(
            np.array([[1.0, 0.0, 0.0]]), None,
            Time(sec=1, nanosec=0), "cam")

    def test_publishes_label_filtered_stamp_and_frame(self):
        node = self._node()
        stamp = Time(sec=4, nanosec=200)
        cargo = np.array([[0.1, 0.0, 1.0], [0.2, 0.0, 1.0]], dtype=np.float64)
        node._publish_untracked(cargo, np.array([[9.0, 0.0, 0.0]]), stamp, "cam")
        self.assertEqual(len(node._untracked_pub.messages), 1)
        msg = node._untracked_pub.messages[0]
        self.assertEqual(msg.header.stamp.sec, 4)
        self.assertEqual(msg.header.stamp.nanosec, 200)
        self.assertEqual(msg.header.frame_id, "cam")
        self.assertEqual(msg.width, 2)
        self.assertEqual(node._counts["untracked_publish_count"], 1)
        self.assertEqual(node._counts["untracked_point_count"], 2)

    def test_include_obstacle(self):
        node = self._node(include_obstacle=True)
        node._publish_untracked(
            np.array([[1.0, 0.0, 0.0]]),
            np.array([[2.0, 0.0, 0.0]]),
            Time(sec=1, nanosec=0), "cam")
        self.assertEqual(node._untracked_pub.messages[0].width, 2)


if __name__ == "__main__":
    unittest.main()
