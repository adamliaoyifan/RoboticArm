#!/usr/bin/env python3
"""Rate-limited segmenter stats must still carry spawn identity.

`stats_publish_hz` defaults to 1.0, so the timer is the normal publish path.
When it republished only `stats + stamp + frame_id`, every eval driver that
waits on `yolo_boxes_ready` starved on the missing `generation`, and the
pack-to-full run aborted with YOLO_NOT_READY while YOLO was in fact detecting
the box at 0.73 confidence.
"""

import importlib.util
import json
import os
import unittest

from luggage_gazebo.eval_metrics import yolo_boxes_ready

_HERE = os.path.dirname(os.path.abspath(__file__))
_NODE = os.path.normpath(os.path.join(
    _HERE, "..", "scripts", "semantic_segmenter_node.py"))


def _load_node_module():
    spec = importlib.util.spec_from_file_location(
        "semantic_segmenter_node", _NODE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _Output(object):
    stamp = 28.446
    frame_id = "camera_color_optical_frame"
    stats = {"raw_cargo": True, "accepted_cargo_count": 1,
             "backend": "bbox_fill:yolov8s-world.pt"}


class _Clock(object):
    class _Now(object):
        nanoseconds = 28_446_000_000

    def now(self):
        return self._Now()


class _Segmenter(object):
    self_body_mask = None


class _Pub(object):
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class TestSegmenterStatsRateLimit(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        try:
            cls.module = _load_node_module()
        except ImportError as exc:
            raise unittest.SkipTest("segmenter node deps unavailable: %s" % exc)

    def _node(self, interval_sec):
        node = self.module.SemanticSegmenterNode.__new__(
            self.module.SemanticSegmenterNode)
        node._stats_interval_sec = interval_sec
        node._stats_dirty = False
        node._last_stats_pub = 0.0
        node._pending_stats = None
        node._drop_count = 0
        node._self_body_source = "mesh"
        node._box_generation = 2
        node._box_id = "pickup_box_0001_carryon"
        node._segmenter = _Segmenter()
        node._stats_pub = _Pub()
        node.get_clock = lambda: _Clock()
        return node

    def _published(self, node):
        self.assertEqual(len(node._stats_pub.messages), 1)
        return json.loads(node._stats_pub.messages[0].data)

    def test_immediate_path_carries_spawn_identity(self):
        node = self._node(0.0)
        node._publish_stats(_Output())
        record = self._published(node)
        self.assertEqual(record["generation"], 2)
        self.assertEqual(record["instance_id"], "pickup_box_0001_carryon")

    def test_rate_limited_path_carries_the_same_record(self):
        node = self._node(1.0)
        node._publish_stats(_Output())
        self.assertEqual(node._stats_pub.messages, [],
                         "rate limit must defer the publish")
        node._on_stats_timer()
        record = self._published(node)
        self.assertEqual(record["generation"], 2)
        self.assertEqual(record["instance_id"], "pickup_box_0001_carryon")
        self.assertEqual(record["frame_id"], _Output.frame_id)
        self.assertEqual(record["stamp"], _Output.stamp)

    def test_rate_limited_record_satisfies_the_eval_readiness_gate(self):
        node = self._node(1.0)
        node._publish_stats(_Output())
        node._on_stats_timer()
        record = self._published(node)
        self.assertTrue(yolo_boxes_ready(
            record, expected_generation=2,
            expected_id="pickup_box_0001_carryon",
            min_stamp=_Output.stamp - 1.0))

    def test_timer_without_a_pending_record_publishes_nothing(self):
        node = self._node(1.0)
        node._stats_dirty = True
        node._on_stats_timer()
        self.assertEqual(node._stats_pub.messages, [])

    def test_timer_does_not_republish_a_consumed_record(self):
        node = self._node(1.0)
        node._publish_stats(_Output())
        node._on_stats_timer()
        node._last_stats_pub = 0.0
        node._stats_dirty = True
        node._on_stats_timer()
        self.assertEqual(len(node._stats_pub.messages), 1)


if __name__ == "__main__":
    unittest.main()
