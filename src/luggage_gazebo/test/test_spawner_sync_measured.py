#!/usr/bin/env python3
"""PAYLOAD-GEOM: sync_detected_pickup_box handler behavior (bare node).

Drives the spawner's sync/clear/get_current handlers directly with a stub
publisher (same pattern as test_pf_r5a_fix1_spawn_fail_closed.py):
CAS on generation, height_valid gating, measured lifecycle, and the
schema-2 topic carrying no GT fields.
"""

import importlib.util
import json
import os
import unittest

from geometry_msgs.msg import Pose
from luggage_msgs.msg import DetectedLuggage
from luggage_msgs.srv import GetCurrentBox, SyncPickupBox

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.normpath(os.path.join(_HERE, ".."))  # src/luggage_gazebo

_spec = importlib.util.spec_from_file_location(
    "pickup_box_spawner_node",
    os.path.join(_PKG_ROOT, "scripts", "pickup_box_spawner_node.py"))
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)


class _Logger(object):
    def info(self, *_a, **_k):
        pass

    def error(self, *_a, **_k):
        pass


class _Pub(object):
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


def _detected(width=0.68, depth=0.44, height=0.26,
              height_valid=True, height_source=1):
    box = DetectedLuggage()
    box.id = "suitcase_standard_vintage_0003"
    box.width = float(width)
    box.depth = float(depth)
    box.height = float(height)
    box.yaw_valid = True
    box.height_valid = bool(height_valid)
    box.height_source = int(height_source)
    box.pose = Pose()
    return box


def _make_spawner(generation=7, with_box=True):
    node = object.__new__(_mod.PickupBoxSpawner)
    node._generation = int(generation)
    node._published_id = (
        "suitcase_standard_vintage_0003" if with_box else "")
    node._current_box = _detected() if with_box else None
    node._current_model = "pickup_box_0042_std" if with_box else None
    node._current_mass = 12.5
    node._current_ref = None
    node._measured = None
    node._box_pub = _Pub()
    node.get_logger = lambda: _Logger()
    return node


def _sync_request(expected_generation, box=None):
    request = SyncPickupBox.Request()
    request.box = box if box is not None else _detected()
    request.expected_generation = int(expected_generation)
    return request


class TestSyncDetected(unittest.TestCase):

    def test_sync_publishes_measured_without_generation_bump(self):
        node = _make_spawner(generation=7)
        response = node.handle_sync_detected(
            _sync_request(7, _detected()), None)
        self.assertTrue(response.success)
        self.assertEqual(response.generation, 7)
        self.assertEqual(len(node._box_pub.published), 1)
        payload = json.loads(node._box_pub.published[0].data)
        self.assertEqual(payload["schema"], 2)
        self.assertEqual(payload["id"], "suitcase_standard_vintage_0003")
        self.assertEqual(payload["generation"], 7)
        self.assertEqual(
            [payload["measured"]["width"], payload["measured"]["depth"],
             payload["measured"]["height"]],
            [0.68, 0.44, 0.26])
        self.assertEqual(payload["measured"]["height_source"], 1)
        # Privilege boundary: no GT fields on the topic.
        for key in ("width", "depth", "height", "yaw", "mass_kg", "pose",
                    "model_name", "size_mode", "visual_kind", "gt_reference"):
            self.assertNotIn(key, payload)

    def test_generation_mismatch_is_rejected(self):
        node = _make_spawner(generation=7)
        response = node.handle_sync_detected(
            _sync_request(6, _detected()), None)
        self.assertFalse(response.success)
        self.assertIn("GENERATION_MISMATCH", response.message)
        self.assertEqual(response.generation, 7)
        self.assertEqual(node._box_pub.published, [])
        self.assertIsNone(node._measured)

    def test_height_invalid_is_rejected(self):
        node = _make_spawner(generation=7)
        response = node.handle_sync_detected(
            _sync_request(7, _detected(height_valid=False)), None)
        self.assertFalse(response.success)
        self.assertIn("DETECT_FULL_GEOMETRY_REQUIRED", response.message)
        self.assertIsNone(node._measured)

    def test_non_positive_dimension_is_rejected(self):
        node = _make_spawner(generation=7)
        response = node.handle_sync_detected(
            _sync_request(7, _detected(height=0.0)), None)
        self.assertFalse(response.success)
        self.assertIn("DETECT_FULL_GEOMETRY_REQUIRED", response.message)
        self.assertIsNone(node._measured)

    def test_no_current_box_is_rejected(self):
        node = _make_spawner(generation=7, with_box=False)
        response = node.handle_sync_detected(_sync_request(7), None)
        self.assertFalse(response.success)
        self.assertEqual(response.message, "NO_CURRENT_BOX")

    def test_clear_drops_measured(self):
        node = _make_spawner(generation=7)
        node.handle_sync_detected(_sync_request(7), None)
        self.assertIsNotNone(node._measured)
        node._delete_model = lambda *_a, **_k: None
        node._current_model = "pickup_box_0042_std"
        response = node.handle_clear(None, None)
        self.assertTrue(response.success)
        self.assertIsNone(node._measured)
        payload = json.loads(node._box_pub.published[-1].data)
        self.assertEqual(payload["id"], "")
        self.assertNotIn("measured", payload)

    def test_resync_overwrites_previous_measurement(self):
        node = _make_spawner(generation=7)
        node.handle_sync_detected(_sync_request(7, _detected()), None)
        node.handle_sync_detected(
            _sync_request(7, _detected(0.72, 0.50, 0.31)), None)
        payload = json.loads(node._box_pub.published[-1].data)
        self.assertEqual(
            [payload["measured"]["width"], payload["measured"]["depth"],
             payload["measured"]["height"]],
            [0.72, 0.50, 0.31])


class TestGetCurrentPull(unittest.TestCase):

    def test_response_carries_gt_and_generation(self):
        node = _make_spawner(generation=7)
        response = node.handle_get_current(None, None)
        self.assertTrue(response.success)
        self.assertEqual(response.generation, 7)
        self.assertAlmostEqual(response.mass_kg, 12.5)
        self.assertEqual(response.model_name, "pickup_box_0042_std")
        self.assertAlmostEqual(response.box.width, 0.68)

    def test_empty_spawner_fails(self):
        node = _make_spawner(generation=9, with_box=False)
        response = node.handle_get_current(None, None)
        self.assertFalse(response.success)
        self.assertEqual(response.generation, 9)


if __name__ == "__main__":
    unittest.main()
