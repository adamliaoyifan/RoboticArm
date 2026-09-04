#!/usr/bin/env python3
"""Unit tests for CargoInstanceTracker (no ROS)."""

from __future__ import division

import unittest

import numpy as np

from luggage_perception.cargo_instance_tracker import (
    SOURCE_EMPTY,
    SOURCE_HOLD_TRACK,
    SOURCE_MEASURE,
    SOURCE_REJECT_CLUTTER,
    CargoInstanceTracker,
    parse_current_box_payload,
    transform_points,
    xyz_array,
)


def _cloud(cx, cy, cz=0.1, n=8, jitter=0.0):
    rng = np.random.RandomState(0)
    pts = np.zeros((n, 3), dtype=np.float64)
    pts[:, 0] = cx + jitter * rng.randn(n)
    pts[:, 1] = cy + jitter * rng.randn(n)
    pts[:, 2] = cz
    return pts


class TestParseCurrentBox(unittest.TestCase):

    def test_empty_payload(self):
        self.assertEqual(parse_current_box_payload(""), ("", 0))
        self.assertEqual(parse_current_box_payload(None), ("", 0))
        self.assertEqual(parse_current_box_payload("{}"), ("", 0))

    def test_id_and_generation(self):
        payload = '{"id": "pickup_box_0004_carryon", "generation": 7}'
        self.assertEqual(
            parse_current_box_payload(payload),
            ("pickup_box_0004_carryon", 7))

    def test_model_name_fallback(self):
        self.assertEqual(
            parse_current_box_payload('{"model_name": "box_a", "generation": 2}'),
            ("box_a", 2))

    def test_dict_passthrough(self):
        self.assertEqual(
            parse_current_box_payload({"id": "x", "generation": 3}),
            ("x", 3))


class TestXyzArray(unittest.TestCase):

    def test_empty(self):
        self.assertEqual(xyz_array([]).shape, (0, 3))
        self.assertEqual(xyz_array(None).shape, (0, 3))

    def test_tuples_with_label(self):
        arr = xyz_array([(1.0, 2.0, 3.0, 2, 1)])
        self.assertEqual(arr.shape, (1, 3))
        np.testing.assert_allclose(arr[0], [1.0, 2.0, 3.0])


class TestTransformPoints(unittest.TestCase):

    def test_identity(self):
        pts = np.array([[1.0, 2.0, 3.0]])
        out = transform_points(pts, np.eye(3), [0.5, 0.0, 0.0])
        np.testing.assert_allclose(out, [[1.5, 2.0, 3.0]])


class TestCargoInstanceTracker(unittest.TestCase):

    def test_first_measurement_is_accepted(self):
        tracker = CargoInstanceTracker()
        tracker.set_epoch(2, "pickup_box_0001_carryon")
        src = tracker.observe(1.0, _cloud(0.4, 0.2))
        self.assertEqual(src, SOURCE_MEASURE)
        self.assertEqual(tracker.n_points, 8)
        self.assertAlmostEqual(tracker.centroid[0], 0.4, places=6)

    def test_nearby_measurement_replaces_cloud(self):
        tracker = CargoInstanceTracker(associate_radius_m=0.15)
        tracker.set_epoch(2, "box")
        tracker.observe(1.0, _cloud(0.40, 0.20, n=4))
        src = tracker.observe(1.1, _cloud(0.41, 0.20, n=12))
        self.assertEqual(src, SOURCE_MEASURE)
        self.assertEqual(tracker.n_points, 12)
        self.assertAlmostEqual(tracker.cloud_stamp, 1.1)

    def test_far_measurement_is_clutter(self):
        tracker = CargoInstanceTracker(associate_radius_m=0.15)
        tracker.set_epoch(2, "box")
        tracker.observe(1.0, _cloud(0.40, 0.20, n=4))
        src = tracker.observe(1.2, _cloud(1.40, 0.20, n=9))
        self.assertEqual(src, SOURCE_REJECT_CLUTTER)
        self.assertEqual(tracker.n_points, 4)
        self.assertAlmostEqual(tracker.centroid[0], 0.40, places=6)
        self.assertAlmostEqual(tracker.cloud_stamp, 1.2)

    def test_yolo_miss_holds_track(self):
        tracker = CargoInstanceTracker()
        tracker.set_epoch(2, "box")
        tracker.observe(1.0, _cloud(0.4, 0.2))
        src = tracker.observe(1.3, np.zeros((0, 3)))
        self.assertEqual(src, SOURCE_HOLD_TRACK)
        self.assertEqual(tracker.n_points, 8)
        self.assertAlmostEqual(tracker.cloud_stamp, 1.3)

    def test_epoch_wipe(self):
        tracker = CargoInstanceTracker()
        tracker.set_epoch(2, "old")
        tracker.observe(1.0, _cloud(0.4, 0.2))
        changed = tracker.set_epoch(3, "")
        self.assertTrue(changed)
        self.assertEqual(tracker.n_points, 0)
        self.assertEqual(tracker.source, SOURCE_EMPTY)
        self.assertEqual(tracker.generation, 3)
        self.assertEqual(tracker.as_dict()["n_points"], 0)

    def test_cleared_epoch_ignores_lingering_cloud(self):
        tracker = CargoInstanceTracker()
        tracker.set_epoch(4, "")
        src = tracker.observe(2.0, _cloud(0.4, 0.2))
        self.assertEqual(src, SOURCE_EMPTY)
        self.assertEqual(tracker.n_points, 0)

    def test_same_epoch_is_noop(self):
        tracker = CargoInstanceTracker()
        self.assertTrue(tracker.set_epoch(1, "a"))
        tracker.observe(1.0, _cloud(0.1, 0.1))
        self.assertFalse(tracker.set_epoch(1, "a"))
        self.assertEqual(tracker.n_points, 8)

    def test_generation_zero_without_id_still_tracks(self):
        """Real robot: no current_box topic, generation stays 0."""
        tracker = CargoInstanceTracker()
        src = tracker.observe(1.0, _cloud(0.2, 0.1))
        self.assertEqual(src, SOURCE_MEASURE)
        self.assertEqual(tracker.n_points, 8)

    def test_tf_miss_does_not_replace(self):
        tracker = CargoInstanceTracker()
        tracker.set_epoch(2, "box")
        tracker.observe(1.0, _cloud(0.4, 0.2, n=5))
        src = tracker.note_tf_miss(1.4)
        self.assertEqual(src, SOURCE_HOLD_TRACK)
        self.assertEqual(tracker.n_points, 5)
        self.assertAlmostEqual(tracker.cloud_stamp, 1.4)


if __name__ == "__main__":
    unittest.main()
