#!/usr/bin/env python3
"""YOLO wait scans the stats ring, not only the latest latch (no ROS)."""

from __future__ import division

import unittest

from luggage_gazebo.eval_metrics import first_yolo_ready, yolo_boxes_ready


def _cargo(generation, instance_id, stamp, raw_cargo=True, held=False):
    det = {"label": 2, "held": held, "confidence": 0.73}
    return {
        "generation": generation,
        "instance_id": instance_id,
        "stamp": stamp,
        "raw_cargo": raw_cargo,
        "detections": [det],
    }


class TestFirstYoloReadyRing(unittest.TestCase):

    def test_matching_ring_sample_wins_over_stale_latch(self):
        latch = _cargo(1, "pickup_box_0000_carryon", 9.0)
        ready = _cargo(2, "pickup_box_0001_carryon", 11.0)
        match = first_yolo_ready(
            [ready, latch], expected_generation=2,
            expected_id="pickup_box_0001_carryon", min_stamp=10.0)
        self.assertIs(match, ready)

    def test_newest_first_order_finds_matching_generation(self):
        stale = _cargo(1, "old", 10.5)
        ready = _cargo(2, "pickup_box_0001_carryon", 11.0)
        match = first_yolo_ready(
            [ready, stale], 2, expected_id="pickup_box_0001_carryon",
            min_stamp=10.0)
        self.assertIs(match, ready)
        self.assertFalse(yolo_boxes_ready(
            stale, 2, "pickup_box_0001_carryon", 10.0))

    def test_timeout_keeps_last_checked_when_generation_missing(self):
        latch = {"stamp": 11.0, "raw_cargo": True, "detections": [
            {"label": 2, "held": False}]}
        records = [latch]
        match = first_yolo_ready(
            records, 2, expected_id="pickup_box_0001_carryon", min_stamp=10.0)
        self.assertIsNone(match)
        decision = records[0]
        self.assertEqual(int(decision.get("generation") or 0), 0)

    def test_held_only_does_not_satisfy_gate(self):
        held = _cargo(2, "pickup_box_0001_carryon", 11.0, raw_cargo=False,
                      held=True)
        held["detections"] = [{"label": 2, "held": True, "confidence": 0.73}]
        self.assertIsNone(first_yolo_ready(
            [held], 2, expected_id="pickup_box_0001_carryon", min_stamp=10.0))


if __name__ == "__main__":
    unittest.main()
