#!/usr/bin/env python3
"""Unit tests for exact-stamp DetectionFrame join helpers (no roscore)."""

from __future__ import division

import unittest

from luggage_perception.detection_frame_join import (
    ExactStampJoin,
    empty_cargo_pca_fields,
    yolo_box_fields_from_compact,
    yolo_box_fields_from_detections,
)
from luggage_perception.semantic_segmenter import LABEL_CARGO, compact_detections


class TestExactStampJoin(unittest.TestCase):
    def test_same_stamp_yolo_and_empty_cargo(self):
        join = ExactStampJoin(maxlen=10)
        yolo = {"detections": [{"label": LABEL_CARGO, "bbox": [1, 2, 3, 4]}]}
        self.assertIsNone(join.push_left((1, 0), yolo))
        pair = join.push_right((1, 0), [])
        self.assertIsNotNone(pair)
        self.assertEqual(pair[0], yolo)
        self.assertEqual(pair[1], [])
        fields = empty_cargo_pca_fields(len(pair[1]))
        self.assertFalse(fields["pca_valid"])
        self.assertEqual(fields["pca_reason"], "DETECT_NO_CLOUD")
        self.assertEqual(fields["pca_source"], "empty")
        self.assertEqual(fields["n_cargo_points"], 0)

    def test_out_of_order_sides_still_join(self):
        join = ExactStampJoin(maxlen=10)
        self.assertIsNone(join.push_right((2, 5), "cargo"))
        pair = join.push_left((2, 5), "yolo")
        self.assertEqual(pair, ("yolo", "cargo"))

    def test_maxlen_drops_oldest_unmatched(self):
        join = ExactStampJoin(maxlen=2)
        join.push_left((1, 0), "a")
        join.push_left((2, 0), "b")
        join.push_left((3, 0), "c")
        self.assertIsNone(join.push_right((1, 0), "old"))
        pair = join.push_right((3, 0), "new")
        self.assertEqual(pair, ("c", "new"))


class TestYoloBoxFields(unittest.TestCase):
    def test_compact_detections_map_to_yolo_box_fields(self):
        compact = compact_detections([{
            "label": LABEL_CARGO,
            "prompt": "box",
            "confidence": 0.4,
            "bbox": [1.2, 2.8, 9.1, 10.6],
            "held": True,
            "mask": object(),
        }])
        boxes = yolo_box_fields_from_compact(compact)
        self.assertEqual(len(boxes), 1)
        self.assertEqual(boxes[0]["label"], LABEL_CARGO)
        self.assertEqual(boxes[0]["prompt"], "box")
        self.assertAlmostEqual(boxes[0]["confidence"], 0.4)
        self.assertEqual(boxes[0]["bbox"], [1, 3, 9, 11])
        self.assertTrue(boxes[0]["held"])
        self.assertNotIn("mask", boxes[0])

    def test_missing_bbox_pads_zeros(self):
        boxes = yolo_box_fields_from_detections([{"label": 2, "prompt": "x"}])
        self.assertEqual(boxes[0]["bbox"], [0, 0, 0, 0])


if __name__ == "__main__":
    unittest.main()
