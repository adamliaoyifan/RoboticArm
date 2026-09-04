#!/usr/bin/env python3
"""Unit tests for yolo_window_stats (no ROS, no YOLO)."""

import unittest

from luggage_perception.semantic_segmenter import LABEL_CARGO
from luggage_perception.eval.yolo_window_stats import (
    aabb_from_uv,
    annotate_gt,
    best_raw_cargo_bbox,
    percentile,
    postproc_hints,
    run_lengths,
    summarize_window,
)


def _frame(stamp, raw, bbox=None, held=False, infer_ms=4.0,
           gt_iou=None, dropped=0, prompt="suitcase", conf=0.2):
    dets = []
    if raw or held:
        dets.append({
            "label": LABEL_CARGO,
            "prompt": prompt,
            "confidence": conf,
            "bbox": bbox or [10, 10, 40, 40],
            "held": bool(held and not raw),
        })
    rec = {
        "image_stamp": float(stamp),
        "detect_sim_stamp": float(stamp) + 0.05,
        "detect_wall_sec": 1000.0 + float(stamp) + 0.01,
        "recv_wall_sec": 1000.0 + float(stamp),
        "infer_ms": infer_ms,
        "raw_cargo": bool(raw),
        "held": bool(held and not raw),
        "detections": dets,
        "n_dropped_self_body": int(dropped),
    }
    if gt_iou is not None:
        rec["gt_iou"] = gt_iou
        rec["gt_aligned"] = bool(gt_iou >= 0.3)
    return rec


class TestAabbAndGt(unittest.TestCase):
    def test_aabb_from_uv_clips(self):
        uv = [[-10, 5], [50, 5], [50, 80], [-10, 80]]
        box = aabb_from_uv(uv, valid=[True] * 4, image_size=(40, 60))
        self.assertEqual(box, [0, 5, 40, 60])

    def test_aabb_from_numpy_projection(self):
        import numpy as np
        uv = np.array(
            [[-10.0, 5.0], [50.0, 5.0], [50.0, 80.0], [-10.0, 80.0]],
            dtype=np.float64)
        valid = np.array([True, True, True, True])
        box = aabb_from_uv(uv, valid, image_size=(40, 60))
        self.assertEqual(box, [0, 5, 40, 60])
        self.assertIsNone(aabb_from_uv(np.zeros((0, 2)), np.zeros((0,), dtype=bool)))

    def test_annotate_gt_aligned(self):
        frame = _frame(1.0, True, bbox=[0, 0, 20, 20])
        out = annotate_gt(frame, [0, 0, 20, 20], thresh=0.3)
        self.assertAlmostEqual(out["gt_iou"], 1.0)
        self.assertTrue(out["gt_aligned"])

    def test_annotate_gt_false_positive(self):
        frame = _frame(1.0, True, bbox=[0, 0, 10, 10])
        out = annotate_gt(frame, [80, 80, 100, 100], thresh=0.3)
        self.assertEqual(out["gt_iou"], 0.0)
        self.assertFalse(out["gt_aligned"])

    def test_held_bbox_is_not_raw(self):
        self.assertIsNone(best_raw_cargo_bbox([{
            "label": LABEL_CARGO, "bbox": [1, 1, 8, 8], "held": True,
        }]))


class TestStreaksAndSummary(unittest.TestCase):
    def test_run_lengths(self):
        self.assertEqual(
            run_lengths([True, True, False, False, False, True]),
            [(True, 2), (False, 3), (True, 1)])

    def test_percentile(self):
        self.assertEqual(percentile([1, 2, 3, 4], 50), 2.5)
        self.assertIsNone(percentile([], 50))

    def test_hit_rate_and_miss_streak(self):
        # 7 hits, miss streak of 3 in the middle, 10 frames.
        flags = [1, 1, 1, 0, 0, 0, 1, 1, 1, 1]
        frames = [
            _frame(i * 0.33, bool(h), gt_iou=0.9 if h else None)
            for i, h in enumerate(flags)
        ]
        summary = summarize_window(frames)
        self.assertEqual(summary["n_frames"], 10)
        self.assertEqual(summary["n_raw_hit"], 7)
        self.assertAlmostEqual(summary["hit_rate"], 0.7)
        self.assertEqual(summary["miss_streaks"]["max"], 3)
        self.assertEqual(summary["n_gt_aligned"], 7)
        self.assertEqual(summary["n_false_positive"], 0)

    def test_false_positive_and_held_rescue(self):
        frames = [
            _frame(0.0, True, gt_iou=0.1, dropped=2),
            _frame(0.3, False, held=True, dropped=0),
        ]
        frames[1]["raw_cargo"] = False
        summary = summarize_window(frames)
        self.assertEqual(summary["n_raw_hit"], 1)
        self.assertEqual(summary["n_false_positive"], 1)
        self.assertEqual(summary["n_held_rescue"], 1)
        self.assertGreaterEqual(summary["mean_dropped_self_body"], 1.0)

    def test_low_hit_rate_hint(self):
        frames = [_frame(i, False) for i in range(20)]
        summary = summarize_window(frames)
        text = " ".join(summary["postproc_hints"])
        self.assertIn("cannot invent a box", text)

    def test_high_hit_short_miss_hint(self):
        flags = [1] * 18 + [0, 1]
        frames = [_frame(i * 0.3, bool(h), gt_iou=0.9) for i, h in enumerate(flags)]
        summary = summarize_window(frames)
        text = " ".join(postproc_hints(summary))
        self.assertIn("may be larger than needed", text)

    def test_hit_stamps_recorded(self):
        frames = [_frame(1.0, True, gt_iou=0.9), _frame(1.3, False)]
        summary = summarize_window(frames)
        self.assertEqual(summary["n_hit_stamp_rows"], 1)
        row = summary["hit_stamps"][0]
        self.assertEqual(row["image_stamp"], 1.0)
        self.assertAlmostEqual(row["lag_sim"], 0.05)
        self.assertAlmostEqual(summary["time_to_first_raw_cargo_sec"], 0.0)
