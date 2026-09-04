#!/usr/bin/env python3
"""Unit tests for DetectionTemporalGate (no ROS, no YOLO)."""

import unittest

import numpy as np

from luggage_perception.detection_temporal_gate import (
    DetectionTemporalGate,
    SuitcaseViewWait,
    bbox_iou,
    should_retry_estimate,
)
from luggage_perception.semantic_segmenter import LABEL_CARGO, LABEL_BACKGROUND


def _rgb(value, h=48, w=64):
    return np.full((h, w, 3), int(value), dtype=np.uint8)


def _det(bbox, label=LABEL_CARGO):
    return {"label": label, "prompt": "box", "confidence": 0.2, "bbox": bbox}


class TestBboxIou(unittest.TestCase):
    def test_identical_is_one(self):
        self.assertAlmostEqual(bbox_iou([0, 0, 10, 10], [0, 0, 10, 10]), 1.0)

    def test_disjoint_is_zero(self):
        self.assertEqual(bbox_iou([0, 0, 10, 10], [20, 20, 30, 30]), 0.0)


class TestRetryPolicy(unittest.TestCase):
    def test_estimation_failed_is_retryable(self):
        self.assertTrue(should_retry_estimate("DETECT_ESTIMATION_FAILED"))
        self.assertTrue(should_retry_estimate("DETECT_STALE_CLOUD"))
        self.assertTrue(should_retry_estimate("DETECT_STALE_INSTANCE"))
        self.assertFalse(should_retry_estimate("DETECT_HANDLER_ERROR"))
        self.assertFalse(should_retry_estimate("ok"))


class TestMajorityHold(unittest.TestCase):
    def test_miss_after_majority_paints_consensus_bbox(self):
        gate = DetectionTemporalGate(window_size=5, min_positive_ratio=0.5)
        bbox = [10, 8, 40, 30]
        rgb = _rgb(40)
        for _ in range(3):
            labels = np.zeros((48, 64), dtype=np.uint8)
            labels[8:30, 10:40] = LABEL_CARGO
            out, dets, stats = gate.apply(labels, [_det(bbox)], rgb)
            self.assertFalse(stats["held"])
            self.assertTrue((out == labels).all())
        empty = np.zeros((48, 64), dtype=np.uint8)
        out, dets, stats = gate.apply(empty, [], rgb)
        self.assertTrue(stats["held"])
        self.assertGreater(stats["positive_ratio"], 0.5)
        self.assertEqual(int((out == LABEL_CARGO).sum()), (40 - 10) * (30 - 8))
        self.assertTrue(dets and dets[-1].get("held"))
        self.assertEqual(dets[-1]["bbox"], bbox)

    def test_below_half_does_not_hold(self):
        gate = DetectionTemporalGate(window_size=5, min_positive_ratio=0.5)
        rgb = _rgb(40)
        bbox = [10, 8, 40, 30]
        labels = np.zeros((48, 64), dtype=np.uint8)
        labels[8:30, 10:40] = LABEL_CARGO
        gate.apply(labels, [_det(bbox)], rgb)
        for _ in range(4):
            out, dets, stats = gate.apply(
                np.zeros((48, 64), dtype=np.uint8), [], rgb)
        self.assertFalse(stats["held"])
        self.assertEqual(int((out == LABEL_CARGO).sum()), 0)
        self.assertEqual(dets, [])

    def test_empty_window_does_not_invent_a_box(self):
        gate = DetectionTemporalGate(window_size=5)
        out, dets, stats = gate.apply(
            np.zeros((48, 64), dtype=np.uint8), [], _rgb(40))
        self.assertFalse(stats["held"])
        self.assertTrue((out == LABEL_BACKGROUND).all())
        self.assertEqual(dets, [])


class TestSceneChangeReset(unittest.TestCase):
    def test_rgb_jump_drops_previous_suitcase(self):
        gate = DetectionTemporalGate(
            window_size=5, min_positive_ratio=0.5, scene_change_mad=10.0)
        bbox = [10, 8, 40, 30]
        labels = np.zeros((48, 64), dtype=np.uint8)
        labels[8:30, 10:40] = LABEL_CARGO
        for _ in range(5):
            gate.apply(labels, [_det(bbox)], _rgb(30))
        empty = np.zeros((48, 64), dtype=np.uint8)
        out, dets, stats = gate.apply(empty, [], _rgb(200))
        self.assertTrue(stats["scene_change"])
        self.assertFalse(stats["held"])
        self.assertEqual(int((out == LABEL_CARGO).sum()), 0)

    def test_bbox_iou_collapse_resets_before_hold(self):
        gate = DetectionTemporalGate(
            window_size=5, min_positive_ratio=0.5, bbox_iou_reset=0.3)
        rgb = _rgb(40)
        small = [8, 8, 20, 20]
        large = [4, 4, 60, 44]
        labels_s = np.zeros((48, 64), dtype=np.uint8)
        labels_s[8:20, 8:20] = LABEL_CARGO
        for _ in range(4):
            gate.apply(labels_s, [_det(small)], rgb)
        labels_l = np.zeros((48, 64), dtype=np.uint8)
        labels_l[4:44, 4:60] = LABEL_CARGO
        _, _, stats = gate.apply(labels_l, [_det(large)], rgb)
        self.assertTrue(stats["scene_change"])
        empty = np.zeros((48, 64), dtype=np.uint8)
        out, _, stats = gate.apply(empty, [], rgb)
        self.assertTrue(stats["held"])
        self.assertEqual(stats["held_bbox"], large)


def _platform_with_box(width_frac, value=180, bg=40, h=96, w=128):
    image = np.full((h, w, 3), bg, dtype=np.uint8)
    box_w = max(4, int(w * float(width_frac)))
    x0 = (w - box_w) // 2
    image[20:76, x0:x0 + box_w] = int(value)
    return image


class TestSuitcaseViewWait(unittest.TestCase):
    def test_same_view_stays_pending(self):
        wait = SuitcaseViewWait(
            update_mad=10.0, stable_mad=4.0, stable_frames=2)
        old = _platform_with_box(0.3)
        self.assertTrue(wait.note_box_id("pickup_box_0001", old))
        self.assertTrue(wait.pending)
        self.assertFalse(wait.observe(old))
        self.assertFalse(wait.observe(old))
        self.assertTrue(wait.pending)

    def test_size_jump_then_stable_is_ready(self):
        wait = SuitcaseViewWait(
            update_mad=10.0, stable_mad=4.0, stable_frames=2)
        old = _platform_with_box(0.3)
        new = _platform_with_box(0.7)
        wait.note_box_id("pickup_box_0002", old)
        self.assertFalse(wait.observe(new))
        self.assertTrue(wait.observe(new))
        self.assertFalse(wait.pending)
        self.assertTrue(wait.observe(new))

    def test_clear_cancels_wait(self):
        wait = SuitcaseViewWait()
        wait.note_box_id("pickup_box_0003", _platform_with_box(0.3))
        self.assertTrue(wait.note_box_id("", _platform_with_box(0.3)))
        self.assertFalse(wait.pending)
        self.assertTrue(wait.observe(_platform_with_box(0.3)))

    def test_same_id_does_not_rearm(self):
        wait = SuitcaseViewWait(stable_frames=1)
        old = _platform_with_box(0.3)
        new = _platform_with_box(0.7)
        wait.note_box_id("pickup_box_0004", old)
        wait.observe(new)
        self.assertFalse(wait.pending)
        self.assertFalse(wait.note_box_id("pickup_box_0004", old))
        self.assertFalse(wait.pending)
