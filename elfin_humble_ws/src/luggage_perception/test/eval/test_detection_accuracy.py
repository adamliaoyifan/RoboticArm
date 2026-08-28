#!/usr/bin/env python3
"""Unit tests for DetectionAccuracy — footprint IoU gate, no ROS."""

import math
import unittest

import numpy as np

from luggage_perception.eval.detection_accuracy import (
    AccuracyResult,
    BoxObservation,
    DetectionAccuracy,
)


def _box(**kwargs):
    defaults = dict(
        x=0.0, y=0.0, z=1.0, yaw=0.0,
        width=0.70, depth=0.45, height=0.28,
    )
    defaults.update(kwargs)
    return BoxObservation(**defaults)


class TestDetectionAccuracy(unittest.TestCase):
    def setUp(self):
        self.gate = DetectionAccuracy()

    def test_identical_is_ok(self):
        r = self.gate.compare(_box(), _box())
        self.assertTrue(r.ok)
        self.assertEqual(r.reason, "ok")
        self.assertAlmostEqual(r.err_xy, 0.0)
        self.assertAlmostEqual(r.err_z, 0.0)
        self.assertAlmostEqual(r.err_yaw, 0.0)
        self.assertAlmostEqual(r.iou, 1.0, places=6)
        self.assertFalse(r.near_square)
        self.assertFalse(r.swapped)

    def test_xy_tolerance_boundary(self):
        inside = self.gate.compare(_box(x=0.03), _box())
        outside = self.gate.compare(_box(x=0.031), _box())
        self.assertTrue(inside.ok)
        self.assertFalse(outside.ok)
        self.assertEqual(outside.reason, "xy")

    def test_yaw_pi_folds_to_zero(self):
        r = self.gate.compare(_box(yaw=math.pi), _box())
        self.assertTrue(r.ok)
        self.assertAlmostEqual(r.err_yaw, 0.0, places=9)
        self.assertAlmostEqual(r.iou, 1.0, places=5)

    def test_90_with_wd_swap_matches(self):
        measured = _box(yaw=math.pi / 2.0, width=0.45, depth=0.70)
        r = self.gate.compare(measured, _box())
        self.assertTrue(r.ok)
        self.assertTrue(r.swapped)
        self.assertAlmostEqual(r.iou, 1.0, places=5)
        self.assertLess(abs(r.err_yaw), 1e-9)

    def test_true_90_without_swap_fails_via_iou(self):
        measured = _box(yaw=math.pi / 2.0, width=0.70, depth=0.45)
        r = self.gate.compare(measured, _box())
        self.assertFalse(r.ok)
        self.assertEqual(r.reason, "iou")
        self.assertLess(r.iou, 0.50)

    def test_near_square_zeros_scalar_yaw(self):
        gt = _box(width=0.55, depth=0.50, yaw=0.0)
        measured = _box(width=0.50, depth=0.55, yaw=math.pi / 2.0)
        r = self.gate.compare(measured, gt)
        self.assertTrue(r.near_square)
        self.assertTrue(r.swapped)
        self.assertAlmostEqual(r.err_yaw, 0.0)
        self.assertTrue(r.ok)
        self.assertEqual(r.reason, "near_square")
        self.assertAlmostEqual(r.iou, 1.0, places=5)

    def test_size_fail_is_not_reported_as_xy(self):
        r = self.gate.compare(_box(width=0.90), _box())
        self.assertFalse(r.ok)
        self.assertEqual(r.reason, "size")

    def test_summarize_percentiles_and_pass_rate(self):
        results = [
            AccuracyResult(
                ok=True, err_xy=0.01, err_z=0.0, err_xyz=0.01,
                err_width=0.0, err_depth=0.0, err_height=0.0,
                err_yaw=0.0, swapped=False, reason="ok", iou=0.95,
                near_square=False,
            ),
            AccuracyResult(
                ok=True, err_xy=0.02, err_z=0.01, err_xyz=0.022,
                err_width=0.01, err_depth=0.0, err_height=0.0,
                err_yaw=0.05, swapped=True, reason="ok", iou=0.90,
                near_square=False,
            ),
            AccuracyResult(
                ok=False, err_xy=0.10, err_z=-0.03, err_xyz=0.104,
                err_width=0.0, err_depth=0.0, err_height=0.0,
                err_yaw=0.0, swapped=False, reason="xy", iou=0.40,
                near_square=True,
            ),
        ]
        summary = self.gate.summarize(results)
        self.assertEqual(summary["n"], 3)
        self.assertAlmostEqual(summary["pass_rate"], 2.0 / 3.0)
        self.assertEqual(summary["n_near_square"], 1)
        self.assertEqual(summary["n_swapped"], 1)
        self.assertAlmostEqual(summary["p50"]["err_xy"], 0.02)
        self.assertGreater(summary["p95"]["err_xy"], summary["p50"]["err_xy"])
        self.assertAlmostEqual(summary["p95"]["err_xy"],
                               float(np.percentile([0.01, 0.02, 0.10], 95)))
        self.assertAlmostEqual(summary["p50"]["iou"], 0.90)


if __name__ == "__main__":
    unittest.main()
