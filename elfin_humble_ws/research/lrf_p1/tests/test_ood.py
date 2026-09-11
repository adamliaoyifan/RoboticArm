"""Fail closed on empty, non-finite, and malformed observations."""

from __future__ import division

import os
import sys
import unittest

import numpy as np

WORKSPACE = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, os.path.join(WORKSPACE, "src", "luggage_description"))
sys.path.insert(0, os.path.join(WORKSPACE, "src", "luggage_perception"))
sys.path.insert(0, WORKSPACE)

from research.lrf_p1.baseline_adapter import estimate_from_observation
from research.lrf_p1.candidate import ResidualEnsemble, infer
from research.lrf_p1.contracts import (
    REASON_EMPTY,
    REASON_MALFORMED,
    REASON_NONFINITE,
    REASON_TOO_FEW,
    REASON_UNAVAILABLE,
)


class OodFailClosedTest(unittest.TestCase):
    def test_empty_cloud(self):
        out = estimate_from_observation({"points": np.zeros((0, 3))})
        self.assertFalse(out["ok"])
        self.assertEqual(out["reason"], REASON_EMPTY)

    def test_nonfinite(self):
        pts = np.ones((80, 3))
        pts[3, 1] = np.nan
        out = estimate_from_observation({"points": pts})
        self.assertFalse(out["ok"])
        self.assertEqual(out["reason"], REASON_NONFINITE)

    def test_inf(self):
        pts = np.ones((80, 3))
        pts[4, 2] = np.inf
        out = estimate_from_observation({"points": pts})
        self.assertFalse(out["ok"])
        self.assertEqual(out["reason"], REASON_NONFINITE)

    def test_malformed_shape(self):
        out = estimate_from_observation({"points": np.ones((10,))})
        self.assertFalse(out["ok"])
        self.assertEqual(out["reason"], REASON_MALFORMED)

    def test_too_few(self):
        out = estimate_from_observation({"points": np.ones((8, 3))})
        self.assertFalse(out["ok"])
        self.assertEqual(out["reason"], REASON_TOO_FEW)

    def test_infer_empty_does_not_hallucinate(self):
        ens = ResidualEnsemble()
        ens.available = True
        ens.models = []
        out = infer({"points": np.zeros((0, 3))}, ens)
        self.assertFalse(out["ok"])
        self.assertEqual(out["reason"], REASON_EMPTY)
        self.assertIsNone(out["box"])

    def test_unavailable_model(self):
        pts = np.random.RandomState(0).randn(80, 3) + np.array([0.0, 0.0, 1.0])
        out = infer({"points": pts, "roi_center_xy": (0.0, 0.0)}, None)
        self.assertFalse(out["ok"])
        self.assertEqual(out["reason"], REASON_UNAVAILABLE)
        self.assertTrue(out["unknown"])

    def test_unfitted_ensemble(self):
        ens = ResidualEnsemble()
        pts = np.random.RandomState(1).randn(80, 3)
        out = infer({"points": pts}, ens)
        self.assertEqual(out["reason"], REASON_UNAVAILABLE)

    def test_lock_pose_keeps_xy_yaw(self):
        from research.lrf_p1.candidate import apply_residual
        base = np.array([0.7, 0.4, 0.28, 0.01, -0.02, 0.14, 0.1])
        mean = np.array([0.02, 0.01, 0.04, 0.20, 0.15, -0.10, 1.2])
        locked = apply_residual(base, mean, lock_pose=True)
        self.assertAlmostEqual(locked[3], 0.01)
        self.assertAlmostEqual(locked[4], -0.02)
        self.assertAlmostEqual(locked[6], 0.1)
        self.assertAlmostEqual(locked[2], 0.32)
        self.assertAlmostEqual(locked[5], 0.14 + 0.5 * 0.04)


if __name__ == "__main__":
    unittest.main()
