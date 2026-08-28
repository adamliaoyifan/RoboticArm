#!/usr/bin/env python3
"""Unit tests for the rollout value estimator + CEM calibration (P4). No roscore."""

import os
import sys
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from luggage_packing.free_space_model import FreeSpaceModel
from luggage_packing.ems import EMS
from luggage_packing.value_estimator import (
    sample_item, mean_item_volume, value_hat, greedy_inner, apply_place,
    cem_calibrate, ProxyStrategy,
)
from harness.packing_sequences import catalog_entries, load_catalog

INNER = (1.49, 1.97, 2.01)
CENTER = (0.0, -1.5, 0.145)
SMALL = (0.55, 0.40, 0.25)  # carryon (smallest catalog)


def _entries():
    return catalog_entries(load_catalog())


def _model():
    return FreeSpaceModel(INNER, CENTER, yaw=-1.5708, resolution=0.10)


class TestSampling(unittest.TestCase):
    def test_mean_item_volume_matches_catalog(self):
        """E[V_item] over the catalog == 0.0845 m^3 (design §1.1)."""
        v = mean_item_volume(_entries())
        self.assertAlmostEqual(v, 0.0845, places=3)

    def test_sample_item_distribution(self):
        """Empirical sample frequencies approach the catalog weights."""
        import numpy as np
        rng = np.random.RandomState(0)
        entries = _entries()
        counts = {e["id"]: 0 for e in entries}
        n = 4000
        for _ in range(n):
            size = sample_item(entries, rng)
            for e in entries:
                if list(e["size"]) == size:
                    counts[e["id"]] += 1
                    break
        for e in entries:
            self.assertAlmostEqual(counts[e["id"]] / n, e["weight"], delta=0.03,
                                   msg="id=%s" % e["id"])


class TestValueHat(unittest.TestCase):
    def test_value_hat_in_range_and_deterministic(self):
        """V̂ for a floor candidate is in [0, ~1] and deterministic for a seed."""
        m = _model()
        e = EMS(INNER, min_useful_edge=0.1)
        cands = m.candidates([0.7, 0.45, 0.28], allowed_yaws=[0.0], top_n=5)
        self.assertGreater(len(cands), 0)
        cand = cands[0]
        v1 = value_hat(cand, m, e, _entries(), K=3, M=6, seed=42)
        v2 = value_hat(cand, m, e, _entries(), K=3, M=6, seed=42)
        self.assertGreaterEqual(v1, 0.0)
        self.assertLess(v1, 2.0)
        self.assertAlmostEqual(v1, v2, places=6, msg="same seed must reproduce V̂")

    def test_value_hat_uses_rollout_budget(self):
        """V̂ with K=0 is 0 (no future placements); K>0 accumulates volume."""
        m = _model()
        e = EMS(INNER, min_useful_edge=0.1)
        cand = m.candidates([0.7, 0.45, 0.28], allowed_yaws=[0.0], top_n=1)[0]
        v0 = value_hat(cand, m, e, _entries(), K=0, M=4, seed=1)
        self.assertEqual(v0, 0.0)
        v3 = value_hat(cand, m, e, _entries(), K=3, M=4, seed=1)
        self.assertGreater(v3, 0.0)

    def test_greedy_inner_returns_candidate(self):
        m = _model()
        c = greedy_inner(m, [0.7, 0.45, 0.28])
        self.assertIsNotNone(c)
        self.assertTrue(c["feasible"])

    def test_apply_place_non_mutating(self):
        """apply_place does not disturb the original model."""
        m = _model()
        e = EMS(INNER, min_useful_edge=0.1)
        cand = m.candidates([0.7, 0.45, 0.28], allowed_yaws=[0.0], top_n=1)[0]
        before_boxes = len(m.boxes)
        apply_place(m, e, cand)
        self.assertEqual(len(m.boxes), before_boxes, "original model must be unchanged")


class TestCEMCalibrate(unittest.TestCase):
    def test_cem_returns_weights_and_is_deterministic(self):
        """CEM returns a weight dict + score; same seed reproduces."""
        entries = _entries()
        seqs = [[sample_item(entries, __import__("numpy").random.RandomState(s)) for _ in range(8)]
                for s in range(3)]

        def factory(weights):
            def make_model():
                return FreeSpaceModel(INNER, CENTER, yaw=-1.5708, resolution=0.15)
            # Note: ProxyStrategy uses the module-level default weights; the CEM
            # search here validates the harness runs end-to-end and is deterministic.
            return ProxyStrategy(make_model, entries, INNER, SMALL, weights=weights)

        w1, s1 = cem_calibrate(factory, seqs, pop=6, elites=2, iters=2, seed=7)
        w2, s2 = cem_calibrate(factory, seqs, pop=6, elites=2, iters=2, seed=7)
        self.assertIsInstance(w1, dict)
        self.assertIn("ems_regularity", w1)
        self.assertAlmostEqual(s1, s2, places=6, msg="CEM must be deterministic for a fixed seed")


if __name__ == "__main__":
    unittest.main()
