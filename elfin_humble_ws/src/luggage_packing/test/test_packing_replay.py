#!/usr/bin/env python3
"""Unit tests for the offline packing replay harness (P0-a). No roscore.

Covers BinPackerStrategy geometry, ReachabilityAtlas query, sequence
reproducibility, and ReplaySimulator metrics. The atlas integration test uses
the real s20 atlas if present.
"""

import os
import sys
import unittest
import yaml

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from luggage_packing.packing_replay import (
    BinPackerStrategy,
    ReachabilityAtlas,
    ReplaySimulator,
    STATUS_REACHABLE,
    STATUS_UNKNOWN,
)
from harness.packing_sequences import generate_sequence, catalog_entries, load_catalog

_REAL_ATLAS = os.path.normpath(os.path.join(
    PKG_ROOT, "..", "luggage_planning", "data", "reachability_atlas",
    "s20_container_collision_aware.npz"))


class _MockAtlas(object):
    """Atlas stub: reachable iff the candidate center x is on the opening side."""

    def __init__(self, reachable_x_min, inner_size, v_reachable=1.0):
        self.reachable_x_min = reachable_x_min
        self.inner_size = inner_size
        self._v = v_reachable

    def is_reachable(self, center_local, yaw=0.0):
        return center_local[0] <= self.reachable_x_min

    def reachable_volume(self):
        return self._v


class TestBinPackerStrategy(unittest.TestCase):
    INNER = (1.49, 1.97, 2.01)

    def test_first_box_is_back_left_corner(self):
        """Empty container -> the only candidate is the -X/-Y floor corner (DBLF)."""
        strat = BinPackerStrategy(self.INNER)
        cands = list(strat.candidates([0.7, 0.45, 0.28], []))
        self.assertEqual(len(cands), 1, "empty container yields exactly one corner candidate")
        c = cands[0]
        self.assertAlmostEqual(c["min_x"], -self.INNER[0] * 0.5, places=4)
        self.assertAlmostEqual(c["min_y"], -self.INNER[1] * 0.5, places=4)
        self.assertAlmostEqual(c["min_z"], 0.0, places=4)

    def test_candidates_after_placement(self):
        """After placing a box, generated candidates don't overlap it and have support."""
        strat = BinPackerStrategy(self.INNER)
        first = next(strat.candidates([0.7, 0.45, 0.28], []))
        cands = list(strat.candidates([0.7, 0.45, 0.28], [first]))
        self.assertGreater(len(cands), 0)
        for c in cands:
            self.assertFalse(strat._intersects(c, [first]),
                             "candidate must not overlap the placed box")
            self.assertTrue(strat._has_support(c, [first]),
                            "candidate must be supported (floor or box top)")

    def test_has_support_on_top(self):
        """A candidate resting on a placed box (min_z just above occ.max_z) is supported."""
        strat = BinPackerStrategy(self.INNER)
        first = next(strat.candidates([0.7, 0.45, 0.28], []))
        on_top = {
            "min_x": first["min_x"], "max_x": first["max_x"],
            "min_y": first["min_y"], "max_y": first["max_y"],
            "min_z": first["max_z"] + 0.001,
            "max_z": first["max_z"] + 0.001 + 0.28,
        }
        self.assertTrue(strat._has_support(on_top, [first]))


class TestSequenceReproducibility(unittest.TestCase):
    def test_same_seed_same_sequence(self):
        a = generate_sequence(7, 20)
        b = generate_sequence(7, 20)
        self.assertEqual(a, b)

    def test_different_seed_different_sequence(self):
        a = generate_sequence(1, 20)
        b = generate_sequence(2, 20)
        self.assertNotEqual(a, b)

    def test_catalog_distribution(self):
        """Catalog has the three documented box types with expected probabilities."""
        entries = catalog_entries(load_catalog())
        ids = {e["id"]: e["weight"] for e in entries}
        self.assertIn("carryon", ids)
        self.assertIn("standard", ids)
        self.assertIn("large", ids)
        total = sum(ids.values())
        self.assertAlmostEqual(total, 1.0, places=2)


class TestReplaySimulator(unittest.TestCase):
    INNER = (1.49, 1.97, 2.01)

    def test_places_when_reachable(self):
        """If the atlas says the corner is reachable, B0 places the first box."""
        # Mock: opening is at -X, so the -X corner (min_x=-0.745) is reachable.
        atlas = _MockAtlas(reachable_x_min=0.0, inner_size=self.INNER, v_reachable=1.0)
        strat = BinPackerStrategy(self.INNER)
        sim = ReplaySimulator(strat, atlas=atlas, retry=False)
        metrics = sim.run([[0.7, 0.45, 0.28]])
        self.assertEqual(metrics["items_placed"], 1)
        self.assertGreater(metrics["V_placed"], 0.0)
        self.assertGreater(metrics["reachable_fill_rate"], 0.0)
        self.assertEqual(metrics["first_fail_index"], -1)

    def test_fails_when_unreachable_b0(self):
        """B0: first candidate unreachable -> box lost, first_fail_index=0."""
        atlas = _MockAtlas(reachable_x_min=-1.0, inner_size=self.INNER, v_reachable=1.0)
        strat = BinPackerStrategy(self.INNER)
        sim = ReplaySimulator(strat, atlas=atlas, retry=False)
        metrics = sim.run([[0.7, 0.45, 0.28]])
        self.assertEqual(metrics["items_placed"], 0)
        self.assertEqual(metrics["first_fail_index"], 0)
        self.assertEqual(metrics["reachable_fill_rate"], 0.0)

    def test_metrics_volume_ratios(self):
        """reachable_fill_rate = V_placed / V_reachable; overall = V_placed / V_container."""
        atlas = _MockAtlas(reachable_x_min=0.0, inner_size=self.INNER, v_reachable=0.5)
        strat = BinPackerStrategy(self.INNER)
        sim = ReplaySimulator(strat, atlas=atlas, retry=False)
        metrics = sim.run([[0.7, 0.45, 0.28]])
        v_container = self.INNER[0] * self.INNER[1] * self.INNER[2]
        self.assertAlmostEqual(metrics["V_container"], v_container, places=3)
        self.assertAlmostEqual(metrics["V_reachable"], 0.5, places=3)
        self.assertAlmostEqual(
            metrics["reachable_fill_rate"],
            metrics["V_placed"] / 0.5, places=3)
        self.assertAlmostEqual(
            metrics["overall_fill_rate"],
            metrics["V_placed"] / v_container, places=3)


class TestAtlasIntegration(unittest.TestCase):
    def test_real_atlas_query(self):
        """If the real s20 atlas is present, a known REACHABLE cell queries True."""
        if not os.path.isfile(_REAL_ATLAS):
            self.skipTest("real atlas not present: %s" % _REAL_ATLAS)
        atlas = ReachabilityAtlas(_REAL_ATLAS)
        import numpy as np
        reachable = np.argwhere(atlas.status == STATUS_REACHABLE)
        self.assertGreater(len(reachable), 0, "atlas must have reachable cells")
        ix, iy, iz, iyaw = reachable[0]
        cx = atlas.origin[0] + (ix + 0.5) * atlas.resolution
        cy = atlas.origin[1] + (iy + 0.5) * atlas.resolution
        cz = atlas.origin[2] + (iz + 0.5) * atlas.resolution
        yaw = atlas.yaw_bins[iyaw]
        self.assertTrue(atlas.is_reachable((cx, cy, cz), yaw))
        self.assertGreater(atlas.reachable_volume(), 0.0)
        # Sanity: sidecar metadata (when present) matches the NPZ. Do not
        # hard-code a historical count; rebuilding the atlas is expected.
        metadata_path = os.path.splitext(_REAL_ATLAS)[0] + ".yaml"
        if os.path.isfile(metadata_path):
            with open(metadata_path, "r") as stream:
                metadata = yaml.safe_load(stream) or {}
            expected = int(metadata.get("stats", {}).get(
                "reachable_cells", -1))
            self.assertEqual(
                int((atlas.status == STATUS_REACHABLE).sum()), expected)


if __name__ == "__main__":
    unittest.main()
