#!/usr/bin/env python3
"""seed_chain: same-branch pairing of atlas contact/transit seeds."""

import math
import os
import sys
import unittest

import numpy as np

_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

from luggage_planning import seed_chain  # noqa: E402
from luggage_planning.reachability_atlas import ReachabilityAtlas  # noqa: E402

RES = 0.15
ORIGIN = [-0.75, -0.3, 0.5]
YAW_BINS = [0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0]
GRID = {"resolution_xyz": RES, "origin": ORIGIN, "size": [6, 4, 6],
        "yaw_bins": YAW_BINS}

BRANCH_A = [1.0, -1.2, 0.4, 0.8, 1.57, 0.5]
BRANCH_A_NEAR = [1.05, -1.15, 0.45, 0.85, 1.52, 0.55]
BRANCH_B = [1.0 + math.pi, -1.2, 0.4, 0.8, 1.57, 0.5 + math.pi / 2.0]

SLOT = (0.0, 0.0, 0.65)      # container frame
PORTAL = (-0.7, 0.0, 1.3)
IDENTITY = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


def _make_atlas(slot_seed, portal_seed):
    shape = (6, 4, 6, 4)
    status = np.zeros(shape, dtype=np.uint8)
    status[:] = 3  # REACHABLE everywhere
    contact = np.full(shape + (3, 6), np.nan, dtype=np.float64)
    transit = np.full(shape + (3, 6), np.nan, dtype=np.float64)

    def cell_indices(xyz):
        return tuple(int((v - o) / RES) for v, o in zip(xyz, ORIGIN))

    contact[cell_indices(SLOT) + (0,)] = slot_seed
    transit[cell_indices(PORTAL) + (0,)] = portal_seed
    contact_ik = np.zeros(shape, dtype=bool)
    transit_ik = np.zeros(shape, dtype=bool)
    count = np.zeros(shape, dtype=np.uint8)
    contact_ik[cell_indices(SLOT)] = True
    transit_ik[cell_indices(PORTAL)] = True
    count[cell_indices(SLOT)] = 1
    count[cell_indices(PORTAL)] = 1
    data = {
        "status": status,
        "contact_seeds": contact,
        "transit_seeds": transit,
        "contact_ik": contact_ik,
        "transit_ik": transit_ik,
        "solution_count": count,
        "opening_connected": np.ones(shape, dtype=bool),
    }
    return ReachabilityAtlas(data, {
        "grid": GRID, "contract": {"yaw_bins": YAW_BINS},
        "atlas_version": "3.0", "schema_version": 3})


class _PassthroughRefine(unittest.TestCase):
    """Pairing logic tests: refine is identity so seeds stay as authored."""

    def setUp(self):
        self._orig = seed_chain._refine_seeds
        seed_chain._refine_seeds = (
            lambda urdf, path, xyz, quat, seeds: (
                [list(s) for s in seeds], 0))

    def tearDown(self):
        seed_chain._refine_seeds = self._orig

    def _report(self, slot_seed, portal_seed, heading=0.0):
        atlas = _make_atlas(slot_seed, portal_seed)
        return seed_chain.chain_report(
            atlas, None, None, SLOT, PORTAL, heading, IDENTITY)

    def test_same_branch_pair(self):
        report = self._report(BRANCH_A, BRANCH_A_NEAR)
        self.assertTrue(report["same_branch"])
        self.assertLessEqual(report["branch_gap_rad"], 0.25)
        self.assertEqual(report["n_slot_seeds"], 1)
        self.assertEqual(report["n_portal_seeds"], 1)
        self.assertEqual(report["reason"], "")

    def test_different_branch_pair(self):
        report = self._report(BRANCH_A, BRANCH_B)
        self.assertFalse(report["same_branch"])
        self.assertGreater(report["branch_gap_rad"], 0.25)

    def test_directed_heading_selects_own_bin(self):
        # The same cells carry no seeds in the pi bin: asking with heading
        # pi must report unknown, not silently reuse the 0-bin seeds.
        report = self._report(BRANCH_A, BRANCH_A_NEAR, heading=math.pi)
        self.assertIsNone(report["same_branch"])
        self.assertEqual(report["reason"], "no_slot_seeds")

    def test_no_seeds_is_unknown_never_reject(self):
        atlas = _make_atlas(BRANCH_A, BRANCH_A_NEAR)
        report = seed_chain.chain_report(
            atlas, None, None, (0.05, 0.05, 0.55), PORTAL, 0.0, IDENTITY)
        # Off-seed cell borrows neighbours; with none seeded this stays
        # unknown rather than a fabricated verdict.
        self.assertIn(report["same_branch"], (None, True))


class RefineDropTest(unittest.TestCase):
    def test_all_refine_drops_report_reason(self):
        atlas = _make_atlas(BRANCH_A, BRANCH_A_NEAR)
        self._orig = seed_chain._refine_seeds
        seed_chain._refine_seeds = (
            lambda urdf, path, xyz, quat, seeds: ([], len(seeds)))
        try:
            report = seed_chain.chain_report(
                atlas, None, None, SLOT, PORTAL, 0.0, IDENTITY)
        finally:
            seed_chain._refine_seeds = self._orig
        self.assertIsNone(report["same_branch"])
        self.assertEqual(report["reason"], "refine_dropped_all")
        self.assertEqual(report["refine_dropped"], 2)


if __name__ == "__main__":
    unittest.main()
