#!/usr/bin/env python3
"""Unit tests for the shared placement scorer and the atlas adapter.

The behaviour under test is the one that made E16R stack from the first box:
with the old three-term proxy an unobserved floor column scored 1.000 while a
stack on a 0.32 m box scored 1.208, so stacking won whenever the support was
below ~0.82 m.
"""

import math
import os
import sys
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from luggage_packing.ems import EMS  # noqa: E402
from luggage_packing.free_space_model import FreeSpaceModel  # noqa: E402
from luggage_packing.placement_reachability import (  # noqa: E402
    NEUTRAL_PRIOR,
    annotate_with_atlas,
    atlas_contact_point,
    atlas_query_yaw,
    payload_atlas_path,
    resolve_atlas_path,
    select_payload_atlas,
)
from luggage_packing.placement_scoring import (  # noqa: E402
    DEFAULT_W_FLOOR_FIRST,
    floor_first_term,
    score_candidates,
)

INNER = (1.49, 1.97, 2.01)
CENTER = (0.0, -1.5, 0.145)
FLOOR_Z = 0.53
USABLE = [1.49, 1.97, INNER[2] - FLOOR_Z]
SMALLEST = [0.55, 0.40, 0.25]
LARGE = [0.80, 0.50, 0.32]
CARRYON = [0.55, 0.40, 0.25]


def _model_with_placed_large():
    model = FreeSpaceModel(
        INNER, CENTER, yaw=-1.5708, resolution=0.05,
        floor_z=FLOOR_Z, boundary_margin=0.05)
    # Large box on the floor: center_local z is volume-centred.
    placed_center = [-0.30, -0.30, -INNER[2] * 0.5 + FLOOR_Z + LARGE[2] * 0.5]
    box = model.add_placed_box(placed_center, LARGE)
    ems = EMS(USABLE, min_useful_edge=min(SMALLEST))
    ems.place((box["x0"], box["y0"], box["z0"],
               box["x1"], box["y1"], box["z1"]))
    return model, ems


class _StubResult(object):
    def __init__(self, status, hard_reject_safe=False):
        self.status = status
        self.hard_reject_safe = hard_reject_safe


class _StubAtlas(object):
    """Returns a scripted status per queried Z band."""

    def __init__(self, statuses, hard_reject_z=None):
        self.statuses = statuses
        self.hard_reject_z = hard_reject_z
        self.queries = []

    def query(self, x, y, z, yaw=0.0):
        self.queries.append((x, y, z, yaw))
        if self.hard_reject_z is not None and abs(z - self.hard_reject_z) < 1e-6:
            return _StubResult(1, hard_reject_safe=True)
        return _StubResult(self.statuses.get(round(z, 3), 3))


class TestFloorFirstTerm(unittest.TestCase):
    def test_floor_scores_one_ceiling_scores_zero(self):
        self.assertAlmostEqual(
            floor_first_term({"peak": 0.0}, USABLE[2]), 1.0)
        self.assertAlmostEqual(
            floor_first_term({"peak": USABLE[2]}, USABLE[2]), 0.0)

    def test_term_is_monotonic_in_support_height(self):
        low = floor_first_term({"peak": 0.28}, USABLE[2])
        high = floor_first_term({"peak": 0.90}, USABLE[2])
        self.assertGreater(low, high)


class TestScoreCandidates(unittest.TestCase):
    def test_floor_outranks_a_low_stack(self):
        """The regression that forced E16R to stack from box two."""
        model, ems = _model_with_placed_large()
        candidates = model.candidates(
            CARRYON, allowed_yaws=[0.0, math.pi / 2.0], top_n=200)
        for cand in candidates:
            cand["reachability_prior"] = 1.0
        score_candidates(
            candidates, model, ems, USABLE, SMALLEST,
            opening_side="negative_x")
        best = candidates[0]
        self.assertLessEqual(
            float(best["peak"]), 1e-3,
            "scorer still prefers stacking over an open floor")

    def test_stack_wins_once_the_floor_is_full(self):
        model = FreeSpaceModel(
            INNER, CENTER, yaw=-1.5708, resolution=0.05,
            floor_z=FLOOR_Z, boundary_margin=0.05)
        ems = EMS(USABLE, min_useful_edge=min(SMALLEST))
        # Tile the floor with tops wide enough to fully support a carryon, so
        # the only remaining gaps are too thin for another floor placement.
        for ix in (-0.325, 0.325):
            for iy in (-0.62, 0.0, 0.62):
                box = model.add_placed_box(
                    [ix, iy, -INNER[2] * 0.5 + FLOOR_Z + 0.14],
                    [0.65, 0.62, 0.28])
                ems.place((box["x0"], box["y0"], box["z0"],
                           box["x1"], box["y1"], box["z1"]))
        candidates = model.candidates(
            CARRYON, allowed_yaws=[0.0], top_n=200)
        self.assertTrue(candidates, "no stack candidate on a full floor")
        score_candidates(
            candidates, model, ems, USABLE, SMALLEST,
            opening_side="negative_x")
        self.assertGreater(float(candidates[0]["peak"]), 0.0)

    def test_ranking_is_deterministic_under_ties(self):
        model, ems = _model_with_placed_large()
        first = model.candidates(CARRYON, allowed_yaws=[0.0], top_n=60)
        second = model.candidates(CARRYON, allowed_yaws=[0.0], top_n=60)
        for batch in (first, second):
            score_candidates(batch, model, ems, USABLE, SMALLEST)
        self.assertEqual(
            [c["center_local"][:2] for c in first],
            [c["center_local"][:2] for c in second])

    def test_reachability_prior_shifts_ranking(self):
        model, ems = _model_with_placed_large()
        base = model.candidates(CARRYON, allowed_yaws=[0.0], top_n=40)
        score_candidates(base, model, ems, USABLE, SMALLEST)
        top = base[0]
        penalised = [dict(top)]
        penalised[0]["reachability_prior"] = -1.0
        score_candidates(penalised, model, ems, USABLE, SMALLEST)
        self.assertLess(penalised[0]["score"], top["score"])

    def test_floor_first_weight_zero_restores_stack_bias(self):
        """Shows w_floor_first is what flips the decision, not luck."""
        model, ems = _model_with_placed_large()
        without = model.candidates(CARRYON, allowed_yaws=[0.0], top_n=200)
        score_candidates(
            without, model, ems, USABLE, SMALLEST, w_floor_first=0.0)
        with_term = model.candidates(CARRYON, allowed_yaws=[0.0], top_n=200)
        score_candidates(
            with_term, model, ems, USABLE, SMALLEST,
            w_floor_first=DEFAULT_W_FLOOR_FIRST)
        self.assertLessEqual(float(with_term[0]["peak"]), 1e-3)


class TestAtlasAdapter(unittest.TestCase):
    def test_contact_point_is_box_top_in_container_link(self):
        cand = {
            "center_local": [-0.2, 0.3, 0.0],
            "size": [0.7, 0.45, 0.28],
            "peak": 0.32,
        }
        self.assertEqual(
            atlas_contact_point(cand, FLOOR_Z),
            (-0.2, 0.3, FLOOR_Z + 0.32 + 0.28))

    def test_pi_equivalent_yaw_maps_onto_the_same_bin(self):
        self.assertAlmostEqual(atlas_query_yaw({"box_yaw": math.pi}), 0.0)
        self.assertAlmostEqual(
            atlas_query_yaw({"box_yaw": -math.pi / 2.0}), math.pi / 2.0)

    def test_missing_atlas_is_neutral_and_rejects_nothing(self):
        cands = [{"center_local": [0, 0, 0], "size": LARGE, "peak": 0.0}]
        kept, rejected = annotate_with_atlas(cands, None, FLOOR_Z)
        self.assertEqual(len(kept), 1)
        self.assertEqual(rejected, 0)
        self.assertEqual(kept[0]["reachability_prior"], NEUTRAL_PRIOR)

    def test_only_high_confidence_unreachable_is_dropped(self):
        reject_z = FLOOR_Z + 0.0 + LARGE[2]
        atlas = _StubAtlas({}, hard_reject_z=reject_z)
        cands = [
            {"center_local": [0.0, 0.0, 0.0], "size": LARGE, "peak": 0.0},
            {"center_local": [0.1, 0.1, 0.0], "size": LARGE, "peak": 0.5},
        ]
        kept, rejected = annotate_with_atlas(cands, atlas, FLOOR_Z)
        self.assertEqual(rejected, 1)
        self.assertEqual(len(kept), 1)
        self.assertAlmostEqual(kept[0]["peak"], 0.5)

    def test_unknown_cells_survive_for_the_moveit_filter(self):
        atlas = _StubAtlas({round(FLOOR_Z + LARGE[2], 3): 0})
        cands = [{"center_local": [0.0, 0.0, 0.0], "size": LARGE, "peak": 0.0}]
        kept, rejected = annotate_with_atlas(cands, atlas, FLOOR_Z)
        self.assertEqual(rejected, 0)
        self.assertEqual(kept[0]["reachability_prior"], 0.0)


class TestAtlasPathResolution(unittest.TestCase):
    def test_payload_path_uses_two_decimal_sizes(self):
        self.assertTrue(payload_atlas_path("/d", "base", LARGE).endswith(
            "base_payload_0.80x0.50x0.32.npz"))

    def test_resolve_returns_none_when_nothing_on_disk(self):
        self.assertIsNone(
            resolve_atlas_path("/nonexistent-atlas-dir", "base", LARGE))

    def test_resolve_prefers_payload_then_empty_load(self):
        atlas_dir = self._atlas_dir()
        resolved = resolve_atlas_path(
            atlas_dir, "s20_container_collision_aware", LARGE)
        self.assertIsNotNone(resolved)
        self.assertIn("payload_0.80x0.50x0.32", resolved[0])

    def _atlas_dir(self):
        atlas_dir = os.path.join(
            os.path.dirname(PKG_ROOT), "luggage_planning",
            "data", "reachability_atlas")
        if not os.path.isdir(atlas_dir):
            self.skipTest("atlas data not present in this checkout")
        return atlas_dir

    def test_continuous_size_selects_an_enveloping_atlas(self):
        """Continuous sizes never match a filename; picking a smaller payload
        atlas would make the prior optimistic exactly when it must not be."""
        atlas_dir = self._atlas_dir()
        box = [0.72, 0.46, 0.29]   # between standard and large
        selected = select_payload_atlas(
            atlas_dir, "s20_container_collision_aware", box)
        self.assertIsNotNone(selected)
        self.assertIn("payload_0.80x0.50x0.32", selected)

    def test_smallest_enveloping_atlas_is_preferred(self):
        atlas_dir = self._atlas_dir()
        box = [0.50, 0.38, 0.24]   # smaller than every reference size
        selected = select_payload_atlas(
            atlas_dir, "s20_container_collision_aware", box)
        self.assertIn("payload_0.55x0.40x0.25", selected)

    def test_oversized_box_falls_back_to_largest_atlas(self):
        atlas_dir = self._atlas_dir()
        box = [0.95, 0.60, 0.40]   # beyond every built payload
        selected = select_payload_atlas(
            atlas_dir, "s20_container_collision_aware", box)
        self.assertIn("payload_0.80x0.50x0.32", selected)

    def test_continuous_size_never_falls_back_to_empty_load(self):
        atlas_dir = self._atlas_dir()
        resolved = resolve_atlas_path(
            atlas_dir, "s20_container_collision_aware", [0.72, 0.46, 0.29])
        self.assertIn("payload_", resolved[0])


if __name__ == "__main__":
    unittest.main()
