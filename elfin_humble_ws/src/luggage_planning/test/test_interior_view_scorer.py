#!/usr/bin/env python3
"""Focused tests for deterministic interior candidate scoring."""

from __future__ import division

import json
import os
import sys
import unittest


TEST_ROOT = os.path.dirname(os.path.abspath(__file__))

from luggage_planning.interior_view_scorer import (
    FREE,
    UNKNOWN,
    CameraIntrinsics,
    RaycastConfig,
    SparseOccupancyGrid,
    rank_candidates,
    raycast_information_gain,
    score_candidate,
    stable_diagnostics,
)
from harness.interior_scoring_fixtures import (  # noqa: E402
    candidate,
    corridor_grid,
    narrow_intrinsics,
)


class TestFrustumRaycasting(unittest.TestCase):
    def test_unknown_voxels_are_counted_once_across_frustum_rays(self):
        result = raycast_information_gain(
            candidate("view"),
            corridor_grid(),
            narrow_intrinsics(),
            RaycastConfig(max_range=0.9),
        )
        self.assertEqual(result["rays_cast"], 3)
        self.assertGreater(result["visible_unknown_voxels"], 0)
        self.assertEqual(
            result["information_gain"],
            float(result["visible_unknown_voxels"]),
        )

    def test_occupied_voxel_stops_visibility_behind_it(self):
        clear = raycast_information_gain(
            candidate("clear"),
            corridor_grid(barrier=False),
            narrow_intrinsics(),
            RaycastConfig(max_range=0.9),
        )
        blocked = raycast_information_gain(
            candidate("blocked"),
            corridor_grid(barrier=True),
            narrow_intrinsics(),
            RaycastConfig(max_range=0.9),
        )
        self.assertGreater(blocked["occluded_rays"], 0)
        self.assertLess(
            blocked["visible_unknown_voxels"],
            clear["visible_unknown_voxels"],
        )

    def test_range_decay_discounts_far_unknown_voxels(self):
        intrinsics = CameraIntrinsics(1, 1, 2.0, 2.0)
        near = SparseOccupancyGrid(
            origin=(0.0, -0.05, -0.05),
            shape=(10, 1, 1),
            resolution=0.1,
            cells={(2, 0, 0): UNKNOWN},
            default_state=FREE,
        )
        far = SparseOccupancyGrid(
            origin=(0.0, -0.05, -0.05),
            shape=(10, 1, 1),
            resolution=0.1,
            cells={(8, 0, 0): UNKNOWN},
            default_state=FREE,
        )
        config = RaycastConfig(max_range=1.0, range_decay=0.25)
        near_gain = raycast_information_gain(
            candidate("near"), near, intrinsics, config)["information_gain"]
        far_gain = raycast_information_gain(
            candidate("far"), far, intrinsics, config)["information_gain"]
        self.assertGreater(near_gain, far_gain)
        self.assertGreater(far_gain, 0.0)

    def test_known_normal_discounts_grazing_incidence(self):
        intrinsics = CameraIntrinsics(1, 1, 2.0, 2.0)
        normal_grid = SparseOccupancyGrid(
            origin=(0.0, -0.05, -0.05),
            shape=(6, 1, 1),
            resolution=0.1,
            cells={(2, 0, 0): UNKNOWN},
            normals={(2, 0, 0): (0.0, 1.0, 0.0)},
            default_state=FREE,
        )
        result = raycast_information_gain(
            candidate("grazing"),
            normal_grid,
            intrinsics,
            RaycastConfig(max_range=0.5, grazing_power=1.0),
        )
        self.assertEqual(result["visible_unknown_voxels"], 1)
        self.assertEqual(result["normal_discounted_voxels"], 1)
        self.assertAlmostEqual(result["information_gain"], 0.0)

    def test_minimal_duck_typed_accessor_uses_fallback_traversal(self):
        backing = corridor_grid()

        class MinimalAccessor(object):
            resolution = backing.resolution

            def world_to_index(self, point):
                return backing.world_to_index(point)

            def occupancy(self, index):
                return backing.occupancy(index)

        result = raycast_information_gain(
            candidate("minimal"),
            MinimalAccessor(),
            CameraIntrinsics(1, 1, 2.0, 2.0),
            RaycastConfig(max_range=0.5),
        )
        self.assertGreater(result["information_gain"], 0.0)


class TestCandidateScoring(unittest.TestCase):
    def setUp(self):
        self.grid = corridor_grid()
        self.intrinsics = narrow_intrinsics()
        self.config = RaycastConfig(max_range=0.9)

    def test_hard_rejection_skips_raycast_and_depth_reward(self):
        class FailingAccessor(object):
            def world_to_index(self, _point):
                raise AssertionError("raycast must not run")

            def occupancy(self, _index):
                raise AssertionError("raycast must not run")

        result = score_candidate(
            candidate("bad", collision_free=False, depth=1000.0),
            FailingAccessor(),
            self.intrinsics,
        )
        self.assertFalse(result["feasible"])
        self.assertIsNone(result["score"])
        self.assertEqual(result["components"]["depth_reward"], 0.0)
        self.assertIsNone(result["diagnostics"]["raycast"])
        self.assertEqual(
            result["diagnostics"]["reject_reasons"], ["collision_free"])

    def test_corridor_confidence_scales_effective_information(self):
        trusted = score_candidate(
            candidate("trusted", corridor_confidence=1.0),
            self.grid, self.intrinsics, self.config)
        uncertain = score_candidate(
            candidate("uncertain", corridor_confidence=0.25),
            self.grid, self.intrinsics, self.config)
        self.assertAlmostEqual(
            uncertain["components"]["effective_information_gain"],
            trusted["components"]["effective_information_gain"] * 0.25,
        )
        self.assertGreater(trusted["score"], uncertain["score"])

    def test_all_motion_and_risk_components_affect_score(self):
        weights = {
            "information_gain": 0.0,
            "corridor_confidence": 0.0,
            "depth": 1.0,
            "manipulability": 1.0,
            "joint_margin": 1.0,
            "trajectory": 1.0,
            "risk": 1.0,
        }
        result = score_candidate(
            candidate(
                "components", depth=0.4, manipulability=0.5,
                joint_margin=0.6, trajectory_quality=0.7, risk=0.2),
            self.grid, self.intrinsics, self.config, weights,
        )
        self.assertAlmostEqual(result["score"], 2.0)

    def test_trajectory_cost_is_converted_to_bounded_reward(self):
        result = score_candidate(
            candidate("cost", trajectory_quality=None, trajectory_cost=3.0),
            self.grid, self.intrinsics, self.config,
        )
        self.assertAlmostEqual(result["components"]["trajectory"], 0.25)

    def test_feasible_candidates_always_precede_rejected_candidates(self):
        ranked = rank_candidates(
            [
                candidate("unsafe_high_depth", collision_free=False, depth=999.0),
                candidate("safe", depth=0.0),
            ],
            self.grid,
            self.intrinsics,
            self.config,
        )
        self.assertEqual(
            [item["candidate_id"] for item in ranked],
            ["safe", "unsafe_high_depth"],
        )

    def test_equal_candidates_use_stable_candidate_id_tie_break(self):
        ranked = rank_candidates(
            [candidate("beta"), candidate("alpha")],
            self.grid,
            self.intrinsics,
            self.config,
        )
        self.assertEqual(
            [item["candidate_id"] for item in ranked], ["alpha", "beta"])

    def test_lexicographic_components_break_equal_weighted_scores(self):
        zero_weights = dict(
            information_gain=0.0,
            corridor_confidence=0.0,
            depth=0.0,
            manipulability=0.0,
            joint_margin=0.0,
            trajectory=0.0,
            risk=0.0,
        )
        ranked = rank_candidates(
            [
                candidate("low_corridor", corridor_confidence=0.2),
                candidate("high_corridor", corridor_confidence=0.8),
            ],
            self.grid,
            self.intrinsics,
            self.config,
            zero_weights,
        )
        self.assertEqual(ranked[0]["candidate_id"], "high_corridor")

    def test_diagnostics_are_canonical_and_repeatable(self):
        candidates = [candidate("beta"), candidate("alpha", risk=0.1)]
        first = rank_candidates(
            candidates, self.grid, self.intrinsics, self.config)
        second = rank_candidates(
            candidates, self.grid, self.intrinsics, self.config)
        first_text = stable_diagnostics(first)
        self.assertEqual(first_text, stable_diagnostics(second))
        self.assertEqual(json.loads(first_text), first)
        self.assertNotIn("NaN", first_text)
        self.assertNotIn("Infinity", first_text)


if __name__ == "__main__":
    unittest.main()
