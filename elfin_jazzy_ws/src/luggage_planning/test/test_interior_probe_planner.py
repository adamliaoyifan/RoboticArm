#!/usr/bin/env python3
"""Closed-loop selection tests for interior probes (no ROS required)."""

import os
import sys
import unittest


from luggage_planning.interior_probe_planner import (  # noqa: E402
    evaluate_probe_termination,
    rank_probe_candidates,
)


def _config():
    return {
        "min_improvement": 0.01,
        "stagnation_limit": 2,
        "termination": {"unknown_threshold": 0.15, "max_views": 4},
    }


def _view(name, x, depth, lateral=0.0, valid=True, reason=""):
    return {
        "name": name,
        "camera_xyz": [x, 0.0, 1.0],
        "look_at": [x, 0.0, 0.0],
        "depth": depth,
        "lateral_offset": lateral,
        "valid_geometry": valid,
        "reject_reason": reason,
    }


class TestInteriorProbePlanner(unittest.TestCase):
    def test_frontier_coverage_ranks_matching_probe_first(self):
        views = [_view("left", -0.4, 0.2), _view("right", 0.4, 0.2)]
        ranked, _states = rank_probe_candidates(
            views,
            [[0.4, 0.0, 0.05], [0.42, 0.0, 0.10]],
            set(),
            coverage_radius=0.35,
        )
        self.assertEqual(ranked[0][2]["name"], "right")
        self.assertGreater(ranked[0][0], ranked[1][0])

    def test_no_frontiers_prefers_center_then_near_depth(self):
        views = [
            _view("far_center", 0.0, 0.8),
            _view("near_edge", 0.3, 0.2, lateral=0.3),
            _view("near_center", 0.0, 0.2),
        ]
        ranked, _states = rank_probe_candidates(views, [], set(), 0.5)
        self.assertEqual(
            [entry[2]["name"] for entry in ranked],
            ["near_center", "near_edge", "far_center"],
        )

    def test_invalid_and_used_candidates_are_excluded_with_diagnostics(self):
        views = [
            _view("used", 0.0, 0.2),
            _view("blocked", 0.2, 0.2, valid=False, reason="aperture_blocked"),
            _view("valid", -0.2, 0.2),
        ]
        ranked, states = rank_probe_candidates(views, [], {0}, 0.5)
        self.assertEqual([entry[1] for entry in ranked], [2])
        self.assertEqual(states[0], "used")
        self.assertEqual(states[1], "aperture_blocked")

    def test_unknown_threshold_stops_after_a_completed_view(self):
        result = evaluate_probe_termination(0.14, 1, _config(), 0.30, 0)
        self.assertTrue(result["done"])
        self.assertEqual(result["reason"], "unknown_threshold")

    def test_low_improvement_stops_after_configured_stagnation(self):
        first = evaluate_probe_termination(0.495, 1, _config(), 0.50, 0)
        self.assertFalse(first["done"])
        self.assertEqual(first["stagnant_count"], 1)
        second = evaluate_probe_termination(
            0.491, 2, _config(), first["last_unknown"], first["stagnant_count"]
        )
        self.assertTrue(second["done"])
        self.assertEqual(second["reason"], "low_improvement")

    def test_max_views_is_hard_stop(self):
        result = evaluate_probe_termination(0.9, 4, _config(), 0.9, 0)
        self.assertTrue(result["done"])
        self.assertEqual(result["reason"], "max_views")


if __name__ == "__main__":
    unittest.main()
