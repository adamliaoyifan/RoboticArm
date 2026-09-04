#!/usr/bin/env python3
"""No-ROS tests for the atlas builder's deterministic core."""

import unittest

import luggage_planning.reachability_wavefront as builder


class TestWavefrontHelpers(unittest.TestCase):
    def test_opening_boundary_respects_axis_and_sign(self):
        self.assertEqual(
            builder.opening_boundary_cells((2, 3, 2), 1, 1),
            ((0, 2, 0), (0, 2, 1), (1, 2, 0), (1, 2, 1)),
        )
        self.assertEqual(
            builder.opening_boundary_cells((2, 3, 1), 0, -1),
            ((0, 0, 0), (0, 1, 0), (0, 2, 0)),
        )

    def test_wavefront_only_expands_from_connected_cells(self):
        blocked = {(0, 2, 0)}

        def can_expand(cell, predecessors, is_anchor):
            self.assertEqual(tuple(predecessors), tuple(sorted(predecessors)))
            return cell not in blocked and (is_anchor or bool(predecessors))

        first = builder.deterministic_wavefront(
            (2, 3, 1), 1, 1, can_expand)
        second = builder.deterministic_wavefront(
            (2, 3, 1), 1, 1, can_expand)
        self.assertEqual(first, second)
        self.assertNotIn((0, 2, 0), first)
        # The blocked anchor is reached from the neighboring successful lane.
        self.assertIn((0, 1, 0), first)
        self.assertIn((0, 0, 0), first)

    def test_interpolation_is_bounded_and_includes_goal(self):
        samples = builder.joint_interpolation_samples(
            (0.0, 0.0), (0.25, -0.10), 0.10)
        self.assertEqual(len(samples), 3)
        self.assertEqual(samples[-1], (0.25, -0.10))
        previous = (0.0, 0.0)
        for sample in samples:
            self.assertLessEqual(
                max(abs(a - b) for a, b in zip(previous, sample)),
                0.10)
            previous = sample

    def test_branch_selection_is_deterministic_distinct_and_bounded(self):
        branches = [
            {"transit": (0.01,), "contact": (0.01,), "margin": 0.4},
            {"transit": (1.0,), "contact": (1.0,), "margin": 0.2},
            {"transit": (0.0,), "contact": (0.0,), "margin": 0.5},
        ]
        selected = builder.select_distinct_branches(branches, 2, 0.10)
        self.assertEqual(len(selected), 2)
        self.assertEqual(selected[0]["transit"], (0.0,))
        self.assertEqual(selected[1]["transit"], (1.0,))

    def test_classification_is_conservative(self):
        healthy = [{
            "transit": (0.0,), "contact": (0.0,), "margin": 0.5,
            "repair": False,
        }]
        self.assertEqual(builder.classify_cell([], False), builder.UNREACHABLE)
        self.assertEqual(builder.classify_cell([], True), builder.UNKNOWN)
        self.assertEqual(builder.classify_cell(healthy, False), builder.REACHABLE)
        self.assertEqual(builder.classify_cell(healthy, True), builder.MARGINAL)
        repaired = [dict(healthy[0], repair=True)]
        self.assertEqual(builder.classify_cell(repaired, False), builder.MARGINAL)


if __name__ == "__main__":
    unittest.main()
