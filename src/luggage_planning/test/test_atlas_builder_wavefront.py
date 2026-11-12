#!/usr/bin/env python3
"""No-ROS tests for the atlas builder's deterministic core."""

import unittest

import luggage_planning.atlas.wavefront as builder
from luggage_planning.atlas.wavefront import (
    MIN_LIFT,
    adaptive_lift_clearance,
    contact_below_interference,
    container_tool_z_in_frame,
    retreat_height,
)
from luggage_planning.waypoint_generator import INSERT_CLEARANCE_MIN


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


class TestAdaptiveLift(unittest.TestCase):
    def _center(self, iz):
        return 0.50 + (iz + 0.5) * 0.15

    def test_min_lift_matches_insertion_floor(self):
        self.assertAlmostEqual(MIN_LIFT, INSERT_CLEARANCE_MIN)

    def test_low_layers_keep_the_nominal_lift(self):
        self.assertAlmostEqual(adaptive_lift_clearance(self._center(4)), 0.30)
        self.assertAlmostEqual(adaptive_lift_clearance(0.575), 0.30)

    def test_mid_layers_shrink_to_the_safe_tool_height(self):
        # iz5 center 1.325 -> carry capped at 1.55.
        self.assertAlmostEqual(adaptive_lift_clearance(self._center(5)), 0.225)
        # iz6 center 1.475 -> 0.075, still a real lift.
        self.assertAlmostEqual(adaptive_lift_clearance(self._center(6)), 0.075)
        self.assertAlmostEqual(adaptive_lift_clearance(1.49), 0.06)

    def test_high_contacts_have_no_same_column_lift(self):
        self.assertIsNone(adaptive_lift_clearance(self._center(7)))
        self.assertIsNone(adaptive_lift_clearance(1.55))
        self.assertIsNone(adaptive_lift_clearance(1.70))

    def test_contacts_at_the_elbow_band_are_not_sampled(self):
        self.assertTrue(contact_below_interference(self._center(7)))
        self.assertFalse(contact_below_interference(self._center(8)))
        self.assertFalse(contact_below_interference(1.70))

    def test_container_z_is_shifted_into_the_retreat_frame(self):
        # elfin_base_link: container origin z is -0.86, yaw only.
        self.assertAlmostEqual(
            container_tool_z_in_frame(1.55, -0.86, (0.0, 0.0, -1.5708)),
            0.69)
        # world: container_link origin z is 0.
        self.assertAlmostEqual(
            container_tool_z_in_frame(1.55, 0.0, (0.0, 0.0, 0.0)),
            1.55)

    def test_pitched_container_is_refused(self):
        with self.assertRaises(ValueError):
            container_tool_z_in_frame(1.55, 0.0, (0.1, 0.0, 0.0))

    def test_retreat_height_caps_without_descending(self):
        height, cut = retreat_height(1.40, 0.30, 1.55)
        self.assertAlmostEqual(height, 1.55)
        self.assertAlmostEqual(cut, 0.15)
        height, cut = retreat_height(0.45, 0.15, 1.55)
        self.assertAlmostEqual(height, 0.60)
        self.assertAlmostEqual(cut, 0.0)
        height, cut = retreat_height(1.80, 0.15, 1.55)
        self.assertAlmostEqual(height, 1.80)
        self.assertAlmostEqual(cut, 0.15)

    def test_high_opening_anchor_stays_out_while_interior_climbs(self):
        """A failed high door column must not keep the layer UNKNOWN.

        Vertical lift is legal only at iz 0. Higher opening anchors are
        refused. The interior cell above a connected lower cell is admitted
        by the internal-lift rule (a connected predecessor, not an anchor).
        """
        def z_of(iz):
            return 1.40 + iz * 0.20

        def can_expand(cell, predecessors, is_anchor):
            clearance = adaptive_lift_clearance(
                z_of(cell[2]), 0.30, 1.55, 0.06)
            if clearance is not None:
                return True
            if is_anchor:
                return False
            return bool(predecessors)

        order = builder.deterministic_wavefront((2, 1, 3), 0, -1, can_expand)
        self.assertIn((0, 0, 0), order)
        self.assertNotIn((0, 0, 1), order)
        self.assertNotIn((0, 0, 2), order)
        self.assertIn((1, 0, 0), order)
        self.assertIn((1, 0, 1), order)


if __name__ == "__main__":
    unittest.main()
