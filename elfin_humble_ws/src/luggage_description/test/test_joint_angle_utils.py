#!/usr/bin/env python3
"""Unit tests for joint_angle_utils (no roscore required)."""

import math
import os
import sys
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from luggage_description.joint_angle_utils import (  # noqa: E402
    WRAP_EQUIVALENT_JOINTS,
    closest_angle_equivalent,
    format_rewrites,
    max_joint_error,
    normalize_joint_map,
    normalize_joint_targets,
)

TWO_PI = 2.0 * math.pi
TOL = 1e-6


class TestClosestAngleEquivalent(unittest.TestCase):
    def test_no_change_when_already_close(self):
        # Current and target are on the same branch → no change.
        result = closest_angle_equivalent(1.0, 1.0)
        self.assertAlmostEqual(result, 1.0, delta=TOL)

    def test_snap_target_to_nearest_positive_branch(self):
        # Current=1.6, target=-4.68 (same pose, different branch).
        # -4.68 + 2π ≈ 1.603 → closest to 1.6.
        result = closest_angle_equivalent(1.6234, -4.6598)
        self.assertAlmostEqual(result, -4.6598 + TWO_PI, delta=TOL)

    def test_snap_target_to_nearest_negative_branch(self):
        # Current=-4.6, target=1.6 (same pose, different branch).
        # 1.6 - 2π ≈ -4.683 → closest to -4.6.
        result = closest_angle_equivalent(-4.6598, 1.6234)
        self.assertAlmostEqual(result, 1.6234 - TWO_PI, delta=TOL)

    def test_non_wrap_joint_unchanged(self):
        # elfin_joint2 is NOT in WRAP_EQUIVALENT_JOINTS.
        result = closest_angle_equivalent(-1.3, -1.0)
        self.assertAlmostEqual(result, -1.0, delta=TOL)

    def test_out_of_limit_returns_original(self):
        # Equivalent that would land past the limit → returns original.
        result = closest_angle_equivalent(6.0, 0.0, lower=-6.28, upper=6.28)
        # 0.0 + 2π ≈ 6.283, but that's > upper=6.28 → returns original 0.0.
        self.assertAlmostEqual(result, 0.0, delta=TOL)


class TestNormalizeJointTargets(unittest.TestCase):
    def test_normalize_wraps_only_wrap_joints(self):
        joints = [
            "elfin_joint1",
            "elfin_joint2",
            "elfin_joint3",
            "elfin_joint4",
            "elfin_joint5",
            "elfin_joint6",
        ]
        current = [1.6, -1.3, -1.1, 3.9, 1.6234, 0.45]
        target = [-2.713, -1.3263, -1.0965, 3.9564, -4.6598, 0.4522]
        adjusted, rewrites = normalize_joint_targets(
            joints, current, target, wrap_joints=WRAP_EQUIVALENT_JOINTS
        )
        # J2, J3, J6 are not wrap joints → unchanged.
        self.assertAlmostEqual(adjusted[1], target[1], delta=TOL)
        self.assertAlmostEqual(adjusted[2], target[2], delta=TOL)
        self.assertAlmostEqual(adjusted[5], target[5], delta=TOL)
        # J5 should snap from -4.6598 to ~1.6234 (closest to 1.6234).
        self.assertAlmostEqual(adjusted[4], -4.6598 + TWO_PI, delta=TOL)
        self.assertGreaterEqual(len(rewrites), 1)
        joint_names_rewritten = {name for name, _, _, _ in rewrites}
        self.assertIn("elfin_joint5", joint_names_rewritten)

    def test_no_rewrites_when_already_same_branch(self):
        joints = ["elfin_joint1", "elfin_joint5"]
        current = [3.5702, 1.6234]
        target = [3.5702, 1.6234]
        adjusted, rewrites = normalize_joint_targets(
            joints, current, target, wrap_joints=WRAP_EQUIVALENT_JOINTS
        )
        self.assertEqual(adjusted, list(target))
        self.assertEqual(len(rewrites), 0)

    def test_length_mismatch_raises(self):
        with self.assertRaises(ValueError):
            normalize_joint_targets(["a"], [0.0], [0.0, 1.0])


class TestNormalizeJointMap(unittest.TestCase):
    def test_normalize_j5_only(self):
        current = {"elfin_joint5": 1.6234, "elfin_joint2": -1.3}
        target = {"elfin_joint5": -4.6598, "elfin_joint2": -1.3263}
        adjusted, rewrites = normalize_joint_map(current, target, wrap_joints=WRAP_EQUIVALENT_JOINTS)
        # J2 unchanged.
        self.assertAlmostEqual(adjusted["elfin_joint2"], -1.3263, delta=TOL)
        # J5 snapped.
        self.assertAlmostEqual(adjusted["elfin_joint5"], -4.6598 + TWO_PI, delta=TOL)
        self.assertEqual(len(rewrites), 1)
        self.assertEqual(rewrites[0][0], "elfin_joint5")

    def test_missing_from_current_left_untouched(self):
        target = {"elfin_joint5": -4.6598}
        adjusted, rewrites = normalize_joint_map({}, target, wrap_joints=WRAP_EQUIVALENT_JOINTS)
        self.assertAlmostEqual(adjusted["elfin_joint5"], -4.6598, delta=TOL)
        self.assertEqual(len(rewrites), 0)


class TestWrapEquivalentJoints(unittest.TestCase):
    def test_known_wrap_joints(self):
        self.assertIn("elfin_joint1", WRAP_EQUIVALENT_JOINTS)
        self.assertIn("elfin_joint4", WRAP_EQUIVALENT_JOINTS)
        self.assertIn("elfin_joint5", WRAP_EQUIVALENT_JOINTS)
        self.assertIn("elfin_joint6", WRAP_EQUIVALENT_JOINTS)

    def test_non_wrap_joints_excluded(self):
        self.assertNotIn("elfin_joint2", WRAP_EQUIVALENT_JOINTS)
        self.assertNotIn("elfin_joint3", WRAP_EQUIVALENT_JOINTS)


class TestFormatRewrites(unittest.TestCase):
    def test_single_entry(self):
        s = format_rewrites([("j5", -4.6598, 1.6234, 1.6234)])
        self.assertIn("j5", s)
        self.assertIn("raw=-4.6598", s)
        self.assertIn("adjusted=1.6234", s)

    def test_empty(self):
        self.assertEqual(format_rewrites([]), "")


class TestMaxJointError(unittest.TestCase):
    def test_wrap_joint_j5_branch_only(self):
        joints = ["elfin_joint5"]
        # 7.854 vs 1.6234: raw error ~6.23, wrap-aware ~0.05.
        err = max_joint_error(joints, [7.854], [1.6234], wrap_joints=WRAP_EQUIVALENT_JOINTS)
        self.assertLess(err, 0.1)
        self.assertGreater(abs(7.854 - 1.6234), 6.0)

    def test_real_collapse_j4(self):
        joints = ["elfin_joint4"]
        err = max_joint_error(joints, [0.0], [3.9564], wrap_joints=WRAP_EQUIVALENT_JOINTS)
        self.assertGreater(err, 2.0)

    def test_mixed_observe_vs_collapsed(self):
        joints = [
            "elfin_joint1",
            "elfin_joint2",
            "elfin_joint3",
            "elfin_joint4",
            "elfin_joint5",
            "elfin_joint6",
        ]
        collapsed = [6.283, -1.571, -3.142, 0.0, 7.854, 1.571]
        observe = [3.5702, -1.3263, -1.0965, 3.9564, 1.6234, 0.4522]
        err = max_joint_error(
            joints, collapsed, observe, wrap_joints=WRAP_EQUIVALENT_JOINTS
        )
        # J4 collapse dominates; J5 wrap should not inflate to 6.23.
        self.assertGreater(err, 2.5)
        self.assertLess(err, 4.5)


if __name__ == "__main__":
    unittest.main()