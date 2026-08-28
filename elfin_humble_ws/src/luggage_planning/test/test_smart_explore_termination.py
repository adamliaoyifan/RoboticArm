#!/usr/bin/env python3
"""Unit tests for the phase0 marginal-gain early-stop helper (no ROS)."""

import os
import sys
import unittest


from luggage_planning.smart_explore_termination import (  # noqa: E402
    phase0_gain_exhausted,
    phase0_low_fov,
)


class TestPhase0GainExhausted(unittest.TestCase):
    def test_no_prior_view_never_exhausted(self):
        exhausted, stagnant, reason = phase0_gain_exhausted(
            last_unknown=None, unknown_ratio=0.90, phase0_used=0,
            stagnant_count=0, min_improvement=0.01, stagnation_limit=1,
        )
        self.assertFalse(exhausted)
        self.assertEqual(stagnant, 0)
        self.assertEqual(reason, "")

    def test_phase0_used_guard_blocks_exhaustion_even_with_last_unknown(self):
        # Defensive: phase0_used < 1 must never report exhausted, regardless
        # of last_unknown being populated by a caller bug.
        exhausted, stagnant, reason = phase0_gain_exhausted(
            last_unknown=0.90, unknown_ratio=0.90, phase0_used=0,
            stagnant_count=0, min_improvement=0.01, stagnation_limit=1,
        )
        self.assertFalse(exhausted)
        self.assertEqual(stagnant, 0)
        self.assertEqual(reason, "")

    def test_good_improvement_resets_stagnation_and_is_not_exhausted(self):
        exhausted, stagnant, reason = phase0_gain_exhausted(
            last_unknown=0.90, unknown_ratio=0.70, phase0_used=1,
            stagnant_count=2, min_improvement=0.01, stagnation_limit=1,
        )
        self.assertFalse(exhausted)
        self.assertEqual(stagnant, 0)
        self.assertEqual(reason, "")

    def test_low_improvement_exhausts_at_stagnation_limit_one(self):
        exhausted, stagnant, reason = phase0_gain_exhausted(
            last_unknown=0.97, unknown_ratio=0.94, phase0_used=1,
            stagnant_count=0, min_improvement=0.01, stagnation_limit=1,
        )
        # 0.97 - 0.94 = 0.03 >= 0.01, so this alone should NOT stagnate.
        self.assertFalse(exhausted)
        self.assertEqual(stagnant, 0)

        exhausted2, stagnant2, reason2 = phase0_gain_exhausted(
            last_unknown=0.94, unknown_ratio=0.938, phase0_used=2,
            stagnant_count=0, min_improvement=0.01, stagnation_limit=1,
        )
        self.assertTrue(exhausted2)
        self.assertEqual(stagnant2, 1)
        self.assertEqual(reason2, "low_improvement")

    def test_low_improvement_accumulates_until_stagnation_limit(self):
        exhausted, stagnant, reason = phase0_gain_exhausted(
            last_unknown=0.94, unknown_ratio=0.938, phase0_used=2,
            stagnant_count=0, min_improvement=0.01, stagnation_limit=2,
        )
        self.assertFalse(exhausted)
        self.assertEqual(stagnant, 1)
        self.assertEqual(reason, "")

        exhausted2, stagnant2, reason2 = phase0_gain_exhausted(
            last_unknown=0.938, unknown_ratio=0.937, phase0_used=3,
            stagnant_count=stagnant, min_improvement=0.01, stagnation_limit=2,
        )
        self.assertTrue(exhausted2)
        self.assertEqual(stagnant2, 2)
        self.assertEqual(reason2, "low_improvement")

    def test_exact_min_improvement_counts_as_gain_not_stagnation(self):
        exhausted, stagnant, _reason = phase0_gain_exhausted(
            last_unknown=0.90, unknown_ratio=0.89, phase0_used=1,
            stagnant_count=0, min_improvement=0.01, stagnation_limit=1,
        )
        # improvement == min_improvement is not < min_improvement.
        self.assertFalse(exhausted)
        self.assertEqual(stagnant, 0)

    def test_negative_improvement_counts_as_stagnation(self):
        exhausted, stagnant, reason = phase0_gain_exhausted(
            last_unknown=0.80, unknown_ratio=0.82, phase0_used=1,
            stagnant_count=0, min_improvement=0.01, stagnation_limit=1,
        )
        self.assertTrue(exhausted)
        self.assertEqual(stagnant, 1)
        self.assertEqual(reason, "low_improvement")


class TestPhase0LowFov(unittest.TestCase):
    def test_first_view_never_skipped_even_with_zero_fov(self):
        # The mandatory first opening view always runs, regardless of its own
        # FOV -- it also seeds container_opening_estimator and a safe entry
        # pose.
        self.assertFalse(
            phase0_low_fov(
                phase0_used=0, inside_container_fov_ratio=0.0,
                min_inside_fov=0.5))

    def test_none_ratio_fails_open(self):
        # Metric unavailable (opening geometry not yet known, or the
        # evaluate service is unreachable/erroring) must never be treated as
        # "low FOV" -- a metric outage cannot silently drop phase0 views.
        self.assertFalse(
            phase0_low_fov(
                phase0_used=2, inside_container_fov_ratio=None,
                min_inside_fov=0.5))

    def test_ratio_below_threshold_skips(self):
        self.assertTrue(
            phase0_low_fov(
                phase0_used=1, inside_container_fov_ratio=0.1,
                min_inside_fov=0.5))

    def test_ratio_at_or_above_threshold_does_not_skip(self):
        self.assertFalse(
            phase0_low_fov(
                phase0_used=1, inside_container_fov_ratio=0.5,
                min_inside_fov=0.5))
        self.assertFalse(
            phase0_low_fov(
                phase0_used=1, inside_container_fov_ratio=0.9,
                min_inside_fov=0.5))

    def test_zero_threshold_never_skips(self):
        # The shipped default (0.0, pending real-data calibration): any
        # ratio is >= 0.0, so the gate is a no-op until tuned.
        self.assertFalse(
            phase0_low_fov(
                phase0_used=3, inside_container_fov_ratio=0.0,
                min_inside_fov=0.0))


if __name__ == "__main__":
    unittest.main()
