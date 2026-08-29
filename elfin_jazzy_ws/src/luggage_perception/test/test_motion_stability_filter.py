#!/usr/bin/env python3
import os
import sys
import unittest


from luggage_perception.motion_stability_filter import (  # noqa: E402
    DISPLACEMENT,
    VELOCITY,
    MotionStabilityGate,
)


class TestMotionStabilityGate(unittest.TestCase):
    def test_static_positions_settle_despite_biased_reported_velocity(self):
        gate = MotionStabilityGate(
            joint_names=["j1"],
            velocity_threshold=0.02,
            settle_time_sec=0.5,
        )
        self.assertEqual(
            gate.update(["j1"], [0.4], [0.08], stamp=1.0, now=1.0),
            "settling",
        )
        self.assertEqual(
            gate.update(["j1"], [0.4], [0.08], stamp=1.6, now=1.6),
            "stable",
        )
        diagnostics = gate.diagnostics(now=1.6)
        self.assertEqual(diagnostics["criterion"], DISPLACEMENT)
        self.assertEqual(diagnostics["peak_excursion"], 0.0)
        self.assertGreater(
            diagnostics["max_velocity"], diagnostics["velocity_threshold"])

    def test_real_motion_is_rejected_despite_low_reported_velocity(self):
        gate = MotionStabilityGate(
            joint_names=["j1"],
            velocity_threshold=0.02,
            settle_time_sec=0.5,
        )
        gate.update(["j1"], [0.00], [0.001], stamp=1.0, now=1.0)
        gate.update(["j1"], [0.02], [0.001], stamp=1.3, now=1.3)
        self.assertEqual(
            gate.update(["j1"], [0.03], [0.001], stamp=1.6, now=1.6),
            "moving",
        )
        diagnostics = gate.diagnostics(now=1.6)
        self.assertGreaterEqual(
            diagnostics["peak_excursion"],
            diagnostics["displacement_tolerance"],
        )
        self.assertLess(
            diagnostics["max_velocity"], diagnostics["velocity_threshold"])

    def test_peak_to_peak_rejects_oscillation_returning_to_start(self):
        gate = MotionStabilityGate(
            joint_names=["j1"],
            velocity_threshold=0.02,
            settle_time_sec=0.5,
        )
        gate.update(["j1"], [0.00], [0.0], stamp=1.0, now=1.0)
        gate.update(["j1"], [0.02], [0.0], stamp=1.2, now=1.2)
        gate.update(["j1"], [0.00], [0.0], stamp=1.4, now=1.4)
        self.assertEqual(
            gate.update(["j1"], [0.00], [0.0], stamp=1.6, now=1.6),
            "moving",
        )

    def test_displacement_window_must_span_settle_time(self):
        gate = MotionStabilityGate(
            joint_names=["j1"], settle_time_sec=0.5)
        self.assertEqual(
            gate.update(["j1"], [0.0], [0.0], stamp=1.0, now=1.0),
            "settling",
        )
        self.assertEqual(
            gate.update(["j1"], [0.0], [0.0], stamp=1.49, now=1.49),
            "settling",
        )
        self.assertEqual(
            gate.update(["j1"], [0.0], [0.0], stamp=1.51, now=1.51),
            "stable",
        )

    def test_velocity_comparison_mode_preserves_old_state_machine(self):
        gate = MotionStabilityGate(
            joint_names=["j1"],
            velocity_threshold=0.02,
            settle_time_sec=0.5,
            joint_state_timeout_sec=2.0,
            criterion=VELOCITY,
        )
        self.assertEqual(
            gate.update(["j1"], [0.0], [0.1], stamp=1.0, now=1.0),
            "moving",
        )
        self.assertEqual(
            gate.update(["j1"], [0.0], [0.0], stamp=1.1, now=1.1),
            "settling",
        )
        self.assertEqual(
            gate.update(["j1"], [0.0], [0.0], stamp=1.7, now=1.7),
            "stable",
        )
        self.assertFalse(gate.accepts_cloud(1.6, now=1.7))
        self.assertTrue(gate.accepts_cloud(1.7, now=1.7))

    def test_zero_stamp_cloud_accepted_when_stable(self):
        # Regression: a cloud with stamp=0 (uninitialized header) used to be
        # silently rejected (0 < stable_since). It should be treated as "now".
        gate = MotionStabilityGate(
            joint_names=["j1"], velocity_threshold=0.02, settle_time_sec=0.5
        )
        gate.update(["j1"], [0.0], [0.0], stamp=1.0, now=1.0)
        gate.update(["j1"], [0.0], [0.0], stamp=2.0, now=2.0)  # stable since 1.5
        self.assertTrue(gate.accepts_cloud(0.0, now=2.0))

    def test_position_fallback_estimates_velocity(self):
        gate = MotionStabilityGate(
            joint_names=["j1"], velocity_threshold=0.2, settle_time_sec=0.0,
            criterion=VELOCITY,
        )
        self.assertEqual(
            gate.update(["j1"], [0.0], [], stamp=1.0, now=1.0),
            "unknown",
        )
        self.assertEqual(
            gate.update(["j1"], [0.5], [], stamp=2.0, now=2.0),
            "moving",
        )
        self.assertEqual(
            gate.update(["j1"], [0.5], [], stamp=3.0, now=3.0),
            "stable",
        )

    def test_missing_joint_and_timeout_fail_closed(self):
        gate = MotionStabilityGate(
            joint_names=["j1", "j2"],
            settle_time_sec=0.0,
            joint_state_timeout_sec=0.5,
        )
        self.assertEqual(
            gate.update(["j1"], [0.0], [0.0], stamp=1.0, now=1.0),
            "unknown",
        )
        gate.update(
            ["j1", "j2"], [0.0, 0.0], [0.0, 0.0],
            stamp=2.0, now=2.0,
        )
        self.assertEqual(gate.state(now=3.0), "stale")
        self.assertFalse(gate.accepts_cloud(3.0, now=3.0))

    def test_invalid_position_and_time_reversal_reset_the_window(self):
        gate = MotionStabilityGate(
            joint_names=["j1"], settle_time_sec=0.5)
        gate.update(["j1"], [0.0], [0.0], stamp=2.0, now=2.0)
        self.assertEqual(
            gate.update(["j1"], [float("nan")], [0.0],
                        stamp=2.1, now=2.1),
            "unknown",
        )
        gate.update(["j1"], [0.0], [0.0], stamp=3.0, now=3.0)
        self.assertEqual(
            gate.update(["j1"], [0.0], [0.0], stamp=2.9, now=2.9),
            "unknown",
        )

    def test_invalid_criterion_fails_at_startup(self):
        with self.assertRaises(ValueError):
            MotionStabilityGate(criterion="average_velocity")


if __name__ == "__main__":
    unittest.main()
