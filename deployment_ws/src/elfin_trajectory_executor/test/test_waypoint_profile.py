"""实机代码 (site / real-cell). Not part of the Gazebo simulation stack."""

import math
import unittest

from elfin_trajectory_executor.waypoint_profile import (
    REASON_ACCEL_BELOW_MIN_VEL_RULE,
    REASON_CONTROLLER_LIMIT,
    REASON_JOINT_LIMIT,
    REASON_NONFINITE,
    REASON_TIME_NOT_INCREASING,
    ControllerLimits,
    build_waypoint_profiles,
    clamp_command_profile,
    parse_joint_limit_row,
)


class WaypointProfileTest(unittest.TestCase):
    def test_parse_limit_row_rejects_short_nan_nonpositive(self):
        self.assertEqual(
            parse_joint_limit_row(["120", "120", "120", "120", "120", "120"]),
            [120.0] * 6,
        )
        self.assertIsNone(parse_joint_limit_row(["120"] * 5))
        self.assertIsNone(parse_joint_limit_row(["nan"] * 6))
        self.assertIsNone(parse_joint_limit_row(["0", "1", "1", "1", "1", "1"]))
        self.assertIsNone(parse_joint_limit_row(["-1"] + ["10"] * 5))

    def test_historical_54_108_rejected_54_60_accepted(self):
        vel, accel, reason = clamp_command_profile(
            54.0, command_acceleration_deg=108.0, max_velocity_deg=60.0,
        )
        self.assertIsNone(vel)
        self.assertEqual(reason, REASON_CONTROLLER_LIMIT)
        vel, accel, reason = clamp_command_profile(
            54.0, command_acceleration_deg=60.0, max_velocity_deg=60.0,
        )
        self.assertAlmostEqual(vel, 54.0)
        self.assertAlmostEqual(accel, 60.0)
        self.assertIsNone(reason)

    def test_vel_clamped_to_accel_minus_one_and_controller_min(self):
        vel, accel, reason = clamp_command_profile(
            80.0,
            command_acceleration_deg=60.0,
            max_velocity_deg=80.0,
            min_controller_vel=40.0,
            min_controller_acc=55.0,
        )
        self.assertIsNone(reason)
        self.assertAlmostEqual(accel, 55.0)
        self.assertAlmostEqual(vel, 40.0)
        self.assertLessEqual(vel, accel - 1.0)

    def test_h1_accel_0_5_rejected(self):
        vel, accel, reason = clamp_command_profile(
            20.0, command_acceleration_deg=0.5, max_velocity_deg=20.0,
        )
        self.assertIsNone(vel)
        self.assertEqual(reason, REASON_ACCEL_BELOW_MIN_VEL_RULE)

    def test_build_rejects_nonfinite_and_nonincreasing_time(self):
        q0 = [0.0] * 6
        q1 = [10.0] + [0.0] * 5
        built = build_waypoint_profiles(
            [q0, [float("nan")] + [0] * 5], [0.0, 1.0], [0, 1],
            max_velocity_deg=20.0, command_acceleration_deg=60.0,
        )
        self.assertEqual(built.reason, REASON_NONFINITE)
        self.assertEqual(built.commands, [])
        built = build_waypoint_profiles(
            [q0, q1], [1.0, 1.0], [0, 1],
            max_velocity_deg=20.0, command_acceleration_deg=60.0,
        )
        self.assertEqual(built.reason, REASON_TIME_NOT_INCREASING)

    def test_build_rejects_later_joint_limit_with_no_commands(self):
        q0 = [0.0] * 6
        q1 = [0.0] * 6
        q1[1] = 400.0
        built = build_waypoint_profiles(
            [q0, q1], [0.0, 1.0], [0, 1],
            max_velocity_deg=20.0, command_acceleration_deg=60.0,
        )
        self.assertEqual(built.reason, REASON_JOINT_LIMIT)
        self.assertEqual(built.commands, [])

    def test_build_accepts_two_kept_waypoints(self):
        q0 = [0.0] * 6
        q1 = [0.0] * 6
        q1[1] = 25.0
        built = build_waypoint_profiles(
            [q0, q1], [0.0, 1.0], [0, 1],
            max_velocity_deg=20.0,
            default_velocity_deg=20.0,
            command_acceleration_deg=60.0,
            controller_limits=ControllerLimits(
                vel_deg=[100.0] * 6, acc_deg=[80.0] * 6,
            ),
        )
        self.assertIsNone(built.reason)
        self.assertEqual(len(built.commands), 2)
        for cmd in built.commands:
            self.assertLessEqual(cmd.accel_deg, 60.0)
            self.assertLessEqual(cmd.vel_deg, cmd.accel_deg - 1.0)
            self.assertGreaterEqual(cmd.vel_deg, 1.0)


if __name__ == "__main__":
    unittest.main()
