"""Rest-to-rest cartesian timing (no ROS graph)."""

import math
import unittest

from luggage_planning.trajectory_timing import (
    HARDWARE_ACC_RAD,
    HARDWARE_VEL_RAD,
    cartesian_joint_limits,
    time_parameterize_cartesian,
    time_parameterize_rest_to_rest,
)


def _q(j2_rad):
    q = [0.0] * 6
    q[1] = float(j2_rad)
    return q


class TrajectoryTimingTest(unittest.TestCase):
    def test_hardware_caps_beat_scaled_urdf_acc(self):
        v_max, a_max = cartesian_joint_limits(0.6, 0.6)
        self.assertAlmostEqual(v_max, min(1.57 * 0.6, HARDWARE_VEL_RAD))
        self.assertAlmostEqual(a_max, HARDWARE_ACC_RAD)
        self.assertLess(a_max, 3.14 * 0.6)

    def test_short_path_is_accel_limited_not_vmax(self):
        delta = math.radians(2.0)
        timed = time_parameterize_cartesian([_q(0.0), _q(delta)], 0.6, 0.6)
        v_max, a_max = cartesian_joint_limits(0.6, 0.6)
        self.assertGreater(timed.path_length, 0.0)
        self.assertGreater(timed.duration, timed.path_length / v_max)
        self.assertGreaterEqual(timed.duration, 0.3)
        self.assertLess(timed.v_peak, v_max)
        self.assertEqual(timed.velocities_rad[0], [0.0] * 6)
        self.assertEqual(timed.velocities_rad[-1], [0.0] * 6)

    def test_rest_to_rest_respects_vel_and_acc_caps(self):
        start = _q(0.0)
        end = _q(math.radians(20.0))
        mid = _q(math.radians(10.0))
        timed = time_parameterize_rest_to_rest(
            [start, mid, end], HARDWARE_VEL_RAD, HARDWARE_ACC_RAD)
        for vel in timed.velocities_rad:
            peak = max(abs(v) for v in vel)
            self.assertLessEqual(peak, HARDWARE_VEL_RAD + 1e-9)
        for acc in timed.accelerations_rad:
            peak = max(abs(a) for a in acc)
            self.assertLessEqual(peak, HARDWARE_ACC_RAD + 1e-6)
        for prev, nxt in zip(timed.times_s, timed.times_s[1:]):
            self.assertGreater(nxt, prev)

    def test_zero_path_stays_at_rest(self):
        timed = time_parameterize_cartesian([_q(0.1), _q(0.1)], 0.6, 0.6)
        self.assertAlmostEqual(timed.path_length, 0.0)
        self.assertEqual(timed.velocities_rad[0], [0.0] * 6)
        self.assertEqual(timed.velocities_rad[-1], [0.0] * 6)
        self.assertEqual(timed.accelerations_rad[0], [0.0] * 6)

    def test_launch_scale_does_not_use_totg_108_deg_s2(self):
        _v_max, a_max = cartesian_joint_limits(0.6, 0.6)
        totg_acc = 3.14 * 0.6
        self.assertAlmostEqual(a_max, HARDWARE_ACC_RAD)
        self.assertLess(a_max + 1e-9, totg_acc)
        timed = time_parameterize_cartesian(
            [_q(0.0), _q(math.radians(20.0))], 0.6, 0.6)
        for acc in timed.accelerations_rad:
            self.assertLessEqual(max(abs(a) for a in acc), a_max + 1e-6)
        self.assertGreater(timed.duration, 20.0 * math.pi / 180.0 / totg_acc)


if __name__ == "__main__":
    unittest.main()
