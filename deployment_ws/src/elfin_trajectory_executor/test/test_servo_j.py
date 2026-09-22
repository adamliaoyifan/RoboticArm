"""实机代码 (site / real-cell). Not part of the Gazebo simulation stack."""

import math
import unittest

from elfin_trajectory_executor.servo_j import (
    SERVO_MAX_ACCEL_DEG,
    SERVO_MAX_DECEL_DEG,
    densify_servo_j_path,
    fjt_to_servo_j_deg,
    limit_servo_braking,
    servo_j_hold_count,
)


def _q(j2_rad: float):
    q = [0.0] * 6
    q[1] = j2_rad
    return q


class ServoJPathTest(unittest.TestCase):
    def test_moveit_010s_knots_resample_to_002s_grid(self):
        positions = [_q(0.0), _q(math.radians(10.0))]
        times = [0.0, 0.10]
        grid, reason = fjt_to_servo_j_deg(positions, times, dt_s=0.02)
        self.assertIsNone(reason)
        self.assertEqual(grid[0][1], 0.0)
        self.assertAlmostEqual(grid[-1][1], 10.0, places=6)
        self.assertEqual(len(grid), 6)

    def test_site_13_waypoint_010s_span(self):
        n = 13
        times = [0.1 * i for i in range(n)]
        positions = [_q(math.radians(float(i))) for i in range(n)]
        grid, reason = fjt_to_servo_j_deg(positions, times, dt_s=0.02)
        self.assertIsNone(reason)
        self.assertGreater(len(grid), n)
        self.assertAlmostEqual(grid[-1][1], float(n - 1), places=5)

    def test_densify_inserts_steps_under_velocity_cap(self):
        start = [0.0] * 6
        end = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        path = densify_servo_j_path([start, end], 0.02, 50.0)
        self.assertGreater(len(path), 2)
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], end)
        max_step = 50.0 * 0.02
        for prev, nxt in zip(path, path[1:]):
            delta = max(abs(nxt[i] - prev[i]) for i in range(6))
            self.assertLessEqual(delta, max_step + 1e-9)

    def test_braking_limit_stretches_a_hard_stop(self):
        # 40 deg/s down to 0 in one 20 ms step is 2000 deg/s^2.
        start = [0.0] * 6
        mid = [0.8, 0.0, 0.0, 0.0, 0.0, 0.0]
        end = [0.8, 0.0, 0.0, 0.0, 0.0, 0.0]
        path = limit_servo_braking([start, mid, end], 0.02, SERVO_MAX_DECEL_DEG)
        self.assertGreater(len(path), 3)
        self.assertEqual(path[0], start)
        self.assertAlmostEqual(path[-1][0], 0.8)
        prev_v = 0.0
        dt = 0.02
        for prev, nxt in zip(path, path[1:]):
            vel = (nxt[0] - prev[0]) / dt
            if prev_v * vel < 0.0:
                delta = abs(prev_v) + abs(vel)
            else:
                delta = abs(abs(prev_v) - abs(vel))
            self.assertLessEqual(delta / dt, SERVO_MAX_ACCEL_DEG + 1e-4)
            self.assertLessEqual(delta / dt, SERVO_MAX_DECEL_DEG + 1e-4)
            prev_v = vel
        # The hold repeats the last point, so the stream must already be stopped.
        self.assertLessEqual(abs(prev_v) / dt, SERVO_MAX_DECEL_DEG + 1e-4)

    def test_braking_limit_keeps_joint_path(self):
        path = []
        for index in range(15):
            row = [0.0] * 6
            row[0] = index * 0.8
            row[1] = index * 0.2
            path.append(row)
        path.append(list(path[-1]))
        limited = limit_servo_braking(path, 0.02, SERVO_MAX_DECEL_DEG)
        self.assertAlmostEqual(limited[-1][0], path[-1][0])
        self.assertAlmostEqual(limited[-1][1], path[-1][1])
        for sample in limited:
            if sample[0] > 1e-3:
                self.assertAlmostEqual(sample[1] / sample[0], 0.25, places=3)

    def test_hold_covers_lookahead_window(self):
        self.assertEqual(servo_j_hold_count(0.1, 0.02), 5)
        self.assertEqual(servo_j_hold_count(0.0, 0.02), 1)

    def test_knot_velocities_use_hermite_not_linear_chord(self):
        positions = [_q(0.0), _q(math.radians(2.0)), _q(math.radians(10.0))]
        times = [0.0, 0.20, 0.40]
        velocities = [
            [0.0] * 6,
            [0.0, math.radians(20.0), 0.0, 0.0, 0.0, 0.0],
            [0.0] * 6,
        ]
        linear, reason = fjt_to_servo_j_deg(positions, times, dt_s=0.02)
        hermite, hermite_reason = fjt_to_servo_j_deg(
            positions, times, dt_s=0.02, velocities_rad=velocities)
        self.assertIsNone(reason)
        self.assertIsNone(hermite_reason)
        self.assertEqual(len(linear), len(hermite))
        mid = hermite[5][1]
        linear_mid = linear[5][1]
        self.assertNotAlmostEqual(mid, linear_mid, places=3)
        self.assertAlmostEqual(hermite[0][1], 0.0, places=6)
        self.assertAlmostEqual(hermite[-1][1], 10.0, places=6)


if __name__ == "__main__":
    unittest.main()
