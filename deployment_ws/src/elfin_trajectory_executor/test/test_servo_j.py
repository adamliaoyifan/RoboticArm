"""实机代码 (site / real-cell). Not part of the Gazebo simulation stack."""

import math
import unittest

from elfin_trajectory_executor.servo_j import (
    densify_servo_j_path,
    fjt_to_servo_j_deg,
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
