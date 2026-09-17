"""实机代码 (site / real-cell). Not part of the Gazebo simulation stack."""

import unittest

from elfin_trajectory_executor.servo_esj import (
    VALUES_PER_POINT,
    batch_joint_points,
    flatten_servo_esj_points,
    interpolate_joint_grid,
    servo_push_allowed,
)


def _q(j2: float):
    q = [0.0] * 6
    q[1] = j2
    return q


class ServoEsjTest(unittest.TestCase):
    def test_interpolate_keeps_final_point_and_grid(self):
        joints = [_q(0.0), _q(10.0)]
        times = [0.0, 0.10]
        grid, reason = interpolate_joint_grid(joints, times, dt_s=0.02)
        self.assertIsNone(reason)
        self.assertEqual(grid[0], _q(0.0))
        self.assertEqual(grid[-1], _q(10.0))
        self.assertGreaterEqual(len(grid), 6)

    def test_batch_boundaries_499_500_501(self):
        pts = [_q(float(i)) for i in range(499)]
        batches = batch_joint_points(pts, 500)
        self.assertEqual(len(batches), 1)
        self.assertEqual(len(batches[0]), 499)
        pts = [_q(float(i)) for i in range(500)]
        batches = batch_joint_points(pts, 500)
        self.assertEqual([len(b) for b in batches], [500])
        pts = [_q(float(i)) for i in range(501)]
        batches = batch_joint_points(pts, 500)
        self.assertEqual([len(b) for b in batches], [500, 1])
        flat = flatten_servo_esj_points(batches[0])
        self.assertEqual(len(flat), 500 * VALUES_PER_POINT)
        self.assertEqual(VALUES_PER_POINT, 7)
        one = flatten_servo_esj_points([[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]])
        self.assertEqual(one, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0])
        timed = flatten_servo_esj_points(
            [[1.0, 2.0, 3.0, 4.0, 5.0, 6.0], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]],
            extras=[0.0, 0.02],
        )
        self.assertEqual(timed[-1], 0.02)
        self.assertEqual(timed[6], 0.0)

    def test_push_allowed_state(self):
        self.assertTrue(servo_push_allowed(["0"]))
        self.assertFalse(servo_push_allowed(["1"]))
        self.assertIsNone(servo_push_allowed([]))


if __name__ == "__main__":
    unittest.main()
