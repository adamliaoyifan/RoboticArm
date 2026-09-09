import math
import unittest

from elfin_trajectory_executor.cps_parse import (
    as_float_list,
    finite_diff_deg_s,
    rpy_deg_to_quat_xyzw,
    tcp_from_read_act_pos,
)


class CpsParseTest(unittest.TestCase):
    def test_as_float_list_accepts_cps_strings(self):
        raw = ["1.5", "2", "-3.25", "0", "10", "20", "ignored"]
        self.assertEqual(
            as_float_list(raw, 6),
            [1.5, 2.0, -3.25, 0.0, 10.0, 20.0],
        )

    def test_as_float_list_rejects_short_or_garbage(self):
        self.assertIsNone(as_float_list(["1", "2"], 6))
        self.assertIsNone(as_float_list(["a"] * 6, 6))
        self.assertIsNone(as_float_list(None, 6))

    def test_tcp_slice_matches_read_act_tcp_pos(self):
        joints = [10, 20, 30, 40, 50, 60]
        tcp = [100.0, 200.0, 300.0, 1.0, 2.0, 3.0]
        rest = [0] * 12
        self.assertEqual(tcp_from_read_act_pos(joints + tcp + rest), tcp)
        self.assertIsNone(tcp_from_read_act_pos(joints))

    def test_finite_diff_vel(self):
        prev = [0.0] * 6
        curr = [10.0, 0, 0, 0, 0, 0]
        self.assertEqual(finite_diff_deg_s(prev, curr, 0.1)[0], 100.0)
        self.assertIsNone(finite_diff_deg_s(prev, curr, 1e-6))

    def test_rpy_identity_and_90_yaw(self):
        qx, qy, qz, qw = rpy_deg_to_quat_xyzw(0.0, 0.0, 0.0)
        self.assertAlmostEqual(qx, 0.0)
        self.assertAlmostEqual(qy, 0.0)
        self.assertAlmostEqual(qz, 0.0)
        self.assertAlmostEqual(qw, 1.0)
        qx, qy, qz, qw = rpy_deg_to_quat_xyzw(0.0, 0.0, 90.0)
        self.assertAlmostEqual(qx, 0.0)
        self.assertAlmostEqual(qy, 0.0)
        self.assertAlmostEqual(qz, math.sin(math.radians(45.0)))
        self.assertAlmostEqual(qw, math.cos(math.radians(45.0)))


if __name__ == "__main__":
    unittest.main()
