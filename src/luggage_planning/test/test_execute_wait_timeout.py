"""Duration-aware ExecuteTrajectory wait for the typed timed-joint path.

A fixed 45 s wall deadline is shorter than a ~45 s nominal transit leg
and fails even at RTF 1.0 (rehearsal_07). The wait must be floored by
the leg's own trajectory duration with real-time-factor headroom; short
legs keep the caller's timeout.
"""

import unittest

from luggage_planning.motion_executor import execute_wait_timeout_sec


def _point(sec, nanosec=0):
    class _Stamp(object):
        pass

    class _Point(object):
        pass

    point = _Point()
    point.time_from_start = _Stamp()
    point.time_from_start.sec = int(sec)
    point.time_from_start.nanosec = int(nanosec)
    return point


class TestExecuteWaitTimeout(unittest.TestCase):
    def test_short_leg_keeps_caller_timeout(self):
        points = [_point(0), _point(2, 500_000_000)]
        self.assertEqual(execute_wait_timeout_sec(45.0, points), 45.0)

    def test_transit_length_leg_gets_headroom(self):
        # The certified transit leg is ~45 s nominal: the old fixed 45 s
        # deadline failed it even at RTF 1.0.
        points = [_point(0), _point(44), _point(45)]
        self.assertEqual(execute_wait_timeout_sec(45.0, points), 100.0)

    def test_longer_leg_scales(self):
        points = [_point(0), _point(60)]
        self.assertEqual(execute_wait_timeout_sec(45.0, points), 130.0)

    def test_larger_caller_timeout_is_kept(self):
        points = [_point(0), _point(10)]
        self.assertEqual(execute_wait_timeout_sec(120.0, points), 120.0)

    def test_missing_or_invalid_stamps_fall_back(self):
        class _Bare(object):
            pass

        self.assertEqual(execute_wait_timeout_sec(45.0, [_Bare()]), 45.0)
        self.assertEqual(execute_wait_timeout_sec(45.0, []), 45.0)
        zero = execute_wait_timeout_sec(45.0, [_point(0)])
        self.assertEqual(zero, 45.0)


if __name__ == "__main__":
    unittest.main()
