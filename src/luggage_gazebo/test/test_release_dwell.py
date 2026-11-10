#!/usr/bin/env python3
"""The release dwell: box falls clear before the retreat's first stroke.

gate3_aprime_r5 evidence: the retreat began 0.39 s after vacuum-off
while the box still sat at the release height; the panel swept the box
top for 20 ms. The dwell holds the certified knot on the SIM clock and
records the box pose at dwell end. Offline contract: zero dwell skips;
a positive dwell waits past anchor+dwell and samples the pose once; a
missing anchor degrades to 0 without crashing; the record never aborts.
"""
import os
import sys
import types
import unittest

_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

try:
    import rclpy  # noqa: F401
    import luggage_msgs.srv  # noqa: F401
    import place_smoke_driver
except ImportError:  # peers of the script may be absent
    place_smoke_driver = None


@unittest.skipIf(place_smoke_driver is None, "driver peers unavailable")
class ReleaseDwellTest(unittest.TestCase):

    def setUp(self):
        # The module is shared with the other driver tests: neutralize
        # only time.sleep (the dwell's wait loop) and restore it after.
        self._original_time = place_smoke_driver.time
        place_smoke_driver.time = types.SimpleNamespace(
            sleep=lambda _s: None)

    def tearDown(self):
        place_smoke_driver.time = self._original_time

    @staticmethod
    def _host(start, step):
        class _Host:
            def __init__(self):
                self.now = float(start)
                self.pose_queries = 0

            def ros_now_sec(self):
                self.now += step
                return self.now

            def _gz_box_pose(self):
                self.pose_queries += 1
                return [self.now, 1.0, 2.0, 0.64, 0.0, 0.0, 1.5708]
        return _Host()

    def _dwell(self, host, dwell_s, anchor):
        return place_smoke_driver.PlaceSmokeDriver._release_dwell(
            host, dwell_s, anchor)

    def test_zero_dwell_skips_without_touching_clock_or_pose(self):
        host = self._host(start=100.0, step=0.1)
        record = self._dwell(host, 0.0, 99.5)
        self.assertTrue(record.get("skipped"))
        self.assertEqual(record["anchor_sim"], 99.5)
        self.assertEqual(host.pose_queries, 0)
        self.assertNotIn("box_pose_at_dwell_end", record)

    def test_positive_dwell_waits_past_deadline_and_records_pose(self):
        host = self._host(start=100.0, step=0.075)
        record = self._dwell(host, 0.3, 100.0)
        self.assertNotIn("skipped", record)
        # Deadline is anchor + dwell; the loop exits on the first read
        # at/past it and the record's final read adds one more step.
        self.assertGreaterEqual(record["t_end_sim"], 100.3)
        self.assertLessEqual(record["t_end_sim"], 100.3 + 2 * 0.075)
        self.assertEqual(host.pose_queries, 1)
        # The pose sample is the separation evidence the verdict reads.
        self.assertEqual(record["box_pose_at_dwell_end"][3], 0.64)

    def test_missing_anchor_degrades_to_zero_without_crash(self):
        host = self._host(start=5.0, step=0.05)
        record = self._dwell(host, 0.1, None)
        self.assertEqual(record["anchor_sim"], 0.0)
        self.assertGreaterEqual(record["t_end_sim"], 0.1)
        self.assertIn("box_pose_at_dwell_end", record)


if __name__ == "__main__":
    unittest.main()
