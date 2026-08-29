#!/usr/bin/env python3
"""Unit tests for StampRingBuffer (no roscore required)."""

import unittest

from luggage_perception.stamp_ring_buffer import StampRingBuffer


class TestStampRingBuffer(unittest.TestCase):
    def test_capacity_evicts_oldest(self):
        buf = StampRingBuffer(maxlen=3, horizon_sec=10.0)
        buf.insert(1.0, "a")
        buf.insert(1.1, "b")
        buf.insert(1.2, "c")
        buf.insert(1.3, "d")
        self.assertEqual(buf.stamps(), [1.1, 1.2, 1.3])
        self.assertEqual(buf.oldest()[1], "b")

    def test_horizon_prunes_old_stamps(self):
        buf = StampRingBuffer(maxlen=10, horizon_sec=0.2)
        buf.insert(1.0, "a")
        buf.insert(1.20, "b")
        buf.insert(1.35, "c")
        buf.prune(1.35)
        self.assertEqual(buf.stamps(), [1.20, 1.35])

    def test_out_of_order_insert_keeps_sort(self):
        buf = StampRingBuffer(maxlen=10, horizon_sec=1.0, rollback_sec=0.25)
        buf.insert(1.00, "a")
        buf.insert(1.04, "c")
        buf.insert(1.02, "b")
        self.assertEqual(buf.stamps(), [1.00, 1.02, 1.04])
        self.assertEqual([buf.nearest(s, 0.0)[1] for s in buf.stamps()],
                         ["a", "b", "c"])

    def test_rollback_flushes_buffer(self):
        buf = StampRingBuffer(maxlen=10, horizon_sec=1.0, rollback_sec=0.25)
        buf.insert(1.0, "a")
        buf.insert(1.2, "b")
        buf.insert(0.4, "c")
        self.assertEqual(buf.stamps(), [0.4])
        self.assertEqual(buf.latest()[1], "c")

    def test_nearest_slop_and_earlier_tie(self):
        buf = StampRingBuffer(maxlen=10, horizon_sec=1.0)
        buf.insert(1.00, "a")
        buf.insert(1.04, "b")
        hit = buf.nearest(1.02, 0.020)
        self.assertEqual(hit[1], "a")
        self.assertIsNone(buf.nearest(1.10, 0.020))

    def test_rejects_zero_and_nonfinite_stamps(self):
        buf = StampRingBuffer(maxlen=4, horizon_sec=1.0)
        self.assertFalse(buf.insert(0.0, "z"))
        self.assertFalse(buf.insert(float("nan"), "n"))
        self.assertFalse(buf.insert(float("-inf"), "i"))
        self.assertEqual(len(buf), 0)

    def test_same_stamp_replaces_item(self):
        buf = StampRingBuffer(maxlen=4, horizon_sec=1.0)
        buf.insert(1.0, "a")
        buf.insert(1.0, "b")
        self.assertEqual(len(buf), 1)
        self.assertEqual(buf.latest()[1], "b")


if __name__ == "__main__":
    unittest.main()
