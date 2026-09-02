#!/usr/bin/env python3
"""Unit tests for LockedStampWindow (no roscore)."""

from __future__ import division

import threading
import unittest

from luggage_perception.locked_stamp_window import LockedStampWindow


class TestLockedStampWindow(unittest.TestCase):
    def test_capacity_evicts_oldest(self):
        window = LockedStampWindow(maxlen=3)
        for i, stamp in enumerate((1.0, 1.1, 1.2, 1.3)):
            self.assertTrue(window.push(stamp, {"i": i}))
        self.assertEqual(len(window), 3)
        stamps = [item[0] for item in window.snapshot()]
        self.assertEqual(stamps, [1.1, 1.2, 1.3])

    def test_nearest_picks_closer_stamp(self):
        window = LockedStampWindow(maxlen=10)
        window.push(1.00, "a")
        window.push(1.04, "b")
        hit = window.nearest(1.01, 0.02)
        self.assertEqual(hit[1], "a")
        self.assertIsNone(window.nearest(1.10, 0.02))

    def test_get_returns_deepcopy(self):
        window = LockedStampWindow(maxlen=4)
        window.push(1.0, {"v": [1]})
        got = window.latest()[1]
        got["v"].append(2)
        self.assertEqual(window.latest()[1]["v"], [1])
        snap = window.snapshot()
        snap[0][1]["v"].append(3)
        self.assertEqual(window.latest()[1]["v"], [1])

    def test_concurrent_push_and_nearest(self):
        window = LockedStampWindow(maxlen=10)
        errors = []

        def writer(offset):
            try:
                for i in range(40):
                    window.push(1.0 + offset + i * 0.01, {"i": i})
            except Exception as exc:  # noqa: BLE001
                errors.append(exc)

        def reader():
            try:
                for _ in range(40):
                    window.nearest(1.2, 1.0)
                    window.latest()
                    window.snapshot()
            except Exception as exc:  # noqa: BLE001
                errors.append(exc)

        threads = [threading.Thread(target=writer, args=(k * 0.001,))
                   for k in range(3)]
        threads.extend(threading.Thread(target=reader) for _ in range(3))
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        self.assertEqual(errors, [])
        self.assertLessEqual(len(window), 10)


if __name__ == "__main__":
    unittest.main()
