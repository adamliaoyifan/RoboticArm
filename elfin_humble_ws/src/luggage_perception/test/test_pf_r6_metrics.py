#!/usr/bin/env python3
"""Unit tests for PF-R6 pure instrumentation helpers."""

import unittest

from luggage_perception.pf_r6_metrics import (  # noqa: E402
    active_rate,
    first_latency_summary,
    stamp_delta_ms,
    stamp_match_summary,
    stats,
)


class TestPfR6Metrics(unittest.TestCase):

    def test_active_rate_ignores_long_idle_gap(self):
        stamps = [0.0, 0.25, 0.50, 10.0, 10.25]
        self.assertAlmostEqual(active_rate(stamps, max_gap_sec=1.0), 4.0)

    def test_stats_empty_and_percentiles(self):
        self.assertEqual(
            stats([]), {"n": 0, "p50": None, "p95": None, "max": None})
        payload = stats([4, 1, 2, 3])
        self.assertEqual(payload["n"], 4)
        self.assertEqual(payload["p50"], 3.0)
        self.assertEqual(payload["p95"], 4.0)
        self.assertEqual(payload["max"], 4.0)

    def test_stamp_delta_and_match_summary(self):
        stamps = {
            "a": {(1, 0): 10.0, (2, 0): 11.0},
            "b": {(1, 0): 10.125, (3, 0): 12.0},
        }
        self.assertEqual(stamp_delta_ms(stamps, "a", "b"), [125.0])
        self.assertEqual(stamp_match_summary(stamps, "a", "b"), {
            "left": "a",
            "right": "b",
            "left_count": 2,
            "right_count": 2,
            "matched": 1,
            "left_only": 1,
            "right_only": 1,
        })

    def test_first_latency_summary(self):
        events = {
            7: {
                "start": 1.0,
                "first_top_only": 1.2,
                "first_full_3d": 1.75,
            }
        }
        self.assertEqual(first_latency_summary(events), {
            "7": {"first_top_only_ms": 200.0, "first_full_3d_ms": 750.0}
        })


if __name__ == "__main__":
    unittest.main()
