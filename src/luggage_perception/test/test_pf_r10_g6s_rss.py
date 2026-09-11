#!/usr/bin/env python3
"""ROS-free tests for the amended PF-R10 C2 labelled RSS growth gate."""

from __future__ import division

import unittest

from pf_r10_g6s_probe import (
    CurrentBoxLabeler,
    coverage_scorable,
    fit_samples,
    fit_size_adjusted_beta,
    luggage_size_label,
    rss_verdicts,
)


def _row(t, rss, size, generation, occurrence):
    return {
        "t": float(t),
        "rss_mib": float(rss),
        "generation": int(generation),
        "size": size,
        "occurrence": occurrence,
    }


def _cycle_rows(rss_fn, samples_per=12, dt=0.5):
    sizes = ("carryon", "standard", "large")
    rows = []
    t = 0.0
    generation = 1
    for occurrence in (1, 2):
        for size in sizes:
            for _ in range(samples_per):
                rows.append(_row(
                    t, rss_fn(t, size), size, generation, occurrence))
                t += dt
            generation += 1
    return rows


class _Probe(object):
    def __init__(self, rows):
        self._rss = {"luggage_detector": rows}
        self._rss_fit_series = {}
        self._rss_bucket_sec = 5.0
        self._pids = {"luggage_detector": 1}
        self._occupancy = {"filter.depth": [(rows[0]["t"], 0)]}


class TestLuggageSizeLabel(unittest.TestCase):

    def test_catalog_suffix_and_empty(self):
        self.assertEqual(
            luggage_size_label(
                '{"id": "pickup_box_0004_carryon", "generation": 7}'),
            "carryon")
        self.assertEqual(luggage_size_label({"id": "standard"}), "standard")
        self.assertEqual(luggage_size_label({"id": "", "generation": 3}), "")
        self.assertEqual(
            luggage_size_label({"id": "x", "width": 0.8, "depth": 0.5,
                                "height": 0.32}),
            "0.800x0.500x0.320")


class TestCurrentBoxLabeler(unittest.TestCase):

    def test_occurrence_increments_per_size(self):
        labeler = CurrentBoxLabeler()
        first = labeler.update(
            '{"id": "pickup_box_0001_carryon", "generation": 2}')
        self.assertEqual(first["size"], "carryon")
        self.assertEqual(first["occurrence"], 1)
        labeler.update('{"id": "", "generation": 3}')
        second = labeler.update(
            '{"id": "pickup_box_0002_carryon", "generation": 4}')
        self.assertEqual(second["occurrence"], 2)
        large = labeler.update(
            '{"id": "pickup_box_0003_large", "generation": 5}')
        self.assertEqual(large["size"], "large")
        self.assertEqual(large["occurrence"], 1)


class TestCoverageAndFit(unittest.TestCase):

    def test_two_occurrences_are_scorable(self):
        rows = _cycle_rows(lambda t, size: 100.0)
        ok, reason, per_size = coverage_scorable(rows)
        self.assertTrue(ok, reason)
        self.assertEqual(per_size["carryon"]["qualified_occurrences"], 2)

    def test_one_occurrence_is_not_scorable(self):
        rows = [
            _row(i * 0.5, 100.0, "carryon", 1, 1) for i in range(12)]
        ok, reason, _ = coverage_scorable(rows)
        self.assertFalse(ok)
        self.assertIn("carryon", reason)


class TestSizeAdjustedBeta(unittest.TestCase):

    def test_size_step_without_time_trend_has_near_zero_beta(self):
        def rss(_t, size):
            return 120.0 if size == "large" else 100.0

        rows = _cycle_rows(rss)
        settled = fit_samples(rows, occupancy_ready_t=rows[0]["t"])
        fitted = fit_size_adjusted_beta(settled)
        self.assertTrue(fitted["scorable"], fitted)
        self.assertLess(abs(fitted["beta_mib_per_min"]), 0.05)
        self.assertGreater(fitted["size_effects_mib"]["large"], 15.0)

    def test_time_leak_after_size_control_fails_two_mib_bar(self):
        def rss(t, size):
            return 100.0 + (20.0 if size == "large" else 0.0) + 5.0 * (t / 60.0)

        rows = _cycle_rows(rss)
        settled = fit_samples(rows, occupancy_ready_t=rows[0]["t"])
        fitted = fit_size_adjusted_beta(settled)
        self.assertTrue(fitted["scorable"], fitted)
        self.assertGreater(fitted["beta_mib_per_min"], 4.0)

    def test_verdict_does_not_use_raw_slope_as_the_gate(self):
        def rss(_t, size):
            return 120.0 if size == "large" else 90.0

        rows = _cycle_rows(rss)
        verdict = rss_verdicts(_Probe(rows))["luggage_detector"]
        self.assertTrue(verdict["raw_slope_is_diagnostic"])
        self.assertTrue(verdict["growth_scorable"])
        self.assertTrue(verdict["pass"])
        self.assertLess(abs(verdict["beta_mib_per_min"]), 0.05)
        # Mixed carryon-to-large order makes the unadjusted slope look like
        # growth even though the size-controlled coefficient does not.
        self.assertGreater(verdict["slope_mib_per_min"], 2.0)


if __name__ == "__main__":
    unittest.main()
