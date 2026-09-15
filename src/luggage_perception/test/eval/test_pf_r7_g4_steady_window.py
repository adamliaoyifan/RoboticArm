#!/usr/bin/env python3
"""PF-R7 generation-4 support-ready 8 s half-open window. No ROS."""

from __future__ import division

import json
import os
import unittest

from luggage_perception.eval import gate4_scoring as scoring
from luggage_perception.eval.pf_r7_classifier import (
    CLASS_ELIGIBLE_FAIL,
    CLASS_ELIGIBLE_PASS,
    CLASS_EVIDENCE,
    classify_attempt,
)

from pf_r7_fixtures import pass_record


_CARRYON = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "data", "pfr7_g3_carryon00_scores.jsonl")
_STANDARD = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "data", "pfr7_g4_standard00_scores.jsonl")
INSTANCE = "pickup_box_0001_carryon"
GEN = 2
STANDARD_INSTANCE = "pickup_box_0002_standard"
STANDARD_GEN = 4


def _load_jsonl(path):
    rows = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def _load_carryon00():
    return _load_jsonl(_CARRYON)


def _load_standard00():
    return _load_jsonl(_STANDARD)


def _ok_row(**over):
    row = {
        "instance_id": INSTANCE,
        "generation": GEN,
        "stamp_sec": 10.0,
        "monotonic_sec": 10.0,
        "top_surface_valid": True,
        "height_valid": True,
        "geometry_level": 1,
        "height_source": 1,
        "pca_source": "measure",
        "support_reason": "ok",
        "support_inliers": 2000,
        "support_side_coverage": 1.0,
        "err_top_m": 0.004,
        "err_support_m": 0.004,
        "err_height_m": 0.006,
        "err_xy_m": 0.008,
        "err_width_m": 0.010,
        "err_depth_m": 0.009,
        "false_measured_height": False,
        "n_cargo_points": 8000,
    }
    row.update(over)
    return row


def _hold_track(**over):
    params = dict(
        pca_source="hold_track",
        support_reason="DETECT_SUPPORT_STAMP_MISMATCH",
        height_valid=False,
        geometry_level=0,
        height_source=3,
        support_inliers=0,
        support_side_coverage=0.0,
        err_support_m=None,
        err_height_m=None,
    )
    params.update(over)
    return _ok_row(**params)


def _fill(**over):
    params = dict(
        height_valid=False,
        geometry_level=0,
        height_source=3,
        support_reason="DETECT_SUPPORT_UNSTABLE",
        err_support_m=None,
        err_height_m=None,
    )
    params.update(over)
    return _ok_row(**params)


def _transition_prefix(t_steady):
    rows = []
    for i in range(3):
        stamp = t_steady - 1.0 + 0.05 * i
        rows.append(_hold_track(stamp_sec=stamp, monotonic_sec=stamp))
    for i in range(4):
        stamp = t_steady - 0.4 + 0.05 * i
        rows.append(_fill(stamp_sec=stamp, monotonic_sec=stamp))
    return rows


def _window_rows(t_steady, n, dt=0.08, mutate=None):
    rows = []
    for i in range(n):
        stamp = t_steady + dt * i
        row = _ok_row(stamp_sec=stamp, monotonic_sec=stamp)
        if mutate is not None:
            mutate(row, i, stamp)
        rows.append(row)
    return rows


def _split(rows, clock_end_sec=None, drain_rows=None,
           instance_id=INSTANCE, generation=GEN):
    return scoring.split_steady_windows(
        rows, instance_id, generation, drain_rows=drain_rows or [],
        clock_end_sec=clock_end_sec)


def _g4_record(windows, t_prop, t_full=0.2, **over):
    t_steady = windows.get("t_steady")
    t_steady_dt = None
    if t_steady is not None and t_prop is not None:
        t_steady_dt = max(0.0, float(t_steady) - float(t_prop))
    record = pass_record(
        score_mode=scoring.STEADY_START_SUPPORT_READY,
        steady_start=scoring.STEADY_START_SUPPORT_READY,
        settled=windows.get("settled") or [],
        warmup=[],
        n_settled=len(windows.get("settled") or []),
        t_proposal=0.0,
        t_proposal_stamp=t_prop,
        t_steady=t_steady,
        t_steady_sec=t_steady_dt,
        t_first_valid_sec=0.0,
        t_first_full3d_sec=t_full,
        stale_pre_barrier_observed=windows.get("stale_pre_barrier_observed", 0),
        stale_post_barrier_dropped=windows.get("stale_post_barrier_dropped", 0),
        stale_scored_or_fused=windows.get("stale_scored_or_fused", 0),
        recovery={
            "spawn_ok": True,
            "n_settled": len(windows.get("settled") or []),
            "t_first_valid_sec": 0.0,
            "t_first_full3d_sec": t_full,
            "t_steady_sec": t_steady_dt,
        },
        steady_window={
            "t_steady": t_steady,
            "t_steady_end": windows.get("t_steady_end"),
            "window_sec": windows.get("window_sec", scoring.STEADY_WINDOW_SEC),
            "window_complete": windows.get("window_complete"),
            "clock_end_sec": windows.get("clock_end_sec"),
            "missing_stamp": windows.get("missing_stamp"),
            "n_transition": windows.get("n_transition"),
            "n_settled": windows.get("n_settled"),
        },
    )
    record.update(over)
    return record


class TestCarryon00Boundary(unittest.TestCase):
    def test_dump_selects_frame19_and_cannot_pass_g4(self):
        rows = _load_carryon00()
        self.assertEqual(len(rows), 160)
        windows = _split(rows)
        self.assertAlmostEqual(windows["t_steady"], 29.106, places=3)
        self.assertEqual(windows["n_transition"], 19)
        self.assertEqual(len(windows["transition"]), 19)
        self.assertFalse(windows["window_complete"])
        summary = scoring.aggregate([{
            "warmup": [],
            "settled": windows["settled"],
            "instance_id": INSTANCE,
        }])
        self.assertEqual(summary["categories"]["full3d"], 140)
        self.assertEqual(summary["n_settled"], 141)
        self.assertAlmostEqual(summary["full3d_rate"], 140 / 141.0, places=6)
        self.assertAlmostEqual(summary["full3d_rate"], 0.993, places=3)
        t_prop = scoring.row_time_sec(windows["owned_recovery"][0])
        classified = classify_attempt(_g4_record(
            windows, t_prop, t_full=max(0.0, 29.106 - t_prop)))
        self.assertEqual(classified["attempt_class"], CLASS_EVIDENCE, classified)
        self.assertNotEqual(classified["attempt_class"], CLASS_ELIGIBLE_PASS)
        self.assertTrue(
            any("window_complete" in item or "clock_did_not" in item
                for item in classified["reasons"]),
            classified["reasons"])


class TestLeftoverOccupancyIsNotReady(unittest.TestCase):
    def test_leftover_count5_without_admit_does_not_start_window(self):
        leftover = []
        for i in range(10):
            stamp = 10.0 + 0.05 * i
            leftover.append(_hold_track(
                stamp_sec=stamp, monotonic_sec=stamp, pca_source="empty",
                support_sample_admitted=False, support_window_count=5,
                support_window_size=5))
        t_steady = 10.6
        ready = []
        for i in range(40):
            stamp = t_steady + 0.08 * i
            ready.append(_ok_row(
                stamp_sec=stamp, monotonic_sec=stamp,
                support_sample_admitted=True, support_window_count=5,
                support_window_size=5))
        end = t_steady + 8.0
        rows = leftover + ready + [_ok_row(
            stamp_sec=end, monotonic_sec=end,
            support_sample_admitted=True, support_window_count=5,
            support_window_size=5)]
        windows = _split(rows, clock_end_sec=end)
        self.assertAlmostEqual(windows["t_steady"], t_steady, places=6)
        self.assertEqual(windows["n_transition"], 10)
        self.assertTrue(all(
            r.get("support_sample_admitted") is False
            for r in windows["transition"]))
        self.assertTrue(windows["window_complete"])
        classified = classify_attempt(_g4_record(windows, 10.0, t_full=0.6))
        self.assertEqual(
            classified["attempt_class"], CLASS_ELIGIBLE_PASS, classified)

    def test_standard00_dump_selects_45_937(self):
        rows = _load_standard00()
        self.assertEqual(len(rows), 136)
        self.assertEqual(rows[0].get("support_window_count"), 5)
        self.assertFalse(bool(rows[0].get("support_sample_admitted")))
        windows = _split(
            rows, instance_id=STANDARD_INSTANCE, generation=STANDARD_GEN)
        self.assertAlmostEqual(windows["t_steady"], 45.937, places=3)
        self.assertEqual(windows["n_transition"], 10)
        self.assertTrue(all(
            str(r.get("pca_source") or "") != "measure"
            for r in windows["transition"]))
        self.assertGreaterEqual(windows["n_settled"], 30)
        summary = scoring.aggregate([{
            "warmup": [],
            "settled": windows["settled"],
            "instance_id": STANDARD_INSTANCE,
        }])
        self.assertGreaterEqual(summary["full3d_rate"], 0.95)
        self.assertNotAlmostEqual(summary["full3d_rate"], 0.903, places=3)


class TestHalfOpenBoundary(unittest.TestCase):
    def test_includes_start_excludes_exact_end(self):
        t_steady = 10.0
        rows = _transition_prefix(t_steady)
        rows.extend(_window_rows(t_steady, 100, dt=0.08))
        rows.append(_ok_row(stamp_sec=18.0, monotonic_sec=18.0))
        windows = _split(rows, clock_end_sec=18.0)
        self.assertAlmostEqual(windows["t_steady"], t_steady, places=6)
        self.assertTrue(windows["window_complete"])
        stamps = [scoring.row_time_sec(r) for r in windows["settled"]]
        self.assertIn(t_steady, stamps)
        self.assertNotIn(18.0, stamps)
        self.assertLess(max(stamps), t_steady + 8.0)
        self.assertEqual(windows["n_transition"], 7)


class TestRateThresholdUnchanged(unittest.TestCase):
    def _rate_case(self, n_full, n_total=100):
        t_steady = 10.0
        rows = _transition_prefix(t_steady)

        def mutate(row, i, stamp):
            if i >= n_full:
                row["height_valid"] = False
                row["geometry_level"] = 0
                row["height_source"] = 3
                row["err_support_m"] = None
                row["err_height_m"] = None
                row["support_reason"] = "DETECT_SUPPORT_UNOBSERVABLE"

        rows.extend(_window_rows(t_steady, n_total, dt=0.08, mutate=mutate))
        rows.append(_ok_row(stamp_sec=18.0, monotonic_sec=18.0))
        return _split(rows, clock_end_sec=18.0)

    def test_95_of_100_passes_and_94_fails(self):
        self.assertEqual(scoring.GATE4_LIMITS["full3d_rate"], 0.95)
        self.assertEqual(scoring.GATE4_LIMITS["top_surface_rate"], 0.95)
        win95 = self._rate_case(95)
        summary95 = scoring.gate_pass(scoring.aggregate([{
            "warmup": [], "settled": win95["settled"],
            "instance_id": INSTANCE,
        }]))
        self.assertEqual(summary95["categories"]["full3d"], 95)
        self.assertAlmostEqual(summary95["full3d_rate"], 0.95, places=6)
        self.assertTrue(summary95["gate4_pass"], summary95["gate4_failures"])
        classified95 = classify_attempt(_g4_record(win95, 9.0, t_full=0.2))
        self.assertEqual(
            classified95["attempt_class"], CLASS_ELIGIBLE_PASS, classified95)

        win94 = self._rate_case(94)
        summary94 = scoring.gate_pass(scoring.aggregate([{
            "warmup": [], "settled": win94["settled"],
            "instance_id": INSTANCE,
        }]))
        self.assertEqual(summary94["categories"]["full3d"], 94)
        self.assertAlmostEqual(summary94["full3d_rate"], 0.94, places=6)
        self.assertFalse(summary94["gate4_pass"])
        classified94 = classify_attempt(_g4_record(win94, 9.0, t_full=0.2))
        self.assertEqual(
            classified94["attempt_class"], CLASS_ELIGIBLE_FAIL, classified94)
        self.assertTrue(
            any("full3d_rate" in item for item in classified94["reasons"]),
            classified94["reasons"])


class TestTransitionExcludedPostReadyScored(unittest.TestCase):
    def test_pre_ready_hold_and_fill_excluded_post_ready_stay(self):
        t_steady = 10.0
        rows = _transition_prefix(t_steady)

        def mutate(row, i, stamp):
            if i == 10:
                row.clear()
                row.update(_hold_track(stamp_sec=stamp, monotonic_sec=stamp))
            elif i == 20:
                row["support_reason"] = "DETECT_SUPPORT_UNSTABLE"
                row["height_valid"] = False
                row["geometry_level"] = 0
                row["err_support_m"] = None
                row["err_height_m"] = None
            elif i == 30:
                row["support_reason"] = "DETECT_SUPPORT_UNOBSERVABLE"
                row["height_valid"] = False
                row["geometry_level"] = 0
                row["err_support_m"] = None
                row["err_height_m"] = None
            elif i == 40:
                row["pca_source"] = "measure"
                row["support_reason"] = "DETECT_SUPPORT_STAMP_MISMATCH"
                row["height_valid"] = False
                row["geometry_level"] = 0
                row["support_inliers"] = 0
                row["err_support_m"] = None
                row["err_height_m"] = None

        rows.extend(_window_rows(t_steady, 100, dt=0.08, mutate=mutate))
        rows.append(_ok_row(stamp_sec=18.0, monotonic_sec=18.0))
        windows = _split(rows, clock_end_sec=18.0)
        trans_src = [r.get("pca_source") for r in windows["transition"]]
        self.assertEqual(trans_src.count("hold_track"), 3)
        self.assertEqual(len(windows["transition"]), 7)
        reasons = [r.get("support_reason") for r in windows["settled"]]
        sources = [r.get("pca_source") for r in windows["settled"]]
        self.assertIn("DETECT_SUPPORT_UNSTABLE", reasons)
        self.assertIn("DETECT_SUPPORT_UNOBSERVABLE", reasons)
        self.assertIn("DETECT_SUPPORT_STAMP_MISMATCH", reasons)
        self.assertIn("hold_track", sources)
        summary = scoring.aggregate([{
            "warmup": [], "settled": windows["settled"],
            "instance_id": INSTANCE,
        }])
        self.assertEqual(summary["n_settled"], 100)
        self.assertLess(summary["full3d_rate"], 1.0)


class TestFailClosedEvidenceAndLatency(unittest.TestCase):
    def test_no_ready_boundary_is_eligible_fail(self):
        rows = [_hold_track(stamp_sec=10.0 + 0.05 * i,
                            monotonic_sec=10.0 + 0.05 * i)
                for i in range(40)]
        windows = _split(rows, clock_end_sec=18.0)
        self.assertIsNone(windows["t_steady"])
        self.assertFalse(windows["window_complete"])
        classified = classify_attempt(_g4_record(windows, 10.0, t_full=None))
        self.assertEqual(
            classified["attempt_class"], CLASS_ELIGIBLE_FAIL, classified)
        self.assertTrue(
            any("t_steady" in item or "full3d" in item
                for item in classified["reasons"]),
            classified["reasons"])

    def test_late_full3d_is_eligible_fail(self):
        t_steady = 10.0
        rows = _transition_prefix(t_steady)

        def mutate(row, i, stamp):
            if stamp < t_steady + 1.6:
                row["height_valid"] = False
                row["geometry_level"] = 0
                row["height_source"] = 3
                row["err_support_m"] = None
                row["err_height_m"] = None

        rows.extend(_window_rows(t_steady, 100, dt=0.08, mutate=mutate))
        rows.append(_ok_row(stamp_sec=18.0, monotonic_sec=18.0))
        windows = _split(rows, clock_end_sec=18.0)
        self.assertIsNotNone(windows["t_steady"])
        classified = classify_attempt(_g4_record(
            windows, t_prop=9.9, t_full=1.7))
        self.assertEqual(
            classified["attempt_class"], CLASS_ELIGIBLE_FAIL, classified)
        self.assertTrue(
            any("full3d" in item for item in classified["reasons"]),
            classified["reasons"])

    def test_missing_stamp_is_evidence(self):
        t_steady = 10.0
        rows = _transition_prefix(t_steady)
        rows.extend(_window_rows(t_steady, 100, dt=0.08))
        rows[20]["stamp_sec"] = None
        rows[20]["monotonic_sec"] = None
        windows = _split(rows, clock_end_sec=18.0)
        self.assertTrue(windows["missing_stamp"])
        self.assertFalse(windows["window_complete"])
        classified = classify_attempt(_g4_record(windows, 9.0, t_full=0.2))
        self.assertEqual(classified["attempt_class"], CLASS_EVIDENCE, classified)
        self.assertTrue(
            any("stamp" in item for item in classified["reasons"]),
            classified["reasons"])

    def test_incomplete_eight_seconds_is_evidence(self):
        t_steady = 10.0
        rows = _transition_prefix(t_steady)
        rows.extend(_window_rows(t_steady, 40, dt=0.08))
        windows = _split(rows)
        self.assertFalse(windows["window_complete"])
        classified = classify_attempt(_g4_record(windows, 9.0, t_full=0.2))
        self.assertEqual(classified["attempt_class"], CLASS_EVIDENCE, classified)
        self.assertNotEqual(classified["attempt_class"], CLASS_ELIGIBLE_PASS)

    def test_fewer_than_30_rows_cannot_pass(self):
        t_steady = 10.0
        rows = _transition_prefix(t_steady)
        rows.extend(_window_rows(t_steady, 25, dt=0.3))
        rows.append(_ok_row(stamp_sec=18.0, monotonic_sec=18.0))
        windows = _split(rows, clock_end_sec=18.0)
        self.assertTrue(windows["window_complete"])
        self.assertLess(windows["n_settled"], 30)
        classified = classify_attempt(_g4_record(windows, 9.0, t_full=0.2))
        self.assertEqual(
            classified["attempt_class"], CLASS_ELIGIBLE_FAIL, classified)
        self.assertTrue(
            any("settled" in item or "output_hz" in item
                for item in classified["reasons"]),
            classified["reasons"])

    def test_unprovable_clock_closure_is_evidence(self):
        t_steady = 10.0
        rows = _transition_prefix(t_steady)
        rows.extend(_window_rows(t_steady, 80, dt=0.08))
        windows = _split(rows, clock_end_sec=16.0)
        self.assertFalse(windows["window_complete"])
        classified = classify_attempt(_g4_record(windows, 9.0, t_full=0.2))
        self.assertEqual(classified["attempt_class"], CLASS_EVIDENCE, classified)


class TestStaleStillFailClosed(unittest.TestCase):
    def test_quarantined_stale_is_diagnostic(self):
        t_steady = 10.5
        drain = [_ok_row(
            instance_id="pickup_box_0000_standard", generation=1,
            stamp_sec=9.9, monotonic_sec=9.9)]
        rows = _transition_prefix(t_steady)
        rows.extend(_window_rows(t_steady, 40, dt=0.08))
        rows.append(_ok_row(stamp_sec=18.5, monotonic_sec=18.5))
        windows = _split(rows, clock_end_sec=18.5, drain_rows=drain)
        self.assertGreater(windows["stale_pre_barrier_observed"], 0)
        self.assertEqual(windows["stale_scored_or_fused"], 0)
        classified = classify_attempt(_g4_record(windows, 9.5, t_full=0.2))
        self.assertEqual(
            classified["attempt_class"], CLASS_ELIGIBLE_PASS, classified)

    def test_stale_scored_or_fused_fails(self):
        t_steady = 10.0
        rows = _transition_prefix(t_steady)
        rows.extend(_window_rows(t_steady, 40, dt=0.08))
        rows.append(_ok_row(stamp_sec=18.0, monotonic_sec=18.0))
        windows = _split(rows, clock_end_sec=18.0)
        fused = list(windows["settled"])
        fused[3] = _ok_row(
            instance_id="pickup_box_0000_standard", generation=1,
            stamp_sec=fused[3]["stamp_sec"])
        record = _g4_record(windows, 9.0, t_full=0.2, settled=fused,
                            stale_scored_or_fused=1)
        classified = classify_attempt(record)
        self.assertEqual(
            classified["attempt_class"], CLASS_ELIGIBLE_FAIL, classified)
        self.assertIn("stale_scored_or_fused", classified["reasons"])


if __name__ == "__main__":
    unittest.main()
