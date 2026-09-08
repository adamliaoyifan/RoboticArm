#!/usr/bin/env python3
"""PF-G4H: Gate 4 evaluator semantics on synthetic records (PF-R4).

docs/plans/platform_free_height_remediation.md. Absence of samples can
never pass an accuracy metric; warmup is reported but excluded only by
the documented rule; stale instance frames are ignored; spawn gaps do
not dilute the active-window rate.
"""

import unittest

from luggage_perception.eval import gate4_scoring as s


def _row(**over):
    """A good settled FULL_3D row by default."""
    row = {
        "instance_id": "box_0001", "generation": 3,
        "top_surface_valid": True, "height_valid": True,
        "geometry_level": 1, "height_source": 1,
        "err_top_m": 0.005, "err_support_m": 0.005,
        "err_height_m": 0.008, "err_xy_m": 0.010,
        "err_width_m": 0.012, "err_depth_m": 0.010,
        "false_measured_height": False,
    }
    row.update(over)
    return row


def _trial(rows, warmup=5):
    owned = rows
    return {
        "warmup": owned[:warmup], "settled": owned[warmup:],
        "instance_id": "box_0001", "generation": 3,
    }


class TestInstanceFiltering(unittest.TestCase):

    def test_stale_instance_generation_frames_are_ignored(self):
        rows = [
            _row(instance_id="box_0000", generation=2),
            _row(instance_id="box_0001", generation=3),
            _row(instance_id="box_0001", generation=3),
        ]
        kept = s.filter_instance(rows, "box_0001", 3)
        self.assertEqual(len(kept), 2)
        self.assertTrue(all(
            r["instance_id"] == "box_0001" for r in kept))


class TestWarmupSplit(unittest.TestCase):

    def test_warmup_reported_but_excluded_from_scoring(self):
        rows = ([_row(top_surface_valid=False, height_valid=False,
                      geometry_level=0, height_source=0)] * 5   # warmup
                + [_row()] * 20)
        warmup, settled = s.split_warmup(rows, warmup_frames=5)
        self.assertEqual(len(warmup), 5)
        self.assertEqual(len(settled), 20)
        summary = s.aggregate([{"warmup": warmup, "settled": settled}])
        # Warmup TOP_ONLY failures did not contaminate the settled rate.
        self.assertEqual(summary["categories"]["warmup_frames"], 5)
        self.assertEqual(summary["categories"]["failed"], 0)
        self.assertEqual(summary["top_surface_rate"], 1.0)

    def test_fewer_rows_than_warmup_all_warmup(self):
        warmup, settled = s.split_warmup([_row()] * 3, warmup_frames=5)
        self.assertEqual(len(warmup), 3)
        self.assertEqual(len(settled), 0)


class TestZeroSamplesFail(unittest.TestCase):

    def test_zero_full3d_frames_fail_the_gate(self):
        rows = ([_row()] * 5   # warmup
                + [_row(top_surface_valid=True, height_valid=False,
                        geometry_level=0, height_source=0)] * 20)
        summary = s.gate_pass(s.aggregate([_trial(rows)]))
        self.assertFalse(summary["gate4_pass"])
        self.assertTrue(any(
            "zero FULL_3D" in f for f in summary["gate4_failures"]))
        self.assertTrue(any(
            "no samples" in f for f in summary["gate4_failures"]))

    def test_missing_error_samples_fail_required_metrics(self):
        # Tops valid but every estimate lacks width/depth references.
        rows = ([_row()] * 5 + [
            _row(err_top_m=0.004, err_xy_m=0.008,
                 err_support_m=None, err_height_m=None,
                 err_width_m=None, err_depth_m=None,
                 height_valid=True, geometry_level=1)] * 20)
        summary = s.gate_pass(s.aggregate([_trial(rows)]))
        self.assertFalse(summary["gate4_pass"])
        self.assertTrue(any("width_err_m" in f and "no samples" in f
                            for f in summary["gate4_failures"]))
        self.assertTrue(any("depth_err_m" in f and "no samples" in f
                            for f in summary["gate4_failures"]))

    def test_no_settled_frames_at_all_fails(self):
        rows = [_row()] * 4  # everything lands in warmup
        summary = s.gate_pass(s.aggregate([_trial(rows)]))
        self.assertFalse(summary["gate4_pass"])
        self.assertIn("no settled frames", summary["gate4_failures"])

    def test_negative_control_mode_still_gates_accuracy(self):
        # semantic mode with full geometry relaxed is no longer offered;
        # the raw-only control has its own verdict (below). This test
        # keeps the invariant that a semantic run with zero FULL_3D
        # frames fails.
        rows = ([_row()] * 5 + [
            _row(top_surface_valid=True, height_valid=False,
                 geometry_level=0, height_source=0)] * 20)
        summary = s.gate_pass(s.aggregate([_trial(rows)]))
        self.assertFalse(summary["gate4_pass"])


class TestNegativeControlVerdict(unittest.TestCase):
    """Raw-only fail-closed control (PF-R4 rework)."""

    def _control_row(self, **over):
        row = {
            "pca_valid": False,
            "pca_reason": s.CARGO_SEGMENTATION_REQUIRED,
            "top_surface_valid": False, "height_valid": False,
            "geometry_level": 0, "height_source": 0,
        }
        row.update(over)
        return row

    def test_all_segmentation_failures_pass_the_control(self):
        verdict = s.negative_control_verdict(
            [self._control_row() for _ in range(10)])
        self.assertTrue(verdict["negative_control_pass"],
                        verdict["negative_control_failures"])

    def test_zero_frames_fail_the_control(self):
        verdict = s.negative_control_verdict([])
        self.assertFalse(verdict["negative_control_pass"])

    def test_any_valid_top_fails_the_control(self):
        rows = [self._control_row() for _ in range(9)]
        rows.append(self._control_row(top_surface_valid=True))
        verdict = s.negative_control_verdict(rows)
        self.assertFalse(verdict["negative_control_pass"])
        self.assertTrue(any("valid top" in f
                            for f in verdict["negative_control_failures"]))

    def test_any_measured_geometry_fails_the_control(self):
        rows = [self._control_row() for _ in range(9)]
        rows.append(self._control_row(height_valid=True, geometry_level=1))
        verdict = s.negative_control_verdict(rows)
        self.assertFalse(verdict["negative_control_pass"])
        self.assertTrue(any("measured" in f
                            for f in verdict["negative_control_failures"]))

    def test_other_failure_reason_fails_the_control(self):
        rows = [self._control_row() for _ in range(9)]
        rows.append(self._control_row(pca_reason="DETECT_TF_FAILED"))
        verdict = s.negative_control_verdict(rows)
        self.assertFalse(verdict["negative_control_pass"])
        self.assertTrue(any("DETECT_CARGO_SEGMENTATION_REQUIRED" in f
                            for f in verdict["negative_control_failures"]))


class TestExpectedIdentityFromEvalSide(unittest.TestCase):

    def test_stale_frames_are_not_adopted_as_the_new_instance(self):
        """Only stale frames arrive after a spawn: they stay stale.

        The expected identity comes from the eval-side spawn response;
        deriving it from the last detector row would score the previous
        box against the new GT.
        """
        rows = [
            _row(instance_id="box_OLD", generation=2),  # stale
            _row(instance_id="box_OLD", generation=2),  # stale
        ]
        owned, stale = s.filter_expected_instance(rows, "box_NEW")
        self.assertEqual(len(owned), 0)
        self.assertEqual(stale, 2)

    def test_matching_instance_kept(self):
        rows = [
            _row(instance_id="box_OLD", generation=2),
            _row(instance_id="box_NEW", generation=3),
        ]
        owned, stale = s.filter_expected_instance(rows, "box_NEW")
        self.assertEqual(len(owned), 1)
        self.assertEqual(stale, 1)


class TestCoverageGate(unittest.TestCase):

    def _coverage(self, n_sizes=3, n_xy=4, n_yaws=3, trials=30,
                 per_size=(10, 10, 10)):
        return {
            "n_sizes": n_sizes, "n_xy_offsets": n_xy, "n_yaws": n_yaws,
            "n_trials": trials,
            "trials_per_size": {
                "0.4x0.25x0.3": per_size[0],
                "0.55x0.35x0.45": per_size[1],
                "0.7x0.45x0.55": per_size[2]},
        }

    def test_full_matrix_passes(self):
        self.assertEqual(
            [], s.coverage_gate(self._coverage()))

    def test_missing_size_fails(self):
        failures = s.coverage_gate(self._coverage(n_sizes=2))
        self.assertTrue(any("size coverage" in f for f in failures))

    def test_missing_yaw_fails(self):
        failures = s.coverage_gate(self._coverage(n_yaws=2))
        self.assertTrue(any("yaw coverage" in f for f in failures))

    def test_thin_size_fails(self):
        failures = s.coverage_gate(
            self._coverage(per_size=(10, 10, 7)))
        self.assertTrue(any("trials per size" in f for f in failures))

    def test_too_few_trials_fails(self):
        failures = s.coverage_gate(self._coverage(trials=29))
        self.assertTrue(any("trials 29 < 30" in f for f in failures))


class TestMetricIndependence(unittest.TestCase):

    def test_bad_support_independently_fails(self):
        rows = ([_row()] * 5
                + [_row(err_support_m=0.05)] * 20)
        summary = s.gate_pass(s.aggregate([_trial(rows)]))
        self.assertFalse(summary["gate4_pass"])
        self.assertTrue(any("support_z_err_m" in f
                            for f in summary["gate4_failures"]))

    def test_bad_width_independently_fails(self):
        rows = ([_row()] * 5
                + [_row(err_width_m=0.2)] * 20)
        summary = s.gate_pass(s.aggregate([_trial(rows)]))
        self.assertFalse(summary["gate4_pass"])
        self.assertTrue(any("width_err_m" in f and "no samples" not in f
                            for f in summary["gate4_failures"]))

    def test_bad_depth_independently_fails(self):
        rows = ([_row()] * 5
                + [_row(err_depth_m=0.2)] * 20)
        summary = s.gate_pass(s.aggregate([_trial(rows)]))
        self.assertFalse(summary["gate4_pass"])
        self.assertTrue(any("depth_err_m" in f and "no samples" not in f
                            for f in summary["gate4_failures"]))

    def test_clean_run_passes(self):
        rows = [_row()] * 25
        summary = s.gate_pass(s.aggregate([_trial(rows)]))
        self.assertTrue(summary["gate4_pass"], summary["gate4_failures"])

    def test_false_measured_height_fails(self):
        rows = ([_row()] * 5
                + [_row(), _row(false_measured_height=True)] * 10)
        summary = s.gate_pass(s.aggregate([_trial(rows)]))
        self.assertFalse(summary["gate4_pass"])
        self.assertTrue(any("false_measured_height" in f
                            for f in summary["gate4_failures"]))


class TestCategoriesReportedSeparately(unittest.TestCase):

    def test_top_only_prior_failed_never_merge(self):
        rows = ([_row()] * 5 + [
            _row(),                                              # full3d
            _row(height_valid=False, geometry_level=0,
                 height_source=0),                               # top_only
            _row(height_valid=False, geometry_level=0,
                 height_source=3),                               # prior
            _row(top_surface_valid=False, height_valid=False,
                 geometry_level=0, height_source=0),             # failed
        ] * 5)
        summary = s.aggregate([_trial(rows)])
        cat = summary["categories"]
        self.assertEqual(cat["full3d"], 5)
        self.assertEqual(cat["top_only"], 5)
        self.assertEqual(cat["prior"], 5)
        self.assertEqual(cat["failed"], 5)
        self.assertEqual(cat["settled_frames"], 20)


class TestActiveWindowHz(unittest.TestCase):

    def test_spawn_gaps_do_not_reduce_active_hz(self):
        # 10 frames at 5 Hz for three active windows, separated by 20 s
        # orchestration gaps. Interval-based: 27 intervals / 5.4 s = 5 Hz
        # exactly (frame-count/span would report ~5.56 Hz).
        stamps = []
        t0 = 100.0
        for _window in range(3):
            stamps.extend(t0 + i * 0.2 for i in range(10))
            t0 += 20.0
        hz = s.active_window_hz(stamps, gap_sec=2.0)
        self.assertIsNotNone(hz)
        self.assertAlmostEqual(hz, 5.0, delta=0.05)

    def test_contiguous_stream_hz_exact(self):
        stamps = [100.0 + i * 0.25 for i in range(40)]
        hz = s.active_window_hz(stamps, gap_sec=2.0)
        self.assertAlmostEqual(hz, 4.0, delta=0.01)

    def test_detector_stall_inside_window_lowers_hz(self):
        """A 1.5 s stall inside one window is a detector stall, not
        orchestration (below gap threshold): it must lower the rate.
        20 frames, one window: 19 intervals over 6.0 s = 3.17 Hz."""
        stamps = [100.0 + i * 0.25 for i in range(10)]
        stamps += [103.75 + i * 0.25 for i in range(10)]  # 1.5 s hole
        hz = s.active_window_hz(stamps, gap_sec=2.0)
        self.assertAlmostEqual(hz, 19.0 / 6.0, delta=0.05)

    def test_too_short_returns_none(self):
        self.assertIsNone(s.active_window_hz([100.0], gap_sec=2.0))
        self.assertIsNone(s.active_window_hz([], gap_sec=2.0))


if __name__ == "__main__":
    unittest.main()
