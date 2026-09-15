#!/usr/bin/env python3
"""PF-R7 generation-5 scan classes and standard seed-matrix override. No ROS."""

from __future__ import division

import json
import os
import tempfile
import unittest

from luggage_perception.eval.pf_r7_campaign import (
    CampaignDriver,
    FakeClock,
    ScriptedBackend,
    default_seed_matrix,
    g4_imported_standard_rows,
    live_seed_matrix_from_available,
    merge_seed_matrix,
)
from luggage_perception.eval.pf_r7_classifier import (
    CLASS_ELIGIBLE_FAIL,
    CLASS_EVIDENCE,
    SCAN_EVIDENCE,
    SCAN_FIXTURE,
    SCAN_INFRA,
    SCAN_KNOWN_MISS_VALID,
    SCAN_PROPOSAL_AVAILABLE,
    scan_availability_class,
)

from pf_r7_fixtures import miss_record, pass_record


class TestScanAvailabilityClass(unittest.TestCase):
    def test_five_classes(self):
        self.assertEqual(
            scan_availability_class(pass_record())[0],
            SCAN_PROPOSAL_AVAILABLE)
        self.assertEqual(
            scan_availability_class(miss_record())[0],
            SCAN_KNOWN_MISS_VALID)
        self.assertEqual(
            scan_availability_class(miss_record(fixture_invalid=True))[0],
            SCAN_FIXTURE)
        self.assertEqual(
            scan_availability_class(miss_record(controller_ok=False))[0],
            SCAN_INFRA)
        self.assertEqual(
            scan_availability_class(pass_record(dump_health={
                "capture_complete": False,
                "replay_possible": False,
                "missing": ["dump"],
            }))[0],
            SCAN_EVIDENCE)

    def test_geometry_fail_after_proposal_is_available(self):
        record = pass_record(mask_join_failed=True)
        scan_class, classified = scan_availability_class(record)
        self.assertEqual(classified["attempt_class"], CLASS_ELIGIBLE_FAIL)
        self.assertEqual(scan_class, SCAN_PROPOSAL_AVAILABLE)

    def test_short_scan_window_with_proposal_is_available(self):
        record = pass_record(
            score_mode="support-window-ready",
            steady_start="support-window-ready",
            t_steady=9.142,
            steady_window={
                "t_steady": 9.142,
                "window_complete": False,
                "window_sec": 8.0,
            },
            n_settled=2,
            settled=[{
                "top_surface_valid": True,
                "height_valid": True,
                "geometry_level": 1,
                "stamp_sec": 9.2,
                "monotonic_sec": 9.2,
                "n_cargo_points": 8000,
            }] * 2,
            c2={
                "output_hz": 0.25,
                "executor_lag_q4_sec": 0.09,
                "executor_lag_q1_sec": 0.08,
                "rss_beta": {"luggage_detector": 1.2},
                "buffer_unbounded": False,
                "residual_processes": 0,
            },
        )
        scan_class, classified = scan_availability_class(record)
        self.assertEqual(classified["attempt_class"], CLASS_EVIDENCE)
        self.assertEqual(scan_class, SCAN_PROPOSAL_AVAILABLE)


class TestSeedMatrixOverride(unittest.TestCase):
    def test_standard_override_keeps_carryon_large(self):
        base = default_seed_matrix()
        six = list(base["standard"][:6])
        merged = merge_seed_matrix({"standard": six})
        self.assertEqual([s["seed_id"] for s in merged["standard"]],
                         [s["seed_id"] for s in six])
        self.assertEqual(merged["carryon"], base["carryon"])
        self.assertEqual(merged["large"], base["large"])
        self.assertEqual(len(base["standard"]), 16)

    def test_live_matrix_from_available(self):
        available = g4_imported_standard_rows()[:3]
        available.extend([
            {"seed_id": "standard_06", "xy": [0.04, -0.04], "yaw": -0.1},
            {"seed_id": "standard_07", "xy": [-0.04, 0.04], "yaw": 0.3},
            {"seed_id": "standard_08", "xy": [0.04, 0.04], "yaw": -0.3},
        ])
        matrix = live_seed_matrix_from_available(available)
        self.assertEqual(
            [s["seed_id"] for s in matrix["standard"]],
            ["standard_00", "standard_01", "standard_02",
             "standard_06", "standard_07", "standard_08"])
        self.assertEqual(matrix["carryon"], default_seed_matrix()["carryon"])
        self.assertEqual(matrix["large"], default_seed_matrix()["large"])


class TestStandardSeedScan(unittest.TestCase):
    def _run(self, records, imported=None, **kwargs):
        tmp = tempfile.mkdtemp(prefix="pfr7g5_")
        backend = ScriptedBackend(records_by_size={"standard": records})
        driver = CampaignDriver(
            out_dir=tmp,
            backend=backend,
            seed_matrix=default_seed_matrix(),
            clock=FakeClock(),
            poll_dt=1.0,
            git_commit="test",
            install_signals=False,
        )
        verdict = driver.run_standard_scan(
            imported=imported if imported is not None else g4_imported_standard_rows(),
            scan_start=kwargs.get("scan_start", "standard_06"),
            stop_available=kwargs.get("stop_available", 6),
            max_scan=kwargs.get("max_scan", 24),
        )
        return driver, verdict, tmp, backend

    def test_stop_at_six_including_imported(self):
        driver, verdict, tmp, backend = self._run(
            [pass_record() for _ in range(10)])
        self.assertEqual(verdict["outcome"], "pass", verdict)
        self.assertEqual(verdict["n_available"], 6)
        self.assertEqual(verdict["n_scanned"], 3)
        self.assertEqual(
            verdict["available_seed_ids"],
            ["standard_00", "standard_01", "standard_02",
             "standard_06", "standard_07", "standard_08"])
        self.assertEqual(
            [case["seed_id"] for case in backend.spawned],
            ["standard_06", "standard_07", "standard_08"])
        live = json.loads(open(
            os.path.join(tmp, "live_seed_matrix.json"), encoding="utf-8").read())
        self.assertEqual(len(live["standard"]), 6)
        self.assertEqual(live["carryon"], default_seed_matrix()["carryon"])

    def test_blocked_when_inventory_exhausted(self):
        driver, verdict, tmp, backend = self._run(
            [miss_record() for _ in range(10)])
        self.assertEqual(verdict["outcome"], "inconclusive", verdict)
        self.assertEqual(
            verdict["reason"],
            "inconclusive/perception_availability_blocked")
        self.assertEqual(verdict["n_available"], 3)
        self.assertEqual(verdict["n_scanned"], 10)

    def test_evidence_invalid_stops(self):
        broken = pass_record(dump_health={
            "capture_complete": False,
            "replay_possible": False,
            "missing": ["dump"],
        })
        driver, verdict, tmp, backend = self._run(
            [broken, pass_record(), pass_record()])
        self.assertEqual(verdict["outcome"], "fail", verdict)
        self.assertEqual(verdict["reason"], "evidence_invalid")
        self.assertEqual(verdict["n_scanned"], 1)
        self.assertEqual(verdict["defect_seed"], "standard_06")
        self.assertEqual(len(backend.spawned), 1)


if __name__ == "__main__":
    unittest.main()
