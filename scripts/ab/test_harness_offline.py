#!/usr/bin/env python3
"""Offline tests for the frozen harness pieces (user-mandated before the
nine runs): metric computation (init/timing/primary-at-38-sim-s/stop
detection/failure dump), the frozen-input bundle integrity, and the
run-order registration consistency.
"""
import json
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from metrics import compute_metrics, T_PRIMARY_S  # noqa: E402


def _samples(points):
    """points: [(t_rel_s, x, y, z)] -> parsed-sample tuples."""
    out = []
    for t, x, y, z in points:
        out.append((int(t * 1e9), x, y, z, (0.0, 0.0, 0.0, 1.0)))
    return out


class TestMetricComputation(unittest.TestCase):

    def test_primary_at_38_sim_seconds_not_wall(self):
        pts = [(0.0, 0.0, 0.0, 0.6), (19.0, 0.010, 0.0, 0.6),
               (38.0, 0.020, 0.0, 0.6), (44.9, 0.030, 0.0, 0.6)]
        m = compute_metrics(_samples(pts), [], "box", 0.0)
        self.assertTrue(m["valid"])
        self.assertAlmostEqual(
            m["primary_xy_offset_at_38sim_m"], 0.020, places=6)
        # The 44.9 s sample (past the primary) must NOT leak in.
        self.assertAlmostEqual(m["max_xy_offset_m"], 0.030, places=6)

    def test_no_samples_is_failure_not_crash(self):
        m = compute_metrics([], [], "box", 0.0)
        self.assertFalse(m["valid"])
        self.assertEqual(m["reason"], "no_dynamic_samples")

    def test_stop_time_detection(self):
        pts = [(0.0, 0.0, 0.0, 0.6), (10.0, 0.010, 0.0, 0.6),
               (16.0, 0.016, 0.0, 0.6), (16.1, 0.016, 0.0, 0.6),
               (17.5, 0.016, 0.0, 0.6), (40.0, 0.016, 0.0, 0.6)]
        m = compute_metrics(_samples(pts), [], "box", 0.0)
        self.assertIsNotNone(m["stop_time_s"])
        self.assertLessEqual(m["stop_time_s"], 17.5)

    def test_contact_partner_filtering(self):
        contacts = [(1_000_000_000, ["pickup_box_0001_inch20::a",
                                     "airport_container_real::b"])] * 10 \
            + [(2_000_000_000, ["S20::arm"])] * 2
        m = compute_metrics(
            _samples([(0.0, 0, 0, 0.6), (5.0, 0.0, 0, 0.6)]),
            contacts, "pickup_box_0001_inch20", 0.0)
        self.assertIn("airport_container_real", m["contact_partners"])
        self.assertNotIn("S20", m["contact_partners"])
        self.assertNotIn("pickup_box_0001_inch20", m["contact_partners"])


class TestFrozenBundle(unittest.TestCase):

    def test_bundle_files_and_hashes(self):
        root = os.path.normpath(os.path.join(
            _HERE, "..", "..", "src", "luggage_gazebo", "models"))
        manifest = json.load(open(os.path.join(
            root, "ab_generation_manifest.json")))
        for key in ("baseline_yaml_sha256", "a0_stl_sha256"):
            self.assertTrue(manifest.get(key))
        self.assertTrue(manifest["aprime"]["stl_sha256"])
        self.assertTrue(manifest["bprim"]["sdf_sha256"])
        # The frozen input pieces exist in the observation run.
        obs = ("/home/adamliao/work/elfin_humble_ws/docs/status/"
               "evidence/master_pack_recovery/2026-10-14_g3/obs_rel_07")
        for name in ("tool_stream.jsonl", "sdf/release.sdf",
                     "dumps/box_00_ok/trial.json"):
            self.assertTrue(os.path.isfile(os.path.join(obs, name)), name)

    def test_run_order_registration(self):
        reg = os.path.normpath(os.path.join(
            _HERE, "..", "..", "docs", "status", "evidence",
            "occ_owner_b1", "AB_RUN_ORDER.json"))
        order = json.load(open(reg))
        self.assertEqual(len(order["runs"]), 9)
        self.assertEqual(
            [r["arm"] for r in order["runs"]],
            ["A0", "APRIME", "B"] * 3)
        self.assertEqual(order["primary_metric"]["t_sim_s"],
                         T_PRIMARY_S)


if __name__ == "__main__":
    unittest.main()
