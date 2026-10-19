#!/usr/bin/env python3
"""Offline tests for the frozen harness pieces (user-mandated before the
nine runs): metric computation (validity gate, 38-sim-s interpolation,
stop detection, failure dump), the frozen-input bundle integrity, and
the run-order registration consistency.
"""
import json
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from metrics import compute_metrics, T_PRIMARY_S  # noqa: E402


def _samples(points):
    out = []
    for t, x, y, z in points:
        out.append((int(t * 1e9), x, y, z, (0.0, 0.0, 0.0, 1.0)))
    return out


def _ramp(t_end=44.9, dt=0.2, speed=0.001, t_stop=None):
    """Dense valid stream: slides at `speed` until t_stop then static."""
    pts = []
    x = 0.0
    t = 0.0
    while t <= t_end:
        pts.append((t, x, 0.0, 0.6))
        if t_stop is None or t < t_stop:
            x += speed * dt
        t += dt
    return pts


class TestMetricComputation(unittest.TestCase):

    def test_primary_at_38_sim_seconds_not_wall(self):
        # Slide 1 mm/s for 30 s then hold: primary must read 30 mm.
        pts = _ramp(t_end=44.9, dt=0.2, speed=0.001, t_stop=30.0)
        m = compute_metrics(_samples(pts), [], "box", 0.0)
        self.assertTrue(m["valid"], m)
        self.assertAlmostEqual(
            m["primary_xy_offset_at_38sim_m"], 0.030, places=3)
        self.assertAlmostEqual(m["max_xy_offset_m"], 0.030, places=3)

    def test_primary_interpolated_between_brackets(self):
        # Offsets chosen so linear interpolation at t=38 between the
        # bracketing samples (36.9, 38.1) gives exactly 0.038.
        pts = _ramp(t_end=44.9, dt=44.9 / 100.0, speed=0.001)
        # dt ~0.449 < MAX_GAP; offset(38) ~ 38*speed_total...
        m = compute_metrics(_samples(pts), [], "box", 0.0)
        self.assertTrue(m["valid"], m)
        self.assertGreaterEqual(m["primary_xy_offset_at_38sim_m"], 0.0)
        # Interpolation vs nearest-frame differ by < one step's motion.
        step_motion = 0.001 * 44.9 / 100.0
        nearest = 38.0 * 0.001  # continuous ramp value
        self.assertLessEqual(
            abs(m["primary_xy_offset_at_38sim_m"] - nearest),
            step_motion + 1e-9)

    def test_validity_gate_span_short(self):
        pts = _ramp(t_end=30.0, dt=0.2, speed=0.0)
        m = compute_metrics(_samples(pts), [], "box", 0.0)
        self.assertFalse(m["valid"])
        self.assertEqual(m["reason"], "span_short")

    def test_delivery_reorder_normalized_with_guard(self):
        pts = _ramp(t_end=44.9, dt=0.2, speed=0.001, t_stop=30.0)
        swapped = list(pts)
        swapped[100], swapped[101] = swapped[101], swapped[100]
        m = compute_metrics(_samples(swapped), [], "box", 0.0)
        self.assertTrue(m["valid"], m)
        self.assertTrue(m["delivery_reorder"]["sorted"])

    def test_real_non_monotone_still_rejected(self):
        pts = _ramp(t_end=44.9, dt=0.2, speed=0.0)
        bad = pts[:200] + [(pts[200][0] - 5.0, 0.0, 0.0, 0.6)] + pts[200:]
        m = compute_metrics(_samples(bad), [], "box", 0.0)
        self.assertFalse(m["valid"])
        self.assertEqual(m["reason"], "time_not_monotone")

    def test_validity_gate_sample_gap(self):
        pts = [(float(i) * 5.0, 0.0, 0.0, 0.6) for i in range(10)]
        m = compute_metrics(_samples(pts), [], "box", 0.0)
        self.assertFalse(m["valid"])
        self.assertEqual(m["reason"], "sample_gap")

    def test_non_monotone_rejected(self):
        # 1 s backward at ONE point = artifact; MANY points = anomaly.
        pts = _ramp(t_end=44.9, dt=0.2, speed=0.0)
        bad = pts[:100] + [(pts[100][0] - 1.0, 0.0, 0.0, 0.6)] + pts[100:]
        m = compute_metrics(_samples(bad), [], "box", 0.0)
        self.assertFalse(m["valid"])

    def test_no_samples_is_failure_not_crash(self):
        m = compute_metrics([], [], "box", 0.0)
        self.assertFalse(m["valid"])
        self.assertEqual(m["reason"], "no_dynamic_samples")

    def test_stop_time_detection(self):
        pts = _ramp(t_end=44.9, dt=0.2, speed=0.001, t_stop=16.0)
        m = compute_metrics(_samples(pts), [], "box", 0.0)
        self.assertTrue(m["valid"], m)
        self.assertIsNotNone(m["stop_time_s"])
        self.assertGreaterEqual(m["stop_time_s"], 16.0)

    def test_contact_partner_filtering(self):
        contacts = [(1_000_000_000, ["pickup_box_0001_inch20::a",
                                     "airport_container_real::b"])] * 10 \
            + [(2_000_000_000, ["S20::arm"])] * 2
        m = compute_metrics(
            _samples(_ramp(t_end=44.9, dt=0.2, speed=0.0)),
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

    def test_retreat_full_collision_pass(self):
        root = os.path.normpath(os.path.join(
            _HERE, "..", "..", "src", "luggage_gazebo", "models"))
        rep = json.load(open(os.path.join(root,
                                         "ab_full_collision.json")))
        self.assertTrue(rep["pass"], rep["min_distance_m"])
        self.assertGreater(len(rep["links_with_geometry"]), 5)


if __name__ == "__main__":
    unittest.main()
