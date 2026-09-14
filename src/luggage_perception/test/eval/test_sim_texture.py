#!/usr/bin/env python3
"""PF-R10 generation-6 eval-only texture / spawn-flip / C1-G6 fixtures."""

from __future__ import division

import os
import tempfile
import unittest

from luggage_perception.eval.sim_texture import (
    CLASS_FAIL,
    CLASS_INFRA,
    CLASS_NORMAL_PASS,
    CLASS_TEXTURE,
    aabb_wholly_inside,
    c1_g6_gate,
    classify_texture_waiver,
    classify_trial,
    dump_capture_health,
    spawn_is_flip,
    write_dump_manifest,
)


def _det(bbox, conf, accepted=False, reason="cargo_below_min_conf"):
    return {
        "label": 2,
        "prompt": "suitcase",
        "confidence": conf,
        "bbox": list(bbox),
        "accepted": bool(accepted),
        "accept_reason": reason,
    }


def _ransac_ok():
    return {
        "ok": True,
        "plane_z": 1.160,
        "center_xy": [-1.00, 0.00],
        "width": 0.55,
        "depth": 0.35,
    }


def _gt():
    return {
        "gt_top_z": 1.155,
        "gt_xy": [-1.00, 0.00],
        "gt_width": 0.55,
        "gt_depth": 0.36,
        "gt_height": 0.22,
    }


def _health_ok():
    return {"capture_complete": True, "replay_possible": True, "missing": []}


def _recovery_ok():
    return {
        "spawn_ok": True,
        "n_settled": 40,
        "t_first_valid_sec": 0.4,
        "t_first_full3d_sec": 0.5,
    }


def _settled_ok(n=40):
    return [{
        "top_surface_valid": True,
        "height_valid": True,
        "geometry_level": 1,
        "height_source": 1,
        "err_top_m": 0.004,
        "err_support_m": 0.004,
        "err_height_m": 0.006,
        "err_xy_m": 0.008,
        "err_width_m": 0.010,
        "err_depth_m": 0.009,
        "false_measured_height": False,
        "stamp_sec": 10.0 + 0.05 * i,
        "n_cargo_points": 8000,
    } for i in range(n)]


def _texture_record(**over):
    record = {
        "visual_kind": "mesh",
        "gt_bbox": [170, 105, 419, 297],
        "image_wh": (640, 480),
        "detections": [_det([168, 100, 420, 300], 0.070)],
        "raw_depth_ransac": _ransac_ok(),
        "gt": _gt(),
        "n_accepted_cargo": 0,
        "n_cargo_points": 0,
        "dump_health": _health_ok(),
        "recovery": {
            "spawn_ok": True,
            "n_settled": 40,
            "t_first_valid_sec": None,
            "t_first_full3d_sec": None,
        },
        "settled": [{
            "top_surface_valid": False,
            "false_measured_height": False,
            "n_cargo_points": 0,
        }] * 40,
        "edge_fp_accepted": False,
    }
    record.update(over)
    return record


class TestSpawnFlip(unittest.TestCase):
    def test_tilt_is_infrastructure(self):
        self.assertTrue(spawn_is_flip(
            "PLACE_VERIFY_FAILED: tilt=0.21 model=pickup_box_0003_large"))
        classified = classify_trial({
            "spawn_message": "PLACE_VERIFY_FAILED: tilt=0.21 model=x_large",
            "dump_health": _health_ok(),
        })
        self.assertEqual(classified["trial_class"], CLASS_INFRA)

    def test_xy_mismatch_is_not_a_flip(self):
        self.assertFalse(spawn_is_flip(
            "PLACE_VERIFY_FAILED: xy=0.12 model=pickup_box_0001_carryon"))

    def test_flip_is_neither_pass_nor_waiver(self):
        classified = classify_trial(_texture_record(
            spawn_message="PLACE_VERIFY_FAILED: tilt=0.30 model=box",
            infrastructure_invalid=True))
        self.assertEqual(classified["trial_class"], CLASS_INFRA)
        self.assertNotEqual(classified["trial_class"], CLASS_TEXTURE)
        self.assertNotEqual(classified["trial_class"], CLASS_NORMAL_PASS)


class TestTextureWaiver(unittest.TestCase):
    def test_in_frame_low_conf_gt_match_waives(self):
        ok, proof = classify_texture_waiver(_texture_record())
        self.assertTrue(ok, proof)
        classified = classify_trial(_texture_record())
        self.assertEqual(classified["trial_class"], CLASS_TEXTURE)

    def test_clipped_gt_bbox_does_not_waive(self):
        ok, proof = classify_texture_waiver(_texture_record(
            gt_bbox=[-10, 40, 200, 300]))
        self.assertFalse(ok)
        self.assertIn("gt_bbox_not_in_frame", proof["missing"])
        self.assertEqual(classify_trial(_texture_record(
            gt_bbox=[-10, 40, 200, 300]))["trial_class"], CLASS_FAIL)

    def test_no_iou_match_does_not_waive(self):
        ok, proof = classify_texture_waiver(_texture_record(
            detections=[_det([500, 20, 630, 80], 0.09)]))
        self.assertFalse(ok)
        self.assertTrue(
            any("iou" in item for item in proof["missing"])
            or "no_cargo_proposal" in proof["missing"])

    def test_raw_depth_geometry_fail_does_not_waive(self):
        ok, proof = classify_texture_waiver(_texture_record(
            raw_depth_ransac={"ok": False}))
        self.assertFalse(ok)
        self.assertTrue(any("raw_depth" in item for item in proof["missing"]))

    def test_accepted_edge_strip_cannot_waive(self):
        classified = classify_trial(_texture_record(
            detections=[
                _det([170, 105, 419, 297], 0.07),
                _det([600, 20, 640, 200], 0.227, accepted=True,
                     reason="predicate_unavailable"),
            ],
            n_accepted_cargo=1,
            n_cargo_points=1200,
            edge_fp_accepted=True,
        ))
        self.assertEqual(classified["trial_class"], CLASS_FAIL)
        self.assertIn("edge_fp_accepted", classified["reasons"])

    def test_eval_only_low_conf_match_waives_when_live_box_is_edge_strip(self):
        classified = classify_trial(_texture_record(
            detections=[
                _det([607, 115, 640, 300], 0.218, accepted=False,
                     reason="outside_workspace"),
                _det([168, 100, 420, 300], 0.070, accepted=False,
                     reason="eval_low_conf"),
            ],
        ))
        self.assertEqual(classified["trial_class"], CLASS_TEXTURE)


class TestDumpHealth(unittest.TestCase):
    def test_missing_late_snapshot_is_incomplete(self):
        with tempfile.TemporaryDirectory() as tmp:
            health = dump_capture_health(tmp)
            self.assertFalse(health["capture_complete"])
            self.assertIn("late", health["missing"])

    def test_complete_layout(self):
        with tempfile.TemporaryDirectory() as tmp:
            for name in ("trial.json", "scores.jsonl", "gz_pose.json"):
                open(os.path.join(tmp, name), "w").write("{}\n")
            for label in ("early", "mid", "late"):
                snap = os.path.join(tmp, label)
                os.makedirs(os.path.join(snap, "pca_replay"))
                for fname in ("color.png", "overlay.png", "depth.npy",
                              "mask.png", "meta.json",
                              "cargo_camera.ply", "depth_all.ply",
                              "mask_cargo.ply"):
                    open(os.path.join(snap, fname), "wb").write(b"x")
                open(os.path.join(snap, "pca_replay", "pca_replay.json"),
                     "w").write("{}\n")
            write_dump_manifest(tmp, {"trial": 0})
            health = dump_capture_health(tmp)
            self.assertTrue(health["capture_complete"], health)
            self.assertTrue(health["replay_possible"], health)


class TestC1G6Gate(unittest.TestCase):
    def _pass_trial(self, trial, size):
        return {
            "trial": trial,
            "box_id": "pickup_box_%04d_%s" % (trial + 1, size),
            "size": size,
            "trial_class": CLASS_NORMAL_PASS,
            "reasons": [],
            "recovery": _recovery_ok(),
            "warmup": [],
            "settled": _settled_ok(),
        }

    def _waive_trial(self, trial, size):
        rec = _texture_record()
        rec.update({
            "trial": trial,
            "box_id": "pickup_box_%04d_%s" % (trial + 1, size),
            "size": size,
            "trial_class": CLASS_TEXTURE,
            "reasons": [],
            "warmup": [],
        })
        return rec

    def test_four_of_six_with_size_mix_and_two_waivers_passes(self):
        trials = [
            self._pass_trial(0, "carryon"),
            self._pass_trial(1, "standard"),
            self._pass_trial(2, "large"),
            self._waive_trial(3, "carryon"),
            self._pass_trial(4, "standard"),
            self._waive_trial(5, "large"),
        ]
        summary = c1_g6_gate(trials, active_output_hz=12.0)
        self.assertTrue(summary["c1_g6_pass"], summary["c1_g6_failures"])
        self.assertEqual(summary["n_normal_pass"], 4)
        self.assertEqual(summary["n_texture_waiver"], 2)

    def test_three_waivers_fail(self):
        trials = [
            self._pass_trial(0, "carryon"),
            self._pass_trial(1, "standard"),
            self._pass_trial(2, "large"),
            self._waive_trial(3, "carryon"),
            self._waive_trial(4, "standard"),
            self._waive_trial(5, "large"),
        ]
        summary = c1_g6_gate(trials, active_output_hz=12.0)
        self.assertFalse(summary["c1_g6_pass"])
        self.assertTrue(any("texture_waivers" in item
                            for item in summary["c1_g6_failures"]))

    def test_missing_size_in_normal_pass_fails(self):
        trials = [
            self._pass_trial(0, "carryon"),
            self._pass_trial(1, "carryon"),
            self._pass_trial(2, "carryon"),
            self._pass_trial(3, "standard"),
            self._waive_trial(4, "large"),
            self._waive_trial(5, "large"),
        ]
        summary = c1_g6_gate(trials, active_output_hz=12.0)
        self.assertFalse(summary["c1_g6_pass"])
        self.assertTrue(any("missing sizes" in item
                            for item in summary["c1_g6_failures"]))

    def test_spawn_flip_does_not_fill_a_scored_slot(self):
        trials = [
            {"trial": 99, "box_id": "flip", "trial_class": CLASS_INFRA,
             "reasons": ["spawn_flip"], "warmup": [], "settled": [],
             "recovery": {"spawn_ok": False}},
            self._pass_trial(0, "carryon"),
            self._pass_trial(1, "standard"),
            self._pass_trial(2, "large"),
            self._pass_trial(3, "carryon"),
            self._pass_trial(4, "standard"),
            self._pass_trial(5, "large"),
        ]
        summary = c1_g6_gate(trials, active_output_hz=12.0)
        self.assertEqual(summary["n_scored"], 6)
        self.assertEqual(summary["n_infrastructure_invalid"], 1)
        self.assertTrue(summary["c1_g6_pass"], summary["c1_g6_failures"])


class TestAabb(unittest.TestCase):
    def test_plan_bboxes_are_inside(self):
        self.assertTrue(aabb_wholly_inside([170, 105, 419, 297]))
        self.assertTrue(aabb_wholly_inside([92, 45, 494, 405]))
        self.assertFalse(aabb_wholly_inside([-1, 0, 10, 10]))


if __name__ == "__main__":
    unittest.main()
