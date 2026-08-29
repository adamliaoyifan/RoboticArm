#!/usr/bin/env python3
"""Unit tests for Todo 2 sampling-driver rules. No ROS."""

import json
import os
import unittest

import numpy as np

from luggage_perception.eval.detection_gate_sampling import (
    FRESH_SEC,
    MASK_VIZ_RGB,
    annotate_overlay_boxes,
    apply_dump_timestamp_banners,
    build_aligned_dump,
    colorize_mask_rgb,
    depth_vis_uint8,
    dump_failure_bundle,
    failure_dump_stem,
    format_trial_line,
    is_gt_fallback,
    is_perception_estimate,
    pick_joined_stamp,
    project_box_observation,
    quat_xyzw_from_yaw,
    stamp_key_from_sec,
    summarize_trial_records,
    trial_failure_code,
    trial_has_perception_result,
    trial_should_dump_failure,
    wait_ready,
    write_png,
)


def _stable_status(stamp):
    return {
        "flags": {"geometry_ok": True},
        "motion_gate": {"state": "stable"},
        "primary_stamp": stamp,
    }


def _fallback_record():
    return {
        "detect_message": "gt fallback (perception unavailable)",
        "diag": (
            '{"reason": "DETECT_STALE_CLOUD", "source": "gt_fallback",'
            ' "success": true}'
        ),
        "gt": {
            "x": -1.0, "y": 0.0, "z": 1.0, "yaw": 0.0,
            "width": 0.70, "depth": 0.45, "height": 0.28,
        },
        "measured": {
            "x": -1.0, "y": 0.0, "z": 1.0, "yaw": 0.0,
            "width": 0.70, "depth": 0.45, "height": 0.28,
        },
        "result": {
            "ok": True, "iou": 1.0, "err_xy": 0.0, "err_z": 0.0,
            "err_xyz": 0.0, "err_width": 0.0, "err_depth": 0.0,
            "err_height": 0.0, "err_yaw": 0.0, "swapped": False,
            "reason": "ok", "near_square": False,
        },
    }


class TestGtFallback(unittest.TestCase):
    def test_message_token(self):
        self.assertTrue(is_gt_fallback("gt fallback (perception unavailable)"))
        self.assertFalse(is_gt_fallback("perception estimate (conf=1.00)"))

    def test_diag_source(self):
        self.assertTrue(is_gt_fallback(
            "perception estimate (conf=1.00)",
            {"source": "gt_fallback", "reason": "DETECT_STALE_CLOUD"}))

    def test_success_plus_gt_box_is_not_a_perception_estimate(self):
        self.assertFalse(is_perception_estimate(
            True, "gt fallback (perception unavailable)", True,
            '{"source": "gt_fallback", "reason": "DETECT_STALE_CLOUD"}'))

    def test_real_perception_still_counts(self):
        self.assertTrue(is_perception_estimate(
            True, "perception estimate (conf=0.91)", True,
            {"source": "perception", "reason": "ok"}))

    def test_fallback_record_excluded_from_summary(self):
        rec = _fallback_record()
        self.assertFalse(trial_has_perception_result(rec))
        self.assertEqual(
            trial_failure_code(rec), "DETECT_GT_FALLBACK:DETECT_STALE_CLOUD")
        summary = summarize_trial_records([rec] * 20)
        self.assertEqual(summary["n_trials"], 20)
        self.assertEqual(summary["n_with_result"], 0)
        self.assertEqual(summary["n"], 0)
        self.assertEqual(summary["n_gt_fallback"], 20)
        self.assertEqual(summary["pass_rate"], 0.0)

    def test_format_line_does_not_print_iou_for_fallback(self):
        line = format_trial_line(0, _fallback_record())
        self.assertIn("DETECT_GT_FALLBACK", line)
        self.assertNotIn("iou=", line)


class TestWaitFreshness(unittest.TestCase):
    def test_latched_status_without_new_cloud_is_not_ready(self):
        # Subscribe delivered a latch at t=0; wait starts later with no cloud.
        self.assertFalse(wait_ready(
            status_data=_stable_status(10.0),
            cloud_recv=None,
            stamp_at_start=10.0,
            wait_started=100.0,
            now=101.0,
            fresh_sec=FRESH_SEC,
        ))

    def test_status_timer_echo_same_stamp_is_not_ready(self):
        # 1 Hz status timer republishes the same primary_stamp; cloud is live.
        self.assertFalse(wait_ready(
            status_data=_stable_status(10.0),
            cloud_recv=100.5,
            stamp_at_start=10.0,
            wait_started=100.0,
            now=101.0,
        ))

    def test_cloud_from_before_wait_is_not_ready(self):
        self.assertFalse(wait_ready(
            status_data=_stable_status(10.2),
            cloud_recv=99.0,
            stamp_at_start=10.0,
            wait_started=100.0,
            now=101.0,
        ))

    def test_stale_cloud_older_than_fresh_window(self):
        self.assertFalse(wait_ready(
            status_data=_stable_status(10.2),
            cloud_recv=100.1,
            stamp_at_start=10.0,
            wait_started=100.0,
            now=103.0,
        ))

    def test_new_stamp_and_fresh_cloud_is_ready(self):
        self.assertTrue(wait_ready(
            status_data=_stable_status(10.2),
            cloud_recv=100.5,
            stamp_at_start=10.0,
            wait_started=100.0,
            now=101.0,
        ))

    def test_unstable_status_rejected(self):
        self.assertFalse(wait_ready(
            status_data={
                "flags": {"geometry_ok": False},
                "motion_gate": {"state": "stable"},
                "primary_stamp": 10.2,
            },
            cloud_recv=100.5,
            stamp_at_start=10.0,
            wait_started=100.0,
            now=101.0,
        ))


class TestSummarizeMix(unittest.TestCase):
    def test_only_perception_rows_enter_n_with_result(self):
        good = {
            "detect_message": "perception estimate (conf=1.00)",
            "diag": '{"source": "perception", "reason": "ok"}',
            "gt": {
                "x": -1.0, "y": 0.0, "z": 1.0, "yaw": 0.0,
                "width": 0.70, "depth": 0.45, "height": 0.28,
            },
            "measured": {
                "x": -1.0, "y": 0.0, "z": 1.0, "yaw": 0.0,
                "width": 0.70, "depth": 0.45, "height": 0.28,
            },
            "result": {"ok": True, "iou": 1.0, "err_xy": 0.0},
        }
        timeout = {"failure": "GEOMETRY_NOT_STABLE"}
        summary = summarize_trial_records([_fallback_record(), good, timeout])
        self.assertEqual(summary["n_trials"], 3)
        self.assertEqual(summary["n_with_result"], 1)
        self.assertEqual(summary["n"], 1)
        self.assertEqual(summary["n_gt_fallback"], 1)
        self.assertAlmostEqual(summary["pass_rate"], 1.0)


class TestFailureDump(unittest.TestCase):
    def test_passing_trial_is_not_dumped(self):
        rec = {
            "index": 3,
            "detect_message": "perception estimate (conf=1.00)",
            "diag": '{"source": "perception", "reason": "ok"}',
            "gt": {"x": 0, "y": 0, "z": 1, "yaw": 0,
                   "width": 0.7, "depth": 0.45, "height": 0.28},
            "measured": {"x": 0, "y": 0, "z": 1, "yaw": 0,
                         "width": 0.7, "depth": 0.45, "height": 0.28},
            "result": {"ok": True, "reason": "ok", "iou": 1.0, "err_xy": 0.0},
        }
        self.assertFalse(trial_should_dump_failure(rec))
        self.assertIsNone(dump_failure_bundle("/tmp", rec, images={}))

    def test_size_gate_fail_and_stale_are_dumped(self):
        size_fail = {
            "index": 4,
            "detect_message": "perception estimate (conf=1.00)",
            "diag": '{"source": "perception", "reason": "ok"}',
            "result": {"ok": False, "reason": "size", "iou": 0.7, "err_xy": 0.01},
        }
        self.assertTrue(trial_should_dump_failure(size_fail))
        self.assertEqual(failure_dump_stem(size_fail), "trial_04_gate_size")
        self.assertTrue(trial_should_dump_failure(_fallback_record()))

    def test_bundle_writes_json_and_png(self):
        import tempfile
        rec = {
            "index": 7,
            "failure": "GEOMETRY_NOT_STABLE",
            "spawn_id": "pickup_box_0008_gen",
        }
        rgb = np.zeros((2, 3, 3), dtype=np.uint8)
        rgb[0, 0] = (255, 0, 0)
        with tempfile.TemporaryDirectory() as tmp:
            dest = dump_failure_bundle(
                tmp, rec,
                images={"color": rgb},
                extras={"color": {"encoding": "rgb8"}},
            )
            self.assertTrue(dest.endswith("trial_07_GEOMETRY_NOT_STABLE"))
            self.assertTrue(os.path.isfile(os.path.join(dest, "trial.json")))
            self.assertTrue(os.path.isfile(os.path.join(dest, "color.png")))
            payload = json.loads(open(os.path.join(dest, "trial.json")).read())
            self.assertEqual(payload["spawn_id"], "pickup_box_0008_gen")
            self.assertEqual(payload["frame_meta"]["color"]["encoding"], "rgb8")
            write_png(os.path.join(tmp, "grey.png"),
                      np.zeros((2, 2), dtype=np.uint8))
            self.assertGreater(os.path.getsize(os.path.join(tmp, "grey.png")), 8)


class TestFrameJoin(unittest.TestCase):
    def test_pick_joined_stamp_picks_newest_complete_triplet(self):
        buffers = {
            "color": {(1, 0): "c1", (2, 0): "c2"},
            "depth": {(1, 0): "d1", (2, 0): "d2"},
            "overlay": {(1, 0): "o1"},
        }
        self.assertEqual(pick_joined_stamp(buffers), (1, 0))
        buffers["overlay"][(2, 0)] = "o2"
        self.assertEqual(pick_joined_stamp(buffers), (2, 0))

    def test_pick_joined_stamp_min_stamp_drops_old_triplet(self):
        buffers = {
            "color": {(1, 0): "c1", (2, 0): "c2"},
            "depth": {(1, 0): "d1", (2, 0): "d2"},
            "overlay": {(1, 0): "o1", (2, 0): "o2"},
        }
        self.assertEqual(pick_joined_stamp(buffers), (2, 0))
        self.assertEqual(
            pick_joined_stamp(buffers, min_stamp_sec=1.5), (2, 0))
        self.assertIsNone(pick_joined_stamp(buffers, min_stamp_sec=2.0))
        self.assertEqual(
            pick_joined_stamp(
                buffers, required=("color", "depth", "overlay", "cargo"),
                min_stamp_sec=1.0),
            None)
        buffers["cargo"] = {(2, 0): "g2"}
        self.assertEqual(
            pick_joined_stamp(
                buffers, required=("color", "depth", "overlay", "cargo"),
                min_stamp_sec=1.0),
            (2, 0))

    def test_pick_joined_stamp_missing_stream_is_none(self):
        self.assertIsNone(pick_joined_stamp({
            "color": {(1, 0): "c"},
            "depth": {(1, 0): "d"},
            "overlay": {},
        }))
        self.assertIsNone(pick_joined_stamp({}))

    def test_stamp_key_from_sec_roundtrip(self):
        self.assertEqual(stamp_key_from_sec(1.5), (1, 500000000))
        self.assertEqual(stamp_key_from_sec(2.0), (2, 0))

    def test_colorize_mask_cargo_is_red(self):
        labels = np.zeros((2, 3), dtype=np.uint8)
        labels[0, 1] = 2
        rgb = colorize_mask_rgb(labels)
        self.assertEqual(tuple(rgb[0, 1]), MASK_VIZ_RGB[2])
        self.assertEqual(tuple(rgb[0, 0]), MASK_VIZ_RGB[0])

    def test_aligned_dump_writes_depth_npy_and_colorized_mask(self):
        import tempfile
        rec = {"index": 8, "failure": "GEOMETRY_NOT_STABLE"}
        color = np.zeros((2, 2, 3), dtype=np.uint8)
        overlay = np.zeros((2, 2, 3), dtype=np.uint8)
        overlay[0, 0] = (0, 255, 0)
        depth_m = np.array([[0.0, 2.5], [1.25, 0.0]], dtype=np.float32)
        labels = np.zeros((2, 2), dtype=np.uint8)
        labels[1, 1] = 2
        images, arrays, extras = build_aligned_dump(
            color, depth_m, overlay, labels)
        self.assertEqual(tuple(images["mask"][1, 1]), MASK_VIZ_RGB[2])
        self.assertEqual(int(images["depth"][0, 1]), 255)
        self.assertIn("cargo", extras["mask_meaning"])
        with tempfile.TemporaryDirectory() as tmp:
            dest = dump_failure_bundle(
                tmp, rec, images=images, extras=extras, arrays=arrays)
            self.assertTrue(os.path.isfile(os.path.join(dest, "color.png")))
            self.assertTrue(os.path.isfile(os.path.join(dest, "depth.png")))
            self.assertTrue(os.path.isfile(os.path.join(dest, "overlay.png")))
            self.assertTrue(os.path.isfile(os.path.join(dest, "mask.png")))
            loaded = np.load(os.path.join(dest, "depth.npy"))
            np.testing.assert_allclose(loaded, depth_m)
            payload = json.loads(open(os.path.join(dest, "trial.json")).read())
            self.assertTrue(payload["frame_meta"]["mask_meaning"].startswith("HxW"))

    def test_dump_banners_require_matching_stamps(self):
        color = np.zeros((32, 64, 3), dtype=np.uint8)
        overlay = np.zeros((32, 64, 3), dtype=np.uint8)
        overlay[:, :] = (10, 20, 30)
        extras = {
            "aligned": True,
            "color": {"stamp": 12.5},
            "overlay": {"stamp": 12.5},
            "cargo": {"stamp": 12.5},
        }
        images, extras = apply_dump_timestamp_banners(
            {"color": color, "overlay": overlay}, extras,
            dump_stamp=12.7, infer_ms=40.0, detect_stamp=12.5)
        check = extras["stamp_check"]
        self.assertTrue(check["aligned"])
        self.assertTrue(check["same_hw"])
        self.assertTrue(check["cargo_matched"])
        self.assertTrue(check["detect_matched"])
        self.assertAlmostEqual(check["dump_lag_sec"], 0.2)
        self.assertTrue(extras["aligned"])
        extras_bad = {
            "aligned": True,
            "color": {"stamp": 12.5},
            "overlay": {"stamp": 13.0},
        }
        _, extras_bad = apply_dump_timestamp_banners(
            {"color": color, "overlay": overlay}, extras_bad, dump_stamp=13.1)
        self.assertFalse(extras_bad["aligned"])
        self.assertFalse(extras_bad["stamp_check"]["aligned"])

    def test_depth_vis_clips_and_nans(self):
        vis = depth_vis_uint8(np.array([[float("nan"), 10.0]], dtype=np.float32))
        self.assertEqual(int(vis[0, 0]), 0)
        self.assertEqual(int(vis[0, 1]), 255)


class TestOverlayBoxProjection(unittest.TestCase):
    INTRINSICS = (600.0, 600.0, 320.0, 240.0)

    def test_wider_measured_box_spans_more_pixels(self):
        gt = {
            "x": 0.0, "y": 0.0, "z": 2.0, "yaw": 0.0,
            "width": 0.40, "depth": 0.40, "height": 0.20,
        }
        measured = dict(gt)
        measured["width"] = 0.80
        identity = np.eye(3)
        zero = [0.0, 0.0, 0.0]
        proj_gt = project_box_observation(
            gt, identity, zero, self.INTRINSICS)
        proj_m = project_box_observation(
            measured, identity, zero, self.INTRINSICS)
        self.assertGreater(
            proj_m["span"]["width_px"], proj_gt["span"]["width_px"] * 1.8)
        np.testing.assert_allclose(
            proj_gt["centre_uv"], [320.0, 240.0], atol=1e-6)

    def test_yaw_quat_is_z_axis(self):
        q = quat_xyzw_from_yaw(0.0)
        np.testing.assert_allclose(q, [0.0, 0.0, 0.0, 1.0], atol=1e-9)

    def test_annotate_draws_green_gt_when_cv2_present(self):
        try:
            import cv2  # noqa: F401
        except ImportError:
            self.skipTest("cv2 not installed")
        overlay = np.zeros((480, 640, 3), dtype=np.uint8)
        gt = {
            "x": 0.0, "y": 0.0, "z": 2.0, "yaw": 0.0,
            "width": 0.74, "depth": 0.44, "height": 0.27,
        }
        out, meta = annotate_overlay_boxes(
            overlay, gt, None, np.eye(3), [0.0, 0.0, 0.0], self.INTRINSICS)
        self.assertTrue(meta["boxes_projected"])
        self.assertIsNotNone(meta["overlay_boxes"]["gt"]["centre_uv"])
        self.assertGreater(int((out[:, :, 1] > 200).sum()), 0)
        self.assertEqual(int(overlay.sum()), 0)

    def test_missing_camera_leaves_overlay_untouched(self):
        overlay = np.zeros((4, 4, 3), dtype=np.uint8)
        overlay[0, 0] = (9, 9, 9)
        out, meta = annotate_overlay_boxes(
            overlay, {"x": 0, "y": 0, "z": 1, "yaw": 0,
                      "width": 0.4, "depth": 0.4, "height": 0.2},
            None, None, None, None)
        self.assertFalse(meta["boxes_projected"])
        self.assertEqual(meta["project_error"], "no_camera")
        np.testing.assert_array_equal(out, overlay)


if __name__ == "__main__":
    unittest.main()
