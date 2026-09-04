#!/usr/bin/env python3
"""Unit tests for pick/retreat eval_metrics (no ROS)."""

from __future__ import division

import unittest

from luggage_gazebo.eval_metrics import (
    OBSERVE_LOOP_PHASES,
    DEFAULT_VISUAL_TOL_XY,
    TrialRecord, cargo_generation_ready, label_aabb, nearest_catalog_id,
    points_aabb, raised_object_measure, raised_object_size,
    spawn_visual_matches_gt, summarize,
    tracker_epoch_matches, tracker_wait_fail_code, trial_from_dict,
    trial_to_dict, yolo_boxes_ready,
)


def _trial(**kwargs):
    defaults = dict(index=0)
    defaults.update(kwargs)
    return TrialRecord(**defaults)


class TestEvalMetrics(unittest.TestCase):

    def test_all_success_rates_are_one(self):
        records = [
            _trial(
                index=i, catalog_id="standard", visual_id="vintage",
                accuracy_ok=True, accuracy_reason="ok",
                segments_planned=4, segments_succeeded=4,
                retreat_ok=True, retreat_delta_z=0.35,
                attach_xy_err=0.01, attach_z_err=0.005,
            )
            for i in range(4)
        ]
        out = summarize(records)
        self.assertEqual(out["n"], 4)
        self.assertEqual(out["detect_pass_rate"], 1.0)
        self.assertEqual(out["plan_pass_rate"], 1.0)
        self.assertEqual(out["retreat_pass_rate"], 1.0)
        self.assertEqual(out["pick_pass_rate"], 1.0)

    def test_detect_fail_excluded_from_plan_denominator(self):
        records = [
            _trial(index=0, accuracy_ok=False, accuracy_reason="size",
                   detect_failure="", fail_code="DETECT_GATE"),
            _trial(index=1, accuracy_ok=True, accuracy_reason="ok",
                   segments_planned=4, segments_succeeded=4,
                   retreat_ok=True, retreat_delta_z=0.34),
        ]
        out = summarize(records)
        self.assertEqual(out["n_detect_compared"], 2)
        self.assertEqual(out["detect_pass_rate"], 0.5)
        self.assertEqual(out["n_detect_ok"], 1)
        self.assertEqual(out["plan_pass_rate"], 1.0)
        self.assertEqual(out["pick_pass_rate"], 0.5)

    def test_fail_code_histogram(self):
        records = [
            _trial(index=0, fail_code="DETECT_NO_CLOUD"),
            _trial(index=1, fail_code="DETECT_NO_CLOUD"),
            _trial(index=2, fail_code="PLAN_pre_grasp"),
        ]
        out = summarize(records)
        self.assertEqual(out["fail_codes"]["DETECT_NO_CLOUD"], 2)
        self.assertEqual(out["fail_codes"]["PLAN_pre_grasp"], 1)

    def test_empty_does_not_divide_by_zero(self):
        out = summarize([])
        self.assertEqual(out["n"], 0)
        self.assertEqual(out["detect_pass_rate"], 0.0)
        self.assertEqual(out["plan_pass_rate"], 0.0)
        self.assertEqual(out["retreat_pass_rate"], 0.0)
        self.assertIsNone(out["attach_xy"]["mean"])
        self.assertIsNone(out["attach_xy"]["std"])

    def test_partial_run_still_summarizes(self):
        records = [
            _trial(index=0, accuracy_ok=True, segments_planned=4,
                   segments_succeeded=1,
                   segment_failures=(("approach", "timeout"),),
                   fail_code="PLAN_approach"),
        ]
        out = summarize(records)
        self.assertEqual(out["plan_pass_rate"], 0.0)
        self.assertEqual(out["segment_failures"]["approach"], 1)

    def test_attach_stability_std(self):
        records = [
            _trial(index=0, attach_xy_err=0.02, attach_z_err=0.01),
            _trial(index=1, attach_xy_err=0.04, attach_z_err=-0.01),
        ]
        out = summarize(records)
        self.assertAlmostEqual(out["attach_xy"]["mean"], 0.03)
        self.assertGreater(out["attach_xy"]["std"], 0.0)
        self.assertAlmostEqual(out["attach_z_abs"]["mean"], 0.01)

    def test_by_catalog_counts(self):
        records = [
            _trial(index=0, catalog_id="carryon", accuracy_ok=True,
                   segments_planned=4, segments_succeeded=4, retreat_ok=True),
            _trial(index=1, catalog_id="large", accuracy_ok=False),
        ]
        out = summarize(records)
        self.assertEqual(out["by_catalog"]["carryon"]["retreat_ok"], 1)
        self.assertEqual(out["by_catalog"]["large"]["detect_ok"], 0)

    def test_json_round_trip(self):
        rec = _trial(
            index=3, catalog_id="standard", visual_id="vintage",
            accuracy_ok=True, detect_usable=True, segments_planned=4,
            segments_succeeded=3,
            segment_failures=(("pick_retreat", "timeout"),),
            attach_xy_err=0.02, fail_code="PLAN_pick_retreat",
            vac_attach=True, vac_follow=False,
        )
        back = trial_from_dict(trial_to_dict(rec))
        self.assertEqual(back.index, 3)
        self.assertEqual(back.segment_failures, (("pick_retreat", "timeout"),))
        self.assertEqual(back.fail_code, "PLAN_pick_retreat")
        self.assertTrue(back.detect_usable)
        self.assertTrue(back.vac_attach)
        self.assertFalse(back.vac_follow)


class TestObserveLoop(unittest.TestCase):

    def test_clear_happens_after_second_goto_observe(self):
        observe = [
            i for i, name in enumerate(OBSERVE_LOOP_PHASES)
            if name == "goto_observe"]
        self.assertEqual(len(observe), 2)
        self.assertLess(observe[0], OBSERVE_LOOP_PHASES.index("spawn"))
        self.assertGreater(
            OBSERVE_LOOP_PHASES.index("clear_box"), observe[1])
        self.assertLess(
            OBSERVE_LOOP_PHASES.index("plan_execute"), observe[1])

    def test_cargo_generation_ready(self):
        self.assertFalse(cargo_generation_ready(None, 3))
        self.assertFalse(cargo_generation_ready({}, 3))
        self.assertFalse(cargo_generation_ready(
            {"generation": 3, "last_cargo_n_points": 0}, 3))
        self.assertFalse(cargo_generation_ready(
            {"generation": 2, "last_cargo_n_points": 80}, 3))
        self.assertFalse(cargo_generation_ready(
            {"generation": 3, "last_cargo_n_points": -1}, 3))
        self.assertTrue(cargo_generation_ready(
            {"generation": 3, "last_cargo_n_points": 80}, 3))
        self.assertTrue(cargo_generation_ready(
            {"generation": 5, "n_points": 12}, 5))

    def test_yolo_boxes_ready_requires_post_filter_cargo(self):
        self.assertFalse(yolo_boxes_ready(None, 4))
        self.assertFalse(yolo_boxes_ready({}, 4))
        # Previous trial's held box must not count.
        self.assertFalse(yolo_boxes_ready({
            "generation": 4, "instance_id": "box_new", "stamp": 10.0,
            "raw_cargo": False,
            "detections": [{"label": 2, "held": True, "bbox": [1, 1, 8, 8]}],
        }, 4, expected_id="box_new", min_stamp=9.0))
        self.assertFalse(yolo_boxes_ready({
            "generation": 3, "instance_id": "box_new", "stamp": 10.0,
            "raw_cargo": True,
        }, 4, expected_id="box_new", min_stamp=9.0))
        self.assertFalse(yolo_boxes_ready({
            "generation": 4, "instance_id": "", "stamp": 10.0,
            "raw_cargo": True,
        }, 4, expected_id="box_new", min_stamp=9.0))
        self.assertFalse(yolo_boxes_ready({
            "generation": 4, "instance_id": "box_new", "stamp": 8.0,
            "raw_cargo": True,
        }, 4, expected_id="box_new", min_stamp=9.0))
        self.assertTrue(yolo_boxes_ready({
            "generation": 4, "instance_id": "box_new", "stamp": 10.0,
            "raw_cargo": True,
        }, 4, expected_id="box_new", min_stamp=9.0))
        self.assertTrue(yolo_boxes_ready({
            "generation": 4, "instance_id": "box_new", "stamp": 10.0,
            "raw_cargo": False,
            "detections": [{"label": 2, "held": False, "bbox": [2, 2, 9, 9]}],
        }, 4, expected_id="box_new", min_stamp=9.0))

    def test_spawn_to_detect_latency_summary(self):
        records = [
            _trial(index=0, spawn_to_yolo_sec=0.4, spawn_to_visual_sec=0.45,
                   spawn_to_detect_sec=0.5, accuracy_ok=True),
            _trial(index=1, spawn_to_yolo_sec=0.6, spawn_to_visual_sec=0.7,
                   spawn_to_detect_sec=0.7, accuracy_ok=False,
                   fail_code="DETECT_GATE:xy"),
            _trial(index=2, fail_code="YOLO_NOT_READY"),
            _trial(index=3, spawn_to_yolo_sec=0.5, fail_code="SPAWN_VISUAL_MISMATCH"),
        ]
        out = summarize(records)
        self.assertEqual(out["n_yolo_ready"], 3)
        self.assertEqual(out["n_visual_ready"], 2)
        self.assertAlmostEqual(out["yolo_ready_rate"], 3.0 / 4.0)
        self.assertAlmostEqual(out["visual_ready_rate"], 2.0 / 4.0)
        self.assertAlmostEqual(out["spawn_to_yolo_sec"]["mean"], 0.5)
        self.assertAlmostEqual(out["spawn_to_detect_sec"]["mean"], 0.6)
        self.assertEqual(out["n_detect_compared"], 2)
        self.assertEqual(out["fail_codes"]["SPAWN_VISUAL_MISMATCH"], 1)

    def test_plan_passed_when_gt_size_mismatch_but_usable(self):
        rec = _trial(
            index=0, detect_usable=True, accuracy_ok=False,
            accuracy_reason="size", fail_code="DETECT_GATE:size",
            segments_planned=4, segments_succeeded=4,
            retreat_ok=True, retreat_delta_z=0.35)
        self.assertTrue(rec.plan_passed())
        self.assertTrue(rec.retreat_passed())
        out = summarize([rec])
        self.assertEqual(out["detect_pass_rate"], 0.0)
        self.assertEqual(out["detect_usable_rate"], 1.0)
        self.assertEqual(out["plan_pass_rate"], 1.0)
        self.assertEqual(out["pick_pass_rate"], 1.0)

    def test_wait_yolo_before_detect_in_observe_loop(self):
        self.assertLess(
            OBSERVE_LOOP_PHASES.index("spawn"),
            OBSERVE_LOOP_PHASES.index("wait_yolo_boxes"))
        self.assertLess(
            OBSERVE_LOOP_PHASES.index("wait_yolo_boxes"),
            OBSERVE_LOOP_PHASES.index("wait_tracked_cargo"))
        self.assertLess(
            OBSERVE_LOOP_PHASES.index("wait_tracked_cargo"),
            OBSERVE_LOOP_PHASES.index("wait_spawn_visual"))
        self.assertLess(
            OBSERVE_LOOP_PHASES.index("wait_spawn_visual"),
            OBSERVE_LOOP_PHASES.index("detect"))


class TestSpawnVisualMatch(unittest.TestCase):

    def test_matching_catalog_passes(self):
        self.assertTrue(spawn_visual_matches_gt(
            (0.70, 0.45, 0.28), (0.70, 0.45, 0.28)))
        self.assertTrue(spawn_visual_matches_gt(
            (0.45, 0.70, 0.28), (0.70, 0.45, 0.28)))
        self.assertTrue(spawn_visual_matches_gt(
            (0.72, 0.44, 0.27), (0.70, 0.45, 0.28)))

    def test_n20_leftover_sizes_fail(self):
        # Trial 10: carryon leftover vs standard GT.
        self.assertFalse(spawn_visual_matches_gt(
            (0.50, 0.36, 0.25), (0.70, 0.45, 0.28)))
        # Trial 15: large leftover vs carryon GT.
        self.assertFalse(spawn_visual_matches_gt(
            (0.66, 0.44, 0.31), (0.55, 0.40, 0.25)))
        # Trial 17: vintage standard leftover vs large GT (depth blob, not PCA).
        self.assertFalse(spawn_visual_matches_gt(
            (0.64, 0.38, 0.27), (0.80, 0.50, 0.32)))

    def test_carryon_silhouette_matches_aabb_gt(self):
        # n50 trial 00: loafbrr carryon depth blob vs catalog AABB (class path).
        self.assertTrue(spawn_visual_matches_gt(
            (0.465, 0.339, 0.247), (0.55, 0.40, 0.25)))
        self.assertEqual(
            nearest_catalog_id((0.465, 0.339, 0.247), silhouette=True),
            "carryon")
        self.assertEqual(
            nearest_catalog_id((0.55, 0.40, 0.25), silhouette=False),
            "carryon")

    def test_loafbrr_large_blob_vs_catalog_aabb(self):
        # Offset-observe blob (0.625) is nearer standard silhouette than large.
        blob = (0.625, 0.416, 0.316)
        self.assertFalse(spawn_visual_matches_gt(
            blob, (0.80, 0.50, 0.32), expected_class="large"))
        # Centered observe blob is nearer the large silhouette.
        self.assertTrue(spawn_visual_matches_gt(
            (0.671, 0.422, 0.317), (0.80, 0.50, 0.32),
            expected_class="large"))
        self.assertTrue(spawn_visual_matches_gt(
            (0.465, 0.339, 0.247), (0.55, 0.40, 0.25),
            expected_class="carryon"))

    def test_expected_class_keeps_large_tier(self):
        blob = (0.661, 0.424, 0.317)
        self.assertTrue(spawn_visual_matches_gt(
            blob, (0.80, 0.50, 0.32), expected_class="large"))

    def test_missing_observation_fails(self):
        self.assertFalse(spawn_visual_matches_gt(None, (0.7, 0.45, 0.28)))
        self.assertFalse(spawn_visual_matches_gt((0.7, 0.45, 0.28), None))

    def test_raised_object_size_from_box_cloud(self):
        import numpy as np
        platform_z = 0.86
        xs = np.linspace(-1.35, -0.65, 40)
        ys = np.linspace(-0.225, 0.225, 30)
        xx, yy = np.meshgrid(xs, ys)
        top = np.column_stack((
            xx.ravel(), yy.ravel(),
            np.full(xx.size, platform_z + 0.28)))
        platform = np.column_stack((
            xx.ravel(), yy.ravel(),
            np.full(xx.size, platform_z)))
        pts = np.vstack((platform, top))
        size = raised_object_size(
            pts, platform_z, roi_center_xy=(-1.0, 0.0), roi_margin=0.5,
            min_points=200)
        self.assertIsNotNone(size)
        width, depth, height, n = size
        self.assertGreater(n, 200)
        self.assertAlmostEqual(width, 0.70, places=2)
        self.assertAlmostEqual(depth, 0.45, places=2)
        self.assertAlmostEqual(height, 0.28, places=2)
        self.assertTrue(spawn_visual_matches_gt(
            (width, depth, height), (0.70, 0.45, 0.28)))
        measured = raised_object_measure(
            pts, platform_z, roi_center_xy=(-1.0, 0.0), roi_margin=0.5,
            min_points=200)
        self.assertAlmostEqual(measured["x"], -1.0, places=2)
        self.assertAlmostEqual(measured["y"], 0.0, places=2)
        self.assertAlmostEqual(measured["z"], platform_z + 0.14, places=2)

    def test_arm_above_suitcase_does_not_inflate_height(self):
        import numpy as np
        platform_z = 0.86
        xs = np.linspace(-1.35, -0.65, 40)
        ys = np.linspace(-0.225, 0.225, 30)
        xx, yy = np.meshgrid(xs, ys)
        top = np.column_stack((
            xx.ravel(), yy.ravel(),
            np.full(xx.size, platform_z + 0.28)))
        arm = np.column_stack((
            np.full(80, -1.0), np.full(80, 0.0),
            np.full(80, platform_z + 0.64)))
        pts = np.vstack((top, arm))
        unclipped = raised_object_size(
            pts, platform_z, roi_center_xy=(-1.0, 0.0), roi_margin=0.5,
            min_points=200)
        self.assertGreater(unclipped[2], 0.50)
        clipped = raised_object_size(
            pts, platform_z, roi_center_xy=(-1.0, 0.0), roi_margin=0.5,
            min_points=200, max_height=0.36)
        self.assertAlmostEqual(clipped[2], 0.28, places=2)
        self.assertTrue(spawn_visual_matches_gt(
            clipped[:3], (0.70, 0.45, 0.28)))

    def test_visual_tol_catches_catalog_step(self):
        self.assertGreater(0.10, DEFAULT_VISUAL_TOL_XY - 1e-9)

    def test_depth_unproject_optical_z(self):
        import numpy as np
        from luggage_gazebo.eval_metrics import depth_to_camera_xyz
        depth = np.full((10, 10), 1.0, dtype=np.float64)
        depth[0, 0] = 0.0
        pts = depth_to_camera_xyz(depth, fx=100.0, fy=100.0, cx=5.0, cy=5.0,
                                  stride=1)
        self.assertGreater(len(pts), 50)
        self.assertTrue(np.allclose(pts[:, 2], 1.0))

    def test_tracker_epoch_matches(self):
        self.assertFalse(tracker_epoch_matches(None, 4))
        self.assertFalse(tracker_epoch_matches(
            {"generation": 3, "instance_id": "box"}, 4, "box"))
        self.assertFalse(tracker_epoch_matches(
            {"generation": 4, "instance_id": ""}, 4, "box"))
        self.assertTrue(tracker_epoch_matches(
            {"generation": 4, "instance_id": "box"}, 4, "box"))

    def test_label_aabb_empty_and_cargo(self):
        import numpy as np
        empty = np.zeros((8, 8), dtype=np.uint8)
        self.assertIsNone(label_aabb(empty, 2))
        mask = np.zeros((10, 12), dtype=np.uint8)
        mask[2:6, 3:9] = 2
        aabb = label_aabb(mask, 2)
        self.assertEqual(aabb["u_min"], 3)
        self.assertEqual(aabb["v_min"], 2)
        self.assertEqual(aabb["width_px"], 6)
        self.assertEqual(aabb["height_px"], 4)
        self.assertEqual(aabb["n"], 24)

    def test_points_aabb_span(self):
        import numpy as np
        xs, ys, zs = np.meshgrid(
            np.linspace(0.0, 0.50, 20),
            np.linspace(0.0, 0.30, 15),
            np.linspace(0.0, 0.20, 10))
        pts = np.column_stack((xs.ravel(), ys.ravel(), zs.ravel()))
        aabb = points_aabb(pts, percentile=(0.0, 100.0))
        self.assertIsNotNone(aabb)
        self.assertAlmostEqual(aabb[0], 0.50, places=2)
        self.assertAlmostEqual(aabb[1], 0.30, places=2)
        self.assertAlmostEqual(aabb[2], 0.20, places=2)
        self.assertEqual(aabb[3], len(pts))
        self.assertIsNone(points_aabb(np.zeros((2, 3))))

    def test_tracker_wait_fail_code_split(self):
        self.assertEqual(
            tracker_wait_fail_code(
                {"generation": 4, "instance_id": "box", "n_points": 0},
                4, "box"),
            "CARGO_NOT_READY")
        self.assertEqual(
            tracker_wait_fail_code(
                {"generation": 3, "instance_id": "", "n_points": 80},
                4, "box"),
            "TRACKER_STALE")
        self.assertEqual(
            tracker_wait_fail_code(
                {"generation": 4, "instance_id": "", "n_points": 0},
                4, "box"),
            "TRACKER_STALE")
        self.assertEqual(tracker_wait_fail_code(None, 4, "box"), "TRACKER_STALE")


if __name__ == "__main__":
    unittest.main()
