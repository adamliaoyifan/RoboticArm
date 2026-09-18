#!/usr/bin/env python3
"""Unit tests for the pickup XY candidates producer."""
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "fixtures"))
from make_tiny_replay_bag import build_fixture  # noqa: E402

from luggage_perception.eval.pickup_xy_benchmark import (  # noqa: E402
    load_candidates,
    load_labels,
    acceptance,
    score_candidates,
)
from luggage_perception.eval.pickup_xy_candidates import (  # noqa: E402
    bbox_center_pixel,
    candidates_frames,
    deproject_pinhole,
    lid_inlier_center,
    median_depth_window,
    strategies_for_frame,
    write_candidates_file,
)
from luggage_perception.eval.site_pick_replay import (  # noqa: E402
    SitePickConfig,
    replay_site_pick,
)


def _intrinsics(fx=200.0, fy=200.0, cx=8.0, cy=6.0):
    return SimpleNamespace(fx=fx, fy=fy, cx=cx, cy=cy)


def _detector_result(xy, top_z):
    top = SimpleNamespace(center_xy=np.array(xy), top_z=top_z)
    return SimpleNamespace(box=SimpleNamespace(top=top))


class TestStrategyPieces(unittest.TestCase):
    def test_bbox_center(self):
        det = {"bbox": [10, 20, 30, 60]}
        self.assertEqual(bbox_center_pixel(det), (20.0, 40.0))
        self.assertIsNone(bbox_center_pixel(None))

    def test_median_depth_window_skips_zero(self):
        depth = np.array([[0, 0, 900, 0],
                          [0, 1000, 1100, 0],
                          [0, 0, 0, 0]], dtype=np.uint16)
        self.assertAlmostEqual(median_depth_window(depth, 1, 1), 1.0)
        self.assertAlmostEqual(median_depth_window(depth, 2, 0), 1.0)
        empty = np.zeros((4, 4), dtype=np.uint16)
        self.assertIsNone(median_depth_window(empty, 2, 2))
        self.assertIsNone(median_depth_window(None, 2, 2))

    def test_deproject_pinhole_convention(self):
        point = deproject_pinhole(9.0, 7.0, 2.0, _intrinsics())
        self.assertAlmostEqual(point[0], (9.0 - 8.0) * 2.0 / 200.0)
        self.assertAlmostEqual(point[1], (7.0 - 6.0) * 2.0 / 200.0)
        self.assertAlmostEqual(point[2], 2.0)

    def test_lid_inlier_center_thresholds_around_top(self):
        pts = np.array([[0.0, 0.0, 1.0], [0.2, 0.1, 1.005],
                        [0.4, 0.2, 1.5], [0.6, 0.3, 0.9]])
        xy = lid_inlier_center(pts, top_z=1.0, dist_thresh=0.008,
                               min_points=2)
        self.assertIsNotNone(xy)
        self.assertAlmostEqual(xy[0], 0.1)
        self.assertAlmostEqual(xy[1], 0.05)
        self.assertIsNone(lid_inlier_center(pts, 1.0, 0.0001,
                                            min_points=2))
        self.assertIsNone(lid_inlier_center(None, 1.0, 0.01))
        # Below the default density floor the answer is null, not a
        # noise-driven centre.
        self.assertIsNone(lid_inlier_center(pts, 1.0, 0.008))


class TestStrategiesForFrame(unittest.TestCase):
    def test_full_frame_all_strategies(self):
        depth = np.full((12, 16), 1500, dtype=np.uint16)
        det = {"bbox": [2, 3, 10, 9]}
        tf = lambda pts: pts + np.array([1.0, 2.0, 0.0])  # noqa: E731
        out = strategies_for_frame(
            det, depth, _intrinsics(), np.zeros((0, 3)),
            _detector_result((0.5, 0.25), 1.0), 0.008,
            tf_points_fn=tf)
        self.assertEqual(out["pca_center"], [0.5, 0.25])
        self.assertIsNotNone(out["yolo_bbox_center_top_plane"])
        self.assertIsNone(out["lid_inlier_center"])  # empty cloud
        # Blend of two available sources: elementwise median of 2 is
        # their mean.
        blend = out["robust_blended_center"]
        self.assertIsNotNone(blend)

    def test_tf_miss_propagates_null(self):
        depth = np.full((12, 16), 1500, dtype=np.uint16)
        det = {"bbox": [2, 3, 10, 9]}
        out = strategies_for_frame(
            det, depth, _intrinsics(), None, _detector_result((1, 1), 1.0),
            0.008, tf_points_fn=lambda pts: None)
        self.assertIsNone(out["yolo_bbox_center_top_plane"])
        self.assertEqual(out["pca_center"], [1.0, 1.0])

    def test_no_detection_all_null_but_schema_complete(self):
        out = strategies_for_frame(
            None, None, None, None, None, 0.008, tf_points_fn=None)
        self.assertEqual(
            sorted(out),
            ["lid_inlier_center", "pca_center",
             "robust_blended_center", "yolo_bbox_center_top_plane"])
        self.assertTrue(all(value is None for value in out.values()))


class TestCandidateFiles(unittest.TestCase):
    def test_rows_shape_matches_benchmark_loader(self):
        rows = [{
            "stamp_sec": 123.0,
            "pickup_candidates": {"pca_center": [0.1, 0.2],
                                  "yolo_bbox_center_top_plane": None,
                                  "lid_inlier_center": [0.3, 0.4],
                                  "robust_blended_center": [0.2, 0.3]},
        }, {
            "stamp_sec": 124.0,
            "pickup_candidates": None,
        }]
        frames = candidates_frames("tiny", "/bag/tiny.mcap", rows)
        self.assertEqual(len(frames), 1)
        self.assertTrue(frames[0]["frame_id"].startswith("tiny@"))
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "candidates.json")
            count = write_candidates_file(
                path, "tiny", "/bag/tiny.mcap", rows, {"code_revision": "x"})
            self.assertEqual(count, 1)
            parsed = load_candidates(Path(path))
        by_strategy = {}
        for candidate in parsed:
            by_strategy[candidate.strategy] = candidate.xy
        self.assertEqual(list(by_strategy["pca_center"]), [0.1, 0.2])
        self.assertNotIn("yolo_bbox_center_top_plane", by_strategy)

    def test_frame_id_joins_label_by_explicit_id(self):
        rows = [{
            "stamp_sec": 5.0,
            "pickup_candidates": {"pca_center": [0.0, 0.0],
                                  "yolo_bbox_center_top_plane": None,
                                  "lid_inlier_center": None,
                                  "robust_blended_center": None},
        }]
        frames = candidates_frames("tiny", "/bag/tiny.mcap", rows)
        frame_id = frames[0]["frame_id"]
        # A label keyed the same explicit way joins.
        with tempfile.TemporaryDirectory() as tmp:
            labels_path = os.path.join(tmp, "labels.json")
            with open(labels_path, "w", encoding="utf-8") as handle:
                json.dump({"labels": [{
                    "frame_id": frame_id,
                    "suction_safe_lid_center_world_xy": [0.01, 0.0],
                }]}, handle)
            cand_path = os.path.join(tmp, "cand.json")
            write_candidates_file(
                cand_path, "tiny", "/bag/tiny.mcap", rows, {})
            summary = score_candidates(
                load_labels(Path(labels_path)),
                load_candidates(Path(cand_path)))
            # No blend was computable (one source only) -> the gate
            # cannot evaluate it, but the pca rows joined and scored.
            self.assertEqual(
                acceptance(summary, "robust_blended_center")["outcome"],
                "not_evaluated")
            pca = summary["results"]["pca_center"]
            self.assertEqual(pca["sample_count"], 1)
            self.assertAlmostEqual(pca["median_error_m"], 0.01)


class TestReplayEmitsCandidates(unittest.TestCase):
    def test_fixture_replay_writes_candidates_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            bag = build_fixture(os.path.join(tmp, "tiny.mcap"))
            out = os.path.join(tmp, "out")
            cand = os.path.join(tmp, "cand", "candidates.json")
            summary = replay_site_pick(
                bag, out, SitePickConfig(
                    backend="stub", require_backend=False, device="cpu",
                    stride=1, max_frames=2, emit_candidates=cand))
            self.assertEqual(summary["pickup_candidate_frames"], 0)
            # The stub backend yields no detections: zero candidate rows
            # is the honest outcome, and the file still parses.
            payload = json.load(open(cand))
            self.assertEqual(payload["frames"], [])
            self.assertIn("generator", payload)


if __name__ == "__main__":
    unittest.main()
