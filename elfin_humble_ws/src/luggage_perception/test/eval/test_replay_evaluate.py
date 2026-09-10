#!/usr/bin/env python3
"""Unit tests for the pendant replay evaluator (stub backend, fixture bag)."""
import json
import os
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "fixtures"))
from make_tiny_replay_bag import (  # noqa: E402
    BASE_NS,
    FRAME_DT_NS,
    build_fixture,
)

from luggage_perception.eval.bag_frame_join import frame_dir_name  # noqa: E402
from luggage_perception.eval.replay_evaluate import (  # noqa: E402
    ReplayEvalConfig,
    evaluate_bag,
    load_segmenter_config,
    repaint_label_map,
    select_cargo_detection,
)

T0 = BASE_NS
T1 = BASE_NS + FRAME_DT_NS
T2 = BASE_NS + 2 * FRAME_DT_NS
T3 = BASE_NS + 3 * FRAME_DT_NS


def _stub_cfg(**kwargs):
    # 1 ms join tolerance pins the fixture's orphan frames as orphans;
    # the 30 ms production default is covered separately below.
    cfg = ReplayEvalConfig(backend="stub", require_backend=False,
                           join_tolerance_ms=1.0)
    for key, value in kwargs.items():
        setattr(cfg, key, value)
    return cfg


class TestLoadSegmenterConfig(unittest.TestCase):
    def test_defaults_without_yaml(self):
        cfg = _stub_cfg()
        config = load_segmenter_config(cfg)
        self.assertEqual(len(config["prompts"]), 8)
        self.assertEqual(config["class_mapping"]["suitcase"], 2)
        self.assertEqual(config["class_mapping"]["floor"], 0)
        self.assertAlmostEqual(config["confidence_threshold"], 0.005)

    def test_prompts_override(self):
        cfg = _stub_cfg(prompts=["suitcase", "container"],
                        class_mapping_labels=[2, 1])
        config = load_segmenter_config(cfg)
        self.assertEqual(config["class_mapping"],
                         {"suitcase": 2, "container": 1})

    def test_prompt_label_length_mismatch_raises(self):
        cfg = _stub_cfg(prompts=["a", "b"], class_mapping_labels=[2])
        with self.assertRaises(ValueError):
            load_segmenter_config(cfg)

    def test_package_yaml_section(self):
        root = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "..")
        yaml_path = os.path.join(root, "config", "semantic_segmenter.yaml")
        if not os.path.isfile(yaml_path):
            self.skipTest("semantic_segmenter.yaml not found")
        cfg = _stub_cfg(config_yaml=yaml_path)
        config = load_segmenter_config(cfg)
        self.assertGreater(len(config["prompts"]), 0)
        self.assertEqual(
            len(config["class_mapping"]), len(config["prompts"]))


class TestEvaluateBagStub(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.bag = build_fixture(
            os.path.join(cls._tmp.name, "tiny.mcap"))
        cls.out = os.path.join(cls._tmp.name, "out")
        cls.summary = evaluate_bag(cls.bag, cls.out, _stub_cfg())
        cls.bag_out = os.path.join(cls.out, "tiny")
        cls.frames = os.path.join(cls.bag_out, "frames")

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def test_summary(self):
        self.assertEqual(self.summary["frames_processed"], 2)
        self.assertTrue(self.summary["backend"].startswith("stub"))
        self.assertEqual(self.summary["join"]["n_exact"], 2)

    def test_frame_dirs_named_by_stamp(self):
        names = sorted(os.listdir(self.frames))
        self.assertEqual(names, sorted(
            [frame_dir_name(T0), frame_dir_name(T1)]))

    def test_frame_dir_layout(self):
        frame = os.path.join(self.frames, frame_dir_name(T0))
        files = set(os.listdir(frame))
        for expected in ("color.png", "mask.npy", "mask.png",
                         "overlay.png", "detections.json", "meta.json",
                         "joint_state.json", "tcp_pose.json",
                         "depth.npy", "depth_vis.png", "cargo_points.ply"):
            self.assertIn(expected, files)
        # camera_info is per bag, never per frame.
        self.assertNotIn("camera_info.json", files)

    def test_camera_info_written_once(self):
        payload = json.load(open(os.path.join(
            self.bag_out, "camera_info.json")))
        self.assertIn("color", payload)
        self.assertEqual(payload["color_distinct_k"], 1)
        self.assertEqual(
            payload["color"]["k"][0], 200.0)  # fx from the fixture

    def test_join_report(self):
        report = json.load(open(os.path.join(
            self.bag_out, "join_report.json")))
        self.assertEqual(report["join"]["n_pairs"], 2)
        self.assertEqual(report["color_orphans"], [T2])
        self.assertEqual(
            report["depth_orphans"], sorted([T2 + 5_000_000, T3]))
        self.assertEqual(report["duplicates"], {"color": 1, "depth": 0})

    def test_jsonl_rows(self):
        det_lines = open(os.path.join(
            self.bag_out, "detections.jsonl")).read().strip().splitlines()
        frame_lines = open(os.path.join(
            self.bag_out, "frames.jsonl")).read().strip().splitlines()
        self.assertEqual(len(det_lines), 2)
        self.assertEqual(len(frame_lines), 2)

    def test_joint_join_hit_and_miss(self):
        # joint_states sits 20 ms before T0: within the 50 ms window for
        # frame T0, but 53 ms away from T1 -> miss.
        first = json.load(open(os.path.join(
            self.frames, frame_dir_name(T0), "joint_state.json")))
        self.assertEqual(len(first["position"]), 6)
        self.assertAlmostEqual(first["dt_sec"] * 1e3, -20.0, places=3)
        self.assertFalse(os.path.exists(os.path.join(
            self.frames, frame_dir_name(T1), "joint_state.json")))
        meta = json.load(open(os.path.join(
            self.frames, frame_dir_name(T1), "meta.json")))
        self.assertIsNone(meta["joint_dt_ms"])

    def test_stub_detections_empty(self):
        meta = json.load(open(os.path.join(
            self.frames, frame_dir_name(T0), "meta.json")))
        self.assertEqual(meta["detection_count"], 0)
        self.assertEqual(meta["cargo_points"]["n_points"], 0)

    def test_lidar_full_archive(self):
        # Every /livox/lidar message is archived under lidar/<stamp>/,
        # including the duplicate stamp and the scan far from any frame.
        index_path = os.path.join(self.bag_out, "lidar_index.jsonl")
        rows = [json.loads(line) for line in open(index_path)]
        self.assertEqual(len(rows), 3)
        self.assertEqual(
            sum(1 for row in rows if row["dir"] is not None), 3)
        scan_dir = os.path.join(self.bag_out, rows[0]["dir"])
        self.assertTrue(os.path.isfile(
            os.path.join(scan_dir, "points.npy")))
        self.assertTrue(os.path.isfile(
            os.path.join(scan_dir, "lidar.ply")))
        points = np.load(os.path.join(scan_dir, "points.npy"))
        self.assertEqual(points.shape, (4, 4))
        report = json.load(open(os.path.join(
            self.bag_out, "join_report.json")))
        self.assertEqual(report["aux_join"]["lidar_archive"]["n_scans"], 3)
        self.assertEqual(
            report["aux_join"]["lidar_archive"]["n_points_total"], 12)
        # The far scan (t3+30ms, no camera frame nearby) is archived too.
        far = frame_dir_name(T3 + 30_000_000)
        self.assertTrue(os.path.isdir(
            os.path.join(self.bag_out, "lidar", far)))

    def test_frame_meta_points_at_nearest_archived_scan(self):
        meta = json.load(open(os.path.join(
            self.frames, frame_dir_name(T0), "meta.json")))
        self.assertIn("lidar", meta)
        self.assertEqual(meta["lidar"]["nearest_stamp_ns"],
                         T0 + 2_000_000)
        self.assertTrue(meta["lidar"]["nearest_dir"].startswith("lidar/"))

    def test_index_written(self):
        with open(os.path.join(self.bag_out, "INDEX.md")) as handle:
            text = handle.read()
        self.assertIn("| [%s]" % frame_dir_name(T0), text)

    def test_tolerance_rescue_pairs_near_depth(self):
        # The fixture's f2 colour orphan has its nearest depth 5 ms away:
        # inside the 30 ms production tolerance it becomes a tolerance
        # pair with the dt recorded in meta.json.
        with tempfile.TemporaryDirectory() as tmp:
            summary = evaluate_bag(
                self.bag, tmp, _stub_cfg(join_tolerance_ms=30.0))
            self.assertEqual(summary["frames_processed"], 3)
            self.assertEqual(summary["join"]["n_tolerance"], 1)
            meta = json.load(open(os.path.join(
                tmp, "tiny", "frames", frame_dir_name(T2), "meta.json")))
            self.assertEqual(meta["join_source"], "tolerance")
            self.assertEqual(meta["join_dt_ns"], 5_000_000)

    def test_dry_run_skips_frames(self):
        with tempfile.TemporaryDirectory() as tmp:
            summary = evaluate_bag(self.bag, tmp, _stub_cfg(dry_run=True))
            self.assertEqual(summary["frames_processed"], 0)
            self.assertTrue(summary["dry_run"])
            self.assertFalse(os.path.exists(
                os.path.join(tmp, "tiny", "detections.jsonl")))


class TestCargoSelection(unittest.TestCase):
    # 896x504: centre (448, 252), central radius 0.35*896 = 314 px.
    IMG = (896, 504)

    @staticmethod
    def _det(conf, bbox, label=2, prompt="suitcase"):
        return {"label": label, "prompt": prompt, "confidence": conf,
                "bbox": list(bbox)}

    def test_central_max_conf_wins(self):
        dets = [
            self._det(0.95, [698, 0, 896, 197]),   # strong but border
            self._det(0.80, [263, 274, 576, 504]),  # central, true box
            self._det(0.10, [100, 100, 200, 200]),  # below floor
        ]
        kept, info = select_cargo_detection(dets, self.IMG)
        self.assertEqual(kept["bbox"], [263, 274, 576, 504])
        self.assertEqual(info["selection"], "central")
        self.assertEqual(info["n_dropped"], 2)

    def test_off_center_fallback_when_no_central_candidate(self):
        dets = [self._det(0.9, [0, 0, 150, 150]),
                self._det(0.5, [820, 380, 896, 504])]
        kept, info = select_cargo_detection(dets, self.IMG)
        self.assertEqual(info["selection"], "off_center")
        self.assertEqual(kept["confidence"], 0.9)

    def test_below_floor_keeps_nothing(self):
        dets = [self._det(0.29, [263, 274, 576, 504])]
        kept, info = select_cargo_detection(dets, self.IMG)
        self.assertIsNone(kept)
        self.assertEqual(info["selection"], "none")

    def test_non_cargo_labels_ignored(self):
        dets = [self._det(0.99, [300, 200, 500, 400], label=0)]
        kept, info = select_cargo_detection(dets, self.IMG)
        self.assertIsNone(kept)
        self.assertEqual(info["n_cargo_raw"], 0)

    def test_repaint_only_kept_bbox(self):
        shape = (12, 16)
        kept = [self._det(0.9, [2, 3, 6, 9])]
        label_map = repaint_label_map(shape, kept)
        self.assertEqual(int((label_map == 2).sum()), 4 * 6)
        self.assertEqual(int(label_map[5, 4]), 2)
        self.assertEqual(int(label_map[0, 0]), 0)


class TestRealBagIntegration(unittest.TestCase):
    """Gated on $REPLAY_BAG (a real pendant bag path). Not run by default."""

    def test_real_bag_five_frames(self):
        bag = os.environ.get("REPLAY_BAG")
        if not bag:
            self.skipTest("set REPLAY_BAG to a real pendant bag path")
        with tempfile.TemporaryDirectory() as tmp:
            summary = evaluate_bag(bag, tmp, _stub_cfg(max_frames=5))
            self.assertEqual(summary["frames_processed"], 5)
            self.assertEqual(summary["join"]["n_exact"], 5)


if __name__ == "__main__":
    unittest.main()
