#!/usr/bin/env python3
"""Unit tests for semantic_segmenter (no roscore, no ML deps required)."""

import os
import sys
import unittest

import numpy as np

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from luggage_perception.semantic_segmenter import (  # noqa: E402
    LABEL_BACKGROUND,
    LABEL_CARGO,
    LABEL_CONTAINER_WALL,
    LABEL_ROBOT_ARM,
    LABEL_UNKNOWN,
    StubSegmenter,
    apply_self_body_mask,
    apply_wrist_self_body,
    bbox_mask_overlap,
    build_segmenter,
    colorize_label_map,
    compact_detections,
    detections_dropped_by_self_body,
)


class TestClassMapping(unittest.TestCase):
    def test_known_prompts_resolve(self):
        from luggage_perception.semantic_segmenter import _resolve_class_mapping
        prompts = ["suitcase", "container", "floor"]
        mapping = _resolve_class_mapping(prompts, {
            "suitcase": 2, "container": 1, "floor": 0,
        })
        self.assertEqual(mapping["suitcase"], LABEL_CARGO)
        self.assertEqual(mapping["container"], LABEL_CONTAINER_WALL)
        self.assertEqual(mapping["floor"], LABEL_BACKGROUND)

    def test_unknown_prompt_defaults_to_unknown(self):
        from luggage_perception.semantic_segmenter import _resolve_class_mapping
        mapping = _resolve_class_mapping(["weird thing"], {})
        self.assertEqual(mapping["weird thing"], LABEL_UNKNOWN)

    def test_label_name_resolves_when_no_explicit_mapping(self):
        from luggage_perception.semantic_segmenter import _resolve_class_mapping
        mapping = _resolve_class_mapping(["cargo"], {})
        self.assertEqual(mapping["cargo"], LABEL_CARGO)


class TestStubSegmenter(unittest.TestCase):
    def test_stub_returns_all_background(self):
        seg = StubSegmenter(["suitcase", "box"], {"suitcase": 2, "box": 2})
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        label_map, detections = seg.segment(img)
        self.assertEqual(label_map.shape, (480, 640))
        self.assertEqual(label_map.dtype, np.uint8)
        self.assertTrue((label_map == LABEL_BACKGROUND).all())
        self.assertEqual(detections, [])
        stats = seg.last_stats
        self.assertEqual(stats["backend"], "stub")
        self.assertEqual(stats["detection_count"], 0)
        self.assertEqual(stats["label_counts"][LABEL_BACKGROUND], 480 * 640)


class TestBuildSegmenter(unittest.TestCase):
    def test_build_stub(self):
        seg = build_segmenter({
            "backend": "stub",
            "prompts": ["suitcase"],
            "class_mapping": {"suitcase": 2},
        })
        self.assertEqual(seg.last_stats["backend"], "stub")
        self.assertEqual(seg.prompts, ["suitcase"])
        self.assertEqual(seg.class_mapping["suitcase"], LABEL_CARGO)
        self.assertIsNotNone(seg.temporal_gate)

    def test_temporal_window_one_disables_gate(self):
        seg = build_segmenter({
            "backend": "stub",
            "prompts": ["suitcase"],
            "temporal_window_frames": 1,
        })
        self.assertIsNone(seg.temporal_gate)

    def test_build_unknown_backend_raises(self):
        with self.assertRaises(ValueError):
            build_segmenter({"backend": "nonsense", "prompts": []})

    def test_build_yolo_world_falls_back_when_deps_missing(self):
        # Simulate ultralytics being unavailable so the bbox_fill backend
        # cannot construct — build_segmenter must degrade to stub, not raise
        # and not attempt a model download.
        import importlib
        real_ultralytics = sys.modules.pop("ultralytics", None)
        sys.modules["ultralytics"] = None  # forces ImportError on `import`

        try:
            seg = build_segmenter({
                "backend": "yolo_world",
                "prompts": ["suitcase"],
                "class_mapping": {"suitcase": 2},
                "model_name": "yolov8s-world.pt",
                "device": "cpu",
            })
        finally:
            if real_ultralytics is not None:
                sys.modules["ultralytics"] = real_ultralytics
            else:
                del sys.modules["ultralytics"]
            # Drop any cached submodule references left over.
            for key in list(sys.modules.keys()):
                if key.startswith("ultralytics."):
                    sys.modules.pop(key, None)

        self.assertIn("stub", seg.last_stats["backend"])
        self.assertIn("yolo_world", seg.last_stats["backend"])


class TestColorize(unittest.TestCase):
    def test_colorize_shape(self):
        label_map = np.zeros((10, 12), dtype=np.uint8)
        label_map[2:4, 3:7] = LABEL_CARGO
        bgr = colorize_label_map(label_map)
        self.assertEqual(bgr.shape, (10, 12, 3))
        self.assertEqual(bgr.dtype, np.uint8)
        # Cargo palette is (0, 0, 220) in BGR.
        self.assertTrue((bgr[3, 5] == [0, 0, 220]).all())


class TestCompactDetections(unittest.TestCase):
    def test_drops_mask_and_is_json_safe(self):
        import json
        dets = compact_detections([{
            "label": LABEL_CARGO,
            "prompt": "box",
            "confidence": 0.4,
            "bbox": [1.2, 2.8, 9.1, 10.6],
            "held": False,
            "mask": np.ones((4, 4), dtype=bool),
        }])
        self.assertNotIn("mask", dets[0])
        self.assertEqual(dets[0]["bbox"], [1, 3, 9, 11])
        json.dumps(dets)


class TestDrawOverlay(unittest.TestCase):
    def test_overlay_draws_bbox_and_mask(self):
        try:
            import cv2  # noqa: F401
        except ImportError:
            self.skipTest("cv2 not available in test environment")
        from luggage_perception.semantic_segmenter import draw_detections_overlay

        rgb = np.full((60, 80, 3), 200, dtype=np.uint8)  # grey background
        mask = np.zeros((60, 80), dtype=bool)
        mask[15:45, 45:65] = True
        detections = [
            {
                "label": LABEL_CARGO,
                "prompt": "suitcase",
                "confidence": 0.87,
                "bbox": [10, 10, 30, 40],
            },
            {
                "label": LABEL_UNKNOWN,
                "prompt": "box",
                "confidence": 0.55,
                "bbox": [40, 5, 70, 50],
                "mask": mask,
            },
        ]

        bgr = draw_detections_overlay(rgb, detections)
        self.assertEqual(bgr.shape, (60, 80, 3))
        self.assertEqual(bgr.dtype, np.uint8)
        # Cargo bbox top-left corner is drawn in cargo red (0, 0, 220).
        self.assertTrue((bgr[10, 10] == [0, 0, 220]).all())

        # Empty-detections path returns the plain BGR-converted image.
        plain = draw_detections_overlay(rgb, [])
        self.assertTrue((plain == cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)).all())


if __name__ == "__main__":
    unittest.main()


class TestUpdateCopyOutput(unittest.TestCase):
    """update()/copy_output() snapshot contract (Todo 1)."""

    def test_copy_output_none_before_update(self):
        seg = build_segmenter({"backend": "stub", "prompts": ["suitcase"]})
        self.assertIsNone(seg.copy_output())

    def test_update_copies_stamp_and_frame(self):
        seg = build_segmenter({"backend": "stub", "prompts": ["suitcase"]})
        rgb = np.zeros((12, 16, 3), dtype=np.uint8)
        seg.update(rgb, stamp=123.5, frame_id="camera_depth_optical_frame")
        out = seg.copy_output()
        self.assertIsNotNone(out)
        self.assertEqual(out.stamp, 123.5)
        self.assertEqual(out.frame_id, "camera_depth_optical_frame")
        self.assertEqual(out.label_map.shape, (12, 16))
        self.assertEqual(out.label_map.dtype, np.uint8)
        self.assertIsNone(out.instance_map)
        self.assertIn("backend", out.stats)

    def test_second_update_does_not_mutate_first_output(self):
        seg = build_segmenter({"backend": "stub", "prompts": ["suitcase"]})
        rgb = np.zeros((12, 16, 3), dtype=np.uint8)
        seg.update(rgb, stamp=1.0, frame_id="f1")
        first = seg.copy_output()
        seg.update(rgb, stamp=2.0, frame_id="f2")
        self.assertEqual(first.stamp, 1.0)
        self.assertEqual(first.frame_id, "f1")
        self.assertEqual(seg.copy_output().stamp, 2.0)

    def test_mutating_returned_output_does_not_leak(self):
        seg = build_segmenter({"backend": "stub", "prompts": ["suitcase"]})
        rgb = np.zeros((12, 16, 3), dtype=np.uint8)
        seg.update(rgb, stamp=1.0, frame_id="f")
        out = seg.copy_output()
        out.label_map[:] = 9
        out.stats["backend"] = "tampered"
        fresh = seg.copy_output()
        self.assertEqual(int(fresh.label_map.max()), 0)
        self.assertNotEqual(fresh.stats["backend"], "tampered")

    def test_detection_mask_is_copied(self):
        # A backend that returns detections with an HxW bool mask (SAM2 shape).
        mask = np.zeros((12, 16), dtype=bool)
        mask[2:6, 3:9] = True
        detections = [{"label": 2, "prompt": "suitcase", "confidence": 0.9,
                       "bbox": [3, 2, 9, 6], "mask": mask}]

        from luggage_perception.semantic_segmenter import SemanticSegmenter

        class _Det(SemanticSegmenter):
            def segment(self, rgb_image):
                label_map = np.zeros(rgb_image.shape[:2], dtype=np.uint8)
                label_map[mask] = 2
                self._instance_map = np.where(mask, 1, 0).astype(np.uint16)
                return label_map, detections

        seg = _Det(["suitcase"])
        rgb = np.zeros((12, 16, 3), dtype=np.uint8)
        seg.update(rgb, stamp=1.0, frame_id="f")
        out = seg.copy_output()
        self.assertEqual(len(out.detections), 1)
        self.assertIsNotNone(out.instance_map)
        # Mutate the ORIGINAL arrays the backend still references.
        mask[:] = False
        seg._instance_map[:] = 0
        self.assertTrue(out.detections[0]["mask"].any())
        # The snapshot kept the original instance ids.
        self.assertGreater(int(out.instance_map.sum()), 0)

    def test_stub_fallback_backend_name(self):
        # Direct construction with a missing ML dep path is covered by
        # build_segmenter's fallback; assert the naming contract here.
        seg = build_segmenter({"backend": "stub", "prompts": ["x"]})
        self.assertTrue(seg.last_stats["backend"].startswith("stub"))


class TestWristSelfBody(unittest.TestCase):
    def test_disabled_frac_is_noop(self):
        labels = np.zeros((10, 8), dtype=np.uint8)
        labels[8:, :] = LABEL_CARGO
        dets = [{"label": LABEL_CARGO, "prompt": "x", "confidence": 0.2,
                 "bbox": [0, 8, 8, 10]}]
        out, kept, inst, n_self = apply_wrist_self_body(labels, dets, 0.0)
        self.assertEqual(n_self, 0)
        self.assertEqual(int((out == LABEL_CARGO).sum()), 16)
        self.assertEqual(len(kept), 1)
        self.assertIsNone(inst)

    def test_band_relabels_and_drops_bottom_box(self):
        labels = np.zeros((100, 10), dtype=np.uint8)
        labels[20:50, :] = LABEL_CARGO
        labels[85:, :] = LABEL_CARGO
        dets = [
            {"label": LABEL_CARGO, "prompt": "box", "confidence": 0.18,
             "bbox": [0, 20, 10, 50]},
            {"label": LABEL_CARGO, "prompt": "panel", "confidence": 0.22,
             "bbox": [0, 85, 10, 100]},
        ]
        inst = np.zeros((100, 10), dtype=np.uint16)
        inst[85:, :] = 7
        out, kept, inst_out, n_self = apply_wrist_self_body(
            labels, dets, 0.85, instance_map=inst)
        self.assertEqual(n_self, 15 * 10)
        self.assertTrue(np.all(out[85:] == LABEL_ROBOT_ARM))
        self.assertTrue(np.all(out[20:50] == LABEL_CARGO))
        self.assertEqual(len(kept), 1)
        self.assertEqual(kept[0]["bbox"], [0, 20, 10, 50])
        self.assertTrue(np.all(inst_out[85:] == 0))
        self.assertEqual(int(labels[85, 0]), LABEL_CARGO)

    def test_straddling_bbox_dropped_when_half_on_band(self):
        labels = np.zeros((10, 4), dtype=np.uint8)
        dets = [{"label": LABEL_CARGO, "prompt": "x", "confidence": 0.1,
                 "bbox": [0, 6, 4, 10]}]
        out, kept, _, _ = apply_wrist_self_body(labels, dets, 0.8)
        self.assertEqual(kept, [])
        self.assertTrue(np.all(out[8:] == LABEL_ROBOT_ARM))

    def test_mesh_mask_paints_arc_not_full_width(self):
        labels = np.zeros((20, 20), dtype=np.uint8)
        labels[:, :] = LABEL_CARGO
        body = np.zeros((20, 20), dtype=bool)
        body[16:, 6:14] = True
        dets = [
            {"label": LABEL_CARGO, "prompt": "box", "confidence": 0.2,
             "bbox": [0, 0, 20, 12]},
            {"label": LABEL_CARGO, "prompt": "panel", "confidence": 0.3,
             "bbox": [6, 16, 14, 20]},
        ]
        out, kept, _, n_self = apply_self_body_mask(labels, dets, body)
        self.assertEqual(n_self, 4 * 8)
        self.assertTrue(np.all(out[16:, 6:14] == LABEL_ROBOT_ARM))
        self.assertTrue(np.all(out[:16] == LABEL_CARGO))
        self.assertEqual(len(kept), 1)
        self.assertEqual(kept[0]["prompt"], "box")

    def test_bbox_overlap_and_dropped_list(self):
        body = np.zeros((20, 20), dtype=bool)
        body[16:, :] = True
        before = [
            {"label": LABEL_CARGO, "prompt": "box", "confidence": 0.4,
             "bbox": [0, 0, 10, 10]},
            {"label": LABEL_CARGO, "prompt": "panel", "confidence": 0.3,
             "bbox": [0, 16, 20, 20]},
        ]
        after = [before[0]]
        dropped = detections_dropped_by_self_body(before, after, body)
        self.assertEqual(len(dropped), 1)
        self.assertEqual(dropped[0]["prompt"], "panel")
        self.assertGreaterEqual(dropped[0]["self_body_overlap"], 0.99)
        self.assertAlmostEqual(bbox_mask_overlap([0, 0, 10, 10], body), 0.0)

    def test_update_applies_config_frac(self):
        seg = build_segmenter({
            "backend": "stub",
            "prompts": ["x"],
            "self_body_row_start_frac": 0.85,
        })
        rgb = np.zeros((100, 10, 3), dtype=np.uint8)
        seg.update(rgb, stamp=1.0, frame_id="f")
        out = seg.copy_output()
        self.assertTrue(np.all(out.label_map[85:] == LABEL_ROBOT_ARM))
        self.assertTrue(np.all(out.label_map[:85] == LABEL_BACKGROUND))
        self.assertEqual(out.stats["self_body_pixels"], 15 * 10)
        self.assertEqual(out.stats["label_counts"][LABEL_ROBOT_ARM], 150)


class TestCombinedSelfBody(unittest.TestCase):
    """Panel flanks outside the mesh silhouette must still be dropped."""

    def test_union_covers_flanks_the_mesh_misses(self):
        from luggage_perception.semantic_segmenter import (
            combined_self_body_mask)
        mesh = np.zeros((480, 640), dtype=bool)
        mesh[424:, 202:481] = True          # measured silhouette extent
        body = combined_self_body_mask(mesh, 0.85, (480, 640))
        self.assertTrue(body[440, 100])     # left flank, mesh missed it
        self.assertTrue(body[440, 600])     # right flank
        self.assertTrue(body[430, 300])     # still covers the mesh itself
        self.assertFalse(body[392, 300])    # suitcase rows stay free

    def test_no_mesh_falls_back_to_band(self):
        from luggage_perception.semantic_segmenter import (
            combined_self_body_mask)
        body = combined_self_body_mask(None, 0.85, (480, 640))
        self.assertTrue(body[408:].all())
        self.assertFalse(body[:408].any())

    def test_disabled_when_no_mesh_and_no_band(self):
        from luggage_perception.semantic_segmenter import (
            combined_self_body_mask)
        self.assertFalse(combined_self_body_mask(None, 0.0, (48, 64)).any())

    def test_update_drops_flank_cargo_and_reports_miss(self):
        from luggage_perception.semantic_segmenter import SemanticSegmenter
        from luggage_perception.detection_temporal_gate import (
            DetectionTemporalGate)

        class _PanelOnly(SemanticSegmenter):
            """Reproduces trial_03: only the panel flanks score cargo."""

            def segment(self, rgb_image):
                labels = np.zeros(rgb_image.shape[:2], dtype=np.uint8)
                labels[431:480, 17:256] = LABEL_CARGO
                labels[431:480, 431:570] = LABEL_CARGO
                return labels, [
                    {"label": LABEL_CARGO, "prompt": "platform",
                     "confidence": 0.17, "bbox": [17, 431, 256, 480]},
                    {"label": LABEL_CARGO, "prompt": "platform",
                     "confidence": 0.17, "bbox": [431, 431, 570, 480]},
                ]

        seg = _PanelOnly(["platform"], {"platform": LABEL_CARGO})
        seg.self_body_row_start_frac = 0.85
        mesh = np.zeros((480, 640), dtype=bool)
        mesh[424:, 202:481] = True
        seg.self_body_mask = mesh
        seg.temporal_gate = DetectionTemporalGate(window_size=5)

        seg.update(np.full((480, 640, 3), 40, dtype=np.uint8), 1.0, "f")
        out = seg.copy_output()
        self.assertEqual(int((out.label_map == LABEL_CARGO).sum()), 0)
        self.assertEqual(out.detections, ())
        # The vote must now see a genuine miss, not a phantom cargo frame.
        self.assertFalse(out.stats["temporal"]["raw_cargo"])
        self.assertFalse(out.stats["temporal"]["held"])
        self.assertFalse(out.stats["raw_cargo"])
        self.assertEqual(out.stats["n_yolo_cargo_before_self_body"], 2)
        self.assertEqual(out.stats["n_dropped_self_body"], 2)
        dropped = out.stats["detections_dropped_self_body"]
        self.assertEqual(len(dropped), 2)
        for item in dropped:
            self.assertEqual(item["dropped"], "self_body")
            self.assertGreaterEqual(item["self_body_overlap"], 0.5)
            self.assertIn("bbox", item)
        self.assertFalse(out.stats["held"])

    def test_update_premasks_self_body_before_segment(self):
        """A backend that only fires on bright panel pixels must see grey."""
        from luggage_perception.semantic_segmenter import SemanticSegmenter

        class _BrightPanel(SemanticSegmenter):
            def segment(self, rgb_image):
                labels = np.zeros(rgb_image.shape[:2], dtype=np.uint8)
                dets = []
                bottom = rgb_image[85:, :, 0]
                if bottom.size and float(bottom.mean()) > 200.0:
                    labels[85:] = LABEL_CARGO
                    dets.append({
                        "label": LABEL_CARGO, "prompt": "panel",
                        "confidence": 0.9, "bbox": [0, 85, 10, 100],
                    })
                labels[10:40, :] = LABEL_CARGO
                dets.insert(0, {
                    "label": LABEL_CARGO, "prompt": "box",
                    "confidence": 0.9, "bbox": [0, 10, 10, 40],
                })
                return labels, dets

        seg = _BrightPanel(["box"], {"box": LABEL_CARGO})
        seg.self_body_row_start_frac = 0.85
        rgb = np.full((100, 10, 3), 255, dtype=np.uint8)
        seg.update(rgb, stamp=1.0, frame_id="f")
        out = seg.copy_output()
        prompts = [d["prompt"] for d in out.detections]
        self.assertIn("box", prompts)
        self.assertNotIn("panel", prompts)
        self.assertTrue(np.all(out.label_map[85:] != LABEL_CARGO))


class TestUpdateTemporalHold(unittest.TestCase):
    def test_update_holds_bbox_on_flicker_miss(self):
        from luggage_perception.semantic_segmenter import SemanticSegmenter
        from luggage_perception.detection_temporal_gate import (
            DetectionTemporalGate)

        class _Flicker(SemanticSegmenter):
            def __init__(self):
                super().__init__(["box"], {"box": LABEL_CARGO})
                self.n = 0

            def segment(self, rgb_image):
                self.n += 1
                labels = np.zeros(rgb_image.shape[:2], dtype=np.uint8)
                if self.n <= 3:
                    labels[8:30, 10:40] = LABEL_CARGO
                    return labels, [{
                        "label": LABEL_CARGO, "prompt": "box",
                        "confidence": 0.4, "bbox": [10, 8, 40, 30],
                    }]
                return labels, []

        seg = _Flicker()
        seg.temporal_gate = DetectionTemporalGate(window_size=5)
        rgb = np.full((48, 64, 3), 40, dtype=np.uint8)
        for i in range(3):
            seg.update(rgb, stamp=float(i), frame_id="f")
            self.assertFalse(
                seg.copy_output().stats.get("temporal", {}).get("held"))
        seg.update(rgb, stamp=3.0, frame_id="f")
        out = seg.copy_output()
        self.assertTrue(out.stats["temporal"]["held"])
        self.assertGreater(int((out.label_map == LABEL_CARGO).sum()), 0)
        self.assertTrue(any(d.get("held") for d in out.detections))

