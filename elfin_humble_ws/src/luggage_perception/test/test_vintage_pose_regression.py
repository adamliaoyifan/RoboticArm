#!/usr/bin/env python3
"""Vintage-pose segmenter regression (PF-R5 rework).

docs/agents/reviews/2026-09-04_1943_pfr5-vintage-segmenter-decision.md:
the vintage/loafbrr suitcase drops below the 0.04 confidence threshold
at diagonal-yaw poses in one lighting state (0.01-0.03 measured on the
2026-09-04 16-pose sweep). The descriptive prompt added to
``semantic_segmenter.yaml`` lifts those exact frames to 0.17-0.23.

The fixtures under ``fixtures/vintage_pose/`` are the two failing frames
(``fail_pose_*``) and two healthy frames (``ok_pose_*``) from that
sweep, saved verbatim from the preprocessed color image. The test runs
the real BboxFillSegmenter with the shipped config and asserts a
central, luggage-labelled, above-threshold box exists in every fixture.

Requires the YOLO-World weights and a working GPU segmenter; skipped
otherwise (CI/stub environments). Run locally with:
  python3 -m pytest test/test_vintage_pose_regression.py -v
"""

import os
import unittest

import pytest

pytest.importorskip("ultralytics")
pytest.importorskip("cv2")

try:
    import torch
    _CUDA = torch.cuda.is_available()
except Exception:  # noqa: BLE001
    _CUDA = False
pytest.mark.skipif(not _CUDA, reason="no CUDA device for YOLO-World")

import cv2  # noqa: E402
import yaml  # noqa: E402

from luggage_perception.semantic_segmenter import BboxFillSegmenter  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_FIXTURES = os.path.join(_HERE, "fixtures", "vintage_pose")
_CONFIG = os.path.join(_HERE, "..", "config", "semantic_segmenter.yaml")

FRAMES = [
    "fail_pose_yaw109.png",
    "fail_pose_yaw67.png",
    "ok_pose_yaw29.png",
    "ok_pose_yaw4.png",
]


def _build():
    seg = yaml.safe_load(open(_CONFIG))["semantic_segmenter"]["ros__parameters"]
    return BboxFillSegmenter(
        prompts=seg["prompts"],
        class_mapping=dict(zip(seg["prompts"], seg["class_mapping_labels"])),
        confidence_threshold=float(seg["confidence_threshold"]),
        model_name="yolov8s-world.pt",
        device="cuda")


class TestVintagePoseRegression(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.segmenter = _build()

    def _central_luggage_boxes(self, frame):
        img = cv2.cvtColor(
            cv2.imread(os.path.join(_FIXTURES, frame)), cv2.COLOR_BGR2RGB)
        self.assertIsNotNone(img, frame)
        _, detections = self.segmenter.segment(img)
        return [d for d in detections
                if d["label"] == 2
                and d["bbox"][1] < 400            # not the bottom strip
                and d["bbox"][3] - d["bbox"][1] > 60]

    def test_every_pose_has_a_central_luggage_box(self):
        """The two former sub-threshold poses must now detect the
        suitcase above the unchanged 0.04 threshold."""
        for frame in FRAMES:
            boxes = self._central_luggage_boxes(frame)
            self.assertTrue(
                boxes, "%s: no central luggage detection" % frame)
            best = max(d["confidence"] for d in boxes)
            self.assertGreaterEqual(best, 0.04, frame)

    def test_failing_poses_lifted_well_above_threshold(self):
        """Regression margin: the former failures should sit comfortably
        above the threshold (>= 0.10), not scrape it."""
        for frame in ("fail_pose_yaw109.png", "fail_pose_yaw67.png"):
            boxes = self._central_luggage_boxes(frame)
            best = max((d["confidence"] for d in boxes), default=0.0)
            self.assertGreaterEqual(best, 0.10, frame)

    def test_config_contains_the_descriptive_prompt(self):
        seg = yaml.safe_load(open(_CONFIG))["semantic_segmenter"][
            "ros__parameters"]
        self.assertIn(
            "vintage leather suitcase with handles viewed from directly "
            "above", seg["prompts"])
        self.assertEqual(len(seg["prompts"]),
                         len(seg["class_mapping_labels"]))


if __name__ == "__main__":
    unittest.main()
