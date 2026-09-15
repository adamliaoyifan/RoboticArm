#!/usr/bin/env python3
"""Vintage-pose segmenter regression (PF-R5 rework).

docs/agents/reviews/2026-09-04_1943_pfr5-vintage-segmenter-decision.md:
the vintage/loafbrr suitcase drops below a 0.04 YOLO floor
at diagonal-yaw poses in one lighting state (0.01-0.03 measured on the
2026-09-04 16-pose sweep). The descriptive prompt added to
``semantic_segmenter.yaml`` lifts those exact frames to 0.17-0.23.

The shipped production floor is 0.2 (PF-R10 G6 fail-close decision) and
deliberately excludes ``fail_pose_yaw67`` (measured best 0.187): that gap
is a known sim-texture miss covered by the eval-only
SIM_TEXTURE_LOW_CONFIDENCE waiver, NOT by lowering thresholds
(docs/agents/discuss/2026-09-14_0908_pf-r10-g6-sim-texture-amendment.md;
segmenter-only remediation is exhausted per
docs/agents/reviews/2026-09-04_2049_pf-r5-sim-texture-not-visible.md).
This test therefore probes at the historical PF-R5 0.04 floor and asserts
the lift (>= 0.10) — it measures model behaviour, not the production
gate.

The fixtures under ``fixtures/vintage_pose/`` are the two failing frames
(``fail_pose_*``) and two healthy frames (``ok_pose_*``) from that
sweep, saved verbatim from the preprocessed color image.

Requires the untracked YOLO-World weights and a working GPU segmenter;
the detection tests skip (not error) when either is absent. The
prompt-config test always runs. Run locally with:
  python3 -m pytest test/test_vintage_pose_regression.py -v
"""

import importlib.util
import os
import unittest

import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_FIXTURES = os.path.join(_HERE, "fixtures", "vintage_pose")
_CONFIG = os.path.join(_HERE, "..", "config", "semantic_segmenter.yaml")

# Historical PF-R5 probe floor and regression margin. Single source of
# truth for both detection tests.
PROBE_CONFIDENCE_FLOOR = 0.04
FAILING_LIFT_FLOOR = 0.10

FRAMES = [
    "fail_pose_yaw109.png",
    "fail_pose_yaw67.png",
    "ok_pose_yaw29.png",
    "ok_pose_yaw4.png",
]


def _yolo_world_weight():
    """Absolute path to yolov8s-world.pt, or None.

    The weights are untracked (``*.pt`` is gitignored); ultralytics
    resolves a bare model name against the CWD and would silently
    download 27 MB into the workspace, so only absolute paths are used.
    """
    env = os.environ.get("LUGGAGE_YOLO_WORLD_WEIGHT", "").strip()
    if env and os.path.isfile(env):
        return os.path.abspath(env)
    for root in (os.path.join(_HERE, "..", "..", ".."),
                 os.path.join(_HERE, "..")):
        candidate = os.path.join(root, "yolov8s-world.pt")
        if os.path.isfile(candidate):
            return os.path.abspath(candidate)
    return None


_ULTRALYTICS = importlib.util.find_spec("ultralytics") is not None
_CV2 = importlib.util.find_spec("cv2") is not None

try:
    import torch
    _CUDA = torch.cuda.is_available()
except Exception:  # noqa: BLE001 - optional heavyweight import
    _CUDA = False

_WEIGHT = _yolo_world_weight()


def _clip_vendor_dir():
    """Vendored CLIP tree for YOLO-World text embedding, or None.

    Mirrors ``semantic_segmenter._setup_clip_vendor`` lookup (env
    override first, then ``<pkg>/../vendor``); the vendor is untracked
    (``scripts/setup_clip_vendor.sh`` provisions it). Without it the
    YOLO-World backend falls back to ultralytics' online auto-install,
    which is not a test prerequisite.
    """
    env = os.environ.get("LUGGAGE_CLIP_VENDOR_DIR", "").strip()
    if env and os.path.isdir(env):
        return env
    candidate = os.path.normpath(os.path.join(
        _HERE, "..", "luggage_perception", "..", "vendor"))
    return candidate if os.path.isdir(candidate) else None


_CLIP_VENDOR = _clip_vendor_dir()


def _build():
    import cv2  # noqa: F401 - capability guard covered by class skip
    from luggage_perception.semantic_segmenter import BboxFillSegmenter
    seg = yaml.safe_load(open(_CONFIG))["semantic_segmenter"]["ros__parameters"]
    return BboxFillSegmenter(
        prompts=seg["prompts"],
        class_mapping=dict(zip(seg["prompts"], seg["class_mapping_labels"])),
        confidence_threshold=PROBE_CONFIDENCE_FLOOR,
        model_name=_WEIGHT,
        device="cuda")


class TestVintagePromptConfig(unittest.TestCase):

    def test_config_contains_the_descriptive_prompt(self):
        seg = yaml.safe_load(open(_CONFIG))["semantic_segmenter"][
            "ros__parameters"]
        self.assertIn(
            "vintage leather suitcase with handles viewed from directly "
            "above", seg["prompts"])
        self.assertEqual(len(seg["prompts"]),
                         len(seg["class_mapping_labels"]))


@unittest.skipUnless(
    _ULTRALYTICS and _CV2, "ultralytics/cv2 unavailable")
@unittest.skipUnless(_CUDA, "no CUDA device for YOLO-World")
@unittest.skipUnless(
    _WEIGHT is not None,
    "yolov8s-world.pt not found; set LUGGAGE_YOLO_WORLD_WEIGHT")
@unittest.skipUnless(
    _CLIP_VENDOR is not None,
    "vendored CLIP not found; run scripts/setup_clip_vendor.sh or set "
    "LUGGAGE_CLIP_VENDOR_DIR")
class TestVintagePoseSegmenterRegression(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.segmenter = _build()

    def _central_luggage_boxes(self, frame):
        import cv2
        img = cv2.cvtColor(
            cv2.imread(os.path.join(_FIXTURES, frame)), cv2.COLOR_BGR2RGB)
        self.assertIsNotNone(img, frame)
        _, detections = self.segmenter.segment(img)
        return [d for d in detections
                if d["label"] == 2
                and d["bbox"][1] < 400            # not the bottom strip
                and d["bbox"][3] - d["bbox"][1] > 60]

    def test_every_pose_has_a_central_luggage_box(self):
        """Every fixture detects the suitcase at or above the historical
        probe floor. This is not the production gate: the shipped 0.2
        floor in ``semantic_segmenter.yaml`` separately excludes
        ``fail_pose_yaw67`` (0.187) as a documented sim-texture miss."""
        for frame in FRAMES:
            boxes = self._central_luggage_boxes(frame)
            self.assertTrue(
                boxes, "%s: no central luggage detection" % frame)
            best = max(d["confidence"] for d in boxes)
            self.assertGreaterEqual(best, PROBE_CONFIDENCE_FLOOR, frame)

    def test_failing_poses_lifted_well_above_threshold(self):
        """Regression margin: the former failures should sit comfortably
        above the historical floor (>= 0.10), not scrape it."""
        for frame in ("fail_pose_yaw109.png", "fail_pose_yaw67.png"):
            boxes = self._central_luggage_boxes(frame)
            best = max((d["confidence"] for d in boxes), default=0.0)
            self.assertGreaterEqual(best, FAILING_LIFT_FLOOR, frame)


if __name__ == "__main__":
    unittest.main()
