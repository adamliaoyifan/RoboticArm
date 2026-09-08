"""PF-R8 A1/A5: accepted-detection predicate and temporal-hold repair.

Fixture: ``fixtures/pf_r8/acceptance_fixture.json`` — the 440 cargo
detection instances captured in the PF-R6 gen3 failed-case capture
(319 right-edge static-structure negatives, 120 suitcase positives) plus
one geometrically-derived edge-clipped true positive (no real captured
instance exists; provenance recorded in the fixture). Camera geometry is
the nadir pickup_observe pose documented in ``robot_poses.yaml.example``
(camera at world (-1.0, 0.0, 1.9), optical +Z down, centred on the
platform), which the yaw-free radial predicate decides exactly.
"""

import json
import os
import unittest

import numpy as np

from luggage_perception.detection_temporal_gate import (
    DetectionTemporalGate,
    bbox_iou,
    largest_cargo_bbox,
)
from luggage_perception.semantic_segmenter import (
    LABEL_CARGO,
    WorkspaceAcceptanceContext,
    evaluate_detection_acceptance,
)

FIXTURE = os.path.join(
    os.path.dirname(__file__), "fixtures", "pf_r8", "acceptance_fixture.json")


def _ctx_from_fixture(cam, ws):
    return WorkspaceAcceptanceContext(
        fx=float(cam["fx"]), fy=float(cam["fy"]),
        cx=float(cam["cx"]), cy=float(cam["cy"]),
        plane_z=float(ws["plane_z"]),
        center_xy=tuple(float(v) for v in ws["center_xy"]),
        half_xy=tuple(float(v) for v in ws["half_xy"]),
        margin=float(ws["margin"]),
        optical_to_world=tuple(
            tuple(float(v) for v in row) for row in cam["optical_to_world"]),
    )


def _load_fixture():
    with open(FIXTURE, encoding="utf-8") as handle:
        return json.load(handle)


class TestDetectionAcceptancePredicate(unittest.TestCase):
    """A1: 0 false accepts, 0 false rejects on the frozen fixture."""

    @classmethod
    def setUpClass(cls):
        data = _load_fixture()
        cls.ctx = _ctx_from_fixture(data["camera"], data["workspace"])
        cls.entries = data["entries"]

    def test_zero_false_accepts_and_rejects(self):
        wrong = []
        for entry in self.entries:
            accepted, reason = evaluate_detection_acceptance(
                entry["bbox"], self.ctx)
            if accepted != bool(entry["expected_accepted"]):
                wrong.append((entry["bbox"], entry["expected_accepted"],
                              accepted, reason))
        self.assertEqual([], wrong[:8],
                         "first mismatches: %r (total %d)"
                         % (wrong[:8], len(wrong)))

    def test_captured_false_positive_bboxes_rejected(self):
        for bbox in ([611, 112, 640, 295], [620, 119, 640, 291],
                     [611, 111, 640, 296], [606, 116, 640, 365]):
            accepted, reason = evaluate_detection_acceptance(bbox, self.ctx)
            self.assertFalse(accepted, bbox)
            self.assertEqual("outside_workspace", reason)

    def test_in_region_suitcase_accepted(self):
        # The suitcase detection observed at t=59.467 s in the capture.
        accepted, reason = evaluate_detection_acceptance(
            [178, 144, 405, 297], self.ctx)
        self.assertTrue(accepted)
        self.assertEqual("in_workspace", reason)

    def test_edge_clipped_positive_accepted(self):
        # Border-touching (x2 = 639) but the visible centre still projects
        # onto the platform: accepted by the projection rule where a pure
        # border test would fail closed on a real clipping case.
        entry = next(e for e in self.entries
                     if e["source"] == "synthetic_edge_clipped_large_box_derived")
        self.assertGreaterEqual(entry["bbox"][2], 639)
        accepted, reason = evaluate_detection_acceptance(entry["bbox"], self.ctx)
        self.assertTrue(accepted)
        self.assertEqual("in_workspace", reason)

    def test_unavailable_context_fails_open_flagged(self):
        accepted, reason = evaluate_detection_acceptance(
            [611, 112, 640, 295], None)
        self.assertTrue(accepted)
        self.assertEqual("predicate_unavailable", reason)
        empty = WorkspaceAcceptanceContext(
            fx=0.0, fy=0.0, cx=0.0, cy=0.0, plane_z=0.86,
            center_xy=(-1.0, 0.0), half_xy=(0.5, 0.5), margin=0.15)
        accepted, reason = evaluate_detection_acceptance(
            [611, 112, 640, 295], empty)
        self.assertTrue(accepted)
        self.assertEqual("predicate_unavailable", reason)

    def test_horizontal_ray_rejected_not_invented(self):
        # Optical axis horizontal: the ray never descends to the plane.
        # That is a decision (reject), not a fabricated intersection.
        ctx = WorkspaceAcceptanceContext(
            fx=337.22, fy=337.22, cx=320.0, cy=240.0, plane_z=0.86,
            center_xy=(-1.0, 0.0), half_xy=(0.5, 0.5), margin=0.15,
            optical_to_world=((1.0, 0.0, 0.0, 0.0),
                              (0.0, 0.0, 1.0, 0.0),
                              (0.0, -1.0, 0.0, 2.0)))
        accepted, reason = evaluate_detection_acceptance(
            [100, 100, 200, 200], ctx)
        self.assertFalse(accepted)
        self.assertEqual("ray_off_plane", reason)


class TestGateAcceptedOnlySemantics(unittest.TestCase):
    """A2/A5: the hold fires with a persistent unaccepted detection."""

    @staticmethod
    def _det(bbox, conf=0.25, accepted=True, label=LABEL_CARGO):
        det = {
            "label": label,
            "prompt": "suitcase",
            "confidence": conf,
            "bbox": bbox,
        }
        if accepted is not None:
            det["accepted"] = bool(accepted)
            det["accept_reason"] = (
                "in_workspace" if accepted else "outside_workspace")
        return det

    @staticmethod
    def _label_map(had_bbox=None):
        labels = np.zeros((480, 640), dtype=np.uint8)
        if had_bbox is not None:
            x1, y1, x2, y2 = had_bbox
            labels[y1:y2, x1:x2] = LABEL_CARGO
        return labels

    def _steady_rgb(self):
        rng = np.random.default_rng(7)
        base = rng.integers(100, 120, size=(480, 640, 3), dtype=np.uint8)
        return base

    def test_hold_fires_despite_persistent_unaccepted_detection(self):
        gate = DetectionTemporalGate(window_size=5, min_positive_ratio=0.5)
        rgb = self._steady_rgb()
        suitcase = [178, 144, 405, 297]
        fp = [611, 112, 640, 295]
        for _ in range(5):
            _, _, stats = gate.apply(
                self._label_map(suitcase),
                [self._det(suitcase), self._det(fp, conf=0.08, accepted=False)],
                rgb)
            self.assertTrue(stats["accepted_cargo"])
        # Suitcase disappears; only the rejected false positive remains.
        # Before PF-R8 this frame was "positive" (had = any cargo bbox) and
        # the hold could never fire.
        _, dets, stats = gate.apply(
            self._label_map(),
            [self._det(fp, conf=0.08, accepted=False)],
            rgb)
        self.assertFalse(stats["raw_cargo"] is False)  # raw cargo still seen
        self.assertTrue(stats["raw_cargo"])
        self.assertFalse(stats["accepted_cargo"])
        self.assertTrue(stats["held"], stats)
        held_bbox = stats["held_bbox"]
        self.assertGreater(bbox_iou(held_bbox, suitcase), 0.9)
        self.assertTrue(any(d.get("held") for d in dets))

    def test_window_reset_ignores_suitcase_false_positive_alternation(self):
        gate = DetectionTemporalGate(window_size=5, min_positive_ratio=0.5)
        rgb = self._steady_rgb()
        suitcase = [178, 144, 405, 297]
        fp = [611, 112, 640, 295]
        # Two accepted suitcase frames build a window; the unaccepted FP
        # appearing/disappearing between them must not clear it. A miss
        # frame records had_cargo=False with bbox=None — never the FP.
        gate.apply(self._label_map(suitcase), [self._det(suitcase)], rgb)
        _, _, stats_miss = gate.apply(
            self._label_map(),
            [self._det(fp, conf=0.08, accepted=False)], rgb)
        gate.apply(self._label_map(suitcase), [self._det(suitcase)], rgb)
        self.assertFalse(stats_miss["scene_change"])
        self.assertEqual(3, len(gate._window))
        self.assertEqual(
            [True, False, True],
            [rec["had_cargo"] for rec in gate._window])
        self.assertEqual(
            [suitcase, None, suitcase],
            [rec["bbox"] for rec in gate._window])

    def test_alternating_largest_bbox_does_not_clear_window(self):
        # Largest-area bbox alternates between the suitcase and a larger
        # unaccepted detection; window identity stays on the suitcase and
        # no alternation-driven scene_change fires.
        gate = DetectionTemporalGate(window_size=5, min_positive_ratio=0.5)
        rgb = self._steady_rgb()
        suitcase = [178, 144, 405, 297]
        big_fp = [20, 20, 630, 460]  # larger area, rejected
        for _ in range(3):
            gate.apply(self._label_map(suitcase), [self._det(suitcase)], rgb)
        for flip in range(4):
            det = self._det(big_fp, accepted=False) if flip % 2 else \
                self._det(suitcase)
            gate.apply(
                self._label_map(suitcase if flip % 2 == 0 else None),
                [det], rgb)
        self.assertEqual(5, len(gate._window))  # never cleared, maxlen 5
        self.assertEqual(
            [suitcase, suitcase, None, suitcase, None],
            [rec["bbox"] for rec in gate._window])

    def test_epoch_change_clears_hold(self):
        gate = DetectionTemporalGate(window_size=5, min_positive_ratio=0.5)
        rgb = self._steady_rgb()
        suitcase = [178, 144, 405, 297]
        for _ in range(5):
            gate.apply(self._label_map(suitcase), [self._det(suitcase)], rgb)
        # Epoch change: the node calls reset() on /luggage/current_box
        # generation change (semantic_segmenter_node._on_current_box).
        gate.reset()
        _, _, stats = gate.apply(self._label_map(), [], rgb)
        self.assertFalse(stats["held"])
        self.assertEqual(1, stats["window"])

    def test_hold_expires_to_empty_not_stale_bbox(self):
        gate = DetectionTemporalGate(window_size=5, min_positive_ratio=0.5)
        rgb = self._steady_rgb()
        suitcase = [178, 144, 405, 297]
        for _ in range(5):
            gate.apply(self._label_map(suitcase), [self._det(suitcase)], rgb)
        held_seen = False
        for _ in range(5):
            _, _, stats = gate.apply(self._label_map(), [], rgb)
            held_seen = held_seen or stats["held"]
        self.assertTrue(held_seen)
        # After k > window * (1 - ratio) consecutive misses the ratio drops
        # below 0.5 and the gate emits empty, never a stale bbox.
        _, _, stats = gate.apply(self._label_map(), [], rgb)
        self.assertFalse(stats["held"])
        self.assertLess(stats["positive_ratio"], 0.5)

    def test_largest_cargo_bbox_accepted_only(self):
        big_fp = [20, 20, 630, 460]  # area 264600 > suitcase 34731
        dets = [self._det(big_fp, accepted=False),
                self._det([178, 144, 405, 297])]
        self.assertEqual(big_fp, largest_cargo_bbox(dets, (LABEL_CARGO,)))
        self.assertEqual([178, 144, 405, 297],
                         largest_cargo_bbox(dets, (LABEL_CARGO,),
                                            accepted_only=True))
        # Missing 'accepted' key keeps historic accept-all behaviour.
        dets_legacy = [{"label": LABEL_CARGO, "prompt": "x",
                        "confidence": 0.5, "bbox": [611, 112, 640, 295]}]
        self.assertEqual([611, 112, 640, 295],
                         largest_cargo_bbox(dets_legacy, (LABEL_CARGO,),
                                            accepted_only=True))


class TestPredicateMarginSeparation(unittest.TestCase):
    """The accept radius must sit between suitcases and the pedestal."""

    def test_measured_separation(self):
        data = _load_fixture()
        ctx = _ctx_from_fixture(data["camera"], data["workspace"])
        from luggage_perception.semantic_segmenter import (
            bbox_center_on_plane)
        fp_point = bbox_center_on_plane([611, 112, 640, 295], ctx)
        suitcase_point = bbox_center_on_plane([178, 144, 405, 297], ctx)
        fp_dist = np.hypot(fp_point[0] - ctx.center_xy[0],
                           fp_point[1] - ctx.center_xy[1])
        suit_dist = np.hypot(suitcase_point[0] - ctx.center_xy[0],
                             suitcase_point[1] - ctx.center_xy[1])
        radius = max(ctx.half_xy) + ctx.margin
        # Capture-measured: pedestal ~0.95 m, suitcase centre ~0.18 m,
        # accept radius 0.65 m — separation on both sides.
        self.assertGreater(fp_dist, radius + 0.2, fp_dist)
        self.assertLess(suit_dist, radius - 0.3, suit_dist)


if __name__ == "__main__":
    unittest.main()
