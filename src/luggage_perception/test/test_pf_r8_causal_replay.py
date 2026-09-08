"""PF-R8 A3: causal offline replay of the repaired temporal hold.

Drives ``DetectionTemporalGate`` causally over the 155 captured failure
frames of the PF-R6 gen3 failed-case capture, in file order, with the
production accepted-detection predicate classifying each cargo detection
under the frozen nadir fixture geometry.

Documented assumptions (plan A3):

- The capture stores detection metadata, not RGB frames and label maps, so
  each label map is reconstructed as the union of bbox rectangles — exact
  for the captured ``bbox_fill`` backend, whose label map *is* that union.
- 20 frames have an RGB snapshot and use it for the scene-change signal;
  the remaining frames feed a constant image, i.e. a no-scene-change
  signal.
- No epoch signal exists in the capture, so the replay issues no epoch
  resets (the live node resets the gate on /luggage/current_box changes;
  tested separately in test_pf_r8_acceptance).

Required outcomes: 0 acausal holds, 0 missed holds, and the count of
target frames unrecoverable by any causal 5-frame hold is reported (that
count belongs to A4, not to the gate).
"""

import json
import os
import unittest

import numpy as np

from luggage_perception.detection_temporal_gate import DetectionTemporalGate
from luggage_perception.semantic_segmenter import (
    LABEL_CARGO,
    evaluate_detection_acceptance,
)

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
CAPTURE = os.path.join(
    REPO_ROOT, "docs", "status", "evidence", "platform_free_height",
    "2026-09-07_pfr6_gen3", "failed_case_capture", "failed_cases.jsonl")
IMAGES = os.path.dirname(CAPTURE) if os.path.exists(CAPTURE) else None
FIXTURE = os.path.join(
    os.path.dirname(__file__), "fixtures", "pf_r8", "acceptance_fixture.json")


def _load_ctx():
    with open(FIXTURE, encoding="utf-8") as handle:
        data = json.load(handle)
    cam, ws = data["camera"], data["workspace"]
    from luggage_perception.semantic_segmenter import (
        WorkspaceAcceptanceContext)
    return WorkspaceAcceptanceContext(
        fx=float(cam["fx"]), fy=float(cam["fy"]),
        cx=float(cam["cx"]), cy=float(cam["cy"]),
        plane_z=float(ws["plane_z"]),
        center_xy=tuple(float(v) for v in ws["center_xy"]),
        half_xy=tuple(float(v) for v in ws["half_xy"]),
        margin=float(ws["margin"]),
        optical_to_world=tuple(
            tuple(float(v) for v in row) for row in cam["optical_to_world"]))


def _snapshot_index():
    """Map capture stamp -> RGB snapshot path for the 20 captured images."""
    index = {}
    images_dir = os.path.join(
        os.path.dirname(CAPTURE), "images")
    if not os.path.isdir(images_dir):
        return index
    for name in os.listdir(images_dir):
        if not name.endswith(".png") or not name.startswith("fail_"):
            continue
        # fail_003_54.318000000.png -> stamp 54.318000000
        parts = name[:-len(".png")].split("_", 2)
        if len(parts) == 3:
            index[parts[2]] = os.path.join(images_dir, name)
    return index


_SNAPSHOTS = _snapshot_index()
_CONSTANT_RGB = None


def _snapshot_for(row):
    """(rgb, is_snapshot): captured RGB when available, else a constant image.

    The constant image yields MAD=0 against itself, i.e. the documented
    no-scene-change signal.
    """
    path = _SNAPSHOTS.get(str(row.get("stamp", "")))
    if path is not None:
        try:
            import cv2  # noqa: WPS433 present wherever the overlay runs
            img = cv2.imread(path)
            if img is not None:
                return np.ascontiguousarray(img[:, :, ::-1]), True  # BGR->RGB
        except ImportError:
            pass
    return np.full((480, 640, 3), 128, dtype=np.uint8), False


def _label_map_from_detections(dets, shape=(480, 640)):
    labels = np.zeros(shape, dtype=np.uint8)
    for det in dets:
        x1, y1, x2, y2 = (int(v) for v in det["bbox"])
        labels[max(0, y1):max(0, y2), max(0, x1):max(0, x2)] = int(
            det.get("label", 0))
    return labels


def replay():
    """Run the causal replay; returns (rows_out, summary)."""
    ctx = _load_ctx()
    gate = DetectionTemporalGate(window_size=5, min_positive_ratio=0.5)
    with open(CAPTURE, encoding="utf-8") as handle:
        rows = [json.loads(line) for line in handle if line.strip()]
    out = []
    n_snapshots = 0
    for idx, row in enumerate(rows):
        row["_index"] = idx
        dets = []
        for det in (row.get("yolo") or {}).get("detections", []):
            item = dict(det)
            item["label"] = int(det.get("label", -1))
            item["confidence"] = float(det.get("conf", 0.0))
            if item["label"] == LABEL_CARGO:
                accepted, reason = evaluate_detection_acceptance(
                    item["bbox"], ctx)
                item["accepted"] = accepted
                item["accept_reason"] = reason
            dets.append(item)
        rgb, is_snapshot = _snapshot_for(row)
        if is_snapshot:
            n_snapshots += 1
        label_map = _label_map_from_detections(dets)
        _, _, stats = gate.apply(label_map, dets, rgb)
        out.append({
            "index": idx,
            "stamp": row.get("stamp"),
            "miss": not stats["accepted_cargo"],
            "held": bool(stats["held"]),
            "positive_frames": int(stats["positive_frames"]),
            "ratio": float(stats["positive_ratio"]),
            "snapshot": bool(is_snapshot),
        })
    miss = [r for r in out if r["miss"]]
    held = [r for r in miss if r["held"]]
    unrecoverable = [r for r in miss if r["ratio"] + 1e-12 < 0.5]
    summary = {
        "frames": len(out),
        "snapshot_frames": n_snapshots,
        "miss_frames": len(miss),
        "held_frames": len(held),
        "unrecoverable_by_causal_5frame_hold": len(unrecoverable),
        "unrecoverable_indices": [r["index"] for r in unrecoverable],
    }
    return out, summary


class TestCausalReplay(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not os.path.exists(CAPTURE):
            raise unittest.SkipTest("capture jsonl not present: %s" % CAPTURE)
        cls.rows, cls.summary = replay()

    def test_zero_acausal_holds(self):
        # Every held frame's hold must derive from prior accepted positives
        # that entered the window before this frame.
        bad = [r for r in self.rows if r["held"] and r["positive_frames"] < 1]
        self.assertEqual([], bad)

    def test_zero_missed_holds(self):
        # Every miss frame whose (causally built) window reaches the
        # majority ratio must emit a hold.
        qualifying = [r for r in self.rows
                      if r["miss"] and r["ratio"] + 1e-12 >= 0.5]
        missed = [r for r in qualifying if not r["held"]]
        self.assertEqual([], missed,
                         "missed holds at indices %r" % missed[:8])

    def test_replay_shape_and_reported_unrecoverable(self):
        # 155 captured frames; the unrecoverable count is reported for A4,
        # recovering those frames is explicitly NOT required of the gate.
        self.assertEqual(155, self.summary["frames"])
        self.assertGreaterEqual(self.summary["snapshot_frames"], 18)
        print("\nA3 replay summary: %s" % json.dumps(self.summary))
        self.assertGreaterEqual(self.summary["unrecoverable_by_causal_5frame_hold"], 1)


if __name__ == "__main__":
    unittest.main()
