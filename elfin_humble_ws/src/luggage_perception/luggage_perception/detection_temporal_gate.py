"""Multi-frame cargo-detection vote (no ROS).

YOLO on one RGB frame flickers: a miss publishes an empty mask, the point
filter sends no cargo, and DetectLuggage fails. This gate keeps a short
window of recent frames and, when the current frame has no cargo box but a
majority of the window did, paints the consensus bbox onto the current mask.

A scene-change (RGB jump or bbox IoU collapse) clears the window so a hold
cannot reuse the previous suitcase after spawn.
"""

from __future__ import division

from collections import deque

import numpy as np

# Must match semantic_segmenter.LABEL_CARGO. Kept local to avoid an import
# cycle (SemanticSegmenter.update calls this gate).
LABEL_CARGO = 2

# DetectLuggage should wait for a newer cloud on these reasons, not fall
# back to GT on the first attempt.
RETRYABLE_ESTIMATE_REASONS = (
    "DETECT_NO_CLOUD",
    "DETECT_STALE_CLOUD",
    "DETECT_TOO_FEW_POINTS",
    "DETECT_TF_FAILED",
    "DETECT_ESTIMATION_FAILED",
    "DETECT_LOW_CONFIDENCE",
)


def should_retry_estimate(reason):
    return str(reason or "") in RETRYABLE_ESTIMATE_REASONS


def bbox_iou(a, b):
    """Axis-aligned IoU for [x1, y1, x2, y2]. Empty/invalid -> 0."""
    if a is None or b is None or len(a) != 4 or len(b) != 4:
        return 0.0
    ax1, ay1, ax2, ay2 = (float(v) for v in a)
    bx1, by1, bx2, by2 = (float(v) for v in b)
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    if union <= 1e-9:
        return 0.0
    return inter / union


def _bbox_area(bbox):
    if bbox is None or len(bbox) != 4:
        return 0.0
    return max(0.0, float(bbox[2]) - float(bbox[0])) * max(
        0.0, float(bbox[3]) - float(bbox[1]))


def largest_cargo_bbox(detections, cargo_labels):
    """Largest-area cargo bbox in *detections*, or None."""
    best = None
    best_area = 0.0
    labels = set(int(v) for v in cargo_labels)
    for det in detections or []:
        if int(det.get("label", -1)) not in labels:
            continue
        bbox = det.get("bbox")
        area = _bbox_area(bbox)
        if area > best_area:
            best_area = area
            best = [int(round(float(v))) for v in bbox]
    return best


def median_bbox(bboxes):
    if not bboxes:
        return None
    arr = np.asarray(bboxes, dtype=np.float64)
    return [int(round(v)) for v in np.median(arr, axis=0)]


def _paint_bbox(label_map, bbox, label_id):
    height, width = label_map.shape[:2]
    x1, y1, x2, y2 = bbox
    ix1 = max(0, min(width, int(x1)))
    iy1 = max(0, min(height, int(y1)))
    ix2 = max(0, min(width, int(x2)))
    iy2 = max(0, min(height, int(y2)))
    if ix2 <= ix1 or iy2 <= iy1:
        return False
    label_map[iy1:iy2, ix1:ix2] = int(label_id)
    return True


def rgb_signature(rgb_uint8, out_hw=(36, 48)):
    """Small grayscale downsample for scene-change MAD (0-255)."""
    image = np.asarray(rgb_uint8)
    if image.ndim == 3:
        gray = image.astype(np.float64).mean(axis=2)
    else:
        gray = image.astype(np.float64)
    out_h, out_w = int(out_hw[0]), int(out_hw[1])
    height, width = gray.shape[:2]
    if height < 1 or width < 1 or out_h < 1 or out_w < 1:
        return np.zeros(out_hw, dtype=np.float64)
    # Center crop so platform/suitcase dominate, not the container wall.
    y0, y1 = int(height * 0.08), int(height * 0.85)
    x0, x1 = int(width * 0.08), int(width * 0.85)
    crop = gray[y0:y1, x0:x1]
    if crop.size == 0:
        crop = gray
    # Block-mean resize without cv2.
    ys = np.linspace(0, crop.shape[0], out_h + 1).astype(int)
    xs = np.linspace(0, crop.shape[1], out_w + 1).astype(int)
    out = np.zeros((out_h, out_w), dtype=np.float64)
    for row in range(out_h):
        for col in range(out_w):
            block = crop[ys[row]:max(ys[row] + 1, ys[row + 1]),
                         xs[col]:max(xs[col] + 1, xs[col + 1])]
            out[row, col] = float(block.mean()) if block.size else 0.0
    return out


def signature_mad(sig_a, sig_b):
    """Mean absolute difference of two rgb_signature arrays, or None."""
    if sig_a is None or sig_b is None:
        return None
    return float(np.mean(np.abs(np.asarray(sig_a) - np.asarray(sig_b))))


class SuitcaseViewWait:
    """Wait until RGB leaves the pre-spawn view and then settles.

    Arm with the camera frame present when the pickup box id changes (old
    suitcase or empty platform). Ready after a later frame differs by
    ``update_mad`` and ``stable_frames`` subsequent frames stay within
    ``stable_mad``. Clearing the box (empty id) cancels the wait.
    """

    def __init__(self, update_mad=10.0, stable_mad=4.0, stable_frames=2):
        self.update_mad = float(update_mad)
        self.stable_mad = float(stable_mad)
        self.stable_frames = max(1, int(stable_frames))
        self._box_id = None
        self._baseline = None
        self._pending = False
        self._seen_update = False
        self._stable_count = 0
        self._last_sig = None

    @property
    def box_id(self):
        return self._box_id

    @property
    def pending(self):
        return bool(self._pending)

    def note_box_id(self, box_id, rgb_uint8=None):
        """Arm or cancel on a current_box id change. True if state reset."""
        box_id = str(box_id or "")
        if box_id == (self._box_id or ""):
            return False
        self._box_id = box_id
        self._baseline = (
            rgb_signature(rgb_uint8) if rgb_uint8 is not None else None)
        self._pending = bool(box_id)
        self._seen_update = False
        self._stable_count = 0
        self._last_sig = self._baseline
        return True

    def set_baseline_if_missing(self, rgb_uint8):
        if self._baseline is not None or rgb_uint8 is None:
            return
        self._baseline = rgb_signature(rgb_uint8)
        self._last_sig = self._baseline

    def observe(self, rgb_uint8):
        """Feed one RGB frame. True when the new suitcase view has settled."""
        if not self._pending:
            return True
        if rgb_uint8 is None:
            return False
        sig = rgb_signature(rgb_uint8)
        if self._baseline is None:
            self._baseline = sig
            self._last_sig = sig
            return False
        if not self._seen_update:
            mad = signature_mad(sig, self._baseline)
            if mad is None or mad < self.update_mad:
                return False
            self._seen_update = True
            self._last_sig = sig
            self._stable_count = 1
            if self._stable_count >= self.stable_frames:
                self._pending = False
                return True
            return False
        mad_prev = signature_mad(sig, self._last_sig)
        if mad_prev is not None and mad_prev <= self.stable_mad:
            self._stable_count += 1
        else:
            self._stable_count = 1
        self._last_sig = sig
        if self._stable_count >= self.stable_frames:
            self._pending = False
            return True
        return False


class DetectionTemporalGate:
    """Sliding-window majority vote over cargo bboxes."""

    def __init__(self, window_size=5, min_positive_ratio=0.5,
                 scene_change_mad=10.0, bbox_iou_reset=0.3,
                 cargo_labels=None, min_frames=2, cargo_label=LABEL_CARGO):
        self.window_size = max(1, int(window_size))
        self.min_positive_ratio = min(1.0, max(0.0, float(min_positive_ratio)))
        self.scene_change_mad = float(scene_change_mad)
        self.bbox_iou_reset = float(bbox_iou_reset)
        self.cargo_labels = tuple(
            int(v) for v in (cargo_labels if cargo_labels is not None
                             else (LABEL_CARGO,)))
        self.min_frames = max(1, int(min_frames))
        self.cargo_label = int(cargo_label)
        self._window = deque(maxlen=self.window_size)
        self._last_sig = None

    def reset(self):
        self._window.clear()
        self._last_sig = None

    def apply(self, label_map, detections, rgb_uint8):
        """Return (label_map, detections, stats). Inputs are not mutated."""
        labels = np.asarray(label_map)
        dets = list(detections or [])
        stats = {
            "held": False,
            "scene_change": False,
            "window": 0,
            "positive_frames": 0,
            "positive_ratio": 0.0,
            "raw_cargo": False,
        }
        bbox = largest_cargo_bbox(dets, self.cargo_labels)
        had = bbox is not None
        stats["raw_cargo"] = bool(had)

        sig = rgb_signature(rgb_uint8)
        if (self._last_sig is not None
                and float(np.mean(np.abs(sig - self._last_sig)))
                >= self.scene_change_mad):
            self._window.clear()
            stats["scene_change"] = True
        self._last_sig = sig

        if had and self._window:
            prev = None
            for rec in reversed(self._window):
                if rec.get("bbox") is not None:
                    prev = rec["bbox"]
                    break
            if prev is not None and bbox_iou(bbox, prev) < self.bbox_iou_reset:
                self._window.clear()
                stats["scene_change"] = True

        self._window.append({"had_cargo": had, "bbox": bbox})
        n = len(self._window)
        n_pos = sum(1 for rec in self._window if rec["had_cargo"])
        ratio = (float(n_pos) / float(n)) if n else 0.0
        stats["window"] = n
        stats["positive_frames"] = n_pos
        stats["positive_ratio"] = ratio

        if had:
            return labels, dets, stats
        if (n < self.min_frames
                or n_pos == 0
                or ratio + 1e-12 < self.min_positive_ratio):
            return labels, dets, stats

        consensus = median_bbox(
            [rec["bbox"] for rec in self._window if rec["bbox"] is not None])
        if consensus is None:
            return labels, dets, stats
        painted = np.array(labels, copy=True)
        if not _paint_bbox(painted, consensus, self.cargo_label):
            return labels, dets, stats
        held = {
            "label": self.cargo_label,
            "prompt": "temporal_hold",
            "confidence": float(ratio),
            "bbox": consensus,
            "held": True,
        }
        stats["held"] = True
        stats["held_bbox"] = list(consensus)
        return painted, dets + [held], stats
