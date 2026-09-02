#!/usr/bin/env python3
"""Exact-stamp join and PCA field helpers for DetectionFrame (no ROS)."""

from __future__ import division

from luggage_perception.semantic_segmenter import compact_detections


class ExactStampJoin(object):
    """Pair two streams by an exact stamp key. Evicts oldest per side."""

    def __init__(self, maxlen=10):
        if int(maxlen) < 1:
            raise ValueError("maxlen must be >= 1")
        self.maxlen = int(maxlen)
        self._left = {}
        self._right = {}

    def clear(self):
        self._left.clear()
        self._right.clear()

    def push_left(self, key, item):
        self._left[key] = item
        self._trim(self._left)
        return self._take(key)

    def push_right(self, key, item):
        self._right[key] = item
        self._trim(self._right)
        return self._take(key)

    def _take(self, key):
        if key in self._left and key in self._right:
            return self._left.pop(key), self._right.pop(key)
        return None

    def _trim(self, buf):
        while len(buf) > self.maxlen:
            del buf[min(buf)]


def stamp_key(stamp):
    """Hashable join key from a ROS stamp or ``(sec, nanosec)`` pair."""
    if stamp is None:
        return None
    if isinstance(stamp, tuple) and len(stamp) >= 2:
        return (int(stamp[0]), int(stamp[1]))
    return (int(stamp.sec), int(stamp.nanosec))


def yolo_box_fields_from_compact(compact):
    """Map ``compact_detections`` dicts onto YoloBox field dicts."""
    boxes = []
    for item in compact or []:
        bbox = list(item.get("bbox") or [0, 0, 0, 0])
        while len(bbox) < 4:
            bbox.append(0)
        boxes.append({
            "label": int(item.get("label", -1)),
            "prompt": str(item.get("prompt", "")),
            "confidence": float(item.get("confidence", 0.0) or 0.0),
            "bbox": [int(v) for v in bbox[:4]],
            "held": bool(item.get("held", False)),
        })
    return boxes


def yolo_box_fields_from_detections(detections):
    return yolo_box_fields_from_compact(compact_detections(detections))


def empty_cargo_pca_fields(n_points=0, reason="DETECT_NO_CLOUD"):
    """PCA payload when there is no cargo cloud to fit."""
    return {
        "pca_valid": False,
        "pca_reason": str(reason or "DETECT_NO_CLOUD"),
        "pca_source": "empty",
        "pca_confidence": 0.0,
        "n_cargo_points": int(n_points),
        "centroid": (0.0, 0.0, 0.0),
    }


def pca_fields_from_failure(reason, n_points, source="empty", centroid=None):
    cx, cy, cz = (0.0, 0.0, 0.0) if centroid is None else centroid
    return {
        "pca_valid": False,
        "pca_reason": str(reason or "DETECT_ESTIMATION_FAILED"),
        "pca_source": str(source or "empty"),
        "pca_confidence": 0.0,
        "n_cargo_points": int(n_points),
        "centroid": (float(cx), float(cy), float(cz)),
    }


def pca_fields_from_estimate(est, n_points, source="measure"):
    center = est.center_xyz
    return {
        "pca_valid": True,
        "pca_reason": "ok",
        "pca_source": str(source or "measure"),
        "pca_confidence": float(est.confidence),
        "n_cargo_points": int(n_points),
        "centroid": (float(center[0]), float(center[1]), float(center[2])),
        "box_id": est.matched_catalog_id or "detected_box",
        "width": float(est.width),
        "depth": float(est.depth),
        "height": float(est.height),
        "yaw_valid": bool(est.yaw_valid),
        "aspect_ratio": float(est.aspect_ratio),
        "quaternion_xyzw": tuple(float(v) for v in est.quaternion_xyzw),
    }
