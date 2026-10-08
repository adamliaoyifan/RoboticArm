"""Bounded same-stamp bundle for an unbound multi-detection frame.

The point filter records the measurement. Detections for that exact stamp
complete it. A one-detection stamp is dropped. At most eight bundles are
kept; older ones are deleted.
"""

import json
from pathlib import Path

import numpy as np

MAX_BUNDLES = 8


def _stamp_name(stamp_ns):
    return "%d" % int(stamp_ns)


class AmbiguityCapture(object):
    def __init__(self, root):
        self.root = Path(root)
        self._pending = {}
        self._detections = {}

    def note_measurement(self, stamp_ns, measurement):
        """Arrays and epoch from the filter, before detections arrive."""
        self._pending[int(stamp_ns)] = dict(measurement)
        self._trim()
        return self._complete(int(stamp_ns))

    def note_detections(self, stamp_ns, detections, selected_index):
        """Boxes from the YOLO message with this stamp. Not a later frame."""
        self._detections[int(stamp_ns)] = {
            "detections": [dict(row) for row in detections],
            "selected_index": selected_index,
            "selection_used": False,
        }
        self._trim()
        return self._complete(int(stamp_ns))

    def _trim(self):
        for store in (self._pending, self._detections):
            while len(store) > MAX_BUNDLES:
                oldest = sorted(store)[0]
                del store[oldest]

    def _complete(self, stamp_ns):
        if stamp_ns not in self._pending or stamp_ns not in self._detections:
            return None
        detected = self._detections.pop(stamp_ns)
        measurement = self._pending.pop(stamp_ns)
        if len(detected["detections"]) < 2:
            return None
        out = self.root / _stamp_name(stamp_ns)
        if out.exists():
            return str(out)
        out.mkdir(parents=True, exist_ok=True)
        meta = {
            "stamp_ns": int(stamp_ns),
            "epoch": measurement.get("epoch") or {},
            "tf": measurement.get("tf") or {},
            "frames": measurement.get("frames") or {},
            "detections": detected["detections"],
            "selected_index": detected["selected_index"],
            "selection_used": False,
            "reason": "DETECT_TARGET_ASSOCIATION_AMBIGUOUS",
        }
        (out / "frame.json").write_text(
            json.dumps(meta, indent=2, sort_keys=True) + "\n")
        arrays = {}
        for key in ("depth", "label_map", "instance_map",
                    "cargo_before", "cargo_after"):
            value = measurement.get(key)
            if value is not None:
                arrays[key] = np.asarray(value)
        if arrays:
            np.savez_compressed(out / "arrays.npz", **arrays)
        self._trim_files()
        return str(out)

    def _trim_files(self):
        if not self.root.is_dir():
            return
        names = sorted(
            path for path in self.root.iterdir()
            if path.is_dir() and path.name.isdigit())
        while len(names) > MAX_BUNDLES:
            old = names.pop(0)
            for child in old.iterdir():
                child.unlink()
            old.rmdir()


def selected_detection_index(detections):
    """Highest confidence, then bbox area. The index is recorded, not applied."""
    best_i = None
    best_key = None
    for i, det in enumerate(detections):
        bbox = det.get("bbox") or (0, 0, 0, 0)
        area = max(0, int(bbox[2]) - int(bbox[0])) * max(
            0, int(bbox[3]) - int(bbox[1]))
        key = (float(det.get("confidence") or 0.0), area)
        if best_key is None or key > best_key:
            best_key = key
            best_i = i
    return best_i
