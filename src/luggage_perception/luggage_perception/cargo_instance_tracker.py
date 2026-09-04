#!/usr/bin/env python3
"""Single-suitcase cargo cloud tracker (no ROS).

Associates each joined cargo measurement in a fixed world frame by centroid
distance and replaces the tracked cloud in place. Motion does not freeze
updates. An empty current_box epoch (generation > 0, no id) stays empty so
a lingering YOLO mask cannot republish the deleted suitcase.
"""

from __future__ import division

import json

import numpy as np


SOURCE_EMPTY = "empty"
SOURCE_MEASURE = "measure"
SOURCE_HOLD_TRACK = "hold_track"
SOURCE_REJECT_CLUTTER = "reject_clutter"


def parse_current_box_payload(payload):
    """Return ``(box_id, generation)`` from ``/luggage/current_box`` JSON.

    Empty or invalid payloads are ``("", 0)``.
    """
    if not payload:
        return "", 0
    if isinstance(payload, dict):
        data = payload
    else:
        try:
            data = json.loads(payload)
        except (TypeError, ValueError):
            return "", 0
        if not isinstance(data, dict):
            return "", 0
    box_id = str(data.get("id") or data.get("model_name") or "")
    try:
        generation = int(data.get("generation") or 0)
    except (TypeError, ValueError):
        generation = 0
    return box_id, generation


def xyz_array(points):
    """Nx3 float64 from cargo tuples or an array. Empty input -> (0, 3)."""
    if points is None:
        return np.zeros((0, 3), dtype=np.float64)
    if isinstance(points, np.ndarray):
        arr = np.asarray(points, dtype=np.float64)
        if arr.size == 0:
            return np.zeros((0, 3), dtype=np.float64)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        return np.ascontiguousarray(arr[:, :3], dtype=np.float64)
    if not points:
        return np.zeros((0, 3), dtype=np.float64)
    return np.ascontiguousarray(
        [(p[0], p[1], p[2]) for p in points], dtype=np.float64)


def rotation_from_xyzw(qx, qy, qz, qw):
    """3x3 rotation matrix from a ``(x, y, z, w)`` quaternion."""
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),
         2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz),
         2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
         1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def transform_points(points, rotation, translation):
    """Apply ``p' = R p + t`` to an Nx3 cloud."""
    pts = xyz_array(points)
    if pts.shape[0] == 0:
        return pts
    rot = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trans = np.asarray(translation, dtype=np.float64).reshape(3)
    return pts.dot(rot.T) + trans


class CargoInstanceTracker:
    """Replace-in-place world-frame track of one suitcase cloud."""

    def __init__(self, associate_radius_m=0.15):
        self.associate_radius_m = max(0.0, float(associate_radius_m))
        self.generation = 0
        self.instance_id = ""
        self.points_world = None
        self.centroid = None
        self.n_points = 0
        self.cloud_stamp = 0.0
        self.source = SOURCE_EMPTY
        self.motion_weight = 1.0

    def reset(self):
        """Drop the geometric track. Epoch id/generation are left as-is."""
        self.points_world = None
        self.centroid = None
        self.n_points = 0
        self.cloud_stamp = 0.0
        self.source = SOURCE_EMPTY

    def set_epoch(self, generation, instance_id=""):
        """Bind a current_box generation. True when the epoch changed."""
        generation = int(generation or 0)
        instance_id = str(instance_id or "")
        if generation == self.generation and instance_id == self.instance_id:
            return False
        self.reset()
        self.generation = generation
        self.instance_id = instance_id
        return True

    def _frozen_empty(self):
        """Sim clear: a generation is active but there is no suitcase id."""
        return self.generation > 0 and not self.instance_id

    def _accept(self, points_world, centroid, stamp_sec, source):
        self.points_world = np.ascontiguousarray(
            points_world, dtype=np.float64)
        self.centroid = np.asarray(centroid, dtype=np.float64).reshape(3)
        self.n_points = int(self.points_world.shape[0])
        self.cloud_stamp = float(stamp_sec)
        self.source = source
        self.motion_weight = 1.0

    def observe(self, stamp_sec, points_world):
        """Ingest a world-frame measurement. Empty array is a YOLO miss.

        Returns the source label for this tick (``empty``, ``measure``,
        ``hold_track``, or ``reject_clutter``).
        """
        stamp_sec = float(stamp_sec)
        pts = xyz_array(points_world)
        if pts.shape[0]:
            finite = np.isfinite(pts).all(axis=1)
            pts = pts[finite]
        n = int(pts.shape[0])

        if self._frozen_empty():
            self.cloud_stamp = stamp_sec
            self.source = SOURCE_EMPTY
            return self.source

        if n == 0:
            self.cloud_stamp = stamp_sec
            if self.n_points > 0:
                self.source = SOURCE_HOLD_TRACK
            else:
                self.source = SOURCE_EMPTY
            return self.source

        centroid = pts.mean(axis=0)
        if self.centroid is None:
            self._accept(pts, centroid, stamp_sec, SOURCE_MEASURE)
            return self.source

        dist = float(np.linalg.norm(centroid - self.centroid))
        if dist <= self.associate_radius_m:
            self._accept(pts, centroid, stamp_sec, SOURCE_MEASURE)
            return self.source

        self.cloud_stamp = stamp_sec
        self.source = SOURCE_REJECT_CLUTTER
        return self.source

    def note_tf_miss(self, stamp_sec):
        """TF failed; do not replace the track. Advance the publish stamp."""
        self.cloud_stamp = float(stamp_sec)
        if self._frozen_empty() or self.n_points <= 0:
            self.source = SOURCE_EMPTY
        else:
            self.source = SOURCE_HOLD_TRACK
        return self.source

    def as_dict(self):
        centroid = None
        if self.centroid is not None:
            centroid = [float(v) for v in self.centroid]
        return {
            "generation": int(self.generation),
            "instance_id": str(self.instance_id),
            "n_points": int(self.n_points),
            "cloud_stamp": float(self.cloud_stamp),
            "source": str(self.source),
            "centroid": centroid,
            "motion_weight": float(self.motion_weight),
            "associate_radius_m": float(self.associate_radius_m),
        }
