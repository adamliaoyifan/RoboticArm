#!/usr/bin/env python3
"""Semantic point filter core (no ROS, no rospy).

Given a depth point cloud (in the depth optical frame) and a per-pixel
semantic label map (aligned with the color image), project each 3D point
into the color image plane and route it to one of several output streams
based on its semantic label.

Alignment model (RealSense D435):
  - Depth and color have separate pinhole intrinsics.
  - Depth optical frame is the cloud's frame_id.
  - ``depth_to_color`` extrinsic (R, t) maps a point in depth optical frame
    to color optical frame:  p_color = R * p_depth + t.
  - Projection: u = (fx * x + cx * z) / z, v = (fy * y + cy * z) / z
    where (fx, fy, cx, cy) are the *color* intrinsics.

All math is plain numpy so the module is unit-testable without roscore.
"""

from __future__ import division

import math

import numpy as np


class CameraIntrinsics:
    """Pinhole camera intrinsics + optional distortion (kept for completeness)."""

    def __init__(self, fx, fy, cx, cy, width, height,
                 distortion_coeffs=None, distortion_model="plumb_bob"):
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)
        self.width = int(width)
        self.height = int(height)
        self.distortion_coeffs = list(distortion_coeffs or [])
        self.distortion_model = str(distortion_model)

    @classmethod
    def from_dict(cls, d):
        intr = d.get("intrinsics", d)
        return cls(
            fx=float(intr["fx"]),
            fy=float(intr["fy"]),
            cx=float(intr["cx"]),
            cy=float(intr["cy"]),
            width=int(d.get("width", 0)),
            height=int(d.get("height", 0)),
            distortion_coeffs=intr.get("distortion_coeffs"),
            distortion_model=intr.get("distortion_model", "plumb_bob"),
        )

    def as_array(self):
        return np.array([self.fx, self.fy, self.cx, self.cy], dtype=np.float64)


class DepthToColorExtrinsics:
    """Rigid transform from depth optical frame to color optical frame."""

    def __init__(self, rotation, translation):
        self.rotation = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
        self.translation = np.asarray(translation, dtype=np.float64).reshape(3)

    @classmethod
    def from_dict(cls, d):
        return cls(
            rotation=d.get("rotation", np.eye(3)),
            translation=d.get("translation", [0.0, 0.0, 0.0]),
        )

    @classmethod
    def identity(cls):
        """Degenerate extrinsics: depth and color share one sensor (gz sim)."""
        return cls(rotation=np.eye(3), translation=np.zeros(3))


def _project_to_color(points_depth, rotation, translation, color_intr):
    """Transform depth-frame points to color frame and project to pixel coords.

    Args:
        points_depth: (N, 3) array of (x, y, z) in depth optical frame.
        rotation: (3, 3) depth -> color rotation.
        translation: (3,) depth -> color translation.
        color_intr: CameraIntrinsics for the color camera.

    Returns:
        (uv, depth_color_z): uv is (N, 2) int pixel coords; depth_color_z is
        (N,) per-point depth in color frame. Points with non-positive z get
        uv=(-1, -1) so callers can mask them out.
    """
    pts = np.asarray(points_depth, dtype=np.float64).reshape(-1, 3)
    if pts.size == 0:
        return np.zeros((0, 2), dtype=np.int32), np.zeros((0,), dtype=np.float64)

    rotated = pts @ rotation.T + translation  # (N, 3)
    z = rotated[:, 2]
    uv = np.full((pts.shape[0], 2), -1, dtype=np.int32)

    valid = z > 1e-6
    if not np.any(valid):
        return uv, z

    z_valid = z[valid]
    x_valid = rotated[valid, 0]
    y_valid = rotated[valid, 1]
    u = (color_intr.fx * x_valid + color_intr.cx * z_valid) / z_valid
    v = (color_intr.fy * y_valid + color_intr.cy * z_valid) / z_valid

    u_round = np.rint(u).astype(np.int32)
    v_round = np.rint(v).astype(np.int32)

    # Mark points that project outside the image as invalid (-1) so callers
    # can count them as out-of-frame rather than silently clamping to the
    # border (which would assign them the wrong label).
    in_image = (
        (u_round >= 0) & (u_round < color_intr.width)
        & (v_round >= 0) & (v_round < color_intr.height)
    )

    valid_idx = np.where(valid)[0]
    keep = valid_idx[in_image]
    uv[keep, 0] = u_round[in_image]
    uv[keep, 1] = v_round[in_image]
    return uv, z


class SemanticPointFilter:
    """Route depth points to output streams based on a semantic label map.

    Output streams are integer label sets; a point whose mask label is in
    ``cargo_labels`` goes to the cargo stream, a point in ``obstacle_labels``
    goes to the obstacle stream. A label may belong to both streams.
    """

    def __init__(self, color_intrinsics, depth_intrinsics,
                 depth_to_color, cargo_labels, obstacle_labels,
                 exclude_labels=None):
        self.color_intrinsics = color_intrinsics
        self.depth_intrinsics = depth_intrinsics
        self.depth_to_color = depth_to_color
        self.cargo_labels = set(int(l) for l in cargo_labels)
        self.obstacle_labels = set(int(l) for l in obstacle_labels)
        self.exclude_labels = set(int(l) for l in (exclude_labels or []))
        self._last_stats = {
            "raw_count": 0,
            "cargo_count": 0,
            "obstacle_count": 0,
            "excluded_count": 0,
            "out_of_frame_count": 0,
        }

    @property
    def last_stats(self):
        return dict(self._last_stats)

    def filter_points(self, points_depth, label_map, instance_map=None):
        """Filter depth-frame points using the color-aligned label map.

        Args:
            points_depth: list/array of (x, y, z) in depth optical frame.
            label_map: HxW uint8 label map aligned with the color image.
            instance_map: optional HxW uint16 instance ID map (0 = no instance).

        Returns:
            (cargo_points, obstacle_points): each is a list of tuples.
            When ``instance_map`` is provided the tuples are
            ``(x, y, z, label, instance_id)``; otherwise ``(x, y, z)``.
        """
        pts = np.asarray(points_depth, dtype=np.float64).reshape(-1, 3)
        raw_count = int(pts.shape[0])
        has_instance = instance_map is not None
        if raw_count == 0:
            self._last_stats = {
                "raw_count": 0,
                "cargo_count": 0,
                "obstacle_count": 0,
                "excluded_count": 0,
                "out_of_frame_count": 0,
            }
            return [], []

        uv, _z = _project_to_color(
            pts, self.depth_to_color.rotation, self.depth_to_color.translation,
            self.color_intrinsics,
        )

        h, w = label_map.shape[:2]
        in_frame = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
        out_of_frame = int((~in_frame).sum())

        # Vectorized label routing (semantics identical to the per-point loop
        # this replaces: exclude wins, a label may feed both streams, and
        # labels in neither stream count as excluded). The loop form spent
        # ~1.5 s per 300k-point frame in Python, which pushed published
        # clouds past the detector's staleness window.
        valid_idx = np.where(in_frame)[0]
        vu = uv[valid_idx, 1]
        uu = uv[valid_idx, 0]
        label_arr = label_map[vu, uu].astype(np.int64)
        inst_arr = (instance_map[vu, uu].astype(np.int64)
                    if has_instance else np.zeros(len(valid_idx), dtype=np.int64))

        def _mask(label_set):
            if not label_set:
                return np.zeros(len(valid_idx), dtype=bool)
            return np.isin(label_arr, np.asarray(sorted(label_set), dtype=np.int64))

        excl_mask = _mask(self.exclude_labels)
        cargo_mask = _mask(self.cargo_labels) & ~excl_mask
        obstacle_mask = _mask(self.obstacle_labels) & ~excl_mask
        neither = ~(cargo_mask | obstacle_mask) & ~excl_mask
        excluded = int(excl_mask.sum() + neither.sum())

        pts_valid = pts[valid_idx]
        if has_instance:
            def _tuples(sel):
                block = pts_valid[sel]
                return [(block[i, 0], block[i, 1], block[i, 2],
                         int(label_arr[sel][i]), int(inst_arr[sel][i]))
                        for i in range(block.shape[0])]
            cargo_pts = _tuples(cargo_mask)
            obstacle_pts = _tuples(obstacle_mask)
        else:
            # Stay in numpy. tolist() of a 100k-point cargo cloud was several
            # seconds per frame and stalled the exclusive join callback.
            cargo_pts = pts_valid[cargo_mask]
            obstacle_pts = pts_valid[obstacle_mask]

        self._last_stats = {
            "raw_count": raw_count,
            "cargo_count": len(cargo_pts),
            "obstacle_count": len(obstacle_pts),
            "excluded_count": excluded,
            "out_of_frame_count": out_of_frame,
        }
        return cargo_pts, obstacle_pts


class JoinStampTracker:
    """Camera stamps for cargo join replay. 0 join stamp means never joined.

    ``last_cargo_n_points`` is -1 until the first join, then 0 means that
    join had no cargo pixels (not that join never happened).
    """

    def __init__(self):
        self.last_cloud_stamp = 0.0
        self.last_mask_stamp = 0.0
        self.last_join_stamp = 0.0
        self.last_cargo_n_points = -1
        self.generation = 0
        self.instance_id = ""

    def note_cloud(self, stamp_sec):
        self.last_cloud_stamp = float(stamp_sec)

    def note_mask(self, stamp_sec):
        self.last_mask_stamp = float(stamp_sec)

    def note_join(self, stamp_sec, n_cargo):
        self.last_join_stamp = float(stamp_sec)
        self.last_cargo_n_points = int(n_cargo)

    def note_epoch(self, generation, instance_id=""):
        self.generation = int(generation or 0)
        self.instance_id = str(instance_id or "")

    def as_dict(self):
        return {
            "last_cloud_stamp": self.last_cloud_stamp,
            "last_mask_stamp": self.last_mask_stamp,
            "last_join_stamp": self.last_join_stamp,
            "last_cargo_n_points": self.last_cargo_n_points,
            "generation": int(self.generation),
            "instance_id": str(self.instance_id),
        }
