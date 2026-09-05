#!/usr/bin/env python3
"""Platform-free top-surface / local-support estimation (no ROS).

Implements E1 of docs/plans/platform_free_height_eng_todo.md: split the
mathematical work out of the detector into deterministic, ROS-free
estimators so simulation and hardware run the same code.

Two validity levels:
- TOP_ONLY: top plane + rectangle measured; height unavailable.
- FULL_3D: a local support plane was also measured in the same
  acquisition; height and center Z are valid.

v1 is horizontal-only: top and support planes must be within
``normal_tolerance_deg`` of +Z; tilt is rejected, not modeled.
"""

from __future__ import division

import math
from collections import deque
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from luggage_perception.luggage_box_estimator import (
    _pca_rectangle,
    _ransac_horizontal_plane,
    _refine_rectangle,
    voxel_downsample,
)

# Failure/status codes (E0 contract; mirrored as constants so pure modules
# and ROS adapters share one spelling).
DETECT_TOP_UNOBSERVABLE = "DETECT_TOP_UNOBSERVABLE"
DETECT_SUPPORT_UNOBSERVABLE = "DETECT_SUPPORT_UNOBSERVABLE"
DETECT_SUPPORT_UNSTABLE = "DETECT_SUPPORT_UNSTABLE"
DETECT_SUPPORT_STAMP_MISMATCH = "DETECT_SUPPORT_STAMP_MISMATCH"
DETECT_HEIGHT_PRIOR_ONLY = "DETECT_HEIGHT_PRIOR_ONLY"
DETECT_FULL_GEOMETRY_REQUIRED = "DETECT_FULL_GEOMETRY_REQUIRED"
# PF-R2: an unsegmented raw depth cloud is not a luggage observation; the
# dominant pickup platform must never be reported as a valid luggage top.
DETECT_CARGO_SEGMENTATION_REQUIRED = "DETECT_CARGO_SEGMENTATION_REQUIRED"
# PF-R3: stamped preprocessor-status validation. Missing, malformed, or
# stale motion evidence may still permit a top-only result but never a
# MEASURED_SUPPORT height.
DETECT_SUPPORT_STATUS_MISSING = "DETECT_SUPPORT_STATUS_MISSING"
DETECT_SUPPORT_STATUS_MALFORMED = "DETECT_SUPPORT_STATUS_MALFORMED"
DETECT_SUPPORT_STATUS_STALE = "DETECT_SUPPORT_STATUS_STALE"

# Height source enum values (DetectedLuggage.height_source).
HEIGHT_SOURCE_UNAVAILABLE = 0
HEIGHT_SOURCE_MEASURED_SUPPORT = 1
HEIGHT_SOURCE_CONFIGURED_SUPPORT = 2
HEIGHT_SOURCE_CATALOG_PRIOR = 3

# Geometry level values (DetectionFrame.geometry_level).
GEOMETRY_TOP_ONLY = 0
GEOMETRY_FULL_3D = 1


@dataclass
class TopSupportConfig:
    """Deployment-neutral parameters (E3); no scene-truth values here."""

    # Pickup workspace (world frame): center XY + half extents.
    workspace_center_xy: Sequence[float] = (0.0, 0.0)
    workspace_half_extents: Sequence[float] = (0.5, 0.5)
    # Plausible luggage height band [m]; support candidates must sit inside
    # top_z - max_height .. top_z - min_height.
    min_luggage_height: float = 0.15
    max_luggage_height: float = 0.60
    # Top estimation.
    voxel_size: float = 0.01
    min_top_points: int = 50
    ransac_max_iter: int = 200
    ransac_dist_thresh: float = 0.008
    # Support annulus around the fitted top rectangle [m].
    support_inner_margin: float = 0.03
    support_outer_margin: float = 0.18
    min_support_points: int = 80
    support_ransac_max_iter: int = 200
    support_ransac_dist_thresh: float = 0.008
    # Horizontal-plane tolerance (degrees off vertical).
    normal_tolerance_deg: float = 5.0
    # Side coverage: rectangle sides that must show support inliers
    # (1-4; 4 = full ring, 2 = opposite sides, 1 = any side).
    min_support_sides: int = 2
    min_inliers_per_side: int = 15


@dataclass
class TopSurfaceEstimate:
    """Fitted top plane + robust rectangle; top Z preserved explicitly."""

    center_xy: np.ndarray
    top_z: float
    yaw: float
    width: float
    depth: float
    confidence: float
    stamp: float = 0.0
    frame: str = "world"
    yaw_valid: bool = True
    aspect_ratio: float = 1.0
    reason: str = "ok"


@dataclass
class SupportPlaneEstimate:
    """Local support plane fitted from the raw cloud in the same stamp."""

    support_z: float
    residual: float = float("nan")
    normal_alignment: float = 1.0
    side_coverage: float = 0.0
    inlier_count: int = 0
    confidence: float = 0.0
    stamp: float = 0.0
    frame: str = "world"
    reason: str = "ok"


@dataclass
class BoxGeometryEstimate:
    """Composed detection result with machine-readable validity."""

    top: Optional[TopSurfaceEstimate]
    support: Optional[SupportPlaneEstimate]
    height_valid: bool
    height_source: int
    center_xyz: Optional[np.ndarray] = None
    width: float = 0.0
    depth: float = 0.0
    height: float = 0.0
    reason: str = "ok"
    geometry_level: int = GEOMETRY_TOP_ONLY


def _crop_workspace(points, center_xy, half_extents):
    cx, cy = float(center_xy[0]), float(center_xy[1])
    hx, hy = float(half_extents[0]), float(half_extents[1])
    mask = (
        (points[:, 0] >= cx - hx) & (points[:, 0] <= cx + hx)
        & (points[:, 1] >= cy - hy) & (points[:, 1] <= cy + hy)
    )
    return points[mask]


def estimate_top_surface(cargo_points_world, workspace, config=None):
    """Fit the highest valid horizontal cargo plane + top rectangle.

    ``workspace`` is ``(center_xy, half_extents)`` in the world frame.
    Returns :class:`TopSurfaceEstimate` or ``None`` with the reason in
    ``DETECT_TOP_UNOBSERVABLE`` semantics (caller logs).
    """
    config = config or TopSupportConfig()
    points = np.asarray(cargo_points_world, dtype=np.float64).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    points = _crop_workspace(points, workspace[0], workspace[1])
    if len(points) < int(config.min_top_points):
        return None
    points = voxel_downsample(points, config.voxel_size)
    if len(points) < int(config.min_top_points):
        return None

    inlier_mask, plane_z = _ransac_horizontal_plane(
        points,
        max_iter=config.ransac_max_iter,
        dist_thresh=config.ransac_dist_thresh,
        min_inliers=max(3, config.min_top_points // 2),
        normal_thresh=math.radians(config.normal_tolerance_deg),
    )
    if inlier_mask is None or plane_z is None:
        return None
    inliers = points[inlier_mask]
    yaw, extent_0, extent_1, _eigen_ratio = _pca_rectangle(inliers[:, :2])
    yaw_valid = True
    try:
        yaw, extent_0, extent_1, rectangle_center = _refine_rectangle(
            inliers[:, :2], yaw)
    except (ValueError, TypeError):
        rectangle_center = inliers[:, :2].mean(axis=0)
    ratio = max(extent_0, extent_1) / max(1e-9, min(extent_0, extent_1))
    if ratio < 1.15:
        yaw_valid = False
    n_inliers = int(inlier_mask.sum())
    confidence = min(1.0, n_inliers / max(1, config.min_top_points * 4))
    width, depth = (
        (extent_0, extent_1) if extent_0 >= extent_1 else (extent_1, extent_0))
    return TopSurfaceEstimate(
        center_xy=np.asarray(rectangle_center, dtype=np.float64),
        top_z=float(plane_z),
        yaw=float(yaw),
        width=float(width),
        depth=float(depth),
        confidence=float(confidence),
        yaw_valid=yaw_valid,
        aspect_ratio=float(ratio),
    )


def _rotate_to_rect_axes(points_xy, center_xy, yaw):
    cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
    dx = points_xy[:, 0] - center_xy[0]
    dy = points_xy[:, 1] - center_xy[1]
    u = dx * cos_y - dy * sin_y   # along width axis
    v = dx * sin_y + dy * cos_y   # along depth axis
    return u, v


def estimate_local_support(raw_points_world, top_estimate, workspace,
                          config=None):
    """Fit the horizontal support plane around the top rectangle.

    Candidates: raw points in the outer annulus (footprint + inner margin
    .. + outer margin), below ``top_z`` by at least ``min_luggage_height``
    and at most ``max_luggage_height``, inside the pickup workspace.
    A larger floor below the plausible band is excluded by the band, not
    by trusting any configured platform height.
    """
    config = config or TopSupportConfig()
    points = np.asarray(raw_points_world, dtype=np.float64).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    points = _crop_workspace(points, workspace[0], workspace[1])
    # PF-R6 opt 3: the height band needs only top_z (already known), so
    # apply it BEFORE the rectangle rotation/annulus math — on the raw
    # depth cloud most points sit far outside the plausible support band
    # (floor below, box body above) and are rejected by two comparisons
    # instead of the full rotate + annulus pipeline.
    band = (
        (top_estimate.top_z - points[:, 2] >= config.min_luggage_height)
        & (top_estimate.top_z - points[:, 2] <= config.max_luggage_height)
    )
    points = points[band]
    if len(points) < int(config.min_support_points):
        return SupportPlaneEstimate(
            support_z=float("nan"), reason=DETECT_SUPPORT_UNOBSERVABLE)

    u, v = _rotate_to_rect_axes(
        points[:, :2], top_estimate.center_xy, top_estimate.yaw)
    half_w = top_estimate.width * 0.5
    half_d = top_estimate.depth * 0.5
    inner_w = half_w + config.support_inner_margin
    inner_d = half_d + config.support_inner_margin
    outer_w = half_w + config.support_outer_margin
    outer_d = half_d + config.support_outer_margin
    in_annulus = (
        (np.abs(u) > inner_w) | (np.abs(v) > inner_d)
    ) & (
        (np.abs(u) < outer_w) & (np.abs(v) < outer_d)
    )
    candidates = points[in_annulus]
    if len(candidates) < int(config.min_support_points):
        return SupportPlaneEstimate(
            support_z=float("nan"), reason=DETECT_SUPPORT_UNOBSERVABLE)

    inlier_mask, support_z = _ransac_horizontal_plane(
        candidates,
        max_iter=config.support_ransac_max_iter,
        dist_thresh=config.support_ransac_dist_thresh,
        min_inliers=int(config.min_support_points),
        normal_thresh=math.radians(config.normal_tolerance_deg),
    )
    if inlier_mask is None or support_z is None:
        return SupportPlaneEstimate(
            support_z=float("nan"), reason=DETECT_SUPPORT_UNOBSERVABLE)
    inliers = candidates[inlier_mask]

    # Side coverage: which rectangle edges have support inliers nearby.
    u_i, v_i = _rotate_to_rect_axes(
        inliers[:, :2], top_estimate.center_xy, top_estimate.yaw)
    band_half = config.support_outer_margin * 0.5
    sides = {
        "+u": (np.abs(u_i - outer_w) < band_half),
        "-u": (np.abs(u_i + outer_w) < band_half),
        "+v": (np.abs(v_i - outer_d) < band_half),
        "-v": (np.abs(v_i + outer_d) < band_half),
    }
    covered = sum(
        1 for mask in sides.values()
        if int(mask.sum()) >= config.min_inliers_per_side)
    side_coverage = covered / 4.0
    if covered < int(config.min_support_sides):
        return SupportPlaneEstimate(
            support_z=float(support_z),
            residual=float(np.median(np.abs(
                inliers[:, 2] - support_z))),
            side_coverage=float(side_coverage),
            inlier_count=int(inlier_mask.sum()),
            reason=DETECT_SUPPORT_UNSTABLE)

    residual = float(np.median(np.abs(inliers[:, 2] - support_z)))
    confidence = min(
        1.0,
        0.5 * side_coverage + 0.5 * min(
            1.0, int(inlier_mask.sum())
            / max(1, config.min_support_points * 3)))
    return SupportPlaneEstimate(
        support_z=float(support_z),
        residual=residual,
        normal_alignment=1.0,  # horizontal model: plane normal is +Z
        side_coverage=float(side_coverage),
        inlier_count=int(inlier_mask.sum()),
        confidence=float(confidence))


def compose_box_geometry(top, support=None, platform_z=None,
                        catalog_height=None):
    """Compose a BoxGeometryEstimate from top (+ optional support).

    ``platform_z`` is only consumed as CONFIGURED_SUPPORT (explicit
    deployment mode); it never overrides a measured support. A
    ``catalog_height`` prior populates the numeric height with
    ``height_valid=false`` (HEIGHT_SOURCE_CATALOG_PRIOR).
    """
    if top is None:
        return BoxGeometryEstimate(
            top=None, support=None, height_valid=False,
            height_source=HEIGHT_SOURCE_UNAVAILABLE,
            reason=DETECT_TOP_UNOBSERVABLE)

    height = 0.0
    center = None
    source = HEIGHT_SOURCE_UNAVAILABLE
    support_ok = (
        support is not None
        and support.reason == "ok"
        and np.isfinite(support.support_z))
    if support_ok:
        height = top.top_z - support.support_z
        center = np.array([
            top.center_xy[0], top.center_xy[1],
            (top.top_z + support.support_z) * 0.5])
        source = HEIGHT_SOURCE_MEASURED_SUPPORT
    elif platform_z is not None and np.isfinite(float(platform_z)):
        height = top.top_z - float(platform_z)
        center = np.array([
            top.center_xy[0], top.center_xy[1],
            (top.top_z + float(platform_z)) * 0.5])
        source = HEIGHT_SOURCE_CONFIGURED_SUPPORT
    elif catalog_height is not None:
        height = float(catalog_height)
        center = np.array([
            top.center_xy[0], top.center_xy[1],
            top.top_z - height * 0.5])
        source = HEIGHT_SOURCE_CATALOG_PRIOR

    height_valid = source in (
        HEIGHT_SOURCE_MEASURED_SUPPORT, HEIGHT_SOURCE_CONFIGURED_SUPPORT)
    reason = "ok" if height_valid else (
        DETECT_HEIGHT_PRIOR_ONLY if source == HEIGHT_SOURCE_CATALOG_PRIOR
        else (support.reason if support is not
              None else DETECT_SUPPORT_UNOBSERVABLE))
    return BoxGeometryEstimate(
        top=top,
        support=support,
        height_valid=height_valid,
        height_source=source,
        center_xyz=center,
        width=top.width,
        depth=top.depth,
        height=height,
        reason=reason,
        geometry_level=(
            GEOMETRY_FULL_3D if height_valid else GEOMETRY_TOP_ONLY))


class SupportStabilityFilter:
    """Short-window support-Z stability gate (update/copy_output pattern)."""

    def __init__(self, window=5, max_z_spread=0.015):
        self.window = int(window)
        self.max_z_spread = float(max_z_spread)
        self._history = deque(maxlen=self.window)
        self._output = None

    def update(self, support):
        """Feed a :class:`SupportPlaneEstimate` (or None). Copies stored.

        A ``None``/non-finite sample both invalidates the output and
        clears the history, so a new luggage instance cannot inherit a
        previous box's Z window.
        """
        if support is None or not np.isfinite(support.support_z):
            self._output = None
            self._history.clear()
            return self.copy_output()
        self._history.append(float(support.support_z))
        history = list(self._history)
        spread = max(history) - min(history)
        self._output = SupportPlaneEstimate(
            support_z=float(np.median(history)),
            residual=support.residual,
            normal_alignment=support.normal_alignment,
            side_coverage=support.side_coverage,
            inlier_count=support.inlier_count,
            confidence=support.confidence,
            stamp=support.stamp,
            frame=support.frame,
            reason=(
                "ok" if len(history) >= self.window
                and spread <= self.max_z_spread
                else DETECT_SUPPORT_UNSTABLE))
        return self.copy_output()

    def copy_output(self):
        if self._output is None:
            return None
        return SupportPlaneEstimate(
            support_z=self._output.support_z,
            residual=self._output.residual,
            normal_alignment=self._output.normal_alignment,
            side_coverage=self._output.side_coverage,
            inlier_count=self._output.inlier_count,
            confidence=self._output.confidence,
            stamp=self._output.stamp,
            frame=self._output.frame,
            reason=self._output.reason)
