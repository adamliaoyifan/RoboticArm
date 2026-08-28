#!/usr/bin/env python3
"""Estimate box pose and dimensions from a 3-D point cloud.

Pure-numpy algorithm with no ROS dependencies so it can be unit-tested
independently.  The pipeline is designed for a top-down camera view of a
single box sitting on a known-height platform:

  1. ROI spatial crop  (approximate pickup position ± margin)
  2. RANSAC horizontal plane fit  (box top surface)
  3. PCA on the inlier rectangle  (yaw + width/depth extents)
  4. Height from platform Z
  5. Optional catalog matching to snap noisy dimensions
"""

from __future__ import division

import math
import time
from collections import namedtuple

import numpy as np

from .box_geometry import aspect_ratio as _aspect_ratio

BoxEstimate = namedtuple(
    "BoxEstimate",
    [
        "center_xyz",       # (3,) ndarray – geometric centre in the input frame
        "quaternion_xyzw",  # (4,) ndarray – orientation as (x, y, z, w)
        "width",            # float – extent along the principal axis
        "depth",            # float – extent along the secondary axis
        "height",           # float – vertical extent
        "confidence",       # float 0-1
        "matched_catalog_id",  # str or None
        "aspect_ratio",     # float – max(W, D) / min(W, D) of reported size
        "yaw_valid",        # bool – False when PCA eigenvalues are too isotropic
    ],
    defaults=(1.0, True),
)

# Covariance λ_max / λ_min below this → in-plane heading is noise.
MIN_EIGEN_RATIO_FOR_YAW = 1.2


def yaw_valid_from_eigen_ratio(eigen_ratio, min_ratio=MIN_EIGEN_RATIO_FOR_YAW):
    """True when the top-face covariance is elongated enough to trust PCA yaw."""
    try:
        ratio = float(eigen_ratio)
    except (TypeError, ValueError):
        return False
    if not math.isfinite(ratio):
        return False
    return ratio >= float(min_ratio)


# ---------------------------------------------------------------------------
# Quaternion helper
# ---------------------------------------------------------------------------

def _quaternion_from_yaw(yaw):
    """Return (x, y, z, w) for a rotation about the world Z-axis."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return np.array([0.0, 0.0, sy, cy], dtype=np.float64)


# ---------------------------------------------------------------------------
# RANSAC horizontal plane
# ---------------------------------------------------------------------------

def _ransac_horizontal_plane(points, max_iter=200, dist_thresh=0.008,
                             normal_thresh=0.15, min_inliers=25):
    """Fit a horizontal plane to the *highest* cluster of points.

    Returns (inlier_mask, plane_z) or (None, None) on failure.  Only planes
    whose normal is within *normal_thresh* radians of vertical are accepted.
    """
    if len(points) < 3:
        return None, None

    best_inliers = None
    best_z = None
    best_count = 0
    rng = np.random.RandomState(42)

    for _ in range(max_iter):
        idx = rng.choice(len(points), 3, replace=False)
        p0, p1, p2 = points[idx]
        normal = np.cross(p1 - p0, p2 - p0)
        norm_len = np.linalg.norm(normal)
        if norm_len < 1e-9:
            continue
        normal /= norm_len
        # Accept only near-vertical normals (Z-dominant).
        if abs(normal[2]) < math.cos(normal_thresh):
            continue
        if normal[2] < 0:
            normal = -normal
        d = -normal.dot(p0)
        dists = np.abs(points.dot(normal) + d)
        inlier_mask = dists < dist_thresh
        count = int(inlier_mask.sum())
        if count >= max(3, int(min_inliers)):
            plane_z = float(np.median(points[inlier_mask, 2]))
            # Prefer the highest sufficiently-supported plane (box top), then
            # the largest inlier set among near-equal heights. The previous
            # count-first rule let the much larger pickup platform dominate
            # raw RGBD input.
            better_height = best_z is None or plane_z > best_z + 0.01
            same_height_better_count = (
                best_z is not None
                and abs(plane_z - best_z) <= 0.01
                and count > best_count
            )
            if not (better_height or same_height_better_count):
                continue
            best_inliers = inlier_mask
            best_z = plane_z
            best_count = count

    return best_inliers, best_z


# ---------------------------------------------------------------------------
# PCA rectangle fit
# ---------------------------------------------------------------------------

def _pca_rectangle(points_2d):
    """PCA on 2-D points → (yaw, extent_0, extent_1, eigen_ratio).

    *eigen_ratio* is λ_max / λ_min of the covariance. Near 1.0 the in-plane
    heading is not identifiable.
    """
    if len(points_2d) < 4:
        return 0.0, 0.0, 0.0, 0.0
    centroid = points_2d.mean(axis=0)
    centered = points_2d - centroid
    cov = np.cov(centered, rowvar=False)
    eigvals, eigvecs = np.linalg.eigh(cov)
    # Principal axis = eigenvector with the larger eigenvalue.
    order = np.argsort(eigvals)[::-1]
    principal = eigvecs[:, order[0]]
    yaw = float(math.atan2(principal[1], principal[0]))
    proj = centered.dot(eigvecs[:, order])
    extent_0 = float(proj[:, 0].max() - proj[:, 0].min())
    extent_1 = float(proj[:, 1].max() - proj[:, 1].min())
    lam_max = float(eigvals[order[0]])
    lam_min = float(eigvals[order[1]])
    if lam_max < 1e-18:
        eigen_ratio = 0.0
    else:
        eigen_ratio = lam_max / max(lam_min, 1e-18)
        if not math.isfinite(eigen_ratio):
            eigen_ratio = 0.0
    return yaw, extent_0, extent_1, eigen_ratio


# Percentile trim used to reject depth outliers when bounding the top face.
_RECT_TRIM_PERCENT = 0.5
# Trimming discards the very points that define an edge, so the measured
# extent is biased small by the trimmed fraction. For points spread roughly
# uniformly across the face the shrinkage is extent * 2 * trim, which showed up
# as a consistent ~10 mm underestimate on a 0.7 m box. Scaling it back keeps
# the outlier rejection without the bias -- and underestimating a footprint is
# the dangerous direction, since the map then believes the box is smaller than
# it is.
_RECT_TRIM_COMPENSATION = 1.0 / (1.0 - 2.0 * _RECT_TRIM_PERCENT / 100.0)


def _refine_rectangle(points_2d, initial_yaw, search_deg=20.0, step_deg=0.25):
    """Minimum-area robust rectangle around a PCA seed.

    PCA orientation and arithmetic-mean center are biased by perspective point
    density. Scanning a narrow angle band and using geometric projection bounds
    recovers the physical rectangle axes and center.
    """
    best = None
    steps = int(round(2.0 * search_deg / step_deg)) + 1
    for delta_deg in np.linspace(-search_deg, search_deg, steps):
        yaw = initial_yaw + math.radians(float(delta_deg))
        axis = np.array([math.cos(yaw), math.sin(yaw)])
        side = np.array([-math.sin(yaw), math.cos(yaw)])
        p0 = points_2d.dot(axis)
        p1 = points_2d.dot(side)
        lo0, hi0 = np.percentile(
            p0, [_RECT_TRIM_PERCENT, 100.0 - _RECT_TRIM_PERCENT])
        lo1, hi1 = np.percentile(
            p1, [_RECT_TRIM_PERCENT, 100.0 - _RECT_TRIM_PERCENT])
        extent0 = float(hi0 - lo0) * _RECT_TRIM_COMPENSATION
        extent1 = float(hi1 - lo1) * _RECT_TRIM_COMPENSATION
        score = extent0 * extent1
        if best is None or score < best[0]:
            # The center is the midpoint of the trimmed bounds; trimming is
            # symmetric so it does not bias the center, only the extents.
            center = axis * ((lo0 + hi0) * 0.5) + side * (
                (lo1 + hi1) * 0.5)
            best = (score, yaw, extent0, extent1, center)
    return best[1], best[2], best[3], best[4]


# ---------------------------------------------------------------------------
# Catalog matching
# ---------------------------------------------------------------------------

def match_catalog(width, depth, height, catalog_entries, tolerance=0.05):
    """Find the best matching catalog entry within *tolerance* on each axis.

    Returns ``(matched_id, snapped_width, snapped_depth, snapped_height)``
    or ``(None, width, depth, height)`` when no match is close enough.
    """
    best_id = None
    best_err = float("inf")
    best_size = (width, depth, height)

    for entry in catalog_entries:
        ew, ed, eh = entry["size"]
        # Try both axis assignments (width↔depth may be swapped depending on
        # the box yaw alignment relative to the PCA principal axis).
        for cw, cd in ((ew, ed), (ed, ew)):
            err = abs(cw - width) + abs(cd - depth) + abs(eh - height)
            if err < best_err and abs(cw - width) < tolerance and abs(cd - depth) < tolerance and abs(eh - height) < tolerance:
                best_err = err
                best_id = entry["id"]
                best_size = (cw, cd, eh)

    return best_id, best_size[0], best_size[1], best_size[2]


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def voxel_downsample(points, voxel_size):
    """Uniform-grid downsample keeping the FIRST point per voxel.

    First-point (not centroid) keeps rectangle extents from shrinking by
    half a voxel per edge; the estimator already undersizes box edges, so
    centroid shrink would compound in the dangerous direction. Order of the
    returned points is deterministic for a given input.

    ``voxel_size <= 0`` returns an unchanged copy.
    """
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if voxel_size is None or voxel_size <= 0.0 or len(pts) == 0:
        return pts.copy()
    grid = np.floor(pts / float(voxel_size)).astype(np.int64)
    # Single-key trick: view each (i,j,k) triple as one void scalar so
    # np.unique returns first-occurrence indices without a lexicographic
    # 3-column sort.
    key = np.ascontiguousarray(grid).view(
        np.dtype((np.void, grid.dtype.itemsize * 3))).ravel()
    _, first_idx = np.unique(key, return_index=True)
    return pts[first_idx]


def estimate_box(points, roi_center_xy=None, roi_margin=0.3,
                 platform_z=None, catalog_entries=None,
                 catalog_tolerance=0.05, min_points=50,
                 ransac_max_iter=200, ransac_dist_thresh=0.008,
                 min_height_above_platform=0.03,
                 voxel_size=0.0, timing=None):
    """Estimate box pose and dimensions from a point cloud.

    Parameters
    ----------
    points : ndarray (N, 3)
        XYZ point cloud in the frame where the box sits (typically *world*
        or *camera_depth_optical_frame* after the caller has transformed).
    roi_center_xy : (float, float) or None
        Approximate (x, y) of the pickup area.  When given, points outside
        ``roi_center ± roi_margin`` are discarded before fitting.
    roi_margin : float
        Half-side of the square ROI crop (metres).
    platform_z : float or None
        Known Z of the platform top surface in the same frame as *points*.
        Used to compute box height = ``plane_z - platform_z``.
    catalog_entries : list[dict] or None
        Output of ``box_catalog_entries()``; each entry must have ``size``
        and ``id`` keys.
    catalog_tolerance : float
        Maximum per-axis deviation (m) for catalog snapping.
    min_points : int
        Minimum number of ROI-filtered points required for estimation.
    ransac_max_iter : int
        RANSAC iterations for plane fitting.
    ransac_dist_thresh : float
        RANSAC inlier distance threshold (m).
    voxel_size : float
        Optional uniform-grid downsample applied AFTER the ROI and platform
        band filters and BEFORE RANSAC. 0 (default) disables it. Plane and
        rectangle estimation depend on inlier ratio and spatial spread, not
        absolute density, so 0.01 m is statistically equivalent on
        full-resolution cargo clouds (~110k -> ~5k points). Supported range
        is <= ~0.02 m: the confidence score below is count-based and a
        coarser voxel can starve it. NOTE the plane selection rule is
        height-dominant (a plane >1 cm higher wins regardless of inlier
        count, as long as it clears min_inliers) - downsampling does not
        change that rule; see _ransac_horizontal_plane.
    timing : dict or None
        When given, filled with per-stage milliseconds (``voxel_ms``,
        ``ransac_ms``, ``refine_ms``) plus ``voxel_from``/``voxel_to``
        point counts. Plain-data out-param; keeps this module ROS-free.

    Returns
    -------
    BoxEstimate or None
        ``None`` when estimation fails (too few points, plane fit failure …).
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] < 3:
        return None
    pts = pts[:, :3]

    # ---- ROI filter ----
    if roi_center_xy is not None:
        cx, cy = float(roi_center_xy[0]), float(roi_center_xy[1])
        mask = (
            (pts[:, 0] >= cx - roi_margin) & (pts[:, 0] <= cx + roi_margin)
            & (pts[:, 1] >= cy - roi_margin) & (pts[:, 1] <= cy + roi_margin)
        )
        pts = pts[mask]

    if len(pts) < min_points:
        return None

    # Raw RGBD includes the pickup platform, whose plane usually has many more
    # points than the suitcase top. Remove it before RANSAC. Semantic cargo
    # clouds are unaffected because their points already lie above the plane.
    if platform_z is not None:
        pts = pts[pts[:, 2] >= float(platform_z) + float(
            min_height_above_platform)]
        if len(pts) < min_points:
            return None

    # ---- Optional voxel downsample (after crop, before fitting) ----
    voxel_from = len(pts)
    if voxel_size and voxel_size > 0.0:
        _t0 = time.monotonic()
        pts = voxel_downsample(pts, voxel_size)
        if timing is not None:
            timing["voxel_ms"] = (time.monotonic() - _t0) * 1000.0
        if len(pts) < min_points:
            return None
    voxel_to = len(pts)

    # ---- RANSAC plane (box top) ----
    _t0 = time.monotonic()
    inlier_mask, plane_z = _ransac_horizontal_plane(
        pts, max_iter=ransac_max_iter, dist_thresh=ransac_dist_thresh,
        min_inliers=max(3, min_points // 2),
    )
    if timing is not None:
        timing["ransac_ms"] = (time.monotonic() - _t0) * 1000.0
    if inlier_mask is None or plane_z is None:
        return None
    inliers = pts[inlier_mask]
    if len(inliers) < min_points // 2:
        return None

    # ---- PCA rectangle on top-surface inliers ----
    yaw, extent_0, extent_1, eigen_ratio = _pca_rectangle(inliers[:, :2])
    yaw_valid = yaw_valid_from_eigen_ratio(eigen_ratio)
    _t0 = time.monotonic()
    yaw, extent_0, extent_1, rectangle_center = _refine_rectangle(
        inliers[:, :2], yaw)
    if timing is not None:
        timing["refine_ms"] = (time.monotonic() - _t0) * 1000.0
        timing["voxel_from"] = voxel_from
        timing["voxel_to"] = voxel_to
    # width = larger extent, depth = smaller (consistent with catalog order).
    if extent_0 >= extent_1:
        width, depth = extent_0, extent_1
    else:
        width, depth = extent_1, extent_0
        yaw += math.pi / 2  # rotate so principal axis aligns with width
    # Normalise yaw to [-pi, pi].
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))

    # ---- Height ----
    if platform_z is not None:
        height = max(0.01, plane_z - platform_z)
    else:
        height = max(0.01, plane_z - float(pts[:, 2].min()))

    # ---- Catalog matching (hybrid) ----
    matched_id = None
    if catalog_entries:
        matched_id, width, depth, height = match_catalog(
            width, depth, height, catalog_entries, catalog_tolerance,
        )

    # ---- Centre ----
    cx = float(rectangle_center[0])
    cy = float(rectangle_center[1])
    cz = plane_z - height * 0.5

    # ---- Confidence heuristic ----
    n_inliers = int(inlier_mask.sum())
    # Count-based, scaled by the caller's min_points. Within the supported
    # voxel range (<=0.02 m) a real box top still saturates this to 1.0;
    # at ~0.05 m the inlier count can drop below min_points*4 and yield a
    # spurious DETECT_LOW_CONFIDENCE. Ratio-based scoring is the long-term
    # fix but requires recalibrating the gate in the same change.
    confidence = min(1.0, n_inliers / max(1, min_points * 4))

    return BoxEstimate(
        center_xyz=np.array([cx, cy, cz]),
        quaternion_xyzw=_quaternion_from_yaw(yaw),
        width=width,
        depth=depth,
        height=height,
        confidence=confidence,
        matched_catalog_id=matched_id,
        aspect_ratio=_aspect_ratio(width, depth),
        yaw_valid=yaw_valid,
    )
