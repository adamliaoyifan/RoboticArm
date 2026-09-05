#!/usr/bin/env python3
"""PF-R6-RANSAC-RESEARCH comparator estimators (ROS-free, research only).

Every comparator is a drop-in replacement for the committed production
helper ``_ransac_horizontal_plane(points, max_iter, dist_thresh,
normal_thresh, min_inliers) -> (inlier_mask|None, plane_z|None)`` so the
harness can run the *exact* committed ``estimate_local_support`` validation
(band, annulus, side coverage, reason codes, min points) around each one.
All methods therefore receive identical, non-privileged candidate sets and
differ only in how they pick the horizontal support plane.

Comparators:

- ``baseline_ransac_plane``: verbatim committed 3-point minimal-sample
  RANSAC (the "triangle" minimal set of standard plane RANSAC), 200 fixed
  iterations, highest-supported-plane preference.
- ``zmode_median_plane``: fail-closed dominant-z-cluster / median estimator
  (handoff "Proposed Direction"): bin candidate z at ~2x dist_thresh, pick
  the highest cluster dense enough, support_z = cluster median, residual =
  median |z - support_z| over inliers.
- ``one_point_constrained_ransac``: nearest constrained-RANSAC candidate.
  With the horizontal-plane prior the normal is fixed to +Z, so the minimal
  sample collapses from 3 points to 1 (z = z0). Adds the classical
  RANSAC confidence-based early-stopping rule (adaptive iteration bound),
  which is the standard sampling-efficiency acceleration. This is NOT
  claimed to be the recalled automotive method; it is the closest
  well-supported constrained-RANSAC family for a horizontal plane.
- ``grid_seed_robust_z``: grid-seed robust-z estimator in the
  Himmelsbach-2008 / Axelsson-2000 lineage (per-cell lowest-point seeds,
  then robust-z over seeds). Labelled explicitly as an adaptation, not the
  original polar-grid line-fitting algorithm.

Determinism: baseline uses RandomState(42) (as committed); the 1-point
variant uses an injected seed defaulting to 42. z-mode and grid-seed are
rng-free.
"""

from __future__ import division

import math

import numpy as np

from luggage_perception.luggage_box_estimator import _ransac_horizontal_plane


def baseline_ransac_plane(points, max_iter=200, dist_thresh=0.008,
                          normal_thresh=0.15, min_inliers=25):
    """Verbatim committed production support-plane RANSAC (3-point sample)."""
    return _ransac_horizontal_plane(
        points, max_iter=max_iter, dist_thresh=dist_thresh,
        normal_thresh=normal_thresh, min_inliers=min_inliers)


def _densest_plane_from_bin_centers(z, centers, dist_thresh, min_inliers,
                                    prefer="highest"):
    """Scan bin centers top-down; first center with enough inliers wins."""
    order = np.argsort(centers)[::-1] if prefer == "highest" else np.argsort(centers)
    for ci in order:
        c = centers[ci]
        mask = np.abs(z - c) < dist_thresh
        count = int(mask.sum())
        if count >= max(3, int(min_inliers)):
            plane_z = float(np.median(z[mask]))
            # Re-derive the final inlier set around the refined plane z.
            mask = np.abs(z - plane_z) < dist_thresh
            if int(mask.sum()) >= max(3, int(min_inliers)):
                return mask, float(plane_z)
    return None, None


def zmode_median_plane(points, max_iter=200, dist_thresh=0.008,
                       normal_thresh=0.15, min_inliers=25,
                       bin_width=None):
    """Dominant-z-cluster / median support estimator (fail-closed).

    Bin width defaults to the committed support distance threshold scale
    (8 mm), matching the handoff's 8-10 mm guidance. Cluster preference is
    *highest dense plane first*, mirroring the committed RANSAC's
    ``better_height`` rule so both methods answer the same question.
    """
    if len(points) < 3:
        return None, None
    bw = float(bin_width) if bin_width else max(float(dist_thresh), 0.008)
    z = points[:, 2]
    lo = float(np.floor(z.min() / bw) * bw)
    hi = float(np.ceil(z.max() / bw) * bw)
    n_bins = int(max(1, round((hi - lo) / bw)))
    centers = lo + bw * (np.arange(n_bins) + 0.5)
    return _densest_plane_from_bin_centers(
        z, centers, dist_thresh, min_inliers, prefer="highest")


def one_point_constrained_ransac(points, max_iter=200, dist_thresh=0.008,
                                 normal_thresh=0.15, min_inliers=25,
                                 seed=42, confidence=0.99):
    """1-point normal-constrained RANSAC with adaptive early stopping.

    The horizontal prior fixes the normal to +Z, so one sample point fully
    determines the plane hypothesis z = z0 (no normal check needed; the
    constraint is the check). Keeps the committed acceptance rule (highest
    sufficiently-supported plane, then largest inlier set within 10 mm) so
    its answers are comparable to the baseline's.
    """
    if len(points) < 3:
        return None, None
    rng = np.random.RandomState(seed)
    z = points[:, 2]
    n = len(z)
    best_mask = None
    best_z = None
    best_count = 0
    it_done = 0
    it_needed = max_iter
    for _ in range(int(max_iter)):
        it_done += 1
        idx = rng.randint(n)
        mask = np.abs(z - z[idx]) < dist_thresh
        count = int(mask.sum())
        if count >= max(3, int(min_inliers)):
            plane_z = float(np.median(z[mask]))
            # Committed preference rule: highest plane first, then the
            # largest inlier set among near-equal heights.
            better_height = best_z is None or plane_z > best_z + 0.01
            same_height_better_count = (
                best_z is not None
                and abs(plane_z - best_z) <= 0.01
                and count > best_count)
            if better_height or same_height_better_count:
                best_mask = mask
                best_z = plane_z
                best_count = count
                # Adaptive stopping: with a 1-point minimal set, P(one
                # clean draw) = w; stop once the confidence bound is met.
                w = min(1.0, best_count / float(n))
                denom = math.log(max(1e-12, 1.0 - w))
                if denom < 0:
                    it_needed = int(math.ceil(
                        math.log(1.0 - float(confidence)) / denom))
        if best_z is not None and it_done >= it_needed:
            break
    return best_mask, best_z


def grid_seed_robust_z(points, max_iter=200, dist_thresh=0.008,
                       normal_thresh=0.15, min_inliers=25,
                       cell=0.05):
    """Grid-seed lowest-point robust-z (Himmelsbach/Axelsson lineage).

    Rectangular XY grid (5 cm cells); the lowest point per cell is a ground
    seed (Himmelsbach 2008 polar-grid seed idea; Axelsson 2000 TIN seed
    idea). Flyers above the true surface are suppressed because any cell
    containing a support point keeps its lowest sample. The support plane
    is then the highest dense robust-z cluster over seeds, and the final
    inlier set is re-expanded over all candidates.
    """
    if len(points) < 3:
        return None, None
    cell = float(cell)
    z = points[:, 2]
    gx = np.floor(points[:, 0] / cell).astype(np.int64)
    gy = np.floor(points[:, 1] / cell).astype(np.int64)
    key = gx * 1000003 + gy
    order = np.lexsort((z, key))  # sort by cell key, then z ascending
    k_sorted = key[order]
    first = np.ones(len(order), dtype=bool)
    first[1:] = k_sorted[1:] != k_sorted[:-1]
    seed_idx = order[first]
    seed_z = z[seed_idx]
    if len(seed_z) < 3:
        return None, None
    bw = max(float(dist_thresh), 0.008)
    lo = float(np.floor(seed_z.min() / bw) * bw)
    hi = float(np.ceil(seed_z.max() / bw) * bw)
    n_bins = int(max(1, round((hi - lo) / bw)))
    centers = lo + bw * (np.arange(n_bins) + 0.5)
    seed_min = max(3, int(min_inliers) // 4)  # seeds subsample the annulus
    mask_seeds, seed_plane_z = _densest_plane_from_bin_centers(
        seed_z, centers, dist_thresh, seed_min, prefer="highest")
    if mask_seeds is None:
        return None, None
    plane_z = float(np.median(seed_z[mask_seeds]))
    mask = np.abs(z - plane_z) < dist_thresh
    if int(mask.sum()) < max(3, int(min_inliers)):
        return None, None
    return mask, plane_z


COMPARATORS = {
    "baseline_ransac": baseline_ransac_plane,
    "zmode_median": zmode_median_plane,
    "one_point_constrained_ransac": one_point_constrained_ransac,
    "grid_seed_robust_z": grid_seed_robust_z,
}
