#!/usr/bin/env python3
"""Dynamic instance-derived top-surface estimation (no ROS).

DYNAMIC-SUCTION ST-1 design decision A: starting from the isolated instance
depth component (see :mod:`luggage_perception.instance_depth_component`),
estimate the box top as

    component pixels -> world points -> gravity-constrained plane candidates
    -> connected top patch -> full-range rectangle.

Key properties fixed by the approved plan
(``docs/plans/dynamic_top_surface_and_suction_patch.md``):

- all gravity-consistent planes with at least ``min_plane_points_frac`` of
  the retained points and at least ``min_plane_inliers`` inliers are
  generated; the plane normal must be within ``normal_tol_deg`` of world
  +Z; the RANSAC-era 8 mm distance is a geometry grouping tolerance only;
- plane candidates are scored by instance support, connected image area,
  inlier fraction, residual and height. Height alone never selects, and a
  plane holding less than ``min_support_area_ratio`` of the best
  candidate's connected support area cannot win at all;
- the winning plane's connected inliers are projected into plane
  coordinates; PCA is a diagnostic seed only; the final rectangle comes
  from the convex hull over the full 0-90 degree rectangular symmetry
  range, so a PCA error greater than 20 degrees remains recoverable;
- aspect ratio below ``aspect_yaw_threshold`` yields ``yaw_valid=False``
  (never stabilized by scene_tf or a catalog prior);
- scene_tf / workspace / catalog values are never inputs to this module.

Plane generation is fully deterministic (z-cluster seeds + fixed-iteration
least-squares refinement; no RNG anywhere), so identical inputs produce
byte-identical outputs. The module performs no I/O and imports nothing from
ROS; the node layer looks up TF at the acquisition stamp and passes the
4x4 optical->world matrix in as plain numbers.
"""

from __future__ import division

import math
import time
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

from luggage_perception.depth_deprojection import deproject_selected
from luggage_perception.instance_depth_component import (
    largest_component_mask,
)
from luggage_perception.top_support_estimator import TopSurfaceEstimate

DETECT_DYNAMIC_TOP_NO_PLANE = "DETECT_DYNAMIC_TOP_NO_PLANE"
DETECT_DYNAMIC_TOP_NO_CONNECTED_PATCH = "DETECT_DYNAMIC_TOP_NO_CONNECTED_PATCH"
DETECT_DYNAMIC_TOP_COMPONENT_REJECTED = "DETECT_DYNAMIC_TOP_COMPONENT_REJECTED"
DETECT_DYNAMIC_TOP_TOO_FEW_POINTS = "DETECT_DYNAMIC_TOP_TOO_FEW_POINTS"
DETECT_DYNAMIC_TOP_INVALID_TRANSFORM = "DETECT_DYNAMIC_TOP_INVALID_TRANSFORM"

#: plane candidate diagnostics retained per estimate (bounded, evidence only)
MAX_PLANE_DIAGNOSTICS = 8


@dataclass
class DynamicTopConfig:
    """Plan-A thresholds; changing one requires a new plan generation."""

    # Plane candidate gates.
    normal_tol_deg: float = 8.0
    min_plane_points_frac: float = 0.15
    min_plane_inliers: int = 80
    plane_group_tol_m: float = 0.008   # grouping tolerance only
    #: LSQ refit iterations per z-cluster seed. 8 lets a seed band on a
    #: 6 deg tilted top converge onto the full plane (3 iterations left
    #: the labelled pixels short and downstream residual gates rejected
    #: every footprint).
    refit_iterations: int = 8
    #: plane candidates closer than this in angle and centroid height are
    #: the same physical surface (dedupe of z-cluster seeds). Surfaces
    #: further apart than the grouping tolerance stay distinct candidates.
    dedupe_angle_deg: float = 2.0
    dedupe_height_m: float = 0.009
    # Connected-support rule.
    min_support_area_ratio: float = 0.60
    # Yaw observability.
    aspect_yaw_threshold: float = 1.15
    # Scoring weights (height is deliberately absent: it only breaks exact
    # ties among eligible candidates, see _select_plane).
    weight_mask_support: float = 0.45
    weight_connected_area: float = 0.30
    weight_inlier_fraction: float = 0.15
    weight_residual: float = 0.10
    # Rectangle.
    hull_trim_residual_m: float = 0.006
    #: per-axis extent compensation for pixel quantization, in metres.
    extent_compensation_m: float = 0.0


@dataclass
class PlaneCandidate:
    """Diagnostic record for one plane candidate (bounded evidence)."""

    plane_id: int
    normal: tuple
    inliers: int
    rms_residual: float
    height: float
    connected_area: int
    mask_support: float
    inlier_fraction: float
    score: float
    eligible: bool
    rejected_reason: str = ""


@dataclass
class DynamicTopResult(TopSurfaceEstimate):
    """TopSurfaceEstimate plus dynamic-path diagnostics."""

    plane_candidates: tuple = ()
    pca_yaw: float = 0.0
    component_coverage: float = 0.0
    top_source: str = "dynamic"
    pixel_count: int = 0
    #: (H, W) uint8 per-pixel plane label, 0 = none, 1..N in candidate
    #: order (first candidate wins overlaps). Consumed by the suction
    #: patch evaluator (ST-2) so plane labelling is computed once.
    plane_labels: object = None
    #: index (1-based into plane_labels) of the winning plane.
    winner_plane_id: int = 0
    #: winning plane frame as ((u_axis, v_axis, normal), d): orthonormal
    #: basis in plane coords plus the plane offset (p.n + d = height).
    plane_basis: tuple = ()
    plane_offset: float = 0.0


def _identity_like(mat):
    mat = np.asarray(mat, dtype=np.float64)
    if mat.shape != (4, 4):
        raise ValueError("optical_to_world must be a 4x4 matrix")
    return mat


def _world_points(depth_mm, component_mask, intrinsics, mat4):
    """Component pixels -> optical points -> world points (+ pixel provenance).

    Returns ``(points_world (N,3) float64, vu (N,), uu (N,))`` or
    ``(None, None, None)`` when nothing valid remains.
    """
    mask = np.asarray(component_mask, dtype=bool)
    vu, uu = np.nonzero(mask)
    if len(vu) == 0:
        return None, None, None
    depth = np.asarray(depth_mm)
    valid = np.isfinite(depth[vu, uu]) & (depth[vu, uu] > 0)
    if not valid.all():
        uu, vu = uu[valid], vu[valid]
    if len(vu) == 0:
        return None, None, None
    cam_pts, _ = deproject_selected(depth, uu, vu, intrinsics)
    if len(cam_pts) == 0:
        return None, None, None
    hom = np.concatenate(
        (cam_pts.astype(np.float64),
         np.ones((len(cam_pts), 1), dtype=np.float64)), axis=1)
    world = (hom @ _identity_like(mat4).T)[:, :3]
    finite = np.isfinite(world).all(axis=1)
    if not finite.all():
        world, vu, uu = world[finite], vu[finite], uu[finite]
        if len(world) == 0:
            return None, None, None
    return world, vu, uu


def _fit_plane_lsq(points):
    """Least-squares plane through points -> (unit normal (+Z-ish), d).

    Returns ``(None, None)`` for degenerate input.
    """
    if len(points) < 3:
        return None, None
    centroid = points.mean(axis=0)
    centered = points - centroid
    try:
        _, _, vt = np.linalg.svd(centered, full_matrices=False)
    except np.linalg.LinAlgError:
        return None, None
    normal = vt[-1]
    norm = float(np.linalg.norm(normal))
    if norm < 1e-12:
        return None, None
    normal = normal / norm
    if normal[2] < 0.0:
        normal = -normal
    d = -float(normal @ centroid)
    return normal, d


def _plane_candidates(points, cfg):
    """All gravity-consistent planes; deterministic z-cluster + LSQ refits.

    Seeds come from z-bins of the grouping tolerance (each needing the
    min inlier count); every seed is refined by a fixed number of
    least-squares refits with inliers re-selected inside the grouping
    tolerance. Duplicated refinements of one physical surface collapse.
    """
    n = len(points)
    min_inliers = max(int(cfg.min_plane_inliers),
                      int(math.ceil(cfg.min_plane_points_frac * n)))
    if n < 3 or min_inliers < 3:
        return []
    bw = float(cfg.plane_group_tol_m)
    z = points[:, 2]
    lo = math.floor(float(z.min()) / bw) * bw
    hi = math.ceil(float(z.max()) / bw) * bw
    n_bins = max(1, int(round((hi - lo) / bw)))
    centers = lo + bw * (np.arange(n_bins) + 0.5)
    counts = np.empty(n_bins, dtype=np.int64)
    for i, c in enumerate(centers):
        counts[i] = int((np.abs(z - c) < bw).sum())
    seeds = centers[counts >= min_inliers]

    cos_tol = math.cos(math.radians(cfg.dedupe_angle_deg))
    normal_tol = math.cos(math.radians(cfg.normal_tol_deg))
    candidates = []
    for seed in seeds:
        normal = np.array([0.0, 0.0, 1.0])
        d = -float(seed)
        inlier_idx = None
        for _ in range(max(1, int(cfg.refit_iterations))):
            dist = points @ normal + d
            inlier_idx = np.abs(dist) <= bw
            if int(inlier_idx.sum()) < min_inliers:
                break
            fitted = _fit_plane_lsq(points[inlier_idx])
            if fitted[0] is None:
                break
            normal, d = fitted
        if inlier_idx is None or int(inlier_idx.sum()) < min_inliers:
            continue
        if float(normal[2]) < normal_tol:
            continue
        # Dedupe against accepted candidates (same physical surface).
        duplicate = False
        for cand in candidates:
            if (float(cand["normal"] @ normal) >= cos_tol
                    and abs(cand["height"] - _plane_height(
                        normal, d, points[inlier_idx])) <=
                    float(cfg.dedupe_height_m)):
                if int(inlier_idx.sum()) > int(cand["inlier_idx"].sum()):
                    cand.update(
                        normal=normal, d=d, height=_plane_height(
                            normal, d, points[inlier_idx]),
                        inlier_idx=inlier_idx.copy())
                duplicate = True
                break
        if not duplicate:
            candidates.append({
                "normal": normal, "d": d,
                "height": _plane_height(normal, d, points[inlier_idx]),
                "inlier_idx": inlier_idx.copy(),
                "rms": float(np.sqrt(np.mean(
                    (points[inlier_idx] @ normal + d) ** 2))),
            })
    return candidates


def _plane_height(normal, d, points):
    """Plane world-Z at the centroid of ``points`` (on-plane height)."""
    centroid = points.mean(axis=0)
    return float(-(d + normal[0] * centroid[0]
                   + normal[1] * centroid[1]) / normal[2])


def _connected_stats(shape, inlier_pixels):
    """Largest 8-connected blob of an inlier pixel set -> (mask, area)."""
    blob_mask = np.zeros(shape, dtype=bool)
    blob_mask[inlier_pixels[0], inlier_pixels[1]] = True
    largest = largest_component_mask(blob_mask)
    return largest, int(largest.sum())


def _mask_support(blob_mask, instance_region):
    """Fraction of blob pixels inside the original instance region."""
    if instance_region is None:
        return 1.0
    region = np.asarray(instance_region, dtype=bool)
    total = int(blob_mask.sum())
    if total == 0:
        return 0.0
    return float((blob_mask & region).sum()) / float(total)


def _select_plane(candidates, blob_masks, areas, mask_supports, inlier_fracs,
                  rms_list, heights, cfg):
    """Deterministic plan-A selection.

    Eligibility: connected support area >= ``min_support_area_ratio`` times
    the best candidate's. Score: weighted mask support / connected area /
    inlier fraction / residual — height only breaks exact score ties
    (prefer the higher plane), so height alone can never select a plane.
    Returns the winning index and per-candidate diagnostics.
    """
    best_area = max(areas) if areas else 0
    eligible = [a >= float(cfg.min_support_area_ratio) * best_area
                for a in areas]
    diag = []
    for i in range(len(candidates)):
        diag.append(PlaneCandidate(
            plane_id=i,
            normal=tuple(float(v) for v in candidates[i]["normal"]),
            inliers=int(candidates[i]["inlier_idx"].sum()),
            rms_residual=float(rms_list[i]),
            height=float(heights[i]),
            connected_area=int(areas[i]),
            mask_support=float(mask_supports[i]),
            inlier_fraction=float(inlier_fracs[i]),
            score=0.0, eligible=bool(eligible[i])))
    if not any(eligible):
        return None, diag
    scores = []
    for i in range(len(candidates)):
        area_frac = areas[i] / float(best_area) if best_area else 0.0
        res_score = 1.0 - min(
            1.0, rms_list[i] / float(cfg.plane_group_tol_m))
        scores.append(
            cfg.weight_mask_support * mask_supports[i]
            + cfg.weight_connected_area * area_frac
            + cfg.weight_inlier_fraction * inlier_fracs[i]
            + cfg.weight_residual * res_score)
    for i in range(len(diag)):
        diag[i].score = float(scores[i])
    # Highest score wins; exact ties -> higher plane; then lower plane_id.
    winner = max(
        (i for i in range(len(candidates)) if eligible[i]),
        key=lambda i: (scores[i], heights[i], -i))
    for i in range(len(diag)):
        if eligible[i] and i != winner:
            diag[i].rejected_reason = "outscored"
        elif not eligible[i]:
            diag[i].rejected_reason = "support_area_below_ratio"
    return winner, diag


def _plane_basis(normal):
    """Stable in-plane orthonormal basis (u, v), u ~ world +X.

    Gram-Schmidt against the world axis least aligned with ``normal``:
    cross(z, n) degenerates as n approaches +Z (the normal case here), so
    it cannot seed the basis.
    """
    n = np.asarray(normal, dtype=np.float64)
    seed = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 \
        else np.array([0.0, 1.0, 0.0])
    u = seed - float(seed @ n) * n
    norm = float(np.linalg.norm(u))
    if norm < 1e-12:
        u = np.array([0.0, 1.0, 0.0]) - n[1] * n
        norm = float(np.linalg.norm(u))
    u = u / norm
    v = np.cross(n, u)
    return u, v


def _convex_hull(points_2d):
    """Monotone-chain hull; returns (m, 2) vertices CCW (m >= 3)."""
    pts = np.asarray(points_2d, dtype=np.float64)
    order = np.lexsort((pts[:, 1], pts[:, 0]))
    pts = pts[order]
    # Drop duplicates for a strictly monotone chain.
    keep = np.ones(len(pts), dtype=bool)
    keep[1:] = np.any(pts[1:] != pts[:-1], axis=1)
    pts = pts[keep]
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return ((a[0] - o[0]) * (b[1] - o[1])
                - (a[1] - o[1]) * (b[0] - o[0]))

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in pts[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    hull = np.array(lower[:-1] + upper[:-1], dtype=np.float64)
    return hull if len(hull) >= 3 else pts


def _min_area_rectangle(hull):
    """Rotating-calipers minimum-area rectangle over the full 0-90 deg.

    Returns ``(yaw_axis_a, extent_a, extent_b, center_uv)`` where
    ``yaw_axis_a`` is the RAW hull-edge-aligned axis angle (not reduced
    modulo 90 deg — the caller needs to know which returned extent belongs
    to which axis to pick the long side). Deterministic: first strictly
    smaller area in hull edge order wins.
    """
    if len(hull) < 3:
        if len(hull):
            return 0.0, 0.0, 0.0, (float(hull[:, 0].mean()),
                                    float(hull[:, 1].mean()))
        return 0.0, 0.0, 0.0, (0.0, 0.0)
    best = None
    m = len(hull)
    for i in range(m):
        edge = hull[(i + 1) % m] - hull[i]
        elen = float(np.linalg.norm(edge))
        if elen < 1e-12:
            continue
        axis = edge / elen
        side = np.array([-axis[1], axis[0]])
        proj_a = hull @ axis
        proj_b = hull @ side
        extent_a = float(proj_a.max() - proj_a.min())
        extent_b = float(proj_b.max() - proj_b.min())
        area = extent_a * extent_b
        if best is None or area < best[0] - 1e-15:
            center = (axis * ((proj_a.max() + proj_a.min()) * 0.5)
                      + side * ((proj_b.max() + proj_b.min()) * 0.5))
            yaw = float(math.atan2(axis[1], axis[0]))
            best = (area, yaw, extent_a, extent_b, tuple(center))
    area, yaw, extent_a, extent_b, center = best
    return yaw, extent_a, extent_b, center


def estimate_dynamic_top_surface(depth_mm, component_mask, intrinsics,
                                 optical_to_world, stamp=0.0, frame="world",
                                 instance_region=None, config=None,
                                 timing=None):
    """Full dynamic top estimate for one acquisition.

    Args:
        depth_mm: (H, W) millimetre aligned depth.
        component_mask: (H, W) bool from
            :func:`~luggage_perception.instance_depth_component.\
isolate_depth_component`.
        intrinsics: object with fx/fy/cx/cy of the colour grid.
        optical_to_world: 4x4 transform at the acquisition stamp (plain
            numbers; the node looked it up, this module never does TF).
        stamp/frame: acquisition identity carried on the result.
        instance_region: original (non-eroded) bbox/mask region for mask
            support scoring; None means the component region itself.
        config: :class:`DynamicTopConfig`.

    Returns: :class:`DynamicTopResult` (``reason != "ok"`` = fail-closed
    with ``top_z/yaw`` zeroed semantics preserved by the caller rejecting).
    """
    cfg = config or DynamicTopConfig()
    timing = timing if timing is not None else {}

    def _fail(reason, plane_candidates=()):
        return DynamicTopResult(
            center_xy=np.zeros(2), top_z=0.0, yaw=0.0, width=0.0,
            depth=0.0, confidence=0.0, stamp=float(stamp),
            frame=str(frame), yaw_valid=False, aspect_ratio=1.0,
            reason=str(reason),
            plane_candidates=tuple(
                d for d in plane_candidates[:MAX_PLANE_DIAGNOSTICS]))

    _t0 = time.monotonic()
    world, vu, uu = _world_points(depth_mm, component_mask, intrinsics,
                                  optical_to_world)
    if world is None or len(world) < 3:
        return _fail(DETECT_DYNAMIC_TOP_TOO_FEW_POINTS)
    n_points = len(world)

    candidates = _plane_candidates(world, cfg)
    timing["dynamic_plane_candidates"] = len(candidates)
    if not candidates:
        return _fail(DETECT_DYNAMIC_TOP_NO_PLANE)

    shape = np.asarray(component_mask).shape
    # Instance region for mask-support scoring: the original region when
    # provided, else the component itself (support 1.0).
    region = None
    if instance_region is not None:
        region_arr = np.asarray(instance_region, dtype=bool)
        if region_arr.shape == shape:
            region = region_arr
    blob_masks, areas, mask_supports, inlier_fracs, rms_list, heights = \
        [], [], [], [], [], []
    plane_labels = np.zeros(shape, dtype=np.uint8)
    for i, cand in enumerate(candidates):
        idx = cand["inlier_idx"]
        lab_view = plane_labels[vu[idx], uu[idx]]
        plane_labels[vu[idx], uu[idx]] = np.where(
            lab_view == 0, i + 1, lab_view)
        blob, area = _connected_stats(shape, (vu[idx], uu[idx]))
        if area <= 0:
            blob_masks.append(blob)
            areas.append(0)
            mask_supports.append(0.0)
            inlier_fracs.append(float(idx.sum()) / n_points)
            rms_list.append(cand["rms"])
            heights.append(cand["height"])
            continue
        blob_masks.append(blob)
        areas.append(area)
        mask_supports.append(_mask_support(blob, region))
        inlier_fracs.append(float(idx.sum()) / n_points)
        rms_list.append(cand["rms"])
        heights.append(cand["height"])

    winner, diag = _select_plane(
        candidates, blob_masks, areas, mask_supports, inlier_fracs,
        rms_list, heights, cfg)
    if winner is None:
        return _fail(DETECT_DYNAMIC_TOP_NO_CONNECTED_PATCH,
                     plane_candidates=tuple(
                         d for d in diag[:MAX_PLANE_DIAGNOSTICS]))
    cand = candidates[winner]
    blob = blob_masks[winner]
    inlier_idx = cand["inlier_idx"]
    # Restrict to the winning connected patch pixels.
    subset = np.flatnonzero(inlier_idx)
    in_blob = blob[vu[subset], uu[subset]]
    patch_idx = subset[in_blob]
    if len(patch_idx) < 3:
        return _fail(DETECT_DYNAMIC_TOP_NO_CONNECTED_PATCH,
                     plane_candidates=tuple(
                         d for d in diag[:MAX_PLANE_DIAGNOSTICS]))
    patch_world = world[patch_idx]

    # Robust trim: plane residual gate before the hull so depth flyers on
    # the plane band cannot inflate the extents.
    dist = patch_world @ cand["normal"] + cand["d"]
    keep = np.abs(dist) <= float(cfg.hull_trim_residual_m)
    if int(keep.sum()) >= 3:
        patch_world = patch_world[keep]

    u_axis, v_axis = _plane_basis(cand["normal"])
    pts_2d = np.column_stack((patch_world @ u_axis,
                              patch_world @ v_axis))
    _t1 = time.monotonic()
    pca_yaw, _, _, _ = _pca_yaw(pts_2d)
    hull = _convex_hull(pts_2d)
    yaw_in_plane, extent_a, extent_b, center_uv = _min_area_rectangle(hull)
    timing["dynamic_rectangle_ms"] = (time.monotonic() - _t1) * 1000.0

    comp = float(cfg.extent_compensation_m)
    extent_a_comp = max(0.0, extent_a + comp)
    extent_b_comp = max(0.0, extent_b + comp)
    width, depth = (
        (extent_a_comp, extent_b_comp)
        if extent_a_comp >= extent_b_comp else (extent_b_comp, extent_a_comp))
    aspect = width / max(1e-9, depth)

    # Long-axis world yaw (0-90 deg symmetry); a in-plane angle theta from
    # the u axis maps to world direction cos(t)*u + sin(t)*v.
    if extent_a_comp >= extent_b_comp:
        long_dir = (math.cos(yaw_in_plane) * u_axis
                    + math.sin(yaw_in_plane) * v_axis)
    else:
        theta = yaw_in_plane + math.pi / 2.0
        long_dir = (math.cos(theta) * u_axis
                    + math.sin(theta) * v_axis)
    yaw_world = float(math.atan2(long_dir[1], long_dir[0])) % math.pi

    # A point with in-plane coords (a, b) lies at a*u + b*v - d*n
    # (u, v orthonormal in-plane, so only the normal term carries d).
    center_world = (u_axis * center_uv[0] + v_axis * center_uv[1]
                    - cand["normal"] * cand["d"])
    center_xy = center_world[:2]
    top_z = float(center_world[2])

    inlier_frac = inlier_fracs[winner]
    res_score = 1.0 - min(1.0, rms_list[winner]
                          / float(cfg.plane_group_tol_m))
    confidence = float(
        cfg.weight_mask_support * mask_supports[winner]
        + cfg.weight_inlier_fraction * inlier_frac
        + cfg.weight_residual * res_score
        + cfg.weight_connected_area * 1.0)
    timing["dynamic_total_ms"] = (time.monotonic() - _t0) * 1000.0

    return DynamicTopResult(
        center_xy=np.asarray(center_xy, dtype=np.float64),
        top_z=float(top_z), yaw=float(yaw_world),
        width=float(width), depth=float(depth),
        confidence=float(min(1.0, confidence)),
        stamp=float(stamp), frame=str(frame),
        yaw_valid=bool(aspect >= float(cfg.aspect_yaw_threshold)),
        aspect_ratio=float(aspect), reason="ok",
        plane_candidates=tuple(d for d in diag[:MAX_PLANE_DIAGNOSTICS]),
        pca_yaw=float(pca_yaw),
        pixel_count=int(blob.sum()),
        plane_labels=plane_labels,
        winner_plane_id=int(winner) + 1,
        plane_basis=(u_axis, v_axis, cand["normal"]),
        plane_offset=float(cand["d"]))


def _pca_yaw(points_2d):
    """PCA heading (diagnostic only in the dynamic path)."""
    if len(points_2d) < 4:
        return 0.0, 0.0, 0.0, 0.0
    centered = points_2d - points_2d.mean(axis=0)
    cov = np.cov(centered, rowvar=False)
    eigvals, eigvecs = np.linalg.eigh(cov)
    order = np.argsort(eigvals)[::-1]
    principal = eigvecs[:, order[0]]
    yaw = float(math.atan2(principal[1], principal[0]))
    proj = centered @ eigvecs[:, order]
    extent_0 = float(proj[:, 0].max() - proj[:, 0].min())
    extent_1 = float(proj[:, 1].max() - proj[:, 1].min())
    lam_max = float(eigvals[order[0]])
    lam_min = float(eigvals[order[1]])
    ratio = lam_max / max(lam_min, 1e-18) if lam_max > 1e-18 else 0.0
    if not math.isfinite(ratio):
        ratio = 0.0
    return yaw, extent_0, extent_1, ratio
