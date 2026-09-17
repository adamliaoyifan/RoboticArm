#!/usr/bin/env python3
"""Height map over the winning top-plane (DYNAMIC-SUCTION plan section B).

Aggregates the labelled component pixels (every pixel carrying a plane
label from the dynamic top stage, so a footprint straddling a step sees
BOTH surfaces) into a uniform grid in the winning plane's coordinate
system:

- ``cell_size_m`` grid over the component extent (plan: 5 mm);
- per cell: valid point count, median height over the winning plane,
  majority plane label, instance-mask membership, and a finite-difference
  local normal (world frame);
- per-cell point groupings kept as (point ids sorted by cell, cell
  boundaries) so a footprint's raw points are gathered without a search.

Pure numpy/stdlib; no ROS, no I/O; deterministic. Aggregation is
vectorized: medians and majorities come from one global sort plus
bincount tricks, not per-cell Python loops (B6 latency budget).
"""

from __future__ import division

from dataclasses import dataclass

import numpy as np

from luggage_perception.depth_deprojection import deproject_selected
from luggage_perception.dynamic_top_surface import _plane_basis


@dataclass
class HeightMap:
    """One acquisition's top-plane height grid plus its frame."""

    cell_size_m: float
    origin_u: float            # plane coord of cell (0, 0) corner, metres
    origin_v: float
    nu: int                    # cells along u
    nv: int                    # cells along v
    valid_count: np.ndarray    # (nu, nv) int32
    median_h: np.ndarray       # (nu, nv) float, nan where invalid
    #: 5x5 nan-aware box-smoothed median heights (the robust surface the
    #: evaluator fits and gates on; raw medians stay for evidence dumps).
    smooth_h: np.ndarray       # (nu, nv) float, nan where < 20 neighbours
    plane_label: np.ndarray    # (nu, nv) uint8, 0 where no points
    mask_member: np.ndarray    # (nu, nv) bool (majority membership)
    local_normal: np.ndarray   # (nu, nv, 3) world frame, nan at borders
    u_axis: np.ndarray         # winning-plane basis (world frame)
    v_axis: np.ndarray
    normal: np.ndarray
    plane_d: float             # plane offset: p . n + d = height
    point_ids: np.ndarray      # point indices sorted by (cell, height)
    point_cell_bounds: np.ndarray   # (n_nonempty_cells+1,) into point_ids
    point_cells: np.ndarray    # flat cell id per nonempty cell
    point_h: np.ndarray        # per-point height over the plane (sorted)
    point_uv: np.ndarray       # (N, 2) plane coords (sorted order)

    @property
    def shape(self):
        return (int(self.nu), int(self.nv))

    def cell_world_center(self, iu, iv):
        """World point at a cell centre, on the winning plane."""
        u = self.origin_u + (float(iu) + 0.5) * self.cell_size_m
        v = self.origin_v + (float(iv) + 0.5) * self.cell_size_m
        return (self.u_axis * u + self.v_axis * v
                - self.normal * self.plane_d)


def build_height_map(depth_mm, intrinsics, optical_to_world, top,
                     instance_region=None, cell_size_m=0.005):
    """Aggregate the labelled component into a top-plane height map.

    Args:
        depth_mm: (H, W) aligned depth, millimetres.
        intrinsics: fx/fy/cx/cy of the colour grid.
        optical_to_world: 4x4 at the acquisition stamp (plain numbers).
        top: :class:`DynamicTopResult` with ``reason == "ok"`` and
            ``plane_labels``/``plane_basis`` set by the ST-1 estimator.
        instance_region: (H, W) bool original (non-eroded) bbox/mask
            region for mask-membership scoring; None means the labelled
            pixels themselves.
        cell_size_m: grid resolution (plan B: 0.005).

    Returns: :class:`HeightMap` (empty grid when nothing is labelled).
    """
    labels = getattr(top, "plane_labels", None) if top is not None else None
    basis = getattr(top, "plane_basis", None) if top is not None else None
    if labels is None or getattr(top, "reason", None) != "ok":
        return _empty_map(top, cell_size_m)
    labelled = np.asarray(labels) > 0
    vu, uu = np.nonzero(labelled)
    depth = np.asarray(depth_mm)
    if len(vu):
        valid = np.isfinite(depth[vu, uu]) & (depth[vu, uu] > 0)
        if not valid.all():
            uu, vu, labelled = uu[valid], vu[valid], labelled[vu, uu]
    if len(vu) == 0:
        return _empty_map(top, cell_size_m)
    pixel_labels = np.asarray(labels)[vu, uu]

    cam_pts, _ = deproject_selected(depth, uu, vu, intrinsics)
    hom = np.concatenate((cam_pts.astype(np.float64),
                          np.ones((len(cam_pts), 1))), axis=1)
    world = (hom @ np.asarray(optical_to_world,
                              dtype=np.float64).T)[:, :3]
    finite = np.isfinite(world).all(axis=1)
    if not finite.all():
        world, vu, uu = world[finite], vu[finite], uu[finite]
        pixel_labels = pixel_labels[finite]
        if len(world) == 0:
            return _empty_map(top, cell_size_m)

    if basis is not None and len(basis) == 3:
        u_axis, v_axis, n_axis = (np.asarray(b, dtype=np.float64)
                                  for b in basis)
        plane_d = float(getattr(top, "plane_offset", 0.0))
    else:
        n_axis = np.array([0.0, 0.0, 1.0])
        u_axis, v_axis = _plane_basis(n_axis)
        plane_d = -float(top.top_z)

    pts_u = world @ u_axis
    pts_v = world @ v_axis
    pts_h = world @ n_axis + plane_d

    cell = float(cell_size_m)
    origin_u = float(np.floor(pts_u.min() / cell) * cell)
    origin_v = float(np.floor(pts_v.min() / cell) * cell)
    iu = np.floor((pts_u - origin_u) / cell).astype(np.int64)
    iv = np.floor((pts_v - origin_v) / cell).astype(np.int64)
    nu, nv = int(iu.max()) + 1, int(iv.max()) + 1

    flat = iu * nv + iv
    # Sort by (cell, height): one global sort gives per-cell runs whose
    # medians are index arithmetic.
    order = np.lexsort((pts_h, flat))
    flat_sorted = flat[order]
    h_sorted = pts_h[order]
    bounds = np.concatenate(
        ([0], np.flatnonzero(np.diff(flat_sorted)) + 1,
         [len(flat_sorted)]))
    kept_cells = flat_sorted[bounds[:-1]]
    counts = np.diff(bounds).astype(np.int64)
    n_cells = len(kept_cells)

    # Per-cell medians from the sorted runs.
    mid_lo = (bounds[:-1] + bounds[1:] - 1) // 2
    med = h_sorted[mid_lo].astype(np.float64)
    even = (counts % 2) == 0
    if even.any():
        hi = np.minimum(mid_lo + 1, bounds[1:] - 1)
        med[even] = 0.5 * (h_sorted[mid_lo[even]] + h_sorted[hi[even]])

    valid_count = np.zeros((nu, nv), dtype=np.int32)
    median_h = np.full((nu, nv), np.nan)
    plane_label = np.zeros((nu, nv), dtype=np.uint8)
    mask_member = np.zeros((nu, nv), dtype=bool)
    kept_iu = kept_cells // nv
    kept_iv = kept_cells % nv
    valid_count[kept_iu, kept_iv] = counts
    median_h[kept_iu, kept_iv] = med

    # Per-cell majority plane label (<= 8 labels) and mask membership,
    # both via one bincount over (run, value) combos. np.argmax breaks
    # ties to the smallest value: deterministic.
    run_id = np.repeat(np.arange(n_cells), counts)
    labels_sorted = pixel_labels[order]
    n_label_values = int(labels_sorted.max()) + 1
    combo = run_id * n_label_values + labels_sorted
    label_counts = np.bincount(combo,
                               minlength=n_cells * n_label_values
                               ).reshape(n_cells, n_label_values)
    plane_label[kept_iu, kept_iv] = np.argmax(label_counts, axis=1)

    if instance_region is not None:
        region = np.asarray(instance_region, dtype=bool)
        mask_sorted = region[vu[order], uu[order]]
    else:
        mask_sorted = np.ones(len(order), dtype=bool)
    mask_counts = np.bincount(
        run_id * 2 + mask_sorted.astype(np.int64),
        minlength=n_cells * 2).reshape(n_cells, 2)
    mask_member[kept_iu, kept_iv] = mask_counts[:, 1] * 2 > counts

    # Robust surface: 5x5 nan-aware box smooth of the median grid. The
    # smoothed heights feed the evaluator's fit and its residual/step/
    # bimodal gates (raw per-cell medians still carry ~1.2 mm noise at
    # 2 mm pixel sigma because a 5 mm cell holds only 2-3 pixels — every
    # footprint would trip the 6 mm peak-to-valley gate). Local normals
    # take a +-2-cell central difference of the smoothed grid: a 2-cell
    # stencil on the raw medians carries ~10 deg gradient noise, which
    # tripped the 5 deg local-normal gate on every 2 mm-noise case.
    local_normal = np.full((nu, nv, 3), np.nan)
    smooth = np.full((nu, nv), np.nan)
    if nu >= 5 and nv >= 5:
        valid_grid = np.isfinite(median_h)
        safe_h = np.where(valid_grid, median_h, 0.0)
        s_h = np.zeros((nu + 1, nv + 1))
        s_h[1:, 1:] = np.cumsum(np.cumsum(safe_h, axis=0), axis=1)
        s_n = np.zeros((nu + 1, nv + 1))
        s_n[1:, 1:] = np.cumsum(np.cumsum(
            valid_grid.astype(np.int64), axis=0), axis=1)

        def _win_sum(s):
            ii = np.arange(2, nu - 2)[:, None]
            jj = np.arange(2, nv - 2)[None, :]
            acc = np.zeros((nu - 4, nv - 4), dtype=np.float64)
            for dr in (-2, -1, 0, 1, 2):
                for dc in (-2, -1, 0, 1, 2):
                    acc += (s[ii + dr + 1, jj + dc + 1]
                            - s[ii + dr, jj + dc + 1]
                            - s[ii + dr + 1, jj + dc]
                            + s[ii + dr, jj + dc])
            return acc

        cnt = np.zeros((nu, nv), dtype=np.float64)
        total = np.zeros((nu, nv), dtype=np.float64)
        cnt[2:-2, 2:-2] = _win_sum(s_n)
        total[2:-2, 2:-2] = _win_sum(s_h)
        enough = cnt >= 20
        smooth = np.where(enough, total / np.maximum(1, cnt), np.nan)
        good = enough[2:-2, 2:-2] & valid_grid[2:-2, 2:-2] \
            & np.isfinite(smooth[4:, 2:-2]) & np.isfinite(smooth[:-4, 2:-2]) \
            & np.isfinite(smooth[2:-2, 4:]) & np.isfinite(smooth[2:-2, :-4])
        dh_du = (smooth[4:, 2:-2] - smooth[:-4, 2:-2]) / (4.0 * cell)
        dh_dv = (smooth[2:-2, 4:] - smooth[2:-2, :-4]) / (4.0 * cell)
        nloc = np.stack((-dh_du, -dh_dv, np.ones_like(dh_du)), axis=-1)
        norm = np.linalg.norm(nloc, axis=-1)
        with np.errstate(invalid="ignore"):
            nloc = nloc / np.where(norm < 1e-12, np.nan, norm)[..., None]
        inner = local_normal[2:-2, 2:-2]
        inner[good] = (nloc[good, 0:1] * u_axis
                       + nloc[good, 1:2] * v_axis
                       + nloc[good, 2:3] * n_axis)

    return HeightMap(
        cell_size_m=cell, origin_u=origin_u, origin_v=origin_v,
        nu=nu, nv=nv, valid_count=valid_count, median_h=median_h,
        smooth_h=smooth, plane_label=plane_label, mask_member=mask_member,
        local_normal=local_normal, u_axis=u_axis, v_axis=v_axis,
        normal=n_axis, plane_d=plane_d,
        point_ids=order, point_cell_bounds=bounds, point_cells=kept_cells,
        point_h=h_sorted,
        point_uv=np.column_stack((pts_u[order], pts_v[order])))


def _empty_map(top, cell_size_m):
    u_axis, v_axis = _plane_basis(np.array([0.0, 0.0, 1.0]))
    return HeightMap(
        cell_size_m=float(cell_size_m), origin_u=0.0, origin_v=0.0,
        nu=0, nv=0,
        valid_count=np.zeros((0, 0), dtype=np.int32),
        median_h=np.zeros((0, 0)),
        smooth_h=np.zeros((0, 0)),
        plane_label=np.zeros((0, 0), dtype=np.uint8),
        mask_member=np.zeros((0, 0), dtype=bool),
        local_normal=np.zeros((0, 0, 3)),
        u_axis=u_axis, v_axis=v_axis, normal=np.array([0.0, 0.0, 1.0]),
        plane_d=-float(getattr(top, "top_z", 0.0) if top is not None
                       else 0.0),
        point_ids=np.zeros(0, dtype=np.int64),
        point_cell_bounds=np.zeros(1, dtype=np.int64),
        point_cells=np.zeros(0, dtype=np.int64),
        point_h=np.zeros(0), point_uv=np.zeros((0, 2)))
