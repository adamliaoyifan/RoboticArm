#!/usr/bin/env python3
"""Sealable suction-patch selection (DYNAMIC-SUCTION plan sections B and C).

ROS-free :class:`SuctionPatchEvaluator`: given one acquisition's aligned
depth, the ST-1 dynamic top (winning plane + per-pixel plane labels), the
original instance region and a plain :class:`ContactModel`, it

1. builds the 5 mm height map in the winning plane's coordinate system
   (:mod:`luggage_perception.suction_height_map`);
2. samples candidate centres on a 10 mm grid (the geometric centre is one
   sample among many, never privileged);
3. accepts a candidate only when ALL plan-B conditions hold (valid-cell
   coverage, mask coverage after the physical boundary margin, single
   connected plane label, RMS / P95 / peak-to-valley residual bounds,
   local-normal deviation, adjacent-step and bimodal height gates, +Z
   tilt, shared acquisition identity — the last holds by construction,
   one ``update`` call is one acquisition);
4. ranks accepted candidates by the plan's deterministic tuple
   (discontinuity count, valid coverage, P95 residual, boundary
   clearance, distance to the estimated box centre, lexicographic XY)
   and keeps at most ``max_candidates`` separated by >= 50 mm centre
   distance or footprint IoU <= 0.25.

Rejected candidates stay as bounded machine-readable diagnostics
(<= ``max_rejected_diagnostics``), never an unbounded message. Publishing
zero candidates is a valid outcome; the detection layer turns it into
``DETECT_NO_SEALABLE_PATCH``.

State lives only between ``update`` and ``copy_output``; outputs are deep
copies. No ROS/TF imports, no I/O, no RNG: identical inputs produce
byte-identical candidates (gate B6).
"""

from __future__ import division

import copy
import math
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from luggage_perception.suction_height_map import build_height_map

SUCTION_PATCH_NO_TOP = "SUCTION_PATCH_NO_TOP"
SUCTION_PATCH_NO_CELLS = "SUCTION_PATCH_NO_CELLS"
#: consumer-side gate: the selected candidate must share the acquisition
#: identity of the DetectedLuggage observation it is being planned from.
SUCTION_CANDIDATE_IDENTITY_MISMATCH = "SUCTION_CANDIDATE_IDENTITY_MISMATCH"

SUCTION_REJECT_VALID_CELLS = "SUCTION_REJECT_VALID_CELLS"
SUCTION_REJECT_MASK_COVERAGE = "SUCTION_REJECT_MASK_COVERAGE"
#: hard gate implementing B1's "selected position lies inside the true
#: surface eroded by the physical boundary margin": the four corners of
#: a 2*margin square centred on the candidate must project inside the
#: ORIGINAL instance-region pixels (the inscribed margin disk then lies
#: inside the region for any shape). The height map cannot prove it
#: alone — cells beyond the mask edge hold no points, exactly like
#: dropout holes — and the 95% mask fraction admits rotated corners
#: poking past the erosion, deterministically even at zero noise.
SUCTION_REJECT_BOUNDARY_MARGIN = "SUCTION_REJECT_BOUNDARY_MARGIN"
SUCTION_REJECT_PLANE_COVERAGE = "SUCTION_REJECT_PLANE_COVERAGE"
SUCTION_REJECT_NORMAL_TILT = "SUCTION_REJECT_NORMAL_TILT"
SUCTION_REJECT_RMS = "SUCTION_REJECT_RMS"
SUCTION_REJECT_P95 = "SUCTION_REJECT_P95"
SUCTION_REJECT_PEAK_TO_VALLEY = "SUCTION_REJECT_PEAK_TO_VALLEY"
SUCTION_REJECT_NORMAL_DEVIATION = "SUCTION_REJECT_NORMAL_DEVIATION"
SUCTION_REJECT_NORMAL_NO_SAMPLES = "SUCTION_REJECT_NORMAL_NO_SAMPLES"
SUCTION_REJECT_ADJACENT_STEP = "SUCTION_REJECT_ADJACENT_STEP"
SUCTION_REJECT_BIMODAL = "SUCTION_REJECT_BIMODAL"

#: neighbour offsets (di, dj) whose centre distance stays within the
#: 10 mm adjacency rule at a 5 mm cell size (max 2 cells = 10 mm).
_ADJACENT_OFFSETS = (
    (1, 0), (0, 1), (1, 1), (2, 0), (0, 2),
)


@dataclass
class SuctionCandidateRecord:
    """One accepted candidate (message adapters map this 1:1)."""

    candidate_id: str
    rank: int
    center_uv: tuple          # plane coords (u, v), metres
    center_world: tuple       # (x, y, z) on the fitted candidate plane
    quaternion_xyzw: tuple    # +Z along the candidate outward normal
    normal_world: tuple
    score: float
    valid_coverage: float
    mask_coverage: float
    plane_coverage: float
    rms_residual: float
    p95_residual: float
    peak_to_valley: float
    normal_deviation_p95: float
    max_adjacent_step: float
    boundary_clearance: float
    discontinuity_count: int
    stamp: float
    frame: str
    instance_id: str
    generation: int
    model_version: int
    model_hash: str


@dataclass
class SuctionPatchEvaluation:
    """Result of one acquisition; ``accepted`` may legitimately be empty."""

    ok: bool
    reason: str
    accepted: tuple = ()
    rejected: tuple = ()      # (candidate_id, reason, metrics-dict)
    stamp: float = 0.0
    frame: str = ""
    instance_id: str = ""
    generation: int = 0
    model_version: int = 0
    model_hash: str = ""
    timing_ms: float = 0.0


def suction_identity_mismatch(candidate, stamp, frame_id, instance_id,
                              generation, tolerance_sec=0.0):
    """None when the candidate shares the observation's identity.

    Otherwise ``(SUCTION_CANDIDATE_IDENTITY_MISMATCH, detail)``. Planning
    layers must reject a mismatched candidate before building any
    waypoint (plan section C). ``tolerance_sec`` allows float-stamp
    representation error only (default: exact).
    """
    if candidate is None:
        return (SUCTION_CANDIDATE_IDENTITY_MISMATCH, "no_candidate")
    checks = (
        ("stamp", float(candidate.stamp), float(stamp),
         float(tolerance_sec)),
        ("frame", str(candidate.frame), str(frame_id), 0.0),
        ("instance_id", str(candidate.instance_id), str(instance_id), 0.0),
        ("generation", int(candidate.generation), int(generation), 0.0),
    )
    for name, got, want, tol in checks:
        if isinstance(got, float):
            if abs(got - want) > tol:
                return (SUCTION_CANDIDATE_IDENTITY_MISMATCH,
                        "%s %.9f != %.9f" % (name, got, want))
        elif got != want:
            return (SUCTION_CANDIDATE_IDENTITY_MISMATCH,
                    "%s %r != %r" % (name, got, want))
    return None


def _integral(mask):
    """Zero-padded 2-D integral image of a bool/int array."""
    s = np.zeros((mask.shape[0] + 1, mask.shape[1] + 1),
                 dtype=np.int64)
    s[1:, 1:] = np.cumsum(np.cumsum(mask.astype(np.int64), axis=0),
                          axis=1)
    return s


def _rect_sums(s, i0, i1, j0, j1):
    """Vectorized rectangle sums from an integral image."""
    return (s[i1, j1] - s[i0, j1] - s[i1, j0] + s[i0, j0])


def _sorted_pct(sorted_rows, counts, q, offsets=None):
    """Linear-method percentile of per-row valid subsets.

    ``sorted_rows`` must be ascending along the last axis with invalid
    entries parked at ``+inf`` (valid block starts at index 0, offsets
    ``None``) or at ``-inf`` (``offsets`` gives each row's first valid
    index). Matches ``np.percentile(..., method="linear")`` on the
    valid subset of each row.
    """
    rows = np.arange(sorted_rows.shape[0])
    if offsets is None:
        base = np.zeros(sorted_rows.shape[0], dtype=np.int64)
    else:
        base = np.asarray(offsets, dtype=np.int64)
    counts = np.asarray(counts, dtype=np.int64)
    pos = base + q * np.maximum(counts - 1, 0)
    last = base + np.maximum(counts - 1, 0)
    k = np.clip(np.floor(pos).astype(np.int64), base, last)
    k1 = np.minimum(k + 1, last)
    lo = sorted_rows[rows, k]
    hi = sorted_rows[rows, k1]
    return lo + (pos - k) * (hi - lo)


def _bimodal_splits(sorted_heights, counts, min_fraction):
    """Per-row largest sorted-gap split, sides >= min_fraction each.

    Valid heights occupy ``[0, count)`` of each ascending-sorted row
    (invalid entries park at ``+inf``); mirrors the single-candidate
    ``h[k_min:m-k_min+1] - h[k_min-1:m-k_min]`` gap scan.
    """
    out = np.zeros(len(counts))
    rows = np.nonzero(counts >= 4)[0]
    if len(rows) == 0:
        return out
    m = counts[rows]
    k_min = np.maximum(1, np.ceil(min_fraction * m).astype(np.int64))
    tmax = int((m - 2 * k_min).max())
    if tmax < 0:
        return out
    t = np.arange(tmax + 1)
    ok = t[None, :] <= (m - 2 * k_min)[:, None]
    hs = sorted_heights[rows]
    r = np.arange(len(rows))[:, None]
    hi_idx = np.clip(k_min[:, None] + t[None, :], 0, hs.shape[1] - 1)
    lo_idx = np.clip(k_min[:, None] - 1 + t[None, :], 0, hs.shape[1] - 1)
    gaps = np.where(ok, hs[r, hi_idx] - hs[r, lo_idx], 0.0)
    out[rows] = gaps.max(axis=1)
    return out


def _step_violation_map(hmap, step_gate, max_distance):
    """Global adjacency-violation count per cell (robust surface, detrended).

    Measure: ``d(k)`` is the height difference between two adjacent
    6-cell (30 mm) block means of the SMOOTHED heights across boundary
    k; ``r(k) = d(k) - (d(k-1) + d(k+1)) / 2`` removes the local slope,
    so a planar tilt contributes ~0 and a hard step survives at near
    full height. A violation (``|r| > step_gate``) is credited to
    boundary cell k; per-candidate checks reduce to an integral-image
    rectangle sum (B6 latency). Field choice matches the per-window
    gate (``_max_adjacent_step``): raw per-cell medians carry
    sigma ~1.3-2 mm at real camera density (~2 points per 5 mm cell)
    whose block-mean max statistic false-trips the 4 mm gate on flat
    planes; the smoothed field's floor stays ~2.3 mm max.
    """
    block = 6
    raw = hmap.smooth_h
    valid = (hmap.valid_count > 0) & np.isfinite(hmap.smooth_h)
    violations = np.zeros((hmap.nu, hmap.nv), dtype=np.int64)
    for axis in (0, 1):
        safe = np.where(valid, raw, 0.0)
        csum = np.cumsum(safe, axis=axis)
        nsum = np.cumsum(valid.astype(np.float64), axis=axis)
        pad = [(0, 0), (0, 0)]
        pad[axis] = (1, 0)
        csum = np.pad(csum, pad, mode="constant")
        nsum = np.pad(nsum, pad, mode="constant")
        n = raw.shape[axis]

        def block_mean(starts):
            cs = (np.take(csum, np.clip(starts + block, 0, n), axis=axis)
                  - np.take(csum, np.clip(starts, 0, n), axis=axis))
            ns = (np.take(nsum, np.clip(starts + block, 0, n), axis=axis)
                  - np.take(nsum, np.clip(starts, 0, n), axis=axis))
            with np.errstate(invalid="ignore", divide="ignore"):
                return np.where(ns >= 3, cs / np.maximum(1.0, ns), np.nan)

        # Boundary k sits between the left block [k-block, k) and the
        # right block [k, k+block).
        ks = np.arange(block, n - block + 1)
        if len(ks) < 3:
            continue
        d = block_mean(ks) - block_mean(ks - block)
        # Detrend along the boundary axis with the neighbouring
        # boundaries' differences.
        mid = [slice(None), slice(None)]
        lo = [slice(None), slice(None)]
        hi = [slice(None), slice(None)]
        mid[axis], lo[axis], hi[axis] = (
            slice(1, -1), slice(None, -2), slice(2, None))
        r = d[tuple(mid)] - 0.5 * (d[tuple(lo)] + d[tuple(hi)])
        bad = np.isfinite(r) & (np.abs(r) > step_gate)
        if not bad.any():
            continue
        idx = np.nonzero(bad)
        other = 1 - axis
        coords = [None, None]
        coords[axis] = ks[idx[axis] + 1]
        coords[other] = idx[other]
        violations[coords[0], coords[1]] += 1
    return violations


def _quaternion_aligning_z_to(normal):
    """Minimal rotation taking world +Z onto ``normal`` (unit)."""
    z = np.array([0.0, 0.0, 1.0])
    axis = np.cross(z, normal)
    s = float(np.linalg.norm(axis))
    c = float(z @ normal)
    if s < 1e-12:
        if c > 0.0:
            return (0.0, 0.0, 0.0, 1.0)
        return (1.0, 0.0, 0.0, 0.0)   # 180 deg about X
    axis = axis / s
    angle = math.atan2(s, c)
    return (axis[0] * math.sin(angle / 2.0),
            axis[1] * math.sin(angle / 2.0),
            axis[2] * math.sin(angle / 2.0),
            math.cos(angle / 2.0))


class SuctionPatchEvaluator(object):
    """Stateful per-acquisition evaluator (update / copy_output)."""

    def __init__(self, contact_model):
        self._model = contact_model

    @property
    def contact_model(self):
        return self._model

    def update(self, depth_mm, intrinsics, optical_to_world, top,
               instance_region=None, stamp=0.0, frame_id="world",
               instance_id="", generation=0, timing=None):
        """Evaluate one acquisition; state is replaced, never fused.

        Every input belongs to the same acquisition identity (the caller
        enforces exact-stamp joins upstream); the identity is echoed on
        the output so consumers can reject mismatches with
        ``SUCTION_CANDIDATE_IDENTITY_MISMATCH`` before planning.
        """
        timing = timing if timing is not None else {}
        t0 = time.monotonic()
        if top is None or getattr(top, "reason", None) != "ok":
            self._output = SuctionPatchEvaluation(
                ok=False, reason=SUCTION_PATCH_NO_TOP,
                stamp=float(stamp), frame=str(frame_id),
                instance_id=str(instance_id), generation=int(generation),
                model_version=int(self._model.model_version),
                model_hash=str(self._model.identity_hash))
            return
        height_map = build_height_map(
            depth_mm, intrinsics, optical_to_world, top,
            instance_region=instance_region,
            cell_size_m=float(self._model.cell_size_m))
        timing["suction_height_map_ms"] = (
            time.monotonic() - t0) * 1000.0
        accepted, rejected = self._evaluate_candidates(
            height_map, top, stamp, frame_id, instance_id, generation,
            timing, instance_region=instance_region,
            intrinsics=intrinsics, optical_to_world=optical_to_world)
        self._output = SuctionPatchEvaluation(
            ok=True, reason="ok", accepted=accepted, rejected=rejected,
            stamp=float(stamp), frame=str(frame_id),
            instance_id=str(instance_id), generation=int(generation),
            model_version=int(self._model.model_version),
            model_hash=str(self._model.identity_hash),
            timing_ms=(time.monotonic() - t0) * 1000.0)

    def copy_output(self):
        """Independent copy; caller mutation cannot corrupt state."""
        return copy.deepcopy(self._output)

    # -- candidate scan ----------------------------------------------------

    def _evaluate_candidates(self, hmap, top, stamp, frame_id,
                             instance_id, generation, timing,
                             instance_region=None, intrinsics=None,
                             optical_to_world=None):
        model = self._model
        if hmap.nu == 0 or hmap.nv == 0:
            return (), ()
        nu, nv, cell = hmap.nu, hmap.nv, hmap.cell_size_m
        half_u = model.footprint_size_xy_m[0] / 2.0
        half_v = model.footprint_size_xy_m[1] / 2.0
        hf_u = half_u / cell          # footprint half-size in cells
        hf_v = half_v / cell
        mg_u = model.boundary_margin_m / cell
        mg_v = model.boundary_margin_m / cell

        # Grid of candidate centres (10 mm; include the geometric centre
        # of the labelled extent as one unprivileged sample).
        step = max(1, int(round(model.candidate_grid_m / cell)))
        centers_i = np.arange(int(math.floor(hf_u)),
                              max(int(math.floor(hf_u)) + 1,
                                  int(math.ceil(nu - hf_u))),
                              step)
        centers_j = np.arange(int(math.floor(hf_v)),
                              max(int(math.floor(hf_v)) + 1,
                                  int(math.ceil(nv - hf_v))),
                              step)
        gi = (nu - 1) // 2
        gj = (nv - 1) // 2
        if gi not in centers_i:
            centers_i = np.sort(np.append(centers_i, gi))
        if gj not in centers_j:
            centers_j = np.sort(np.append(centers_j, gj))
        grid_i, grid_j = np.meshgrid(centers_i, centers_j,
                                     indexing="ij")
        grid_i = grid_i.ravel()
        grid_j = grid_j.ravel()

        # Stage 1 (vectorized over the whole grid): valid-cell coverage
        # and post-margin mask coverage via integral images.
        valid_cell = hmap.valid_count > 0
        s_valid = _integral(valid_cell)
        s_mask = _integral(hmap.mask_member)
        i0 = np.maximum(0, grid_i - int(math.ceil(hf_u)))
        i1 = np.minimum(nu, grid_i + int(math.ceil(hf_u)) + 1)
        j0 = np.maximum(0, grid_j - int(math.ceil(hf_v)))
        j1 = np.minimum(nv, grid_j + int(math.ceil(hf_v)) + 1)
        n_cells = (i1 - i0) * (j1 - j0)
        valid_n = _rect_sums(s_valid, i0, i1, j0, j1)
        valid_frac = np.where(n_cells > 0, valid_n / np.maximum(
            1, n_cells), 0.0)
        mi0 = np.maximum(0, grid_i - int(math.floor(hf_u - mg_u)))
        mi1 = np.minimum(nu, grid_i + int(math.floor(hf_u - mg_u)) + 1)
        mj0 = np.maximum(0, grid_j - int(math.floor(hf_v - mg_v)))
        mj1 = np.minimum(nv, grid_j + int(math.floor(hf_v - mg_v)) + 1)
        m_area = (mi1 - mi0) * (mj1 - mj0)
        mask_frac = np.where(m_area > 0, _rect_sums(
            s_mask, mi0, mi1, mj0, mj1) / np.maximum(1, m_area), 0.0)

        # Stage 1 (vectorized): valid-cell coverage and post-margin mask
        # coverage gates as boolean masks; rejection tuples are built
        # lazily at the end so the bounded diagnostics budget keeps the
        # most informative rejections (highest valid coverage), not the
        # first grid-order ones.
        bad_valid = valid_frac < float(model.min_valid_cell_fraction)
        bad_frac = (~bad_valid) & (
            mask_frac < float(model.min_mask_coverage))
        frac_ok = np.nonzero(~(bad_valid | bad_frac))[0]
        # Boundary-margin gate on the survivors: the margin-eroded
        # footprint's corners must project inside the ORIGINAL instance
        # region (pixel-domain truth; the height map cannot distinguish
        # beyond-the-edge from dropout — both hold no points).
        outside = self._margin_outside_region(
            hmap, grid_i[frac_ok], grid_j[frac_ok], instance_region,
            intrinsics, optical_to_world)
        survivors = frac_ok[~outside]
        timing["suction_candidates_total"] = int(len(grid_i))
        timing["suction_candidates_stage1"] = int(len(survivors))
        if len(survivors) == 0:
            diag = self._stage1_diag(
                grid_i, grid_j, valid_frac, mask_frac, bad_valid,
                bad_frac)
            for k, out in zip(frac_ok, outside):
                if out:
                    diag.append((valid_frac[k], int(grid_i[k]),
                                 int(grid_j[k]),
                                 SUCTION_REJECT_BOUNDARY_MARGIN, {
                                     "valid_coverage": float(
                                         valid_frac[k])}))
            diag.sort(key=lambda d: (-d[0], d[1], d[2]))
            return (), tuple(
                ("C%03d_%03d" % (i, j), reason, metrics)
                for _, i, j, reason, metrics
                in diag[:int(model.max_rejected_diagnostics)])

        # Vectorized boundary clearance + distance-to-centre for every
        # survivor, then a cheap deterministic order. The expensive gates
        # (fit / residuals / steps / bimodal) run in bounded vectorized
        # chunks over that order; once enough gate-passing candidates
        # exist the remaining chunks are skipped (identical result to
        # full evaluation while bounding latency to the B6 budget).
        surv_i = grid_i[survivors]
        surv_j = grid_j[survivors]
        surv_vf = valid_frac[survivors]
        s_i0 = np.maximum(0, surv_i - int(math.ceil(hf_u)))
        s_i1 = np.minimum(nu, surv_i + int(math.ceil(hf_u)) + 1)
        s_j0 = np.maximum(0, surv_j - int(math.ceil(hf_v)))
        s_j1 = np.minimum(nv, surv_j + int(math.ceil(hf_v)) + 1)
        clearances = self._clearances(hmap, s_i0, s_i1, s_j0, s_j1)
        # Global adjacency-violation map: one pass over the height map,
        # then O(1) rectangle sums per candidate (B6 latency budget).
        step_map = _step_violation_map(
            hmap, float(model.max_adjacent_step_m),
            float(model.adjacent_step_distance_m))
        s_step = _integral(step_map)
        step_counts = _rect_sums(s_step, s_i0, s_i1, s_j0, s_j1)
        box_u, box_v = self._box_center_uv(hmap, top)
        center_u = hmap.origin_u + (surv_i + 0.5) * cell
        center_v = hmap.origin_v + (surv_j + 0.5) * cell
        dist_center = np.hypot(center_u - box_u, center_v - box_v)
        order = np.lexsort((surv_j, surv_i, dist_center, -clearances,
                            -surv_vf))

        survivors_step = order[step_counts[order] == 0]
        timing["suction_candidates_stage1p5"] = int(len(survivors_step))
        # Bounded full-evaluation scan (B6 latency budget): candidates
        # beyond the cap stay unevaluated and are reported as
        # scan_capped; high-clearance order puts crossing and island
        # candidates in the first chunks.
        scan_cap = 400
        scanned = survivors_step[:scan_cap]
        timing["suction_scan_capped"] = max(
            0, len(survivors_step) - len(scanned))
        accepted = []
        stage2_rejected = []
        n_stage2 = 0
        chunk_size = 64
        target = int(model.max_candidates) + 8
        for start in range(0, len(scanned), chunk_size):
            chunk = scanned[start:start + chunk_size]
            records, rejects = self._evaluate_batch(
                hmap, surv_i[chunk], surv_j[chunk],
                clearances[chunk],
                stamp, frame_id, instance_id, generation)
            accepted.extend(records)
            stage2_rejected.extend(rejects)
            n_stage2 += len(chunk)
            if len(accepted) >= target:
                break
        timing["suction_candidates_evaluated"] = int(
            len(accepted) + len(stage2_rejected))

        # Deterministic ranking tuple (plan section B).
        accepted.sort(key=lambda rec: (
            rec.discontinuity_count,
            -rec.valid_coverage,
            rec.p95_residual,
            -rec.boundary_clearance,
            math.hypot(rec.center_uv[0] - box_u,
                       rec.center_uv[1] - box_v),
            rec.center_uv[0], rec.center_uv[1]))

        kept = self._separate(accepted, model)
        # Bounded diagnostics: most informative first. Every rejection
        # (stage 1, 1.5, 2) carries its stage-1 valid coverage; sort by
        # (-valid_frac, i, j) so a full budget never hides the
        # near-passing candidates behind grid-order rim junk.
        diag = self._stage1_diag(
            grid_i, grid_j, valid_frac, mask_frac, bad_valid, bad_frac)
        for k, out in zip(frac_ok, outside):
            if out:
                diag.append((valid_frac[k], int(grid_i[k]),
                             int(grid_j[k]),
                             SUCTION_REJECT_BOUNDARY_MARGIN, {
                                 "valid_coverage": float(valid_frac[k])}))
        for k in order:
            if step_counts[k] == 0:
                continue
            diag.append((surv_vf[k], int(surv_i[k]), int(surv_j[k]),
                         SUCTION_REJECT_ADJACENT_STEP, {
                             "discontinuity_count": int(step_counts[k])}))
        for rec in stage2_rejected:
            diag.append((rec[3], rec[0], rec[1], rec[2], rec[4]))
        diag.sort(key=lambda d: (-d[0], d[1], d[2]))
        rejected = tuple(
            ("C%03d_%03d" % (i, j), reason, metrics)
            for _, i, j, reason, metrics
            in diag[:int(model.max_rejected_diagnostics)])
        return tuple(kept), rejected

    @staticmethod
    def _stage1_diag(grid_i, grid_j, valid_frac, mask_frac, bad_valid,
                     bad_frac):
        """Stage-1 rejection tuples (informative order, unsorted)."""
        diag = []
        for k in np.nonzero(bad_valid)[0]:
            diag.append((valid_frac[k], int(grid_i[k]), int(grid_j[k]),
                         SUCTION_REJECT_VALID_CELLS, {
                             "valid_coverage": float(valid_frac[k])}))
        for k in np.nonzero(bad_frac)[0]:
            diag.append((valid_frac[k], int(grid_i[k]), int(grid_j[k]),
                         SUCTION_REJECT_MASK_COVERAGE, {
                             "valid_coverage": float(valid_frac[k]),
                             "mask_coverage": float(mask_frac[k])}))
        return diag

    def _margin_outside_region(self, hmap, surv_i, surv_j, region,
                               intrinsics, optical_to_world):
        """Candidates whose centre is not margin-deep inside the region.

        Projects the four corners of a ``2*boundary_margin`` square
        centred on each candidate (top-plane frame) back into image
        pixels and samples the caller's instance region. The 15 mm disk
        is inscribed in that square, so all-corners-inside implies the
        centre lies inside the region eroded by the margin — B1's
        "selected position inside the true eroded surface" — for any
        region shape and any footprint size. The height map cannot
        prove this alone: cells beyond the mask edge hold no points,
        exactly like dropout holes.
        """
        n = len(surv_i)
        if n == 0:
            return np.zeros(0, dtype=bool)
        if region is None:
            return np.zeros(n, dtype=bool)
        margin = float(self._model.boundary_margin_m)
        region = np.asarray(region, dtype=bool)
        mat = np.asarray(optical_to_world, dtype=np.float64)
        rot, trans = mat[:3, :3], mat[:3, 3]
        fx = float(getattr(intrinsics, "fx", None) or intrinsics["fx"])
        fy = float(getattr(intrinsics, "fy", None) or intrinsics["fy"])
        cx = float(getattr(intrinsics, "cx", None) or intrinsics["cx"])
        cy = float(getattr(intrinsics, "cy", None) or intrinsics["cy"])
        h, w = region.shape
        cell = hmap.cell_size_m
        cu = hmap.origin_u + (surv_i + 0.5) * cell
        cv = hmap.origin_v + (surv_j + 0.5) * cell
        h_cell = hmap.median_h[surv_i, surv_j]
        h_ref = float(np.nanmedian(hmap.median_h))
        h_cell = np.where(np.isfinite(h_cell), h_cell, h_ref)
        center = (hmap.u_axis[None, :] * cu[:, None]
                  + hmap.v_axis[None, :] * cv[:, None]
                  + hmap.normal[None, :] * (h_cell - hmap.plane_d)
                  [:, None])
        inside_any = np.ones(n, dtype=bool)
        for su, sv in ((-margin, -margin), (-margin, margin),
                       (margin, -margin), (margin, margin)):
            corner = (center + su * hmap.u_axis[None, :]
                      + sv * hmap.v_axis[None, :])
            optical = (corner - trans[None, :]) @ rot
            oz = optical[:, 2]
            with np.errstate(divide="ignore", invalid="ignore"):
                px = np.rint(fx * optical[:, 0] / oz + cx).astype(
                    np.int64)
                py = np.rint(fy * optical[:, 1] / oz + cy).astype(
                    np.int64)
            inb = (oz > 0.0) & (px >= 0) & (px < w) & (py >= 0) \
                & (py < h)
            sample = np.zeros(n, dtype=bool)
            sample[inb] = region[py[inb], px[inb]]
            inside_any &= sample
        return ~inside_any

    def _clearances(self, hmap, i0s, i1s, j0s, j1s):
        """Vectorized ring clearance for an array of footprints.

        Grows every footprint rectangle ring by ring (clipped to the
        map); a candidate's clearance is the first ring whose BAND adds a
        bad cell (invalid or outside the instance mask), or the map rim
        when expansion is exhausted. Interior imperfections inside the
        footprint itself do not reduce edge clearance.
        """
        good = (hmap.valid_count > 0) & hmap.mask_member
        s_good = _integral(good)
        nu, nv = hmap.nu, hmap.nv
        n = len(i0s)
        clearance = np.zeros(n)
        active = np.ones(n, dtype=bool)
        prev_area = (i1s - i0s) * (j1s - j0s)
        prev_bad = prev_area - _rect_sums(s_good, i0s, i1s, j0s, j1s)
        ring = 0
        while active.any():
            ring += 1
            ri0 = np.maximum(0, i0s - ring)
            ri1 = np.minimum(nu, i1s + ring)
            rj0 = np.maximum(0, j0s - ring)
            rj1 = np.minimum(nv, j1s + ring)
            area = (ri1 - ri0) * (rj1 - rj0)
            bad = area - _rect_sums(s_good, ri0, ri1, rj0, rj1)
            # Capped at the map rim when the rectangle no longer grows;
            # else bounded by the first ring that adds a bad cell.
            stop = (area == prev_area) | (bad > prev_bad)
            clearance[active & stop] = (ring - 1) * hmap.cell_size_m
            active &= ~stop
            prev_area = np.where(stop, prev_area, area)
            prev_bad = np.where(stop, prev_bad, bad)
        return clearance

    @staticmethod
    def _box_center_uv(hmap, top):
        center_world = np.array([
            float(top.center_xy[0]), float(top.center_xy[1]),
            float(top.top_z)])
        return (float(center_world @ hmap.u_axis),
                float(center_world @ hmap.v_axis))

    def _evaluate_batch(self, hmap, ci_arr, cj_arr, clearances,
                        stamp, frame_id, instance_id, generation):
        """Stage-2 gates for one vectorized chunk of candidates.

        Semantics equal per-candidate evaluation, through a single
        vectorized code path (the B6 determinism and latency budget:
        repeated calls are byte-identical and no per-candidate LAPACK /
        percentile calls remain). Windows are uniform because candidate
        centres sit >= ceil(half-footprint) cells from the map rim and
        every configured footprint is an integer number of cells.
        Returns ``(accepted_records, rejects)`` with rejects as
        ``(i, j, reason, valid_frac, metrics)`` tuples.
        """
        model = self._model
        cell = hmap.cell_size_m
        hf_u = model.footprint_size_xy_m[0] / 2.0 / cell
        hf_v = model.footprint_size_xy_m[1] / 2.0 / cell
        if hf_u != int(hf_u) or hf_v != int(hf_v):
            raise NotImplementedError(
                "batched evaluator requires an integer footprint "
                "half-size in cells (got %r, %r)" % (hf_u, hf_v))
        hf_u = int(hf_u)
        hf_v = int(hf_v)
        wi = 2 * hf_u + 1
        wj = 2 * hf_v + 1
        n = len(ci_arr)
        if n == 0:
            return [], []

        def windows(field):
            """Gather (n, wi, wj) candidate windows from a 2-D field."""
            view = np.lib.stride_tricks.sliding_window_view(
                field, (wi, wj))
            return view[ci_arr - hf_u, cj_arr - hf_v]

        # Gates run on the robust (smoothed) surface; a valid cell needs
        # both depth presence and a smoothable neighbourhood.
        valid = (windows(hmap.valid_count) > 0) \
            & np.isfinite(windows(hmap.smooth_h))
        win_smooth = windows(hmap.smooth_h)
        flat_valid = valid.reshape(n, -1)
        n_cells = flat_valid.shape[1]
        n_valid = flat_valid.sum(axis=1).astype(np.int64)
        vfrac = n_valid / float(n_cells)

        # Majority plane label of each footprint's own valid cells; ties
        # break to the smallest label (argmax over ascending counts).
        labels = windows(hmap.plane_label).reshape(n, -1)
        n_labels = int(hmap.plane_label.max()) + 1
        onehot = labels[:, :, None] == np.arange(n_labels)[None, None, :]
        counts = (onehot & flat_valid[:, :, None]).sum(axis=1)
        label_frac = np.where(n_valid > 0,
                              counts.max(axis=1)
                              / np.maximum(1, n_valid), 0.0)

        # Total-least-squares plane fit from window-local moment sums
        # (translation-invariant: same normal the per-candidate SVD
        # produced; heights stay absolute plane heights).
        lu = (np.arange(wi) - hf_u + 0.5) * cell
        lv = (np.arange(wj) - hf_v + 0.5) * cell
        LU, LV = np.meshgrid(lu, lv, indexing="ij")
        u_lin = np.broadcast_to(LU.ravel(), (n, n_cells))
        v_lin = np.broadcast_to(LV.ravel(), (n, n_cells))
        h_lin = np.where(flat_valid, win_smooth.reshape(n, n_cells), 0.0)
        su = (flat_valid * u_lin).sum(axis=1)
        sv = (flat_valid * v_lin).sum(axis=1)
        sh = h_lin.sum(axis=1)
        suu = (flat_valid * u_lin * u_lin).sum(axis=1)
        svv = (flat_valid * v_lin * v_lin).sum(axis=1)
        suv = (flat_valid * u_lin * v_lin).sum(axis=1)
        suh = (u_lin * h_lin).sum(axis=1)
        svh = (v_lin * h_lin).sum(axis=1)
        shh = (h_lin * h_lin).sum(axis=1)
        inv_n = 1.0 / np.maximum(n_valid, 1)
        c_u = su * inv_n
        c_v = sv * inv_n
        c_h = sh * inv_n
        s00 = suu - n_valid * c_u * c_u
        s01 = suv - n_valid * c_u * c_v
        s02 = suh - n_valid * c_u * c_h
        s11 = svv - n_valid * c_v * c_v
        s12 = svh - n_valid * c_v * c_h
        s22 = shh - n_valid * c_h * c_h
        scatter = np.empty((n, 3, 3))
        scatter[:, 0, 0] = s00
        scatter[:, 1, 1] = s11
        scatter[:, 2, 2] = s22
        scatter[:, 0, 1] = scatter[:, 1, 0] = s01
        scatter[:, 0, 2] = scatter[:, 2, 0] = s02
        scatter[:, 1, 2] = scatter[:, 2, 1] = s12
        try:
            eigvals, eigvecs = np.linalg.eigh(scatter)
        except np.linalg.LinAlgError:
            eigvals = np.full((n, 3), np.nan)
            eigvecs = np.zeros((n, 3, 3))
        fit_ok = np.isfinite(eigvals[:, 0])
        normal = eigvecs[:, :, 0]                 # smallest eigenvalue
        flip = normal[:, 2] < 0.0
        normal[flip] = -normal[flip]
        n_u, n_v, n_z = normal[:, 0], normal[:, 1], normal[:, 2]
        # Plane through the centroid in window-local coords; the
        # candidate centre sits at local (0.5*cell, 0.5*cell).
        d_fit = -(n_u * c_u + n_v * c_v + n_z * c_h)
        tilt_deg = np.degrees(np.arccos(np.clip(np.abs(n_z), 0.0, 1.0)))

        # Residuals per row, sorted ascending; invalid cells park at
        # +/-inf so per-row percentiles index the valid subset exactly
        # like np.percentile(method="linear").
        res = (n_u[:, None, None] * LU[None]
               + n_v[:, None, None] * LV[None]
               + n_z[:, None, None] * win_smooth
               + d_fit[:, None, None])
        res = np.where(valid, res, np.inf)
        res_sorted = np.sort(res.reshape(n, -1), axis=1)
        res2 = np.where(valid, res * res, 0.0).reshape(n, -1).sum(axis=1)
        rms = np.sqrt(res2 * inv_n)
        abs_sorted = np.sort(np.where(valid, np.abs(res), np.inf).reshape(
            n, -1), axis=1)
        p95 = _sorted_pct(abs_sorted, n_valid, 0.95)
        pv = _sorted_pct(res_sorted, n_valid, 0.99) \
            - _sorted_pct(res_sorted, n_valid, 0.01)

        # Local-normal agreement with the fitted plane normal.
        n_view = np.lib.stride_tricks.sliding_window_view(
            hmap.local_normal, (wi, wj, 3))[ci_arr - hf_u, cj_arr - hf_v, 0]
        norm_ok = valid & np.isfinite(n_view).all(axis=-1)
        n_norm = norm_ok.reshape(n, -1).sum(axis=1).astype(np.int64)
        normal_world = (n_u[:, None] * hmap.u_axis
                        + n_v[:, None] * hmap.v_axis
                        + n_z[:, None] * hmap.normal)
        dots = np.einsum("nijc,nc->nij",
                         np.where(norm_ok[:, :, :, None], n_view, 0.0),
                         normal_world)
        dev_flat = np.where(
            norm_ok,
            np.degrees(np.arccos(np.clip(dots, -1.0, 1.0))),
            -np.inf).reshape(n, -1)
        dev_sorted = np.sort(dev_flat, axis=1)
        dev_p95 = _sorted_pct(dev_sorted, n_norm, 0.95,
                              offsets=n_cells - n_norm)

        # Sustained-discontinuity and bimodal gates.
        step, step_count = self._max_adjacent_step(
            win_smooth, valid, cell,
            float(model.adjacent_step_distance_m),
            float(model.max_adjacent_step_m))
        hs = np.sort(np.where(valid, win_smooth, np.inf).reshape(
            n, -1), axis=1)
        bimodal = _bimodal_splits(hs, n_valid,
                                  float(model.bimodal_min_fraction))

        # Gate priority identical to the historical single-candidate
        # order: the first failing condition names the rejection.
        conds = [
            label_frac < float(model.min_connected_plane_fraction),
            n_valid < 3,
            ~fit_ok,
            tilt_deg > float(model.max_normal_tilt_deg),
            rms > float(model.max_rms_residual_m),
            p95 > float(model.max_p95_residual_m),
            pv > float(model.max_peak_to_valley_m),
            n_norm < 3,
            dev_p95 > float(model.max_normal_deviation_p95_deg),
            step_count > 0,
            bimodal >= float(model.bimodal_min_separation_m),
        ]
        reasons = np.select(conds, [
            SUCTION_REJECT_PLANE_COVERAGE,
            SUCTION_REJECT_VALID_CELLS,
            SUCTION_REJECT_RMS,               # degenerate fit
            SUCTION_REJECT_NORMAL_TILT,
            SUCTION_REJECT_RMS,
            SUCTION_REJECT_P95,
            SUCTION_REJECT_PEAK_TO_VALLEY,
            SUCTION_REJECT_NORMAL_NO_SAMPLES,
            SUCTION_REJECT_NORMAL_DEVIATION,
            SUCTION_REJECT_ADJACENT_STEP,
            SUCTION_REJECT_BIMODAL], default="")

        rejects = []
        for r in np.nonzero(reasons != "")[0]:
            reason = str(reasons[r])
            if reason == SUCTION_REJECT_PLANE_COVERAGE:
                metrics = {"plane_coverage": float(label_frac[r])}
            elif reason == SUCTION_REJECT_VALID_CELLS:
                metrics = {"valid_cells": int(n_valid[r])}
            elif reason == SUCTION_REJECT_NORMAL_TILT:
                metrics = {"tilt_deg": float(tilt_deg[r])}
            elif reason == SUCTION_REJECT_P95:
                metrics = {"p95": float(p95[r])}
            elif reason == SUCTION_REJECT_PEAK_TO_VALLEY:
                metrics = {"peak_to_valley": float(pv[r])}
            elif reason == SUCTION_REJECT_NORMAL_NO_SAMPLES:
                metrics = {"normal_samples": int(n_norm[r])}
            elif reason == SUCTION_REJECT_NORMAL_DEVIATION:
                metrics = {"normal_deviation_p95": float(dev_p95[r])}
            elif reason == SUCTION_REJECT_ADJACENT_STEP:
                metrics = {"max_adjacent_step": float(step[r]),
                           "discontinuity_count": int(step_count[r])}
            elif reason == SUCTION_REJECT_BIMODAL:
                metrics = {"bimodal_separation": float(bimodal[r])}
            elif reason == SUCTION_REJECT_RMS:
                metrics = ({"fit": "degenerate"} if not fit_ok[r]
                           else {"rms": float(rms[r])})
            else:
                metrics = {}
            rejects.append((int(ci_arr[r]), int(cj_arr[r]), reason,
                            float(vfrac[r]), metrics))

        mask_frac = (windows(hmap.mask_member).reshape(n, -1)
                     & flat_valid).sum(axis=1) / np.maximum(1, n_valid)
        records = []
        for r in np.nonzero(reasons == "")[0]:
            cid = "C%03d_%03d" % (int(ci_arr[r]), int(cj_arr[r]))
            center_uv = (hmap.origin_u + (ci_arr[r] + 0.5) * cell,
                         hmap.origin_v + (cj_arr[r] + 0.5) * cell)
            # Centre height on the fitted plane at the candidate centre
            # (window-local coords: candidate centre at (0.5, 0.5)*cell).
            h_c = -(d_fit[r] + (n_u[r] + n_v[r]) * 0.5 * cell) / n_z[r]
            center_world = (hmap.u_axis * center_uv[0]
                            + hmap.v_axis * center_uv[1]
                            + hmap.normal * (h_c - hmap.plane_d))
            normal_w = normal_world[r]
            clearance = float(clearances[r])
            score = float(
                0.5 * vfrac[r]
                + 0.3 * (1.0 - min(1.0, p95[r]
                                   / float(model.max_p95_residual_m)))
                + 0.2 * min(1.0, clearance / 0.05))
            records.append(SuctionCandidateRecord(
                candidate_id=cid, rank=0,
                center_uv=(float(center_uv[0]), float(center_uv[1])),
                center_world=(float(center_world[0]),
                              float(center_world[1]),
                              float(center_world[2])),
                quaternion_xyzw=_quaternion_aligning_z_to(normal_w),
                normal_world=(float(normal_w[0]), float(normal_w[1]),
                              float(normal_w[2])),
                score=score, valid_coverage=float(vfrac[r]),
                mask_coverage=float(mask_frac[r]),
                plane_coverage=float(label_frac[r]),
                rms_residual=float(rms[r]), p95_residual=float(p95[r]),
                peak_to_valley=float(pv[r]),
                normal_deviation_p95=float(dev_p95[r]),
                max_adjacent_step=float(step[r]),
                boundary_clearance=clearance,
                discontinuity_count=int(step_count[r]),
                stamp=float(stamp), frame=str(frame_id),
                instance_id=str(instance_id), generation=int(generation),
                model_version=int(model.model_version),
                model_hash=str(model.identity_hash)))
        return records, rejects

    @staticmethod
    def _max_adjacent_step(heights, valid, cell_size, max_distance,
                           step_gate):
        """Largest height discontinuity and sustained-violation count.

        The gate field is the robust (smoothed) cell height: raw
        per-cell medians carry sigma ~1.3-2 mm of pixel noise at real
        camera density (~2 points per 5 mm cell), and the max of their
        20 mm block-mean differences over a footprint window exceeds
        the 4 mm gate on physically flat planes (measured 5.1-5.3 mm at
        footprint 0.18 m / noise 2 mm). On the smoothed field the same
        statistic stays <= ~2.3 mm while a hard step of any boundary
        angle survives at >= 0.84x its height (verified numerically:
        6 mm -> >= 5.04 mm), so the frozen 4 mm threshold keeps its
        exact-``>`` semantics. A violation COUNTS only when confirmed by
        an adjacent boundary on the same axis: physical steps produce
        runs of violations, isolated noise spikes do not. ``heights``
        and ``valid`` may carry leading batch dimensions. Returns
        ``(max_step_m, confirmed_count)``; ``max_step_m`` is the largest
        finite block difference (reported diagnostically even when the
        count is zero).
        """
        block = 4
        heights = np.asarray(heights)
        valid = np.asarray(valid)
        ndim = heights.ndim
        lead_shape = heights.shape[:-2]
        max_step = np.zeros(lead_shape, dtype=np.float64)
        violations = np.zeros(lead_shape, dtype=np.int64)
        for axis in (ndim - 2, ndim - 1):
            safe = np.where(valid, heights, 0.0)
            csum = np.cumsum(safe, axis=axis)
            nsum = np.cumsum(valid.astype(np.float64), axis=axis)
            pad = [(0, 0)] * ndim
            pad[axis] = (1, 0)
            csum = np.pad(csum, pad, mode="constant")
            nsum = np.pad(nsum, pad, mode="constant")
            n = heights.shape[axis]

            def block_mean(starts):
                """(...,) mean over [k, k+block) along the axis."""
                cs = (np.take(csum, np.clip(starts + block, 0, n),
                              axis=axis)
                      - np.take(csum, np.clip(starts, 0, n), axis=axis))
                ns = (np.take(nsum, np.clip(starts + block, 0, n),
                              axis=axis)
                      - np.take(nsum, np.clip(starts, 0, n), axis=axis))
                with np.errstate(invalid="ignore", divide="ignore"):
                    return np.where(ns >= 2, cs / np.maximum(1.0, ns),
                                    np.nan)

            for off in (1, 2):
                if off * cell_size > max_distance + 1e-9:
                    continue
                # left block ends at k, right block starts at k + off
                ks = np.arange(1, n - block + 1)
                diff = np.abs(block_mean(ks - block)
                              - block_mean(ks + off))
                finite = np.isfinite(diff)
                if finite.any():
                    max_step = np.maximum(
                        max_step,
                        np.where(finite, diff, -np.inf).max(
                            axis=(-2, -1)))
                bad = finite & (diff > step_gate)
                if not bad.any():
                    continue
                prev = np.roll(bad, 1, axis=axis)
                nxt = np.roll(bad, -1, axis=axis)
                # Zero the wrap-around copies at the axis edges.
                edge_lo = [slice(None)] * ndim
                edge_hi = [slice(None)] * ndim
                edge_lo[axis] = 0
                edge_hi[axis] = -1
                prev[tuple(edge_lo)] = False
                nxt[tuple(edge_hi)] = False
                conf = bad & (prev | nxt)
                violations = violations + conf.sum(axis=(-2, -1))
        return max_step, violations


    def _separate(self, ranked, model):
        """Greedy separation: >= 50 mm centre distance or IoU <= 0.25."""
        kept = []
        half = (model.footprint_size_xy_m[0] / 2.0,
                model.footprint_size_xy_m[1] / 2.0)
        for record in ranked:
            if len(kept) >= int(model.max_candidates):
                break
            separated = True
            for other in kept:
                du = record.center_uv[0] - other.center_uv[0]
                dv = record.center_uv[1] - other.center_uv[1]
                if math.hypot(du, dv) \
                        >= float(model.min_candidate_separation_m):
                    continue
                iou = _rectangle_iou(record.center_uv, other.center_uv,
                                     half)
                if iou > float(model.max_candidate_iou):
                    separated = False
                    break
            if separated:
                record.rank = len(kept) + 1
                kept.append(record)
        return kept


def _rectangle_iou(center_a, center_b, half):
    inter_u = max(0.0, min(center_a[0] + half[0], center_b[0] + half[0])
                  - max(center_a[0] - half[0], center_b[0] - half[0]))
    inter_v = max(0.0, min(center_a[1] + half[1], center_b[1] + half[1])
                  - max(center_a[1] - half[1], center_b[1] - half[1]))
    inter = inter_u * inter_v
    if inter <= 0.0:
        return 0.0
    area = 4.0 * half[0] * half[1]
    return inter / area


def _fit_plane(points):
    """LSQ plane (unit normal +Z-ish, offset) or (None, None).

    Retained for evidence/diagnostic tooling; the evaluator's batched
    path fits through window-local moment sums instead.
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
    return normal, -float(normal @ centroid)
