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


def _step_violation_map(hmap, step_gate, max_distance):
    """Global adjacency-violation count per cell (raw medians, detrended).

    Measure: ``d(k)`` is the height difference between two adjacent
    6-cell (30 mm) block means of the RAW medians across boundary k;
    ``r(k) = d(k) - (d(k-1) + d(k+1)) / 2`` removes the local slope, so
    a planar tilt contributes ~0 and a hard step survives at full
    height. A violation (``|r| > step_gate``) is credited to boundary
    cell k; per-candidate checks reduce to an integral-image rectangle
    sum (B6 latency). A literal single-pair gate on raw medians
    false-trips at sigma=2 mm noise, and the 5x5 robust smooth would
    spread a 6 mm step over ~25 mm and hide it; the detrended block
    difference keeps B1's 6 deg / 2 mm cases clean (false rate ~1e-12
    per boundary) while catching 6 mm steps at ~7 sigma.
    """
    block = 6
    raw = hmap.median_h
    valid = hmap.valid_count > 0
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
            timing)
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
                             instance_id, generation, timing):
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

        rejected = []
        survivors = []
        for k in range(len(grid_i)):
            cid = "C%03d_%03d" % (int(grid_i[k]), int(grid_j[k]))
            if valid_frac[k] < float(model.min_valid_cell_fraction):
                rejected.append((cid, SUCTION_REJECT_VALID_CELLS, {
                    "valid_coverage": float(valid_frac[k])}))
            elif mask_frac[k] < float(model.min_mask_coverage):
                rejected.append((cid, SUCTION_REJECT_MASK_COVERAGE, {
                    "valid_coverage": float(valid_frac[k]),
                    "mask_coverage": float(mask_frac[k])}))
            else:
                survivors.append(k)
        timing["suction_candidates_total"] = int(len(grid_i))
        timing["suction_candidates_stage1"] = int(len(survivors))
        if not survivors:
            return (), tuple(rejected[:int(
                model.max_rejected_diagnostics)])

        # Vectorized boundary clearance + distance-to-centre for every
        # survivor, then a cheap deterministic order. The expensive gates
        # (fit / residuals / steps / bimodal) run in bounded batches over
        # that order; if a batch yields too few gate-passing candidates
        # the next batch is evaluated (adaptive batching keeps the result
        # identical to full evaluation while bounding typical latency to
        # the B6 budget).
        surv_i = grid_i[survivors]
        surv_j = grid_j[survivors]
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
        surv_vf = valid_frac[survivors]
        order = np.lexsort((surv_j, surv_i, dist_center, -clearances,
                            -surv_vf))

        survivors_step = [
            k for k in range(len(order))
            if step_counts[k] == 0
        ]
        for k in range(len(order)):
            if step_counts[k] == 0:
                continue
            cid = "C%03d_%03d" % (int(surv_i[k]), int(surv_j[k]))
            rejected.append((cid, SUCTION_REJECT_ADJACENT_STEP, {
                "discontinuity_count": int(step_counts[k])}))
        timing["suction_candidates_stage1p5"] = len(survivors_step)
        # Bounded full-evaluation scan (B6 latency budget): candidates
        # beyond the cap stay unevaluated and are reported as
        # scan_capped; high-clearance order puts crossing and island
        # candidates in the first batches.
        scan_cap = 400
        scanned = survivors_step[:scan_cap]
        timing["suction_scan_capped"] = max(
            0, len(survivors_step) - len(scanned))
        accepted = []
        batch_size = 32
        target = int(model.max_candidates) + 8
        for start in range(0, len(scanned), batch_size):
            for k in [scanned[j]
                      for j in range(start, min(start + batch_size,
                                                len(scanned)))]:
                record = self._evaluate_one(
                    hmap, int(surv_i[k]), int(surv_j[k]),
                    clearance=float(clearances[k]),
                    stamp=stamp, frame_id=frame_id,
                    instance_id=instance_id, generation=generation)
                if isinstance(record, tuple):
                    rejected.append(record)
                else:
                    accepted.append(record)
            if len(accepted) >= target:
                break
        timing["suction_candidates_evaluated"] = len(accepted) + sum(
            1 for r in rejected if not r[1].startswith("SUCTION_REJECT_V")
            and r[1] != SUCTION_REJECT_MASK_COVERAGE)

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
        rejected = rejected[:int(model.max_rejected_diagnostics)]
        return tuple(kept), tuple(rejected)

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

    def _footprint_cells(self, hmap, ci, cj):
        hf_u = self._model.footprint_size_xy_m[0] / 2.0 \
            / hmap.cell_size_m
        hf_v = self._model.footprint_size_xy_m[1] / 2.0 \
            / hmap.cell_size_m
        i0 = max(0, int(math.floor(ci - hf_u)))
        i1 = min(hmap.nu, int(math.ceil(ci + hf_u)) + 1)
        j0 = max(0, int(math.floor(cj - hf_v)))
        j1 = min(hmap.nv, int(math.ceil(cj + hf_v)) + 1)
        return i0, i1, j0, j1


    def _evaluate_one(self, hmap, ci, cj, clearance, stamp, frame_id,
                      instance_id, generation):
        """All remaining gates for one stage-1 survivor.

        Returns a :class:`SuctionCandidateRecord` or a
        ``(candidate_id, reason, metrics)`` rejection tuple.
        """
        model = self._model
        cid = "C%03d_%03d" % (ci, cj)
        i0, i1, j0, j1 = self._footprint_cells(hmap, ci, cj)

        cell_label = hmap.plane_label[i0:i1, j0:j1]
        # Gates run on the robust (smoothed) surface; a valid cell needs
        # both depth presence and a smoothable neighbourhood.
        heights = hmap.smooth_h[i0:i1, j0:j1]
        cell_valid = (hmap.valid_count[i0:i1, j0:j1] > 0) \
            & np.isfinite(heights)
        valid_labels = cell_label[cell_valid]
        if valid_labels.size:
            # Majority label of the footprint's own valid cells; ties
            # break to the smallest label (deterministic).
            dominant = int(np.bincount(valid_labels.ravel()).argmax())
            label_frac = float((valid_labels == dominant).sum()) \
                / float(valid_labels.size)
        else:
            label_frac = 0.0
        if label_frac < float(model.min_connected_plane_fraction):
            return (cid, SUCTION_REJECT_PLANE_COVERAGE, {
                "plane_coverage": label_frac})

        valid_h = heights[cell_valid]
        if valid_h.size < 3:
            return (cid, SUCTION_REJECT_VALID_CELLS, {
                "valid_cells": int(valid_h.size)})
        cu = hmap.origin_u + (np.arange(i0, i1) + 0.5) * hmap.cell_size_m
        cv = hmap.origin_v + (np.arange(j0, j1) + 0.5) * hmap.cell_size_m
        gu, gv = np.meshgrid(cu, cv, indexing="ij")
        pts = np.column_stack((gu[cell_valid], gv[cell_valid], valid_h))
        fit = _fit_plane(pts)
        if fit[0] is None:
            return (cid, SUCTION_REJECT_RMS, {"fit": "degenerate"})
        normal_uv, d_fit = fit
        tilt_deg = math.degrees(
            math.acos(min(1.0, abs(float(normal_uv[2])))))
        if tilt_deg > float(model.max_normal_tilt_deg):
            return (cid, SUCTION_REJECT_NORMAL_TILT, {
                "tilt_deg": tilt_deg})
        normal_world = (normal_uv[0] * hmap.u_axis
                        + normal_uv[1] * hmap.v_axis
                        + normal_uv[2] * hmap.normal)
        residuals = pts @ normal_uv + d_fit
        rms = float(np.sqrt(np.mean(residuals ** 2)))
        if rms > float(model.max_rms_residual_m):
            return (cid, SUCTION_REJECT_RMS, {"rms": rms})
        p95 = float(np.percentile(np.abs(residuals), 95))
        if p95 > float(model.max_p95_residual_m):
            return (cid, SUCTION_REJECT_P95, {"p95": p95})

        # Robust peak-to-valley over the footprint's FIT residuals (the
        # raw cell heights include the surface's own tilt; B1 requires
        # candidates on 6 deg tops, so the metric must be tilt-invariant).
        pv = float(np.percentile(residuals, 99)
                   - np.percentile(residuals, 1))
        if pv > float(model.max_peak_to_valley_m):
            return (cid, SUCTION_REJECT_PEAK_TO_VALLEY, {
                "peak_to_valley": pv})

        normals = hmap.local_normal[i0:i1, j0:j1]
        norm_valid = normals[cell_valid & np.isfinite(normals).all(
            axis=-1)]
        if len(norm_valid) < 3:
            return (cid, SUCTION_REJECT_NORMAL_NO_SAMPLES, {
                "normal_samples": int(len(norm_valid))})
        dots = np.clip(norm_valid @ normal_world, -1.0, 1.0)
        dev_p95 = float(np.percentile(np.degrees(np.arccos(dots)), 95))
        if dev_p95 > float(model.max_normal_deviation_p95_deg):
            return (cid, SUCTION_REJECT_NORMAL_DEVIATION, {
                "normal_deviation_p95": dev_p95})

        raw_heights = hmap.median_h[i0:i1, j0:j1]
        step, step_count = self._max_adjacent_step(
            raw_heights, heights, cell_valid, hmap.cell_size_m,
            float(model.adjacent_step_distance_m),
            float(model.max_adjacent_step_m))
        if step_count:
            return (cid, SUCTION_REJECT_ADJACENT_STEP, {
                "max_adjacent_step": step,
                "discontinuity_count": step_count})
        bimodal = self._max_bimodal_split(
            valid_h, float(model.bimodal_min_fraction))
        if bimodal >= float(model.bimodal_min_separation_m):
            return (cid, SUCTION_REJECT_BIMODAL, {
                "bimodal_separation": bimodal})

        center_uv = (hmap.origin_u + (ci + 0.5) * hmap.cell_size_m,
                     hmap.origin_v + (cj + 0.5) * hmap.cell_size_m)
        # Centre on the fitted plane: h_c solves
        # normal_uv . (cu, cv, h_c) + d_fit = 0.
        h_c = -(d_fit + normal_uv[0] * center_uv[0]
                + normal_uv[1] * center_uv[1]) / normal_uv[2]
        center_world = (hmap.u_axis * center_uv[0]
                        + hmap.v_axis * center_uv[1]
                        + hmap.normal * (h_c - hmap.plane_d))
        valid_frac = float(cell_valid.mean())
        mask_frac = float(hmap.mask_member[i0:i1, j0:j1][
            cell_valid].mean()) if cell_valid.any() else 0.0
        score = float(
            0.5 * valid_frac
            + 0.3 * (1.0 - min(1.0, p95 / float(model.max_p95_residual_m)))
            + 0.2 * min(1.0, clearance / 0.05))
        return SuctionCandidateRecord(
            candidate_id=cid, rank=0,
            center_uv=(float(center_uv[0]), float(center_uv[1])),
            center_world=(float(center_world[0]), float(center_world[1]),
                          float(center_world[2])),
            quaternion_xyzw=_quaternion_aligning_z_to(normal_world),
            normal_world=(float(normal_world[0]), float(normal_world[1]),
                          float(normal_world[2])),
            score=score, valid_coverage=valid_frac,
            mask_coverage=mask_frac, plane_coverage=label_frac,
            rms_residual=rms, p95_residual=p95, peak_to_valley=pv,
            normal_deviation_p95=dev_p95, max_adjacent_step=step,
            boundary_clearance=clearance,
            discontinuity_count=int(step_count),
            stamp=float(stamp), frame=str(frame_id),
            instance_id=str(instance_id), generation=int(generation),
            model_version=int(model.model_version),
            model_hash=str(model.identity_hash))


    @staticmethod
    def _max_adjacent_step(raw_heights, smooth_heights, valid, cell_size,
                           max_distance, step_gate):
        """Largest height discontinuity across boundaries (raw grid).

        A literal single-pair 4 mm gate on raw medians false-trips at
        sigma=2 mm pixel noise (~1.8 % per pair over thousands of
        pairs), while the 5x5 robust smooth spreads a 6 mm step over
        ~25 mm and hides it. The discontinuity is therefore the height
        difference across a boundary between 4-cell (20 mm) block MEANS
        of the RAW medians, for boundaries at 1-2 cell offsets (<= 10 mm
        apart): noise sigma drops ~2x below the gate while a hard step
        survives at full height. Returns ``(max_step_m, count)`` where
        the count covers block pairs beyond ``step_gate``.
        """
        block = 4
        violations = 0
        max_step = 0.0
        for axis in (0, 1):
            safe = np.where(valid, raw_heights, 0.0)
            csum = np.cumsum(safe, axis=axis)
            nsum = np.cumsum(valid.astype(np.float64), axis=axis)
            pad = [(0, 0), (0, 0)]
            pad[axis] = (1, 0)
            csum = np.pad(csum, pad, mode="constant")
            nsum = np.pad(nsum, pad, mode="constant")
            n = raw_heights.shape[axis]

            def block_mean(starts):
                """(…) mean over [k, k+block) per start index k."""
                cs = np.take(csum, np.clip(starts + block, 0, n),
                             axis=axis) - np.take(
                    csum, np.clip(starts, 0, n), axis=axis)
                ns = np.take(nsum, np.clip(starts + block, 0, n),
                             axis=axis) - np.take(
                    nsum, np.clip(starts, 0, n), axis=axis)
                with np.errstate(invalid="ignore", divide="ignore"):
                    return np.where(ns >= 2, cs / np.maximum(1.0, ns),
                                    np.nan)

            for off in (1, 2):
                if off * cell_size > max_distance + 1e-9:
                    continue
                # left block ends at k, right block starts at k + off
                ks = np.arange(1, n - block + 1)
                left = block_mean(ks - block)
                right = block_mean(ks + off)
                both = np.isfinite(left) & np.isfinite(right)
                if not both.any():
                    continue
                diff = np.abs(left - right)[both]
                violations += int((diff > step_gate).sum())
                if diff.size:
                    max_step = max(max_step, float(diff.max()))
        return max_step, violations

    @staticmethod
    def _max_bimodal_split(valid_h, min_fraction):
        """Largest sorted-gap split whose sides each keep >= min_fraction."""
        m = len(valid_h)
        if m < 4:
            return 0.0
        h = np.sort(valid_h)
        k_min = max(1, int(math.ceil(min_fraction * m)))
        gaps = h[k_min:m - k_min + 1] - h[k_min - 1:m - k_min]
        if len(gaps) == 0:
            return 0.0
        return float(gaps.max())


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
    """LSQ plane (unit normal +Z-ish, offset) or (None, None)."""
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
