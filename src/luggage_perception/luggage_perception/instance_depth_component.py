#!/usr/bin/env python3
"""Instance depth-component isolation from an aligned depth image (no ROS).

DYNAMIC-SUCTION ST-1 design decision A: the hardware top-surface pipeline
starts from the YOLO instance ROI, not from a scene_tf window. For the
``bbox_fill`` backend the ROI is a rectangle that may contain background
(platform, distractors); the box surface is isolated as one depth component
inside the box before any geometry is produced:

- erode the box by ``erode_bbox_px`` before choosing seeds;
- seed depth = median valid depth in the central ``central_fraction`` of the
  eroded box;
- retain the 8-connected component reached from the seed pixels through
  neighbours whose depth differs from the seed median by at most
  ``median_band_mm`` and whose adjacent depth jump is at most
  ``adjacent_jump_mm`` (the jump threshold is frozen at the seed median, so
  the reached set is a graph-reachability closure and therefore
  order-independent and deterministic);
- reject when retained coverage is below ``min_coverage_frac`` of the
  non-eroded region or fewer than ``min_pixels_at_vga`` valid pixels remain
  at 640x480 (pixel counts scale by image area at other resolutions).

A true pixel instance mask (SAM2-class backend) is preferred when present:
it is eroded by ``erode_mask_px`` for fitting while the caller keeps the
original mask for boundary and coverage scoring.

Every rejection carries a machine-readable reason (fail-closed). The module
is numpy/stdlib only, performs no I/O, and imports nothing from ROS.
"""

from __future__ import division

import math
from dataclasses import dataclass, field

import numpy as np

COMPONENT_OK = "ok"
COMPONENT_NO_REGION = "COMPONENT_NO_REGION"
COMPONENT_NO_SEED_DEPTH = "COMPONENT_NO_SEED_DEPTH"
COMPONENT_COVERAGE_LOW = "COMPONENT_COVERAGE_LOW"
COMPONENT_TOO_FEW_PIXELS = "COMPONENT_TOO_FEW_PIXELS"
COMPONENT_BOUNDARY_CONTINUOUS = "COMPONENT_BOUNDARY_CONTINUOUS"

VGA_PIXELS = 640 * 480

# 8-neighbourhood offsets (dr, dc).
_NEIGHBOURS8 = (
    (-1, -1), (-1, 0), (-1, 1),
    (0, -1), (0, 1),
    (1, -1), (1, 0), (1, 1),
)


@dataclass
class ComponentConfig:
    """Plan-fixed isolation thresholds (DYNAMIC-SUCTION plan section A)."""

    erode_bbox_px: int = 5
    erode_mask_px: int = 3
    central_fraction: float = 0.30
    median_band_mm: float = 30.0
    adjacent_jump_mm: float = 20.0
    min_coverage_frac: float = 0.35
    min_pixels_at_vga: int = 200
    # A luggage top must step away from its surroundings: at least one
    # region-boundary side must show a depth step larger than the band. The
    # gate rejects platform/background-only ROIs whose surface continues
    # seamlessly outside the ROI (an instance is separable from its
    # background; a scene surface is not). 0 disables the gate.
    boundary_step_mm: float = 30.0
    boundary_strip_px: int = 2


@dataclass
class ComponentResult:
    """One isolation attempt; ``mask`` is valid only when reason is ok."""

    mask: np.ndarray            # (H, W) bool, all-False on rejection
    valid_pixels: int
    region_pixels: int          # non-eroded reference area for coverage
    coverage: float
    seed_depth_mm: float
    reason: str
    diagnostics: dict = field(default_factory=dict)

    @property
    def ok(self):
        return self.reason == COMPONENT_OK


def erode_mask(mask, px):
    """Binary erosion of a 2-D bool mask by ``px`` pixels (numpy shifts).

    Deterministic, edge-clipped: pixels within ``px`` of the image border or
    of a False region are removed. ``px <= 0`` returns the mask unchanged.
    """
    arr = np.asarray(mask, dtype=bool)
    px = int(px)
    if px <= 0 or arr.size == 0:
        return arr.copy()
    # Chebyshev (square) erosion: repeat a 1-pixel 8-neighbour AND ``px``
    # times; equivalent to a (2*px+1) square structuring element.
    out = arr.copy()
    for _ in range(px):
        shifted = out.copy()
        for dr, dc in _NEIGHBOURS8:
            r0, r1 = max(0, -dr), out.shape[0] - max(0, dr)
            c0, c1 = max(0, -dc), out.shape[1] - max(0, dc)
            neigh = np.zeros_like(out)
            neigh[r0:r1, c0:c1] = out[r0 + dr:r1 + dr, c0 + dc:c1 + dc]
            shifted &= neigh
        out = shifted
    return out


def _clip_bbox(bbox, shape):
    """Clip an (x0, y0, x1, y1) pixel bbox to the image; end-exclusive."""
    h, w = int(shape[0]), int(shape[1])
    x0, y0, x1, y1 = (int(v) for v in bbox)
    x0 = max(0, min(x0, w))
    x1 = max(x0 + 1, min(x1, w))
    y0 = max(0, min(y0, h))
    y1 = max(y0 + 1, min(y1, h))
    return x0, y0, x1, y1


def _region_windows(shape, bbox, erode_px):
    """Non-eroded and eroded bbox windows for a bbox region."""
    x0, y0, x1, y1 = _clip_bbox(bbox, shape)
    ex0 = min(x0 + erode_px, x1 - 1)
    ex1 = max(x1 - erode_px, ex0 + 1)
    ey0 = min(y0 + erode_px, y1 - 1)
    ey1 = max(y1 - erode_px, ey0 + 1)
    return (x0, y0, x1, y1), (ex0, ey0, ex1, ey1)


def _valid_depth(depth_mm):
    depth = np.asarray(depth_mm)
    return np.isfinite(depth) & (depth > 0)


def _central_window(x0, y0, x1, y1, fraction):
    """Central sub-rectangle keeping ``fraction`` of each side length."""
    fw = max(1, int(round((x1 - x0) * fraction)))
    fh = max(1, int(round((y1 - y0) * fraction)))
    cx0 = x0 + max(0, ((x1 - x0) - fw) // 2)
    cy0 = y0 + max(0, ((y1 - y0) - fh) // 2)
    return cx0, cy0, min(cx0 + fw, x1), min(cy0 + fh, y1)


def _flood_from_seeds(eligible, jump_ok_to, seeds):
    """Multi-source reachability over 8-neighbours (vectorized dilation).

    ``eligible``: (H, W) bool band mask. ``jump_ok_to``: dict keyed by
    neighbour offset -> (H, W) bool "moving from that neighbour into this
    pixel satisfies the adjacent-jump rule". ``seeds``: (H, W) bool.
    Returns the reached closure (bool array).
    """
    reached = seeds & eligible
    if not reached.any():
        return reached
    h, w = reached.shape
    while True:
        grew = False
        for dr, dc in _NEIGHBOURS8:
            # candidate pixels: reached shifted by (dr, dc)
            cand = np.zeros_like(reached)
            r0, r1 = max(0, -dr), h - max(0, dr)
            c0, c1 = max(0, -dc), w - max(0, dc)
            cand[r0:r1, c0:c1] = reached[
                r0 + dr:r1 + dr, c0 + dc:c1 + dc]
            cand &= eligible & ~reached & jump_ok_to[(dr, dc)]
            if cand.any():
                reached |= cand
                grew = True
        if not grew:
            return reached


def _boundary_step_ok(depth, valid, component, x0, y0, x1, y1, cfg):
    """Instance-separability gate: the region must step off its background.

    For each side of the component's bounding rectangle, compare the median
    depth of *component* pixels just inside the boundary with the median
    valid depth just outside it. At least one side must show a step larger
    than ``boundary_step_mm``; a surface that continues seamlessly outside
    on every side is a scene surface (platform/background), not an
    instance. Sampling inside via the component mask (not the rectangle)
    keeps rotated tops working: the rectangle edge touches them only at
    the corners. Returns ``(ok, diagnostics)``; ``ok=True`` when the gate
    is disabled or no side has comparable samples (nothing to prove).
    """
    if cfg.boundary_step_mm <= 0.0:
        return True, {"boundary_gate": "disabled"}
    depth = np.asarray(depth)
    strip = max(1, int(cfg.boundary_strip_px))
    h, w = depth.shape
    near = {}
    for edge in ("left", "right", "top", "bottom"):
        band = np.zeros((h, w), dtype=bool)
        if edge == "left":
            band[y0:y1, x0:min(w, x0 + strip)] = True
        elif edge == "right":
            band[y0:y1, max(0, x1 - strip):x1] = True
        elif edge == "top":
            band[y0:min(h, y0 + strip), x0:x1] = True
        else:
            band[max(0, y1 - strip):y1, x0:x1] = True
        near[edge] = band
    outside = {
        "left": (slice(y0, y1), slice(max(0, x0 - strip), x0)),
        "right": (slice(y0, y1), slice(x1, min(w, x1 + strip))),
        "top": (slice(max(0, y0 - strip), y0), slice(x0, x1)),
        "bottom": (slice(y1, min(h, y1 + strip)), slice(x0, x1)),
    }
    steps = {}
    for edge in ("left", "right", "top", "bottom"):
        inside_px = component & near[edge]
        or_slice, oc_slice = outside[edge]
        outside_px = np.zeros((h, w), dtype=bool)
        outside_px[or_slice, oc_slice] = valid[or_slice, oc_slice]
        if int(inside_px.sum()) < 2 or int(outside_px.sum()) < 2:
            steps[edge] = None
            continue
        steps[edge] = float(
            np.median(depth[inside_px]) - np.median(depth[outside_px]))
    comparable = [abs(v) for v in steps.values() if v is not None]
    if not comparable:
        return True, {"boundary_gate": "no_samples", "steps": steps}
    ok = max(comparable) > float(cfg.boundary_step_mm)
    return ok, {"boundary_gate": "ok" if ok else "continuous",
                "steps": steps}


def isolate_depth_component(depth_mm, bbox=None, mask=None, config=None):
    """Isolate the instance surface inside a bbox or pixel mask.

    Args:
        depth_mm: (H, W) millimetre depth (uint16-family or float view).
        bbox: (x0, y0, x1, y1) end-exclusive pixel box (bbox_fill path).
        mask: (H, W) bool pixel instance mask (preferred when provided).
        config: :class:`ComponentConfig`.

    Returns: :class:`ComponentResult`. The component mask is the retained
    surface; every rejection path returns an all-False mask plus its named
    reason (fail-closed).
    """
    cfg = config or ComponentConfig()
    depth = np.asarray(depth_mm)
    if depth.ndim != 2:
        raise ValueError("depth_mm must be a 2-D (H, W) array")
    valid = _valid_depth(depth)
    shape = depth.shape
    empty = np.zeros(shape, dtype=bool)

    def _reject(reason, diag=None):
        return ComponentResult(
            mask=empty, valid_pixels=0, region_pixels=0, coverage=0.0,
            seed_depth_mm=0.0, reason=reason,
            diagnostics=dict(diag or {}))

    if mask is not None:
        instance = np.asarray(mask, dtype=bool)
        if instance.shape != shape:
            return _reject(COMPONENT_NO_REGION,
                           {"mask_shape": list(instance.shape)})
        region = instance
        fitted = erode_mask(region, cfg.erode_mask_px)
        region_pixels = int(region.sum())
        if region_pixels <= 0 or not fitted.any():
            return _reject(COMPONENT_NO_REGION,
                           {"region_pixels": region_pixels})
        # A pixel mask bounds the instance, but loose masks can still carry
        # background: keep the depth band around the eroded-mask median
        # (same coherence rule as the bbox path), then the largest
        # 8-connected valid component.
        mask_valid = fitted & valid
        if not mask_valid.any():
            return _reject(COMPONENT_NO_SEED_DEPTH,
                           {"region_pixels": region_pixels})
        seed_depth = float(np.median(depth[mask_valid]))
        band = mask_valid & (
            np.abs(depth.astype(np.float64) - seed_depth)
            <= float(cfg.median_band_mm))
        labels, count = label_components_8(band)
        if count == 0:
            return _reject(COMPONENT_NO_SEED_DEPTH,
                           {"region_pixels": region_pixels})
        sizes = np.bincount(labels.ravel())
        sizes[0] = 0
        best = int(np.argmax(sizes))
        component = labels == best
        cols = component.any(axis=0)
        rows = component.any(axis=1)
        x0 = int(np.argmax(cols))
        x1 = int(len(cols) - np.argmax(cols[::-1]))
        y0 = int(np.argmax(rows))
        y1 = int(len(rows) - np.argmax(rows[::-1]))
        n_component = int(component.sum())
        min_pixels = _min_pixels(shape, cfg)
        coverage = n_component / float(region_pixels)
        diag = {"region_pixels": region_pixels,
                "valid_pixels": n_component,
                "min_pixels": min_pixels, "coverage": coverage,
                "seed_depth_mm": seed_depth,
                "path": "mask"}
        if n_component < min_pixels:
            return _reject(COMPONENT_TOO_FEW_PIXELS, diag)
        if coverage < float(cfg.min_coverage_frac):
            return _reject(COMPONENT_COVERAGE_LOW, diag)
        ok, boundary_diag = _boundary_step_ok(
            depth, valid, component, x0, y0, x1, y1, cfg)
        diag.update(boundary_diag)
        if not ok:
            return _reject(COMPONENT_BOUNDARY_CONTINUOUS, diag)
        return ComponentResult(
            mask=component, valid_pixels=n_component,
            region_pixels=region_pixels, coverage=float(coverage),
            seed_depth_mm=seed_depth, reason=COMPONENT_OK,
            diagnostics=diag)

    if bbox is None:
        return _reject(COMPONENT_NO_REGION, {"bbox": None})
    (x0, y0, x1, y1), (ex0, ey0, ex1, ey1) = _region_windows(
        shape, bbox, cfg.erode_bbox_px)
    region_pixels = int((x1 - x0) * (y1 - y0))
    if region_pixels <= 0:
        return _reject(COMPONENT_NO_REGION, {"region_pixels": region_pixels})

    eroded_valid = valid[ey0:ey1, ex0:ex1]
    if not eroded_valid.any():
        return _reject(COMPONENT_NO_SEED_DEPTH,
                       {"region_pixels": region_pixels})

    # Seed: median valid depth in the central fraction of the eroded box.
    cx0, cy0, cx1, cy1 = _central_window(
        ex0, ey0, ex1, ey1, cfg.central_fraction)
    central_valid = valid[cy0:cy1, cx0:cx1]
    if not central_valid.any():
        return _reject(COMPONENT_NO_SEED_DEPTH,
                       {"region_pixels": region_pixels})
    seed_depth = float(np.median(depth[cy0:cy1, cx0:cx1][central_valid]))

    # Band mask inside the eroded region; reachability from the central
    # seed pixels through adjacent jumps <= adjacent_jump_mm.
    eroded_depth = depth[ey0:ey1, ex0:ex1].astype(np.float64)
    band = eroded_valid & (
        np.abs(eroded_depth - seed_depth) <= float(cfg.median_band_mm))
    jump = float(cfg.adjacent_jump_mm)
    jump_ok_to = {}
    for dr, dc in _NEIGHBOURS8:
        r0, r1 = max(0, -dr), band.shape[0] - max(0, dr)
        c0, c1 = max(0, -dc), band.shape[1] - max(0, dc)
        src = np.full(band.shape, np.inf)
        src[r0:r1, c0:c1] = eroded_depth[r0 + dr:r1 + dr, c0 + dc:c1 + dc]
        jump_ok_to[(dr, dc)] = np.abs(eroded_depth - src) <= jump
    seeds = np.zeros(band.shape, dtype=bool)
    seeds[cy0 - ey0:cy1 - ey0, cx0 - ex0:cx1 - ex0] = band[
        cy0 - ey0:cy1 - ey0, cx0 - ex0:cx1 - ex0]
    component_window = _flood_from_seeds(band, jump_ok_to, seeds)
    n_component = int(component_window.sum())
    min_pixels = _min_pixels(shape, cfg)
    coverage = n_component / float(region_pixels)
    diag = {"region_pixels": region_pixels,
            "seed_depth_mm": seed_depth,
            "valid_pixels": n_component,
            "min_pixels": min_pixels,
            "coverage": coverage,
            "path": "bbox"}
    if not component_window.any():
        return _reject(COMPONENT_NO_SEED_DEPTH, diag)
    if n_component < min_pixels:
        return _reject(COMPONENT_TOO_FEW_PIXELS, diag)
    if coverage < float(cfg.min_coverage_frac):
        return _reject(COMPONENT_COVERAGE_LOW, diag)

    component = np.zeros(shape, dtype=bool)
    component[ey0:ey1, ex0:ex1] = component_window

    # Boundary gate at the component's own bounding rectangle: an instance
    # top must step off its surroundings; a platform/background surface
    # continues seamlessly across the boundary on every side.
    wcols = component_window.any(axis=0)
    wrows = component_window.any(axis=1)
    cx0 = int(np.argmax(wcols)) + ex0
    cx1 = int(len(wcols) - np.argmax(wcols[::-1])) + ex0
    cy0 = int(np.argmax(wrows)) + ey0
    cy1 = int(len(wrows) - np.argmax(wrows[::-1])) + ey0
    ok, boundary_diag = _boundary_step_ok(
        depth, valid, component, cx0, cy0, cx1, cy1, cfg)
    diag.update(boundary_diag)
    if not ok:
        return _reject(COMPONENT_BOUNDARY_CONTINUOUS, diag)

    return ComponentResult(
        mask=component, valid_pixels=n_component,
        region_pixels=region_pixels, coverage=float(coverage),
        seed_depth_mm=seed_depth, reason=COMPONENT_OK, diagnostics=diag)


def _min_pixels(shape, cfg):
    """Area-scaled minimum pixel count (200 at 640x480)."""
    scale = (float(shape[0]) * float(shape[1])) / float(VGA_PIXELS)
    return max(1, int(math.ceil(cfg.min_pixels_at_vga * scale)))


def label_components_8(mask):
    """Label 8-connected components via row-run union-find (no scipy).

    Returns ``(labels, count)`` with background 0 and components 1..count in
    deterministic first-encountered order. Deterministic for a given input.
    """
    arr = np.asarray(mask, dtype=bool)
    if arr.size == 0:
        return np.zeros(arr.shape, dtype=np.int32), 0
    h, w = arr.shape
    parent = [0]  # index 0 unused; find() with path compression

    def find(a):
        root = a
        while parent[root] != root:
            root = parent[root]
        while parent[a] != root:
            parent[a], a = root, parent[a]
        return root

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            if ra < rb:
                parent[rb] = ra
            else:
                parent[ra] = rb

    labels = np.zeros(arr.shape, dtype=np.int32)
    prev_runs = []  # (start_col, end_col, label)
    next_label = 1
    for r in range(h):
        row = arr[r]
        if not row.any():
            prev_runs = []
            continue
        padded = np.concatenate(([False], row, [False]))
        edges = np.flatnonzero(padded[1:] != padded[:-1])
        runs = list(zip(edges[0::2], edges[1::2]))  # end-exclusive
        cur_runs = []
        for start, end in runs:
            label = next_label
            next_label += 1
            parent.append(label)
            # Union with previous-row runs overlapping [start-1, end+1).
            for ps, pe, plabel in prev_runs:
                if ps <= end and pe >= start:
                    union(label, plabel)
            cur_runs.append((start, end, label))
            labels[r, start:end] = label
        prev_runs = cur_runs

    if next_label == 1:
        return labels, 0
    # Compact root labels in first-encountered order.
    flat = labels.ravel()
    roots = np.empty(next_label, dtype=np.int32)
    for lbl in range(1, next_label):
        roots[lbl] = find(lbl)
    root_map = {}
    flat_out = np.zeros_like(flat)
    count = 0
    for value in np.unique(flat):
        if value == 0:
            continue
        root = int(roots[value])
        if root not in root_map:
            count += 1
            root_map[root] = count
        flat_out[flat == value] = root_map[root]
    return flat_out.reshape(arr.shape), count


def largest_component_mask(mask):
    """Bool mask of the largest 8-connected component (ties -> first)."""
    labels, count = label_components_8(mask)
    if count == 0:
        return np.zeros(np.asarray(mask).shape, dtype=bool)
    sizes = np.bincount(labels.ravel(), minlength=count + 1)
    sizes[0] = 0
    return labels == int(np.argmax(sizes))
