#!/usr/bin/env python3
"""Layout atlas: evaluate robot-base Y positions for container coverage.

Pure-Python module (no ROS). Provides:
- effective_scene_tf: shift container Y (relative-motion equivalence).
- reliable_coverage_mask: REACHABLE + opening_connected.
- score_fixed_layout: multi-criteria scoring for a single Y position.
- greedy_set_cover: multi-stop theoretical union.
- build_union_artifact: union npz data.
- verify_grid_compatibility: cross-slice grid consistency.
- depth_lateral_layer_stats: regional coverage breakdown.

See docs/plans/layout_atlas_plan.md.
"""

from __future__ import division

import copy
import math

import numpy as np

# Status constants (must match reachability_atlas.py / builder).
UNKNOWN = np.uint8(0)
UNREACHABLE = np.uint8(1)
MARGINAL = np.uint8(2)
REACHABLE = np.uint8(3)


def model_prefix_from_robot_name(robot_name, default="s20"):
    """Derive an atlas filename/dir prefix from a robot_name.

    ``elfin_s30_with_camera`` -> ``s30``; ``elfin_s20_with_camera`` -> ``s20``.
    Used so S20 and S30 atlases/sweeps land in model-specific files/dirs
    (``s20_container_*.npz`` / ``s30_container_*.npz``,
    ``data/layout_atlas_s20/`` / ``data/layout_atlas_s30/``).
    """
    prefix = str(robot_name or "")
    if prefix.startswith("elfin_"):
        prefix = prefix[len("elfin_"):]
    if prefix.endswith("_with_camera"):
        prefix = prefix[: -len("_with_camera")]
    return prefix or default


# ── Effective scene generation ────────────────────────────────────────

def effective_scene_tf_xyz(base_config, dx, dy, dz):
    """Deep-copy scene_tf and shift container by the opposite of the base move.

    Relative-motion equivalence: base +(dx,dy,dz)  <=>  container -(dx,dy,dz).
    Modifies the ``container_link`` static_transform translation.

    Args:
        base_config: baseline scene_tf config dict (read-only, deep-copied).
        dx, dy, dz: robot base offset in world (meters) per axis.

    Returns:
        Modified config dict (independent copy).
    """
    cfg = copy.deepcopy(base_config)
    delta = (-float(dx), -float(dy), -float(dz))
    for tf in cfg.get("static_transforms", []):
        if tf.get("child") == "container_link":
            t = list(tf.get("translation", [0.0, 0.0, 0.0]))
            t[0] += delta[0]
            t[1] += delta[1]
            t[2] += delta[2]
            tf["translation"] = t
            break
    return cfg


def effective_scene_tf(base_config, base_y_offset):
    """Backward-compatible Y-only shift (delegates to :func:`effective_scene_tf_xyz`)."""
    return effective_scene_tf_xyz(base_config, 0.0, base_y_offset, 0.0)


def baseline_container_xyz(base_config):
    """Return the baseline container_link world translation as [x, y, z]."""
    for tf in base_config.get("static_transforms", []):
        if tf.get("child") == "container_link":
            return [float(v) for v in tf.get("translation", [0.0, 0.0, 0.0])]
    return [0.0, 0.0, 0.0]


def baseline_container_y(base_config):
    """Return the baseline container_link world Y translation."""
    return baseline_container_xyz(base_config)[1]


def slice_base_offset(slice_entry):
    """Return a (dx, dy, dz) offset for a slice, accepting either form.

    Newer 3-axis slices carry ``base_offset``; legacy Y-only slices carry
    ``base_y``. Missing axes default to 0.
    """
    offset = slice_entry.get("base_offset")
    if offset is not None:
        dx, dy, dz = (list(offset) + [0.0, 0.0, 0.0])[:3]
        return (float(dx), float(dy), float(dz))
    return (0.0, float(slice_entry.get("base_y", 0.0)), 0.0)


# ── Coverage ──────────────────────────────────────────────────────────

def reliable_coverage_mask(data):
    """Boolean mask of reliably-covered cells (REACHABLE + opening_connected).

    MARGINAL and UNKNOWN are NOT reliable.
    """
    status = data["status"]
    opening_connected = data["opening_connected"]
    return (status == REACHABLE) & opening_connected


# ── Grid compatibility ────────────────────────────────────────────────

def verify_grid_compatibility(meta_a, meta_b):
    """Check that two slices have the same grid definition.

    Returns (ok: bool, reason: str).
    """
    ga = meta_a.get("grid", {})
    gb = meta_b.get("grid", {})
    for key in ("resolution_xyz", "origin", "size", "yaw_bins"):
        if ga.get(key) != gb.get(key):
            return False, "grid %s mismatch: %s vs %s" % (
                key, ga.get(key), gb.get(key))
    return True, "ok"


# ── Fixed layout scoring ──────────────────────────────────────────────

def _opening_axes(opening_side):
    """Return (depth_axis, lateral_axis) from opening_side string."""
    if "x" in opening_side:
        return 0, 1  # depth=X, lateral=Y
    return 1, 0  # depth=Y, lateral=X


def depth_lateral_layer_stats(coverage_mask, meta):
    """Break down coverage by depth/lateral/layer regions.

    Returns dict with per-region coverage rates.
    """
    nx, ny, nz, nyaw = coverage_mask.shape
    opening_side = meta.get("container", {}).get("opening_side", "negative_x")
    depth_axis, lateral_axis = _opening_axes(opening_side)
    dims = [nx, ny, nz]
    nd = dims[depth_axis]      # depth cells
    nl = dims[lateral_axis]    # lateral cells
    nz_layers = nz

    # Collapse yaw (any yaw reachable).
    cov = np.any(coverage_mask, axis=-1)  # (nx, ny, nz)

    stats = {}
    # Depth bands: front 1/3, mid 1/3, back 1/3.
    for band, (lo, hi) in {
        "depth_front": (0, nd // 3),
        "depth_mid": (nd // 3, 2 * nd // 3),
        "depth_back": (2 * nd // 3, nd),
    }.items():
        sl = [slice(None)] * 3
        sl[depth_axis] = slice(lo, hi)
        region = cov[tuple(sl)]
        stats[band] = float(np.count_nonzero(region)) / max(1, region.size)

    # Lateral bands: left 1/3, center 1/3, right 1/3.
    for band, (lo, hi) in {
        "lateral_left": (0, nl // 3),
        "lateral_center": (nl // 3, 2 * nl // 3),
        "lateral_right": (2 * nl // 3, nl),
    }.items():
        sl = [slice(None)] * 3
        sl[lateral_axis] = slice(lo, hi)
        region = cov[tuple(sl)]
        stats[band] = float(np.count_nonzero(region)) / max(1, region.size)

    # Layers: bottom 1/3, mid 1/3, top 1/3.
    for band, (lo, hi) in {
        "layer_bottom": (0, nz_layers // 3),
        "layer_mid": (nz_layers // 3, 2 * nz_layers // 3),
        "layer_top": (2 * nz_layers // 3, nz_layers),
    }.items():
        region = cov[:, :, lo:hi]
        stats[band] = float(np.count_nonzero(region)) / max(1, region.size)

    return stats


def score_fixed_layout(data, meta):
    """Score a single fixed-Y layout.

    Returns dict with:
        has_opening_anchor: bool (gate)
        coverage_rate: float
        worst_region: float
        mean_joint_margin: float
        mean_neighbor_confidence: float
        score: float (0..1, higher better)
        reason: str
    """
    mask = reliable_coverage_mask(data)
    total = mask.size
    covered = int(np.count_nonzero(mask))

    # Opening anchor gate.
    has_anchor = covered > 0
    if not has_anchor:
        return {
            "has_opening_anchor": False,
            "coverage_rate": 0.0,
            "worst_region": 0.0,
            "mean_joint_margin": 0.0,
            "mean_neighbor_confidence": 0.0,
            "score": 0.0,
            "reason": "no_opening_connected_anchor",
        }

    coverage_rate = covered / max(1, total)

    # Regional stats.
    region_stats = depth_lateral_layer_stats(mask, meta)
    region_values = list(region_stats.values())
    worst_region = min(region_values) if region_values else 0.0

    # Joint margin (only for covered cells).
    joint_margin = data.get("joint_margin", np.zeros_like(mask, dtype=np.float32))
    covered_margins = joint_margin[mask]
    mean_margin = float(np.mean(covered_margins)) if covered_margins.size > 0 else 0.0

    # Neighbor confidence (only for covered cells).
    neighbor_conf = data.get("neighbor_confidence", np.ones_like(mask, dtype=np.float32))
    covered_conf = neighbor_conf[mask]
    mean_conf = float(np.mean(covered_conf)) if covered_conf.size > 0 else 0.0

    # Composite score (weighted).
    score = (
        0.40 * coverage_rate
        + 0.25 * worst_region
        + 0.15 * min(1.0, mean_margin / 0.5)  # normalize margin to 0..1
        + 0.15 * mean_conf
        + 0.05 * (1.0 if worst_region > 0.1 else 0.0)  # balance bonus
    )

    return {
        "has_opening_anchor": True,
        "coverage_rate": round(coverage_rate, 4),
        "worst_region": round(worst_region, 4),
        "mean_joint_margin": round(mean_margin, 4),
        "mean_neighbor_confidence": round(mean_conf, 4),
        "score": round(score, 4),
        "region_stats": region_stats,
        "reason": "ok",
    }


# ── Multi-stop set cover ──────────────────────────────────────────────

def greedy_set_cover(slices, target_coverage=0.95, max_stops=5):
    """Greedy set-cover across base-pose slices (X/Y/Z or legacy Y-only).

    Args:
        slices: list of dicts, each with:
            "mask": bool array (reliable coverage)
            "base_offset": (dx, dy, dz) -- preferred; or legacy "base_y": float
            "joint_margin": float array
            "neighbor_confidence": float array
            "meta": dict
        target_coverage: stop when this fraction of total cells is covered.
        max_stops: maximum number of stops.

    Returns:
        dict with:
            selected: list of selected slice indices
            union_mask: bool array
            preferred_stop_index: int array (per-cell, -1 if uncovered)
            preferred_base_xyz: float array (per-cell, [dx,dy,dz])
            preferred_base_y: float array (per-cell, back-compat = dy)
            coverage_count: int array (how many stops cover each cell)
            selected_offsets: list of (dx,dy,dz) for selected slices
            total_covered: int
            total_cells: int
            coverage_rate: float
            remaining_blind: int
    """
    if not slices:
        return _empty_set_cover_result()

    shape = slices[0]["mask"].shape
    total = int(np.prod(shape))
    union = np.zeros(shape, dtype=np.bool_)
    coverage_count = np.zeros(shape, dtype=np.uint8)
    preferred_stop = np.full(shape, -1, dtype=np.int16)
    preferred_xyz = np.zeros(shape + (3,), dtype=np.float32)

    selected = []
    selected_offsets = []
    remaining = list(range(len(slices)))

    while remaining and len(selected) < max_stops:
        # Find the slice that covers the most NEW cells.
        best_idx = None
        best_gain = 0
        for i in remaining:
            new_cells = int(np.count_nonzero(slices[i]["mask"] & ~union))
            if new_cells > best_gain:
                best_gain = new_cells
                best_idx = i

        if best_idx is None or best_gain == 0:
            break

        sel = slices[best_idx]
        offset = slice_base_offset(sel)
        new_mask = sel["mask"] & ~union

        # Update preferred stop for newly covered cells.
        for idx in np.argwhere(new_mask):
            if preferred_stop[tuple(idx)] == -1:
                preferred_stop[tuple(idx)] = len(selected)
                preferred_xyz[tuple(idx)] = offset

        union |= sel["mask"]
        coverage_count += sel["mask"].astype(np.uint8)
        selected.append(best_idx)
        selected_offsets.append(offset)
        remaining.remove(best_idx)

        if np.count_nonzero(union) / max(1, total) >= target_coverage:
            break

    total_covered = int(np.count_nonzero(union))
    return {
        "selected": selected,
        "selected_offsets": selected_offsets,
        "union_mask": union,
        "preferred_stop_index": preferred_stop,
        "preferred_base_xyz": preferred_xyz,
        "preferred_base_y": preferred_xyz[..., 1].copy(),
        "coverage_count": coverage_count,
        "total_covered": total_covered,
        "total_cells": total,
        "coverage_rate": round(total_covered / max(1, total), 4),
        "remaining_blind": total - total_covered,
    }


def _empty_set_cover_result():
    return {
        "selected": [],
        "selected_offsets": [],
        "union_mask": None,
        "preferred_stop_index": None,
        "preferred_base_xyz": None,
        "preferred_base_y": None,
        "coverage_count": None,
        "total_covered": 0,
        "total_cells": 0,
        "coverage_rate": 0.0,
        "remaining_blind": 0,
    }


# ── Union artifact ────────────────────────────────────────────────────

def build_union_artifact(set_cover_result, slices):
    """Build the union NPZ data from a set-cover result.

    Returns dict of numpy arrays suitable for np.savez_compressed.
    """
    sc = set_cover_result
    if sc["union_mask"] is None:
        return {}

    # Build union status: REACHABLE if covered, UNREACHABLE otherwise.
    union_status = np.where(sc["union_mask"], REACHABLE, UNREACHABLE).astype(np.uint8)
    # Union opening_connected = any slice's opening_connected.
    union_oc = np.zeros_like(sc["union_mask"])
    for i in sc["selected"]:
        union_oc |= slices[i].get("opening_connected", slices[i]["mask"])

    return {
        "union_status": union_status,
        "union_opening_connected": union_oc,
        "preferred_stop_index": sc["preferred_stop_index"],
        "preferred_base_y": sc["preferred_base_y"],
        "preferred_base_xyz": sc["preferred_base_xyz"],
        "coverage_count": sc["coverage_count"],
    }


# ── Base movement envelope ────────────────────────────────────────────

def base_movement_envelope(offsets):
    """Per-axis min/max over a list of contributing base offsets.

    Args:
        offsets: iterable of (dx, dy, dz) tuples for stops that contribute
            at least one new cell to the reliable union.

    Returns:
        dict {x, y, z} -> {"min", "max", "count"} (min/max None when empty).
    """
    axes = ("x", "y", "z")
    empty = {ax: {"min": None, "max": None, "count": 0} for ax in axes}
    rows = []
    for off in offsets or []:
        dx, dy, dz = (list(off) + [0.0, 0.0, 0.0])[:3]
        rows.append([float(dx), float(dy), float(dz)])
    if not rows:
        return empty
    arr = np.array(rows, dtype=np.float64)
    result = {}
    for i, ax in enumerate(axes):
        result[ax] = {
            "min": float(arr[:, i].min()),
            "max": float(arr[:, i].max()),
            "count": int(arr.shape[0]),
        }
    return result


# ── Decision ──────────────────────────────────────────────────────────

def evaluate_decision(baseline_score, best_fixed_score, union_result,
                      kinematic_scores=None, multi_axis=False):
    """Determine the recommendation.

    With ``multi_axis=True`` (X/Y/Z sweep), a significant union gain is
    reported as ``multi_axis_promising`` instead of ``multi_stop_promising``.

    Returns:
        dict with "recommendation" and "reason".
    """
    if not best_fixed_score.get("has_opening_anchor"):
        rec = "y_axis_insufficient" if not multi_axis else "multi_axis_insufficient"
        return {
            "recommendation": rec,
            "reason": "no layout has opening-connected anchor",
        }

    baseline_rate = baseline_score.get("coverage_rate", 0.0)
    best_rate = best_fixed_score.get("coverage_rate", 0.0)
    union_rate = union_result.get("coverage_rate", 0.0)

    # Best fixed barely better than baseline.
    if best_rate <= baseline_rate * 1.05:
        rec = "y_axis_insufficient" if not multi_axis else "multi_axis_insufficient"
        return {
            "recommendation": rec,
            "reason": "best fixed (%.1f%%) not significantly better than baseline (%.1f%%)"
            % (best_rate * 100, baseline_rate * 100),
        }

    # Union significantly reduces blind spots.
    if union_rate > best_rate * 1.15:
        rec = "multi_axis_promising" if multi_axis else "multi_stop_promising"
        return {
            "recommendation": rec,
            "reason": "union (%.1f%%) significantly > best fixed (%.1f%%)"
            % (union_rate * 100, best_rate * 100),
        }

    return {
        "recommendation": "fixed",
        "reason": "best fixed (%.1f%%) is sufficient; union gain modest (%.1f%%)"
        % (best_rate * 100, union_rate * 100),
    }
