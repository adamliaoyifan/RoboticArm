#!/usr/bin/env python3
"""Rule-based placement candidate generation on a 2.5D surface map.

Pure-Python (no ROS) so it can be unit tested directly. The solver consumes the
``surface_map_2d`` contract published by ``cargo_volume_mapper`` and, for a given
box size, slides an axis-aligned footprint over the container-local height map to
produce scored candidate center poses.

First-version rules (matching the MVP plan):
  - box is placed horizontally (no tilt)
  - yaw restricted to the requested set, snapped to 0 / 90 deg footprints
  - z = max(height under footprint) + box_height / 2
  - unobserved columns are allowed only when the box lands on the a-priori
    configured floor (peak ≈ floor_z, "floor_prior"); stacking on an unobserved surface
    (peak > 0) is rejected as "unknown_above_floor" (see design §4.2.2)
  - top clearance must exceed box height margin
"""

from __future__ import division

import math


def _default_params():
    return {
        "clearance_margin": 0.03,   # required free space above the placed box [m]
        "support_tol": 0.05,        # cells within this of the peak count as support [m]
        "min_support_ratio": 0.6,   # required contact ratio of the footprint
        "stride_cells": 1,          # grid stride for the sliding window
        "w_support": 0.5,
        "w_clearance": 0.3,
        "w_compactness": 0.2,       # prefer low placements (good stacking)
        "w_confidence": 0.15,       # penalizes geometry-only or unknown support
        "top_n": 8,
        "keep_rejected": 6,         # rejected candidates kept for visualization
    }


def _snap_yaws(allowed_yaws):
    """Snap requested yaws to {0, pi/2} footprint orientations.

    The grid is axis-aligned in the container-local frame, so only 0 / 90 deg
    footprints stay axis-aligned. Other yaws collapse onto the nearest of these
    two for footprint sizing while preserving the requested world yaw.
    """
    snapped = []
    seen = set()
    for yaw in allowed_yaws or [0.0]:
        # Reduce to [0, pi) since a box footprint is symmetric under pi.
        reduced = yaw % math.pi
        is_rotated = abs(reduced - math.pi / 2.0) < abs(reduced - 0.0)
        key = 1 if is_rotated else 0
        if key in seen:
            continue
        seen.add(key)
        snapped.append((yaw, is_rotated))
    return snapped


def _footprint_cells(footprint_l, footprint_w, resolution):
    cells_x = max(1, int(math.ceil(footprint_l / resolution - 1e-9)))
    cells_y = max(1, int(math.ceil(footprint_w / resolution - 1e-9)))
    return cells_x, cells_y


def _window_stats(surface_map, ix0, iy0, cells_x, cells_y):
    heights = surface_map["height"]
    states = surface_map["state"]
    confidences = surface_map.get("confidence")
    max_h = 0.0
    has_unknown = False
    cell_heights = []
    sensor_cells = 0
    total_cells = 0
    for ix in range(ix0, ix0 + cells_x):
        for iy in range(iy0, iy0 + cells_y):
            total_cells += 1
            if states[ix][iy] == "unknown":
                has_unknown = True
            h = heights[ix][iy]
            cell_heights.append(h)
            if h > max_h:
                max_h = h
            if confidences and confidences[ix][iy] == "sensor":
                sensor_cells += 1
    confidence_ratio = float(sensor_cells) / float(total_cells) if total_cells else 0.0
    return max_h, has_unknown, cell_heights, confidence_ratio


def _support_ratio(cell_heights, peak, tol):
    if not cell_heights:
        return 0.0
    contact = sum(1 for h in cell_heights if h >= peak - tol)
    return float(contact) / float(len(cell_heights))


def _local_to_base(center_base, yaw, lx, ly, lz):
    bx = center_base[0] + math.cos(yaw) * lx - math.sin(yaw) * ly
    by = center_base[1] + math.sin(yaw) * lx + math.cos(yaw) * ly
    bz = center_base[2] + lz
    return [bx, by, bz]


def generate_candidates(surface_map, box_size, allowed_yaws=None, params=None):
    """Return scored placement candidates sorted feasible-first by score.

    ``surface_map`` is the ``surface_map_2d`` dict. ``box_size`` is
    ``[length, width, height]`` in meters.
    """
    cfg = _default_params()
    if params:
        cfg.update(params)

    box_l, box_w, box_h = [float(v) for v in box_size]
    resolution = float(surface_map["resolution"])
    nx = int(surface_map["nx"])
    ny = int(surface_map["ny"])
    inner_l, inner_w, inner_h = [float(v) for v in surface_map["inner_size"]]
    floor_z = float(surface_map.get("floor_z", 0.0))
    center_base = surface_map["center_base"]
    map_yaw = float(surface_map["yaw"])
    half_l = inner_l * 0.5
    half_w = inner_w * 0.5
    half_h = inner_h * 0.5
    stride = max(1, int(cfg["stride_cells"]))

    candidates = []
    for box_yaw, rotated in _snap_yaws(allowed_yaws):
        footprint_l, footprint_w = (box_w, box_l) if rotated else (box_l, box_w)
        if footprint_l > inner_l + 1e-6 or footprint_w > inner_w + 1e-6:
            continue
        cells_x, cells_y = _footprint_cells(footprint_l, footprint_w, resolution)
        if cells_x > nx or cells_y > ny:
            continue

        for ix0 in range(0, nx - cells_x + 1, stride):
            for iy0 in range(0, ny - cells_y + 1, stride):
                peak, has_unknown, cell_heights, confidence_ratio = _window_stats(
                    surface_map, ix0, iy0, cells_x, cells_y
                )
                support_ratio = _support_ratio(cell_heights, peak, cfg["support_tol"])
                top_of_box = peak + box_h
                clearance_top = inner_h - top_of_box

                lx = -half_l + (ix0 + cells_x * 0.5) * resolution
                ly = -half_w + (iy0 + cells_y * 0.5) * resolution
                lz = -half_h + peak + box_h * 0.5
                center_base_xyz = _local_to_base(center_base, map_yaw, lx, ly, lz)

                feasible = True
                reason = "ok"
                support_source = "sensor"
                if has_unknown:
                    if abs(peak - floor_z) > cfg["support_tol"]:
                        # Stacking on an unobserved support surface -> reject.
                        # "floor exists" is geometric prior, but "something
                        # unseen holds the box at peak>0" is not trustworthy.
                        feasible = False
                        reason = "unknown_above_floor"
                    else:
                        # peak ≈ floor_z: lands on the a-priori container floor.
                        # The floor's *existence* is geometric prior (scene_tf),
                        # not a perception claim, so allow even though the
                        # column is unobserved. confidence_ratio == 0 makes
                        # these score below observed positions. See §4.2.2.
                        support_source = "floor_prior"
                if feasible and clearance_top < cfg["clearance_margin"]:
                    feasible = False
                    reason = "insufficient_clearance"
                elif feasible and support_ratio < cfg["min_support_ratio"]:
                    feasible = False
                    reason = "insufficient_support"

                clearance_score = max(0.0, min(1.0, clearance_top / max(box_h, 1e-6)))
                compactness = 1.0 - min(1.0, peak / max(inner_h, 1e-6))
                w_conf = float(cfg.get("w_confidence", 0.0))
                score = (
                    cfg["w_support"] * support_ratio
                    + cfg["w_clearance"] * clearance_score
                    + cfg["w_compactness"] * compactness
                    + w_conf * confidence_ratio
                )

                candidates.append({
                    "center_base": center_base_xyz,
                    "center_local": [lx, ly, lz],
                    "yaw": map_yaw + box_yaw,
                    "box_yaw": box_yaw,
                    "footprint": [footprint_l, footprint_w],
                    "size": [box_l, box_w, box_h],
                    "support_score": round(support_ratio, 4),
                    "clearance_score": round(clearance_score, 4),
                    "clearance_top": round(clearance_top, 4),
                    "collision_margin": round(clearance_top, 4),
                    "confidence_ratio": round(confidence_ratio, 4),
                    "support_source": support_source,
                    "reachability_score": -1.0,
                    "score": round(score, 4),
                    "feasible": feasible,
                    "reason": reason,
                })

    candidates.sort(key=lambda c: (not c["feasible"], -c["score"]))
    feasible = [c for c in candidates if c["feasible"]][: int(cfg["top_n"])]
    rejected = [c for c in candidates if not c["feasible"]][: int(cfg["keep_rejected"])]
    return feasible + rejected


def best_candidate(candidates):
    for cand in candidates:
        if cand["feasible"]:
            return cand
    return None


if __name__ == "__main__":
    # Minimal self-test on a synthetic empty container.
    sm = {
        "resolution": 0.1,
        "nx": 20,
        "ny": 16,
        "inner_size": [2.0, 1.6, 1.5],
        "center_base": [2.0, 0.0, 0.75],
        "yaw": 0.0,
        "height": [[0.0] * 16 for _ in range(20)],
        "state": [["free"] * 16 for _ in range(20)],
        "clearance": [[1.5] * 16 for _ in range(20)],
        "known_ratio": [[1.0] * 16 for _ in range(20)],
        "confidence": [["sensor"] * 16 for _ in range(20)],
    }
    cands = generate_candidates(sm, [0.7, 0.45, 0.28], allowed_yaws=[0.0, math.pi / 2.0])
    print("candidates:", len(cands))
    top = best_candidate(cands)
    print("best:", top["center_base"], "score", top["score"], "reason", top["reason"])
