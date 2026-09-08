#!/usr/bin/env python3
"""Deterministic synthetic fixtures for PF-R6-RANSAC-RESEARCH.

World frame: support plane at z=0 (GT), box top at z=h with the top
rectangle centered at origin, yawed. The raw cloud mirrors what
``estimate_local_support`` sees in production: box top + side points,
visible support-annulus points, a floor far below the plausible band,
optional wrong-height coherent clutter, sparse flyers, and non-finite
rows. Every estimator receives only this raw cloud + the top estimate +
the non-privileged workspace/band config — synthetic GT is eval-only.

Fixture families (frozen manifest before final measurement):
- positive: flat_clean, quantized_noise, asymmetric_sectors,
  missing_side, competing_lower_plane, wrong_height_clutter_partial,
  isolated_outliers, lower_floor_excluded  -> expect valid support at z~0
- negative: insufficient_points, nonfinite_starved, all_nonfinite,
  flyers_only, clutter_single_side -> expect reject / no valid height

Candidate counts are tuned into the measured PF-R6 range (20k-35k).
"""

from __future__ import division

import hashlib
import json
import math

import numpy as np

from luggage_perception.top_support_estimator import TopSurfaceEstimate

# Catalog-ish sizes (heights inside the committed plausible band
# [0.15, 0.60] m). Footprints only matter for the annulus geometry.
BOX_SIZES = {
    "carryon": (0.36, 0.23, 0.50),
    "standard": (0.45, 0.32, 0.58),
    "large": (0.60, 0.40, 0.55),
}

WORKSPACE = ((0.0, 0.0), (0.5, 0.5))

MASTER_SEED = 20260905


def _quantize_noise(rng, n, sigma):
    """Gaussian z noise quantized to 1 mm (depth-quantization stand-in)."""
    return np.round(rng.normal(0.0, sigma, n) / 0.001) * 0.001


def _annulus_points(rng, w, d, count, sectors, z=0.0, sigma=0.002,
                    inner_margin=0.03, outer_margin=0.18):
    """Uniform points in the rect annulus restricted to angular sectors.

    ``sectors`` is a list of (start_rad, end_rad) windows measured from the
    box +u axis; ``count`` points are returned (rejection sampling with
    analytic oversampling), each carrying quantized Gaussian z noise.
    """
    half_w, half_d = w / 2.0, d / 2.0
    iw, id_ = half_w + inner_margin, half_d + inner_margin
    ow, od = half_w + outer_margin, half_d + outer_margin
    rect_area = 4.0 * ow * od
    ring_area = (2 * ow) * (2 * od) - (2 * iw) * (2 * id_)
    accept = max(1e-3, ring_area / rect_area)
    total_span = sum(max(0.0, e - s) for s, e in sectors)
    pts = []
    remaining = int(count)
    sector_list = []
    for start, end in sectors:
        span = max(0.0, end - start)
        if span <= 0:
            continue
        sector_list.append((start, span, span / max(total_span, 1e-9)))
    guard = 0
    while remaining > 0 and guard < 100:
        for start, span, frac in sector_list:
            if remaining <= 0:
                break
            n_i = max(16, int(math.ceil(
                remaining * frac / accept * 1.1)) + 16)
            u = rng.uniform(-ow, ow, n_i)
            v = rng.uniform(-od, od, n_i)
            in_annulus = ((np.abs(u) > iw) | (np.abs(v) > id_)) & (
                (np.abs(u) < ow) & (np.abs(v) < od))
            u, v = u[in_annulus], v[in_annulus]
            if len(u) == 0:
                continue
            ang = np.arctan2(v, u)
            delta = (ang - start) % (2 * math.pi)
            in_sector = delta < span
            u, v = u[in_sector], v[in_sector]
            if len(u) == 0:
                continue
            take = min(len(u), remaining)
            pts.append(np.stack([u[:take], v[:take]], axis=1))
            remaining -= take
        guard += 1
    if not pts:
        return np.zeros((0, 3))
    uv = np.concatenate(pts, axis=0)[:count]
    # Rotate box-frame (u, v) back to world XY with the fixture yaw = 0
    # (the top estimate carries the yaw; the estimator rotates candidates
    # itself, so generating in world axes with yaw=0 keeps that honest).
    xy = uv
    n = len(xy)
    zz = z + _quantize_noise(rng, n, sigma)
    return np.column_stack([xy, zz])


def _box_points(rng, w, d, h, spacing=0.004):
    """Top face + side walls of the suitcase (occluder)."""
    half_w, half_d = w / 2.0, d / 2.0
    nx = max(4, int(w / spacing))
    nz = max(4, int(h / spacing))
    xs = np.linspace(-half_w, half_w, nx)
    ys = np.linspace(-half_d, half_d, max(4, int(d / spacing)))
    top = np.stack(np.meshgrid(xs, ys), axis=-1).reshape(-1, 2)
    top = np.column_stack([top, np.full(len(top), h)])
    side_pts = []
    for x_edge in (-half_w, half_w):
        gy, gz = np.meshgrid(ys, np.linspace(0, h, nz))
        side_pts.append(np.column_stack([
            np.full(gy.size, x_edge), gy.ravel(), gz.ravel()]))
    for y_edge in (-half_d, half_d):
        gx, gz = np.meshgrid(xs, np.linspace(0, h, nz))
        side_pts.append(np.column_stack([
            gx.ravel(), np.full(gx.size, y_edge), gz.ravel()]))
    side = np.concatenate(side_pts, axis=0)
    pick = rng.choice(len(side), size=min(len(side), 12000), replace=False)
    return np.concatenate([top, side[pick]], axis=0)


def _floor_points(rng, z=-0.9, n=6000):
    xy = rng.uniform(-0.6, 0.6, (n, 2))
    return np.column_stack([xy, np.full(n, z)])


def _flyers(rng, w, d, count, top_z, band_lo, band_hi,
            inner_margin=0.03, outer_margin=0.18):
    half_w, half_d = w / 2.0, d / 2.0
    iw, id_ = half_w + inner_margin, half_d + inner_margin
    ow, od = half_w + outer_margin, half_d + outer_margin
    u = rng.uniform(-ow, ow, count)
    v = rng.uniform(-od, od, count)
    m = ((np.abs(u) > iw) | (np.abs(v) > id_)) & (
        (np.abs(u) < ow) & (np.abs(v) < od))
    z = top_z - rng.uniform(band_lo, band_hi, count)
    return np.column_stack([u[m], v[m], z[m]])


FULL = [(0.0, 2 * math.pi)]
THREE_SIDES = [(-math.pi * 0.75, math.pi * 0.75)]  # 270 deg, one quadrant dark
TWO_ADJACENT = [(-math.pi / 2, math.pi / 2)]       # +u and +v sides only


def _make_raw(rng, w, d, h, annulus_count, sectors, clutter=None,
              flyer_count=0, nonfinite_frac=0.0, floor=True,
              annulus_sigma=0.002):
    parts = [_box_points(rng, w, d, h)]
    parts.append(_annulus_points(
        rng, w, d, annulus_count, sectors, sigma=annulus_sigma))
    if clutter is not None:
        z_c, count_c, sectors_c, sigma_c = clutter
        parts.append(_annulus_points(
            rng, w, d, count_c, sectors_c, z=z_c, sigma=sigma_c))
    if flyer_count:
        parts.append(_flyers(rng, w, d, flyer_count, h, 0.15, 0.60))
    if floor:
        parts.append(_floor_points(rng))
    raw = np.concatenate(parts, axis=0)
    if nonfinite_frac > 0:
        k = int(len(raw) * nonfinite_frac)
        idx = rng.choice(len(raw), k, replace=False)
        bad = rng.choice(3, k)  # poison x, y, or z
        for col in (0, 1, 2):
            sel = idx[bad == col]
            raw[sel, col] = np.nan if rng.rand() < 0.7 else np.inf
    return raw


def build_fixture(fid, family, size_name, **kw):
    w, d, h = BOX_SIZES[size_name]
    digest = hashlib.sha256(fid.encode()).digest()
    rng = np.random.RandomState(
        int.from_bytes(digest[:4], "little"))
    raw = _make_raw(rng, w, d, h, **kw)
    top = TopSurfaceEstimate(
        center_xy=np.zeros(2), top_z=float(h), yaw=0.0,
        width=float(w), depth=float(d), confidence=1.0)
    return {
        "id": fid, "family": family, "size": size_name,
        "raw": raw, "top": top, "workspace": WORKSPACE,
        "gt_support_z": 0.0, "seed_digest": hashlib.sha256(
            fid.encode()).hexdigest()[:16],
    }


def _count_candidates(fix, config):
    """How many points survive the committed band+annulus candidate crop."""
    from luggage_perception.top_support_estimator import (
        _crop_workspace, _rotate_to_rect_axes)
    pts = np.asarray(fix["raw"], dtype=np.float64).reshape(-1, 3)
    pts = pts[np.isfinite(pts).all(axis=1)]
    pts = _crop_workspace(pts, fix["workspace"][0], fix["workspace"][1])
    top = fix["top"]
    band = ((top.top_z - pts[:, 2] >= config.min_luggage_height)
            & (top.top_z - pts[:, 2] <= config.max_luggage_height))
    pts = pts[band]
    u, v = _rotate_to_rect_axes(pts[:, :2], top.center_xy, top.yaw)
    half_w = top.width * 0.5
    half_d = top.depth * 0.5
    iw = half_w + config.support_inner_margin
    id_ = half_d + config.support_inner_margin
    ow = half_w + config.support_outer_margin
    od = half_d + config.support_outer_margin
    m = (((np.abs(u) > iw) | (np.abs(v) > id_))
         & ((np.abs(u) < ow) & (np.abs(v) < od)))
    return int(m.sum()), pts[m]


def build_all_fixtures(config=None):
    from luggage_perception.top_support_estimator import TopSupportConfig
    config = config or TopSupportConfig()
    fixes = []

    def add(fid, family, size, expect, **kw):
        f = build_fixture(fid, family, size, **kw)
        f["expect"] = expect
        fixes.append(f)

    # --- positives: valid measured support at GT z=0 ---
    for size, count in (("carryon", 24000), ("standard", 29000),
                        ("large", 33000)):
        add("flat_clean_%s" % size, "flat_clean", size, "valid",
            annulus_count=count, sectors=FULL)
    add("flat_clean_noise", "quantized_noise", "standard", "valid",
        annulus_count=27000, sectors=FULL, annulus_sigma=0.004)
    for size in ("carryon", "standard", "large"):
        add("asym_%s" % size, "asymmetric_sectors", size, "valid",
            annulus_count=26000, sectors=THREE_SIDES)
    for size in ("carryon", "large"):
        add("missing_side_%s" % size, "missing_side", size, "valid",
            annulus_count=22000, sectors=TWO_ADJACENT)
    # Dense coherent plane 5 cm below support, full ring, same density:
    # correct answer stays z~0 (support is the highest dense plane).
    add("competing_lower", "competing_lower_plane", "standard", "valid",
        annulus_count=26000, sectors=FULL,
        clutter=(-0.05, 26000, FULL, 0.002))
    # Clutter 8 cm below covering only one side; support must still win
    # and never report the clutter plane as measured support. +/-15 deg
    # around +u keeps the clutter inside the +u outer band only (a wider
    # sector bleeds into the +/-v bands and counts as extra sides).
    add("clutter_partial", "wrong_height_clutter_partial", "carryon",
        "valid", annulus_count=22000, sectors=FULL,
        clutter=(-0.08, 9000, [(-math.pi / 12, math.pi / 12)], 0.002))
    add("flyers_mild", "isolated_outliers", "standard", "valid",
        annulus_count=26000, sectors=FULL, flyer_count=400)
    add("floor_below_band", "lower_floor_excluded", "large", "valid",
        annulus_count=30000, sectors=FULL)

    # --- negatives: no valid measured support may be reported ---
    add("insufficient", "insufficient_points", "standard", "reject",
        annulus_count=45, sectors=FULL)
    add("nonfinite_starved", "nonfinite", "standard", "reject",
        annulus_count=20000, sectors=FULL, nonfinite_frac=0.999)
    add("all_nonfinite", "nonfinite", "standard", "reject",
        annulus_count=20000, sectors=FULL, nonfinite_frac=1.0)
    add("flyers_only", "flyers_only", "standard", "reject",
        annulus_count=0, sectors=FULL, flyer_count=300, floor=False)
    # A single coherent plane at wrong height covering ONE side only:
    # plane may be fittable but side coverage (1 < min_support_sides=2)
    # must force a fail-closed reject for every comparator.
    add("clutter_single_side", "clutter_single_side", "carryon", "reject",
        annulus_count=0, sectors=FULL, floor=False,
        clutter=(-0.08, 9000, [(-math.pi / 12, math.pi / 12)], 0.002),
        flyer_count=300)

    for f in fixes:
        n, cand = _count_candidates(f, config)
        f["candidate_count"] = n
        f["candidates_sha256"] = hashlib.sha256(
            cand.tobytes()).hexdigest() if len(cand) else ""
    return fixes


def manifest(fixtures, extra=None):
    m = {
        "master_seed": MASTER_SEED,
        "box_sizes": BOX_SIZES,
        "workspace": {"center": list(WORKSPACE[0]),
                      "half_extents": list(WORKSPACE[1])},
        "fixtures": [
            {
                "id": f["id"], "family": f["family"], "size": f["size"],
                "expect": f["expect"], "gt_support_z": f["gt_support_z"],
                "candidate_count": f["candidate_count"],
                "candidates_sha256": f["candidates_sha256"],
                "seed_digest": f["seed_digest"],
            } for f in fixtures],
    }
    if extra:
        m.update(extra)
    return m


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    fx = build_all_fixtures()
    man = manifest(fx)
    with open(args.out, "w") as fh:
        json.dump(man, fh, indent=2, sort_keys=True)
    print("fixtures: %d" % len(fx))
    for f in fx:
        print("%-28s %-28s %-9s cand=%6d expect=%s"
              % (f["id"], f["family"], f["size"], f["candidate_count"],
                 f["expect"]))
