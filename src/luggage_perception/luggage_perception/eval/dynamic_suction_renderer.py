#!/usr/bin/env python3
"""Synthetic RGB-D case renderer for the dynamic-suction A gates (ST-1).

Deterministic numpy ray-cast renderer for a nadir D555-like camera over a
pickup platform (the pf_r8 fixture geometry). Produces one
:class:`SyntheticCase` per generated case: aligned 640x480 depth in
millimetres, a YOLO-style bbox (+10 % beyond the true top projection), the
truth reference, and the exact-stamp optical->world transform.

Realism knobs fixed by the plan: 5 % missing depth, sigma 2 mm Gaussian Z
noise, 2 % uniform outliers (A2); platform background fractions, a +40 mm
distractor occupying 10 % of the ROI, and a robot-self-mask hole (A3);
partial-visibility cuts calibrated to a target PCA bias (A4).

Eval-layer module: it may read files, but it never publishes a pose and
the online path imports none of it.
"""

from __future__ import division

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

from luggage_perception.instance_depth_component import (
    ComponentConfig,
    isolate_depth_component,
)

# pf_r8 fixture intrinsics (test/fixtures/pf_r8/acceptance_fixture.json:
# D555-like fx/fy, 640x480, nadir optical +Z down) on a 2.2 m synthetic
# standoff. The fixture's 1.9 m pose crops the plan's A2 corner cases
# (large box, yaw 90, dy=+-0.30 reaches |y|=0.70 m -> v>480); 2.2 m keeps
# the full size x offset x yaw matrix inside the frame while the
# intrinsics stay D555-like. The real rig's FOV behaviour is gate A5's
# domain, not the synthetic matrix's.
DEFAULT_CAMERA = {
    "fx": 337.22194822727283,
    "fy": 337.22194822727283,
    "cx": 320.0,
    "cy": 240.0,
    "width": 640,
    "height": 480,
    "optical_to_world": [
        [1.0, 0.0, 0.0, -1.0],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, 2.2],
        [0.0, 0.0, 0.0, 1.0],
    ],
    "platform_z": 0.86,
    "platform_poly": [[-1.5, -0.5], [-0.5, -0.5], [-0.5, 0.5], [-1.5, 0.5]],
}

# CATALOG (test/test_luggage_box_estimator.py): carryon/standard/large.
DEFAULT_CATALOG = (
    ("carryon", (0.55, 0.40, 0.25)),
    ("standard", (0.70, 0.45, 0.28)),
    ("large", (0.80, 0.50, 0.32)),
)


@dataclass
class SyntheticCase:
    """One scored acquisition plus its separated truth reference."""

    case_id: str
    gate: str
    depth_mm: np.ndarray            # (H, W) uint16
    bbox: tuple                     # (x0, y0, x1, y1) pixels
    camera: dict
    gt_center_xy: tuple
    gt_top_z: float
    gt_yaw_deg: float
    gt_size_wh: tuple
    stamp: float
    frame: str = "world"
    seed: int = 0
    generation: int = 1
    instance_id: str = "synthetic"
    meta: dict = field(default_factory=dict)

    @property
    def mat4(self):
        return np.asarray(self.camera["optical_to_world"], dtype=np.float64)


class NadirRenderer(object):
    """Polygon rasterizer for horizontal convex surfaces, nadir camera."""

    def __init__(self, camera=None):
        self.camera = dict(DEFAULT_CAMERA)
        if camera:
            self.camera.update(dict(camera))
        self._uu, self._vu = np.meshgrid(
            np.arange(self.camera["width"]),
            np.arange(self.camera["height"]))

    def world_xy_at(self, z_world):
        oz = self.camera["optical_to_world"][2][3] - z_world
        wx = (self.camera["optical_to_world"][0][3]
              + (self._uu - self.camera["cx"]) * oz / self.camera["fx"])
        wy = (self.camera["optical_to_world"][1][3]
              - (self._vu - self.camera["cy"]) * oz / self.camera["fy"])
        return wx, wy

    def render(self, surfaces, noise_sigma_mm=2.0, missing_frac=0.05,
               outlier_frac=0.02, seed=0, hole_poly=None):
        """Rasterize surfaces [(z, poly)] highest-wins; then sensor model.

        ``hole_poly`` carves a robot-self-mask hole (invalid depth) from
        the given convex world polygon.
        """
        rng = np.random.default_rng(seed)
        shape = (self.camera["height"], self.camera["width"])
        depth = np.zeros(shape, dtype=np.float64)
        for z_world, poly in sorted(surfaces, key=lambda s: s[0]):
            wx, wy = self.world_xy_at(z_world)
            inside = self._inside(wx, wy, poly)
            depth[inside] = ((
                self.camera["optical_to_world"][2][3] - z_world) * 1000.0)
        if hole_poly is not None:
            # The hole invalidates whatever surface shows there; use the
            # platform level for the containment test (holes sit on the
            # visible top, the z choice only shifts the mask by <1 px).
            wx, wy = self.world_xy_at(self.camera["platform_z"] + 0.30)
            depth[self._inside(wx, wy, hole_poly)] = 0.0
        seen = depth > 0
        if noise_sigma_mm > 0:
            depth[seen] += rng.normal(0.0, noise_sigma_mm,
                                      size=int(seen.sum()))
        if outlier_frac > 0:
            out = (rng.random(shape) < outlier_frac) & seen
            depth[out] += rng.uniform(-80.0, 80.0, size=int(out.sum()))
        if missing_frac > 0:
            missing = (rng.random(shape) < missing_frac) & seen
            depth[missing] = 0.0
        return np.clip(np.round(depth), 0, 65535).astype(np.uint16)

    @staticmethod
    def _inside(wx, wy, poly):
        inside_pos = np.ones(wx.shape, dtype=bool)
        inside_neg = np.ones(wx.shape, dtype=bool)
        n = len(poly)
        for i in range(n):
            x0, y0 = poly[i]
            x1, y1 = poly[(i + 1) % n]
            cross = ((x1 - x0) * (wy - y0) - (y1 - y0) * (wx - x0))
            inside_pos &= cross >= 0.0
            inside_neg &= cross <= 0.0
        return inside_pos | inside_neg


def box_polygon(size_wh, center_xy, yaw_deg):
    """Convex quad of a yawed box top in world coordinates."""
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    hw, hd = size_wh[0] / 2.0, size_wh[1] / 2.0
    return [
        (center_xy[0] + du * c - dv * s, center_xy[1] + du * s + dv * c)
        for du, dv in ((-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd))]


def pixel_bbox(renderer, poly, z_world, expand_frac=0.05):
    """True-projection pixel bbox expanded by ``expand_frac`` per side."""
    cam = renderer.camera
    oz = cam["optical_to_world"][2][3] - z_world
    us, vs = [], []
    for x, y in poly:
        us.append(cam["fx"] * (x - cam["optical_to_world"][0][3]) / oz
                  + cam["cx"])
        vs.append(cam["fy"] * (cam["optical_to_world"][1][3] - y) / oz
                  + cam["cy"])
    u0, u1, v0, v1 = min(us), max(us), min(vs), max(vs)
    wu, wv = (u1 - u0) * expand_frac, (v1 - v0) * expand_frac
    return (int(math.floor(u0 - wu)), int(math.floor(v0 - wv)),
            int(math.ceil(u1 + wu)), int(math.ceil(v1 + wv)))


def platform_surfaces(camera_poly):
    return [(DEFAULT_CAMERA["platform_z"],
             [tuple(p) for p in camera_poly])]


def make_a2_case(renderer, size_id, size_whd, center_xy, yaw_deg, seed,
                 stamp):
    """Plan A2: clean dynamic-position case (324-case matrix member)."""
    top_z = DEFAULT_CAMERA["platform_z"] + size_whd[2]
    poly = box_polygon(size_whd[:2], center_xy, yaw_deg)
    surfaces = platform_surfaces(DEFAULT_CAMERA["platform_poly"])
    surfaces.append((top_z, poly))
    depth = renderer.render(surfaces, seed=seed)
    bbox = pixel_bbox(renderer, poly, top_z)
    return SyntheticCase(
        case_id="a2_%s_%+d%+d_%d_%d" % (
            size_id, round(center_xy[0] * 100), round(center_xy[1] * 100),
            int(yaw_deg), seed),
        gate="A2", depth_mm=depth, bbox=bbox, camera=renderer.camera,
        gt_center_xy=tuple(center_xy), gt_top_z=top_z,
        gt_yaw_deg=float(yaw_deg), gt_size_wh=tuple(size_whd[:2]),
        stamp=stamp, seed=seed,
        meta={"size_id": size_id, "offset": tuple(center_xy),
              "yaw_deg": yaw_deg})


def make_a3_case(renderer, size_id, size_whd, background_frac, seed, stamp):
    """Plan A3: bbox contamination + multi-plane selection.

    Construction (documented for the evidence manifest): the ROI is the
    scene window of 1.75x the box span around the box; the platform fills
    ``background_frac`` of the ROI by widening the visible platform strip;
    a +40 mm distractor occupying ~10 % of the ROI sits adjacent to the
    box; a self-mask hole (~5 % of the bbox) is carved near a top edge; a
    +15 mm same-band strip (~20 % of the bbox width) abuts the box inside
    the bbox so the merged component genuinely contains two planes. The
    YOLO bbox stays the +10 % true-projection box.
    """
    center_xy = (-1.0, 0.0)
    yaw_deg = 0.0
    top_z = DEFAULT_CAMERA["platform_z"] + size_whd[2]
    w, d = size_whd[:2]
    poly = box_polygon((w, d), center_xy, yaw_deg)
    # Platform: whole ROI window; the "background fraction" is realized by
    # shrinking the box's share of the ROI through ROI widening: the ROI
    # window is the platform extent actually rendered inside the image.
    roi_half = max(w, d) * 0.5 / max(0.30, 1.0 - background_frac)
    platform_poly = [
        (center_xy[0] - roi_half * 1.6, center_xy[1] - roi_half * 1.2),
        (center_xy[0] + roi_half * 1.6, center_xy[1] - roi_half * 1.2),
        (center_xy[0] + roi_half * 1.6, center_xy[1] + roi_half * 1.2),
        (center_xy[0] - roi_half * 1.6, center_xy[1] + roi_half * 1.2),
    ]
    surfaces = [(DEFAULT_CAMERA["platform_z"], platform_poly)]
    # Same-band raised strip: +15 mm, ~20% of the box width, abutting the
    # +X face (image-adjacent via 2 cm overlap).
    x_edge = center_xy[0] + w / 2.0
    strip_len = 0.20 * w + 0.02
    surfaces.append((top_z + 0.015,
                     [(x_edge - 0.02, center_xy[1] - d / 2.0),
                      (x_edge - 0.02 + strip_len, center_xy[1] - d / 2.0),
                      (x_edge - 0.02 + strip_len, center_xy[1] + d / 2.0),
                      (x_edge - 0.02, center_xy[1] + d / 2.0)]))
    surfaces.append((top_z, poly))
    # +40 mm distractor near the -X/-Y corner of the box, ~10% of ROI area.
    side = math.sqrt(0.10 * (2 * roi_half) ** 2 * 0.5)
    surfaces.append((top_z + 0.04,
                     [(center_xy[0] - w / 2.0 - side, center_xy[1] - d / 2.0),
                      (center_xy[0] - w / 2.0, center_xy[1] - d / 2.0),
                      (center_xy[0] - w / 2.0, center_xy[1] - d / 2.0 + side),
                      (center_xy[0] - w / 2.0 - side,
                       center_xy[1] - d / 2.0 + side)]))
    # Self-mask hole: ~5% of the bbox area near the +Y edge of the top.
    hole_side = math.sqrt(0.05) * w
    hole_poly = [
        (center_xy[0] + 0.30 * w - hole_side / 2,
         center_xy[1] + d / 2.0 - hole_side),
        (center_xy[0] + 0.30 * w + hole_side / 2,
         center_xy[1] + d / 2.0 - hole_side),
        (center_xy[0] + 0.30 * w + hole_side / 2, center_xy[1] + d / 2.0),
        (center_xy[0] + 0.30 * w - hole_side / 2, center_xy[1] + d / 2.0),
    ]
    depth = renderer.render(surfaces, seed=seed, hole_poly=hole_poly)
    bbox = pixel_bbox(renderer, poly, top_z)
    return SyntheticCase(
        case_id="a3_%s_%02d_%d" % (size_id, int(background_frac * 100), seed),
        gate="A3", depth_mm=depth, bbox=bbox, camera=renderer.camera,
        gt_center_xy=tuple(center_xy), gt_top_z=top_z,
        gt_yaw_deg=yaw_deg, gt_size_wh=(w, d), stamp=stamp, seed=seed,
        meta={"size_id": size_id, "background_frac": background_frac,
              "construction": "ROI-level platform fraction, +40mm "
                              "distractor (10% ROI), self-mask hole, "
                              "+15mm same-band strip inside bbox"})


def make_a4_case(renderer, size_id, size_whd, target_bias_deg, seed, stamp,
                 component_cfg=None):
    """Plan A4: partial visibility calibrated to a target PCA bias.

    Occlusion shape: axis-aligned corner notches in the BOX frame (main
    notch at one corner, smaller counter-notch at the opposite corner).
    Box-frame notches keep both axis extremes in the hull — the full-range
    rectangle can still recover the true extents — while rotating PCA the
    way a partial view does. The notch depth is swept so the MEASURED PCA
    bias lands as close to ``target_bias_deg`` as the coverage gate allows
    (the 40 deg column tops out near ~30 deg measured; recorded honestly
    in ``measured_bias_deg``). The bbox is the VISIBLE top's projection
    +10 % per axis — YOLO boxes what it can see; the truth reference stays
    the full rectangle.
    """
    center_xy = (-1.0, 0.0)
    yaw_deg = 30.0
    top_z = DEFAULT_CAMERA["platform_z"] + size_whd[2]
    w, d = size_whd[:2]
    poly = box_polygon((w, d), center_xy, yaw_deg)
    surfaces = platform_surfaces(DEFAULT_CAMERA["platform_poly"])
    surfaces.append((top_z, poly))
    depth_full = renderer.render(surfaces, seed=seed)

    cam = renderer.camera
    oz = cam["optical_to_world"][2][3] - top_z
    top_depth = round((cam["optical_to_world"][2][3] - top_z) * 1000)
    on_top = (depth_full > 0) & (
        np.abs(depth_full.astype(np.int32) - top_depth) <= 25)
    uu, vu = np.meshgrid(np.arange(cam["width"]), np.arange(cam["height"]))
    wx = (cam["optical_to_world"][0][3]
          + (uu - cam["cx"]) * oz / cam["fx"])
    wy = (cam["optical_to_world"][1][3]
          - (vu - cam["cy"]) * oz / cam["fy"])
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    bu = (wx - center_xy[0]) * c + (wy - center_xy[1]) * s
    bv = -(wx - center_xy[0]) * s + (wy - center_xy[1]) * c
    a, b = w / 2.0, d / 2.0
    counter_ratio = 0.25 / 0.65   # counter-notch depth per unit main depth

    def notch_for(depth_frac):
        t, sq = depth_frac * 2.0 * a, depth_frac * 2.0 * b
        notch = (bu > a - t) & (bv > b - sq)
        if depth_frac > 0:
            t2, s2 = (depth_frac * counter_ratio * 2.0 * a,
                      depth_frac * counter_ratio * 2.0 * b)
            notch |= (bu < -a + t2) & (bv < -b + s2)
        return notch

    def pca_bias(visible):
        pts = np.column_stack((wx[visible], wy[visible]))
        centered = pts - pts.mean(axis=0)
        eigvals, eigvecs = np.linalg.eigh(np.cov(centered, rowvar=False))
        principal = eigvecs[:, np.argsort(eigvals)[::-1][0]]
        pca_yaw = math.degrees(math.atan2(principal[1], principal[0]))
        return abs((pca_yaw - yaw_deg + 90.0) % 180.0 - 90.0)

    def coverage_ok(notch):
        depth_cut = depth_full.copy()
        depth_cut[on_top & notch] = round(
            (cam["optical_to_world"][2][3]
             - DEFAULT_CAMERA["platform_z"]) * 1000)
        visible = on_top & ~notch
        if visible.sum() < 200:
            return False, None, None
        ys, xs = np.nonzero(visible)
        span_u, span_v = float(xs.max() - xs.min()), float(
            ys.max() - ys.min())
        bbox = (int(math.floor(xs.min() - 0.05 * span_u)),
                int(math.floor(ys.min() - 0.05 * span_v)),
                int(math.ceil(xs.max() + 0.05 * span_u)),
                int(math.ceil(ys.max() + 0.05 * span_v)))
        cfg = component_cfg if component_cfg is not None else ComponentConfig()
        comp = isolate_depth_component(depth_cut, bbox=bbox, config=cfg)
        return comp.ok, depth_cut, bbox

    best = None   # (|bias - target|, depth_frac, bias, depth_cut, bbox)
    for depth_frac in np.linspace(0.0, 0.65, 27):
        notch = notch_for(float(depth_frac))
        visible = on_top & ~notch
        if visible.sum() < 200:
            continue
        ok, depth_cut, bbox = coverage_ok(notch)
        if not ok:
            continue
        bias = pca_bias(visible)
        cand = (abs(bias - float(target_bias_deg)), float(depth_frac))
        if best is None or cand < (best[0], best[1]):
            best = (cand[0], cand[1], bias, depth_cut, bbox)
    if best is None:
        depth_cut, bbox = depth_full, pixel_bbox(renderer, poly, top_z)
        best = (abs(float(target_bias_deg)), 0.0, 0.0, depth_cut, bbox)
    _, depth_frac, measured_bias, depth, bbox = best
    return SyntheticCase(
        case_id="a4_%s_%02d_%d" % (size_id, int(target_bias_deg), seed),
        gate="A4", depth_mm=depth, bbox=bbox, camera=renderer.camera,
        gt_center_xy=tuple(center_xy), gt_top_z=top_z,
        gt_yaw_deg=yaw_deg, gt_size_wh=(w, d), stamp=stamp, seed=seed,
        meta={"size_id": size_id, "target_bias_deg": target_bias_deg,
              "measured_bias_deg": round(measured_bias, 2),
              "notch_depth_frac": round(float(depth_frac), 3),
              "construction": "box-frame double corner notch, "
                              "visible-top bbox +10%"})


def make_platform_only_case(renderer, center_xy, seed, stamp):
    """Negative probe: platform/background-only ROI must never yield a
    valid top (A2 'false top count exactly zero')."""
    depth = renderer.render(
        platform_surfaces(DEFAULT_CAMERA["platform_poly"]), seed=seed)
    u = int(cam_px(renderer, center_xy, DEFAULT_CAMERA["platform_z"]))
    half = 60
    bbox = (max(0, u - half), 180, min(renderer.camera["width"], u + half),
            300)
    return SyntheticCase(
        case_id="a2neg_%+d%+d_%d" % (round(center_xy[0] * 100),
                                     round(center_xy[1] * 100), seed),
        gate="A2-negative", depth_mm=depth, bbox=bbox,
        camera=renderer.camera, gt_center_xy=None, gt_top_z=None,
        gt_yaw_deg=0.0, gt_size_wh=None, stamp=stamp, seed=seed,
        meta={"kind": "platform_only"})


def make_near_square_case(renderer, seed, stamp):
    """Aspect observability probe: a 0.42x0.40 top must report
    ``yaw_valid=False`` (A2 sub-criterion)."""
    center_xy = (-1.0, 0.0)
    size = (0.42, 0.40)
    top_z = DEFAULT_CAMERA["platform_z"] + 0.25
    poly = box_polygon(size, center_xy, 30.0)
    surfaces = platform_surfaces(DEFAULT_CAMERA["platform_poly"])
    surfaces.append((top_z, poly))
    depth = renderer.render(surfaces, seed=seed)
    bbox = pixel_bbox(renderer, poly, top_z)
    return SyntheticCase(
        case_id="a2sq_%d" % seed, gate="A2-aspect", depth_mm=depth,
        bbox=bbox, camera=renderer.camera, gt_center_xy=tuple(center_xy),
        gt_top_z=top_z, gt_yaw_deg=30.0, gt_size_wh=size, stamp=stamp,
        seed=seed, meta={"kind": "near_square", "aspect": 1.05})


def cam_px(renderer, world_xy, z_world):
    cam = renderer.camera
    oz = cam["optical_to_world"][2][3] - z_world
    return cam["fx"] * (world_xy[0] - cam["optical_to_world"][0][3]) \
        / oz + cam["cx"]
