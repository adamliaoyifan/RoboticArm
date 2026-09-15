#!/usr/bin/env python3
"""ST-1 dynamic top-surface tests (ROS-free, plain pytest).

Covers the DYNAMIC-SUCTION plan gates A0-A4 at algorithm level: component
isolation reasons, multi-plane selection rules, full-range rectangle
recovery, yaw observability, determinism, stamp/frame propagation, and the
architecture isolation rules (no ROS imports, no I/O in the new modules).
"""

from __future__ import division

import math
import os
import unittest
from collections import namedtuple

import numpy as np

from luggage_perception.dynamic_top_surface import (
    DETECT_DYNAMIC_TOP_NO_CONNECTED_PATCH,
    DETECT_DYNAMIC_TOP_NO_PLANE,
    DETECT_DYNAMIC_TOP_TOO_FEW_POINTS,
    DynamicTopConfig,
    estimate_dynamic_top_surface,
)
from luggage_perception.instance_depth_component import (
    COMPONENT_BOUNDARY_CONTINUOUS,
    COMPONENT_COVERAGE_LOW,
    COMPONENT_NO_SEED_DEPTH,
    COMPONENT_TOO_FEW_PIXELS,
    ComponentConfig,
    erode_mask,
    isolate_depth_component,
    label_components_8,
    largest_component_mask,
)

Intrinsics = namedtuple("Intrinsics", "fx fy cx cy")

# Realistic D555-like nadir geometry (matches the pf_r8 fixture): camera at
# world (-1, 0, 1.9) looking straight down, fx=fy=337.22, 640x480.
FX = FY = 337.222
CX, CY = 320.0, 240.0
CAM_X, CAM_Y, CAM_Z = -1.0, 0.0, 1.9
PLATFORM_Z = 0.86
# optical -> world for the nadir fixture camera.
OPTICAL_TO_WORLD = np.array([
    [1.0, 0.0, 0.0, CAM_X],
    [0.0, -1.0, 0.0, CAM_Y],
    [0.0, 0.0, -1.0, CAM_Z],
    [0.0, 0.0, 0.0, 1.0],
])
INTR = Intrinsics(FX, FY, CX, CY)
H, W = 480, 640


def world_to_pixel(x, y):
    """Nadir projection: world XY -> pixel (u, v)."""
    ox, oy = x - CAM_X, CAM_Y - y
    depth = None  # filled by caller context; here only XY mapping
    return ox, oy


def pixel_for_world(x, y, z):
    ox, oy, oz = x - CAM_X, CAM_Y - y, CAM_Z - z
    u = FX * ox / oz + CX
    v = FY * oy / oz + CY
    return u, v


def render_nadir_depth(surfaces, shape=(H, W), noise_sigma_mm=0.0,
                       missing_frac=0.0, outlier_frac=0.0, seed=0):
    """Rasterize horizontal convex surfaces into a nadir depth image.

    ``surfaces``: list of (z_world, polygon) with polygon a convex world
    XY vertex list [(x, y), ...] in any consistent orientation. A pixel
    shows the highest surface containing its ray hit point (nadir camera,
    no perspective occlusion between horizontal surfaces).
    """
    rng = np.random.default_rng(seed)
    uu, vu = np.meshgrid(np.arange(shape[1]), np.arange(shape[0]))
    depth = np.zeros(shape, dtype=np.float64)
    for z_world, poly in sorted(surfaces, key=lambda s: s[0]):
        oz = CAM_Z - z_world
        wx = CAM_X + (uu - CX) * oz / FX
        wy = CAM_Y - (vu - CY) * oz / FY
        inside_pos = np.ones(shape, dtype=bool)
        inside_neg = np.ones(shape, dtype=bool)
        n_vert = len(poly)
        for i in range(n_vert):
            x0, y0 = poly[i]
            x1, y1 = poly[(i + 1) % n_vert]
            cross = ((x1 - x0) * (wy - y0) - (y1 - y0) * (wx - x0))
            inside_pos &= cross >= 0.0
            inside_neg &= cross <= 0.0
        depth[inside_pos | inside_neg] = oz * 1000.0
    if noise_sigma_mm > 0:
        depth[depth > 0] += rng.normal(
            0.0, noise_sigma_mm, size=shape)[depth > 0]
    if outlier_frac > 0:
        out = (rng.random(shape) < outlier_frac) & (depth > 0)
        depth[out] += rng.uniform(-80.0, 80.0, size=int(out.sum()))
    if missing_frac > 0:
        missing = (rng.random(shape) < missing_frac) & (depth > 0)
        depth[missing] = 0.0
    return np.clip(np.round(depth), 0, 65535).astype(np.uint16)


def box_polygon(box_wh, center_xy, yaw_deg):
    """World-space convex quad of a yawed box top."""
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    hw, hd = box_wh[0] / 2.0, box_wh[1] / 2.0
    corners = []
    for du, dv in ((-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd)):
        corners.append((center_xy[0] + du * c - dv * s,
                        center_xy[1] + du * s + dv * c))
    return corners


def box_surfaces(box_wh, center_xy, yaw_deg, top_z, height=0.25):
    """Top surface of a yawed box (nadir view sees the top only)."""
    return [(top_z, box_polygon(box_wh, center_xy, yaw_deg))]


PLATFORM_POLY = [(-1.5, -0.5), (-0.5, -0.5), (-0.5, 0.5), (-1.5, 0.5)]


def platform_plus(box_wh, center_xy, yaw_deg, top_z):
    return [(PLATFORM_Z, PLATFORM_POLY)] + box_surfaces(
        box_wh, center_xy, yaw_deg, top_z)


def bbox_for(box_wh, center_xy, yaw_deg, top_z, expand=0.05):
    """Pixel bbox of the true top projection expanded by ``expand``.

    ``expand`` is the fraction of each axis span added per side (A2's
    "YOLO bbox expanded 10% beyond the true projection" = +10% total per
    axis).
    """
    poly = box_polygon(box_wh, center_xy, yaw_deg)
    us, vs = [], []
    for x, y in poly:
        u, v = pixel_for_world(x, y, top_z)
        us.append(u)
        vs.append(v)
    u0, u1, v0, v1 = min(us), max(us), min(vs), max(vs)
    wu, wv = (u1 - u0) * expand, (v1 - v0) * expand
    return (int(math.floor(u0 - wu)), int(math.floor(v0 - wv)),
            int(math.ceil(u1 + wu)), int(math.ceil(v1 + wv)))


def default_component_cfg():
    return ComponentConfig()


def default_top_cfg():
    return DynamicTopConfig()


def run_dynamic(depth, bbox, cfg_top=None, cfg_comp=None, mask=None,
                instance_region=None, stamp=123.5, frame="world"):
    comp = isolate_depth_component(depth, bbox=bbox, mask=mask,
                                   config=cfg_comp)
    if not comp.ok:
        return None, comp
    region = instance_region
    if region is None and bbox is not None:
        x0, y0, x1, y1 = bbox
        region = np.zeros(depth.shape, dtype=bool)
        region[y0:y1, x0:x1] = True
    result = estimate_dynamic_top_surface(
        depth, comp.mask, INTR, OPTICAL_TO_WORLD, stamp=stamp,
        frame=frame, instance_region=region, config=cfg_top)
    return result, comp


class TestErodeMask(unittest.TestCase):

    def test_erosion_removes_border(self):
        mask = np.zeros((9, 9), dtype=bool)
        mask[2:7, 2:7] = True
        out = erode_mask(mask, 1)
        self.assertTrue(out[3:6, 3:6].all())
        self.assertEqual(int(out.sum()), 9)
        self.assertFalse(out[2, :].any())

    def test_erosion_zero_returns_copy(self):
        mask = np.ones((4, 4), dtype=bool)
        out = erode_mask(mask, 0)
        self.assertTrue(out.all())
        out[:] = False
        self.assertTrue(mask.all())     # independent copy


class TestLabelComponents(unittest.TestCase):

    def test_two_blobs_deterministic(self):
        mask = np.zeros((10, 12), dtype=bool)
        mask[1:3, 1:4] = True      # 6 px
        mask[6:9, 7:11] = True     # 12 px (largest)
        labels, count = label_components_8(mask)
        self.assertEqual(count, 2)
        largest = largest_component_mask(mask)
        self.assertEqual(int(largest.sum()), 12)
        self.assertTrue(largest[6:9, 7:11].all())
        # Determinism: repeat gives identical labels.
        labels2, _ = label_components_8(mask)
        self.assertTrue((labels == labels2).all())

    def test_diagonal_connectivity_is_8(self):
        mask = np.zeros((6, 6), dtype=bool)
        mask[0, 0] = mask[1, 1] = mask[2, 2] = True
        _, count = label_components_8(mask)
        self.assertEqual(count, 1)


class TestComponentIsolation(unittest.TestCase):

    def test_box_top_isolated_from_platform(self):
        box = (0.55, 0.40)
        center = (-1.0, 0.0)
        top_z = PLATFORM_Z + 0.25
        depth = render_nadir_depth(
            platform_plus(box, center, 0.0, top_z),
            noise_sigma_mm=2.0, seed=1)
        bbox = bbox_for(box, center, 0.0, top_z)
        result = isolate_depth_component(depth, bbox=bbox)
        self.assertTrue(result.ok, result.reason)
        self.assertGreater(result.valid_pixels, 200)
        # Component depth sits on the box top, not the platform.
        comp_depth = depth[result.mask]
        self.assertLess(abs(float(np.median(comp_depth))
                            - (CAM_Z - top_z) * 1000.0), 15.0)

    def test_low_coverage_rejected(self):
        # A mostly-platform bbox with a small box corner inside.
        box = (0.25, 0.20)
        center = (-1.0, 0.0)
        top_z = PLATFORM_Z + 0.25
        depth = render_nadir_depth(platform_plus(box, center, 0.0, top_z))
        # Bbox four times the box area -> coverage ~ 1/4 < 0.35.
        x0, y0, x1, y1 = bbox_for(box, center, 0.0, top_z)
        big = (x0 - 60, y0 - 40, x1 + 60, y1 + 40)
        result = isolate_depth_component(depth, bbox=big)
        self.assertEqual(result.reason, COMPONENT_COVERAGE_LOW)

    def test_too_few_pixels_scaled(self):
        # Tiny image: min_pixels scales with area; a minuscule box fails.
        shape = (120, 160)
        tiny = np.full(shape, int((CAM_Z - PLATFORM_Z) * 1000),
                       dtype=np.uint16)
        # 4x4 box patch at the centre.
        tiny[58:62, 78:82] = int((CAM_Z - PLATFORM_Z - 0.25) * 1000)
        result = isolate_depth_component(tiny, bbox=(70, 50, 90, 70))
        self.assertIn(result.reason,
                      (COMPONENT_TOO_FEW_PIXELS, COMPONENT_NO_SEED_DEPTH,
                       COMPONENT_COVERAGE_LOW))

    def test_platform_only_rejected_by_boundary(self):
        # Bbox entirely on the flat platform: surface continues outside.
        depth = render_nadir_depth([(PLATFORM_Z, PLATFORM_POLY)])
        result = isolate_depth_component(depth, bbox=(200, 140, 440, 340))
        self.assertEqual(result.reason, COMPONENT_BOUNDARY_CONTINUOUS)

    def test_distractor_excluded_from_component(self):
        box = (0.55, 0.40)
        center = (-1.0, 0.0)
        top_z = PLATFORM_Z + 0.25
        surfaces = platform_plus(box, center, 0.0, top_z)
        # +40 mm block on the box corner (a step > band and > jump).
        surfaces.append((top_z + 0.04,
                         [(-1.30, -0.15), (-1.15, -0.15),
                          (-1.15, -0.02), (-1.30, -0.02)]))
        depth = render_nadir_depth(surfaces, noise_sigma_mm=1.0, seed=3)
        bbox = bbox_for(box, center, 0.0, top_z)
        x0, y0, x1, y1 = bbox
        result = isolate_depth_component(
            depth, bbox=(x0, y0, x1, y1),
            config=ComponentConfig(boundary_step_mm=0.0))
        self.assertTrue(result.ok, result.reason)
        # The distractor (40 mm higher) must not be in the component.
        u0, v0 = pixel_for_world(-1.30, -0.15, top_z + 0.04)
        u1, v1 = pixel_for_world(-1.15, -0.02, top_z + 0.04)
        self.assertFalse(
            result.mask[int(v0):int(v1), int(u0):int(u1)].any())

    def test_mask_path_prefers_instance_mask(self):
        box = (0.55, 0.40)
        center = (-1.0, 0.0)
        top_z = PLATFORM_Z + 0.25
        depth = render_nadir_depth(platform_plus(box, center, 0.0, top_z))
        bbox = bbox_for(box, center, 0.0, top_z)
        mask = np.zeros(depth.shape, dtype=bool)
        mask[bbox[1]:bbox[3], bbox[0]:bbox[2]] = True
        result = isolate_depth_component(depth, mask=mask)
        self.assertTrue(result.ok, result.reason)
        self.assertEqual(result.diagnostics.get("path"), "mask")


class TestPlaneSelection(unittest.TestCase):

    def _two_plane_scene(self, step_mm=15.0, strip_len=0.18,
                         strip_half_y=0.24, strip_offset_x=0.02,
                         box=(0.55, 0.40), expand=0.22):
        """Box top plus one same-band raised neighbour inside the bbox.

        The strip overlaps the box edge slightly so the two surfaces are
        image-adjacent (a depth step creates a small parallax slit that
        would otherwise put platform pixels between them).
        """
        center = (-1.0, 0.0)
        top_z = PLATFORM_Z + 0.25
        surfaces = platform_plus(box, center, 0.0, top_z)
        x_edge = center[0] + box[0] / 2.0
        x0 = x_edge - strip_offset_x
        x1 = x_edge - strip_offset_x + strip_len
        surfaces.append((top_z + step_mm * 0.001,
                         [(x0, -strip_half_y), (x1, -strip_half_y),
                          (x1, strip_half_y), (x0, strip_half_y)]))
        depth = render_nadir_depth(surfaces, noise_sigma_mm=1.0, seed=7)
        bbox = bbox_for(box, center, 0.0, top_z, expand=expand)
        return depth, bbox, top_z, box, center

    def test_higher_smaller_plane_never_wins(self):
        depth, bbox, top_z, box, center = self._two_plane_scene()
        result, comp = run_dynamic(depth, bbox)
        self.assertIsNotNone(result)
        self.assertEqual(result.reason, "ok")
        # The winning plane is the box top, not the +15 mm strip.
        self.assertLess(abs(result.top_z - top_z), 0.010)
        higher = [c for c in result.plane_candidates
                  if c.height > top_z + 0.005]
        self.assertTrue(higher)
        for cand in higher:
            self.assertFalse(cand.eligible)
            self.assertEqual(cand.rejected_reason,
                             "support_area_below_ratio")

    def test_height_never_selects_alone(self):
        # An eligible higher plane (>= 60% of the best connected area) with
        # lower instance support still loses to the lower, better
        # supported box top: height cannot outweigh the score terms.
        depth, bbox, top_z, box, center = self._two_plane_scene(
            step_mm=18.0, strip_len=0.34, strip_half_y=0.30,
            box=(0.40, 0.30))
        result, comp = run_dynamic(depth, bbox)
        self.assertIsNotNone(result)
        self.assertEqual(result.reason, "ok")
        self.assertLess(abs(result.top_z - top_z), 0.010)
        higher = [c for c in result.plane_candidates
                  if c.height > top_z + 0.005 and c.eligible]
        if higher:
            for cand in higher:
                self.assertEqual(cand.rejected_reason, "outscored")
                box_cands = [c for c in result.plane_candidates
                             if abs(c.height - top_z) < 0.005]
                self.assertTrue(box_cands)
                self.assertTrue(all(
                    cand.score < max(c.score for c in box_cands)
                    for cand in higher))

    def test_no_plane_fails_closed(self):
        # A component with pure noise depth spread produces no candidate.
        rng = np.random.default_rng(11)
        depth = np.full((H, W), int((CAM_Z - PLATFORM_Z) * 1000),
                        dtype=np.uint16)
        noise = rng.integers(700, 1100, size=(H, W)).astype(np.uint16)
        depth[140:340, 200:440] = noise[140:340, 200:440]
        comp = isolate_depth_component(
            depth, bbox=(200, 140, 440, 340),
            config=ComponentConfig(boundary_step_mm=0.0,
                                   median_band_mm=30.0))
        # Wide noise fails the band/coverage gates first; either closed
        # outcome is acceptable, never a valid estimate.
        if comp.ok:
            result = estimate_dynamic_top_surface(
                depth, comp.mask, INTR, OPTICAL_TO_WORLD)
            self.assertNotEqual(result.reason, "ok")


class TestRectangleRecovery(unittest.TestCase):

    def _run_box(self, box_wh, yaw_deg, center=(-1.0, 0.0),
                 top_z=PLATFORM_Z + 0.25, seed=5, cut_corner=None):
        depth = render_nadir_depth(
            platform_plus(box_wh, center, yaw_deg, top_z),
            noise_sigma_mm=2.0, missing_frac=0.05, outlier_frac=0.02,
            seed=seed)
        bbox = bbox_for(box_wh, center, yaw_deg, top_z)
        if cut_corner is not None:
            # Partial visibility: remove a triangular corner region of the
            # component pixels (post-isolation cut drives the PCA bias).
            comp = isolate_depth_component(depth, bbox=bbox)
            if not comp.ok:
                return None, comp
            mask = comp.mask.copy()
            u0, v0 = pixel_for_world(cut_corner[0], cut_corner[1], top_z)
            du = np.arange(W)[None, :]
            dv = np.arange(H)[:, None]
            cut = ((du - u0) + (v0 - dv)) < 40
            mask &= ~cut
            region = np.zeros(depth.shape, dtype=bool)
            region[bbox[1]:bbox[3], bbox[0]:bbox[2]] = True
            result = estimate_dynamic_top_surface(
                depth, mask, INTR, OPTICAL_TO_WORLD, stamp=1.0,
                instance_region=region)
            return result, comp
        return run_dynamic(depth, bbox)

    def test_yawed_box_recovered(self):
        result, comp = self._run_box((0.55, 0.40), 30.0)
        self.assertIsNotNone(result)
        self.assertEqual(result.reason, "ok")
        yaw_err = abs((result.yaw - math.radians(30.0) + math.pi / 2)
                      % math.pi - math.pi / 2)
        self.assertLess(math.degrees(yaw_err), 5.0)
        self.assertLess(abs(result.width - 0.55), 0.030)
        self.assertLess(abs(result.depth - 0.40), 0.030)
        self.assertLess(abs(result.top_z - (PLATFORM_Z + 0.25)), 0.010)
        self.assertLess(abs(result.center_xy[0] + 1.0), 0.020)
        self.assertLess(abs(result.center_xy[1] - 0.0), 0.020)
        self.assertTrue(result.yaw_valid)

    def test_pca_bias_recovered_by_full_range_rectangle(self):
        # Corner cut biases PCA far off the true axis; the full-range
        # rectangle must still recover it (plan A.7).
        result, comp = self._run_box(
            (0.55, 0.40), 30.0, cut_corner=(-1.28, -0.20))
        self.assertIsNotNone(result)
        self.assertEqual(result.reason, "ok")
        yaw_err = abs((result.yaw - math.radians(30.0) + math.pi / 2)
                      % math.pi - math.pi / 2)
        self.assertLess(math.degrees(yaw_err), 5.0)
        self.assertLess(abs(result.width - 0.55), 0.030)

    def test_near_square_yaw_invalid(self):
        result, comp = self._run_box((0.42, 0.40), 45.0)
        self.assertIsNotNone(result)
        self.assertEqual(result.reason, "ok")
        self.assertFalse(result.yaw_valid)

    def test_determinism_identical_repeats(self):
        depth = render_nadir_depth(
            platform_plus((0.55, 0.40), (-1.0, 0.0), 60.0,
                          PLATFORM_Z + 0.25),
            noise_sigma_mm=2.0, missing_frac=0.05, outlier_frac=0.02,
            seed=29)
        bbox = bbox_for((0.55, 0.40), (-1.0, 0.0), 60.0,
                        PLATFORM_Z + 0.25)
        r1, _ = run_dynamic(depth, bbox, stamp=7.25)
        r2, _ = run_dynamic(depth, bbox, stamp=7.25)
        self.assertEqual(r1.reason, "ok")
        self.assertEqual(r1.yaw, r2.yaw)
        self.assertEqual(r1.width, r2.width)
        self.assertEqual(r1.top_z, r2.top_z)
        self.assertTrue(
            (r1.center_xy == r2.center_xy).all())
        self.assertEqual(
            [c.plane_id for c in r1.plane_candidates],
            [c.plane_id for c in r2.plane_candidates])

    def test_offsets_do_not_break_estimate(self):
        for dx, dy in ((-0.30, -0.30), (0.30, 0.30), (0.0, -0.30)):
            result, _ = self._run_box(
                (0.55, 0.40), 0.0, center=(-1.0 + dx, dy))
            self.assertIsNotNone(result)
            self.assertEqual(result.reason, "ok")
            self.assertLess(abs(result.center_xy[0] - (-1.0 + dx)), 0.025)
            self.assertLess(abs(result.center_xy[1] - dy), 0.025)


class TestStampsAndFailClosed(unittest.TestCase):

    def test_stamp_and_frame_carried(self):
        depth = render_nadir_depth(
            platform_plus((0.55, 0.40), (-1.0, 0.0), 0.0,
                          PLATFORM_Z + 0.25), seed=2)
        bbox = bbox_for((0.55, 0.40), (-1.0, 0.0), 0.0, PLATFORM_Z + 0.25)
        result, _ = run_dynamic(depth, bbox, stamp=91.125, frame="world")
        self.assertEqual(result.stamp, 91.125)
        self.assertEqual(result.frame, "world")
        self.assertGreater(result.confidence, 0.70)

    def test_degenerate_component_fails_closed(self):
        depth = np.full((H, W), int((CAM_Z - PLATFORM_Z) * 1000),
                        dtype=np.uint16)
        comp = isolate_depth_component(
            depth, bbox=(300, 200, 306, 206),
            config=ComponentConfig(boundary_step_mm=0.0))
        if comp.ok:
            result = estimate_dynamic_top_surface(
                depth, comp.mask, INTR, OPTICAL_TO_WORLD)
            self.assertNotEqual(result.reason, "ok")
            self.assertIn(result.reason,
                          (DETECT_DYNAMIC_TOP_NO_PLANE,
                           DETECT_DYNAMIC_TOP_NO_CONNECTED_PATCH,
                           DETECT_DYNAMIC_TOP_TOO_FEW_POINTS))

    def test_nan_transform_fails_closed(self):
        depth = render_nadir_depth(
            platform_plus((0.55, 0.40), (-1.0, 0.0), 0.0,
                          PLATFORM_Z + 0.25), seed=4)
        bbox = bbox_for((0.55, 0.40), (-1.0, 0.0), 0.0, PLATFORM_Z + 0.25)
        comp = isolate_depth_component(depth, bbox=bbox)
        self.assertTrue(comp.ok)
        bad = np.full((4, 4), np.nan)
        result = estimate_dynamic_top_surface(
            depth, comp.mask, INTR, bad, stamp=1.0)
        self.assertNotEqual(result.reason, "ok")


class TestPipelineTopSurfaceKwarg(unittest.TestCase):
    """PlatformFreeDetector.update(top_surface=...) composes the dynamic
    top through the unchanged support/compose paths (A0 integration)."""

    def _dynamic_top(self, yaw=30.0):
        box = (0.55, 0.40)
        center = (-1.0, 0.0)
        top_z = PLATFORM_Z + 0.25
        depth = render_nadir_depth(
            platform_plus(box, center, yaw, top_z),
            noise_sigma_mm=2.0, missing_frac=0.05, outlier_frac=0.02,
            seed=5)
        bbox = bbox_for(box, center, yaw, top_z)
        result, comp = run_dynamic(depth, bbox, stamp=11.5)
        self.assertIsNotNone(result)
        self.assertEqual(result.reason, "ok")
        return result

    def test_dynamic_top_composes_through_pipeline(self):
        from luggage_perception.platform_free_pipeline import (
            PlatformFreeDetector,
        )
        from luggage_perception.top_support_estimator import (
            TopSupportConfig,
        )
        top = self._dynamic_top()
        pipeline = PlatformFreeDetector(
            config=TopSupportConfig(), support_mode="auto")
        pts = np.column_stack((
            np.random.default_rng(3).uniform(-0.2, 0.2, (200, 2)),
            np.full(200, top.top_z)))
        result = pipeline.update(
            pts, None, source="measure", geometry_ok=True,
            raw_same_stamp=False, stamp_sec=11.5, top_surface=top)
        self.assertTrue(result.top_valid)
        self.assertEqual(result.top_source, "dynamic")
        self.assertEqual(result.top_reason, "ok")
        # Composition consumes the dynamic rectangle verbatim.
        self.assertAlmostEqual(result.box.width, top.width, places=9)
        self.assertAlmostEqual(result.box.depth, top.depth, places=9)
        self.assertAlmostEqual(
            float(result.box.top.center_xy[0]), float(top.center_xy[0]),
            places=9)
        # No same-stamp raw cloud -> support skipped with its own reason;
        # height stays TOP_ONLY (catalog prior absent here).
        self.assertEqual(result.support_gate, "raw_stamp_mismatch")
        self.assertFalse(result.height_valid)

    def test_legacy_default_path_unchanged(self):
        from luggage_perception.platform_free_pipeline import (
            PlatformFreeDetector,
        )
        pipeline = PlatformFreeDetector(support_mode="auto")
        rng = np.random.default_rng(4)
        pts = np.column_stack((
            rng.uniform(-0.2, 0.2, (400, 1)),
            rng.uniform(-0.15, 0.15, (400, 1)),
            np.full(400, PLATFORM_Z + 0.25),
        ))
        result = pipeline.update(pts, None, source="measure",
                                 stamp_sec=2.0)
        self.assertTrue(result.top_valid)
        self.assertEqual(result.top_source, "legacy")


MODULE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          os.pardir, "luggage_perception")
NEW_MODULES = (
    "instance_depth_component.py",
    "dynamic_top_surface.py",
)
SRC_ROOT = os.path.abspath(os.path.join(MODULE_DIR, os.pardir, os.pardir))
DETECTOR_NODE = os.path.join(
    MODULE_DIR, os.pardir, "scripts", "luggage_detector_node.py")
HARDWARE_LAUNCH = os.path.join(
    SRC_ROOT, "luggage_planning", "launch", "hardware_pick.launch.py")
SIM_LAUNCH = os.path.join(
    SRC_ROOT, "luggage_gazebo", "launch", "sim_world.launch.py")


class TestArchitectureIsolation(unittest.TestCase):
    """A0: no ROS/message/TF imports and no file I/O in algorithm modules."""

    FORBIDDEN = ("rclpy", "rospy", "tf2_ros", "sensor_msgs", "luggage_msgs",
                 "builtin_interfaces", "std_msgs", "geometry_msgs")
    FORBIDDEN_IO = ("open(", "np.load", "np.save", "os.remove",
                    "pathlib", "yaml.")

    def test_no_ros_imports_or_io(self):
        for name in NEW_MODULES:
            path = os.path.join(MODULE_DIR, name)
            with open(path) as fh:
                source = fh.read()
            for token in self.FORBIDDEN:
                self.assertNotIn(
                    "import %s" % token, source,
                    "%s must not import %s" % (name, token))
            for token in self.FORBIDDEN_IO:
                self.assertNotIn(
                    token, source,
                    "%s must not perform file I/O (%s)" % (name, token))


class TestSceneIndependenceStructure(unittest.TestCase):
    """A1 structural half: no scene_tf pose enters the detect path and the
    hardware launch enables the dynamic top with crop/predicate off."""

    def test_detector_reads_no_scene_pose(self):
        with open(DETECTOR_NODE) as fh:
            source = fh.read()
        self.assertNotIn(
            "pickup_source_in_world", source,
            "detector must not read scene_tf pickup_source (ST-1)")
        self.assertIn("DETECT_CONFIG_WORKSPACE_CENTER_REQUIRED", source,
                      "crop without an explicit center must fail closed")

    def test_hardware_launch_params(self):
        with open(HARDWARE_LAUNCH) as fh:
            source = fh.read()
        self.assertIn('"top_surface_mode": "dynamic"', source)
        self.assertIn('"crop_to_workspace": False', source)
        self.assertIn('"workspace_accept_enabled": False', source)

    def test_sim_launch_unchanged_defaults(self):
        with open(SIM_LAUNCH) as fh:
            source = fh.read()
        self.assertNotIn("top_surface_mode", source)
        self.assertNotIn("crop_to_workspace", source)


if __name__ == "__main__":
    unittest.main()
