#!/usr/bin/env python3
"""Unit tests for FreeSpaceModel (P1). No roscore, pure numpy.

Covers: floor-prior init (§4.2.2), vectorized candidate generation (§5.1),
LBCP stability (§5.3), the unknown_above_floor gate, and merge_surface_2d.
"""

import os
import sys
import time
import math
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from luggage_packing.free_space_model import (  # noqa: E402
    FreeSpaceModel,
    STATE_FLOOR_PRIOR,
    STATE_OCCUPIED,
    STATE_FREE,
    SRC_FLOOR_PRIOR,
    SRC_GEOMETRY,
    rect_intersect_corners,
    convex_hull,
    point_in_convex,
)

INNER = (1.49, 1.97, 2.01)
CENTER = (0.0, -1.5, 0.145)  # matches s20 scene_tf interior center


def _model(res=0.05, **kw):
    return FreeSpaceModel(INNER, CENTER, yaw=-1.5708, resolution=res, **kw)


class TestGeometryHelpers(unittest.TestCase):
    def test_rect_intersect(self):
        c = rect_intersect_corners(0, 0, 2, 2, 1, 1, 3, 3)
        self.assertEqual(c, [(1, 1), (2, 1), (2, 2), (1, 2)])
        self.assertEqual(rect_intersect_corners(0, 0, 1, 1, 2, 2, 3, 3), [])

    def test_convex_hull_and_point_in(self):
        hull = convex_hull([(0, 0), (2, 0), (2, 2), (0, 2), (1, 1)])
        self.assertTrue(point_in_convex(hull, (1, 1)))
        self.assertTrue(point_in_convex(hull, (0.5, 0.5)))
        self.assertFalse(point_in_convex(hull, (3, 1)))


class TestFloorPrior(unittest.TestCase):
    def test_init_floor_prior(self):
        m = _model()
        self.assertTrue((m.state == STATE_FLOOR_PRIOR).all())
        self.assertTrue((m.H == 0).all())
        self.assertEqual(len(m.lbcp), 1)  # floor LBCP only

    def test_empty_container_candidates_span_floor(self):
        """Empty container -> floor_prior candidates across the floor, not one corner."""
        m = _model()
        cands = m.candidates([0.7, 0.45, 0.28], allowed_yaws=[0.0], top_n=200)
        self.assertGreater(len(cands), 10)
        for c in cands:
            self.assertEqual(c["support_source"], SRC_FLOOR_PRIOR)
            self.assertTrue(c["feasible"])
            self.assertEqual(c["reason"], "ok")
            self.assertAlmostEqual(c["peak"], 0.0, places=2)
        positions = {(round(c["center_local"][0], 2), round(c["center_local"][1], 2))
                     for c in cands}
        self.assertGreater(len(positions), 10,
                           "floor_prior candidates must span the floor")

    def test_preserves_pi_equivalent_robot_orientation(self):
        candidates = _model().candidates(
            [0.7, 0.45, 0.28],
            allowed_yaws=[0.0, math.pi], top_n=500)
        yaws = {round(item["box_yaw"], 5) for item in candidates}
        self.assertIn(0.0, yaws)
        self.assertIn(round(math.pi, 5), yaws)

    def test_no_floor_prior_when_disabled(self):
        m = _model(floor_prior=False)
        self.assertTrue((m.state != STATE_FLOOR_PRIOR).all())

    def test_candidates_respect_physical_boundary_margin(self):
        margin = 0.05
        m = _model(boundary_margin=margin)
        size = [0.8, 0.5, 0.32]
        candidates = m.candidates(size, allowed_yaws=[0.0], top_n=200)
        self.assertTrue(candidates)
        for candidate in candidates:
            lx, ly = candidate["center_local"][:2]
            self.assertGreaterEqual(
                lx - size[0] * 0.5, -INNER[0] * 0.5 + margin - 1e-6)
            self.assertGreaterEqual(
                ly - size[1] * 0.5, -INNER[1] * 0.5 + margin - 1e-6)

    def test_slab_floor_offset_is_applied_to_candidate_pose(self):
        floor_z = 0.53
        m = _model(floor_z=floor_z)
        candidate = m.candidates(
            [0.70, 0.45, 0.28], allowed_yaws=[0.0], top_n=1)[0]
        expected_center_z = (
            CENTER[2] - INNER[2] * 0.5 + floor_z + 0.28 * 0.5)
        self.assertAlmostEqual(
            candidate["center_base"][2], expected_center_z, places=6)
        self.assertAlmostEqual(
            candidate["clearance_top"],
            INNER[2] - floor_z - 0.28,
            places=4,
        )


class TestLBCPStability(unittest.TestCase):
    def test_floor_placement_stable(self):
        """Floor placements (peak=0) are stable via LBCP_0 (inner rect)."""
        m = _model()
        self.assertTrue(m.is_stable(0.0, 0.0, [0.7, 0.45], 0.0))

    def test_stacking_fully_supported(self):
        """Stacking directly on a box (CoG over the box top) is stable."""
        m = _model()
        # Box center 0.14 above floor -> top at 0.28 (floor-relative).
        m.add_placed_box([0.0, 0.0, -INNER[2] * 0.5 + 0.14], [0.7, 0.45, 0.28])
        self.assertTrue(m.is_stable(0.0, 0.0, [0.7, 0.45], 0.28))

    def test_narrow_stack_gets_exact_support_center_candidate(self):
        m = _model()
        m.add_placed_box(
            [0.0, 0.0, -INNER[2] * 0.5 + 0.14],
            [0.70, 0.45, 0.28])
        candidates = m.candidates(
            [0.55, 0.40, 0.25], allowed_yaws=[0.0], top_n=200)
        centered = [
            item for item in candidates
            if item.get("contained_support_center")]
        self.assertTrue(centered)
        self.assertAlmostEqual(centered[0]["center_local"][0], 0.0)
        self.assertAlmostEqual(centered[0]["center_local"][1], 0.0)
        self.assertTrue(m.has_full_support(
            0.0, 0.0, [0.55, 0.40], 0.28))
        aligned = [
            item for item in candidates
            if item.get("contained_support_aligned")]
        self.assertTrue(aligned)
        for item in aligned:
            self.assertTrue(m.has_full_support(
                item["center_local"][0], item["center_local"][1],
                [0.55, 0.40], 0.28))

    def test_pool_keeps_floor_level_when_a_stack_level_exists(self):
        """Stack candidates must not consume the whole pool before scoring.

        A placed-box top yields exact-geometry candidates carrying
        contained_support flags. A single global sort put those first and
        truncated the floor layer away, so the scorer never saw a floor option.
        """
        m = _model()
        m.add_placed_box(
            [0.0, 0.0, -INNER[2] * 0.5 + 0.14],
            [0.70, 0.45, 0.28])
        candidates = m.candidates(
            [0.55, 0.40, 0.25], allowed_yaws=[0.0], top_n=8)
        peaks = [round(float(item["peak"]), 3) for item in candidates]
        self.assertIn(0.0, peaks, "floor layer starved out of the pool")
        self.assertIn(0.28, peaks, "stack layer missing from the pool")
        self.assertEqual(len(candidates), 8)

    def test_pool_is_not_ordered_by_support_height(self):
        """Pool order must not encode a floor-vs-stack policy; that is the scorer's job."""
        m = _model()
        m.add_placed_box(
            [0.0, 0.0, -INNER[2] * 0.5 + 0.14],
            [0.70, 0.45, 0.28])
        candidates = m.candidates(
            [0.55, 0.40, 0.25], allowed_yaws=[0.0], top_n=40)
        levels = sorted({round(float(c["peak"]), 3) for c in candidates})
        self.assertEqual(levels, [0.0, 0.28])
        for level in levels:
            count = sum(
                1 for c in candidates if round(float(c["peak"]), 3) == level)
            self.assertGreaterEqual(
                count, 4, "level %.2f under-represented" % level)

    def test_stacking_off_edge_unstable(self):
        """Stacking shifted so CoG falls outside the support polygon is unstable."""
        m = _model()
        m.add_placed_box([0.0, 0.0, -INNER[2] * 0.5 + 0.14], [0.5, 0.5, 0.28])
        # box top spans [-0.25, 0.25]; a 0.5-wide box with CoG at x=0.4 is off-edge.
        self.assertFalse(m.is_stable(0.4, 0.0, [0.5, 0.5], 0.28))

    def test_candidates_reject_unknown_above_floor(self):
        """A candidate straddling a box + unobserved floor (peak>0) is rejected."""
        m = _model()
        m.add_placed_box([0.0, 0.0, -INNER[2] * 0.5 + 0.14], [0.7, 0.45, 0.28])
        cands = m.candidates([0.7, 0.45, 0.28], allowed_yaws=[0.0], top_n=200)
        # No feasible candidate may sit at peak>0 over a partly-unobserved footprint.
        for c in cands:
            if c["peak"] > 0.01:
                # stacking candidates must be fully on the placed box (geometry),
                # not floor_prior (which would mean unobserved support).
                self.assertEqual(c["support_source"], SRC_GEOMETRY)


class TestMergeSurface2d(unittest.TestCase):
    def test_merge_observed_free(self):
        """Merging an observed-free heightmap marks columns STATE_FREE."""
        m = _model(floor_prior=True)
        res = m.resolution
        nx, ny = m.nx, m.ny
        surface = {
            "resolution": res, "nx": nx, "ny": ny,
            "height": [[0.0] * ny for _ in range(nx)],
            "state": [["free"] * ny for _ in range(nx)],
        }
        m.merge_surface_2d(surface)
        self.assertTrue((m.state == STATE_FREE).all())

    def test_merge_occupied(self):
        m = _model()
        res, nx, ny = m.resolution, m.nx, m.ny
        surface = {
            "resolution": res, "nx": nx, "ny": ny,
            "height": [[0.3] * ny for _ in range(nx)],
            "state": [["occupied"] * ny for _ in range(nx)],
        }
        m.merge_surface_2d(surface)
        self.assertTrue((m.state == STATE_OCCUPIED).all())
        self.assertTrue((m.H >= 0.3 - 1e-6).all())

    def test_merge_absolute_surface_subtracts_floor_z(self):
        m = _model(floor_z=0.53)
        res, nx, ny = m.resolution, m.nx, m.ny
        surface = {
            "resolution": res, "nx": nx, "ny": ny,
            "height": [[0.83] * ny for _ in range(nx)],
            "state": [["occupied"] * ny for _ in range(nx)],
        }
        m.merge_surface_2d(surface)
        self.assertTrue((m.H >= 0.30 - 1e-6).all())

    def test_geometry_voxel_height_does_not_override_exact_box_top(self):
        m = _model()
        center = [0.0, 0.0, -INNER[2] * 0.5 + 0.16]
        m.add_placed_box(center, [0.8, 0.5, 0.32])
        res, nx, ny = m.resolution, m.nx, m.ny
        surface = {
            "resolution": res, "nx": nx, "ny": ny,
            "height": [[0.0] * ny for _ in range(nx)],
            "state": [["unknown"] * ny for _ in range(nx)],
            "confidence": [["none"] * ny for _ in range(nx)],
        }
        occupied = m.state == STATE_OCCUPIED
        for ix, iy in zip(*occupied.nonzero()):
            surface["height"][ix][iy] = 0.40  # voxel-ceil artefact
            surface["state"][ix][iy] = "occupied"
            surface["confidence"][ix][iy] = "geometry"
        m.merge_surface_2d(surface)
        self.assertAlmostEqual(float(m.H[occupied].max()), 0.32, places=5)
        stacked = [
            c for c in m.candidates(
                [0.7, 0.45, 0.28], [0.0], top_n=500)
            if c["peak"] > 0.0
        ]
        self.assertTrue(stacked)


class TestVectorizedPerf(unittest.TestCase):
    def test_candidate_generation_under_20ms(self):
        """P1 acceptance: vectorized candidate generation p95 < 20 ms (§8.1)."""
        m = _model(res=0.05)
        box = [0.7, 0.45, 0.28]
        # Warm up.
        m.candidates(box, allowed_yaws=[0.0, 1.5707963])
        times = []
        for _ in range(20):
            t0 = time.time()
            m.candidates(box, allowed_yaws=[0.0, 1.5707963])
            times.append((time.time() - t0) * 1000.0)
        times.sort()
        p95 = times[int(0.95 * len(times)) - 1]
        self.assertLess(p95, 20.0, "p95 = %.1f ms (target <20 ms)" % p95)


if __name__ == "__main__":
    unittest.main()
