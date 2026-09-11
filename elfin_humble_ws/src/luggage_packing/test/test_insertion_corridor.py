#!/usr/bin/env python3
"""Unit tests for insertion corridor + no-block-deep-EMS + proxy score. No roscore."""

import os
import sys
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from luggage_packing.ems import EMS  # noqa: E402
from luggage_packing.insertion_corridor import (  # noqa: E402
    corridor_blocked,
    blocks_deep_space,
    proxy_score,
)
from luggage_packing.free_space_model import FreeSpaceModel  # noqa: E402

INNER = (2.0, 2.0, 2.0)
SMALL = (0.4, 0.4, 0.25)  # smallest catalog size


class TestCorridor(unittest.TestCase):
    def test_corridor_free_when_empty(self):
        ems = (0.5, -0.5, 0.0, 1.0, 0.5, 0.5)  # deep EMS
        self.assertFalse(corridor_blocked(ems, [], INNER, SMALL))

    def test_corridor_blocked_by_wall_near_opening(self):
        """A box walling off the opening blocks the deep EMS corridor."""
        deep_ems = (0.5, -0.5, 0.0, 1.0, 0.5, 0.5)
        wall = (-1.0, -1.0, 0.0, -0.5, 1.0, 0.5)  # near opening, full y, z[0,0.5]
        self.assertTrue(corridor_blocked(deep_ems, [wall], INNER, SMALL))

    def test_full_width_wall_allows_1mm_slop(self):
        """Catalog width == inner_w plus yaw noise still counts as a wall."""
        inner = (1.49, 1.97, 1.48)
        deep = (0.125, -0.2, 0.0, 0.675, 0.2, 0.25)
        wall = (-0.4, -0.984995, 0.0, 0.0, 0.985005, 0.32)
        self.assertTrue(corridor_blocked(deep, [wall], inner, (0.55, 0.40, 0.25)))

    def test_corridor_not_blocked_by_box_outside_z(self):
        """A box at a different z-level does not block the corridor."""
        deep_ems = (0.5, -0.5, 0.0, 1.0, 0.5, 0.5)
        wall = (-1.0, -1.0, 1.0, -0.5, 1.0, 1.5)  # z[1,1.5] -- above the EMS
        self.assertFalse(corridor_blocked(deep_ems, [wall], INNER, SMALL))


class TestBlocksDeepSpace(unittest.TestCase):
    def test_walling_off_deep_space_is_blocked(self):
        """Placing a wall near the opening blocks the deep EMS -> is_blocked."""
        e = EMS(INNER, min_useful_edge=0.1)
        cand_box = (-1.0, -1.0, 0.0, -0.5, 1.0, 0.5)  # wall near opening
        blocked, is_blocked = blocks_deep_space(
            cand_box, e, [], INNER, SMALL,
            v_min=SMALL[0] * SMALL[1] * SMALL[2])
        self.assertTrue(is_blocked, "a wall near the opening must block deep space")
        self.assertGreater(blocked, 0.0)

    def test_floor_box_not_blocked(self):
        """A single floor box in the corner does not wall off the deep interior."""
        e = EMS(INNER, min_useful_edge=0.1)
        e.place((-0.9, -0.9, 0.0, -0.5, -0.5, 0.25))  # corner box
        cand_box = (0.3, 0.3, 0.0, 0.7, 0.7, 0.25)  # mid-floor, not a wall
        _, is_blocked = blocks_deep_space(
            cand_box, e, [], INNER, SMALL, v_min=SMALL[0] * SMALL[1] * SMALL[2],
            blocked_tol=0.5)  # generous tol: only large blockage counts
        self.assertFalse(is_blocked)


class TestProxyScore(unittest.TestCase):
    def _cand(self, peak=0.0, source="floor_prior"):
        return {
            "center_local": [0.0, 0.0, peak + 0.14 - 1.0],
            "size": [0.7, 0.45, 0.28],
            "support_source": source,
            "peak": peak,
            "clearance_top": 2.0 - peak - 0.28,
            "confidence_ratio": 1.0 if source != "floor_prior" else 0.0,
        }

    def test_proxy_returns_score_and_breakdown(self):
        m = FreeSpaceModel(INNER, (0.0, 0.0, 1.0), resolution=0.1)
        e = EMS(INNER, min_useful_edge=0.1)
        score, bd = proxy_score(self._cand(), m, e, INNER, SMALL,
                                reachability_prior=0.5)
        self.assertIsInstance(score, float)
        self.assertIn("ems_regularity", bd)
        self.assertIn("blocked_deep_ems_ratio", bd)
        self.assertIn("cog_height", bd)

    def test_floor_prior_scores_lower_than_observed(self):
        """An unobserved floor_prior candidate scores below an observed one."""
        m = FreeSpaceModel(INNER, (0.0, 0.0, 1.0), resolution=0.1)
        e = EMS(INNER, min_useful_edge=0.1)
        s_prior, _ = proxy_score(self._cand(source="floor_prior"), m, e,
                                 INNER, SMALL, reachability_prior=0.5)
        s_obs, _ = proxy_score(self._cand(source="sensor"), m, e,
                               INNER, SMALL, reachability_prior=0.5)
        self.assertLess(s_prior, s_obs,
                        "floor_prior must score below observed (lower confidence)")


def _scene_floor_geometry():
    from luggage_description.container_geometry import normalize_descriptor
    return normalize_descriptor(
        {
            "schema_version": 1,
            "frame_id": "container_link",
            "length": 1.49,
            "width": 1.97,
            "floor_z": 0.0,
            "ceiling_z": 1.48,
            "chamfer": {
                "side": "positive_y",
                "floor_y": 0.55,
                "wall_y": 0.985,
                "wall_z": 0.37,
            },
        }
    )


class TestGateG4Corridor(unittest.TestCase):
    INNER = (1.49, 1.97, 1.48)
    SMALL = (0.55, 0.40, 0.25)

    def test_unsupported_opening_side_fails_closed(self):
        ems = (0.2, -0.2, 0.0, 0.6, 0.2, 0.25)
        with self.assertRaises(ValueError) as ctx:
            corridor_blocked(
                ems, [], self.INNER, self.SMALL, opening_side="positive_x"
            )
        self.assertIn("unsupported opening_side", str(ctx.exception))

    def test_low_z_true_width_wall_blocks_without_nominal_span(self):
        """A low-Z wall covering the eroded hull width blocks a deep EMS.

        The wall does not reach +inner_w/2, so the legacy AABB-width test
        would miss it.
        """
        geom = _scene_floor_geometry()
        deep = (0.20, -0.40, 0.0, 0.65, 0.20, 0.25)
        wall = (-0.74, -0.985, 0.0, -0.30, 0.55, 0.30)
        self.assertTrue(
            corridor_blocked(
                deep, [wall], self.INNER, self.SMALL, geometry=geom
            )
        )
        self.assertFalse(
            corridor_blocked(
                deep, [wall], self.INNER, self.SMALL
            ),
            "cuboid fallback must not treat a 0.55-max-y wall as full width",
        )

    def test_wedge_only_broad_phase_overlap_is_not_an_obstacle(self):
        geom = _scene_floor_geometry()
        deep = (0.20, -0.40, 0.0, 0.65, 0.20, 0.25)
        wedge = (-0.70, 0.70, 0.0, -0.20, 0.98, 0.30)
        self.assertFalse(
            corridor_blocked(
                deep, [wedge], self.INNER, self.SMALL, geometry=geom
            )
        )

    def test_blocks_deep_space_uses_hull_volume(self):
        geom = _scene_floor_geometry()
        e = EMS(self.INNER, min_useful_edge=0.1)
        wall = (-0.74, -0.985, 0.0, -0.30, 0.55, 0.30)
        blocked, is_blocked = blocks_deep_space(
            wall,
            e,
            [],
            self.INNER,
            self.SMALL,
            v_min=self.SMALL[0] * self.SMALL[1] * self.SMALL[2],
            geometry=geom,
        )
        self.assertTrue(is_blocked)
        self.assertGreater(blocked, 0.0)
        wedge = (-0.70, 0.70, 0.0, -0.20, 0.98, 0.30)
        _, wedge_blocked = blocks_deep_space(
            wedge,
            e,
            [],
            self.INNER,
            self.SMALL,
            v_min=self.SMALL[0] * self.SMALL[1] * self.SMALL[2],
            geometry=geom,
            blocked_tol=0.5,
        )
        self.assertFalse(wedge_blocked)

    def test_empty_eroded_interval_blocks_without_boxes(self):
        """A payload that cannot occupy the carry Z fails closed with no ledger.

        The empty-interval check used to live inside the box loop, so an empty
        map returned False even when payload_center_y_interval inverted.
        """
        from luggage_description.container_geometry import (
            payload_center_y_interval,
        )
        geom = _scene_floor_geometry()
        too_wide = (0.40, 1.60, 0.25)
        deep = (0.20, -0.40, 0.0, 0.65, 0.20, 0.25)
        y_lo, y_hi = payload_center_y_interval(geom, too_wide, 0.0, 0.25)
        self.assertLess(y_hi, y_lo)
        self.assertTrue(
            corridor_blocked(
                deep, [], self.INNER, too_wide, geometry=geom
            )
        )
        self.assertFalse(
            corridor_blocked(
                deep, [], self.INNER, self.SMALL, geometry=geom
            )
        )


if __name__ == "__main__":
    unittest.main()
