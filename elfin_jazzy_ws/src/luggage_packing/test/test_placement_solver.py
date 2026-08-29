#!/usr/bin/env python3
"""Unit tests for placement_solver floor-prior gate (no roscore, no ROS deps).

Covers design §4.2.2: the container floor's *existence* is geometric prior
(scene_tf), not a perception claim. So:

  - unobserved columns at ``peak ≈ floor_z`` are ALLOWED (``support_source =
    floor_prior``) -- empty container must yield floor-spanning candidates;
  - stacking on an unobserved surface (``peak > 0`` with unknown in the
    footprint) is REJECTED as ``unknown_above_floor`` -- never blind-stack;
  - fully-observed floor / observed stacking are unaffected (no regression).
"""

import os
import sys
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from luggage_packing.placement_solver import generate_candidates  # noqa: E402


def _make_map(nx=20, ny=16, inner=(2.0, 1.6, 1.5), res=0.1,
              center_base=(2.0, 0.0, 0.75), yaw=0.0,
              state="unknown", height=0.0, confidence="none", floor_z=0.0):
    """Build a uniform surface_map_2d matching the cargo_volume_mapper contract."""
    return {
        "resolution": res,
        "nx": nx,
        "ny": ny,
        "inner_size": list(inner),
        "floor_z": floor_z,
        "center_base": list(center_base),
        "yaw": yaw,
        "height": [[height] * ny for _ in range(nx)],
        "state": [[state] * ny for _ in range(nx)],
        "confidence": [[confidence] * ny for _ in range(nx)],
    }


def _set_region(sm, ix_range, iy_range, state=None, height=None, confidence=None):
    for ix in ix_range:
        for iy in iy_range:
            if state is not None:
                sm["state"][ix][iy] = state
            if height is not None:
                sm["height"][ix][iy] = height
            if confidence is not None:
                sm["confidence"][ix][iy] = confidence


def _peak(cand, inner_h, box_h):
    """Recover the landing height (peak) from a candidate's center_local[2]."""
    half_h = inner_h * 0.5
    return cand["center_local"][2] + half_h - box_h * 0.5


class TestFloorPriorGate(unittest.TestCase):
    BOX = [0.7, 0.45, 0.28]  # [length, width, height]
    INNER_H = 1.5

    def test_empty_container_floor_prior(self):
        """All-unknown empty container -> feasible floor_prior candidates spanning the floor."""
        sm = _make_map(state="unknown", height=0.0, confidence="none")
        cands = generate_candidates(sm, self.BOX, allowed_yaws=[0.0])
        feasible = [c for c in cands if c["feasible"]]
        self.assertGreater(
            len(feasible), 1,
            "empty container must yield >1 feasible floor_prior candidate, not 0")
        for c in feasible:
            self.assertEqual(c["support_source"], "floor_prior")
            self.assertEqual(c["reason"], "ok")
        # Candidates must span multiple floor positions, not collapse to one corner.
        positions = {
            (round(c["center_local"][0], 3), round(c["center_local"][1], 3))
            for c in feasible
        }
        self.assertGreater(
            len(positions), 1,
            "floor_prior candidates must cover multiple floor positions, not one corner")

    def test_slab_top_floor_prior_uses_absolute_elevation(self):
        """E12: a box lands on slab Z=0.53, not container-link Z=0."""
        box = [0.70, 0.45, 0.28]
        inner_h = 2.01
        floor_z = 0.53
        sm = _make_map(
            inner=(2.0, 1.6, inner_h),
            center_base=(0.0, 0.0, inner_h * 0.5),
            state="unknown",
            height=floor_z,
            confidence="none",
            floor_z=floor_z,
        )
        feasible = [
            cand for cand in generate_candidates(
                sm, box, allowed_yaws=[0.0])
            if cand["feasible"]
        ]
        self.assertGreater(len(feasible), 0)
        candidate = feasible[0]
        self.assertEqual(candidate["support_source"], "floor_prior")
        center_z = candidate["center_base"][2]
        contact_z = center_z + box[2] * 0.5
        self.assertAlmostEqual(center_z, floor_z + box[2] * 0.5, places=6)
        self.assertAlmostEqual(contact_z, floor_z + box[2], places=6)

    def test_unknown_above_floor_rejected(self):
        """Footprint spanning a known obstacle + unknown floor (peak>0, has_unknown) -> rejected."""
        sm = _make_map(state="unknown", height=0.0)
        # A known placed box in the -X/-Y corner.
        _set_region(sm, range(0, 6), range(0, 6),
                    state="occupied", height=0.3, confidence="geometry")
        cands = generate_candidates(sm, self.BOX, allowed_yaws=[0.0])
        straddlers = [c for c in cands if c["reason"] == "unknown_above_floor"]
        self.assertTrue(
            straddlers,
            "a footprint spanning occupied+unknown must be rejected as unknown_above_floor")
        for c in straddlers:
            self.assertFalse(c["feasible"])

    def test_observed_floor_free(self):
        """Fully-observed free floor (state=free) -> feasible, not floor_prior (no regression)."""
        sm = _make_map(state="free", height=0.0, confidence="sensor")
        cands = generate_candidates(sm, self.BOX, allowed_yaws=[0.0])
        feasible = [c for c in cands if c["feasible"]]
        self.assertGreater(len(feasible), 0)
        for c in feasible:
            self.assertNotEqual(c["support_source"], "floor_prior")
            self.assertEqual(c["reason"], "ok")

    def test_normal_stacking(self):
        """Stacking on a fully-observed placed box (no unknown in footprint) -> feasible."""
        # The whole floor is a known placed box (occupied, height 0.3); stacking
        # on top must be feasible, not rejected as unknown_above_floor.
        sm = _make_map(state="occupied", height=0.3, confidence="geometry")
        cands = generate_candidates(sm, self.BOX, allowed_yaws=[0.0])
        box_h = self.BOX[2]
        stacking = [
            c for c in cands
            if c["feasible"] and _peak(c, self.INNER_H, box_h) > 0.05
        ]
        self.assertGreater(
            len(stacking), 0,
            "stacking on a fully-observed box must be feasible (no unknown_above_floor)")
        for c in stacking:
            self.assertEqual(c["reason"], "ok")


if __name__ == "__main__":
    unittest.main()
