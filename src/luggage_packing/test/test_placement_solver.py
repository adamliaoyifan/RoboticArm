#!/usr/bin/env python3
"""Unit tests for placement_solver floor-prior, stacking, and flatten-first.

Covers design §4.2.2 and docs/architecture/placement.md:

  - unobserved columns at ``peak ≈ floor_z`` are ALLOWED (``support_source =
    floor_prior``) -- empty container must yield floor-spanning candidates;
  - stacking on an unobserved surface (``peak > 0`` with unknown in the
    footprint) is REJECTED as ``unknown_above_floor`` -- never blind-stack;
  - fully-observed stacking is feasible; live score still prefers a remaining
    floor slot over a low stack.
"""

import os
import sys
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from luggage_packing.placement_solver import (  # noqa: E402
    generate_candidates,
    placement_constraint_verdict,
    ranked_feasible_candidates,
    solve_placement,
)


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


KEEP = {"top_n": 400, "keep_rejected": 80}


class TestLiveFlattenFirstAndCommitStack(unittest.TestCase):
    """Live Humble solver: flatten-first ranking and commit-surface stacking.

    Codifies docs/architecture/placement.md: weights do not change after a
    commit; a known occupied top becomes a stack candidate; as long as a
    feasible floor slot remains, score still picks the floor.
    """

    BOX = [0.7, 0.45, 0.28]
    INNER_H = 1.5
    COMMIT_H = 0.28

    def _committed_corner_map(self):
        sm = _make_map(state="free", height=0.0, confidence="sensor")
        # 0.8 m x 0.6 m occupied patch: fully supports BOX (0.7 x 0.45).
        _set_region(
            sm, range(0, 8), range(0, 6),
            state="occupied", height=self.COMMIT_H, confidence="geometry")
        return sm

    def _committed_aabb(self):
        half_l, half_w, res = 1.0, 0.8, 0.1
        return (
            -half_l, -half_w, 0.0,
            -half_l + 8 * res, -half_w + 6 * res, self.COMMIT_H,
        )

    def _validator(self, placed_aabbs):
        def _validate(candidate):
            return placement_constraint_verdict(
                candidate, self.INNER_H, placed_aabbs=placed_aabbs)
        return _validate

    def test_floor_outranks_a_low_stack(self):
        sm = self._committed_corner_map()
        result = solve_placement(
            sm, self.BOX, allowed_yaws=[0.0], params=KEEP,
            candidate_validator=self._validator([self._committed_aabb()]))
        self.assertTrue(result["success"], result["message"])
        feasible = [c for c in result["candidates"] if c["feasible"]]
        peaks = [_peak(c, self.INNER_H, self.BOX[2]) for c in feasible]
        self.assertTrue(any(p <= 0.05 for p in peaks), "need a floor slot")
        self.assertTrue(any(p > 0.05 for p in peaks), "need a stack slot")
        selected_peak = _peak(result["selected"], self.INNER_H, self.BOX[2])
        self.assertLessEqual(
            selected_peak, 0.05,
            "live score must flatten-first while a floor slot remains")

    def test_stack_wins_once_the_floor_is_occupied(self):
        sm = _make_map(
            state="occupied", height=self.COMMIT_H, confidence="geometry")
        result = solve_placement(
            sm, self.BOX, allowed_yaws=[0.0], params=KEEP)
        self.assertTrue(result["success"], result["message"])
        selected_peak = _peak(result["selected"], self.INNER_H, self.BOX[2])
        self.assertGreater(selected_peak, 0.05)
        self.assertEqual(result["selected"]["reason"], "ok")

    def test_placed_aabb_blocks_floor_through_box_when_surface_stale(self):
        """Heights come from surface_2d; overlap comes from request.placed.

        If the map is still empty while placed AABBs already contain the box,
        floor windows through that volume must be overlap-rejected. A matching
        committed height map instead lifts those windows onto the top (no
        positive-volume overlap).
        """
        sm = _make_map(state="free", height=0.0, confidence="sensor")
        result = solve_placement(
            sm, self.BOX, allowed_yaws=[0.0], params=KEEP,
            candidate_validator=self._validator([self._committed_aabb()]))
        rejected_overlap = [
            c for c in result["candidates"] if c["reason"] == "overlap"]
        self.assertTrue(
            rejected_overlap,
            "stale floor windows through request.placed must be overlap")
        for cand in rejected_overlap:
            self.assertFalse(cand["feasible"])
            self.assertFalse(cand["capacity_feasible"])
            self.assertLessEqual(_peak(cand, self.INNER_H, self.BOX[2]), 0.05)


class TestRankedFeasibleCandidates(unittest.TestCase):
    def test_topk_matches_selected(self):
        sm = _make_map(state="unknown", height=0.0, confidence="none")
        result = solve_placement(
            sm, [0.7, 0.45, 0.28], allowed_yaws=[0.0],
            params={"top_n": 20, "keep_rejected": 5})
        self.assertTrue(result["success"])
        top = ranked_feasible_candidates(result, 5)
        self.assertGreaterEqual(len(top), 2)
        self.assertLessEqual(len(top), 5)
        self.assertEqual(top[0]["center_base"], result["selected"]["center_base"])
        self.assertAlmostEqual(top[0]["score"], result["selected"]["score"])


if __name__ == "__main__":
    unittest.main()
