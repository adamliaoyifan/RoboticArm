#!/usr/bin/env python3
"""Payload semantics + search session tests (G3 integration items 1-2).

Item 1: hard gates consume the CONSERVATIVE envelope; same-height
adjacency consumes the PREDICTED NOMINAL top (``StructuralPayload``),
so a downward-grown envelope beside an equal measured top is rewarded
exactly once -- via the nominal object, never the envelope object.

Item 2: when every route attempt on the legal structural batch fails,
``next_batch`` continues -- structural remainder, then grid fallback --
and a controlled route oracle that rejects all structural positions but
accepts one grid-only position must receive it; exhaustion and budget
cuts are terminal STATES, never a "container full" conclusion.
"""

import math
import os
import sys
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.normpath(os.path.join(PKG_ROOT, "..", "..")))

from test_placement_hull_gates import (  # noqa: E402
    BOX, DEPLOY_DESCRIPTOR, uld_map)
from test_placement_structural_candidates import _load_map, _old_chain  # noqa: E402
from luggage_packing.placement_solver import generate_candidates  # noqa: E402
from luggage_packing.placement_structural_candidates import (  # noqa: E402
    structural_search)
from luggage_packing.placement_structural_payload import (  # noqa: E402
    StructuralPayload)
from luggage_packing.placement_search_session import (  # noqa: E402
    PlacementSearchSession, SearchSessionExpired)
from luggage_packing.placement_snug_ranking import (  # noqa: E402
    snug_rank, adjacency_score)

YAWS = [0.0, math.pi / 2]
FLOOR_Z = 0.53


class TestStructuralPayload(unittest.TestCase):

    def test_from_sizes_zero_offset(self):
        payload = StructuralPayload.from_sizes((0.44, 0.33, 0.60),
                                               (0.40, 0.30, 0.56), "d1")
        top = payload.predicted_nominal_top([0.0, 0.0, FLOOR_Z + 0.30])
        self.assertAlmostEqual(top, FLOOR_Z + 0.30 + 0.28, places=9)

    def test_from_attachment_away_growth_shifts_nominal_object(self):
        from luggage_planning.planning_contract.attachment_geometry import freeze_geometry
        from luggage_planning.planning_contract.payload_envelope import EnvelopeLimits
        limits = EnvelopeLimits(
            yaw_half_range_deg=0.0,
            size_growth_fraction_wdh=(0.0, 0.0, 0.1),
            position_half_range_tool_xy_m=(0.0, 0.0),
            away_from_contact_position_error_m=0.0)
        # Bottom face at the suction origin: the envelope grows AWAY
        # from the contact face (upward). The envelope top then sits
        # above the measured box top -- the exact ranking-object
        # mismatch the payload exists to prevent.
        geometry = freeze_geometry(
            (0.40, 0.30, 0.60), (0.0, 0.0, 0.30), (0.0, 0.0, 0.0, 1.0),
            limits)
        payload = StructuralPayload.from_attachment(geometry)
        self.assertAlmostEqual(payload.conservative_size_wdh[2],
                               0.60 * 1.1, places=9)
        self.assertLess(payload.nominal_center_offset[2], 0.0)
        env_center_z = FLOOR_Z + 0.33  # envelope resting on the floor
        top = payload.predicted_nominal_top([0.0, 0.0, env_center_z])
        self.assertAlmostEqual(top, FLOOR_Z + 0.60, places=6,
                               msg="nominal top must be the measured "
                                   "box top, not the envelope top")
        self.assertAlmostEqual(env_center_z + 0.33, FLOOR_Z + 0.66,
                               places=6)

    def test_adjacency_uses_nominal_top_not_envelope_top(self):
        # Neighbour measured top at floor + 0.60 (the nominal box top of
        # an away-grown envelope candidate). The ENVELOPE top is floor +
        # 0.66: beyond the 0.01 tolerance, so the legacy object never
        # matches; the predicted NOMINAL top matches exactly.
        payload = StructuralPayload(
            (0.44, 0.33, 0.66), (0.44, 0.33, 0.60),
            (0.0, 0.0, -0.03), "d2")
        candidate = {
            "candidate_id": "c000",
            "center_container": [0.0, -0.65, FLOOR_Z + 0.33],
            "footprint": [0.44, 0.33],
            "size": [0.44, 0.33, 0.66],
        }
        neighbor = {"cx": 0.0, "cy": -0.32, "half_l": 0.22,
                    "half_w": 0.15, "yaw": 0.0, "top_z": FLOOR_Z + 0.60,
                    "source": "map_plateau"}
        self.assertEqual(
            adjacency_score(candidate, [neighbor]), 0.0,
            "envelope-top object must NOT match an equal nominal top")
        self.assertGreater(
            adjacency_score(candidate, [neighbor], payload=payload), 0.0,
            "equal measured tops must score via the nominal object")

    def test_search_rejects_size_payload_mismatch(self):
        surface, _placed = _load_map(0.05, 0)
        payload = StructuralPayload.from_sizes((0.44, 0.33, 0.66),
                                               (0.44, 0.33, 0.60), "d3")
        with self.assertRaises(ValueError):
            structural_search(surface, list(BOX), YAWS, payload=payload)

    def test_gates_keep_consuming_the_envelope(self):
        # Same map, envelope 0.66 vs 0.60 height near the ceiling is a
        # gate question, not a ranking question: the taller envelope
        # loses slots the shorter one keeps, payload or not.
        surface, _placed = _load_map(0.05, 5)
        tight = {"top_n": 10 ** 9, "keep_rejected": 0,
                 "clearance_margin": 0.03}
        short = generate_candidates(
            surface, [0.44, 0.33, 0.60], allowed_yaws=YAWS, params=tight)
        tall = generate_candidates(
            surface, [0.44, 0.33, 0.66], allowed_yaws=YAWS, params=tight)
        n_short = sum(1 for c in short if c["feasible"])
        n_tall = sum(1 for c in tall if c["feasible"])
        self.assertGreaterEqual(n_short, n_tall)


class TestSearchSession(unittest.TestCase):

    def _session(self, surface, budget_s=60.0, batch_size=8, **kwargs):
        return PlacementSearchSession(
            surface, list(BOX), YAWS, structural_policy=None,
            budget_s=budget_s, batch_size=batch_size, **kwargs)

    def test_first_batch_is_structural_and_ranked(self):
        surface, _placed = _load_map(0.05, 5)
        session = self._session(surface)
        batch = session.start()
        self.assertEqual(batch["source"], "structural")
        self.assertGreater(len(batch["candidates"]), 0)
        a_values = [c.get("adjacency_a") for c in batch["candidates"]]
        self.assertTrue(all(v is not None for v in a_values))

    def test_no_candidate_served_twice(self):
        surface, _placed = _load_map(0.05, 5)
        session = self._session(surface, batch_size=16)
        seen = []
        batch = session.start()
        while batch["candidates"]:
            seen.extend(c["candidate_id"] for c in batch["candidates"])
            if batch["coverage_complete"] or len(seen) > 4000:
                break
            batch = session.next_batch()
        self.assertEqual(len(seen), len(set(seen)),
                         "a candidate id must be served at most once")

    def test_route_failure_continuation_reaches_grid_fallback(self):
        """Controlled route oracle: every structural position fails its
        route, exactly one grid-only position is reachable. The session
        must continue past the structural batch and return that
        position -- never conclude 'full' while it is unseen."""
        surface, _placed = _load_map(0.05, 0)
        reference = structural_search(surface, list(BOX), YAWS)
        structural_ids = {
            c["candidate_id"] for c in reference["structural_candidates"]
            if c["feasible"]}
        old_feasible = {c["candidate_id"] for c in _old_chain(surface)
                        if c["feasible"]}
        grid_only = sorted(old_feasible - structural_ids)
        self.assertGreater(len(grid_only), 0,
                           "need grid-only legal positions for the oracle")
        accepted = {grid_only[0], grid_only[len(grid_only) // 2]}

        def route_oracle(candidate):
            # Routes fail for every structural position and for every
            # grid position except the accepted pair.
            return candidate["candidate_id"] in accepted

        session = self._session(surface, batch_size=8, budget_s=120.0)
        batch = session.start()
        served = []
        placed = None
        for _ in range(200):
            for candidate in batch["candidates"]:
                served.append(candidate["candidate_id"])
                if route_oracle(candidate):
                    placed = candidate["candidate_id"]
                    break
            if placed is not None:
                break
            if batch["coverage_complete"]:
                break
            batch = session.next_batch()
        self.assertIsNotNone(
            placed,
            "session must continue into grid fallback and serve the "
            "reachable grid-only position")
        self.assertIn(placed, accepted)
        self.assertIn(placed, old_feasible)
        # Continuation actually happened: structural batches came first,
        # then at least one grid batch.
        self.assertGreater(session._batches_served, 1)

    def test_budget_expiry_is_a_state_not_full(self):
        surface, _placed = _load_map(0.05, 0)
        session = self._session(surface, budget_s=0.0)
        with self.assertRaises(SearchSessionExpired):
            session.start()

    def test_session_identity_binds_inputs(self):
        surface, _placed = _load_map(0.05, 5)
        payload = StructuralPayload.from_sizes(BOX, BOX, "digest-x")
        session = self._session(surface, payload=payload)
        identity = session.identity()
        self.assertEqual(identity["attachment_digest"], "digest-x")
        self.assertTrue(identity["map_geometry_hash"])


if __name__ == "__main__":
    unittest.main()
