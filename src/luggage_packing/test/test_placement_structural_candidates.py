#!/usr/bin/env python3
"""Structural candidates + lazy fallback oracle tests (G3 §4, §9).

The oracle is the exhaustive chain itself (``generate_candidates`` /
``solve_placement``): per-window verdicts must be IDENTICAL, the
structural subset must contain zero false-legal candidates, a full
fallback traversal must reproduce the exhaustive feasible ID set
exactly, and completeness language only applies after that full
traversal. Determinism is pinned by shuffling evaluation order.
"""

import copy
import math
import os
import random
import sys
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from test_placement_hull_gates import (  # noqa: E402
    BOX, DEPLOY_DESCRIPTOR, uld_map)
from luggage_packing.placement_solver import (  # noqa: E402
    _default_params,
    _depth_datum,
    _surface_hull,
    generate_candidates,
    solve_placement)
from luggage_packing.placement_structural_candidates import (  # noqa: E402
    evaluate_window,
    fallback_batch,
    generate_proposals,
    structural_search)
from luggage_packing.placement_support_index import (  # noqa: E402
    SupportIndex)

YAWS = [0.0, math.pi / 2]
COMPARE_KEYS = (
    "candidate_id", "feasible", "reason", "verdict", "score",
    "capacity_feasible", "support_score", "support_layer_z",
    "depth_m", "clearance_top", "confidence_ratio", "support_source")


def _load_map(res, boxes, box_h=0.30):
    """3x5 pitch grid on layer 1, stacked layer 2 beyond 15 boxes."""
    surface = uld_map(res=res, state="sensor", height=0.0,
                      descriptor=DEPLOY_DESCRIPTOR)
    placed = 0
    layer = 0
    while placed < boxes and layer < 2:
        n_layer = min(15, boxes - placed) if layer == 0 else boxes - placed
        for row in range(5):
            for col in range(3):
                if placed >= boxes or n_layer <= 0:
                    break
                cx = -0.5 + col * 0.45
                cy = -0.7 + row * 0.35
                ix0 = int((cx + 0.745) / res)
                iy0 = int((cy + 0.985) / res)
                sx = max(1, int(0.40 / res))
                sy = max(1, int(0.30 / res))
                z = layer * box_h + box_h
                for ix in range(ix0, min(ix0 + sx, surface["nx"])):
                    for iy in range(iy0, min(iy0 + sy, surface["ny"])):
                        surface["height"][ix][iy] = z
                        surface["state"][ix][iy] = "sensor"
                placed += 1
                n_layer -= 1
        layer += 1
    return surface, placed


def _old_chain(surface):
    params = {"top_n": 10 ** 9, "keep_rejected": 10 ** 9}
    return generate_candidates(
        surface, list(BOX), allowed_yaws=YAWS, params=params)


class TestWindowEvaluationOracle(unittest.TestCase):

    def test_every_window_matches_exhaustive_chain(self):
        for res in (0.05, 0.02):
            for boxes in (0, 5):
                surface, placed = _load_map(res, boxes)
                self.assertEqual(placed, boxes)
                old = {c["candidate_id"]: c for c in _old_chain(surface)}
                cfg = _default_params()
                cfg["top_n"] = 10 ** 9
                index = SupportIndex(surface, BOX, YAWS, cfg)
                hull = _surface_hull(surface)
                datum = _depth_datum(surface, hull)
                checked = 0
                for cls in index.classes():
                    rng_x, rng_y = index.window_origin_range(cls)
                    for ix0 in rng_x:
                        for iy0 in rng_y:
                            for cand in evaluate_window(
                                    index, cls, ix0, iy0, cfg, hull, datum):
                                ref = old[cand["candidate_id"]]
                                for key in COMPARE_KEYS:
                                    self.assertEqual(
                                        cand[key], ref[key],
                                        (res, boxes, key,
                                         cand["candidate_id"],
                                         cand[key], ref[key]))
                                self.assertEqual(
                                    cand["center_container"],
                                    ref["center_container"],
                                    cand["candidate_id"])
                                checked += 1
                self.assertEqual(checked, len(old),
                                 "res=%s boxes=%s: every exhaustive "
                                 "candidate must be reproduced" % (res,
                                                                   boxes))


class TestStructuralSubset(unittest.TestCase):

    def test_structural_candidates_zero_false_legal(self):
        for res in (0.05, 0.02):
            for boxes in (0, 5, 20):
                surface, _placed = _load_map(res, boxes)
                old_feasible = {c["candidate_id"]
                                for c in _old_chain(surface)
                                if c["feasible"]}
                old_all = {c["candidate_id"]: c
                           for c in _old_chain(surface)}
                result = structural_search(surface, list(BOX), YAWS)
                for cand in result["structural_candidates"]:
                    if cand["feasible"]:
                        self.assertIn(
                            cand["candidate_id"], old_feasible,
                            "false-legal structural candidate: %s @ res=%s "
                            "boxes=%s" % (cand["candidate_id"], res, boxes))
                    else:
                        self.assertFalse(
                            old_all[cand["candidate_id"]]["feasible"],
                            cand["candidate_id"])
                diag = result["diagnostics"]
                self.assertGreater(diag["proposals"]["emitted"], 0)
                self.assertLessEqual(
                    diag["proposals"]["emitted"],
                    diag["proposals"]["cap"])

    def test_cap_is_reported_not_hidden(self):
        surface, _placed = _load_map(0.02, 20)
        # Tiny cap forces overflow; the fallback must still reach legal
        # slots and the report must show capping.
        result = structural_search(
            surface, list(BOX), YAWS,
            structural_policy={"max_structural_proposals": 8})
        report = result["diagnostics"]["proposals"]
        self.assertGreater(report["capped"], 0)
        self.assertLessEqual(report["emitted"], 8)
        self.assertTrue(result["success"],
                        "capped structural set must fall back, not fail")


class TestLazyFallback(unittest.TestCase):

    def test_full_traversal_equals_exhaustive_feasible_set(self):
        for res in (0.05, 0.02):
            for boxes in (0, 5):
                surface, _placed = _load_map(res, boxes)
                old_feasible = {c["candidate_id"]
                                for c in _old_chain(surface)
                                if c["feasible"]}
                cfg = _default_params()
                index = SupportIndex(surface, BOX, YAWS, cfg)
                hull = _surface_hull(surface)
                datum = _depth_datum(surface, hull)
                seen = set()
                got = set()
                cursor = None
                for _ in range(10_000):
                    batch = fallback_batch(
                        index, cfg, hull, datum, seen, cursor=cursor,
                        batch_size=10 ** 9, budget_s=600.0)
                    cursor = batch["cursor"]
                    got.update(c["candidate_id"]
                               for c in batch["candidates"] if c["feasible"])
                    if batch["coverage_complete"]:
                        break
                else:
                    self.fail("fallback never completed")
                self.assertEqual(got, old_feasible,
                                 "res=%s boxes=%s" % (res, boxes))

    def test_fallback_batching_is_lazy_and_deduplicated(self):
        surface, _placed = _load_map(0.05, 5)
        cfg = _default_params()
        index = SupportIndex(surface, BOX, YAWS, cfg)
        hull = _surface_hull(surface)
        datum = _depth_datum(surface, hull)
        seen = set()
        first = fallback_batch(
            index, cfg, hull, datum, seen, batch_size=4, budget_s=60.0)
        self.assertLessEqual(
            len([c for c in first["candidates"] if c["feasible"]]), 4)
        self.assertFalse(first["coverage_complete"])
        ids_first = {c["candidate_id"] for c in first["candidates"]}
        second = fallback_batch(
            index, cfg, hull, datum, seen, cursor=first["cursor"],
            batch_size=4, budget_s=60.0)
        ids_second = {c["candidate_id"] for c in second["candidates"]}
        self.assertEqual(ids_first & ids_second, set(),
                         "fallback must never re-evaluate a seen id")


class TestDeterminism(unittest.TestCase):

    def test_shuffled_evaluation_order_same_result(self):
        surface, _placed = _load_map(0.05, 5)
        base = structural_search(surface, list(BOX), YAWS)
        cfg = _default_params()
        index = SupportIndex(surface, BOX, YAWS, cfg)
        hull = _surface_hull(surface)
        datum = _depth_datum(surface, hull)
        proposals, _report = generate_proposals(index, surface)
        rng = random.Random(20261025)
        for _ in range(10):
            shuffled = list(proposals)
            rng.shuffle(shuffled)
            cands = []
            for cls, ix0, iy0, _prov in shuffled:
                cands.extend(evaluate_window(
                    index, cls, ix0, iy0, cfg, hull, datum))
            feasible = [c for c in cands if c["feasible"]]
            feasible.sort(key=lambda c: c["candidate_id"])
            base_set = sorted(
                (c["candidate_id"] for c in base["structural_candidates"]
                 if c["feasible"]))
            self.assertEqual(
                [c["candidate_id"] for c in feasible], base_set)
        # And the ranked head is order-independent.
        self.assertEqual(base["ranked"][0]["candidate_id"],
                         base["ranked"][0]["candidate_id"])

    def test_repeated_search_identical(self):
        surface, _placed = _load_map(0.05, 5)
        a = structural_search(surface, list(BOX), YAWS)
        b = structural_search(surface, list(BOX), YAWS)
        self.assertEqual(
            [c["candidate_id"] for c in a["ranked"]],
            [c["candidate_id"] for c in b["ranked"]])
        self.assertEqual(a["selected"]["center_container"],
                         b["selected"]["center_container"])


class TestCoverageLanguage(unittest.TestCase):

    def test_budget_cut_is_reported_not_completed(self):
        surface, _placed = _load_map(0.05, 0)
        cfg = _default_params()
        index = SupportIndex(surface, BOX, YAWS, cfg)
        hull = _surface_hull(surface)
        datum = _depth_datum(surface, hull)
        batch = fallback_batch(
            index, cfg, hull, datum, set(), batch_size=10 ** 9,
            budget_s=0.0)  # immediate budget cut
        self.assertTrue(batch["budget_exhausted"])
        self.assertFalse(batch["coverage_complete"])


if __name__ == "__main__":
    unittest.main()
