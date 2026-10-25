#!/usr/bin/env python3
"""Snug ranking semantics tests (G3 §5).

A rewards equal-height snug neighbours with a gap-decayed shared-edge
weight: side-by-side on the same measured top scores, a gap past
adjacency_max scores zero, a different top height never scores, diagonal
neighbours and footprint overlap never score, and the weight is
perimeter-normalized. The order key is total and order-independent.
"""

import copy
import os
import random
import sys
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from luggage_packing.placement_snug_ranking import (  # noqa: E402
    adjacency_score, snug_rank)

TOP = 0.53 + 0.30  # candidate top at floor + support 0 + box height 0.60


def _candidate(x=0.0, y=0.0, top=TOP, layer=0.0, depth=1.0, score=0.5,
               cid="c000_000_f0_h0"):
    return {
        "candidate_id": cid,
        "center_container": [x, y, top - 0.30],
        "footprint": [0.44, 0.33],
        "size": [0.44, 0.33, 0.60],
        "support_layer_z": layer,
        "depth_m": depth,
        "score": score,
    }


def _neighbor(x, y, half_l=0.20, half_w=0.15, top=TOP):
    return {"cx": x, "cy": y, "half_l": half_l, "half_w": half_w,
            "yaw": 0.0, "top_z": top, "source": "map_plateau"}


class TestAdjacencyScore(unittest.TestCase):

    def test_equal_height_snug_is_rewarded(self):
        # Candidate y in [-0.815, -0.485]; neighbour y in [-0.47, -0.17]:
        # side by side with a 0.015 m gap on the measured top.
        cand = _candidate(y=-0.65)
        nb = _neighbor(0.0, -0.32, half_l=0.22, half_w=0.15)
        score = adjacency_score(cand, [nb])
        self.assertGreater(score, 0.0)

    def test_gap_decay_to_zero_at_max(self):
        cand = _candidate(y=-0.65)
        scores = []
        for gap in (0.005, 0.04, 0.079, 0.081):
            nb = _neighbor(0.0, -0.485 + gap + 0.15, half_l=0.22,
                           half_w=0.15)
            scores.append(adjacency_score(cand, [nb]))
        self.assertGreater(scores[0], scores[1])
        self.assertGreater(scores[1], scores[2])
        self.assertEqual(scores[3], 0.0)

    def test_different_top_never_scores(self):
        cand = _candidate(y=-0.65)
        nb = _neighbor(0.0, -0.32, top=TOP + 0.02)
        self.assertEqual(adjacency_score(cand, [nb]), 0.0)
        nb = _neighbor(0.0, -0.32, top=TOP - 0.02)
        self.assertEqual(adjacency_score(cand, [nb]), 0.0)

    def test_same_layer_z_but_different_top_no_reward(self):
        # Support layer equality alone is not adjacency: the measured
        # neighbour top must match the candidate's predicted top.
        cand = _candidate(y=-0.65, top=TOP)
        nb = _neighbor(0.0, -0.32, top=TOP + 0.05)
        self.assertEqual(adjacency_score(cand, [nb]), 0.0)

    def test_diagonal_and_overlap_never_score(self):
        cand = _candidate(y=-0.65)
        diagonal = _neighbor(0.60, -0.30)
        self.assertEqual(adjacency_score(cand, [diagonal]), 0.0)
        overlapping = _neighbor(0.0, -0.65)
        self.assertEqual(adjacency_score(cand, [overlapping]), 0.0)

    def test_perimeter_normalization(self):
        cand = _candidate(y=-0.65)
        nb = _neighbor(0.0, -0.32, half_l=0.22, half_w=0.15)
        base = adjacency_score(cand, [nb])
        self.assertLess(base, 1.0)
        two = adjacency_score(cand, [nb, dict(nb, cx=0.9)])
        # Second neighbour at cx=0.9 is too far to matter.
        self.assertAlmostEqual(two, base, places=9)


class TestRankingOrder(unittest.TestCase):

    def test_order_layer_adjacency_depth_score_id(self):
        # c001 is deeper but far from the neighbour; c002 is snug and
        # shallower. Same layer: adjacency outranks depth.
        low = _candidate(y=0.3, layer=0.0, depth=1.0, cid="c001")
        snug = _candidate(y=-0.65, layer=0.0, depth=0.5, cid="c002")
        nb = _neighbor(0.0, -0.32)
        ranked = snug_rank([low, snug], [nb])
        self.assertEqual(ranked[0]["candidate_id"], "c002")
        # Higher layer loses regardless of adjacency.
        high = _candidate(y=-0.65, layer=0.30, cid="c003")
        ranked = snug_rank([high, low], [nb])
        self.assertEqual(ranked[0]["candidate_id"], "c001",
                         "support layer outranks adjacency")
        # Same layer, same adjacency: deeper wins.
        far_snug = _candidate(y=-0.65, layer=0.0, depth=0.2, cid="c004")
        ranked = snug_rank([snug, far_snug], [nb])
        self.assertEqual(ranked[0]["candidate_id"], "c002")

    def test_total_key_deterministic_under_shuffle(self):
        nb = _neighbor(0.0, -0.32)
        cands = [
            _candidate(y=-0.65, depth=1.0 + 0.01 * i, cid="c%03d" % i)
            for i in range(20)
        ]
        rng = random.Random(7)
        reference = [c["candidate_id"] for c in snug_rank(cands, [nb])]
        for _ in range(10):
            shuffled = list(cands)
            rng.shuffle(shuffled)
            self.assertEqual(
                [c["candidate_id"] for c in snug_rank(shuffled, [nb])],
                reference)

    def test_no_neighbors_degrades_to_baseline_order(self):
        cands = [
            _candidate(depth=0.5, score=0.9, cid="a"),
            _candidate(depth=1.0, score=0.1, cid="b"),
        ]
        ranked = snug_rank(cands, [])
        self.assertEqual(ranked[0]["candidate_id"], "b")

    def test_input_candidates_not_mutated_order(self):
        nb = _neighbor(0.0, -0.32)
        cands = [_candidate(cid="a", depth=1.0), _candidate(cid="b",
                                                            depth=2.0)]
        snapshot = [c["candidate_id"] for c in cands]
        snug_rank(cands, [nb])
        self.assertEqual(snapshot, [c["candidate_id"] for c in cands])


if __name__ == "__main__":
    unittest.main()
