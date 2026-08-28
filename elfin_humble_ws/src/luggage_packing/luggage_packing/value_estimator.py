#!/usr/bin/env python3
"""Rollout value estimator V̂ + CEM proxy-weight calibration (P4, design §5.7/§6.1).

Pure Python + numpy, no ROS. ``value_hat`` estimates the future-space value
V̂(s') of a candidate via a K-step x M-sample distribution-sampled rollout with
a fast greedy inner policy (FreeSpaceModel candidates + LBCP, no IK -- §5.7).
By the §2.2 corollary (current item volume is constant w.r.t. placement), V̂ is
the *objective itself*, not a bonus.

``cem_calibrate`` is a lightweight cross-entropy-method harness (§6.1) that
optimizes the §5.7 proxy weights against the offline replay metric. It runs
pure-Python replays (no Gazebo) and is deterministic for a fixed seed.
"""

from __future__ import division

import copy

import numpy as np

from luggage_packing.ems import EMS, volume
from luggage_packing.insertion_corridor import proxy_score, blocks_deep_space


# --------------------------------------------------------------------------- #
# Distribution sampling (reuses the catalog weighted-sampling contract)
# --------------------------------------------------------------------------- #

def sample_item(entries, rng, size_range=None):
    """Sample a future box size [l,w,h].

    ``size_range`` mirrors the spawner's continuous sampling. Rolling out
    against the three catalog sizes while the robot actually receives arbitrary
    sizes would make the lookahead optimistic about how neatly future boxes fit.
    """
    if size_range is not None:
        return [
            float(rng.uniform(float(low), float(high)))
            for low, high in size_range
        ]
    total = sum(max(0.0, e["weight"]) for e in entries)
    if total <= 0.0:
        return list(entries[rng.randint(len(entries))]["size"])
    pick = rng.uniform(0.0, total)
    running = 0.0
    for e in entries:
        running += max(0.0, e["weight"])
        if pick <= running:
            return list(e["size"])
    return list(entries[-1]["size"])


def mean_item_volume(entries, size_range=None):
    """E[V_item] over the box distribution."""
    if size_range is not None:
        # Axes are independent uniforms, so E[V] is the product of the means.
        volume = 1.0
        for low, high in size_range:
            volume *= 0.5 * (float(low) + float(high))
        return volume
    total_w = sum(max(0.0, e["weight"]) for e in entries)
    if total_w <= 0.0:
        return 0.0
    acc = 0.0
    for e in entries:
        w = max(0.0, e["weight"])
        s = e["size"]
        acc += w * s[0] * s[1] * s[2]
    return acc / total_w


# --------------------------------------------------------------------------- #
# Rollout V̂ (§5.7)
# --------------------------------------------------------------------------- #

def _cand_box_floor(model, cand):
    """Candidate AABB in floor-relative coords (for EMS split)."""
    lx, ly = cand["center_local"][0], cand["center_local"][1]
    peak = cand.get("peak", 0.0)
    bl, bw, bh = cand["size"]
    lz_center = peak + bh * 0.5
    return (lx - bl * 0.5, ly - bw * 0.5, lz_center - bh * 0.5,
            lx + bl * 0.5, ly + bw * 0.5, lz_center + bh * 0.5)


def apply_place(model, ems, cand):
    """Non-mutating: return (model, ems) snapshots with ``cand`` placed."""
    m = model.snapshot()
    m.add_placed_box(cand["center_local"], cand["size"])
    e = ems.ems_after(_cand_box_floor(model, cand))
    return m, e


def greedy_inner(model, item_size):
    """Fast inner policy: best FreeSpaceModel candidate (observed-first, low-peak).

    No IK / no corridor check -- uses the vectorized candidate gate only, per
    the §5.7 rollout budget (single inner step ≈ 1-2 ms).
    """
    cands = model.candidates(item_size, allowed_yaws=[0.0, 1.5707963], top_n=10)
    if not cands:
        return None
    return cands[0]


def value_hat(cand, model, ems, entries, K=3, M=8, seed=0, size_range=None):
    """K-step x M-sample rollout V̂(s'), normalized by E[V_item] in [0, ~1]."""
    rng = np.random.RandomState(seed)
    base_m, base_e = apply_place(model, ems, cand)
    mean_v = mean_item_volume(entries, size_range) or 1.0
    if K <= 0 or M <= 0:
        return 0.0
    total = 0.0
    for _ in range(M):
        m = base_m.snapshot()
        e = copy.copy(base_e)
        e.spaces = list(base_e.spaces)
        for _ in range(K):
            item = sample_item(entries, rng, size_range)
            c = greedy_inner(m, item)
            if c is None:
                break
            m.add_placed_box(c["center_local"], c["size"])
            total += item[0] * item[1] * item[2]
    return total / (M * K * mean_v)


# --------------------------------------------------------------------------- #
# CEM proxy-weight calibration (§6.1)
# --------------------------------------------------------------------------- #

class ProxyStrategy(object):
    """Placement strategy that ranks FreeSpaceModel candidates by proxy_score.

    Used by ``cem_calibrate`` to evaluate a set of proxy weights on the offline
    replay. Falls back to the first candidate if scoring fails.
    """

    def __init__(self, model_factory, entries, inner_size, smallest_size,
                 weights=None, reachability_prior=0.5):
        self.model_factory = model_factory  # () -> fresh FreeSpaceModel
        self.entries = entries
        self.inner_size = inner_size
        self.smallest_size = smallest_size
        self.weights = weights or _DEFAULT_PROXY_WEIGHTS
        self.reachability_prior = reachability_prior

    def place(self, box_size, placed):
        """Return the best candidate dict (proxy_score-ranked) or None."""
        model = self.model_factory()
        ems = EMS(self.inner_size, min_useful_edge=0.1)
        for p in placed:
            model.add_placed_box(p["center_local"], p["size"])
            ems.place(_box_from_cand(p))
        cands = model.candidates(box_size, allowed_yaws=[0.0, 1.5707963], top_n=30)
        if not cands:
            return None
        best, best_score = None, -1e18
        for c in cands:
            score, _ = proxy_score(
                c, model, ems, self.inner_size, self.smallest_size,
                reachability_prior=self.reachability_prior)
            if score > best_score:
                best, best_score = c, score
        return best


def _box_from_cand(cand):
    lx, ly = cand["center_local"][0], cand["center_local"][1]
    peak = cand.get("peak", 0.0)
    bl, bw, bh = cand["size"]
    lz_center = peak + bh * 0.5
    return (lx - bl * 0.5, ly - bw * 0.5, lz_center - bh * 0.5,
            lx + bl * 0.5, ly + bw * 0.5, lz_center + bh * 0.5)


_DEFAULT_PROXY_WEIGHTS = {
    "ems_regularity": 0.35, "compactness": 0.20, "reachability": 0.15,
    "support_quality": 0.10, "insertion_clearance": 0.10,
    "observation_confidence": 0.10, "blocked_deep_ems": -0.30, "cog_height": -0.10,
}


def cem_calibrate(strategy_factory, sequences, pop=16, elites=4, iters=3, seed=0):
    """Cross-entropy-method search over proxy weights (§6.1).

    ``strategy_factory(weights)`` returns a strategy whose ``place(box, placed)``
    picks a candidate by those weights. ``sequences`` is a list of box sequences.
    Returns the best weight dict found and its mean placed-volume score.
    """
    rng = np.random.RandomState(seed)
    keys = list(_DEFAULT_PROXY_WEIGHTS.keys())
    mean = np.array([_DEFAULT_PROXY_WEIGHTS[k] for k in keys], dtype=np.float64)
    std = np.full(len(keys), 0.1, dtype=np.float64)
    best_w, best_score = dict(_DEFAULT_PROXY_WEIGHTS), -1e18

    def evaluate(weights):
        strat = strategy_factory({k: float(w) for k, w in zip(keys, weights)})
        scores = []
        for seq in sequences:
            placed = []
            for box in seq:
                c = strat.place(box, placed)
                if c is None:
                    break
                placed.append(c)
            v = sum(c["size"][0] * c["size"][1] * c["size"][2] for c in placed)
            scores.append(v)
        return float(np.mean(scores)) if scores else 0.0

    for _ in range(iters):
        samples = []
        for _ in range(pop):
            w = mean + std * rng.randn(len(keys))
            samples.append(w)
        scored = [(evaluate(w), w) for w in samples]
        scored.sort(key=lambda t: t[0], reverse=True)
        if scored[0][0] > best_score:
            best_score = scored[0][0]
            best_w = {k: float(w) for k, w in zip(keys, scored[0][1])}
        elite = np.array([w for _, w in scored[:elites]])
        mean = elite.mean(axis=0)
        std = elite.std(axis=0) + 1e-3
    return best_w, best_score
