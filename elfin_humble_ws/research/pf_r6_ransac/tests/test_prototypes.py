"""Focused prototype tests (PF-R6-RANSAC-RESEARCH).

Run: PYTHONPATH=research/pf_r6_ransac python3 -m pytest -q \
     research/pf_r6_ransac/tests/test_prototypes.py
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from luggage_perception.luggage_box_estimator import (  # noqa: E402
    _ransac_horizontal_plane)
from luggage_perception.top_support_estimator import (  # noqa: E402
    TopSupportConfig, TopSurfaceEstimate)
from pfr6bench import fixtures as fx  # noqa: E402
from pfr6bench import harness  # noqa: E402
from pfr6bench.methods import (  # noqa: E402
    COMPARATORS, baseline_ransac_plane, grid_seed_robust_z,
    one_point_constrained_ransac, zmode_median_plane)

CONFIG = TopSupportConfig()
WORKSPACE = ((0.0, 0.0), (0.5, 0.5))


def _flat(n=20000, z=0.0, sigma=0.002, seed=0):
    rng = np.random.RandomState(seed)
    pts = rng.uniform(-0.35, 0.35, (n, 2))
    zz = z + np.round(rng.normal(0, sigma, n) / 0.001) * 0.001
    return np.column_stack([pts, zz])


def _top(w=0.45, d=0.32, h=0.58):
    return TopSurfaceEstimate(
        center_xy=np.zeros(2), top_z=float(h), yaw=0.0,
        width=float(w), depth=float(d), confidence=1.0)


@pytest.mark.parametrize("name", sorted(COMPARATORS))
def test_flat_support_valid_near_gt(name):
    est = harness.run_support(
        _flat(), _top(), WORKSPACE, COMPARATORS[name], CONFIG)
    assert est is not None and est.reason == "ok"
    # grid_seed carries a known ~1.5-sigma low bias (min-z per cell);
    # 12 mm is well inside the plan's 15 mm p95 gate.
    tol = 0.012 if name == "grid_seed_robust_z" else 0.006
    assert abs(est.support_z) < tol


def test_baseline_is_verbatim_committed_helper():
    pts = _flat(n=5000, seed=3)
    m1, z1 = baseline_ransac_plane(pts, 200, 0.008, 0.15, 80)
    m2, z2 = _ransac_horizontal_plane(pts, 200, 0.008, 0.15, 80)
    assert z1 == z2
    assert np.array_equal(m1, m2)


@pytest.mark.parametrize("name", sorted(COMPARATORS))
def test_lower_dense_plane_does_not_steal_support(name):
    """A full-ring plane 5 cm below must not become the measured support."""
    support = _flat(n=20000, z=0.0, seed=1)
    lower = _flat(n=20000, z=-0.05, seed=2)
    pts = np.concatenate([support, lower])
    est = harness.run_support(
        pts, _top(), WORKSPACE, COMPARATORS[name], CONFIG)
    assert est.reason == "ok"
    tol = 0.012 if name == "grid_seed_robust_z" else 0.006
    assert abs(est.support_z) < tol


@pytest.mark.parametrize("name", sorted(COMPARATORS))
def test_insufficient_points_fail_closed(name):
    est = harness.run_support(
        _flat(n=40, seed=4), _top(), WORKSPACE, COMPARATORS[name], CONFIG)
    assert est.reason == "DETECT_SUPPORT_UNOBSERVABLE"


@pytest.mark.parametrize("name", sorted(COMPARATORS))
def test_flyers_only_fail_closed(name):
    rng = np.random.RandomState(5)
    n = 300
    xy = rng.uniform(-0.4, 0.4, (n, 2))
    z = 0.58 - rng.uniform(0.15, 0.60, n)  # inside band, no plane
    est = harness.run_support(
        np.column_stack([xy, z]), _top(), WORKSPACE,
        COMPARATORS[name], CONFIG)
    assert est.reason == "DETECT_SUPPORT_UNOBSERVABLE"
    assert not np.isfinite(est.support_z) or est.reason != "ok"


@pytest.mark.parametrize("name", sorted(COMPARATORS))
def test_single_side_clutter_fails_side_coverage(name):
    """Coherent wrong-height plane on one side only -> UNSTABLE reject."""
    rng = np.random.RandomState(6)
    u = rng.uniform(0.0, 0.35, 6000)
    v = rng.uniform(-0.05, 0.05, 6000)  # narrow strip near +u axis
    z = -0.08 + np.round(rng.normal(0, 0.002, len(u)) / 0.001) * 0.001
    est = harness.run_support(
        np.column_stack([u, v, z]), _top(0.36, 0.23, 0.50), WORKSPACE,
        COMPARATORS[name], CONFIG)
    # Either reject path is fail-closed: UNSTABLE (plane found, sides
    # insufficient) or UNOBSERVABLE (grid seeds too sparse to even fit).
    assert est.reason in ("DETECT_SUPPORT_UNSTABLE",
                          "DETECT_SUPPORT_UNOBSERVABLE")


def test_one_point_ransac_deterministic_per_seed():
    pts = np.concatenate([_flat(n=9000, seed=7),
                          _flat(n=9000, z=-0.05, seed=8)])
    outs = []
    for _ in range(3):
        m, z = one_point_constrained_ransac(
            pts, 200, 0.008, 0.15, 80, seed=42)
        outs.append((z, None if m is None else int(m.sum())))
    assert len(set(outs)) == 1


def test_zmode_and_grid_are_rng_free():
    pts = _flat(n=8000, seed=9)
    r1 = zmode_median_plane(pts, 200, 0.008, 0.15, 80)
    r2 = zmode_median_plane(pts, 200, 0.008, 0.15, 80)
    assert r1[1] == r2[1] and np.array_equal(r1[0], r2[0])
    g1 = grid_seed_robust_z(pts, 200, 0.008, 0.15, 80)
    g2 = grid_seed_robust_z(pts, 200, 0.008, 0.15, 80)
    assert g1[1] == g2[1] and np.array_equal(g1[0], g2[0])


@pytest.mark.parametrize("name", ["zmode_median", "grid_seed_robust_z",
                                  "one_point_constrained_ransac"])
def test_comparators_much_faster_than_baseline(name):
    pts = _flat(n=31000, seed=10)
    t_base = harness.time_fitter(
        COMPARATORS["baseline_ransac"], pts, repeats=5, warmup=1)
    t_cand = harness.time_fitter(
        COMPARATORS[name], pts, repeats=10, warmup=2)
    assert t_cand["p95_ms"] * 2.0 <= t_base["p50_ms"]


def test_fixture_manifest_is_deterministic():
    f1 = fx.build_all_fixtures(CONFIG)
    f2 = fx.build_all_fixtures(CONFIG)
    ids1 = [(f["id"], f["candidate_count"], f["candidates_sha256"])
            for f in f1]
    ids2 = [(f["id"], f["candidate_count"], f["candidates_sha256"])
            for f in f2]
    assert ids1 == ids2
    assert len(f1) >= 18


@pytest.mark.parametrize("name", sorted(COMPARATORS))
def test_nonfinite_rows_do_not_crash(name):
    pts = _flat(n=9000, seed=11)
    pts[::1000, 2] = np.nan
    pts[7, 0] = np.inf
    est = harness.run_support(
        pts, _top(), WORKSPACE, COMPARATORS[name], CONFIG)
    assert est.reason in ("ok", "DETECT_SUPPORT_UNSTABLE",
                          "DETECT_SUPPORT_UNOBSERVABLE")
    if est.reason == "ok":
        assert abs(est.support_z) <= 0.008  # one dist-threshold quantum
