"""ST09-D acceptance: the eight map negative/coverage cases, sim/deploy.

Every case runs against both scene identities (SIM_SCENE / DEPLOY_SCENE
fixture snapshots): empty view, missing depth, TF stale, duplicate view,
hash conflict, duplicate commit, partial coverage, coverage sufficient.
Missing sensors never promote cells to free or support.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from luggage_perception.cargo_view_integration import (
    SnapshotViewIntegrator,
    integrate_view_with_report,
)

from st09_contracts._base import ContractViolation
from st09_contracts.fixtures import DEPLOY_SCENE, SIM_SCENE, make_snapshot
from st09_contracts.planning import ObservationRequirement
from st09_contracts.reasons import Reason

SCENES = {"sim": SIM_SCENE, "deploy": DEPLOY_SCENE}
GEOMETRY_HASH = make_snapshot(map_revision=1).geometry_hash

# A small requirement over the deep half of the fixture grid.
REQUIREMENT_CELLS_FOOTPRINT = tuple(
    (i, j) for i in (14, 15) for j in (5, 6))
REQUIREMENT_CELLS_CORRIDOR = tuple(
    (i, 5) for i in range(0, 14))


def requirement(candidate_ids=("cand-fixture-1",)):
    return ObservationRequirement(
        requirement_id="req-view-1",
        candidate_ids=candidate_ids,
        footprint_cells=REQUIREMENT_CELLS_FOOTPRINT,
        corridor_cells=REQUIREMENT_CELLS_CORRIDOR,
        missing_cells=REQUIREMENT_CELLS_FOOTPRINT
        + REQUIREMENT_CELLS_CORRIDOR,
        missing_reasons=tuple(
            "unknown_%d_%d" % cell
            for cell in REQUIREMENT_CELLS_FOOTPRINT
            + REQUIREMENT_CELLS_CORRIDOR),
        note="fixture requirement",
    )


def view_payload(points=((0.0, -0.30, 0.10),), observed=(),
                 stamp=2_000_000_000, tf_stamp=None,
                 geometry_hash=GEOMETRY_HASH, depth_present=True):
    return {
        "acquisition_stamp_ns": stamp,
        "tf_stamp_ns": stamp if tf_stamp is None else tf_stamp,
        "geometry_hash": geometry_hash,
        "points_container": [tuple(p) for p in points],
        "observed_cells": [tuple(c) for c in observed],
        "depth_present": depth_present,
    }


def snapshot_for(scene):
    return make_snapshot(map_revision=1, scene_revision=SCENES[scene])


def refusal_of(exc):
    return str(exc).split(":", 1)[0]


class TestEightMapCases(object):
    @pytest.mark.parametrize("scene", ["sim", "deploy"])
    def test_empty_view(self, scene):
        with pytest.raises(ContractViolation) as excinfo:
            integrate_view_with_report(
                snapshot_for(scene), requirement(),
                view_payload(points=(), observed=()))
        assert refusal_of(excinfo.value) == Reason.VIEW_EMPTY.value

    @pytest.mark.parametrize("scene", ["sim", "deploy"])
    def test_missing_depth(self, scene):
        with pytest.raises(ContractViolation) as excinfo:
            integrate_view_with_report(
                snapshot_for(scene), requirement(),
                view_payload(depth_present=False))
        assert refusal_of(excinfo.value) == Reason.VIEW_DEPTH_MISSING.value

    @pytest.mark.parametrize("scene", ["sim", "deploy"])
    def test_tf_stale(self, scene):
        with pytest.raises(ContractViolation) as excinfo:
            integrate_view_with_report(
                snapshot_for(scene), requirement(),
                view_payload(tf_stamp=2_000_000_000 + 300_000_000))
        assert refusal_of(excinfo.value) == Reason.VIEW_TF_STALE.value

    @pytest.mark.parametrize("scene", ["sim", "deploy"])
    def test_hash_conflict(self, scene):
        with pytest.raises(ContractViolation) as excinfo:
            integrate_view_with_report(
                snapshot_for(scene), requirement(),
                view_payload(geometry_hash="foreign-hull"))
        assert refusal_of(excinfo.value) == \
            Reason.MAP_GEOMETRY_HASH_MISMATCH.value

    @pytest.mark.parametrize("scene", ["sim", "deploy"])
    def test_duplicate_view_is_idempotent(self, scene):
        snapshot = snapshot_for(scene)
        first, _inv, report = integrate_view_with_report(
            snapshot, requirement(), view_payload())
        assert report["outcome"] in ("integrated", "partial_coverage")
        assert first is not snapshot
        assert first.map_revision == snapshot.map_revision + 1
        # same acquisition again: unchanged snapshot, no second revision
        again, _inv2, report2 = integrate_view_with_report(
            first, requirement(), view_payload())
        assert again is first
        assert report2["outcome"] == "duplicate"
        assert report2["reason"] == Reason.VIEW_DUPLICATE.value
        assert again.map_revision == first.map_revision

    @pytest.mark.parametrize("scene", ["sim", "deploy"])
    def test_duplicate_commit_counts_once(self, scene):
        snapshot = snapshot_for(scene)
        first, _, _ = integrate_view_with_report(
            snapshot, requirement(), view_payload(stamp=2_000_000_000))
        second, _, _ = integrate_view_with_report(
            first, requirement(), view_payload(stamp=3_000_000_000))
        replay, _, report = integrate_view_with_report(
            second, requirement(), view_payload(stamp=3_000_000_000))
        assert replay is second
        assert report["outcome"] == "duplicate"
        assert second.map_revision == snapshot.map_revision + 2

    @pytest.mark.parametrize("scene", ["sim", "deploy"])
    def test_partial_coverage(self, scene):
        snapshot = snapshot_for(scene)
        # one deep footprint cell observed empty, nothing else
        partial, _inv, report = integrate_view_with_report(
            snapshot, requirement(),
            view_payload(observed=[(14, 5), (14, 6)]))
        assert report["outcome"] == "partial_coverage"
        assert report["reason"] == Reason.VIEW_PARTIAL_COVERAGE.value
        assert partial.state_at(14, 5).value == "free"
        assert partial.state_at(15, 5).value == "unknown"
        assert report["still_unknown_after"]

    @pytest.mark.parametrize("scene", ["sim", "deploy"])
    def test_coverage_sufficient(self, scene):
        snapshot = snapshot_for(scene)
        covered = list(REQUIREMENT_CELLS_FOOTPRINT) + \
            list(REQUIREMENT_CELLS_CORRIDOR)
        complete, _inv, report = integrate_view_with_report(
            snapshot, requirement(),
            view_payload(points=(), observed=covered))
        assert report["outcome"] == "integrated"
        assert report["reason"] is None
        for cell in covered:
            assert complete.state_at(cell[0], cell[1]).value == "free"
        assert not report["still_unknown_after"]

    @pytest.mark.parametrize("scene", ["sim", "deploy"])
    def test_missing_sensors_never_promote(self, scene):
        """Hits only: exactly the hit cells turn occupied; every other
        unknown cell — including cells the view never covered — stays
        unknown. A view with neither hits nor coverage raises instead of
        freeing."""
        snapshot = snapshot_for(scene)
        before = snapshot.occupancy_counts()
        after, _inv, report = integrate_view_with_report(
            snapshot, requirement(),
            view_payload(points=[(0.0, -0.30, 0.10)]))
        counts = after.occupancy_counts()
        assert counts["occupied"] == before["occupied"] + 1
        assert counts["free"] == before["free"]
        # the far unknown region was never observed by this view
        assert after.state_at(17, 10).value == "unknown"
        assert report["n_cells_freed"] == 0

    def test_protocol_signature(self):
        integrator = SnapshotViewIntegrator()
        snapshot = snapshot_for("sim")
        new_snapshot, invalidation = integrator.integrate_view(
            snapshot, requirement(), view_payload(observed=[(14, 5)]))
        assert new_snapshot is not None
        assert invalidation is None
        assert new_snapshot.map_revision == snapshot.map_revision + 1
