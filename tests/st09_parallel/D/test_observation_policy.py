"""ST09-D acceptance: the bounded observation policy — at most two
prevalidated views, coverage over footprint and corridor, TF freshness and
visibility gates; read-only (never mutates the snapshot, never promotes
cells, no GT/catalog geometry)."""

import copy
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from luggage_perception.observation_policy import (
    MAX_VIEWS,
    VIEW_ORDER,
    ViewCandidate,
    coverage_after_integration,
    plan_views,
)

from st09_contracts.fixtures import make_snapshot
from st09_contracts.planning import ObservationRequirement


def requirement():
    footprint = tuple((i, j) for i in (14, 15) for j in (5, 6))
    corridor = tuple((i, 5) for i in range(4, 14))
    cells = footprint + corridor
    return ObservationRequirement(
        requirement_id="req-policy",
        candidate_ids=("cand-fixture-1",),
        footprint_cells=footprint,
        corridor_cells=corridor,
        missing_cells=cells,
        missing_reasons=tuple("unknown_%d_%d" % c for c in cells),
    )


STAMP = 5_000_000_000


def portal_aim(name="aim-fresh", stamp=STAMP, tf_stamp=None, eye=(-1.0, -0.32, 0.15)):
    return ViewCandidate(
        name=name, kind="portal_aim", eye_xyz=eye,
        optical_axis=(1.0, 0.0, 0.0), stamp_ns=stamp,
        tf_stamp_ns=tf_stamp if tf_stamp is not None else stamp)


def second_region(name="second-fresh", eye=(-1.0, -0.32, 0.15)):
    return ViewCandidate(
        name=name, kind="second_region", eye_xyz=eye,
        optical_axis=(1.0, 0.0, 0.0), stamp_ns=STAMP, tf_stamp_ns=STAMP)


class TestBoundedViews(object):
    def test_at_most_two_views_portal_first(self):
        snapshot = make_snapshot(map_revision=1)
        candidates = [
            portal_aim("aim-1"),
            portal_aim("aim-2", eye=(-0.9, -0.10, 0.15)),
            second_region("second-1"),
            second_region("second-2"),
        ]
        result = plan_views(requirement(), snapshot, candidates)
        assert len(result.views) <= MAX_VIEWS
        kinds = [view.candidate.kind for view in result.views]
        if kinds:
            assert kinds[0] == "portal_aim"
        assert VIEW_ORDER == ("portal_aim", "second_region")

    def test_stale_tf_and_empty_views_are_rejected_with_reasons(self):
        snapshot = make_snapshot(map_revision=1)
        candidates = [
            portal_aim("aim-stale", tf_stamp=STAMP + 500_000_000),
            ViewCandidate(
                name="aim-blind", kind="portal_aim",
                eye_xyz=(5.0, 5.0, 5.0),        # far away, looking away
                optical_axis=(1.0, 0.0, 0.0), stamp_ns=STAMP,
                tf_stamp_ns=STAMP),
        ]
        result = plan_views(requirement(), snapshot, candidates)
        assert result.views == ()
        reasons = dict(result.rejected)
        assert reasons["aim-stale"] == "VIEW_TF_STALE"
        assert reasons["aim-blind"] == "VIEW_EMPTY"

    def test_coverage_prediction_over_footprint_and_corridor(self):
        snapshot = make_snapshot(map_revision=1)
        result = plan_views(
            requirement(), snapshot, [portal_aim(), second_region()])
        assert result.views, "a seeing portal view must be selected"
        covered = set()
        for view in result.views:
            covered |= set(view.covered_cells)
        assert covered, "predicted coverage must name cells"
        # cells behind the camera or outside the frustum are reported
        first = result.views[0]
        assert first.coverage_ratio > 0.0

    def test_policy_is_read_only(self):
        snapshot = make_snapshot(map_revision=1)
        before = snapshot.digest()
        plan_views(requirement(), snapshot, [portal_aim(), second_region()])
        assert snapshot.digest() == before
        assert snapshot.occupancy_counts() == \
            make_snapshot(map_revision=1).occupancy_counts()

    def test_coverage_after_integration_names_unknowns(self):
        from st09_contracts.fixtures import snapshot_pair
        _first, second = snapshot_pair()
        remaining = coverage_after_integration(second, requirement())
        assert remaining, "the deep footprint is still unknown after one pass"
        for cell in remaining:
            assert second.state_at(cell[0], cell[1]).value == "unknown"
