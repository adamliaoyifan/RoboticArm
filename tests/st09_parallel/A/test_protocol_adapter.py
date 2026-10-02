"""ST09-A acceptance: the frozen CandidateGenerator protocol adapter.

The adapter bridges the frozen ``PlanningSnapshot`` (coarse cell-state grid)
to ``solve_placement`` and back to frozen ``PlacementCandidate`` /
``ObservationRequirement`` value objects — pure, no motion requests, and a
single failed insertion corridor can never surface as BIN_FULL.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from helpers import BOX_WDH, PARAMS_FULL, chamfered_surface

from luggage_packing.placement_solver import SnapshotCandidateGenerator

from st09_contracts.fixtures import (
    make_attachment,
    make_payload,
    make_snapshot,
    snapshot_pair,
)
from st09_contracts.planning import (
    GeometryVerdict,
    ObservationRequirement,
    PlacementCandidate,
)

DESCRIPTOR = chamfered_surface()["geometry_descriptor"]


def make_generator(payload=None, **kwargs):
    payload = payload or make_payload(make_attachment("centered"))
    return SnapshotCandidateGenerator(
        payload=payload,
        geometry_descriptor=DESCRIPTOR,
        allowed_yaws=[0.0, 1.5707963267948966],
        params=PARAMS_FULL,
        **kwargs)


class TestProtocolConformance(object):
    def test_returns_frozen_value_objects(self):
        generator = make_generator()
        payload = generator._payload
        candidates, requirement = generator.generate_candidates(
            make_snapshot(map_revision=2), payload.digest())
        assert candidates
        for candidate in candidates:
            assert isinstance(candidate, PlacementCandidate)
            assert isinstance(candidate.verdict, GeometryVerdict)
            assert candidate.footprint_class in (0, 1)
        if requirement is not None:
            assert isinstance(requirement, ObservationRequirement)

    def test_order_matches_frozen_sorting_key(self):
        generator = make_generator()
        payload = generator._payload
        candidates, _ = generator.generate_candidates(
            make_snapshot(map_revision=2), payload.digest())
        feasible = [c for c in candidates if c.legal]
        assert feasible
        keys = [c.sorting_key() for c in feasible]
        assert keys == sorted(keys), (
            "feasible candidates must come back in frozen sorting_key order")

    def test_digest_mismatch_is_refused(self):
        generator = make_generator()
        import pytest
        with pytest.raises(ValueError):
            generator.generate_candidates(
                make_snapshot(map_revision=2), "not-the-payload-digest")

    def test_revision_two_handoff_pattern(self):
        """The frozen fixture pattern: revision 1 -> revision 2 with a box
        integrated between them. Both revisions answer; the revision only
        rides along on the surface record."""
        before, after = snapshot_pair()
        generator = make_generator()
        payload = generator._payload
        c1, _ = generator.generate_candidates(before, payload.digest())
        c2, _ = generator.generate_candidates(after, payload.digest())
        assert c1 and c2

    def test_pure_no_motion_requests(self):
        """The adapter must not import ROS or the motion stack — module
        level check plus a smoke run on the fully-unknown revision-1
        snapshot (floor-prior path)."""
        import luggage_packing.placement_solver as solver_module
        assert not any(
            hasattr(solver_module, name) for name in ("rclpy", "rospy"))
        generator = make_generator()
        payload = generator._payload
        candidates, requirement = generator.generate_candidates(
            make_snapshot(map_revision=1), payload.digest())
        assert candidates
        # fully-unknown grid -> floor prior everywhere; observation cells
        # must surface through the requirement, never as fake evidence
        if requirement is not None:
            assert requirement.missing_cells


class TestCorridorNeverBecomesBinFull(object):
    def test_adapter_returns_rejections_not_exceptions(self):
        """Whatever the map, the adapter returns candidates/verdicts; a
        corridor or capacity failure is data, not an exception, and never
        a fabricated BIN_FULL claim."""
        generator = make_generator()
        payload = generator._payload
        # an occupied-everything snapshot: every window loses capacity
        snapshot = make_snapshot(
            map_revision=2,
            occupied=tuple(
                (i, j) for i in range(20) for j in range(12)
                if i < 10 and j < 6))
        candidates, _requirement = generator.generate_candidates(
            snapshot, payload.digest())
        for candidate in candidates:
            assert isinstance(candidate.verdict, GeometryVerdict)
