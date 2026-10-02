"""ST09-I S2 offline: the real MoveIt validity adapter's pure half.

The sampling rule MUST mirror the certifier's own subdivision exactly —
a parity oracle runs the real TrajectoryCertifier with a counting
adapter and asserts every (edge, index) the certifier asks for is one
my ``sample_states`` enumeration produces, in order. The analytic
clearance geometry and the fail-closed contract are pinned directly;
the live move_group probe layer is exercised on the mock graph later.
"""

from __future__ import division

import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "luggage_planning"))

import pytest  # noqa: E402

from st09_contracts._base import ContractViolation  # noqa: E402
from st09_contracts.fixtures import (  # noqa: E402
    make_attachment,
    make_graph_fixture,
    make_route,
    make_snapshot,
)
from st09_contracts.graph import TimedTrajectory  # noqa: E402

from luggage_planning.moveit_edge_adapter import BoxSpec, PayloadSpec  # noqa: E402
from luggage_planning.moveit_state_validity import (  # noqa: E402
    MoveItStateValidityAdapter,
    payload_feature_points,
    point_box_distance,
    sample_states,
)
from luggage_planning.moveit_validity_adapter import (  # noqa: E402
    StaticMoveItValidityAdapter,
    ValiditySample,
)
from luggage_planning.trajectory_certifier import (  # noqa: E402
    CertificationContext,
    TrajectoryCertifier,
)


class CountingAdapter(StaticMoveItValidityAdapter):
    def __init__(self, samples):
        super().__init__(samples)
        self.requested = []

    def validate_sample(self, edge_id, sample_index):
        self.requested.append((str(edge_id), int(sample_index)))
        return super().validate_sample(edge_id, sample_index)


def _certify_with_counter():
    attachment = make_attachment("centered", symmetry="none")
    snapshot = make_snapshot(1)
    _nodes, edges = make_graph_fixture(attachment)
    route = make_route(attachment, snapshot)
    samples = {}
    for edge in edges:
        for index in range(len(sample_states(edge.trajectory))):
            samples[(edge.edge_id, index)] = ValiditySample(
                edge_id=edge.edge_id, sample_index=index, valid=True,
                min_clearance_m=0.05, contacts=())
    adapter = CountingAdapter(samples)
    context = CertificationContext(
        route=route, snapshot=snapshot, attachment=attachment,
        edges=tuple(edges), validity_adapter=adapter,
        now_ns=attachment.acquisition_stamp_ns + 1)
    result = TrajectoryCertifier().certify(context)
    return result, adapter.requested, edges


class TestSamplingParity:

    def test_certifier_requests_match_sample_states_exactly(self):
        result, requested, edges = _certify_with_counter()
        assert result.passed, result.reason
        expected = []
        for edge_id in ("e-entry", "e-insert", "e-retreat"):
            expected.extend((edge_id, i) for i in range(len(
                sample_states([e for e in edges
                               if e.edge_id == edge_id][0].trajectory))))
        assert requested == expected

    def test_subdivision_rule(self):
        # 40 ms interval -> 4 subdivisions -> 5 samples per interval.
        trajectory = TimedTrajectory(
            joint_names=tuple("elfin_joint%d" % i for i in range(1, 7)),
            positions=((0.0,) * 6, (0.04,) * 6, (0.08,) * 6),
            velocities=((0.0,) * 6,) * 3,
            accelerations=((0.0,) * 6,) * 3,
            time_from_start_ns=(0, 40_000_000, 80_000_000))
        states = sample_states(trajectory)
        assert len(states) == 10  # 5 + 5 across the two intervals
        assert states[0] == (0.0,) * 6
        assert states[4] == (0.04,) * 6
        assert states[-1] == (0.08,) * 6

    def test_single_knot_trajectory_degenerates_to_one_state(self):
        trajectory = TimedTrajectory(
            joint_names=tuple("elfin_joint%d" % i for i in range(1, 7)),
            positions=((0.1,) * 6,),
            velocities=((0.0,) * 6,),
            accelerations=((0.0,) * 6,),
            time_from_start_ns=(0,))
        assert sample_states(trajectory) == ((0.1,) * 6,)


class TestClearanceGeometry:

    def test_point_inside_oriented_box_is_zero(self):
        box = BoxSpec(object_id="b", center=(0.5, 0.0, 0.3),
                      size_whd=(0.4, 0.3, 0.2))
        assert point_box_distance((0.5, 0.0, 0.3), box) == 0.0

    def test_axis_aligned_distance(self):
        box = BoxSpec(object_id="b", center=(0.5, 0.0, 0.3),
                      size_whd=(0.4, 0.3, 0.2))
        # 0.2 (half x) + 0.1 gap from the +x face
        assert abs(point_box_distance((0.8, 0.0, 0.3), box) - 0.1) < 1e-12

    def test_rotated_box_distance(self):
        # 90-degree yaw: the box's long axis (w=0.4) now spans y.
        box = BoxSpec(object_id="b", center=(0.5, 0.0, 0.3),
                      size_whd=(0.4, 0.3, 0.2),
                      quaternion_xyzw=(0.0, 0.0, math.sqrt(0.5),
                                       math.sqrt(0.5)))
        # Point out along +x now faces the 0.3-deep side (half 0.15).
        assert abs(point_box_distance((0.75, 0.0, 0.3), box) - 0.10) < 1e-9

    def test_payload_feature_points_count_and_offset(self):
        payload = PayloadSpec(
            dimensions_wdh=(0.30, 0.20, 0.20),
            t_tcp_box_translation=(0.0, 0.0, -0.10),
            t_tcp_box_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0))
        points = payload_feature_points(
            (0.5, 0.0, 0.4), (0.0, 0.0, 0.0, 1.0), payload)
        assert len(points) == 1 + 6 + 8
        # Box center sits 0.10 below the TCP.
        assert abs(min(p[2] for p in points) - (0.30 - 0.10)) < 1e-12
        assert abs(max(p[2] for p in points) - (0.50 - 0.10)) < 1e-12


class TestAdapterFailClosed:

    def _adapter(self):
        class _NodeStub(object):
            def create_client(self, *_args, **_kwargs):
                raise AssertionError("offline layer must not create clients")

        return MoveItStateValidityAdapter(_NodeStub(), fk=lambda q: (
            (0.5, 0.0, 0.4), (0.0, 0.0, 0.0, 1.0)))

    def test_unknown_edge_fails_closed(self):
        adapter = self._adapter()
        with pytest.raises(ContractViolation):
            adapter.validate_sample("e-nope", 0)

    def test_missing_scene_state_fails_at_load(self):
        from st09_contracts.fixtures import make_timed_trajectory

        adapter = self._adapter()
        edge = type("Edge", (), {
            "edge_id": "e-1", "trajectory": make_timed_trajectory()})()
        with pytest.raises(ContractViolation):
            adapter.load_edges((edge,), scene_states={})

    def test_out_of_range_sample_fails_closed(self):
        from luggage_planning.moveit_edge_adapter import SceneState

        from st09_contracts.fixtures import make_timed_trajectory

        adapter = self._adapter()
        edge = type("Edge", (), {
            "edge_id": "e-1", "trajectory": make_timed_trajectory()})()
        adapter.load_edges(
            (edge,), scene_states={"e-1": SceneState(load_state="carrying")})
        with pytest.raises(ContractViolation):
            adapter.validate_sample("e-1", 10_000)
