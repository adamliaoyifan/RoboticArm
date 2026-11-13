"""C0 shared-contract behaviour tests: positives and negatives.

Covers the C0 freeze rules: immutable arrays, NaN rejection at
construction, representation equivalence never refusing alone, node
identity by full q, trajectory field preservation, and the single
reason taxonomy.
"""

from __future__ import annotations

import dataclasses
import math

import pytest

from luggage_planning.planning_contract import (
    AttachmentRecord,
    AttachmentSourceKind,
    AttachmentValidity,
    CertificationResult,
    EffectCounters,
    EffectTrace,
    GraphEdge,
    GraphNode,
    LoadState,
    MapDecision,
    MapRevalidationDecision,
    PayloadIdentity,
    PayloadSymmetry,
    PlaceTarget,
    PlannedRoute,
    PlacementCandidate,
    GeometryVerdict,
    PlanningSnapshot,
    Reason,
    LegacyCode,
    LEGACY_TO_REASON,
)
from luggage_planning.planning_contract._base import ContractViolation
from luggage_planning.planning_contract.fixtures import (
    make_attachment,
    make_box_target,
    make_effect_trace,
    make_graph_fixture,
    make_payload,
    make_route,
    make_snapshot,
    make_timed_trajectory,
    snapshot_pair,
)
from luggage_planning.planning_contract.protocols import (
    CandidateGenerator,
    Certifier,
    RoutePlanner,
    ViewIntegrator,
)


def _snapshot():
    return make_snapshot(map_revision=1)


def _attachment():
    return make_attachment("centered")


class TestPlanningSnapshot:
    def test_revision_handoff_pattern(self):
        first, second = snapshot_pair()
        assert first.map_revision == 1
        assert second.map_revision == 2
        assert second.commit_revision == 2
        counts = second.occupancy_counts()
        assert counts["occupied"] == 4
        assert counts["free"] > 0
        # front half observed free in revision 2, rear stays unknown
        assert second.state_at(1, 1) is not None

    def test_arrays_immutable(self):
        snap = _snapshot()
        with pytest.raises(Exception):
            snap.cells_i[0] = 99  # tuples refuse item assignment

    def test_unknown_never_promoted_and_digest_stable(self):
        first, second = snapshot_pair()
        assert second.state_at(18, 0).value == "unknown"
        again = make_snapshot(map_revision=2,
                              occupied=[(14, 5), (14, 6), (15, 5), (15, 6)],
                              acquisition_stamp_ns=2_000_000_000, commit_revision=2)
        assert again.digest() == second.digest() != first.digest()

    def test_negative_bad_state_string(self):
        with pytest.raises(ContractViolation):
            PlanningSnapshot(
                geometry_id="g", geometry_hash="h", map_revision=1,
                acquisition_stamp_ns=1, commit_revision=1,
                occupancy_digest="d", resolution_m=0.05, origin_xy=(0.0, 0.0),
                cells_i=(0,), cells_j=(0,), cell_states=("platinum",))


class TestPayloadIdentity:
    def test_positive(self):
        payload = make_payload(_attachment())
        assert payload.symmetry is PayloadSymmetry.PI
        assert payload.footprint_classes() == (
            (payload.dimensions_wdh[0], payload.dimensions_wdh[1]),
            (payload.dimensions_wdh[1], payload.dimensions_wdh[0]))

    def test_negative_nan_and_nonpositive_dimensions(self):
        base = dict(
            cargo_id="c", box_id="b", request_id="r", cycle_id="y",
            dimensions_wdh=(0.646, 0.409, 0.277),
            symmetry=PayloadSymmetry.PI,
            t_tcp_box_translation=(0.0, 0.0, 0.1),
            t_tcp_box_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            attachment_incarnation=0, acquisition_stamp_ns=1)
        with pytest.raises(ContractViolation):
            PayloadIdentity(**{**base, "dimensions_wdh": (0.646, 0.409, float("nan"))})
        with pytest.raises(ContractViolation):
            PayloadIdentity(**{**base, "dimensions_wdh": (0.646, 0.409, 0.0)})

    def test_negative_nonunit_quaternion(self):
        with pytest.raises(ContractViolation):
            PayloadIdentity(
                cargo_id="c", box_id="b", request_id="r", cycle_id="y",
                dimensions_wdh=(1, 1, 1), symmetry=PayloadSymmetry.NONE,
                t_tcp_box_translation=(0, 0, 0),
                t_tcp_box_quaternion_xyzw=(0.0, 0.0, 0.0, 2.0),
                attachment_incarnation=0, acquisition_stamp_ns=1)


class TestPlacementCandidate:
    def test_positive_and_sorting_key_deep_first(self):
        deep = PlacementCandidate(
            candidate_id="c2", footprint_class=0, heading_theta=0.0,
            position_container=(0.4, 0.0, 0.14), quaternion_xyzw_container=(0, 0, 0, 1),
            support_layer_z=0.0, depth_m=0.8, verdict=GeometryVerdict.LEGAL,
            score=0.5, detail={"route_clearance_m": 0.1})
        shallow = PlacementCandidate(
            candidate_id="c1", footprint_class=0, heading_theta=0.0,
            position_container=(0.1, 0.0, 0.14), quaternion_xyzw_container=(0, 0, 0, 1),
            support_layer_z=0.0, depth_m=0.5, verdict=GeometryVerdict.LEGAL,
            score=0.9, detail={"route_clearance_m": 0.1})
        assert deep.legal and shallow.legal
        assert deep.sorting_key() < shallow.sorting_key()

    def test_negative_tilt_unsupported(self):
        with pytest.raises(ContractViolation):
            PlacementCandidate(
                candidate_id="c", footprint_class=0, heading_theta=0.0,
                position_container=(0, 0, 0), quaternion_xyzw_container=(0, 0, 0, 1),
                support_layer_z=0.0, depth_m=0.5,
                verdict="support_tilt_unsupported",  # not a GeometryVerdict
                score=1.0)


class TestGraphContracts:
    def test_node_identity_is_full_q_not_xyz(self):
        base = dict(tcp_position=(0.2, 0.0, 0.4), tcp_quaternion_xyzw=(0, 0, 0, 1),
                    region="slot", load_state=LoadState.CARRYING)
        a = GraphNode("n1", q=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6), **base)
        b = GraphNode("n2", q=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6), **base)
        c = GraphNode("n3", q=(-0.1, 0.2, 0.3, 0.4, 0.5, 0.6), **base)
        assert a.same_state(b)
        assert not a.same_state(c)  # same TCP XYZ, different q: never merged

    def test_ik_family_label_is_not_state(self):
        base = dict(q=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
                    tcp_position=(0.2, 0.0, 0.4), tcp_quaternion_xyzw=(0, 0, 0, 1),
                    region="slot", load_state=LoadState.CARRYING)
        a = GraphNode("n1", ik_family="f1", **base)
        b = GraphNode("n2", ik_family="f2", **base)
        assert a.same_state(b)  # family label change alone never refuses

    def test_trajectory_preserves_order_velocity_acceleration_ns(self):
        traj = make_timed_trajectory()
        assert traj.joint_names[0] == "elfin_joint1"
        assert traj.positions[-1][0] == pytest.approx(0.40)
        assert traj.velocities[1][0] == pytest.approx(0.2)
        assert traj.time_from_start_ns == (0, 500_000_000, 1_000_000_000)

    def test_negative_dropped_times(self):
        from luggage_planning.planning_contract.graph import TimedTrajectory as TT
        good = make_timed_trajectory()
        with pytest.raises(ContractViolation):
            TT(joint_names=good.joint_names, positions=good.positions,
               velocities=good.velocities, accelerations=good.accelerations,
               time_from_start_ns=(0, 500_000_000))  # dropped final time
        with pytest.raises(ContractViolation):
            TT(joint_names=good.joint_names, positions=good.positions,
               velocities=good.velocities, accelerations=good.accelerations,
               time_from_start_ns=(0, 0, 1))  # non-monotonic

    def test_route_and_edge_fixture(self):
        attachment = _attachment()
        nodes, edges = make_graph_fixture(attachment)
        route = make_route(attachment, _snapshot())
        assert route.entry_edge_ids == ("e-entry", "e-insert")
        assert route.retreat_edge_ids == ("e-retreat",)  # post-release retreat is separate
        assert edges[2].load_state is LoadState.UNLOADED


class TestCertificationContracts:
    def test_positive_verdict_reports_coverage(self):
        route = make_route(_attachment(), _snapshot())
        cert = CertificationResult(
            certification_id="c1", route_id=route.route_id, passed=True,
            reason=Reason.OK, checks=("fk", "joint_limits"),
            validation_revision=1, sampling_dt_max_ns=10_000_000,
            sampling_motion_bound_m=0.025,
            method_note="adaptive subdivision")
        assert cert.passed and "fk" in cert.checks

    def test_negative_fail_closed_consistency(self):
        route = make_route(_attachment(), _snapshot())
        with pytest.raises(ContractViolation):
            CertificationResult(
                certification_id="c1", route_id=route.route_id, passed=True,
                reason=Reason.CERT_TIMEOUT,  # passed with a failure reason
                checks=("fk",), validation_revision=1)

    def test_revalidation_decisions(self):
        cont = MapRevalidationDecision(
            decision=MapDecision.CONTINUE, reason=Reason.OK,
            map_revision=2, validation_epoch=1, affected_edge_ids=())
        replan = MapRevalidationDecision(
            decision=MapDecision.REPLAN, reason=Reason.MAP_REVISION_MISMATCH,
            map_revision=3, validation_epoch=1, affected_edge_ids=("e-insert",))
        assert cont.decision is MapDecision.CONTINUE
        assert replan.affected_edge_ids == ("e-insert",)

    def test_effect_trace_pre_effect_zero_delta(self):
        trace = make_effect_trace(phase="pre_pick_admission")
        assert all(v == 0 for v in trace.business_effect_delta.values())


class TestReasonTaxonomy:
    def test_plan_named_reasons_exist(self):
        required = [
            "PLANNING_BUDGET_EXHAUSTED", "ATLAS_ORIENTATION_UNCOVERED",
            "OBSERVATION_REQUIRED", "BIN_FULL", "HULL_VIOLATION",
            "ATTACHMENT_GT_AS_MEASURED", "ATTACHMENT_REATTACH",
            "ATTACHMENT_SLIP", "ATTACHMENT_EXPIRED", "MAP_STALE",
            "VIEW_PARTIAL_COVERAGE", "CARRY_FAULT", "CERT_SAMPLING_UNBOUNDED",
        ]
        for name in required:
            assert Reason[name].value == name

    def test_legacy_mapping_total(self):
        assert set(LEGACY_TO_REASON) == set(LegacyCode)
        for legacy, reason in LEGACY_TO_REASON.items():
            assert isinstance(reason, Reason)


class TestProtocols:
    def test_frozen_call_protocols_raise_not_implemented(self):
        snap = _snapshot()
        gen = CandidateGenerator()
        planner = RoutePlanner()
        cert = Certifier()
        integ = ViewIntegrator()
        for fn, args in (
                (gen.generate_candidates, (snap, "payload")),
                (planner.plan_route, (snap, _attachment(), None, (), 1)),
                (cert.certify, (None,)),
                (cert.revalidate, (None, snap)),
                (integ.integrate_view, (snap, None, {}))):
            with pytest.raises(NotImplementedError):
                fn(*args)

    def test_freeze_rules_documented(self):
        import luggage_planning.planning_contract as pkg
        doc = pkg.__doc__
        for rule in ("q``/``-q", "IK family", "unknown", "ROS-free"):
            assert rule in doc


class TestImmutability:
    def test_frozen_dataclasses(self):
        for cls in (PlanningSnapshot, PayloadIdentity, PlacementCandidate,
                    GraphNode, GraphEdge, PlannedRoute, CertificationResult,
                    MapRevalidationDecision, EffectTrace, AttachmentRecord,
                    PlaceTarget):
            assert dataclasses.is_dataclass(cls)
            snap = None
            assert cls.__dataclass_params__.frozen  # type: ignore[attr-defined]
