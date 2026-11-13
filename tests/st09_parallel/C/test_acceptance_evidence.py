from __future__ import annotations

import dataclasses
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "luggage_planning"))

from luggage_planning.place.map_revalidation import (
    MapRevalidator,
    MockControllerStopProbe,
    RevalidationInput,
    StopPolicy,
)
from luggage_planning.motion.moveit_validity_adapter import StaticMoveItValidityAdapter, ValiditySample
from luggage_planning.motion.trajectory_certifier import CertificationContext, TrajectoryCertifier
from luggage_planning.planning_contract._base import ContractViolation
from luggage_planning.planning_contract.attachment import AttachmentSourceKind, AttachmentValidity
from luggage_planning.planning_contract.certification import EffectCounters, MapDecision
from luggage_planning.planning_contract.fixtures import make_attachment, make_graph_fixture, make_route, make_snapshot
from luggage_planning.planning_contract.graph import TimedTrajectory
from luggage_planning.planning_contract.planning import PlanningSnapshot
from luggage_planning.planning_contract.reasons import Reason


def _current_rev():
    return subprocess.check_output(["git", "rev-parse", "--short=12", "HEAD"], cwd=ROOT, text=True).strip()


def _evidence_root():
    path = ROOT / "docs" / "status" / "evidence" / "frame_migration" / (
        "ST09-parallel-C-%s" % _current_rev())
    path.mkdir(parents=True, exist_ok=True)
    return path


def _json_default(value):
    if hasattr(value, "value"):
        return value.value
    if isinstance(value, Path):
        return str(value)
    return str(value)


def _edge_with_cells(edge, cells=((1, 1),), **attributes):
    merged = dict(edge.attributes)
    merged.update(attributes)
    merged["occupancy_cells"] = tuple(cells)
    return dataclasses.replace(edge, attributes=merged)


def _adapter_for(edges, *, valid=True, clearance=0.06, contacts=()):
    samples = {}
    for edge in edges:
        for index in range(120):
            samples[(edge.edge_id, index)] = ValiditySample(
                edge_id=edge.edge_id,
                sample_index=index,
                valid=valid,
                min_clearance_m=clearance,
                contacts=tuple(contacts),
            )
    return StaticMoveItValidityAdapter(samples)


def _context(attachment=None, snapshot=None, edges=None, adapter=None, now_ns=2_000_000_000):
    attachment = attachment or make_attachment("centered")
    snapshot = snapshot or make_snapshot(map_revision=2, observed_free=[(1, 1)])
    _, base_edges = make_graph_fixture(attachment)
    edges = tuple(_edge_with_cells(edge) for edge in (edges or base_edges))
    return CertificationContext(
        route=make_route(attachment, snapshot),
        snapshot=snapshot,
        attachment=attachment,
        edges=edges,
        validity_adapter=adapter or _adapter_for(edges),
        now_ns=now_ns,
    )


def _full_blocked(snapshot):
    return dataclasses.replace(
        snapshot,
        cell_states=tuple("occupied" for _ in snapshot.cell_states),
        occupancy_domain_complete=True)


def _map_request(latest, *, cells=((1, 1),), in_flight=False, now_ns=2_100_000_000,
                 runtime=True):
    attachment = make_attachment("centered")
    planned = make_snapshot(map_revision=2, observed_free=[(1, 1)], acquisition_stamp_ns=2_000_000_000)
    _, edges = make_graph_fixture(attachment)
    edges = tuple(_edge_with_cells(edge, cells=cells) for edge in edges)
    return RevalidationInput(
        route=make_route(attachment, planned),
        planned_snapshot=planned,
        latest_snapshot=latest,
        edges=edges,
        now_ns=now_ns,
        in_flight=in_flight,
        runtime_lease_valid=runtime,
    )


def _certify_with_edge_attr(**attrs):
    ctx = _context()
    edges = list(ctx.edges)
    edges[0] = _edge_with_cells(edges[0], **attrs)
    ctx = dataclasses.replace(ctx, edges=tuple(edges), validity_adapter=_adapter_for(edges))
    return TrajectoryCertifier().certify(ctx)


def _run_map_acceptance_rows(profiles=("sim", "deploy")):
    """Map-revalidation rows. Independent of the attachment/mutation rows."""
    map_records = []
    for profile in profiles:
        unrelated = make_snapshot(map_revision=3, observed_free=[(1, 1), (5, 5)],
                                  acquisition_stamp_ns=2_050_000_000)
        related = make_snapshot(map_revision=3, occupied=[(12, 1)],
                                acquisition_stamp_ns=2_050_000_000)
        stale = make_snapshot(map_revision=3, observed_free=[(1, 1)], acquisition_stamp_ns=1)
        restart = make_snapshot(map_revision=1, observed_free=[(1, 1)],
                                acquisition_stamp_ns=2_050_000_000)
        blocked = _full_blocked(unrelated)
        cases = [
            ("unrelated_update_continue", _map_request(unrelated), MapDecision.CONTINUE, Reason.OK),
            ("related_update_replan", _map_request(related, cells=((12, 1),)),
             MapDecision.REPLAN, Reason.MAP_REVISION_MISMATCH),
            ("fully_blocked_reject", _map_request(blocked), MapDecision.FAULT, Reason.MAP_FULLY_BLOCKED),
            ("in_flight_conflict_stop", _map_request(related, cells=((12, 1),), in_flight=True),
             MapDecision.STOP, Reason.CARRY_FAULT),
            ("confirmed_old_continue", _map_request(stale, now_ns=4_000_000_000),
             MapDecision.CONTINUE, Reason.OK),
            ("unconfirmed_change_reject", _map_request(dataclasses.replace(
                stale, occupancy_confirmed=False, change_event="box_removed"),
                now_ns=4_000_000_000),
             MapDecision.FAULT, Reason.MAP_CONFIRMATION_REQUIRED),
            ("restart_reject", _map_request(restart), MapDecision.FAULT, Reason.MAP_RESTART),
        ]
        for case_id, revalidation_input, expected_decision, expected_reason in cases:
            decision = MapRevalidator().revalidate(revalidation_input)
            assert decision.decision is expected_decision
            assert decision.reason is expected_reason
            map_records.append({
                "profile": profile,
                "case_id": case_id,
                "decision": decision.decision.value,
                "reason": decision.reason.value,
                "affected_edge_ids": list(decision.affected_edge_ids),
                "map_revision": decision.map_revision,
            })

    return map_records


def test_st09_c_map_acceptance_rows_run_on_their_own():
    records = _run_map_acceptance_rows()
    by_case = {row["case_id"] for row in records}
    assert {"confirmed_old_continue", "unconfirmed_change_reject",
            "unrelated_update_continue", "restart_reject"} <= by_case
    reasons = {row["case_id"]: row["reason"] for row in records}
    assert reasons["confirmed_old_continue"] == "OK"
    assert reasons["unconfirmed_change_reject"] == "MAP_CONFIRMATION_REQUIRED"


def test_st09_c_acceptance_matrix_and_evidence(request):
    evidence_root = _evidence_root()
    certifier = TrajectoryCertifier()
    node_ids = [request.node.nodeid]

    map_records = _run_map_acceptance_rows()

    mutation_records = []
    base_ctx = _context()
    base_edges = list(base_ctx.edges)
    mutation_cases = []
    try:
        good = base_edges[0].trajectory
        TimedTrajectory(
            joint_names=good.joint_names,
            positions=good.positions,
            velocities=good.velocities[:-1],
            accelerations=good.accelerations,
            time_from_start_ns=good.time_from_start_ns,
        )
    except ContractViolation:
        mutation_cases.append(("missing_velocity", Reason.CERT_TIME_PARAMETERIZATION))
    try:
        good = base_edges[0].trajectory
        TimedTrajectory(
            joint_names=good.joint_names,
            positions=good.positions,
            velocities=good.velocities,
            accelerations=good.accelerations,
            time_from_start_ns=good.time_from_start_ns[:-1],
        )
    except ContractViolation:
        mutation_cases.append(("missing_time", Reason.CERT_TIME_PARAMETERIZATION))
    result = _certify_with_edge_attr(orientation_payload="position_only")
    assert result.reason is Reason.CERT_ORIENTATION
    mutation_cases.append(("position_only_orientation", result.reason))
    result = _certify_with_edge_attr(attachment_digest="wrong")
    assert result.reason is Reason.ATTACHMENT_REQUEST_MISMATCH
    mutation_cases.append(("wrong_attachment", result.reason))
    result = _certify_with_edge_attr(forged_certification="stale")
    assert result.reason is Reason.CERT_INVALID_EVIDENCE
    mutation_cases.append(("forged_stale_certification", result.reason))

    joint_order_ctx = _context()
    edges = list(joint_order_ctx.edges)
    traj = edges[0].trajectory
    bad = TimedTrajectory(
        joint_names=tuple(reversed(traj.joint_names)),
        positions=tuple(tuple(reversed(row)) for row in traj.positions),
        velocities=tuple(tuple(reversed(row)) for row in traj.velocities),
        accelerations=tuple(tuple(reversed(row)) for row in traj.accelerations),
        time_from_start_ns=traj.time_from_start_ns,
    )
    edges[0] = dataclasses.replace(edges[0], trajectory=bad)
    result = certifier.certify(dataclasses.replace(
        joint_order_ctx, edges=tuple(edges), validity_adapter=_adapter_for(edges)))
    assert result.reason is Reason.CERT_JOINT_DISCONTINUITY
    mutation_cases.append(("wrong_joint_order", result.reason))

    collision_ctx = _context()
    result = certifier.certify(dataclasses.replace(
        collision_ctx, validity_adapter=_adapter_for(collision_ctx.edges, valid=False, clearance=-0.01)))
    assert result.reason is Reason.CERT_COLLISION
    mutation_cases.append(("interpolation_collision", result.reason))
    unknown_ctx = _context(snapshot=make_snapshot(map_revision=1))
    result = certifier.certify(unknown_ctx)
    assert result.reason is Reason.CERT_OCCUPANCY
    mutation_cases.append(("unknown_occupancy", result.reason))

    for case_id, reason in mutation_cases:
        mutation_records.append({"case_id": case_id, "reason": reason.value})

    attachment = make_attachment("centered")
    route = make_route(attachment, make_snapshot(map_revision=2, observed_free=[(1, 1)]))
    pre_before = EffectCounters(plan_calls=1)
    pre_after = EffectCounters(plan_calls=1)
    attachment_cases = [
        ("missing", certifier.classify_attachment_rejection(None, route, 2_000_000_000)),
        ("nan_dimensions", certifier.classify_attachment_rejection(
            {"dimensions_wdh": (0.5, float("nan"), 0.2),
             "t_tcp_box_quaternion_xyzw": (0, 0, 0, 1)}, route, 2_000_000_000)),
        ("non_positive_dimensions", certifier.classify_attachment_rejection(
            {"dimensions_wdh": (0.5, 0.0, 0.2),
             "t_tcp_box_quaternion_xyzw": (0, 0, 0, 1)}, route, 2_000_000_000)),
        ("illegal_quaternion", certifier.classify_attachment_rejection(
            {"dimensions_wdh": (0.5, 0.4, 0.2),
             "t_tcp_box_quaternion_xyzw": (0, 0, 0, 2)}, route, 2_000_000_000)),
        ("expired", certifier.classify_attachment_rejection(attachment, route, 20_000_000_000)),
        ("wrong_frame", certifier.classify_attachment_rejection(
            dataclasses.replace(attachment, tcp_frame="wrong_tcp"), route, 2_000_000_000)),
        ("wrong_request", certifier.classify_attachment_rejection(
            make_attachment("centered", request_id="wrong-request"), route, 2_000_000_000)),
        ("reattach", certifier.classify_attachment_rejection(
            make_attachment("centered", validity=AttachmentValidity.INVALIDATED,
                            invalidation_reason=Reason.ATTACHMENT_REATTACH.value),
            route, 2_000_000_000)),
        ("gt_as_measured", certifier.classify_attachment_rejection(
            make_attachment("centered", source=AttachmentSourceKind.REFERENCE_SIM_GT),
            route, 2_000_000_000)),
    ]
    expected_attachment_reasons = {
        "missing": Reason.ATTACHMENT_MISSING,
        "nan_dimensions": Reason.ATTACHMENT_INVALID_DIMENSIONS,
        "non_positive_dimensions": Reason.ATTACHMENT_INVALID_DIMENSIONS,
        "illegal_quaternion": Reason.ATTACHMENT_INVALID_TRANSFORM,
        "expired": Reason.ATTACHMENT_EXPIRED,
        "wrong_frame": Reason.ATTACHMENT_FRAME_MISMATCH,
        "wrong_request": Reason.ATTACHMENT_REQUEST_MISMATCH,
        "reattach": Reason.ATTACHMENT_REATTACH,
        "gt_as_measured": Reason.ATTACHMENT_GT_AS_MEASURED,
    }
    attachment_records = []
    for case_id, reason in attachment_cases:
        assert reason is expected_attachment_reasons[case_id]
        attachment_records.append({
            "case_id": case_id,
            "reason": reason.value,
            "effect_delta": pre_before.business_delta(pre_after),
        })

    stop_probe = MockControllerStopProbe()
    stop_trace = stop_probe.cancel_and_hold(
        "req-fixture-1", "route-fixture-1",
        EffectCounters(plan_calls=1, execute_calls=1, attach_calls=1),
        StopPolicy(deadline_ns=250_000_000))
    assert stop_trace.allowed_cancel_hold
    assert all(value == 0 for value in stop_trace.business_effect_delta.values())
    assert stop_probe.traces[0]["latency_ns"] >= 0
    assert stop_probe.traces[0]["latency_ns"] <= stop_probe.traces[0]["deadline_ns"]

    healthy = certifier.certify(_context())
    assert healthy.passed
    certification_results = {
        "schema": "st09.certification_results/2",
        "revision": _current_rev(),
        "test_node_ids": node_ids,
        "healthy_certification": {
            "passed": healthy.passed,
            "reason": healthy.reason.value,
            "sampling_dt_max_ns": healthy.sampling_dt_max_ns,
            "sampling_motion_bound_m": healthy.sampling_motion_bound_m,
            "checks": list(healthy.checks),
        },
        "map_cases": map_records,
        "trajectory_mutations": mutation_records,
        "attachment_negatives": attachment_records,
    }
    stop_measurements = {
        "schema": "st09.stop_measurements/1",
        "deadline_ns": 250_000_000,
        "measurements": stop_probe.traces,
        "stop_envelope": {
            "max_latency_ns": max(row["latency_ns"] for row in stop_probe.traces),
            "allowed_effects": ["cancelled_goals", "held_goals"],
            "business_effect_delta": dict(stop_trace.business_effect_delta),
        },
    }
    mock_traces = {
        "schema": "st09.mock_controller_traces/1",
        "traces": stop_probe.traces,
    }
    (evidence_root / "certification_results.json").write_text(
        json.dumps(certification_results, indent=2, sort_keys=True, default=_json_default) + "\n")
    (evidence_root / "mock_controller_traces.json").write_text(
        json.dumps(mock_traces, indent=2, sort_keys=True, default=_json_default) + "\n")
    (evidence_root / "stop_measurements.json").write_text(
        json.dumps(stop_measurements, indent=2, sort_keys=True, default=_json_default) + "\n")
    (evidence_root / "SUMMARY.md").write_text(
        "# ST09-C certification evidence\n\n"
        "- map cases: %d\n"
        "- trajectory mutations: %d\n"
        "- attachment negatives: %d\n"
        "- stop measurements: %d\n"
        "- test node IDs: `%s`\n" % (
            len(map_records), len(mutation_records), len(attachment_records),
            len(stop_probe.traces), "`, `".join(node_ids)))

    assert len(map_records) == 12
    assert len(mutation_records) >= 8
    assert len(attachment_records) == 9
