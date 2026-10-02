from __future__ import annotations

import dataclasses
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "luggage_planning"))

from luggage_planning.moveit_validity_adapter import StaticMoveItValidityAdapter, ValiditySample
from luggage_planning.trajectory_certifier import CertificationContext, TrajectoryCertifier
from st09_contracts.attachment import AttachmentSourceKind, AttachmentValidity
from st09_contracts.fixtures import make_attachment, make_graph_fixture, make_route, make_snapshot
from st09_contracts.graph import TimedTrajectory
from st09_contracts.reasons import Reason


def _edge_with_cells(edge, cells=((1, 1),)):
    return dataclasses.replace(edge, attributes={"occupancy_cells": tuple(cells)})


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


def test_certifies_full_route_with_sampling_bounds():
    result = TrajectoryCertifier().certify(_context())
    assert result.passed
    assert result.sampling_dt_max_ns == 10_000_000
    assert result.sampling_motion_bound_m <= 0.025
    assert "attached_payload" in result.checks
    assert "occupancy_map" in result.checks


@pytest.mark.parametrize(
    "mutate,reason",
    [
        ("joint_order", Reason.CERT_JOINT_DISCONTINUITY),
        ("joint_limit", Reason.CERT_JOINT_LIMIT),
        ("interpolation_collision", Reason.CERT_COLLISION),
        ("unknown_occupancy", Reason.CERT_OCCUPANCY),
    ],
)
def test_trajectory_mutations_fail_closed(mutate, reason):
    ctx = _context()
    edges = list(ctx.edges)
    adapter = ctx.validity_adapter
    snapshot = ctx.snapshot
    if mutate == "joint_order":
        traj = edges[0].trajectory
        bad = TimedTrajectory(
            joint_names=tuple(reversed(traj.joint_names)),
            positions=tuple(tuple(reversed(row)) for row in traj.positions),
            velocities=tuple(tuple(reversed(row)) for row in traj.velocities),
            accelerations=tuple(tuple(reversed(row)) for row in traj.accelerations),
            time_from_start_ns=traj.time_from_start_ns,
        )
        edges[0] = dataclasses.replace(edges[0], trajectory=bad)
        adapter = _adapter_for(edges)
    elif mutate == "joint_limit":
        traj = edges[0].trajectory
        rows = list(traj.positions)
        rows[1] = (9.0,) + rows[1][1:]
        edges[0] = dataclasses.replace(edges[0], trajectory=dataclasses.replace(traj, positions=tuple(rows)))
        adapter = _adapter_for(edges)
    elif mutate == "interpolation_collision":
        adapter = _adapter_for(edges, valid=False, clearance=-0.01)
    elif mutate == "unknown_occupancy":
        snapshot = make_snapshot(map_revision=1)
        ctx = _context(snapshot=snapshot)
        edges = list(ctx.edges)
        adapter = ctx.validity_adapter
    bad_ctx = dataclasses.replace(ctx, snapshot=snapshot, edges=tuple(edges), validity_adapter=adapter)
    result = TrajectoryCertifier().certify(bad_ctx)
    assert not result.passed
    assert result.reason is reason


def test_support_contact_exemption_is_limited_to_target_support():
    contacts = ({"type": "support_contact", "support_id": "target-support"},)
    ctx = _context()
    ok = dataclasses.replace(ctx, validity_adapter=_adapter_for(ctx.edges, valid=False, contacts=contacts))
    assert TrajectoryCertifier().certify(ok).passed
    bad_contacts = ({"type": "support_contact", "support_id": "neighbor-box"},)
    bad = dataclasses.replace(ctx, validity_adapter=_adapter_for(ctx.edges, valid=False, contacts=bad_contacts))
    result = TrajectoryCertifier().certify(bad)
    assert result.reason is Reason.CERT_COLLISION


@pytest.mark.parametrize(
    "attachment_kwargs,route_from_original,reason",
    [
        ({"source": AttachmentSourceKind.REFERENCE_SIM_GT}, False, Reason.ATTACHMENT_GT_AS_MEASURED),
        ({"source": AttachmentSourceKind.MEASURED_PRE_ATTACH_ESTIMATE}, False, Reason.ATTACHMENT_SOURCE_UNQUALIFIED),
        ({"stamp_ns": 1}, False, Reason.ATTACHMENT_EXPIRED),
        ({"request_id": "wrong-request"}, True, Reason.ATTACHMENT_REQUEST_MISMATCH),
        ({"validity": AttachmentValidity.INVALIDATED, "invalidation_reason": Reason.ATTACHMENT_REATTACH.value}, False, Reason.ATTACHMENT_REATTACH),
        ({"validity": AttachmentValidity.INVALIDATED, "invalidation_reason": Reason.ATTACHMENT_SLIP.value}, False, Reason.ATTACHMENT_SLIP),
    ],
)
def test_attachment_negatives_have_named_reasons(attachment_kwargs, route_from_original, reason):
    original = make_attachment("centered")
    attachment = make_attachment("centered", **attachment_kwargs)
    ctx = _context(attachment=original if route_from_original else attachment, now_ns=20_000_000_000)
    ctx = dataclasses.replace(ctx, attachment=attachment)
    result = TrajectoryCertifier().certify(ctx)
    assert result.reason is reason


def test_invalid_attachment_dimensions_and_quaternion_are_rejected_by_contract():
    with pytest.raises(Exception):
        make_attachment("centered", dimensions_wdh=(0.5, 0.4, 0.0))
    base = make_attachment("centered")
    with pytest.raises(Exception):
        dataclasses.replace(base, t_tcp_box_quaternion_xyzw=(0.0, 0.0, 0.0, 2.0))
