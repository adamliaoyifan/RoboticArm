from __future__ import annotations

import dataclasses
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "luggage_planning"))

from luggage_planning.map_revalidation import MapRevalidator, RevalidationInput, StopPolicy
from st09_contracts.certification import EffectCounters, MapDecision
from st09_contracts.fixtures import make_attachment, make_graph_fixture, make_route, make_snapshot
from st09_contracts.reasons import Reason


def _edge_with_cells(edge, cells=((1, 1),)):
    return dataclasses.replace(edge, attributes={"occupancy_cells": tuple(cells)})


def _request(latest, *, in_flight=False, now_ns=2_100_000_000, runtime=True, start=True,
             cells=((1, 1),)):
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
        start_joints_match=start,
    )


def test_unrelated_update_continues_after_revalidation():
    latest = make_snapshot(map_revision=3, observed_free=[(1, 1), (5, 5)], acquisition_stamp_ns=2_050_000_000)
    decision = MapRevalidator().revalidate(_request(latest))
    assert decision.decision is MapDecision.CONTINUE
    assert decision.reason is Reason.OK


def test_related_update_requires_replan_before_send():
    latest = make_snapshot(map_revision=3, occupied=[(12, 1)], acquisition_stamp_ns=2_050_000_000)
    decision = MapRevalidator().revalidate(_request(latest, cells=((12, 1),)))
    assert decision.decision is MapDecision.REPLAN
    assert decision.affected_edge_ids


def test_in_flight_conflict_stops_and_enters_carry_fault():
    latest = make_snapshot(map_revision=3, occupied=[(12, 1)], acquisition_stamp_ns=2_050_000_000)
    decision = MapRevalidator().revalidate(_request(latest, in_flight=True, cells=((12, 1),)))
    assert decision.decision is MapDecision.STOP
    assert decision.reason is Reason.CARRY_FAULT


def test_stale_restart_and_start_joint_mismatch_fail_closed():
    stale = make_snapshot(map_revision=3, observed_free=[(1, 1)], acquisition_stamp_ns=1)
    assert MapRevalidator().revalidate(_request(stale, now_ns=4_000_000_000)).reason is Reason.MAP_STALE
    restart = make_snapshot(map_revision=1, observed_free=[(1, 1)], acquisition_stamp_ns=2_050_000_000)
    assert MapRevalidator().revalidate(_request(restart)).reason is Reason.MAP_RESTART
    latest = make_snapshot(map_revision=3, observed_free=[(1, 1)], acquisition_stamp_ns=2_050_000_000)
    assert MapRevalidator().revalidate(_request(latest, start=False)).reason is Reason.CERT_JOINT_DISCONTINUITY


def test_controlled_stop_allows_only_cancel_and_hold_business_delta_zero():
    before = EffectCounters(plan_calls=1, execute_calls=1, attach_calls=1)
    policy = StopPolicy(deadline_ns=250)
    trace = policy.controlled_stop_trace(
        request_id="req", route_id="route", before=before,
        detected_after_ns=1000, stopped_after_ns=1200)
    assert policy.stop_meets_deadline(1000, 1200)
    assert trace.after.cancelled_goals == before.cancelled_goals + 1
    assert trace.after.held_goals == before.held_goals + 1
    assert all(value == 0 for value in trace.business_effect_delta.values())
