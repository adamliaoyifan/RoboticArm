from __future__ import annotations

import dataclasses
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "luggage_planning"))

from luggage_planning.place.map_revalidation import MapRevalidator, RevalidationInput, StopPolicy
from luggage_planning.planning_contract.certification import EffectCounters, MapDecision
from luggage_planning.planning_contract.fixtures import make_attachment, make_graph_fixture, make_route, make_snapshot
from luggage_planning.planning_contract.reasons import Reason


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


def test_same_revision_with_different_column_heights_conflicts():
    latest = dataclasses.replace(
        make_snapshot(map_revision=2, observed_free=[(1, 1)],
                      acquisition_stamp_ns=2_000_000_000),
        column_height_digest="height-b")
    request = _request(latest, now_ns=2_100_000_000)
    planned = dataclasses.replace(
        request.planned_snapshot, column_height_digest="height-a")
    request = dataclasses.replace(
        request, planned_snapshot=planned, latest_snapshot=latest)
    decision = MapRevalidator().revalidate(request)
    assert decision.reason is Reason.MAP_HASH_CONFLICT
    assert decision.reason is not Reason.MAP_GEOMETRY_HASH_MISMATCH


def test_matching_column_heights_stay_admissible():
    latest = dataclasses.replace(
        make_snapshot(map_revision=2, observed_free=[(1, 1)],
                      acquisition_stamp_ns=2_000_000_000),
        column_height_digest="height-a")
    request = _request(latest, now_ns=2_100_000_000)
    planned = dataclasses.replace(
        request.planned_snapshot, column_height_digest="height-a")
    request = dataclasses.replace(
        request, planned_snapshot=planned, latest_snapshot=latest)
    decision = MapRevalidator().revalidate(request)
    assert decision.reason is Reason.OK


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


def _identity(stamp_ns, source, placed=0, observed=False):
    return dataclasses.replace(
        make_snapshot(map_revision=3, observed_free=[(1, 1)],
                      acquisition_stamp_ns=stamp_ns),
        map_source=source, placed_count=placed, cargo_observed=observed)


def test_static_empty_first_box_skips_the_age_window():
    latest = _identity(0, "static_empty")
    decision = MapRevalidator().revalidate(
        _request(latest, now_ns=40_000_000_000))
    assert decision.decision is MapDecision.CONTINUE
    freshness = decision.detail["freshness"]
    assert freshness["freshness_policy"] == "static_empty"
    assert freshness["why_allowed"] == "static_empty_unplaced_unobserved"
    assert freshness["placed_count"] == 0
    assert freshness["last_acquisition_stamp_ns"] == 0


def test_static_empty_after_place_or_observation_is_refused():
    placed = _identity(0, "static_empty", placed=1)
    observed = _identity(0, "static_empty", observed=True)
    for latest in (placed, observed):
        decision = MapRevalidator().revalidate(
            _request(latest, now_ns=40_000_000_000))
        assert decision.reason is Reason.MAP_STALE
        assert decision.detail["freshness"]["why_allowed"] == ""
        assert decision.detail["freshness"]["why_refused"]


def test_measured_or_release_without_stamp_is_refused():
    for source, placed in (("measured", 0), ("release_integrated", 1)):
        latest = _identity(0, source, placed=placed, observed=(source == "measured"))
        decision = MapRevalidator().revalidate(
            _request(latest, now_ns=40_000_000_000))
        assert decision.reason is Reason.MAP_STALE, source
        assert decision.detail["freshness"]["why_refused"] == \
            "acquisition_stamp_required"


def test_unlabeled_stamp_zero_is_not_a_fresh_sensor_map():
    latest = make_snapshot(
        map_revision=3, observed_free=[(1, 1)], acquisition_stamp_ns=0)
    decision = MapRevalidator().revalidate(
        _request(latest, now_ns=40_000_000_000))
    assert decision.reason is Reason.MAP_STALE
    assert decision.detail["freshness"]["why_refused"] == \
        "acquisition_stamp_required"


def test_stamp_inside_the_window_continues():
    latest = _identity(39_000_000_000, "measured", observed=True)
    decision = MapRevalidator().revalidate(
        _request(latest, now_ns=40_000_000_000))
    assert decision.decision is MapDecision.CONTINUE
    assert decision.detail["freshness"]["why_allowed"] == \
        "confirmed_occupancy"


def test_confirmed_map_remains_valid_after_the_age_window():
    latest = _identity(1_000_000_000, "measured", observed=True)
    decision = MapRevalidator().revalidate(
        _request(latest, now_ns=40_000_000_000))
    assert decision.decision is MapDecision.CONTINUE
    assert decision.reason is Reason.OK
    freshness = decision.detail["freshness"]
    assert freshness["age_exceeds_window"] is True
    assert freshness["last_acquisition_stamp_ns"] == 1_000_000_000
    assert freshness["why_allowed"] == "confirmed_occupancy"


def test_change_event_requires_reconfirmation():
    latest = dataclasses.replace(
        _identity(39_000_000_000, "measured", observed=True),
        occupancy_confirmed=False, change_event="box_placed")
    decision = MapRevalidator().revalidate(
        _request(latest, now_ns=40_000_000_000))
    assert decision.reason is Reason.MAP_CONFIRMATION_REQUIRED
    assert decision.detail["freshness"]["why_refused"] == \
        "occupancy_change_unconfirmed"
    assert decision.detail["freshness"]["change_event"] == "box_placed"


def _pending(stamp_ns=39_000_000_000, source="release_integrated", placed=1):
    return dataclasses.replace(
        _identity(stamp_ns, source, placed=placed),
        occupancy_confirmed=False, change_event="box_placed")


def test_unloaded_observation_route_runs_while_a_change_is_pending():
    """The move to the observation pose is what confirms the change."""
    request = dataclasses.replace(
        _request(_pending(), now_ns=40_000_000_000), allow_unconfirmed=True)
    decision = MapRevalidator().revalidate(request)
    assert decision.decision is MapDecision.CONTINUE
    freshness = decision.detail["freshness"]
    assert freshness["why_allowed"] == "unconfirmed_unloaded_observation"
    assert freshness["occupancy_confirmed"] is False


def test_pending_change_still_refuses_every_other_route():
    decision = MapRevalidator().revalidate(
        _request(_pending(), now_ns=40_000_000_000))
    assert decision.reason is Reason.MAP_CONFIRMATION_REQUIRED


def test_unloaded_observation_does_not_require_a_forged_acquisition_stamp():
    """Commit time lives on the pending change. Stamp 0 stays 'no acquisition'."""
    request = dataclasses.replace(
        _request(_pending(stamp_ns=0), now_ns=40_000_000_000),
        allow_unconfirmed=True)
    decision = MapRevalidator().revalidate(request)
    assert decision.decision is MapDecision.CONTINUE
    freshness = decision.detail["freshness"]
    assert freshness["why_allowed"] == "unloaded_observation_before_acquisition"
    assert freshness["last_acquisition_stamp_ns"] == 0
    assert freshness["acquisition_stamp_present"] is False


def test_missing_acquisition_stamp_still_refuses_placement_and_other_sources():
    assert MapRevalidator().revalidate(
        _request(_pending(stamp_ns=0), now_ns=40_000_000_000)
    ).reason is Reason.MAP_STALE
    measured = dataclasses.replace(
        _request(_pending(stamp_ns=0, source="measured"),
                 now_ns=40_000_000_000), allow_unconfirmed=True)
    assert MapRevalidator().revalidate(measured).reason is Reason.MAP_STALE
    static_after_place = dataclasses.replace(
        _request(_pending(stamp_ns=0, source="static_empty", placed=1),
                 now_ns=40_000_000_000), allow_unconfirmed=True)
    assert MapRevalidator().revalidate(
        static_after_place).reason is Reason.MAP_STALE


def test_release_integrated_stamp_inside_the_window_continues():
    latest = _identity(39_000_000_000, "release_integrated", placed=1)
    decision = MapRevalidator().revalidate(
        _request(latest, now_ns=40_000_000_000))
    assert decision.decision is MapDecision.CONTINUE
    assert decision.detail["freshness"]["map_source"] == "release_integrated"
    assert decision.detail["freshness"]["placed_count"] == 1


def test_stale_restart_and_start_joint_mismatch_fail_closed():
    old = make_snapshot(map_revision=3, observed_free=[(1, 1)], acquisition_stamp_ns=1)
    assert MapRevalidator().revalidate(
        _request(old, now_ns=4_000_000_000)).reason is Reason.OK
    restart = make_snapshot(map_revision=1, observed_free=[(1, 1)],
                            acquisition_stamp_ns=2_050_000_000)
    assert MapRevalidator().revalidate(_request(restart)).reason is Reason.MAP_RESTART
    latest = make_snapshot(map_revision=3, observed_free=[(1, 1)], acquisition_stamp_ns=2_050_000_000)
    assert MapRevalidator().revalidate(_request(latest, start=False)).reason is Reason.CERT_JOINT_DISCONTINUITY


def _empty_sparse(source, stamp_ns, placed=0, observed=False):
    return dataclasses.replace(
        _identity(stamp_ns, source, placed=placed, observed=observed),
        cells_i=(), cells_j=(), cell_states=(),
        occupancy_domain_complete=False)


def test_empty_static_snapshot_is_not_a_full_container():
    latest = _empty_sparse("static_empty", 0)
    decision = MapRevalidator().revalidate(
        _request(latest, cells=(), now_ns=40_000_000_000))
    assert decision.decision is MapDecision.CONTINUE
    assert decision.reason is Reason.OK
    occupancy = decision.detail["occupancy"]
    assert occupancy["known_cell_count"] == 0
    assert occupancy["occupied_count"] == 0
    assert occupancy["unknown_omitted"] is True
    assert occupancy["map_source"] == "static_empty"
    assert occupancy["why_allowed"] == "empty_sparse_snapshot"
    assert occupancy["why_refused"] == ""


def test_static_empty_after_place_is_not_an_empty_set_full_map():
    latest = _empty_sparse("static_empty", 0, placed=1)
    decision = MapRevalidator().revalidate(
        _request(latest, cells=(), now_ns=40_000_000_000))
    assert decision.decision is not MapDecision.CONTINUE
    assert decision.reason is Reason.MAP_STALE
    assert decision.reason is not Reason.MAP_FULLY_BLOCKED


def test_empty_measured_snapshot_follows_the_stamp_contract():
    missing = _empty_sparse("measured", 0, observed=True)
    refused = MapRevalidator().revalidate(
        _request(missing, cells=(), now_ns=40_000_000_000))
    assert refused.reason is Reason.MAP_STALE
    assert refused.reason is not Reason.MAP_FULLY_BLOCKED
    fresh = _empty_sparse("measured", 39_000_000_000, observed=True)
    admitted = MapRevalidator().revalidate(
        _request(fresh, cells=(), now_ns=40_000_000_000))
    assert admitted.decision is MapDecision.CONTINUE
    assert admitted.reason is Reason.OK


def test_empty_release_integrated_snapshot_follows_the_stamp_contract():
    missing = _empty_sparse("release_integrated", 0, placed=1)
    refused = MapRevalidator().revalidate(
        _request(missing, cells=(), now_ns=40_000_000_000))
    assert refused.reason is Reason.MAP_STALE
    assert refused.reason is not Reason.MAP_FULLY_BLOCKED
    fresh = _empty_sparse("release_integrated", 39_000_000_000, placed=1)
    admitted = MapRevalidator().revalidate(
        _request(fresh, cells=(), now_ns=40_000_000_000))
    assert admitted.decision is MapDecision.CONTINUE
    assert admitted.reason is Reason.OK


def test_complete_domain_of_occupied_cells_is_fully_blocked():
    dense = make_snapshot(
        map_revision=3, observed_free=[(1, 1)],
        acquisition_stamp_ns=2_050_000_000)
    latest = dataclasses.replace(
        dense,
        cell_states=tuple("occupied" for _ in dense.cell_states),
        occupancy_domain_complete=True,
        map_source="measured")
    decision = MapRevalidator().revalidate(
        _request(latest, cells=()))
    assert decision.reason is Reason.MAP_FULLY_BLOCKED
    occupancy = decision.detail["occupancy"]
    assert occupancy["known_cell_count"] > 0
    assert occupancy["occupied_count"] == occupancy["known_cell_count"]
    assert occupancy["unknown_omitted"] is False
    assert occupancy["why_refused"] == "known_domain_fully_occupied"


def test_sparse_occupied_cells_do_not_fill_the_container():
    latest = dataclasses.replace(
        make_snapshot(map_revision=3, acquisition_stamp_ns=2_050_000_000),
        map_source="measured",
        cells_i=(14,), cells_j=(5,), cell_states=("occupied",),
        occupancy_domain_complete=False)
    decision = MapRevalidator().revalidate(
        _request(latest, cells=()))
    assert decision.decision is MapDecision.CONTINUE
    assert decision.reason is not Reason.MAP_FULLY_BLOCKED
    occupancy = decision.detail["occupancy"]
    assert occupancy["occupied_count"] == occupancy["known_cell_count"] == 1
    assert occupancy["unknown_omitted"] is True
    assert occupancy["why_allowed"] == \
        "known_cells_occupied_domain_incomplete"


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
