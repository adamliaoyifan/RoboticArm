"""Map revalidation and controlled-stop policy for ST09-C."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Callable, Mapping, Sequence, Tuple

from st09_contracts.certification import (
    EffectCounters,
    EffectTrace,
    MapDecision,
    MapRevalidationDecision,
)
from st09_contracts.graph import GraphEdge, PlannedRoute
from st09_contracts.planning import OccupancyCellState, PlanningSnapshot
from st09_contracts.reasons import Reason


@dataclass(frozen=True)
class RevalidationPolicy:
    max_snapshot_age_ns: int = 2_000_000_000
    stop_deadline_ns: int = 250_000_000
    validation_epoch: int = 1


@dataclass(frozen=True)
class RevalidationInput:
    route: PlannedRoute
    planned_snapshot: PlanningSnapshot
    latest_snapshot: PlanningSnapshot
    edges: Tuple[GraphEdge, ...]
    now_ns: int
    in_flight: bool = False
    runtime_lease_valid: bool = True
    start_joints_match: bool = True
    #: An unloaded observation move may run while an occupancy change is
    #: unconfirmed: the move to the observation pose is what confirms it.
    #: Placement and every loaded route keep this False.
    allow_unconfirmed: bool = False


def freshness_verdict(snapshot, now_ns, max_age_ns, allow_unconfirmed=False):
    """Whether this map identity may admit a route.

    A confirmed occupancy snapshot stays valid until a change event or
    a contradictory observation. The acquisition stamp stays the real
    sensor time and is recorded as ``age_ns``. Age past
    ``max_age_ns`` does not refuse that snapshot. ``allow_unconfirmed``
    admits an unconfirmed snapshot for an unloaded observation route only.
    Stamp 0 stays "no sensor acquisition": an unloaded observation of a
    ``release_integrated`` commit may run before any view exists, and that
    admission does not write or require a stamp. Every other route still
    refuses stamp <= 0. ``static_empty`` is the configured empty container
    and is legal only while nothing has been placed and no cargo view has
    been integrated.
    """
    source = str(getattr(snapshot, "map_source", "") or "")
    placed = int(getattr(snapshot, "placed_count", 0) or 0)
    observed = bool(getattr(snapshot, "cargo_observed", False))
    stamp = int(getattr(snapshot, "acquisition_stamp_ns", 0) or 0)
    confirmed = bool(getattr(snapshot, "occupancy_confirmed", True))
    verdict = {
        "map_source": source,
        "placed_count": placed,
        "last_acquisition_stamp_ns": stamp,
        "cargo_observed": observed,
        "occupancy_confirmed": confirmed,
        "change_event": str(getattr(snapshot, "change_event", "") or ""),
        "freshness_policy": "",
        "why_allowed": "",
        "why_refused": "",
        "allowed": False,
        "age_ns": None,
        "age_exceeds_window": False,
    }
    if source == "static_empty":
        verdict["freshness_policy"] = "static_empty"
        if placed == 0 and not observed:
            verdict["allowed"] = True
            verdict["why_allowed"] = "static_empty_unplaced_unobserved"
            return verdict
        verdict["why_refused"] = (
            "static_empty_after_place" if placed else
            "static_empty_after_observation")
        return verdict
    if source not in ("", "measured", "release_integrated"):
        verdict["freshness_policy"] = "reject_unknown_source"
        verdict["why_refused"] = "unknown_map_source"
        return verdict
    verdict["freshness_policy"] = "acquisition_stamp"
    verdict["acquisition_stamp_present"] = stamp > 0
    if stamp <= 0:
        if (allow_unconfirmed and source == "release_integrated"
                and placed > 0):
            verdict["freshness_policy"] = "commit_without_acquisition"
            verdict["allowed"] = True
            verdict["why_allowed"] = "unloaded_observation_before_acquisition"
            return verdict
        verdict["why_refused"] = "acquisition_stamp_required"
        return verdict
    age = int(now_ns) - stamp
    verdict["age_ns"] = age
    verdict["age_exceeds_window"] = age > int(max_age_ns)
    verdict["freshness_policy"] = "confirmed_occupancy"
    if not confirmed:
        if allow_unconfirmed:
            verdict["allowed"] = True
            verdict["why_allowed"] = "unconfirmed_unloaded_observation"
            return verdict
        verdict["why_refused"] = "occupancy_change_unconfirmed"
        return verdict
    verdict["allowed"] = True
    verdict["why_allowed"] = "confirmed_occupancy"
    return verdict


def admission_reason(freshness):
    """Map a freshness verdict to a refusal, or None when it may admit."""
    from st09_contracts.reasons import Reason

    if freshness.get("allowed"):
        return None
    if freshness.get("why_refused") == "occupancy_change_unconfirmed":
        return Reason.MAP_CONFIRMATION_REQUIRED
    return Reason.MAP_STALE


def _occupancy_admission(snapshot, reason):
    """Counts the revalidator actually compared, plus why.

    Unknown cells omitted from a sparse snapshot are not free and are
    not evidence that the container is full. ``why_allowed`` /
    ``why_refused`` here are the occupancy verdict, separate from the
    freshness strings.
    """
    counts = snapshot.occupancy_counts()
    known = int(snapshot.cell_count)
    occupied = int(counts.get("occupied", 0))
    domain_complete = bool(snapshot.occupancy_domain_complete)
    unknown_omitted = not domain_complete
    if reason is Reason.MAP_FULLY_BLOCKED:
        why_refused = "known_domain_fully_occupied"
        why_allowed = ""
    elif known == 0:
        why_refused = ""
        why_allowed = "empty_sparse_snapshot"
    elif occupied == known and unknown_omitted:
        why_refused = ""
        why_allowed = "known_cells_occupied_domain_incomplete"
    else:
        why_refused = ""
        why_allowed = "known_domain_not_fully_occupied"
    return {
        "known_cell_count": known,
        "occupied_count": occupied,
        "unknown_omitted": unknown_omitted,
        "map_source": str(snapshot.map_source or ""),
        "occupancy_domain_complete": domain_complete,
        "why_allowed": why_allowed,
        "why_refused": why_refused,
    }


def _known_domain_fully_occupied(snapshot) -> bool:
    """True only for a declared complete domain that is entirely occupied.

    ``cell_count == 0`` is an empty sparse snapshot (unknown omitted),
    not a full container. Listed cells that are all occupied still do
    not fill the container when unknown cells were left out.
    """
    if not snapshot.occupancy_domain_complete:
        return False
    known = snapshot.cell_count
    occupied = snapshot.occupancy_counts()["occupied"]
    return known > 0 and occupied == known


class MapRevalidator:
    """Decide whether a fresh map can continue, must replan, or must stop."""

    def __init__(self, policy: RevalidationPolicy | None = None):
        self.policy = policy or RevalidationPolicy()

    def revalidate(self, request: RevalidationInput) -> MapRevalidationDecision:
        latest = request.latest_snapshot
        freshness = freshness_verdict(
            latest, request.now_ns, self.policy.max_snapshot_age_ns,
            allow_unconfirmed=request.allow_unconfirmed)
        reason = self._preflight_reason(request, freshness)
        if reason is not Reason.OK:
            decision = MapDecision.STOP if request.in_flight else MapDecision.FAULT
            return self._decision(decision, reason, latest, (), freshness)
        affected = self._affected_edges(request.edges, latest)
        if affected:
            if request.in_flight:
                return self._decision(
                    MapDecision.STOP, Reason.CARRY_FAULT, latest, affected,
                    freshness)
            return self._decision(
                MapDecision.REPLAN, Reason.MAP_REVISION_MISMATCH, latest,
                affected, freshness)
        return self._decision(
            MapDecision.CONTINUE, Reason.OK, latest, (), freshness)

    def _preflight_reason(self, request: RevalidationInput, freshness) -> Reason:
        planned, latest = request.planned_snapshot, request.latest_snapshot
        if not request.runtime_lease_valid:
            return Reason.MAP_RESTART
        if not request.start_joints_match:
            return Reason.CERT_JOINT_DISCONTINUITY
        if latest.geometry_hash != planned.geometry_hash:
            return Reason.MAP_GEOMETRY_HASH_MISMATCH
        planned_heights = str(
            getattr(planned, "column_height_digest", "") or "")
        latest_heights = str(
            getattr(latest, "column_height_digest", "") or "")
        # Same revision and the same hull hash still leave column tops
        # free to change. Both sides must have a digest; a legacy
        # snapshot with none is not treated as a conflict.
        if (planned_heights and latest_heights
                and planned_heights != latest_heights):
            return Reason.MAP_HASH_CONFLICT
        if latest.map_revision < planned.map_revision or latest.commit_revision < planned.commit_revision:
            return Reason.MAP_RESTART
        if not freshness["allowed"]:
            return admission_reason(freshness) or Reason.MAP_STALE
        if _known_domain_fully_occupied(latest):
            return Reason.MAP_FULLY_BLOCKED
        return Reason.OK

    @staticmethod
    def _affected_edges(edges: Sequence[GraphEdge], snapshot: PlanningSnapshot) -> Tuple[str, ...]:
        affected = []
        for edge in edges:
            for cell in edge.attributes.get("occupancy_cells", ()):
                state = snapshot.state_at(int(cell[0]), int(cell[1]))
                if state in (OccupancyCellState.OCCUPIED, OccupancyCellState.UNKNOWN):
                    affected.append(edge.edge_id)
                    break
        return tuple(affected)

    def _decision(self, decision: MapDecision, reason: Reason, snapshot: PlanningSnapshot,
                  affected: Tuple[str, ...], freshness) -> MapRevalidationDecision:
        return MapRevalidationDecision(
            decision=decision,
            reason=reason,
            map_revision=snapshot.map_revision,
            validation_epoch=self.policy.validation_epoch,
            affected_edge_ids=affected,
            detail={
                "policy": "st09-c-map-revalidation",
                "freshness": dict(freshness),
                "occupancy": _occupancy_admission(snapshot, reason),
            },
        )


@dataclass(frozen=True)
class StopPolicy:
    deadline_ns: int = 250_000_000

    def controlled_stop_trace(self, *, request_id: str, route_id: str, before: EffectCounters,
                              detected_after_ns: int, stopped_after_ns: int) -> EffectTrace:
        after = EffectCounters(
            plan_calls=before.plan_calls,
            execute_calls=before.execute_calls,
            attach_calls=before.attach_calls,
            detach_calls=before.detach_calls,
            scene_mutations=before.scene_mutations,
            map_commits=before.map_commits,
            cancelled_goals=before.cancelled_goals + 1,
            held_goals=before.held_goals + 1,
        )
        return EffectTrace(
            trace_id="stop-%s" % route_id,
            causal_sequence=detected_after_ns,
            request_id=request_id,
            route_id=route_id,
            phase="in_flight_conflict_stop",
            stamp_ns=stopped_after_ns,
            before=before,
            after=after,
            allowed_cancel_hold=True,
        )

    def stop_meets_deadline(self, detected_after_ns: int, stopped_after_ns: int) -> bool:
        return 0 <= stopped_after_ns - detected_after_ns <= self.deadline_ns


class MockControllerStopProbe:
    """Measure the mock cancel/hold path used by ST09-C acceptance tests."""

    def __init__(self, clock_ns: Callable[[], int] | None = None):
        self._clock_ns = clock_ns or time.perf_counter_ns
        self.traces = []

    def cancel_and_hold(self, request_id: str, route_id: str, before: EffectCounters,
                        policy: StopPolicy | None = None) -> EffectTrace:
        policy = policy or StopPolicy()
        detected = self._clock_ns()
        after_cancel = self._clock_ns()
        trace = policy.controlled_stop_trace(
            request_id=request_id,
            route_id=route_id,
            before=before,
            detected_after_ns=detected,
            stopped_after_ns=after_cancel)
        self.traces.append({
            "request_id": request_id,
            "route_id": route_id,
            "detected_ns": detected,
            "stopped_ns": after_cancel,
            "latency_ns": after_cancel - detected,
            "deadline_ns": policy.deadline_ns,
            "business_effect_delta": dict(trace.business_effect_delta),
            "cancel_delta": trace.after.cancelled_goals - trace.before.cancelled_goals,
            "hold_delta": trace.after.held_goals - trace.before.held_goals,
        })
        return trace


__all__ = [
    "MapRevalidator",
    "MockControllerStopProbe",
    "RevalidationInput",
    "RevalidationPolicy",
    "StopPolicy",
]
