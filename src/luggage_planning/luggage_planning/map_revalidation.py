"""Map revalidation and controlled-stop policy for ST09-C."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence, Tuple

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


class MapRevalidator:
    """Decide whether a fresh map can continue, must replan, or must stop."""

    def __init__(self, policy: RevalidationPolicy | None = None):
        self.policy = policy or RevalidationPolicy()

    def revalidate(self, request: RevalidationInput) -> MapRevalidationDecision:
        latest = request.latest_snapshot
        reason = self._preflight_reason(request)
        if reason is not Reason.OK:
            decision = MapDecision.STOP if request.in_flight else MapDecision.FAULT
            return self._decision(decision, reason, latest, ())
        affected = self._affected_edges(request.edges, latest)
        if affected:
            if request.in_flight:
                return self._decision(MapDecision.STOP, Reason.CARRY_FAULT, latest, affected)
            return self._decision(MapDecision.REPLAN, Reason.MAP_REVISION_MISMATCH, latest, affected)
        return self._decision(MapDecision.CONTINUE, Reason.OK, latest, ())

    def _preflight_reason(self, request: RevalidationInput) -> Reason:
        planned, latest = request.planned_snapshot, request.latest_snapshot
        if not request.runtime_lease_valid:
            return Reason.MAP_RESTART
        if not request.start_joints_match:
            return Reason.CERT_JOINT_DISCONTINUITY
        if latest.geometry_hash != planned.geometry_hash:
            return Reason.MAP_GEOMETRY_HASH_MISMATCH
        if latest.map_revision < planned.map_revision or latest.commit_revision < planned.commit_revision:
            return Reason.MAP_RESTART
        if request.now_ns - latest.acquisition_stamp_ns > self.policy.max_snapshot_age_ns:
            return Reason.MAP_STALE
        if latest.occupancy_counts()["occupied"] == latest.cell_count:
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
                  affected: Tuple[str, ...]) -> MapRevalidationDecision:
        return MapRevalidationDecision(
            decision=decision,
            reason=reason,
            map_revision=snapshot.map_revision,
            validation_epoch=self.policy.validation_epoch,
            affected_edge_ids=affected,
            detail={"policy": "st09-c-map-revalidation"},
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


__all__ = ["MapRevalidator", "RevalidationInput", "RevalidationPolicy", "StopPolicy"]
