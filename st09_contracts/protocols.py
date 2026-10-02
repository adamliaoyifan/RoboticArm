"""Frozen call protocols between the ST09 module tasks (plan section 3.5).

These typing protocols are the boundary contract A/B/C/D implement and
E records. Each protocol is ROS-free; the module-side adapter converts
between these types and its package's ROS types. I is the only task
allowed to change this file (after C0) and must then publish a new
shared commit and record the affected tasks.
"""

from __future__ import annotations

from typing import Mapping, Optional, Sequence, Tuple

from st09_contracts.attachment import (
    AttachmentInvalidation,
    AttachmentRecord,
    PlaceTarget,
)
from st09_contracts.certification import (
    CertificationResult,
    MapRevalidationDecision,
)
from st09_contracts.graph import PlannedRoute
from st09_contracts.planning import (
    ObservationRequirement,
    PlacementCandidate,
    PlanningSnapshot,
)
from st09_contracts.reasons import Reason


class CandidateGenerator:
    """ST09-A: placement geometry and ordering (pure; no motion requests).

    ``generate_candidates`` consumes the immutable snapshot plus the
    payload identity and returns every candidate with its geometry
    verdict and the observation requirement for the areas it depends
    on. It never issues motion requests and never reports BIN_FULL from
    a single failed insertion corridor.
    """

    def generate_candidates(
        self, snapshot: PlanningSnapshot, payload_digest: str
    ) -> Tuple[Sequence[PlacementCandidate], Optional[ObservationRequirement]]:
        raise NotImplementedError


class RoutePlanner:
    """ST09-B: directed-graph search and MoveIt edge completion.

    ``plan_route`` returns an *unauthorized* route (admission happens in
    the executor) or raises with a named :class:`Reason` failure —
    budget exhaustion is ``PLANNING_BUDGET_EXHAUSTED``, never BIN_FULL.
    """

    def plan_route(
        self,
        snapshot: PlanningSnapshot,
        attachment: AttachmentRecord,
        candidate: PlacementCandidate,
        targets: Sequence[PlaceTarget],
        budget_ns: int,
    ) -> PlannedRoute:
        raise NotImplementedError


class Certifier:
    """ST09-C: whole-trajectory certification and map revalidation.

    Unknown inputs and timeouts fail closed. ``certify`` covers the full
    robot + payload model; a TCP sweep is at most a conservative
    pre-filter. ``revalidate`` decides what a map update means for an
    admitted route before and during execution.
    """

    def certify(self, route: PlannedRoute) -> CertificationResult:
        raise NotImplementedError

    def revalidate(
        self, route: PlannedRoute, snapshot: PlanningSnapshot
    ) -> MapRevalidationDecision:
        raise NotImplementedError


class ViewIntegrator:
    """ST09-D: bounded observation and asynchronous cargo-map integration.

    ``integrate_view`` consumes saved sensor data plus the observation
    requirement and returns the *new* snapshot (immutable; integration
    stays asynchronous and idempotent). Missing sensors never promote
    cells to free or support.
    """

    def integrate_view(
        self,
        snapshot: PlanningSnapshot,
        requirement: ObservationRequirement,
        view_payload: Mapping,
    ) -> Tuple[PlanningSnapshot, Optional[AttachmentInvalidation]]:
        raise NotImplementedError


__all__ = [
    "CandidateGenerator",
    "RoutePlanner",
    "Certifier",
    "ViewIntegrator",
    "Reason",
]
