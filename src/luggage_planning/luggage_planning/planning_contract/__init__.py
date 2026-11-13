"""Frozen ROS-free planning records.

Shared attachment, route, certification, and reason types for the live
planning chain. Copying these definitions into a task package is a contract
violation. The resolved manifest is ``config/planning_contract``.
Orchestration and exploration records stay in ``luggage_planning.contracts``.

The package is intentionally ROS-free: stdlib plus numpy only. No rclpy,
no generated messages, no TF, no MoveIt, no GT.

Freeze rules (plan section 3):

- Immutable value objects: arrays are tuples; mutation raises.
- NaN and non-finite floats are rejected at construction.
- ``q``/``-q`` and ``pi``/``-pi`` are representation equivalences, never
  rejection reasons on their own.
- IK family labels are search/diagnostic attributes; refusals come from
  real continuity and safety constraints.
- Graph reachability priors are not free-space or support evidence, and
  map ``unknown`` is never promoted to ``free``.
"""

from luggage_planning.planning_contract.attachment import (
    ATTACHMENT_SCHEMA_VERSION,
    AttachmentRecord,
    AttachmentSourceKind,
    AttachmentProvider,
    AttachmentInvalidation,
    AttachmentValidity,
    BoxTarget,
    PlaceTarget,
    resolve_place_targets,
    TargetResolutionFailure,
)
from luggage_planning.planning_contract.certification import (
    CertificationResult,
    CheckDomain,
    MapRevalidationDecision,
    MapDecision,
    EffectTrace,
    EffectCounters,
)
from luggage_planning.planning_contract.graph import (
    GraphEdge,
    GraphNode,
    LoadState,
    TimedTrajectory,
    PlannedRoute,
)
from luggage_planning.planning_contract.planning import (
    OccupancyCellState,
    PlanningSnapshot,
    PayloadIdentity,
    PayloadSymmetry,
    PlacementCandidate,
    GeometryVerdict,
    ObservationRequirement,
)
from luggage_planning.planning_contract.protocols import (
    CandidateGenerator,
    RoutePlanner,
    Certifier,
    ViewIntegrator,
)
from luggage_planning.planning_contract.reasons import (
    Reason,
    LegacyCode,
    LEGACY_TO_REASON,
)

__all__ = [
    "ATTACHMENT_SCHEMA_VERSION",
    "AttachmentRecord",
    "AttachmentSourceKind",
    "AttachmentProvider",
    "AttachmentInvalidation",
    "AttachmentValidity",
    "BoxTarget",
    "PlaceTarget",
    "resolve_place_targets",
    "TargetResolutionFailure",
    "CertificationResult",
    "CheckDomain",
    "MapRevalidationDecision",
    "MapDecision",
    "EffectTrace",
    "EffectCounters",
    "GraphEdge",
    "GraphNode",
    "LoadState",
    "TimedTrajectory",
    "PlannedRoute",
    "OccupancyCellState",
    "PlanningSnapshot",
    "PayloadIdentity",
    "PayloadSymmetry",
    "PlacementCandidate",
    "GeometryVerdict",
    "ObservationRequirement",
    "CandidateGenerator",
    "RoutePlanner",
    "Certifier",
    "ViewIntegrator",
    "Reason",
    "LegacyCode",
    "LEGACY_TO_REASON",
]
