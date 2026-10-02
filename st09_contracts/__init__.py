"""ST09 shared ROS-free contracts (C0 freeze).

Single definition module for the ST09 parallel campaign (plan revision
``docs/plans/frame_migration_st09_parallel_execution.md`` 2026-10-02).
Every module task (A/B/C/D/E/Q/I/R) consumes these types through adapters
in its own package; copying the definitions into task-local code is a
contract violation. Only C0 and I may modify this package (see the file
owner manifest in ``st09_acceptance_manifest.json``).

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

from st09_contracts.attachment import (
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
from st09_contracts.certification import (
    CertificationResult,
    CheckDomain,
    MapRevalidationDecision,
    MapDecision,
    EffectTrace,
    EffectCounters,
)
from st09_contracts.graph import (
    GraphEdge,
    GraphNode,
    LoadState,
    TimedTrajectory,
    PlannedRoute,
)
from st09_contracts.planning import (
    OccupancyCellState,
    PlanningSnapshot,
    PayloadIdentity,
    PayloadSymmetry,
    PlacementCandidate,
    GeometryVerdict,
    ObservationRequirement,
)
from st09_contracts.protocols import (
    CandidateGenerator,
    RoutePlanner,
    Certifier,
    ViewIntegrator,
)
from st09_contracts.reasons import (
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
