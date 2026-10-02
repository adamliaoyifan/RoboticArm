"""Certification, map-revalidation and effect-trace value objects (C)."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping, Optional, Tuple

from st09_contracts._base import (
    ContractViolation,
    digest_of,
    digest_str,
    finite,
    nonempty_str,
    stamp_ns,
)
from st09_contracts.reasons import Reason


class CheckDomain(str, Enum):
    """What a certification actually covered (reported, never implied)."""

    SELF_COLLISION = "self_collision"
    STATIC_SCENE = "static_scene"
    ATTACHED_PAYLOAD = "attached_payload"
    PLACED_CARGO = "placed_cargo"
    OCCUPANCY_MAP = "occupancy_map"
    FK = "fk"
    JOINT_LIMITS = "joint_limits"
    JOINT_CONTINUITY = "joint_continuity"
    TIME_PARAMETERIZATION = "time_parameterization"
    ORIENTATION = "orientation"
    SUPPORT_CONTACT = "support_contact"

    def __str__(self) -> str:  # pragma: no cover - trivial
        return self.value


@dataclass(frozen=True)
class CertificationResult:
    """Structured verdict for one trajectory against one snapshot.

    ``passed`` is False for unknown inputs and timeouts (fail-closed).
    The report states sampling bounds; discrete samples are never sold
    as a continuous-safety proof.
    """

    certification_id: str
    route_id: str
    passed: bool
    reason: Reason
    checks: Tuple[str, ...]
    validation_revision: int
    min_clearance_m: Optional[float] = None
    fk_position_error_m: Optional[float] = None
    fk_rotation_error_rad: Optional[float] = None
    max_joint_velocity_rad: Optional[float] = None
    max_joint_acceleration_rad2: Optional[float] = None
    sampling_dt_max_ns: Optional[int] = None
    sampling_motion_bound_m: Optional[float] = None
    contact_reports: Tuple[Mapping[str, Any], ...] = ()
    method_note: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "certification_id", nonempty_str(self.certification_id, "certification_id"))
        object.__setattr__(self, "route_id", nonempty_str(self.route_id, "route_id"))
        if type(self.passed) is not bool:
            raise ContractViolation("passed: bool required (%r)" % (self.passed,))
        if not isinstance(self.reason, Reason):
            raise ContractViolation("reason: Reason required (%r)" % (self.reason,))
        if self.passed != (self.reason is Reason.OK):
            raise ContractViolation(
                "reason: OK iff passed (passed=%r reason=%s)" % (self.passed, self.reason))
        domains = []
        for name in self.checks:
            try:
                domains.append(str(CheckDomain(name)))
            except ValueError as exc:
                raise ContractViolation("checks: unknown domain %r" % (name,)) from exc
        object.__setattr__(self, "checks", tuple(domains))
        if isinstance(self.validation_revision, bool) or not isinstance(self.validation_revision, int) \
                or self.validation_revision < 0:
            raise ContractViolation("validation_revision: non-negative int required")
        for name, limit in (("min_clearance_m", None),
                            ("fk_position_error_m", None),
                            ("fk_rotation_error_rad", None),
                            ("max_joint_velocity_rad", None),
                            ("max_joint_acceleration_rad2", None),
                            ("sampling_motion_bound_m", None)):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, finite(value, name))
        if self.sampling_dt_max_ns is not None:
            object.__setattr__(
                self, "sampling_dt_max_ns", stamp_ns(self.sampling_dt_max_ns, "sampling_dt_max_ns"))
        if not isinstance(self.contact_reports, tuple):
            raise ContractViolation("contact_reports: immutable tuple required")
        object.__setattr__(self, "method_note", str(self.method_note or ""))

    def digest(self) -> str:
        return digest_of(
            self.certification_id, self.route_id, self.passed,
            str(self.reason), self.checks, self.validation_revision,
            self.min_clearance_m, self.fk_position_error_m,
            self.fk_rotation_error_rad, self.max_joint_velocity_rad,
            self.max_joint_acceleration_rad2, self.sampling_dt_max_ns,
            self.sampling_motion_bound_m, self.method_note)


class MapDecision(str, Enum):
    CONTINUE = "continue"
    REPLAN = "replan"
    STOP = "stop"
    FAULT = "fault"

    def __str__(self) -> str:  # pragma: no cover - trivial
        return self.value


@dataclass(frozen=True)
class MapRevalidationDecision:
    """What a map update means for an admitted or executing route."""

    decision: MapDecision
    reason: Reason
    map_revision: int
    validation_epoch: int
    affected_edge_ids: Tuple[str, ...]
    detail: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not isinstance(self.decision, MapDecision):
            raise ContractViolation("decision: MapDecision required (%r)" % (self.decision,))
        if not isinstance(self.reason, Reason):
            raise ContractViolation("reason: Reason required (%r)" % (self.reason,))
        for name in ("map_revision", "validation_epoch"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ContractViolation("%s: non-negative int required (%r)" % (name, value))
        if not isinstance(self.affected_edge_ids, tuple):
            raise ContractViolation("affected_edge_ids: immutable tuple required")
        for edge in self.affected_edge_ids:
            nonempty_str(edge, "affected_edge_ids")
        if not isinstance(self.detail, Mapping):
            raise ContractViolation("detail: mapping required")

    def digest(self) -> str:
        return digest_of(
            str(self.decision), str(self.reason), self.map_revision,
            self.validation_epoch, self.affected_edge_ids)


@dataclass(frozen=True)
class EffectCounters:
    """Business-effect counters (business effect is not dump I/O)."""

    plan_calls: int = 0
    execute_calls: int = 0
    attach_calls: int = 0
    detach_calls: int = 0
    scene_mutations: int = 0
    map_commits: int = 0
    cancelled_goals: int = 0
    held_goals: int = 0

    def __post_init__(self) -> None:
        for name in ("plan_calls", "execute_calls", "attach_calls",
                     "detach_calls", "scene_mutations", "map_commits",
                     "cancelled_goals", "held_goals"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ContractViolation("%s: non-negative int required (%r)" % (name, value))

    def business_delta(self, other: "EffectCounters") -> Mapping[str, int]:
        """Non-cancel/hold deltas: what the chain actually did."""
        keys = ("plan_calls", "execute_calls", "attach_calls",
                "detach_calls", "scene_mutations", "map_commits")
        return {k: getattr(other, k) - getattr(self, k) for k in keys}


@dataclass(frozen=True)
class EffectTrace:
    """Causal record of one request's effects across one phase."""

    trace_id: str
    causal_sequence: int
    request_id: str
    route_id: str
    phase: str
    stamp_ns: int
    before: EffectCounters
    after: EffectCounters
    allowed_cancel_hold: bool

    def __post_init__(self) -> None:
        object.__setattr__(self, "trace_id", nonempty_str(self.trace_id, "trace_id"))
        if isinstance(self.causal_sequence, bool) or not isinstance(self.causal_sequence, int) \
                or self.causal_sequence < 0:
            raise ContractViolation("causal_sequence: non-negative int required")
        object.__setattr__(self, "request_id", nonempty_str(self.request_id, "request_id"))
        object.__setattr__(self, "route_id", str(self.route_id or ""))
        object.__setattr__(self, "phase", nonempty_str(self.phase, "phase"))
        object.__setattr__(self, "stamp_ns", stamp_ns(self.stamp_ns, "stamp_ns"))
        if not isinstance(self.before, EffectCounters) or not isinstance(self.after, EffectCounters):
            raise ContractViolation("before/after: EffectCounters required")
        if type(self.allowed_cancel_hold) is not bool:
            raise ContractViolation("allowed_cancel_hold: bool required")

    @property
    def business_effect_delta(self) -> Mapping[str, int]:
        return self.before.business_delta(self.after)
