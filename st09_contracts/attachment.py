"""Attachment contract, provider protocol and pure place-target synthesis.

Implements technical baseline section 5.1 (plan revision 2026-10-02):

- ``T_a_b`` maps b coordinates into a coordinates. ``T_tcp_box`` is the
  full translation + unit quaternion taking box coordinates into the
  actual TCP frame; TCP local Z follows the real link definition.
- Dimensions are recorded along fixed box axes and never re-sorted.
- ``resolve_place_targets`` is pure: no TF, no MoveIt, no atlas, no GT,
  no route selection. ``T_world_tcp = T_world_box * inverse(T_tcp_box)``
  and the centre height never re-adds half the box height.
- theta / theta+pi are distinct physical rotations that share a
  symmetrical footprint; the expansion happens only when the payload
  model declares PI symmetry, and each heading gets its own TCP target
  through the actual attachment.
- ``q``/``-q`` and ``pi``/``-pi`` equivalences never reject on their own.

The single attachment provider (D) validates and freezes records after
attach confirmation. Sim GT relative transforms are reference-only and
can never register as a measured source.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping, Optional, Sequence, Tuple

from st09_contracts._base import (
    ContractViolation,
    digest_of,
    digest_str,
    finite,
    finite_tuple,
    nonempty_str,
    positive_dimensions,
    stamp_ns,
    unit_quaternion,
)
from st09_contracts.reasons import Reason
from st09_contracts.payload_envelope import PayloadEnvelope

ATTACHMENT_SCHEMA_VERSION = 2

#: Canonical attachment digest bound by explicitly-UNLOADED routes
#: (ST09-I S5.7-4): admission for observation/pick/exit legs is the
#: EXPLICIT LoadState.UNLOADED plus this digest — never an absent
#: AttachmentRecord silently read as "unloaded" (that stays
#: ATTACHMENT_MISSING).
UNLOADED_ATTACHMENT_DIGEST = digest_of("st09-attachment", "unloaded", 2)

#: Frozen attachment policy values (mirrored in resolved_planning_contract.json;
#: a C0 test asserts the two stay identical).
TF_SYNC_MAX_SKEW_NS = 0
ATTACHMENT_MAX_AGE_NS = 60_000_000_000  # 60 s for one box; n boxes use n times this
GRASP_TOLERANCE_M = 0.01
GRASP_TOLERANCE_RAD = 0.10
MEASURED_POSE_AGREEMENT_M = 1e-6
MEASURED_POSE_AGREEMENT_RAD = 1e-6
PURE_GEOMETRY_TOLERANCE = 1e-9


class AttachmentSourceKind(str, Enum):
    """Where the frozen transform came from (5.1)."""

    #: Measured box pose + actual TCP at confirmed attach. Ordinary sources
    #: are same-stamp; the explicitly qualified stationary-box source retains
    #: both real stamps and owner evidence separately. Route-grade after seal.
    MEASURED_ATTACH_CONFIRMED = "measured_attach_confirmed"
    #: Pre-attach measured pose used for contact-time estimation. Must
    #: carry its estimation method; legal only for pre-attach initial
    #: selection, never for a carried route.
    MEASURED_PRE_ATTACH_ESTIMATE = "measured_pre_attach_estimate"
    #: Sim backend GT relative transform. Reference validation only; a
    #: provider must refuse to publish it as a measured record.
    REFERENCE_SIM_GT = "reference_sim_gt"

    @property
    def measured(self) -> bool:
        return self in (AttachmentSourceKind.MEASURED_ATTACH_CONFIRMED,
                        AttachmentSourceKind.MEASURED_PRE_ATTACH_ESTIMATE)

    @property
    def route_grade(self) -> bool:
        return self is AttachmentSourceKind.MEASURED_ATTACH_CONFIRMED

    def __str__(self) -> str:  # pragma: no cover - trivial
        return self.value


class AttachmentValidity(str, Enum):
    VALID = "valid"
    INVALIDATED = "invalidated"
    SUPERSEDED = "superseded"

    def __str__(self) -> str:  # pragma: no cover - trivial
        return self.value


@dataclass(frozen=True)
class AttachmentRecord:
    """Frozen measured attachment (technical baseline 5.1 full field set).

    Axis semantics: ``dimensions_wdh`` are along the fixed box axes;
    ``t_tcp_box`` is the full rigid box->TCP transform. ``psi`` style
    yaw decompositions are diagnostics, never part of this contract.
    """

    schema_version: int
    cargo_id: str
    box_id: str
    request_id: str
    cycle_id: str
    attachment_incarnation: int
    tcp_frame: str
    box_frame: str
    dimensions_wdh: Tuple[float, float, float]
    symmetry: str
    t_tcp_box_translation: Tuple[float, float, float]
    t_tcp_box_quaternion_xyzw: Tuple[float, float, float, float]
    acquisition_stamp_ns: int
    scene_revision: str
    calibration_id: str
    source: AttachmentSourceKind
    source_method: str
    evidence_digest: str
    validity: AttachmentValidity
    invalidation_reason: Optional[str] = None
    collision_envelope: Optional[PayloadEnvelope] = None

    def __post_init__(self) -> None:
        if self.schema_version != ATTACHMENT_SCHEMA_VERSION:
            raise ContractViolation(
                "schema_version: %d required (%r)" % (ATTACHMENT_SCHEMA_VERSION, self.schema_version))
        for name in ("cargo_id", "box_id", "request_id", "cycle_id",
                     "tcp_frame", "box_frame", "scene_revision",
                     "calibration_id"):
            object.__setattr__(self, name, nonempty_str(getattr(self, name), name))
        if isinstance(self.attachment_incarnation, bool) or not isinstance(self.attachment_incarnation, int) \
                or self.attachment_incarnation < 0:
            raise ContractViolation(
                "attachment_incarnation: non-negative int required (%r)" % (self.attachment_incarnation,))
        object.__setattr__(
            self, "dimensions_wdh", positive_dimensions(self.dimensions_wdh, "dimensions_wdh"))
        object.__setattr__(self, "symmetry", nonempty_str(self.symmetry, "symmetry"))
        if self.symmetry not in ("none", "pi"):
            raise ContractViolation("symmetry: 'none' or 'pi' (%r)" % (self.symmetry,))
        object.__setattr__(
            self, "t_tcp_box_translation",
            finite_tuple(self.t_tcp_box_translation, 3, "t_tcp_box_translation"))
        object.__setattr__(
            self, "t_tcp_box_quaternion_xyzw",
            unit_quaternion(self.t_tcp_box_quaternion_xyzw, "t_tcp_box_quaternion_xyzw"))
        object.__setattr__(self, "acquisition_stamp_ns", stamp_ns(self.acquisition_stamp_ns, "acquisition_stamp_ns"))
        if not isinstance(self.source, AttachmentSourceKind):
            raise ContractViolation("source: AttachmentSourceKind required (%r)" % (self.source,))
        object.__setattr__(self, "source_method", nonempty_str(self.source_method, "source_method"))
        object.__setattr__(self, "evidence_digest", digest_str(self.evidence_digest, "evidence_digest"))
        if not isinstance(self.validity, AttachmentValidity):
            raise ContractViolation("validity: AttachmentValidity required (%r)" % (self.validity,))
        if self.validity is AttachmentValidity.VALID and self.invalidation_reason is not None:
            raise ContractViolation("invalidation_reason must be None while valid")
        if self.validity is not AttachmentValidity.VALID and not self.invalidation_reason:
            raise ContractViolation("invalidation_reason required when not valid")

        if self.collision_envelope is not None:
            env = self.collision_envelope
            if not isinstance(env, PayloadEnvelope) or env.nominal_digest != digest_of(
                    self.dimensions_wdh, self.t_tcp_box_translation,
                    self.t_tcp_box_quaternion_xyzw):
                raise ContractViolation("PAYLOAD_ENVELOPE_BINDING_MISMATCH")

    @property
    def usable_for_route(self) -> bool:
        return (
            self.validity is AttachmentValidity.VALID
            and self.source.route_grade)

    def digest(self) -> str:
        return digest_of(
            self.schema_version, self.cargo_id, self.box_id, self.request_id,
            self.cycle_id, self.attachment_incarnation, self.tcp_frame,
            self.box_frame, self.dimensions_wdh, self.symmetry,
            self.t_tcp_box_translation, self.t_tcp_box_quaternion_xyzw,
            self.acquisition_stamp_ns, self.scene_revision, self.calibration_id,
            str(self.source), self.source_method, self.evidence_digest,
            str(self.validity), self.invalidation_reason,
            self.collision_envelope.digest() if self.collision_envelope else None)

    def to_dict(self) -> Mapping[str, Any]:
        return {
            "schema_version": self.schema_version,
            "cargo_id": self.cargo_id,
            "box_id": self.box_id,
            "request_id": self.request_id,
            "cycle_id": self.cycle_id,
            "attachment_incarnation": self.attachment_incarnation,
            "tcp_frame": self.tcp_frame,
            "box_frame": self.box_frame,
            "dimensions_wdh": list(self.dimensions_wdh),
            "symmetry": self.symmetry,
            "t_tcp_box_translation": list(self.t_tcp_box_translation),
            "t_tcp_box_quaternion_xyzw": list(self.t_tcp_box_quaternion_xyzw),
            "acquisition_stamp_ns": self.acquisition_stamp_ns,
            "scene_revision": self.scene_revision,
            "calibration_id": self.calibration_id,
            "source": str(self.source),
            "source_method": self.source_method,
            "evidence_digest": self.evidence_digest,
            "validity": str(self.validity),
            "invalidation_reason": self.invalidation_reason,
            "collision_envelope": self.collision_envelope.to_dict() if self.collision_envelope else None,
        }


@dataclass(frozen=True)
class AttachmentInvalidation:
    """Notification that a previously frozen record stopped being valid."""

    record_digest: str
    box_id: str
    request_id: str
    incarnation: int
    reason: Reason
    stamp_ns: int
    detail: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "record_digest", digest_str(self.record_digest, "record_digest"))
        object.__setattr__(self, "box_id", nonempty_str(self.box_id, "box_id"))
        object.__setattr__(self, "request_id", nonempty_str(self.request_id, "request_id"))
        if isinstance(self.incarnation, bool) or not isinstance(self.incarnation, int):
            raise ContractViolation("incarnation: int required")
        if not isinstance(self.reason, Reason):
            raise ContractViolation("reason: Reason required (%r)" % (self.reason,))
        object.__setattr__(self, "stamp_ns", stamp_ns(self.stamp_ns, "stamp_ns"))


class AttachmentProvider:
    """Frozen provider protocol (D implements; I wires the ROS side).

    One provider instance owns the attach records for the active
    request. The provider and the cargo-map integrator are separate
    modules by contract — a provider must not grow into a map manager.

    Implementations must:

    - publish exactly one record per valid observation + attach
      confirmation; duplicate events are idempotent (same digest);
    - allocate a new ``attachment_incarnation`` on every re-attach;
    - refuse to publish REFERENCE_SIM_GT sources as measured;
    - compute ``T_tcp_box = inverse(T_world_tcp) * T_world_box`` from
      the same-stamp measured box pose and TCP TF (skew <=
      ``TF_SYNC_MAX_SKEW_NS``);
    - freeze after attach confirmation and notify invalidation via
      :meth:`invalidate` for slip, re-attach, frame/calibration change
      or expiry per the frozen policy.
    """

    def latest_record(self, request_id: str) -> AttachmentRecord:
        """Return the frozen valid record for the request or raise
        :class:`ContractViolation` carrying a :class:`Reason`-valued
        code (never a fake valid record)."""
        raise NotImplementedError

    def invalidate(self, reason: Reason, detail: Mapping[str, Any]) -> AttachmentInvalidation:
        """Mark the active record invalid and emit the notification.

        Dependents (routes, admissions) treat the returned notification
        as fail-closed: carry-fault, vacuum retained.
        """
        raise NotImplementedError


@dataclass(frozen=True)
class BoxTarget:
    """Desired box pose (world/container frame) for one heading.

    ``stamp_ns`` is the planning time at which this target is asserted;
    expiry checks compare it against the attachment acquisition stamp.
    """

    candidate_id: str
    request_id: str
    frame_id: str
    scene_revision: str
    heading_theta: float
    position: Tuple[float, float, float]
    quaternion_xyzw: Tuple[float, float, float, float]
    stamp_ns: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "candidate_id", nonempty_str(self.candidate_id, "candidate_id"))
        object.__setattr__(self, "request_id", nonempty_str(self.request_id, "request_id"))
        object.__setattr__(self, "frame_id", nonempty_str(self.frame_id, "frame_id"))
        object.__setattr__(self, "scene_revision", nonempty_str(self.scene_revision, "scene_revision"))
        object.__setattr__(self, "heading_theta", finite(self.heading_theta, "heading_theta"))
        object.__setattr__(self, "position", finite_tuple(self.position, 3, "position"))
        object.__setattr__(self, "quaternion_xyzw", unit_quaternion(self.quaternion_xyzw, "quaternion_xyzw"))
        object.__setattr__(self, "stamp_ns", stamp_ns(self.stamp_ns, "stamp_ns"))


@dataclass(frozen=True)
class PlaceTarget:
    """Resolved full TCP target for one box heading."""

    candidate_id: str
    heading_theta: float
    tcp_frame: str
    frame_id: str
    position: Tuple[float, float, float]
    quaternion_xyzw: Tuple[float, float, float, float]
    attachment_digest: str
    note: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "candidate_id", nonempty_str(self.candidate_id, "candidate_id"))
        object.__setattr__(self, "heading_theta", finite(self.heading_theta, "heading_theta"))
        object.__setattr__(self, "tcp_frame", nonempty_str(self.tcp_frame, "tcp_frame"))
        object.__setattr__(self, "frame_id", nonempty_str(self.frame_id, "frame_id"))
        object.__setattr__(self, "position", finite_tuple(self.position, 3, "position"))
        object.__setattr__(self, "quaternion_xyzw", unit_quaternion(self.quaternion_xyzw, "quaternion_xyzw"))
        object.__setattr__(self, "attachment_digest", digest_str(self.attachment_digest, "attachment_digest"))


@dataclass(frozen=True)
class TargetResolutionFailure:
    """Structured refusal from :func:`resolve_place_targets`."""

    reason: Reason
    field_name: str
    expected: str
    received: str

    def __post_init__(self) -> None:
        if not isinstance(self.reason, Reason):
            raise ContractViolation("reason: Reason required (%r)" % (self.reason,))
        for name in ("field_name", "expected", "received"):
            object.__setattr__(self, name, str(getattr(self, name)))

    @property
    def code(self) -> str:
        return self.reason.value


# -- pure quaternion helpers (xyzw) -------------------------------------

def _q_mul(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _q_conj(q: Sequence[float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    return (-x, -y, -z, w)


def _q_normalize(q: Sequence[float]) -> Tuple[float, float, float, float]:
    norm = math.sqrt(sum(v * v for v in q))
    if norm == 0.0 or not math.isfinite(norm):
        raise ContractViolation("quaternion: zero/non-finite norm")
    return tuple(v / norm for v in q)  # type: ignore[return-value]


def rotate_vector(q_xyzw: Sequence[float], v: Sequence[float]) -> Tuple[float, float, float]:
    """Rotate ``v`` by unit quaternion ``q_xyzw``."""
    qv = _q_mul(_q_mul(tuple(q_xyzw), (v[0], v[1], v[2], 0.0)), _q_conj(tuple(q_xyzw)))
    return (qv[0], qv[1], qv[2])


def compose_transform(
    t1: Sequence[float], q1: Sequence[float], t2: Sequence[float], q2: Sequence[float]
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    """(t1, q1) then (t2, q2): world <- a <- b composition."""
    q = _q_normalize(_q_mul(tuple(q1), tuple(q2)))
    rotated = rotate_vector(q1, t2)
    t = (t1[0] + rotated[0], t1[1] + rotated[1], t1[2] + rotated[2])
    return t, q  # type: ignore[return-value]


def invert_transform(
    t: Sequence[float], q: Sequence[float]
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    """Inverse rigid transform: b <- a from a <- b."""
    qi = _q_conj(_q_normalize(tuple(q)))
    ti = rotate_vector(qi, (-t[0], -t[1], -t[2]))
    return ti, qi


def transform_point(
    t: Sequence[float], q: Sequence[float], p: Sequence[float]
) -> Tuple[float, float, float]:
    """Map point ``p`` (b coords) into a coords through T_a_b = (t, q)."""
    r = rotate_vector(q, p)
    return (t[0] + r[0], t[1] + r[1], t[2] + r[2])


def quaternion_angle_between(
    a: Sequence[float], b: Sequence[float]
) -> float:
    """Rotation angle of the relative rotation a^-1 * b (rad)."""
    rel = _q_mul(_q_conj(_q_normalize(tuple(a))), _q_normalize(tuple(b)))
    w = min(1.0, abs(rel[3]))
    return 2.0 * math.acos(w)


def expand_symmetric_heading(theta: float, symmetry: str) -> Tuple[float, ...]:
    """Theta headings to evaluate for a payload symmetry declaration.

    ``none`` keeps theta; ``pi`` adds theta+pi as a second distinct
    physical rotation. Pure representation: no axis swap, no tool yaw.
    """
    if symmetry not in ("none", "pi"):
        raise ContractViolation("symmetry: 'none' or 'pi' (%r)" % (symmetry,))
    if symmetry == "pi":
        return (theta, theta + math.pi)
    return (theta,)


def resolve_place_targets(
    box_target: BoxTarget,
    attachment: AttachmentRecord,
    *,
    require_attach_confirmed: bool = True,
    max_age_ns: int = ATTACHMENT_MAX_AGE_NS,
) -> Tuple[PlaceTarget, ...]:
    """Pure box-heading -> TCP target synthesis (technical baseline 5.1).

    For each heading allowed by the declared symmetry the TCP target is
    ``T_world_tcp = T_world_box * inverse(T_tcp_box)``; the theta+pi
    heading rotates the box pose by pi about the frame vertical axis
    *before* applying the actual attachment, so an eccentric grasp
    yields a genuinely different TCP pose. No half-box-height is added
    to the centre. Raises :class:`ContractViolation` with a
    :class:`Reason`-derived code on any 5.1 negative; diagnostics ride
    on ``PlaceTarget.note``.
    """
    if attachment.validity is not AttachmentValidity.VALID:
        raise ContractViolation(
            "%s: attachment not valid (%s/%s)" % (
                Reason.ATTACHMENT_MISSING.value, attachment.validity,
                attachment.invalidation_reason))
    if not attachment.source.measured:
        raise ContractViolation(
            "%s: source=%s" % (Reason.ATTACHMENT_GT_AS_MEASURED.value, attachment.source))
    if require_attach_confirmed and not attachment.source.route_grade:
        raise ContractViolation(
            "%s: source=%s (pre-attach estimate is initial-selection only)" % (
                Reason.ATTACHMENT_SOURCE_UNQUALIFIED.value, attachment.source))
    if box_target.request_id != attachment.request_id:
        raise ContractViolation(
            "%s: target request %s vs attachment %s" % (
                Reason.ATTACHMENT_REQUEST_MISMATCH.value,
                box_target.request_id, attachment.request_id))
    if box_target.scene_revision != attachment.scene_revision:
        raise ContractViolation(
            "%s: target scene %s vs attachment %s" % (
                Reason.ATTACHMENT_FRAME_MISMATCH.value,
                box_target.scene_revision, attachment.scene_revision))
    if box_target.stamp_ns + TF_SYNC_MAX_SKEW_NS < attachment.acquisition_stamp_ns:
        raise ContractViolation(
            "%s: target stamp %d before acquisition %d" % (
                Reason.STAMP_INVALID.value,
                box_target.stamp_ns, attachment.acquisition_stamp_ns))
    if box_target.stamp_ns - attachment.acquisition_stamp_ns > max_age_ns:
        raise ContractViolation(
            "%s: %d ns over limit %d ns" % (
                Reason.ATTACHMENT_EXPIRED.value,
                box_target.stamp_ns - attachment.acquisition_stamp_ns, max_age_ns))

    t_box_tcp, q_box_tcp = invert_transform(
        attachment.t_tcp_box_translation, attachment.t_tcp_box_quaternion_xyzw)
    targets = []
    for heading in expand_symmetric_heading(box_target.heading_theta, attachment.symmetry):
        t_world_box, q_world_box = _heading_pose(
            box_target.position, box_target.quaternion_xyzw, heading - box_target.heading_theta)
        t_world_tcp, q_world_tcp = compose_transform(
            t_world_box, q_world_box, t_box_tcp, q_box_tcp)
        targets.append(PlaceTarget(
            candidate_id=box_target.candidate_id,
            heading_theta=heading,
            tcp_frame=attachment.tcp_frame,
            frame_id=box_target.frame_id,
            position=t_world_tcp,
            quaternion_xyzw=q_world_tcp,
            attachment_digest=attachment.digest(),
            note="T_world_box*inverse(T_tcp_box) heading=%.9f" % heading))
    return tuple(targets)


def _heading_pose(
    position: Sequence[float], quaternion_xyzw: Sequence[float], extra_theta: float
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    """Rotate the box pose by ``extra_theta`` about the frame Z axis."""
    if extra_theta == 0.0:
        return (tuple(position), tuple(quaternion_xyzw))  # type: ignore[return-value]
    half = 0.5 * extra_theta
    dz = (0.0, 0.0, math.sin(half), math.cos(half))
    return compose_transform(tuple(position), tuple(quaternion_xyzw), (0.0, 0.0, 0.0), dz)
