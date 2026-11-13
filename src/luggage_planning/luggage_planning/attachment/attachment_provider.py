#!/usr/bin/env python3
"""ST09-D: the measured attachment provider (frozen AttachmentProvider).

ROS-free core with injectable data sources; ST09-I wires live TF, the
vacuum backend and the measured box-pose topic. The provider and the
cargo-map integrator are separate modules by contract — this module must
not grow into a map manager.

Policy values are the frozen ones (``config/planning_contract/resolved_planning_contract.json``
attachment_policy): same-stamp measured box pose + TCP TF (skew <=
``TF_SYNC_MAX_SKEW_NS`` = 0 ns), ``T_tcp_box = inverse(T_world_tcp) *
T_world_box``, freeze exactly once per valid observation + attach
confirmation (duplicates idempotent by digest), new incarnation per
confirmed re-attach, invalidation for slip / re-attach / frame or
calibration change / expiry. GT, planned-slot and PCA-yaw-only sources are
never accepted as measured.

An explicitly qualified stationary-box path preserves the original acquisition
and later confirmation-time TCP stamps in StationaryAttachmentEvidence. It
requires actual vacuum and frozen-footprint admission; it cannot rebind a valid
sealed relation or relax the legacy same-stamp path.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

try:
    from luggage_planning.planning_contract._base import ContractViolation, digest_of
    from luggage_planning.planning_contract.attachment import (
        ATTACHMENT_MAX_AGE_NS,
        ATTACHMENT_SCHEMA_VERSION,
        TF_SYNC_MAX_SKEW_NS,
        AttachmentInvalidation,
        AttachmentRecord,
        AttachmentSourceKind,
        AttachmentValidity,
        compose_transform,
        invert_transform,
        quaternion_angle_between,
        transform_point,
    )
    from luggage_planning.planning_contract.reasons import Reason
except ImportError:  # pragma: no cover - import guard for non-contract envs
    raise ImportError(
        "attachment_provider requires luggage_planning.planning_contract")

from luggage_planning.planning_contract.stationary_attachment import (
    StationaryAttachmentEvidence, measured_pose_digest,
)

GRASP_BINDING_TOLERANCE_M = 0.01
GRASP_BINDING_TOLERANCE_RAD = 0.10
REPRODUCE_TOLERANCE_M = 1e-9

# Source-method markers that can never qualify as a measured attachment
# (frozen v2_field_audit GT rule).
FORBIDDEN_METHOD_MARKERS = (
    "get_current_box",     # GetCurrentBox GT pull
    "planned_slot",        # the slot the planner chose
    "pca_yaw_only",        # detection PCA yaw with no full pose
    "gazebo_physics",      # sim physics lock transform
)


class MeasuredPose:
    """One stamped rigid pose in a named frame (translation + xyzw quat)."""

    __slots__ = ("frame", "position", "quaternion_xyzw", "stamp_ns")

    def __init__(self, frame, position, quaternion_xyzw, stamp_ns):
        self.frame = str(frame)
        self.position = tuple(float(v) for v in position)
        self.quaternion_xyzw = tuple(float(v) for v in quaternion_xyzw)
        self.stamp_ns = int(stamp_ns)


class MeasuredAttachmentProvider:
    """Frozen-protocol provider for one active request at a time.

    Usage (ST09-I wires the live sources into these calls):

    1. ``record_attachment`` at attach confirmation (or, explicitly
       requested, as a pre-attach estimate that is never route-grade);
    2. ``latest_record`` at planning/admission — raises with a named
       ``Reason`` when anything is wrong, never returns a fake record;
    3. ``slip_recheck`` on re-observation during carry;
    4. ``invalidate`` for frame/calibration change or expiry.
    """

    def __init__(self, scene_revision: str, calibration_id: str,
                 tcp_frame: str = "suction_contact_frame",
                 box_frame: str = "world",
                 max_age_ns: Optional[int] = None):
        self._scene_revision = str(scene_revision)
        self._calibration_id = str(calibration_id)
        self._tcp_frame = str(tcp_frame)
        self._box_frame = str(box_frame)
        if max_age_ns is None:
            self._max_age_ns = int(ATTACHMENT_MAX_AGE_NS)
        elif isinstance(max_age_ns, bool) or not isinstance(max_age_ns, int) \
                or max_age_ns < 1:
            raise ValueError(
                "max_age_ns must be a positive int, got %r" % (max_age_ns,))
        else:
            self._max_age_ns = int(max_age_ns)
        self._records: Dict[str, AttachmentRecord] = {}
        self._incarnations: Dict[str, int] = {}
        self._generations: Dict[str, int] = {}
        self._stationary_evidence: Dict[str, StationaryAttachmentEvidence] = {}
        self._active_request: Optional[str] = None
        self._invalidations: Tuple[AttachmentInvalidation, ...] = ()

    # -- introspection ---------------------------------------------------

    @property
    def invalidations(self) -> Tuple[AttachmentInvalidation, ...]:
        return self._invalidations

    def record_for(self, request_id: str) -> Optional[AttachmentRecord]:
        return self._records.get(str(request_id))

    @property
    def max_age_ns(self) -> int:
        """Acquisition window for this run: 60 s times the box count."""
        return self._max_age_ns

    def live_count(self) -> int:
        return sum(
            1 for record in self._records.values()
            if record.validity is AttachmentValidity.VALID)

    def bind_generation(self, request_id: str, generation: Optional[int]) -> None:
        """Remember the attach generation beside the frozen record."""
        if generation is None:
            return
        self._generations[str(request_id)] = int(generation)

    def generation_for(self, request_id: str) -> Optional[int]:
        return self._generations.get(str(request_id))

    # -- protocol --------------------------------------------------------

    def latest_record(self, request_id: str) -> AttachmentRecord:
        record = self._records.get(str(request_id))
        if record is None:
            raise ContractViolation(
                "%s: no attachment record for request %r" % (
                    Reason.ATTACHMENT_MISSING.value, request_id))
        if record.validity is not AttachmentValidity.VALID:
            raise ContractViolation(
                "%s: record invalidated (%s)" % (
                    Reason.ATTACHMENT_MISSING.value,
                    record.invalidation_reason or "invalidated"))
        return record

    def expire_stale(self, request_id: str, now_ns: int) -> None:
        """Apply this run's acquisition->use window.

        The default window is 60 s, one box. A run of n boxes uses
        n times that, the same number as its route budget.

        A zero acquisition stamp is not a live measurement. It is
        invalidated as ``STAMP_INVALID`` instead of being aged from the
        epoch and then reported as a missing record.
        """
        request_id = str(request_id)
        record = self._records.get(request_id)
        if record is None or record.validity is not AttachmentValidity.VALID:
            return
        now_ns = int(now_ns)
        stamp = int(record.acquisition_stamp_ns)
        age = now_ns - stamp
        if stamp <= 0:
            reason = Reason.STAMP_INVALID
        elif age > self._max_age_ns:
            reason = Reason.ATTACHMENT_EXPIRED
        else:
            return
        before = self.live_count()
        self._invalidate_record(record, reason, {
            "record_stamp_ns": stamp,
            "now_stamp_ns": now_ns,
            "age_ns": age,
            "stale_window_ns": self._max_age_ns,
            "box_id": str(record.box_id),
            "generation": self.generation_for(request_id),
            "attachment_digest": record.digest(),
            "attachment_incarnation": int(record.attachment_incarnation),
            "records_before": before,
            "records_after": before - 1,
        })

    def invalidate(self, reason: Reason,
                   detail: Mapping[str, Any]) -> AttachmentInvalidation:
        record = self._records.get(self._active_request or "")
        if record is None:
            raise ContractViolation(
                "%s: invalidate requires an active record" % (reason.value,))
        return self._invalidate_record(record, reason, dict(detail))

    # -- recording -------------------------------------------------------

    def record_attachment(
        self,
        request_id: str,
        box_pose: Optional[MeasuredPose],
        tcp_pose: Optional[MeasuredPose],
        *,
        attach_confirmed: bool,
        source_kind: AttachmentSourceKind,
        source_method: str,
        dimensions_wdh: Sequence[float],
        cargo_id: str,
        box_id: str,
        cycle_id: str,
        symmetry: str = "pi",
        evidence_digest: str = "",
        allow_pre_attach: bool = False,
        conservative_geometry=None,
    ) -> AttachmentRecord:
        return self._record_attachment(request_id, box_pose, tcp_pose,
            attach_confirmed=attach_confirmed, source_kind=source_kind,
            source_method=source_method, dimensions_wdh=dimensions_wdh,
            cargo_id=cargo_id, box_id=box_id, cycle_id=cycle_id, symmetry=symmetry,
            evidence_digest=evidence_digest, allow_pre_attach=allow_pre_attach,
            conservative_geometry=conservative_geometry)

    def stationary_evidence_for(self, request_id):
        """Original two-stamp provenance, without renewing physical freshness."""
        return self._stationary_evidence.get(str(request_id))

    def record_stationary_attachment(self, request_id, box_pose, tcp_pose, *,
                                     evidence, conservative_geometry, now_ns, cycle_id):
        """Freeze once under the explicitly qualified stationary-box premise.

        The complete-operation adapter owns actual vacuum and frozen-footprint
        checks before producing this evidence. No arbitrary skew allowance is
        exposed; legacy record_attachment remains strictly same-stamp.
        """
        if (type(evidence) is not StationaryAttachmentEvidence
                or box_pose is None or tcp_pose is None
                or conservative_geometry is None):
            raise ContractViolation('STATIONARY_ATTACHMENT_EVIDENCE_REQUIRED')
        evidence.__post_init__()
        conservative_geometry.__post_init__()
        if ((str(request_id), self._scene_revision, box_pose.stamp_ns, tcp_pose.stamp_ns,
                measured_pose_digest(box_pose), measured_pose_digest(tcp_pose),
                conservative_geometry.digest()) !=
                (evidence.request_id, evidence.scene_revision, evidence.box_acquisition_stamp_ns,
                 evidence.tcp_confirmation_stamp_ns, evidence.box_pose_sha256,
                 evidence.tcp_pose_sha256, evidence.geometry_sha256)):
            raise ContractViolation('STATIONARY_ATTACHMENT_SOURCE_BINDING_MISMATCH')
        if (type(now_ns) is not int or now_ns < tcp_pose.stamp_ns
                or now_ns - box_pose.stamp_ns > self._max_age_ns):
            raise ContractViolation('STATIONARY_ATTACHMENT_ACQUISITION_NOT_CURRENT')
        previous = self._records.get(str(request_id))
        if previous is not None:
            if (previous.validity is not AttachmentValidity.VALID
                    or self._stationary_evidence.get(str(request_id)) != evidence):
                raise ContractViolation('SEALED_ATTACHMENT_REBIND_REFUSED')
            return previous
        record = self._record_attachment(request_id, box_pose, tcp_pose,
            attach_confirmed=True, source_kind=AttachmentSourceKind.MEASURED_ATTACH_CONFIRMED,
            source_method='stationary_box:frozen_top+confirmed_vacuum_actual_tcp',
            dimensions_wdh=conservative_geometry.measured_dimensions_wdh,
            cargo_id=evidence.box_id, box_id=evidence.box_id, cycle_id=cycle_id,
            evidence_digest=evidence.digest(), conservative_geometry=conservative_geometry,
            stationary_evidence=evidence)
        self._stationary_evidence[str(request_id)] = evidence
        self.bind_generation(request_id, evidence.generation)
        return record

    def _record_attachment(
        self,
        request_id: str,
        box_pose: Optional[MeasuredPose],
        tcp_pose: Optional[MeasuredPose],
        *,
        attach_confirmed: bool,
        source_kind: AttachmentSourceKind,
        source_method: str,
        dimensions_wdh: Sequence[float],
        cargo_id: str,
        box_id: str,
        cycle_id: str,
        symmetry: str = "pi",
        evidence_digest: str = "",
        allow_pre_attach: bool = False,
        conservative_geometry=None,
        stationary_evidence=None,
    ) -> AttachmentRecord:
        request_id = str(request_id)
        sealed = self._stationary_evidence.get(request_id)
        if sealed is not None and stationary_evidence != sealed:
            raise ContractViolation('SEALED_ATTACHMENT_REBIND_REFUSED')
        self._active_request = request_id

        # -- source qualification (before any transform math) ------------
        if source_kind is AttachmentSourceKind.REFERENCE_SIM_GT:
            raise ContractViolation(
                "%s: sim GT relative transform is reference-only" % (
                    Reason.ATTACHMENT_GT_AS_MEASURED.value,))
        method = str(source_method or "").lower()
        for marker in FORBIDDEN_METHOD_MARKERS:
            if marker in method:
                raise ContractViolation(
                    "%s: source method %r can never register as measured" % (
                        Reason.ATTACHMENT_SOURCE_UNQUALIFIED.value,
                        source_method))
        if source_kind not in (
                AttachmentSourceKind.MEASURED_ATTACH_CONFIRMED,
                AttachmentSourceKind.MEASURED_PRE_ATTACH_ESTIMATE):
            raise ContractViolation(
                "%s: unknown source kind %r" % (
                    Reason.ATTACHMENT_SOURCE_UNQUALIFIED.value, source_kind))

        # -- input presence and validity ---------------------------------
        if box_pose is None or tcp_pose is None:
            raise ContractViolation(
                "%s: measured box pose and TCP TF are both required" % (
                    Reason.ATTACHMENT_MISSING.value,))
        if box_pose.frame != self._box_frame or tcp_pose.frame != self._tcp_frame:
            raise ContractViolation(
                "%s: pose frame mismatch (%s/%s expected %s/%s)" % (
                    Reason.ATTACHMENT_FRAME_MISMATCH.value,
                    box_pose.frame, tcp_pose.frame,
                    self._box_frame, self._tcp_frame))
        skew = abs(int(tcp_pose.stamp_ns) - int(box_pose.stamp_ns))
        if stationary_evidence is None and skew > TF_SYNC_MAX_SKEW_NS:
            raise ContractViolation(
                "%s: measured poses are not same-stamp (skew %d ns)" % (
                    Reason.ATTACHMENT_SOURCE_UNQUALIFIED.value, skew))
        dims = tuple(float(v) for v in dimensions_wdh)
        if len(dims) != 3 or not all(
                math.isfinite(v) and v > 0.0 for v in dims):
            raise ContractViolation(
                "%s: dimensions must be three positive finite values" % (
                    Reason.ATTACHMENT_INVALID_DIMENSIONS.value,))
        for pose in (box_pose, tcp_pose):
            norm = math.sqrt(sum(v * v for v in pose.quaternion_xyzw))
            if not math.isfinite(norm) or abs(norm - 1.0) > 1e-9:
                raise ContractViolation(
                    "%s: quaternion must be finite and unit" % (
                        Reason.ATTACHMENT_INVALID_TRANSFORM.value,))

        # -- freeze grade --------------------------------------------------
        if attach_confirmed:
            grade = AttachmentSourceKind.MEASURED_ATTACH_CONFIRMED
        else:
            if not allow_pre_attach:
                raise ContractViolation(
                    "%s: attach confirmation missing; refusing to publish "
                    "a route-grade record" % (
                        Reason.ATTACHMENT_UNCONFIRMED.value,))
            grade = AttachmentSourceKind.MEASURED_PRE_ATTACH_ESTIMATE

        # -- T_tcp_box = inverse(T_world_tcp) * T_world_box ---------------
        t_world_tcp = (tcp_pose.position, tcp_pose.quaternion_xyzw)
        t_world_box = (box_pose.position, box_pose.quaternion_xyzw)
        t_tcp_world = invert_transform(*t_world_tcp)
        t_tcp_box = compose_transform(
            t_tcp_world[0], t_tcp_world[1],
            t_world_box[0], t_world_box[1])
        translation, quaternion = t_tcp_box
        # Reproduce oracle: box -> tcp through the frozen transform must
        # reproduce the measured TCP pose to machine precision.
        reproduced = compose_transform(
            t_world_box[0], t_world_box[1],
            *invert_transform(*t_tcp_box))
        drift = math.sqrt(sum(
            (a - b) ** 2
            for a, b in zip(reproduced[0], tcp_pose.position)))
        if drift > REPRODUCE_TOLERANCE_M:
            raise ContractViolation(
                "%s: frozen transform does not reproduce the measured TCP "
                "(drift %.3e m)" % (
                    Reason.ATTACHMENT_INVALID_TRANSFORM.value, drift))

        # -- idempotency / incarnation ------------------------------------
        previous = self._records.get(request_id)
        if (previous is not None
                and previous.acquisition_stamp_ns == box_pose.stamp_ns
                and previous.source is grade
                and previous.t_tcp_box_translation == tuple(
                    round(v, 12) for v in translation)
                and previous.t_tcp_box_quaternion_xyzw == tuple(
                    round(v, 12) for v in quaternion)):
            # duplicate event: idempotent, no new incarnation
            return previous
        incarnation = self._incarnations.get(request_id, -1) + 1
        self._incarnations[request_id] = incarnation
        if previous is not None and previous.validity is AttachmentValidity.VALID:
            self._invalidate_record(
                previous, Reason.ATTACHMENT_REATTACH,
                {"new_incarnation": incarnation})

        if not evidence_digest:
            # Source/stamp linkage digest: the record must always carry
            # evidence binding (frozen field is non-empty).
            evidence_digest = "att:" + digest_of(
                source_method, int(box_pose.stamp_ns), int(tcp_pose.stamp_ns),
                box_pose.position, box_pose.quaternion_xyzw,
                tcp_pose.position, tcp_pose.quaternion_xyzw)
        envelope = None
        if conservative_geometry is not None:
            if (dims != conservative_geometry.measured_dimensions_wdh
                    or any(abs(a-b) > 1e-9 for a,b in zip(translation, conservative_geometry.t_tool_box_translation))
                    or abs(sum(a*b for a,b in zip(quaternion, conservative_geometry.t_tool_box_quaternion_xyzw))) < 1-1e-9):
                raise ContractViolation('ATTACHMENT_GEOMETRY_BINDING_MISMATCH')
            envelope = conservative_geometry.conservative
            translation = conservative_geometry.t_tool_box_translation
            quaternion = conservative_geometry.t_tool_box_quaternion_xyzw
        record = AttachmentRecord(
            schema_version=ATTACHMENT_SCHEMA_VERSION,
            cargo_id=str(cargo_id),
            box_id=str(box_id),
            request_id=request_id,
            cycle_id=str(cycle_id),
            attachment_incarnation=incarnation,
            tcp_frame=self._tcp_frame,
            box_frame=self._box_frame,
            dimensions_wdh=dims,
            symmetry=str(symmetry),
            t_tcp_box_translation=tuple(float(v) for v in translation),
            t_tcp_box_quaternion_xyzw=tuple(float(v) for v in quaternion),
            acquisition_stamp_ns=int(box_pose.stamp_ns),
            scene_revision=self._scene_revision,
            calibration_id=self._calibration_id,
            source=grade,
            source_method=str(source_method),
            evidence_digest=str(evidence_digest or ""),
            validity=AttachmentValidity.VALID,
            invalidation_reason=None,
            collision_envelope=envelope,
        )
        self._records[request_id] = record
        if stationary_evidence is None:
            self._stationary_evidence.pop(request_id, None)
        return record

    # -- carry-time slip recheck -------------------------------------------

    def slip_recheck(self, request_id: str, box_pose_reobserved: MeasuredPose,
                     tcp_pose_live: MeasuredPose) -> bool:
        """Re-observed box pose composed through the frozen record must
        agree with the live TCP within the frozen binding tolerances.

        Returns True when within tolerance; beyond it the record is
        invalidated (``ATTACHMENT_SLIP``) and the notification is emitted —
        the caller enters carry-fault with vacuum retained.
        """
        record = self.latest_record(request_id)
        t_world_box = (
            box_pose_reobserved.position, box_pose_reobserved.quaternion_xyzw)
        t_box_tcp = invert_transform(
            record.t_tcp_box_translation,
            record.t_tcp_box_quaternion_xyzw)
        predicted_tcp = compose_transform(
            t_world_box[0], t_world_box[1],
            t_box_tcp[0], t_box_tcp[1])
        position_error = math.sqrt(sum(
            (a - b) ** 2
            for a, b in zip(predicted_tcp[0], tcp_pose_live.position)))
        angle_error = quaternion_angle_between(
            predicted_tcp[1], tcp_pose_live.quaternion_xyzw)
        within = (position_error <= GRASP_BINDING_TOLERANCE_M
                  and angle_error <= GRASP_BINDING_TOLERANCE_RAD)
        if not within:
            self._invalidate_record(
                record, Reason.ATTACHMENT_SLIP,
                {"position_error_m": position_error,
                 "angle_error_rad": angle_error})
        return within

    # -- internals ---------------------------------------------------------

    def _invalidate_record(self, record: AttachmentRecord, reason: Reason,
                           detail: Mapping[str, Any]) -> AttachmentInvalidation:
        replacement = AttachmentRecord(
            schema_version=record.schema_version,
            cargo_id=record.cargo_id,
            box_id=record.box_id,
            request_id=record.request_id,
            cycle_id=record.cycle_id,
            attachment_incarnation=record.attachment_incarnation,
            tcp_frame=record.tcp_frame,
            box_frame=record.box_frame,
            dimensions_wdh=record.dimensions_wdh,
            symmetry=record.symmetry,
            t_tcp_box_translation=record.t_tcp_box_translation,
            t_tcp_box_quaternion_xyzw=record.t_tcp_box_quaternion_xyzw,
            acquisition_stamp_ns=record.acquisition_stamp_ns,
            scene_revision=record.scene_revision,
            calibration_id=record.calibration_id,
            source=record.source,
            source_method=record.source_method,
            evidence_digest=record.evidence_digest,
            validity=AttachmentValidity.INVALIDATED,
            invalidation_reason=str(reason.value),
            collision_envelope=record.collision_envelope,
        )
        self._records[record.request_id] = replacement
        notification = AttachmentInvalidation(
            record_digest=record.digest(),
            box_id=record.box_id,
            request_id=record.request_id,
            incarnation=record.attachment_incarnation,
            reason=reason,
            stamp_ns=record.acquisition_stamp_ns,
            detail=dict(detail),
        )
        self._invalidations = self._invalidations + (notification,)
        return notification
