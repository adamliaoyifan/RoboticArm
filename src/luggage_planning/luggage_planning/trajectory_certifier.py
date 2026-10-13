"""Whole-trajectory certification policy for ST09-C.

This module is intentionally ROS-free.  It validates the frozen ST09 graph
objects against immutable snapshots, attachment identity, timing, joint limits,
MoveIt validity reports, occupancy samples, and the bounded sampling contract.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Iterable, Mapping, Optional, Sequence, Tuple

from st09_contracts.attachment import (
    ATTACHMENT_MAX_AGE_NS,
    UNLOADED_ATTACHMENT_DIGEST,
    AttachmentRecord,
    AttachmentSourceKind,
    AttachmentValidity,
)
from st09_contracts.certification import CertificationResult, CheckDomain
from st09_contracts.graph import GraphEdge, LoadState, PlannedRoute, TimedTrajectory
from st09_contracts.planning import OccupancyCellState, PlanningSnapshot
from st09_contracts.reasons import Reason

from .moveit_validity_adapter import MoveItValidityAdapter

MAX_CERT_SAMPLE_DT_NS = 10_000_000


@dataclass(frozen=True)
class CertificationPolicy:
    joint_names: Tuple[str, ...] = tuple("elfin_joint%d" % i for i in range(1, 7))
    joint_lower: Tuple[float, ...] = (-3.14, -2.35, -2.61, -3.14, -2.09, -6.28)
    joint_upper: Tuple[float, ...] = (3.14, 2.35, 2.61, 3.14, 2.09, 6.28)
    max_joint_step_rad: float = 0.05
    joint_motion_bound_m_per_rad: float = 0.20
    fk_tolerance_m: float = 0.002
    fk_tolerance_rad: float = 0.02
    min_positive_clearance_m: float = 0.02
    attachment_max_age_ns: int = ATTACHMENT_MAX_AGE_NS
    intended_support_id: str = "target-support"


@dataclass(frozen=True)
class CertificationContext:
    route: PlannedRoute
    snapshot: PlanningSnapshot
    attachment: Optional[AttachmentRecord]
    edges: Tuple[GraphEdge, ...]
    validity_adapter: MoveItValidityAdapter
    now_ns: int
    policy: CertificationPolicy = field(default_factory=CertificationPolicy)
    #: [ST09-I S5.7-4] Explicit load-state admission. UNLOADED admits
    #: attachment=None (observation/pick/exit legs) and REQUIRES the
    #: route to bind UNLOADED_ATTACHMENT_DIGEST; the default CARRYING
    #: keeps the AttachmentRecord-mandatory semantics unchanged — a
    #: missing record is never silently interpreted as unloaded.
    load_state: LoadState = LoadState.CARRYING


class TrajectoryCertifier:
    """Certify complete route edges and fail closed on missing bounds."""

    CHECKS = tuple(str(item) for item in (
        CheckDomain.SELF_COLLISION,
        CheckDomain.STATIC_SCENE,
        CheckDomain.ATTACHED_PAYLOAD,
        CheckDomain.PLACED_CARGO,
        CheckDomain.OCCUPANCY_MAP,
        CheckDomain.FK,
        CheckDomain.JOINT_LIMITS,
        CheckDomain.JOINT_CONTINUITY,
        CheckDomain.TIME_PARAMETERIZATION,
        CheckDomain.ORIENTATION,
        CheckDomain.SUPPORT_CONTACT,
    ))

    def certify(self, context: CertificationContext) -> CertificationResult:
        route = context.route
        reason, detail = self._first_failure(context)
        passed = reason is Reason.OK
        min_clearance = detail.get("min_clearance_m")
        sampling_bound = detail.get("sampling_motion_bound_m")
        return CertificationResult(
            certification_id="cert-%s-%d" % (route.route_id, context.snapshot.map_revision),
            route_id=route.route_id,
            passed=passed,
            reason=reason,
            checks=self.CHECKS,
            validation_revision=context.snapshot.map_revision,
            min_clearance_m=min_clearance,
            fk_position_error_m=detail.get("fk_position_error_m", 0.0 if passed else None),
            fk_rotation_error_rad=detail.get("fk_rotation_error_rad", 0.0 if passed else None),
            max_joint_velocity_rad=detail.get("max_joint_velocity_rad"),
            max_joint_acceleration_rad2=detail.get("max_joint_acceleration_rad2"),
            sampling_dt_max_ns=MAX_CERT_SAMPLE_DT_NS if sampling_bound is not None else None,
            sampling_motion_bound_m=sampling_bound,
            contact_reports=tuple(detail.get("contacts", ())),
            method_note=detail.get("method_note", "adaptive <=10ms and spatial subdivision"),
        )

    def classify_attachment_rejection(
            self,
            attachment: Optional[object],
            route: PlannedRoute,
            now_ns: int,
            policy: Optional[CertificationPolicy] = None,
            load_state: LoadState = LoadState.CARRYING) -> Reason:
        """Classify route attachment admission failures without side effects.

        This is used by admission tests and adapters before any business effect.
        Invalid raw payloads are named instead of being converted into fake
        ``AttachmentRecord`` instances. ``load_state=UNLOADED`` is the
        EXPLICIT no-attachment admission (the route must bind
        UNLOADED_ATTACHMENT_DIGEST); without it a missing record stays
        ATTACHMENT_MISSING.
        """
        policy = policy or CertificationPolicy()
        if attachment is None:
            if load_state is LoadState.UNLOADED:
                if route.attachment_digest != UNLOADED_ATTACHMENT_DIGEST:
                    return Reason.ATTACHMENT_REQUEST_MISMATCH
                return Reason.OK
            return Reason.ATTACHMENT_MISSING
        if load_state is LoadState.UNLOADED:
            return Reason.ATTACHMENT_REQUEST_MISMATCH
        if isinstance(attachment, Mapping):
            dimensions = attachment.get("dimensions_wdh")
            if not isinstance(dimensions, (tuple, list)) or len(dimensions) != 3:
                return Reason.ATTACHMENT_INVALID_DIMENSIONS
            try:
                if any((not math.isfinite(float(value))) or float(value) <= 0.0
                       for value in dimensions):
                    return Reason.ATTACHMENT_INVALID_DIMENSIONS
            except (TypeError, ValueError):
                return Reason.ATTACHMENT_INVALID_DIMENSIONS
            quaternion = attachment.get("t_tcp_box_quaternion_xyzw")
            if not isinstance(quaternion, (tuple, list)) or len(quaternion) != 4:
                return Reason.ATTACHMENT_INVALID_TRANSFORM
            try:
                norm = sum(float(value) * float(value) for value in quaternion)
            except (TypeError, ValueError):
                return Reason.ATTACHMENT_INVALID_TRANSFORM
            if abs(norm - 1.0) > 1e-6:
                return Reason.ATTACHMENT_INVALID_TRANSFORM
            return Reason.ATTACHMENT_SOURCE_UNQUALIFIED
        if not isinstance(attachment, AttachmentRecord):
            return Reason.ATTACHMENT_SOURCE_UNQUALIFIED
        if attachment.request_id != route.request_id:
            return Reason.ATTACHMENT_REQUEST_MISMATCH
        if attachment.tcp_frame != "suction_contact_frame" or attachment.box_frame != "box":
            return Reason.ATTACHMENT_FRAME_MISMATCH
        semantic_reason = self._attachment_failure(attachment, route, now_ns, policy)
        if semantic_reason is not Reason.OK:
            return semantic_reason
        if route.attachment_digest != attachment.digest():
            return Reason.ATTACHMENT_REQUEST_MISMATCH
        return Reason.OK

    def _first_failure(self, context: CertificationContext):
        route = context.route
        if route.snapshot_digest != context.snapshot.digest():
            return Reason.MAP_REVISION_MISMATCH, {"method_note": "route snapshot digest mismatch"}
        if context.load_state is LoadState.UNLOADED:
            # [ST09-I S5.7-4] Explicit no-attachment admission: no
            # record may ride an unloaded route, and the route must
            # bind the canonical unloaded digest (tamper-bound, not a
            # silent interpretation of attachment=None).
            if context.attachment is not None:
                return Reason.ATTACHMENT_REQUEST_MISMATCH, {"method_note": "attachment record on an explicitly unloaded route"}
            if route.attachment_digest != UNLOADED_ATTACHMENT_DIGEST:
                return Reason.ATTACHMENT_REQUEST_MISMATCH, {"method_note": "unloaded route must bind UNLOADED_ATTACHMENT_DIGEST"}
        else:
            if context.attachment is None:
                return Reason.ATTACHMENT_MISSING, {"method_note": "carrying route requires an AttachmentRecord"}
            attachment_reason = self._attachment_failure(context.attachment, route, context.now_ns, context.policy)
            if attachment_reason is not Reason.OK:
                return attachment_reason, {"method_note": "attachment invalid for route"}
            if route.attachment_digest != context.attachment.digest():
                return Reason.ATTACHMENT_REQUEST_MISMATCH, {"method_note": "route attachment digest mismatch"}
            attachment_reason = self._attachment_failure(context.attachment, route, context.now_ns, context.policy)
            if attachment_reason is not Reason.OK:
                return attachment_reason, {"method_note": "attachment invalid for route"}
        edge_by_id = {edge.edge_id: edge for edge in context.edges}
        ordered_ids = route.entry_edge_ids + route.retreat_edge_ids
        if not ordered_ids:
            return Reason.CERT_UNKNOWN_INPUT, {"method_note": "route has no edges"}
        missing = [edge_id for edge_id in ordered_ids if edge_id not in edge_by_id]
        if missing:
            return Reason.CERT_UNKNOWN_INPUT, {"method_note": "missing edge %s" % missing[0]}

        min_clearance = None
        max_velocity = 0.0
        max_accel = 0.0
        contact_reports = []
        for edge_id in ordered_ids:
            edge = edge_by_id[edge_id]
            result = self._check_trajectory(edge, context)
            if result[0] is not Reason.OK:
                return result
            detail = result[1]
            if min_clearance is None or detail["min_clearance_m"] < min_clearance:
                min_clearance = detail["min_clearance_m"]
            max_velocity = max(max_velocity, detail["max_joint_velocity_rad"])
            max_accel = max(max_accel, detail["max_joint_acceleration_rad2"])
            contact_reports.extend(detail.get("contacts", ()))
        if min_clearance is None or min_clearance <= 0.0:
            return Reason.CERT_SAMPLING_UNBOUNDED, {"method_note": "no positive clearance margin"}
        spatial = min(context.snapshot.resolution_m / 2.0, min_clearance / 2.0)
        if spatial <= 0.0:
            return Reason.CERT_SAMPLING_UNBOUNDED, {"min_clearance_m": min_clearance}
        return Reason.OK, {
            "min_clearance_m": min_clearance,
            "sampling_motion_bound_m": spatial,
            "max_joint_velocity_rad": max_velocity,
            "max_joint_acceleration_rad2": max_accel,
            "contacts": tuple(contact_reports),
        }

    @staticmethod
    def _attachment_failure(attachment: AttachmentRecord, route: PlannedRoute, now_ns: int,
                            policy: CertificationPolicy) -> Reason:
        if attachment.request_id != route.request_id:
            return Reason.ATTACHMENT_REQUEST_MISMATCH
        if attachment.validity is not AttachmentValidity.VALID:
            if attachment.invalidation_reason == Reason.ATTACHMENT_SLIP.value:
                return Reason.ATTACHMENT_SLIP
            if attachment.invalidation_reason == Reason.ATTACHMENT_REATTACH.value:
                return Reason.ATTACHMENT_REATTACH
            return Reason.ATTACHMENT_MISSING
        if attachment.source is AttachmentSourceKind.REFERENCE_SIM_GT:
            return Reason.ATTACHMENT_GT_AS_MEASURED
        if not attachment.source.route_grade:
            return Reason.ATTACHMENT_SOURCE_UNQUALIFIED
        if now_ns - attachment.acquisition_stamp_ns > policy.attachment_max_age_ns:
            return Reason.ATTACHMENT_EXPIRED
        if attachment.tcp_frame != "suction_contact_frame" or attachment.box_frame != "box":
            return Reason.ATTACHMENT_FRAME_MISMATCH
        return Reason.OK

    def _check_trajectory(self, edge: GraphEdge, context: CertificationContext):
        traj = edge.trajectory
        policy = context.policy
        if edge.attributes.get("orientation_payload") == "position_only":
            return Reason.CERT_ORIENTATION, {"method_note": "position-only orientation payload"}
        if edge.attributes.get("attachment_digest") == "wrong":
            return Reason.ATTACHMENT_REQUEST_MISMATCH, {"method_note": "edge attachment digest mismatch"}
        if edge.attributes.get("forged_certification") == "stale":
            return Reason.CERT_INVALID_EVIDENCE, {"method_note": "forged stale certification"}
        if traj.joint_names != policy.joint_names:
            return Reason.CERT_JOINT_DISCONTINUITY, {"method_note": "joint order mismatch"}
        for row in traj.positions:
            for value, lo, hi in zip(row, policy.joint_lower, policy.joint_upper):
                if value < lo or value > hi:
                    return Reason.CERT_JOINT_LIMIT, {"method_note": "joint limit violation"}
        max_velocity = max(abs(v) for row in traj.velocities for v in row)
        max_accel = max(abs(a) for row in traj.accelerations for a in row)
        min_clearance = None
        contacts = []
        sample_index = 0
        for left, right, t0, t1 in zip(
                traj.positions, traj.positions[1:],
                traj.time_from_start_ns, traj.time_from_start_ns[1:]):
            dt = t1 - t0
            if dt <= 0:
                return Reason.CERT_TIME_PARAMETERIZATION, {"method_note": "non-positive segment duration"}
            max_delta = max(abs(a - b) for a, b in zip(left, right))
            if max_delta > policy.max_joint_step_rad * max(1, dt // MAX_CERT_SAMPLE_DT_NS + 1):
                return Reason.CERT_SAMPLING_UNBOUNDED, {"method_note": "joint-space motion bound too coarse"}
            subdivisions = max(1, int((dt + MAX_CERT_SAMPLE_DT_NS - 1) // MAX_CERT_SAMPLE_DT_NS))
            for _ in range(subdivisions + 1):
                validity = context.validity_adapter.validate_sample(edge.edge_id, sample_index)
                sample_index += 1
                if not validity.valid:
                    if self._only_intended_support_contact(validity.contacts, policy.intended_support_id):
                        contacts.extend(validity.contacts)
                    else:
                        return Reason.CERT_COLLISION, {
                            "min_clearance_m": validity.min_clearance_m,
                            "contacts": tuple(validity.contacts),
                            "method_note": "MoveIt validity collision",
                        }
                if validity.min_clearance_m <= 0.0:
                    return Reason.CERT_COLLISION, {"min_clearance_m": validity.min_clearance_m}
                min_clearance = validity.min_clearance_m if min_clearance is None else min(
                    min_clearance, validity.min_clearance_m)
        occ = self._occupancy_failure(edge, context.snapshot)
        if occ is not Reason.OK:
            return occ, {"min_clearance_m": min_clearance}
        if min_clearance is None:
            return Reason.CERT_UNKNOWN_INPUT, {"method_note": "no validity samples requested"}
        return Reason.OK, {
            "min_clearance_m": min_clearance,
            "max_joint_velocity_rad": max_velocity,
            "max_joint_acceleration_rad2": max_accel,
            "contacts": tuple(contacts),
        }

    @staticmethod
    def _only_intended_support_contact(contacts: Iterable[Mapping[str, object]], support_id: str) -> bool:
        seen = False
        for contact in contacts:
            seen = True
            if contact.get("type") != "support_contact" or contact.get("support_id") != support_id:
                return False
        return seen

    @staticmethod
    def _occupancy_failure(edge: GraphEdge, snapshot: PlanningSnapshot) -> Reason:
        for cell in edge.attributes.get("occupancy_cells", ()):
            i, j = int(cell[0]), int(cell[1])
            state = snapshot.state_at(i, j)
            if state is OccupancyCellState.OCCUPIED:
                return Reason.CERT_OCCUPANCY
            if state is OccupancyCellState.UNKNOWN:
                return Reason.CERT_OCCUPANCY
        return Reason.OK


__all__ = [
    "CertificationContext",
    "CertificationPolicy",
    "MAX_CERT_SAMPLE_DT_NS",
    "TrajectoryCertifier",
]
