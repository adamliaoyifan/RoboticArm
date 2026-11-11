"""Pure event barriers for consumption of an already certified operation plan.

Commands are effects for a node adapter, never ROS calls. Validation effects
must check live certification, actual q, current collision scene and attachment
provider freshness. The attached-suffix validator must also check the intended
box target under the actual grasp, not merely collision freedom. Refusal leaves
the payload held for bounded repair/recovery owned by the caller.
"""
from dataclasses import dataclass
import math

from luggage_description.frame_contracts import vector
from luggage_planning.joint_operation_contracts import (
    OperationIdentity, admit_operation_plan, require,
)
from st09_contracts.attachment import AttachmentRecord, AttachmentValidity
from st09_contracts.attachment_geometry import AttachmentGeometry


@dataclass(frozen=True)
class ExecutionCommand:
    kind: str
    token: int
    plan_digest: str
    leg_index: int


@dataclass(frozen=True)
class ValidationReceipt:
    identity: OperationIdentity
    plan_digest: str
    certificate_id: str
    next_leg_index: int
    accepted: bool
    reason: str
    actual_joints: tuple
    # Empty at preflight/after release; measured frozen geometry during carry.
    attachment_geometry_digest: str


class JointOperationExecution:
    """One request, serialized callback owner. Repeated callbacks are inert.

    The node dispatches each returned command once, echoes its token, and obtains
    motion payloads through pending_leg(). A cancel-motion command must reach a
    terminal controller result before recovery; cancellation itself never opens
    vacuum. No global planner is called by this consumer.
    """

    def __init__(self, *, start_tolerance_rad):
        require(type(start_tolerance_rad) in (int, float)
                and math.isfinite(start_tolerance_rad) and start_tolerance_rad >= 0,
                "INVALID_NUMERIC_PAYLOAD", "start_tolerance_rad",
                "configured finite nonnegative tolerance", start_tolerance_rad)
        self.start_tolerance_rad = float(start_tolerance_rad)
        self.state = "IDLE"
        self.reason = ""
        self.plan = None
        self.next_leg_index = 0
        self.may_have_payload = False
        self.actual_geometry = None
        self._token = 0
        self._pending = None

    def _command(self, kind, state):
        self._token += 1
        self.state = state
        self._pending = ExecutionCommand(kind, self._token, self.plan.plan_digest,
                                          self.next_leg_index)
        return self._pending

    def _matches(self, token, state):
        return self.state == state and self._pending is not None and token == self._pending.token

    def _fault(self, reason):
        self.reason = str(reason)
        self.state = "CARRY_FAULT" if self.may_have_payload else "STOPPED"
        self._pending = None
        return None

    def start(self, plan, expected_identity, expected_digest):
        require(self.state == "IDLE", "OPERATION_ALREADY_STARTED", "state", "IDLE", self.state)
        self.plan = admit_operation_plan(plan, expected_identity, expected_digest)
        return self._command("validate_start", "VALIDATING_START")

    def pending_leg(self, token):
        require(self._matches(token, "EXECUTING"), "STALE_OPERATION_CALLBACK",
                "motion_token", "current executing token", token)
        # Frozen tuple of original knots; no retiming, winding reduction or J6 edit.
        return self.plan.legs[self.next_leg_index]

    def validation_finished(self, token, receipt):
        if not any(self._matches(token, state) for state in (
                "VALIDATING_START", "VALIDATING_ATTACHED", "VALIDATING_RELEASED")):
            return None
        require(type(receipt) is ValidationReceipt, "INVALID_OPERATION_VALIDATION",
                "receipt", "ValidationReceipt", type(receipt).__name__)
        expected_geometry = self.actual_geometry.digest() if self.may_have_payload else ""
        valid_binding = (receipt.identity == self.plan.identity
                         and receipt.plan_digest == self.plan.plan_digest
                         and receipt.certificate_id == self.plan.certificate_id
                         and receipt.next_leg_index == self.next_leg_index
                         and receipt.attachment_geometry_digest == expected_geometry)
        if not valid_binding:
            return self._fault("VALIDATION_IDENTITY_MISMATCH")
        if type(receipt.accepted) is not bool or not receipt.accepted:
            return self._fault(receipt.reason or "SUFFIX_REVALIDATION_REFUSED")
        q = vector(receipt.actual_joints, 6, "actual_joints")
        start = self.plan.legs[self.next_leg_index].knots[0].positions
        if max(abs(a-b) for a, b in zip(q, start)) > self.start_tolerance_rad:
            return self._fault("ACTUAL_START_JOINTS_MISMATCH")
        return self._command("execute_leg", "EXECUTING")

    def motion_finished(self, token, *, success):
        if self._matches(token, "CANCELLING"):
            return self._fault("CANCELLED")
        if not self._matches(token, "EXECUTING"):
            return None
        if type(success) is not bool or not success:
            return self._fault("MOTION_FAILED")
        self.next_leg_index += 1
        if self.next_leg_index == len(self.plan.legs):
            self.state, self.reason, self._pending = "COMPLETE", "OK", None
            return None
        barrier = next((b for b in self.plan.barriers
                        if b.before_leg_index == self.next_leg_index), None)
        if barrier is None:
            # Scene/certificate liveness and start-q checks still run in the
            # timed-path executor before every controller submission.
            return self._command("execute_leg", "EXECUTING")
        if barrier.event == "attachment_confirmed":
            # Attachment may succeed even if its callback later fails.
            self.may_have_payload = True
            return self._command("confirm_attachment", "WAIT_ATTACHMENT")
        return self._command("confirm_release", "WAIT_RELEASE")

    def attachment_confirmed(self, token, *, record, geometry, generation):
        if not self._matches(token, "WAIT_ATTACHMENT"):
            return None
        identity = self.plan.identity
        valid = (type(record) is AttachmentRecord and type(geometry) is AttachmentGeometry
                 and record.validity is AttachmentValidity.VALID
                 and record.source.route_grade
                 and record.request_id == identity.request_id
                 and record.box_id == identity.box_id
                 and record.scene_revision == identity.scene_revision
                 and record.tcp_frame == "suction_contact_frame"
                 and record.box_frame == "world"
                 and type(generation) is int and generation == identity.generation
                 and record.dimensions_wdh == geometry.measured_dimensions_wdh
                 and record.t_tcp_box_translation == geometry.t_tool_box_translation
                 and record.t_tcp_box_quaternion_xyzw == geometry.t_tool_box_quaternion_xyzw
                 and record.collision_envelope == geometry.conservative)
        if not valid:
            return self._fault("MEASURED_ATTACHMENT_REQUIRED")
        self.actual_geometry = geometry
        return self._command("validate_attached_suffix", "VALIDATING_ATTACHED")

    def release_confirmed(self, token, *, success):
        if not self._matches(token, "WAIT_RELEASE"):
            return None
        if type(success) is not bool or not success:
            return self._fault("RELEASE_NOT_CONFIRMED")
        self.may_have_payload = False
        self.actual_geometry = None
        # Adapter applies world obstacle at measured/predicted released pose,
        # then validates retreat including that obstacle before replying.
        return self._command("apply_release_and_validate_retreat", "VALIDATING_RELEASED")

    def event_failed(self, token, reason):
        if self._pending is None or token != self._pending.token:
            return None
        if self.state in ("EXECUTING", "CANCELLING"):
            # A transport error does not establish that controller motion ended.
            self.reason = str(reason)
            return self.cancel()
        return self._fault(reason)

    def cancel(self):
        if self.state == "EXECUTING":
            # Keep the motion token: terminal controller result discharges it.
            self.state = "CANCELLING"
            return ExecutionCommand("cancel_motion", self._pending.token,
                                    self.plan.plan_digest, self.next_leg_index)
        if self.state in ("IDLE", "COMPLETE", "STOPPED", "CARRY_FAULT", "CANCELLING"):
            return None
        return self._fault("CANCELLED")
