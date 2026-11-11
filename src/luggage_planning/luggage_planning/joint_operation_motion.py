"""Submit one admitted operation leg without changing its timed joint path.

The caller supplies the motion lock shared by *all* of its motion entry points,
current-source authorization and native suffix validation. This transport owns
an accepted or unresolved goal until a real terminal result is consumed. It has
no planner, vacuum, scene or occupancy write capability. Use outside the node's
executor callbacks (or with independently spinning callback groups).
"""
from dataclasses import dataclass, field
import json
import math
import threading
import time

from luggage_description.frame_contracts import vector
from luggage_planning.joint_operation_contracts import OPERATION_JOINTS, admit_operation_plan
from luggage_planning.joint_operation_execution import ValidationReceipt
from luggage_planning.joint_operation_revalidation import OperationRevalidation
from st09_contracts.attachment_geometry import AttachmentGeometry


@dataclass(frozen=True)
class MeasuredJointSample:
    positions: tuple
    stamp_ns: int
    stamp_valid: bool


@dataclass(frozen=True)
class MotionOutcome:
    success: bool
    terminal: bool
    submitted: bool
    reason: str
    controller_status: int = 0
    moveit_error_code: int = 0


@dataclass
class _Submission:
    leg: object
    read_joints: object
    dispatch_stamp_ns: int
    handle: object = None
    result: object = None
    rejected: bool = False
    stop_reason: str = ''
    uncertainty: str = ''
    capture_error: str = ''
    cancel_sent: bool = False
    changed: threading.Event = field(default_factory=threading.Event)


class StrictOperationMotionPort:
    """One submission at a time; timeout/cancel is not a terminal receipt.

    `validate_current(plan, index, deadline=..., cancelled=...)` must call the
    actual native revalidation port with a captured current scene/attachment.
    `source_current(plan, index)` checks the actual preparation/source owners.
    Both callbacks are mandatory, and must never acquire the shared motion lock
    recursively. Storage liveness alone is not current-source authorization.
    """

    def __init__(self, node, *, motion_lock, start_tolerance_rad,
                 end_tolerance_rad, max_joint_age_ns, recorder=None):
        from moveit_msgs.action import ExecuteTrajectory
        from rclpy.action import ActionClient
        for tolerance in (start_tolerance_rad, end_tolerance_rad):
            if type(tolerance) not in (int, float) or not math.isfinite(tolerance) or tolerance < 0:
                raise ValueError('explicit finite nonnegative joint tolerances required')
        if type(max_joint_age_ns) is not int or max_joint_age_ns <= 0:
            raise ValueError('explicit positive measured joint age required')
        if not isinstance(motion_lock, type(threading.Lock())):
            raise ValueError('owner shared non-reentrant motion lock required')
        self.node, self.motion_lock, self.recorder = node, motion_lock, recorder
        self.start_tolerance_rad, self.end_tolerance_rad = start_tolerance_rad, end_tolerance_rad
        self.max_joint_age_ns = max_joint_age_ns
        self.action = ExecuteTrajectory
        self.client = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
        self._lock, self._active = threading.RLock(), None

    def _record(self, kind, raw):
        if self.recorder is not None:
            self.recorder(kind, raw)

    def _joints(self, read_joints, expected, tolerance, *, not_before=None):
        sample = read_joints()
        if type(sample) is not MeasuredJointSample or sample.stamp_valid is not True:
            raise ValueError('MEASURED_JOINT_STAMP_REQUIRED')
        if type(sample.stamp_ns) is not int or sample.stamp_ns < 0:
            raise ValueError('INVALID_MEASURED_JOINT_STAMP')
        now = self.node.get_clock().now().nanoseconds
        if not 0 <= now - sample.stamp_ns <= self.max_joint_age_ns:
            raise ValueError('STALE_OR_FUTURE_MEASURED_JOINTS')
        if not_before is not None and sample.stamp_ns < not_before:
            raise ValueError('PRE_SUBMISSION_ENDPOINT_SAMPLE')
        q = vector(sample.positions, 6, 'measured_joint_positions')
        self._record('motion_measured_joints', json.dumps({
            'q': q, 'stamp_ns': sample.stamp_ns, 'clock_ns': now,
            'expected': expected, 'tolerance_rad': tolerance}).encode())
        # Raw joint coordinates: a full turn is not interchangeable here.
        if max(abs(a-b) for a,b in zip(q, expected)) > tolerance:
            raise ValueError('MEASURED_JOINTS_MISMATCH')
        return q

    @staticmethod
    def _admitted(validation, plan, index, geometry):
        if type(validation) is not OperationRevalidation:
            return False
        receipt = validation.receipt
        expected_geometry = geometry.digest() if plan.legs[index].phase == 'carry' else ''
        if (type(receipt) is not ValidationReceipt or receipt.accepted is not True
                or receipt.identity != plan.identity or receipt.plan_digest != plan.plan_digest
                or receipt.certificate_id != plan.certificate_id or receipt.next_leg_index != index
                or receipt.attachment_geometry_digest != expected_geometry):
            return False
        return all(isinstance(value, str) and len(value) == 64
                   and all(c in '0123456789abcdef' for c in value)
                   for value in (validation.native_validation_id, validation.current_scene_sha256))

    def execute(self, plan, index, *, expected_identity, expected_digest,
                geometry, validate_current, source_current, read_joints,
                deadline, cancelled):
        """Validate then submit once. A nonterminal result still owns motion.

        After a nonterminal outcome, call poll_terminal() until it returns a
        terminal outcome. Do not send any other motion or release the payload.
        A transport exception is uncertain and requires owner reconciliation;
        there is deliberately no force-clear API.
        """
        with self._lock:
            if self._active is not None:
                return MotionOutcome(False, False, False, 'MOTION_SUBMISSION_UNRESOLVED')
        if not self.motion_lock.acquire(blocking=False):
            return MotionOutcome(False, True, False, 'SHARED_MOTION_BUSY')
        submitted = False
        try:
            admit_operation_plan(plan, expected_identity, expected_digest)
            if (type(index) is not int or not 0 <= index < 6
                    or tuple(leg.name for leg in plan.legs) !=
                       ('align','pick_contact','pick_lift','carry','place_release','place_retreat')
                    or tuple(leg.phase for leg in plan.legs) !=
                       ('pick','pick','carry','carry','carry','retreat')):
                raise ValueError('UNSUPPORTED_COMPLETE_CHAIN')
            if plan.legs[index].phase == 'carry' and type(geometry) is not AttachmentGeometry:
                raise ValueError('ACTUAL_GEOMETRY_REQUIRED')
            if type(deadline) not in (int,float) or not math.isfinite(deadline):
                raise ValueError('FINITE_DEADLINE_REQUIRED')
            if cancelled() or time.monotonic() >= deadline:
                raise ValueError('CANCELLED_OR_DEADLINE')
            if source_current(plan, index) is not True:
                raise ValueError('CURRENT_SOURCE_REFUSED')
            validation = validate_current(plan, index, deadline=deadline, cancelled=cancelled)
            if not self._admitted(validation, plan, index, geometry):
                raise ValueError('BOUND_NATIVE_ADMISSION_REQUIRED')
            leg = plan.legs[index]
            q = vector(validation.receipt.actual_joints, 6, 'native_actual_joints')
            if max(abs(a-b) for a,b in zip(q, leg.knots[0].positions)) > self.start_tolerance_rad:
                raise ValueError('NATIVE_START_JOINTS_MISMATCH')
            while not self.client.server_is_ready():
                if cancelled() or time.monotonic() >= deadline:
                    raise ValueError('CONTROLLER_NOT_READY')
                time.sleep(min(.01, max(0., deadline-time.monotonic())))
            from rclpy.serialization import serialize_message
            from trajectory_msgs.msg import JointTrajectoryPoint
            goal = self.action.Goal()
            goal.trajectory.joint_trajectory.joint_names = list(OPERATION_JOINTS)
            for knot in leg.knots:
                point = JointTrajectoryPoint()
                point.positions, point.velocities = list(knot.positions), list(knot.velocities)
                point.accelerations, point.effort = list(knot.accelerations), list(knot.effort)
                point.time_from_start.sec, point.time_from_start.nanosec = divmod(knot.time_from_start_ns, 10**9)
                goal.trajectory.joint_trajectory.points.append(point)
            self._record('motion_goal', serialize_message(goal))
            self._record('motion_admission', json.dumps({
                'plan_digest': plan.plan_digest, 'certificate_id': plan.certificate_id,
                'index': index, 'native_validation_id': validation.native_validation_id,
                'scene_sha256': validation.current_scene_sha256}).encode())
            if source_current(plan, index) is not True:
                raise ValueError('CURRENT_SOURCE_CHANGED')
            self._joints(read_joints, leg.knots[0].positions, self.start_tolerance_rad)
            if cancelled() or time.monotonic() >= deadline:
                raise ValueError('CANCELLED_OR_DEADLINE')
            session = _Submission(leg, read_joints, self.node.get_clock().now().nanoseconds)
            with self._lock:
                self._active = session
                # Even a synchronous send failure can mean transmission happened.
                submitted = True
                try:
                    self.client.send_goal_async(goal).add_done_callback(
                        lambda future: self._accepted(session, future))
                except Exception as exc:
                    session.uncertainty = 'GOAL_TRANSPORT_UNCERTAIN:' + type(exc).__name__
            while True:
                outcome = self.poll_terminal()
                if outcome is not None:
                    return outcome
                try:
                    stop = 'CANCELLED' if cancelled() else ('DEADLINE' if time.monotonic() >= deadline else '')
                except Exception:
                    stop = 'CANCELLATION_CHECK_FAILED'
                with self._lock:
                    if stop:
                        self._stop(session, stop)
                    elif session.uncertainty:
                        self._stop(session, 'TRANSPORT_UNCERTAIN')
                    if stop or session.uncertainty:
                        return MotionOutcome(False, False, True,
                            session.uncertainty or session.stop_reason)
                session.changed.wait(min(.01, max(0., deadline-time.monotonic())))
                session.changed.clear()
        except Exception as exc:
            return MotionOutcome(False, not submitted, submitted, str(exc) or type(exc).__name__)
        finally:
            if not submitted:
                self.motion_lock.release()

    def _stop(self, session, reason):
        # Called under _lock, including by a late-acceptance callback.
        session.stop_reason = session.stop_reason or reason
        if session.handle is not None and not session.cancel_sent and session.result is None:
            session.cancel_sent = True
            try:
                session.handle.cancel_goal_async().add_done_callback(
                    lambda future: self._cancel_ack(session, future))
            except Exception as exc:
                session.uncertainty = 'CANCEL_TRANSPORT_UNCERTAIN:' + type(exc).__name__
        session.changed.set()

    def cancel(self):
        """Request stop; an acknowledgement never releases motion ownership."""
        with self._lock:
            if self._active is not None:
                self._stop(self._active, 'CANCELLED')

    def _accepted(self, session, future):
        with self._lock:
            try:
                handle = future.result()
                session.handle, session.rejected = handle, not handle.accepted
                try:
                    self._record('motion_acceptance', json.dumps({
                        'accepted': handle.accepted, 'goal_uuid': list(bytes(handle.goal_id.uuid))}).encode())
                except Exception as exc:
                    session.capture_error = 'ACCEPTANCE:' + type(exc).__name__ + ':' + str(exc)
                    self._stop(session, 'ACCEPTANCE_CAPTURE_FAILED')
                if handle.accepted:
                    handle.get_result_async().add_done_callback(
                        lambda result: self._result(session, result))
                    if session.stop_reason:
                        self._stop(session, session.stop_reason)
            except Exception as exc:
                session.uncertainty = 'ACCEPTANCE_TRANSPORT_UNCERTAIN:' + type(exc).__name__
            session.changed.set()

    def _cancel_ack(self, session, future):
        with self._lock:
            try:
                from rclpy.serialization import serialize_message
                self._record('motion_cancel_ack', serialize_message(future.result()))
            except Exception as exc:
                session.uncertainty = 'CANCEL_ACK_UNCERTAIN:' + type(exc).__name__
            session.changed.set()

    def _result(self, session, future):
        with self._lock:
            try:
                from rclpy.serialization import serialize_message
                result = future.result()
                # Only terminal action statuses end controller ownership.
                if result.status not in (4,5,6):
                    raise ValueError('NONTERMINAL_CONTROLLER_RESULT')
                session.result = result
                try:
                    self._record('motion_terminal', serialize_message(result))
                except Exception as exc:
                    session.capture_error = 'TERMINAL:' + type(exc).__name__ + ':' + str(exc)
            except Exception as exc:
                session.uncertainty = 'RESULT_TRANSPORT_UNCERTAIN:' + type(exc).__name__
            session.changed.set()

    def poll_terminal(self):
        """Consume a real terminal outcome, then release the shared motion lock.

        None means unresolved. A second execution must not be attempted until
        the caller has consumed a terminal outcome for its previous submission.
        The owner must serialize execute() and poll_terminal(); executor callbacks
        only deliver receipts and never consume the owner's terminal outcome.
        """
        with self._lock:
            session = self._active
            if session is None or (not session.rejected and session.result is None):
                return None
            status, code, success = 0, 0, False
            reason = 'CONTROLLER_REJECTED'
            if session.result is not None:
                status, code = session.result.status, session.result.result.error_code.val
                reason = session.stop_reason or ('CONTROLLER_SUCCEEDED' if status == 4 and code == 1
                                                else 'CONTROLLER_FAILED')
                if status == 4 and code == 1 and not session.stop_reason:
                    try:
                        self._joints(session.read_joints, session.leg.knots[-1].positions,
                                     self.end_tolerance_rad, not_before=session.dispatch_stamp_ns)
                        success = True
                    except Exception as exc:
                        reason = 'ENDPOINT_CHECK_FAILED:' + (str(exc) or type(exc).__name__)
            if session.capture_error:
                success, reason = False, 'CONTROLLER_CAPTURE_FAILED:' + session.capture_error
            self._active = None
            self.motion_lock.release()
            return MotionOutcome(success, True, True, reason, status, code)
