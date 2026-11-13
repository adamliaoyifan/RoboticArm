"""Runtime-owned settled-view provenance and non-renewing measured continuation.

This service returns independently prepared plans. It never acquires sensors or
dispatches motion; the complete-operation Action remains the motion boundary.
"""
from dataclasses import fields
import json
import math
import threading
import time

from rclpy.serialization import deserialize_message, serialize_message

from luggage_perception.ros_suction_domain import domain_from_message, require_matching_detection
from luggage_perception.stationary_pickup_association import (
    PickupCandidateCorrespondence, StationaryPickupAssociation, StationaryPickupPolicy,
)
from luggage_planning.capture.ros_boundary_capture import RosBoundaryRecorder
from luggage_planning.joint_operation.adapters import operation_plan_from_message
from luggage_planning.joint_operation.contracts import OperationIdentity
from luggage_planning.joint_operation.rpc import OperationRpc


def retain_terminal_view(ledger, task_id, result):
    """Called only with this runtime's actual Action result, before success reply."""
    if not (result.success and result.submitted and result.motion_terminal
            and not result.may_have_payload and result.settled_stamp_valid):
        raise ValueError('SUCCESSFUL_EMPTY_SETTLED_ACTION_REQUIRED')
    ledger.record_settled_view(task_id, plan_cdr=bytes(serialize_message(result.plan)),
        route_digest=result.plan.route_digest, domain_sha256=result.plan.domain_sha256,
        settled_stamp_ns=int(result.settled_stamp_ns))


def _association(request, response):
    """Decode the owner's proof, then bind every external identity before adoption."""
    record = json.loads(response.association_json)
    record['candidates'] = tuple(PickupCandidateCorrespondence(**row) for row in record['candidates'])
    proof = StationaryPickupAssociation(**record)
    policy = StationaryPickupPolicy(**{f.name: getattr(request.association_policy, f.name)
                                      for f in fields(StationaryPickupPolicy)})
    domain = domain_from_message(response.top_domain)
    require_matching_detection(domain, request.detection, generation=request.generation,
                               contact_model_hash=domain.contact_model_hash)
    if (response.detection != request.detection or proof.accepted is not True
            or (proof.before_domain_sha256, proof.before_view_sha256, proof.after_domain_sha256,
                proof.policy_sha256, proof.settled_stamp_ns) != (
                request.previous_view.domain_sha256, request.previous_view.view_sha256,
                domain.digest(), policy.digest(), request.settled_stamp_ns)):
        raise ValueError('CONTINUATION_ASSOCIATION_BINDING_MISMATCH')
    information = json.loads(response.information_json)
    if (information['before_view_sha256'], information['after_view_sha256'],
            information['association_sha256'], information['policy_sha256'], information['query_id']) != (
            proof.before_view_sha256, proof.after_view_sha256, proof.digest(), proof.policy_sha256,
            request.previous_view.query_id):
        raise ValueError('CONTINUATION_INFORMATION_BINDING_MISMATCH')
    if any((probe.domain_sha256, probe.contact_model_hash, probe.placement.identity.request_id)
           != (domain.digest(), domain.contact_model_hash, request.request_id)
           for probe in response.unresolved_contacts):
        raise ValueError('CONTINUATION_PROBE_BINDING_MISMATCH')
    if response.success:
        identity = OperationIdentity(**{f.name: getattr(response.plan, f.name)
                                       for f in fields(OperationIdentity)})
        if (identity.request_id, identity.observation_id, identity.box_id, identity.generation,
                identity.observation_stamp_ns, identity.contact_model_hash, identity.model_sha256) != (
                request.request_id, domain.digest(), domain.box_id, domain.generation,
                domain.stamp_ns, domain.contact_model_hash, request.previous_view.model_sha256):
            raise ValueError('CONTINUATION_PLAN_BINDING_MISMATCH')
        operation_plan_from_message(response.plan, identity, response.plan.plan_digest)
    elif response.plan.plan_digest:
        raise ValueError('INACTIVE_CONTINUATION_FAILURE_REQUIRED')
    return proof


class PickupViewRuntimeContinuation:
    def __init__(self, view):
        from luggage_msgs.srv import ReprepareAfterPickupViewV2
        self.view, self.parent = view, view.parent
        self.wire = ReprepareAfterPickupViewV2
        self.client = self.parent.node.create_client(self.wire,
            '/joint_operation/reprepare_after_pickup_view', callback_group=self.parent.node._group)
        self.service = self.parent.node.create_service(self.wire,
            '/joint_operation/continue_after_pickup_view', self.handle,
            callback_group=self.parent.node._group)
        self.rpc = None

    def handle(self, request, response):
        from luggage_msgs.msg import JointOperationPlanV2, PickupViewPlanV2
        from std_msgs.msg import String
        parent = self.parent
        began = time.monotonic()
        response.success, response.associated = False, False
        response.plan, response.unresolved_contacts = JointOperationPlanV2(), []
        response.trace_json = '[]'
        response.reason = 'PICKUP_VIEW_CONTINUATION_REFUSED'
        if not parent._active.acquire(blocking=False):
            response.reason = 'OPERATION_BUSY'
            return response
        recorder = None
        archive_lock = threading.RLock()
        archive_state = dict(servicing=True, late_terminal=False)
        try:
            if (parent.session is not None or parent.owner.motion_lock.locked()
                    or parent.ports.rpc.unresolved() or self.rpc is not None and self.rpc.unresolved()):
                raise ValueError('CONTINUATION_REQUIRES_TERMINAL_IDLE_RUNTIME')
            folder = parent.evidence_root/(str(time.monotonic_ns())+'_pickup_continuation')
            recorder = RosBoundaryRecorder(folder, run_id=request.request_id,
                                          max_bytes=parent.evidence_limit)
            recorder.record('continuation_request', request)
            if (not request.settled_stamp_valid or not math.isfinite(request.budget_s)
                    or request.budget_s <= 0):
                raise ValueError('CONTINUATION_LIMITS_OR_SETTLEMENT_INVALID')
            task, cdr = self.view.ledger.settled_view(route_digest=request.previous_view.route_digest,
                domain_sha256=request.previous_view.domain_sha256,
                settled_stamp_ns=int(request.settled_stamp_ns))
            # CDR padding need not repeat. Compare decoded complete ROS messages.
            if deserialize_message(cdr, PickupViewPlanV2) != request.previous_view:
                raise ValueError('EXACT_RUNTIME_SETTLED_VIEW_REQUIRED')
            recorder.record('retained_terminal_view', deserialize_message(cdr, PickupViewPlanV2))
            recorder.record('ledger_before', String(data=json.dumps(self.view.ledger.rows[task], sort_keys=True)))
            remaining = self.view.ledger.continuation_remaining(task)
            renewed = deserialize_message(serialize_message(request), self.wire.Request)
            renewed.budget_s = min(request.budget_s, parent.max_budget, remaining,
                float(parent.node.get_parameter('max_preparation_budget_s').value))
            deadline = began+renewed.budget_s

            def record(name, value):
                recorder.record(name, String(data=value.decode()) if isinstance(value, bytes) else value)

            def late_terminal(_):
                # OperationRpc already recorded the late response. It may not
                # adopt a domain or expose the late plan to an execution client.
                with archive_lock:
                    archive_state['late_terminal'] = True
                    if not archive_state['servicing']:
                        recorder.close()

            self.rpc = OperationRpc(record)
            prepared = self.rpc.call(self.client, renewed, deadline, 'measured_reprepare',
                                     late_response=late_terminal)
            if prepared.associated:
                proof = _association(renewed, prepared)
                self.view.ledger.adopt_acquisition(task, proof)
                recorder.record('ledger_after', String(data=json.dumps(self.view.ledger.rows[task], sort_keys=True)))
            elif prepared.success or prepared.plan.plan_digest or prepared.unresolved_contacts:
                raise ValueError('ASSOCIATED_CONTINUATION_REQUIRED')
            response = prepared
        except Exception as error:
            response.success, response.associated = False, False
            response.plan, response.unresolved_contacts = JointOperationPlanV2(), []
            response.reason = 'PICKUP_VIEW_RUNTIME_CONTINUATION_REFUSED:'+type(error).__name__+':'+str(error)
        finally:
            response.elapsed_s = time.monotonic()-began
            try:
                if recorder is not None:
                    recorder.record('continuation_response', response)
            except Exception as error:
                response.success, response.associated = False, False
                response.plan, response.unresolved_contacts = JointOperationPlanV2(), []
                response.reason = 'CONTINUATION_EVIDENCE_FAILED:'+type(error).__name__
            finally:
                try:
                    with archive_lock:
                        archive_state['servicing'] = False
                        if recorder is not None and (self.rpc is None or not self.rpc.unresolved()
                                                     or archive_state['late_terminal']):
                            recorder.close()
                except Exception as error:
                    response.success, response.associated = False, False
                    response.plan, response.unresolved_contacts = JointOperationPlanV2(), []
                    response.reason = 'CONTINUATION_EVIDENCE_CLOSE_FAILED:'+type(error).__name__
                finally:
                    parent._active.release()
        return response
