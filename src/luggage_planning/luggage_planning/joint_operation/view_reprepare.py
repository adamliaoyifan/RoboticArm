"""Read-only continuation through the existing ordinary preparation owner.

The caller supplies an actual execution/settling receipt. An issued view alone
does not prove execution; this adapter never moves or adopts a recovery budget.
"""
from dataclasses import asdict
import json
import math
import time

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.serialization import deserialize_message

from luggage_perception.pickup_view_service import view_from_message
from luggage_perception.ros_suction_domain import domain_to_message
from luggage_perception.stationary_pickup_association import (
    StationaryPickupPolicy, associate_stationary_pickup,
)
from luggage_planning.joint_operation.contact_observation import UnresolvedContactProbe
from luggage_planning.joint_operation.pickup_observation import assess_pickup_observation
from luggage_planning.joint_operation.view_information import compare_pickup_region


def _json(value):
    return json.dumps(value, sort_keys=True, allow_nan=False, separators=(',', ':'))


class PickupViewReprepareOwner:
    def __init__(self, views):
        from luggage_msgs.srv import ReprepareAfterPickupViewV2
        self.views, self.node = views, views.node
        self.service = self.node.create_service(ReprepareAfterPickupViewV2,
            '/joint_operation/reprepare_after_pickup_view', self.handle,
            callback_group=ReentrantCallbackGroup())

    def _request(self, request, response, deadline):
        """Caller holds the ordinary owner's lock; return an immutable handoff."""
        from luggage_msgs.msg import PickupViewPlanV2
        node, views = self.node, self.views
        if node.observer is None:
            raise ValueError('PICKUP_VIEW_REQUIRES_O_BACKEND')
        if (not node.budget.admits(request.budget_s) or not request.request_id.strip()
                or request.request_id == request.previous_view.task_request_id
                or not request.settled_stamp_valid):
            raise ValueError('PICKUP_VIEW_CONTINUATION_INPUT_INVALID')
        policy = StationaryPickupPolicy(**{name: getattr(request.association_policy, name)
                                          for name in StationaryPickupPolicy.__dataclass_fields__})
        message = request.previous_view
        context = views.contexts.lookup(message.task_request_id, message.domain_sha256)
        receipt = views.contexts.issued(message.task_request_id, message.route_digest)
        if (context is None or receipt is None
                or deserialize_message(receipt.message_cdr, PickupViewPlanV2) != message):
            raise ValueError('EXACT_RETAINED_VIEW_REQUIRED')
        probe = context.probe(message.query_id)
        if probe is None:
            raise ValueError('OWNED_UNRESOLVED_CONTACT_REQUIRED')
        hypothesis = UnresolvedContactProbe(probe.query_id, probe.domain_sha256,
            probe.contact_model_hash, tuple(probe.center_uv_top_m), probe.yaw_rad,
            tuple(probe.footprint_uv_top_m), probe.unproven_cell_count)
        assessment = assess_pickup_observation(context.domain, node.preparation.model,
                                               hypothesis, receipt.view)
        node._record('pickup_view_continuation_original_need.json', _json(asdict(assessment)).encode())
        if assessment.need is None:
            raise ValueError('NO_DECISION_RELEVANT_VIEW_GAP:'+assessment.reason)

        renewed = context.request()
        renewed.request_id = request.request_id
        renewed.detection = request.detection
        renewed.generation = request.generation
        # The only target is the owner-retained placement motivating this view.
        # No target coordinates or observation need are accepted from the client.
        renewed.placement_task_mode = ''
        renewed.slots = [probe.placement]
        renewed.slots[0].identity.request_id = request.request_id
        renewed.candidate_limit = 1
        with node._state_lock:
            joints = node.joints
        now = node.get_clock().now().nanoseconds
        if joints is None or not 0 <= now-joints[0] <= views.max_joint_age_ns:
            raise ValueError('PICKUP_VIEW_CONTINUATION_JOINTS_UNAVAILABLE_OR_STALE')
        renewed.robot_state_stamp_valid = True
        renewed.robot_state_stamp_ns, renewed.start_joints = joints[0], list(joints[1])
        domain = node._current(renewed)
        renewed.observation_id = domain.digest()
        query = views.evidence_service.Request(domain_sha256=domain.digest(),
            detection=request.detection, generation=domain.generation)
        output = node._call(views.evidence_client, query, deadline, 'pickup_view_continuation_acquisition')
        if not output.success:
            raise ValueError('PICKUP_VIEW_ACQUISITION_UNAVAILABLE:'+output.reason)
        view = view_from_message(output.view, domain)
        association = associate_stationary_pickup(context.domain, receipt.view, domain, view,
            settled_stamp_ns=int(request.settled_stamp_ns), stationary=request.pickup_stationary,
            policy=policy)
        response.association_json = _json(asdict(association))
        node._record('pickup_view_continuation_association.json', response.association_json.encode())
        if not association.accepted:
            raise ValueError('PICKUP_VIEW_ASSOCIATION_REFUSED:'+association.reason)
        information = compare_pickup_region(assessment.need, receipt.view, view, association, policy,
            # Generated fixed ROS arrays expose numpy.uint32. Normalize at this
            # wire boundary; the ROS-free core requires ordinary Python ints.
            sample_shape=tuple(int(v) for v in request.information_sample_shape))
        response.information_json = _json(asdict(information))
        response.associated = True
        response.detection, response.top_domain = request.detection, domain_to_message(domain)
        node._record('pickup_view_continuation_information.json', response.information_json.encode())
        renewed.budget_s = deadline-time.monotonic()
        if not node.budget.admits(renewed.budget_s):
            raise TimeoutError('PICKUP_VIEW_CONTINUATION_DEADLINE')
        node._record('pickup_view_continuation_handoff', renewed)
        return renewed

    def handle(self, request, response):
        from luggage_msgs.msg import JointOperationPlanV2
        from luggage_msgs.srv import PrepareJointOperationV2
        began = time.monotonic()
        response.success, response.associated = False, False
        response.plan = JointOperationPlanV2()
        response.reason, response.trace_json = 'PICKUP_VIEW_CONTINUATION_REFUSED', '[]'
        response.unresolved_contacts = []
        node = self.node
        locked = node._active.acquire(blocking=False)
        if not locked:
            response.reason = 'OPERATION_PREPARATION_BUSY'
            return response
        try:
            node._record('pickup_view_continuation_request', request)
            if not math.isfinite(request.budget_s):
                raise ValueError('PICKUP_VIEW_CONTINUATION_INPUT_INVALID')
            renewed = self._request(request, response, began+request.budget_s)
            # Ordinary prepare independently reacquires the same lock and checks
            # domain/current joints again. No partial plan can cross this handoff.
            node._active.release(); locked = False
            prepared = node._prepare(renewed, PrepareJointOperationV2.Response(), latest_map=True)
            response.success, response.reason = prepared.success, prepared.reason
            response.plan, response.unresolved_contacts = prepared.plan, prepared.unresolved_contacts
            response.trace_json = prepared.trace_json
        except Exception as error:
            response.success = False
            response.plan, response.unresolved_contacts = JointOperationPlanV2(), []
            response.reason = 'PICKUP_VIEW_CONTINUATION_REFUSED:'+type(error).__name__+':'+str(error)
        finally:
            if locked:
                node._active.release()
            response.elapsed_s = time.monotonic()-began
            try:
                node._record('pickup_view_continuation_response', response)
            except Exception as error:
                response.success = False
                response.plan, response.unresolved_contacts = JointOperationPlanV2(), []
                response.reason = 'PICKUP_VIEW_CONTINUATION_EVIDENCE_FAILED:'+type(error).__name__
        return response
