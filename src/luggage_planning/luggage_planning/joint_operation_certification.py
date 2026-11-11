"""Predicted-operation certification through the existing private MoveIt gate.

This is preparation evidence. It does not create an actual attachment record
or authorize execution. The execution barriers still need their own live checks.
The native checker sees controller-interpolated samples and original knots in
three complete private scenes; this port independently checks endpoint FK.
"""
import hashlib
import json
import math
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation
from luggage_description.frame_transforms import rotate_vector

from luggage_planning.joint_operation_contracts import (
    OPERATION_JOINTS, admit_operation_plan, operation_digest,
)
from luggage_planning.joint_operation_planner import OperationCertification
from luggage_planning.joint_operation_scenes import PHASES, validate_phase_payloads
from luggage_planning.moveit_state_validity import (
    MAX_CERT_SAMPLE_DT_NS, sample_grid, sample_states,
)
from luggage_planning.offline_ik import Urdf
from st09_contracts.graph import TimedTrajectory


class EndpointModel:
    """FK from the exact captured native model, including world-to-base TF."""
    def __init__(self, robot_description, robot_description_semantic, *,
                 position_tolerance_m, orientation_tolerance_rad):
        for value in (position_tolerance_m, orientation_tolerance_rad):
            if type(value) not in (float, int) or not math.isfinite(value) or value <= 0:
                raise ValueError('explicit finite positive FK tolerances required')
        self.model_sha256 = hashlib.sha256(
            (robot_description+robot_description_semantic).encode()).hexdigest()
        self.urdf = Urdf(robot_description)
        self.path = self.urdf.path_to('suction_contact_frame')
        moving = tuple(name for name in self.path if self.urdf.joints[name]['type'] != 'fixed')
        if (self.urdf.root_link != 'world' or moving != OPERATION_JOINTS
                or any(self.urdf.joints[name]['type'] != 'revolute' for name in moving)):
            raise ValueError('world-rooted canonical six-revolute tool model required')
        self.position_tolerance_m = float(position_tolerance_m)
        self.orientation_tolerance_rad = float(orientation_tolerance_rad)

    def matches(self, joints, target):
        actual = self.urdf.tcp(joints, self.path)
        target_rotation = Rotation.from_quat(target.quaternion_xyzw).as_matrix()
        position_error = np.linalg.norm(actual[:3, 3]-np.asarray(target.translation))
        angle_error = Rotation.from_matrix(target_rotation.T @ actual[:3, :3]).magnitude()
        return position_error <= self.position_tolerance_m and angle_error <= self.orientation_tolerance_rad


def _trajectory(leg):
    return TimedTrajectory(leg.joint_names,
        tuple(k.positions for k in leg.knots), tuple(k.velocities for k in leg.knots),
        tuple(k.accelerations for k in leg.knots), tuple(k.time_from_start_ns for k in leg.knots))


def _wire_leg(leg):
    from luggage_msgs.msg import RouteLegV2
    from trajectory_msgs.msg import JointTrajectoryPoint
    trajectory = _trajectory(leg)
    times, states = sample_grid(trajectory), sample_states(trajectory)
    keep = [0]
    for i in range(1, len(times)):
        if times[i] > times[keep[-1]]:
            keep.append(i)
    result = RouteLegV2()
    result.edge_id, result.edge_class = leg.name, leg.name.upper()
    result.carrying, result.load_state_index = leg.phase == 'carry', PHASES.index(leg.phase)
    result.joint_names = list(leg.joint_names)
    result.n_samples = len(keep)
    result.sample_times_ns = [float(times[i]) for i in keep]
    result.sample_positions_flat = [float(q) for i in keep for q in states[i]]
    result.knots.joint_names = list(leg.joint_names)
    for knot in leg.knots:
        point = JointTrajectoryPoint()
        for name in ('positions', 'velocities', 'accelerations', 'effort'):
            setattr(point, name, list(getattr(knot, name)))
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(knot.time_from_start_ns, 10**9)
        result.knots.points.append(point)
    # No support exemption or fixed TCP constraint on a varying motion leg.
    return result


def _valid_response(response, request, model_sha256):
    if (response.request_id != request.request_id or response.route_digest != request.route_digest
            or response.scene_digest != request.scene_digest or response.model_sha256 != model_sha256):
        return 'CERTIFICATION_IDENTITY_MISMATCH'
    if response.result != 0 or response.all_valid is not True:
        return 'NATIVE_CERTIFICATION_REFUSED:'+str(response.detail)
    if len(response.leg_results) != len(request.legs):
        return 'INCOMPLETE_CERTIFICATION'
    for result, leg in zip(response.leg_results, request.legs):
        if (result.edge_id != leg.edge_id or result.valid is not True or result.reason
                or result.n_samples != leg.n_samples or result.first_invalid_sample != leg.n_samples
                or len(result.min_distance_m) != leg.n_samples
                or result.contacts or result.support_contacts):
            return 'INVALID_LEG_CERTIFICATION:'+leg.edge_id
        # This preserves the existing contact-free policy: measured zero or
        # negative clearance rejects. Native collision checks run every sample;
        # NaN means distance was not measured at that sample.
        if not result.min_distance_m or math.isnan(result.min_distance_m[0]):
            return 'MISSING_CLEARANCE_MEASUREMENT:'+leg.edge_id
        if any(not math.isnan(v) and v <= 0 for v in result.min_distance_m):
            return 'NONPOSITIVE_CLEARANCE:'+leg.edge_id
    return ''


class NativeOperationCertificationPort:
    """Optional recorder receives (kind, captured CDR bytes), once per message."""
    def __init__(self, node, endpoint_model, *, velocity_limits, acceleration_limits,
                 release_drop_m, recorder=None, max_samples=50000):
        from luggage_msgs.srv import ValidateRouteV2
        self.service = ValidateRouteV2
        self.client = node.create_client(ValidateRouteV2, '/validate_route_v2')
        self.model = endpoint_model
        for limits in (velocity_limits, acceleration_limits):
            if len(limits) != 6 or any(not math.isfinite(v) or v <= 0 for v in limits):
                raise ValueError('six explicit positive certification limits required')
        if type(max_samples) is not int or not 12 <= max_samples <= 50000:
            raise ValueError('bounded certification sample count required')
        if type(release_drop_m) not in (float,int) or not math.isfinite(release_drop_m) or release_drop_m <= 0:
            raise ValueError('explicit positive deployment release policy required')
        self.release_drop_m = float(release_drop_m)
        self.velocity_limits, self.acceleration_limits = tuple(velocity_limits), tuple(acceleration_limits)
        self.max_samples, self.recorder, self.inflight = max_samples, recorder, None

    def certify(self, draft, candidate, *, deadline, cancelled):
        from moveit_msgs.msg import PlanningScene
        from rclpy.serialization import deserialize_message, serialize_message
        def receipt(reason, certificate_id=''):
            return OperationCertification(draft.identity, candidate.candidate_id,
                operation_digest(draft), candidate.scenes.digests, not bool(reason),
                reason or 'CONTROLLER_SAMPLED_PRIVATE_SCENES', certificate_id)
        def stopped():
            return cancelled() or time.monotonic() >= deadline
        admit_operation_plan(draft, draft.identity, draft.plan_digest)
        if stopped():
            return receipt('CANCELLED_OR_DEADLINE')
        if self.inflight is not None and not self.inflight.done():
            return receipt('CERTIFICATION_BACKEND_BUSY')
        if draft.identity.model_sha256 != self.model.model_sha256:
            return receipt('FK_MODEL_MISMATCH')
        if ((draft.world_from_pick_box, draft.world_from_pick_tool,
             draft.world_from_place_box, draft.world_from_place_tool,
             draft.world_from_settled_box, draft.predicted_attachment) !=
            (candidate.world_from_pick_box, candidate.world_from_pick_tool,
             candidate.world_from_release_box, candidate.world_from_release_tool,
             candidate.world_from_settled_box, candidate.predicted_attachment)):
            return receipt('CANDIDATE_PLAN_MISMATCH')
        up_gap = rotate_vector(candidate.scenes.pick.world_from_container.quaternion_xyzw,
                               (0.,0.,self.release_drop_m))
        expected_release = tuple(a+b for a,b in zip(draft.world_from_settled_box.translation,up_gap))
        if not np.allclose(draft.world_from_place_box.translation, expected_release, atol=1e-9, rtol=0):
            return receipt('RELEASE_SETTLED_POLICY_MISMATCH')
        if not self.client.service_is_ready():
            return receipt('CERTIFICATION_BACKEND_UNAVAILABLE')
        expected_names = ('align', 'pick_contact', 'pick_lift', 'carry', 'place_release', 'place_retreat')
        if (tuple(leg.name for leg in draft.legs) != expected_names
                or tuple(leg.phase for leg in draft.legs) !=
                ('pick','pick','carry','carry','carry','retreat')):
            return receipt('UNSUPPORTED_COMPLETE_CHAIN')
        target_poses = (candidate.pick_approach, draft.world_from_pick_tool,
                        candidate.pick_lift, candidate.place_entry,
                        draft.world_from_place_tool, candidate.place_retreat)
        if any(not self.model.matches(leg.knots[-1].positions, target)
               for leg, target in zip(draft.legs, target_poses)):
            return receipt('ENDPOINT_FK_MISMATCH')
        request = self.service.Request()
        request.request_id, request.scene_revision = draft.identity.request_id, draft.identity.scene_revision
        request.route_digest = operation_digest(draft)
        request.scene_digest = hashlib.sha256(json.dumps(candidate.scenes.digests).encode()).hexdigest()
        request.load_state_scenes = [deserialize_message(candidate.scenes.for_phase(p).scene_cdr,
                                                        PlanningScene) for p in PHASES]
        try:
            validate_phase_payloads(request.load_state_scenes,
                pickup_object_id=candidate.scenes.pickup_object_id, geometry=draft.predicted_attachment,
                world_from_pick_box=draft.world_from_pick_box,
                world_from_settled_box=draft.world_from_settled_box)
        except ValueError as error:
            return receipt('PHASE_GEOMETRY_MISMATCH:'+str(error))
        request.max_sample_dt_ns, request.max_sample_step_m = float(MAX_CERT_SAMPLE_DT_NS), .01
        request.joint_velocity_limits, request.joint_acceleration_limits = list(self.velocity_limits), list(self.acceleration_limits)
        request.distance_stride = 2**30  # every sample collision-checked; once per leg distance, existing policy
        # Estimate the grid before allocating interpolated samples.
        count = sum(sum(max(1, (b.time_from_start_ns-a.time_from_start_ns+MAX_CERT_SAMPLE_DT_NS-1)
                            //MAX_CERT_SAMPLE_DT_NS)+1 for a,b in zip(leg.knots,leg.knots[1:]))
                    for leg in draft.legs)
        if count > self.max_samples:
            return receipt('CERTIFICATION_SAMPLE_CAP')
        request.legs = [_wire_leg(leg) for leg in draft.legs]
        if stopped():
            return receipt('CANCELLED_OR_DEADLINE')
        request.budget_s = deadline-time.monotonic()
        raw_request = serialize_message(request)
        if self.recorder:
            self.recorder('request', raw_request)
        ready = threading.Event()
        self.inflight = self.client.call_async(request)
        self.inflight.add_done_callback(lambda _: ready.set())
        while not ready.wait(min(.01, max(0., deadline-time.monotonic()))):
            if stopped():
                return receipt('CANCELLED_OR_DEADLINE')
        response = self.inflight.result()
        raw_response = serialize_message(response)
        if self.recorder:
            self.recorder('response', raw_response)
        if stopped():
            return receipt('CANCELLED_OR_DEADLINE')
        error = _valid_response(response, request, self.model.model_sha256)
        if error:
            return receipt(error)
        certificate_id = hashlib.sha256(raw_request+raw_response).hexdigest()
        return receipt('', certificate_id)
