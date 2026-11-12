"""Node-side, read-only native validation of an original operation suffix.

This port never plans, repairs knots, changes J6 or applies a scene. Its caller
owns issued-certificate liveness, current observation/map/attachment freshness,
actual full-footprint contact and settled-box occupancy admission. Admissions
are required callbacks; incomplete or non-boolean approvals cannot authorize
a suffix.
"""
from dataclasses import dataclass, replace
import copy
import hashlib
import json
import math
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from luggage_description.frame_contracts import vector
from luggage_description.frame_transforms import rotate_vector
from luggage_planning.joint_operation.certification import _wire_leg, _valid_response
from luggage_planning.joint_operation.contracts import OPERATION_JOINTS, admit_operation_plan, _same_pose
from luggage_planning.joint_operation.execution import ValidationReceipt
from luggage_planning.joint_operation.scenes import (
    PHASES, build_operation_scenes, validate_phase_payloads, _reset_payload_contacts,
)
from luggage_planning.moveit_state_validity import MAX_CERT_SAMPLE_DT_NS
from luggage_planning.placement_prior.contracts import Snapshot, Transform
from st09_contracts.attachment_geometry import AttachmentGeometry


@dataclass(frozen=True)
class OperationRevalidation:
    receipt: ValidationReceipt
    # New evidence for the original path in the current scene, not a new plan.
    native_validation_id: str = ''
    current_scene_sha256: str = ''


def settled_box_from_release(model, plan, geometry, world_from_container, release_drop_m):
    """Actual frozen G at the original release joint endpoint, then policy drop."""
    tool = Transform.from_matrix(model.urdf.tcp(plan.legs[4].knots[-1].positions,model.path))
    box = tool.compose(Transform(geometry.t_tool_box_translation,geometry.t_tool_box_quaternion_xyzw))
    drop = rotate_vector(world_from_container.quaternion_xyzw,(0.,0.,release_drop_m))
    settled = Transform(tuple(a-b for a,b in zip(box.translation,drop)),box.quaternion_xyzw)
    return plan.world_from_settled_box if _same_pose(settled,plan.world_from_settled_box) else settled


class NativeOperationRevalidationPort:
    def __init__(self, node, endpoint_model, *, velocity_limits,
                 acceleration_limits, release_drop_m, start_tolerance_rad,
                 recorder=None, max_samples=50000):
        from luggage_msgs.srv import ValidateRouteV2
        self.service = ValidateRouteV2
        self.client = node.create_client(ValidateRouteV2, '/validate_route_v2')
        self.model = endpoint_model
        for limits in (velocity_limits, acceleration_limits):
            if len(limits) != 6 or any(not math.isfinite(v) or v <= 0 for v in limits):
                raise ValueError('six positive native certification limits required')
        if (not math.isfinite(release_drop_m) or release_drop_m <= 0
                or not math.isfinite(start_tolerance_rad) or start_tolerance_rad < 0
                or type(max_samples) is not int or not 12 <= max_samples <= 50000):
            raise ValueError('explicit finite release/start policy and bounded sample cap required')
        self.velocity_limits = tuple(velocity_limits)
        self.acceleration_limits = tuple(acceleration_limits)
        self.release_drop_m = release_drop_m
        self.start_tolerance_rad = start_tolerance_rad
        self.max_samples, self.recorder, self.inflight = max_samples, recorder, None

    def validate(self, plan, *, expected_identity, expected_digest, snapshot,
                 pickup_object_id, next_leg_index, actual_joints, geometry,
                 certificate_current, source_current, admit_actual_contact, admit_settled_box,
                 deadline, cancelled):
        """Validate remaining original legs from one captured current state.

        `geometry` is None before pickup, the provider's actual frozen geometry
        while carrying, and the last confirmed geometry after release. The
        caller must retain it after release even though no payload is attached.
        Callbacks return literal bool and receive the same immutable snapshot;
        the caller must implement them using its actual owners, not predictions.
        """
        from moveit_msgs.msg import PlanningScene
        from rclpy.serialization import deserialize_message, serialize_message

        phase = ''
        q = ()
        scene_hash = ''
        def result(reason, validation_id=''):
            digest = geometry.digest() if phase == 'carry' and type(geometry) is AttachmentGeometry else ''
            receipt = ValidationReceipt(plan.identity, plan.plan_digest, plan.certificate_id,
                next_leg_index, not bool(reason), reason or 'NATIVE_SUFFIX_CHECKED', q, digest)
            return OperationRevalidation(receipt, validation_id, scene_hash)
        def stopped():
            return cancelled() or time.monotonic() >= deadline
        def current():
            return (certificate_current(plan) is True
                    and source_current(snapshot, geometry) is True)

        try:
            admit_operation_plan(plan, expected_identity, expected_digest)
            if (type(next_leg_index) is not int or not 0 <= next_leg_index < 6
                    or tuple(leg.name for leg in plan.legs) !=
                    ('align','pick_contact','pick_lift','carry','place_release','place_retreat')
                    or tuple(leg.phase for leg in plan.legs) !=
                    ('pick','pick','carry','carry','carry','retreat')):
                return result('UNSUPPORTED_COMPLETE_CHAIN')
            phase = plan.legs[next_leg_index].phase
            if not isinstance(snapshot, Snapshot):
                return result('CURRENT_SNAPSHOT_REQUIRED')
            scene_hash = hashlib.sha256(snapshot.scene_cdr).hexdigest()
            identity = plan.identity
            if ((snapshot.scene_revision, snapshot.map_revision, snapshot.geometry_hash, snapshot.model_hash) !=
                    (identity.scene_revision, identity.map_revision, identity.geometry_hash, identity.model_sha256)
                    or snapshot.model_hash != self.model.model_sha256):
                return result('CURRENT_SOURCE_IDENTITY_MISMATCH')
            q = vector(actual_joints, 6, 'actual_joints')
            if max(abs(a-b) for a,b in zip(q,plan.legs[next_leg_index].knots[0].positions)) > self.start_tolerance_rad:
                return result('ACTUAL_START_JOINTS_MISMATCH')
            if phase == 'pick':
                if geometry is not None:
                    return result('UNEXPECTED_PREPICK_ATTACHMENT')
                active_geometry = plan.predicted_attachment
            elif type(geometry) is AttachmentGeometry:
                geometry.__post_init__()
                active_geometry = geometry
            else:
                return result('ACTUAL_GEOMETRY_REQUIRED')
            if stopped():
                return result('CANCELLED_OR_DEADLINE')
            if not current():
                return result('CERTIFICATE_OR_SOURCE_NOT_CURRENT')
            if phase == 'carry' and admit_actual_contact(plan, snapshot, active_geometry,
                    deadline=deadline, cancelled=cancelled) is not True:
                return result('ACTUAL_CONTACT_ADMISSION_REFUSED')
            if stopped():
                return result('CANCELLED_OR_DEADLINE')

            release_tool = Transform.from_matrix(self.model.urdf.tcp(
                plan.legs[4].knots[-1].positions, self.model.path))
            tool_box = Transform(active_geometry.t_tool_box_translation,
                                 active_geometry.t_tool_box_quaternion_xyzw)
            release_box = release_tool.compose(tool_box)
            intended = plan.world_from_place_box
            error_m = np.linalg.norm(np.asarray(release_box.translation)-intended.translation)
            error_rad = Rotation.from_quat(intended.quaternion_xyzw).inv() * Rotation.from_quat(release_box.quaternion_xyzw)
            if (error_m > self.model.position_tolerance_m
                    or error_rad.magnitude() > self.model.orientation_tolerance_rad):
                return result('ACTUAL_GRASP_MISSES_BOUND_BOX_TARGET')
            up_gap = rotate_vector(snapshot.world_from_container.quaternion_xyzw,
                                   (0.,0.,self.release_drop_m))
            expected_release = tuple(a+b for a,b in zip(plan.world_from_settled_box.translation,up_gap))
            if (max(abs(a-b) for a,b in zip(expected_release,intended.translation)) > 1e-9
                    or abs(sum(a*b for a,b in zip(intended.quaternion_xyzw,
                                                plan.world_from_settled_box.quaternion_xyzw))) < 1.-1e-12):
                return result('RELEASE_SETTLED_POLICY_MISMATCH')
            settled_from_grasp = settled_box_from_release(self.model,plan,active_geometry,
                snapshot.world_from_container,self.release_drop_m)
            # Layout admission belongs to the bound requested slot. FK accuracy
            # was checked above; exact FK-derived geometry remains in every
            # native collision phase below, including the released obstacle.
            if admit_settled_box(snapshot, active_geometry, plan.world_from_settled_box,
                    deadline=deadline, cancelled=cancelled) is not True:
                return result('SETTLED_BOX_ADMISSION_REFUSED')
            if stopped():
                return result('CANCELLED_OR_DEADLINE')

            live = deserialize_message(snapshot.scene_cdr, PlanningScene)
            if live.is_diff or live.robot_state.is_diff:
                return result('FULL_CURRENT_SCENE_REQUIRED')
            state = live.robot_state.joint_state
            if len(state.name) != len(state.position) or len(set(state.name)) != len(state.name):
                return result('CURRENT_SCENE_JOINTS_INVALID')
            positions = dict(zip(state.name,state.position))
            scene_q = vector(tuple(positions[name] for name in OPERATION_JOINTS),6,'scene_joints')
            if max(abs(a-b) for a,b in zip(scene_q,q)) > self.start_tolerance_rad:
                return result('CURRENT_SCENE_JOINTS_MISMATCH')
            # Remove only this operation's owned payload in a private copy to
            # construct phase scenes. The live phase is then independently
            # compared with the actual envelope; missing/changed loads refuse.
            baseline = copy.deepcopy(live)
            baseline.robot_state.attached_collision_objects = [
                item for item in baseline.robot_state.attached_collision_objects
                if item.object.id != pickup_object_id]
            phases = build_operation_scenes(replace(snapshot,scene_cdr=serialize_message(baseline)),
                pickup_object_id=pickup_object_id, geometry=active_geometry,
                world_from_pick_box=plan.world_from_pick_box,
                world_from_settled_box=settled_from_grasp)
            decoded = [deserialize_message(phases.for_phase(p).scene_cdr,PlanningScene) for p in PHASES]
            _reset_payload_contacts(live,pickup_object_id)
            actual_phase = list(decoded)
            actual_phase[PHASES.index(phase)] = live
            validate_phase_payloads(actual_phase,pickup_object_id=pickup_object_id,
                geometry=active_geometry,world_from_pick_box=plan.world_from_pick_box,
                world_from_settled_box=settled_from_grasp)
            validate_phase_payloads(decoded,pickup_object_id=pickup_object_id,
                geometry=active_geometry,world_from_pick_box=plan.world_from_pick_box,
                world_from_settled_box=settled_from_grasp)
        except Exception as error:
            return result('REVALIDATION_ADMISSION_REFUSED:'+str(error))

        if self.inflight is not None and not self.inflight.done():
            return result('REVALIDATION_BACKEND_BUSY')
        if not self.client.service_is_ready():
            return result('REVALIDATION_BACKEND_UNAVAILABLE')
        suffix = plan.legs[next_leg_index:]
        count = sum(sum(max(1,(b.time_from_start_ns-a.time_from_start_ns+MAX_CERT_SAMPLE_DT_NS-1)
            //MAX_CERT_SAMPLE_DT_NS)+1 for a,b in zip(leg.knots,leg.knots[1:])) for leg in suffix)
        if count > self.max_samples:
            return result('REVALIDATION_SAMPLE_CAP')
        request = self.service.Request(request_id=plan.identity.request_id,
            scene_revision=plan.identity.scene_revision, route_digest=plan.plan_digest,
            scene_digest=hashlib.sha256(json.dumps(phases.digests).encode()).hexdigest(),
            load_state_scenes=decoded,max_sample_dt_ns=float(MAX_CERT_SAMPLE_DT_NS),max_sample_step_m=.01,
            joint_velocity_limits=list(self.velocity_limits),joint_acceleration_limits=list(self.acceleration_limits),
            distance_stride=2**30)
        request.legs = [_wire_leg(leg) for leg in suffix]
        if stopped():
            return result('CANCELLED_OR_DEADLINE')
        request.budget_s = deadline-time.monotonic()
        try:
            raw_request = serialize_message(request)
            if self.recorder:
                self.recorder('request',raw_request)
            self.inflight = self.client.call_async(request)
            ready = threading.Event()
            self.inflight.add_done_callback(lambda _:ready.set())
            while not ready.wait(min(.01,max(0.,deadline-time.monotonic()))):
                if stopped():
                    return result('CANCELLED_OR_DEADLINE')
            response = self.inflight.result()
            raw_response = serialize_message(response)
            if self.recorder:
                self.recorder('response',raw_response)
            if stopped():
                return result('CANCELLED_OR_DEADLINE')
            error = _valid_response(response,request,self.model.model_sha256)
            if error:
                if self.recorder and any(not leg.valid for leg in response.leg_results):
                    from luggage_planning.routing.collision_diagnostics import route_refusal_summary
                    self.recorder('refusal_summary.json',json.dumps(
                        route_refusal_summary(request,response,model=self.model),
                        sort_keys=True,allow_nan=False).encode())
                return result(error)
        except Exception as error:
            return result('REVALIDATION_TRANSPORT_REFUSED:'+str(error))
        try:
            if not current():
                return result('CERTIFICATE_OR_SOURCE_CHANGED_DURING_REVALIDATION')
        except Exception as error:
            return result('FINAL_SOURCE_READ_REFUSED:'+str(error))
        if stopped():
            return result('CANCELLED_OR_DEADLINE')
        return result('',hashlib.sha256(raw_request+raw_response).hexdigest())
