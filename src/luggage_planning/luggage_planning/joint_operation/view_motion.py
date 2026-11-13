"""Plan-only native preparation for a source-bound empty-arm camera view.

The owner provides the admitted complete O/static snapshot and its current-source
check. This module never updates maps/scenes, executes or changes vacuum.
"""
from dataclasses import dataclass,field
import hashlib
import math
import time

import numpy as np
from scipy.spatial.transform import Rotation

from luggage_perception.pickup_view_evidence import PickupViewEvidence
from luggage_planning.joint_operation.contracts import OperationIdentity
from luggage_planning.joint_operation.planner import timed_operation_leg
from luggage_planning.joint_operation.view_certification import ViewMotionCertificate
from luggage_planning.joint_operation.view_proposals import PickupCameraProposal
from luggage_planning.joint_operation.view_scene import fixed_tool_from_optical,pickup_view_scene
from luggage_planning.placement_prior.contracts import Transform,finite_tuple


@dataclass(frozen=True)
class PreparedPickupView:
    request_id: str
    domain_sha256: str
    view_sha256: str
    query_id: str
    base_snapshot_identity: tuple
    view_snapshot_identity: tuple
    world_from_optical: Transform
    world_from_tool: Transform
    tool_from_optical: Transform
    leg: object
    route_digest: str
    certificate_id: str
    # Preserve exactly the bytes bound by view_snapshot_identity; serializing an
    # equivalent ROS scene again need not reproduce its CDR alignment bytes.
    view_scene_cdr: bytes = field(repr=False)


@dataclass(frozen=True)
class PickupViewMotionResult:
    reason: str
    plan: object = None
    trace: tuple = ()


class PickupViewMotionPreparation:
    def __init__(self,model,ik,connections,certifier,*,velocity_scale,acceleration_scale):
        if (not all(math.isfinite(v) and 0<v<=1 for v in (velocity_scale,acceleration_scale))):
            raise ValueError('EXPLICIT_VIEW_MOTION_SCALES_REQUIRED')
        self.model,self.ik,self.connections,self.certifier = model,ik,connections,certifier
        self.velocity_scale,self.acceleration_scale = velocity_scale,acceleration_scale

    def prepare(self,snapshot,domain,view,proposal,start_joints,*,request_id,robot_state_stamp_ns,
                pickup_object_id,ik_seeds,max_ik_seeds,admit_static_scene,source_current,
                deadline,cancelled=lambda:False):
        trace=[]
        def stopped():return cancelled() or time.monotonic()>=deadline
        def refused(reason):return PickupViewMotionResult(reason,trace=tuple(trace))
        try:
            if (not request_id.strip() or type(robot_state_stamp_ns) is not int or robot_state_stamp_ns<0
                    or not math.isfinite(deadline) or not callable(source_current)
                    or type(max_ik_seeds) is not int or not 1<=max_ik_seeds<=64):
                return refused('VIEW_MOTION_INPUT_INVALID')
            start=finite_tuple(start_joints,6,'view start joints')
            seeds=tuple(finite_tuple(q,6,'view IK seed') for q in ik_seeds)
            if not seeds or len(seeds)>max_ik_seeds:return refused('VIEW_IK_SEED_BUDGET_INVALID')
            if (type(view) is not PickupViewEvidence or type(proposal) is not PickupCameraProposal
                    or (view.domain_sha256,view.box_id,proposal.domain_sha256,proposal.view_sha256) !=
                       (domain.digest(),domain.box_id,domain.digest(),view.digest())
                    or (view.acquisition.stamp_ns,view.acquisition.instance_id,view.acquisition.generation) !=
                       (domain.stamp_ns,domain.instance_id,domain.generation)
                    or snapshot.model_hash!=self.model.model_sha256):
                return refused('VIEW_MOTION_SOURCE_MISMATCH')
            if stopped():return refused('CANCELLED_OR_DEADLINE')
            if source_current(snapshot) is not True:return refused('VIEW_SOURCE_NOT_CURRENT')
            mount=fixed_tool_from_optical(self.model,view.acquisition.optical_frame_id,start)
            target=proposal.world_from_optical.compose(mount.inverse())
            scene=pickup_view_scene(snapshot,domain,pickup_object_id,admit_static_scene=admit_static_scene)
            identity=OperationIdentity(request_id,snapshot.scene_revision,
                hashlib.sha256(repr(scene.identity).encode()).hexdigest(),snapshot.geometry_hash,
                snapshot.model_hash,snapshot.map_revision,view.digest(),domain.stamp_ns,
                robot_state_stamp_ns,domain.box_id,domain.generation,domain.contact_model_hash)
            for index,seed in enumerate(seeds):
                if stopped():return refused('CANCELLED_OR_DEADLINE')
                query_id=request_id+':ik-'+str(index)
                endpoint=self.ik.refine(scene,target,seed,request_id=query_id,deadline=deadline,cancelled=cancelled)
                trace.append(('ik',index,endpoint.reason))
                if not endpoint.complete or not endpoint.success:continue
                if not self.model.matches(endpoint.joints,target):return refused('VIEW_ENDPOINT_FK_MISMATCH')
                actual_camera=self.model.urdf.tcp(endpoint.joints,
                    self.model.urdf.path_to(view.acquisition.optical_frame_id))
                camera_target=proposal.world_from_optical
                angle=Rotation.from_matrix(actual_camera[:3,:3].T @
                    Rotation.from_quat(camera_target.quaternion_xyzw).as_matrix()).magnitude()
                if (np.linalg.norm(actual_camera[:3,3]-camera_target.translation)>self.model.position_tolerance_m
                        or angle>self.model.orientation_tolerance_rad):
                    return refused('VIEW_CAMERA_ENDPOINT_FK_MISMATCH')
                connection=self.connections.connect(scene,start,endpoint.joints,
                    request_id=request_id+':connection-'+str(index),deadline=deadline,cancelled=cancelled)
                trace.append(('connection',index,connection.reason))
                if not connection.complete or not connection.success:continue
                rows=connection.positions
                if (len(rows)<2 or max(abs(a-b) for a,b in zip(rows[0],start))>1e-8
                        or max(abs(a-b) for a,b in zip(rows[-1],endpoint.joints))>1e-8):
                    return refused('VIEW_CONNECTION_ENDPOINT_MISMATCH')
                leg=timed_operation_leg('align','pick',rows,identity,self.velocity_scale,self.acceleration_scale,
                    required_positions=connection.required_positions)
                digest=hashlib.sha256(repr((snapshot.identity,scene.identity,domain.digest(),view.digest(),
                    proposal.query_id,proposal.world_from_optical,target,mount,leg)).encode()).hexdigest()
                certificate=self.certifier.certify(scene,leg,digest,deadline=deadline,cancelled=cancelled)
                trace.append(('certification',index,certificate.reason))
                if (type(certificate) is not ViewMotionCertificate or certificate.accepted is not True
                        or not certificate.certificate_id):continue
                if stopped():return refused('CANCELLED_OR_DEADLINE')
                if source_current(snapshot) is not True:return refused('VIEW_SOURCE_CHANGED')
                plan=PreparedPickupView(request_id,domain.digest(),view.digest(),proposal.query_id,
                    snapshot.identity,scene.identity,proposal.world_from_optical,target,mount,leg,digest,
                    certificate.certificate_id,scene.scene_cdr)
                return PickupViewMotionResult('VIEW_PATH_PREPARED',plan,tuple(trace))
            return refused('VIEW_SEARCH_INCOMPLETE')
        except (TypeError,ValueError,RuntimeError) as error:
            return refused('INVALID_VIEW_MOTION_INPUT:'+str(error))
