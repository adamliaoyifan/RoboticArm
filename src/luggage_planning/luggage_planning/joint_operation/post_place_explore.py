"""Bounded post-release camera views using existing native IK/path/check ports.

No new planner or map writer. Views rank missing measured top-column coverage;
only fresh sensor O can finish adoption. Controller transport and motion ownership
are the complete operation's existing ones; unresolved transport retains them.
"""
import hashlib
import json
import time
import numpy as np
from scipy.spatial.transform import Rotation

from luggage_description.scene_tf_config_utils import load_scene_tf_config
from luggage_planning.geometry_view_generator import generate_interior_downward_views
from luggage_planning.interior_view_scorer import CameraIntrinsics
from luggage_planning.joint_operation.certification import _wire_leg
from luggage_planning.joint_operation.connection import NativeSnapshotConnectionPort
from luggage_planning.joint_operation.contracts import OPERATION_JOINTS,JointKnot,OperationLeg
from luggage_planning.joint_operation.ik import NativeSnapshotIKPort
from luggage_planning.joint_operation.polyline_timing import time_parameterize_joint_polyline
from luggage_planning.joint_operation.release_coverage import projected_missing_columns,camera_aimed_at_missing
from luggage_planning.moveit_state_validity import MAX_CERT_SAMPLE_DT_NS
from luggage_planning.placement_prior.contracts import Transform


class PostPlaceExplorer:
    def __init__(self,node,ports,motion,*,max_views,refresh_attempts,settled,checkpoint,record):
        from sensor_msgs.msg import CameraInfo
        from rclpy.qos import qos_profile_sensor_data
        if not 1<=max_views<=3:raise ValueError('POST_PLACE_VIEW_BOUND_REQUIRED')
        self.node,self.ports,self.motion=node,ports,motion
        self.max_views,self.refresh_attempts=max_views,refresh_attempts
        self.settled,self.checkpoint,self.record=settled,checkpoint,record
        self.info=None
        node.create_subscription(CameraInfo,'/luggage/preprocessed/camera/depth/camera_info',
            lambda msg:setattr(self,'info',msg),qos_profile_sensor_data)
        self.config=load_scene_tf_config(node.get_parameter('scene_tf_config').value)
        self.ik=NativeSnapshotIKPort(node,ports.model,recorder=lambda name,msg:record('explore_ik_'+name,msg),max_query_s=.5)
        self.connection=NativeSnapshotConnectionPort(node,
            recorder=lambda name,msg:record('explore_connection_'+name,msg),max_edge_s=5.)
        self.velocity_scale=float(node.get_parameter('velocity_scale').value)
        self.acceleration_scale=float(node.get_parameter('acceleration_scale').value)

    def camera_geometry(self):
        info=self.info
        if info is None or info.header.frame_id!='camera_depth_optical_frame':
            raise ValueError('POST_PLACE_CALIBRATED_CAMERA_REQUIRED')
        intrinsics=CameraIntrinsics(info.width,info.height,info.k[0],info.k[4],info.k[2],info.k[5])
        model=self.ports.model;urdf=model.urdf;q=(0.,)*6
        camera_path=urdf.path_to(info.header.frame_id)
        moving=tuple(name for name in camera_path if urdf.joints[name]['type']!='fixed')
        if moving!=OPERATION_JOINTS:raise ValueError('RIGID_TOOL_CAMERA_REQUIRED')
        tool_from_camera=Transform.from_matrix(np.linalg.inv(urdf.tcp(q,model.path))@urdf.tcp(q,camera_path))
        world_from_base=Transform.from_matrix(urdf.tcp(q,urdf.path_to('elfin_base_link')))
        self.record('explore_camera_info',info)
        return intrinsics,tool_from_camera,world_from_base

    def certify(self,leg):
        from moveit_msgs.msg import PlanningScene
        from rclpy.serialization import deserialize_message,serialize_message
        ports=self.ports;snapshot=ports.snapshot
        scene=deserialize_message(snapshot.scene_cdr,PlanningScene)
        if not ports.released or scene.robot_state.attached_collision_objects:
            raise ValueError('POST_PLACE_EXPLORATION_REQUIRES_EMPTY_TOOL')
        request=ports.native.service.Request(request_id=leg.trajectory_request_id,
            scene_revision=ports.revision,
            route_digest=hashlib.sha256(json.dumps(leg.trajectory_record(),sort_keys=True).encode()).hexdigest(),
            scene_digest=hashlib.sha256(serialize_message(scene)).hexdigest(),load_state_scenes=[scene],
            max_sample_dt_ns=float(MAX_CERT_SAMPLE_DT_NS),max_sample_step_m=.01,
            joint_velocity_limits=list(ports.native.velocity_limits),
            joint_acceleration_limits=list(ports.native.acceleration_limits),distance_stride=2**30,
            budget_s=min(30.,max(0.,ports.deadline-time.monotonic())))
        wire=_wire_leg(leg);wire.load_state_index=0;wire.carrying=False
        if wire.n_samples>ports.native.max_samples:raise ValueError('EXPLORATION_SAMPLE_CAP')
        request.legs=[wire]
        response=ports.rpc.call(ports.native.client,request,ports.deadline,'explore_native')
        return request,response

    def move(self,leg):
        ports=self.ports
        for attempt in range(self.refresh_attempts):
            outcome=self.motion.execute_observation(leg,model_sha256=ports.model.model_sha256,
                validate_current=self.certify,
                source_current=lambda:ports.source_current(ports.snapshot,ports.geometry),
                read_joints=ports.joints,deadline=ports.deadline,cancelled=ports.cancelled)
            self.record('explore_motion_outcome.json',json.dumps(dict(attempt=attempt,
                success=outcome.success,terminal=outcome.terminal,submitted=outcome.submitted,
                reason=outcome.reason)).encode())
            if not outcome.terminal:raise ValueError('EXPLORATION_MOTION_UNRESOLVED:'+outcome.reason)
            if (not outcome.submitted and ports.retryable_change and attempt+1<self.refresh_attempts
                    and not ports.cancelled() and time.monotonic()<ports.deadline):
                ports.capture();continue
            return outcome.success
        return False

    def observe(self,initial,settled_stamp):
        ports=self.ports
        if not ports.released:raise ValueError('POST_PLACE_EXPLORATION_BEFORE_RELEASE')
        intrinsics,tool_from_camera,world_from_base=self.camera_geometry()
        views=[view for view in generate_interior_downward_views(self.config) if view['valid_geometry']]
        tried=set();executed=0;captured=initial
        while executed<self.max_views and not ports.cancelled() and time.monotonic()<ports.deadline:
            accepted,_=ports.observed_target(captured)
            coverage=ports.last_release_coverage
            self.record('explore_coverage.json',json.dumps(coverage).encode())
            if accepted:return captured
            current_tool=Transform.from_matrix(ports.model.urdf.tcp(ports.joints().positions,ports.model.path))
            current_camera=current_tool.compose(tool_from_camera)
            ranked=[]
            for view in views:
                if view['name'] in tried:continue
                camera=world_from_base.compose(Transform(tuple(view['camera_xyz']),tuple(view['orientation_quat'])))
                camera=camera_aimed_at_missing(camera.translation,coverage['missing_points_container'],ports.container)
                count=projected_missing_columns(coverage['missing_points_container'],ports.container,camera,intrinsics)
                distance=float(np.linalg.norm(np.asarray(camera.translation)-current_camera.translation))
                ranked.append((-count,distance,view['name'],camera))
            self.record('explore_view_ranking.json',json.dumps([dict(name=row[2],
                projected_missing_columns=-row[0],camera_distance_m=row[1],world_from_camera=row[3].__dict__)
                for row in sorted(ranked,key=lambda row:row[:3])]).encode())
            if not ranked:break
            count,distance,name,camera=min(ranked,key=lambda row:row[:3]);tried.add(name)
            if count==0:break
            ports.capture();snapshot=ports.snapshot;start=ports.joints().positions
            target=camera.compose(tool_from_camera.inverse());request_id=ports.plan.identity.request_id+':explore:'+name
            seeds=(start,ports.plan.legs[3].knots[-1].positions)
            solution=None
            for seed in seeds:
                result=self.ik.refine(snapshot,target,seed,request_id=request_id,
                    deadline=ports.deadline,cancelled=ports.cancelled)
                if result.success:solution=result.joints;break
            if solution is None:continue
            path=self.connection.connect(snapshot,start,solution,request_id=request_id,
                deadline=ports.deadline,cancelled=ports.cancelled)
            if not path.success:continue
            rows,timing=time_parameterize_joint_polyline(path.positions,self.velocity_scale,self.acceleration_scale)
            if timing.duration>=ports.deadline-time.monotonic():continue
            knots=tuple(JointKnot(tuple(q),tuple(v),tuple(a),(),round(t*1e9))
                for q,v,a,t in zip(rows,timing.velocities_rad,timing.accelerations_rad,timing.times_s))
            leg=OperationLeg('post_place_explore','retreat',request_id,ports.revision,
                ports.joints().stamp_ns,OPERATION_JOINTS,knots)
            self.checkpoint('post_place_explore:'+name)
            if not self.move(leg):continue
            executed+=1;settled_stamp=self.settled()
            # Bounded integrations, not synthetic free cells or guessed waits.
            # Every capture contains several actual sensor updates in this cell.
            for capture_index in range(3):
                captured=ports.capture()
                if (captured.source.last_integrated_stamp_ns>=settled_stamp
                        and captured.source.last_integrated_stamp_ns>ports.release_stamp_ns):
                    accepted,_=ports.observed_target(captured)
                    self.record('explore_view_observation.json',json.dumps(dict(view=name,
                        capture_index=capture_index,source=captured.source.__dict__,
                        coverage=ports.last_release_coverage)).encode())
                    if accepted:return captured
        raise ValueError('BOUNDED_POST_PLACE_VIEWS_EXHAUSTED')
