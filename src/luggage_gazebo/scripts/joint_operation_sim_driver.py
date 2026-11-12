#!/usr/bin/env python3
"""Explicitly authorized, observed full-operation Gazebo diagnostic client.

Spawn responses are physics-only and never supply detection/model/map inputs.
Planning consumes exclusively the detector and O owner. Every RPC/action boundary
is serialized with its ROS type so a refusal remains inspectable.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState, Image, CameraInfo, PointCloud2
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from luggage_msgs.action import ExecuteJointOperationV2
from luggage_msgs.msg import SuctionTopDomainV2, DetectionFrame
from luggage_msgs.srv import DetectLuggage, PrepareJointOperationV2, SpawnNextBox, FinalizeCurrentBox
from uld_occ_observer.srv import GetValidity, ResetObservation, GetSnapshot, ReleaseSnapshot, InitializeEmpty
from uld_occ_observer.msg import UpdateReceipt
from luggage_mapping.observer import admit_response, identity_record, ObserverConfiguration, ObserverSnapshotPort
from luggage_planning.joint_operation.runtime_ports import OperationRpc
from luggage_planning.placement_prior.snapshot import build_private_snapshot, COLUMN_PREFIX
from luggage_planning.placement_prior.contracts import Transform
from controller_manager_msgs.srv import ListControllers
import yaml
from luggage_gazebo.operation_readiness import wait_active_controllers, wait_observed_detection, wait_static_scene
from luggage_gazebo.observer_failure_capture import ObserverFailureCapture
from luggage_description.scene_tf_config_utils import load_scene_tf_config, pedestal_enabled, world_transform
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from luggage_description.scene_mesh_utils import container_model_name


JOINTS = ['elfin_joint%d'%i for i in range(1,7)]


from luggage_gazebo.operation_cycle import can_reprepare_before_pick,run_observed_cycle


def load_diagnostic_views(path, launch):
    """Explicit offline-generated observation goals, bound to this exact cell.

    These are MoveGroup goals, never trajectory/collision approval or map data.
    Each executed view still needs actual settled sensor evidence.
    """
    if not path:return {}
    record=json.loads(Path(path).read_text())
    if (record.get('schema')!='diagnostic_observation_poses_v1'
            or record.get('model_sha256')!=launch['model_sha256']
            or record.get('scene_revision')!=launch['scene_revision']):
        raise ValueError('OBSERVATION_VIEW_IDENTITY_MISMATCH')
    poses=record.get('poses')
    if not isinstance(poses,dict) or not poses:
        raise ValueError('OBSERVATION_VIEWS_REQUIRED')
    for name,pose in poses.items():
        values=pose.get('values',[])
        if (not name or pose.get('valid') is not True or len(values)!=6
                or any(type(value) not in (float,int) or not math.isfinite(value) for value in values)):
            raise ValueError('OBSERVATION_VIEW_JOINTS_INVALID')
    return poses


class OperationSimDriver(Node):
    def __init__(self, args):
        super().__init__('joint_operation_sim_driver', parameter_overrides=[])
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.args = args
        self.folder = Path(args.evidence)
        self.folder.mkdir(parents=True, exist_ok=False)
        self.launch = json.loads(Path(args.run_dir,'launch.json').read_text())
        self.extra_views=load_diagnostic_views(args.view_poses,self.launch)
        self.records, self.stages, self.joint, self.domain = [], [], None, None
        self._record_lock=threading.RLock()
        self.images = {}
        self.sensor_diagnostics = {}
        self.clock_count = 0
        self.failure_capture = None
        self._static_tf_blocks={}
        self._capture_number=1
        self.completed_boxes=[]
        if getattr(args, 'failure_hold_s', 0.) > 0:
            self.failure_capture = ObserverFailureCapture(self.folder/'observer_failure',
                disk_limit=(768 if getattr(args,'capture_operation',False) else 256)*1024*1024)
            for topic,cls in [('/joint_states',JointState),('/tf',TFMessage),
                    ('/tf_static',TFMessage),('/uld_occ_observer/receipts',UpdateReceipt),
                    ('/luggage/observation/optical_points',PointCloud2),
                    ('/luggage/observation/lidar_points',PointCloud2),
                    ('/luggage/observation/optical_points_unmasked',PointCloud2),
                    ('/luggage/observation/lidar_points_unmasked',PointCloud2),
                    ('/luggage/observation/self_mask_status',String)]:
                qos = (QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
                       if topic=='/tf_static' else qos_profile_sensor_data)
                self.create_subscription(cls,topic,lambda msg,t=topic,c=cls:self.capture_sensor(t,c,msg),qos)
        self.create_subscription(JointState,'/joint_states',lambda msg:setattr(self,'joint',msg),qos_profile_sensor_data)
        self.create_subscription(SuctionTopDomainV2,'/luggage/suction_top_domain_v2',
            lambda msg:setattr(self,'domain',msg),qos_profile_sensor_data)
        self.create_subscription(Clock,'/clock',lambda msg:setattr(self,'clock_count',self.clock_count+1),qos_profile_sensor_data)
        for key, cls, topic in [('rgb',Image,'/luggage/preprocessed/camera/color/image'),
                ('depth',Image,'/luggage/preprocessed/camera/depth/image'),
                ('info',CameraInfo,'/luggage/preprocessed/camera/depth/camera_info')]:
            self.create_subscription(cls,topic,lambda msg,k=key:self.images.__setitem__(k,msg),qos_profile_sensor_data)
        for key,cls,topic in [
                ('depth_status',String,'/luggage/observation/status'),
                ('lidar_status',String,'/luggage/observation/lidar_status'),
                ('self_mask_status',String,'/luggage/observation/self_mask_status'),
                ('o_receipt',UpdateReceipt,'/uld_occ_observer/receipts'),
                ('lidar',PointCloud2,'/livox/lidar'),
                ('optical_rays',PointCloud2,'/luggage/observation/optical_points')]:
            self.create_subscription(cls,topic,
                lambda msg,k=key:self.sensor_diagnostics.__setitem__(k,msg),qos_profile_sensor_data)
        self.create_subscription(DetectionFrame,'/luggage/perception/detection_frame',
            lambda msg:self.sensor_diagnostics.__setitem__('detection_frame',msg),qos_profile_sensor_data)
        self.create_subscription(String,'/luggage/simulation_release',
            lambda msg:self.record('physics_release_report',msg),QoSProfile(
                depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.prepare = self.create_client(PrepareJointOperationV2,'/joint_operation/prepare')
        self.detect = self.create_client(DetectLuggage,'/luggage_detector/detect_luggage')
        self.validity = self.create_client(GetValidity,'/uld_occ_observer/get_validity')
        self.reset = self.create_client(ResetObservation,'/uld_occ_observer/reset_observation')
        self.initialize = self.create_client(InitializeEmpty,'/uld_occ_observer/initialize_empty')
        self.snapshot = self.create_client(GetSnapshot,'/uld_occ_observer/get_snapshot')
        self.release_snapshot = self.create_client(ReleaseSnapshot,'/uld_occ_observer/release_snapshot')
        self.spawn = self.create_client(SpawnNextBox,'/pickup_box_spawner/spawn_next_box')
        self.finalize=self.create_client(FinalizeCurrentBox,'/pickup_box_spawner/finalize_current_box')
        self.execute = ActionClient(self,ExecuteJointOperationV2,'/joint_operation/execute')
        self.move = ActionClient(self,MoveGroup,'/move_action')
        self.controllers = self.create_client(ListControllers,'/controller_manager/list_controllers')
        self.scene = self.create_client(GetPlanningScene,'/get_planning_scene')
        self.apply_scene = self.create_client(ApplyPlanningScene,'/apply_planning_scene')
        owner=self.launch['map_owner']
        self.observer_rpc=OperationRpc(self.record)
        self.observer=ObserverSnapshotPort(self,configuration=ObserverConfiguration.load(
            owner['geometry_file'],owner['config_file']),call=self.observer_rpc.call,recorder=self.record)

    def capture_sensor(self,topic,cls,message):
        raw=serialize_message(message);kind=cls.__module__+'.'+cls.__name__
        key=tuple(sorted(v.child_frame_id for v in message.transforms)) if topic=='/tf_static' else False
        if key:self._static_tf_blocks[key]=(topic,kind,raw)
        self.failure_capture.add(topic,kind,raw,persistent=key)

    def rotate_capture(self):
        if self.failure_capture is None:return
        self.failure_capture.close()
        if self.failure_capture.dropped:raise RuntimeError('OBSERVER_WINDOW_DROPPED_RECORDS')
        self._capture_number+=1
        self.failure_capture=ObserverFailureCapture(self.folder/('observer_box'+str(self._capture_number)))
        for key,(topic,kind,raw) in tuple(self._static_tf_blocks.items()):
            self.failure_capture.add(topic,kind,raw,persistent=key)

    def record(self, name, message):
        with self._record_lock:self._record_locked(name,message)

    def _record_locked(self, name, message):
        raw = message if isinstance(message,bytes) else serialize_message(message)
        if sum(row['bytes'] for row in self.records)+len(raw)>256*1024*1024:
            raise RuntimeError('DRIVER_CAPTURE_LIMIT')
        file = '%04d_%s.cdr'%(len(self.records),name)
        if name.startswith('physics_'):
            (self.folder/'reference').mkdir(exist_ok=True)
            file = 'reference/'+file
        (self.folder/file).write_bytes(raw)
        self.records.append(dict(file=file,ros_type='raw_bytes' if isinstance(message,bytes) else type(message).__module__+'.'+type(message).__name__,
            bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest(),wall_monotonic=time.monotonic()))
        self.flush()

    def flush(self, **result):
        with self._record_lock:
            # A late lease-release callback may append evidence after the
            # terminal result. Keep that result when refreshing the index.
            self._final_status={**getattr(self,'_final_status',{}),**result}
            (self.folder/'manifest.json').write_text(json.dumps(dict(records=self.records,
                stages=self.stages,completed_boxes=self.completed_boxes, **self._final_status),indent=2)+'\n')

    def wait(self, predicate, timeout, reason):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline and rclpy.ok():
            if predicate():return
            time.sleep(.05)
        raise TimeoutError(reason)

    def future(self, value, timeout, reason):
        done = threading.Event()
        value.add_done_callback(lambda _:done.set())
        if not done.wait(timeout):raise TimeoutError(reason)
        return value.result()

    def call(self, client, request, name, timeout=10.):
        if not client.wait_for_service(timeout_sec=10.):raise TimeoutError(name+'_UNAVAILABLE')
        self.record(name+'_request',request)
        result = self.future(client.call_async(request),timeout,name+'_TIMEOUT')
        self.record(name+'_response',result)
        return result

    def action(self, client, goal, name, timeout):
        if not client.wait_for_server(timeout_sec=10.):raise TimeoutError(name+'_UNAVAILABLE')
        self.record(name+'_goal',goal)
        options={}
        if name.startswith('execute'):
            def feedback(message):
                stage=message.feedback.stage
                if getattr(self.args,'capture_operation',False) or (getattr(self.args,'three_box',False)
                        and stage.startswith('post_place_explore:')):
                    self.failure_capture.trigger()
            options['feedback_callback']=feedback
        handle = self.future(client.send_goal_async(goal,**options),10.,name+'_ADMISSION_TIMEOUT')
        if not handle.accepted:raise RuntimeError(name+'_REJECTED')
        try:
            wrapped = self.future(handle.get_result_async(),timeout,name+'_RESULT_TIMEOUT')
        except TimeoutError:
            self.future(handle.cancel_goal_async(),5.,name+'_CANCEL_TIMEOUT')
            raise
        self.record(name+'_result',wrapped)
        return wrapped

    def execute_prepared(self,plan,detection,domain,name):
        budget=float(self.launch['resolved'].get('operation_execution_budget_s',600.))
        durations=[leg.trajectory.trajectory.points[-1].time_from_start for leg in plan.legs]
        minimum=sum(t.sec+t.nanosec/1e9 for t in durations)
        if not durations or not math.isfinite(budget) or budget<=minimum:
            raise ValueError('TRAJECTORY_TIME_EXCEEDS_OPERATION_BUDGET')
        self.record(name+'_budget',json.dumps(dict(trajectory_time_s=minimum,
            action_budget_s=budget,includes_certification_and_observation=True)).encode())
        return self.action(self.execute,ExecuteJointOperationV2.Goal(plan=plan,
            detection=detection,top_domain=domain,budget_s=budget),name,budget+10.)

    def capture_failure_state(self):
        """Latest failed boundary; preserve capture errors without masking cause."""
        failures=[]
        for name,read in (
                ('planning_scene',lambda:self.read_scene('failure_current_scene')),
                ('observer',lambda:self.observer.capture(
                    scene_revision=self.launch['scene_revision'],request_id='failed-boundary',
                    expected_epoch=self.call(self.validity,GetValidity.Request(),'failure_validity').epoch,
                    expected_revision=None,deadline=time.monotonic()+15.,require_latest=False))):
            try:read()
            except Exception as error:
                failures.append(name)
                self.record('failure_'+name+'_capture_error',json.dumps(dict(
                    error=type(error).__name__+':'+str(error))).encode())
        return failures

    def stage(self, name, function):
        began = time.monotonic()
        row = dict(stage=name,begin=began,outcome='running')
        self.stages.append(row);self.flush()
        try:
            value = function()
            row['outcome']='pass'
            return value
        except Exception as error:
            row.update(outcome='refused',reason=type(error).__name__+':'+str(error))
            raise
        finally:
            row['elapsed_s']=time.monotonic()-began
            self.flush()

    def readiness(self):
        self.wait(lambda:self.joint is not None and self.clock_count>1 and len(self.images)==3,
                  120.,'SENSOR_OR_JOINTS_NOT_READY')
        self.wait(lambda:self.count_publishers('/clock')==1,5.,'CLOCK_PUBLISHER_COUNT')
        for client in (self.prepare,self.detect,self.validity,self.spawn):
            if not client.wait_for_service(timeout_sec=10.):raise TimeoutError(client.srv_name)
        if not self.execute.wait_for_server(timeout_sec=10.):raise TimeoutError('EXECUTOR_NOT_READY')
        required = {'/placement_prior/check_local_paths','/joint_operation/plan_connection',
                    '/validate_route_v2','/luggage/evaluate_directed_suction_v2',
                    '/luggage/retain_suction_observation_v2','/vacuum/command_v2'}
        self.wait(lambda:required.issubset(dict(self.get_service_names_and_types())),10.,'NATIVE_OR_OBSERVATION_PORT_NOT_READY')
        wait_active_controllers(lambda:self.call(self.controllers,ListControllers.Request(),'ready_controllers'),
                                deadline=time.monotonic()+45.)
        if not set(JOINTS).issubset(self.joint.name):raise RuntimeError('SIX_JOINTS_MISSING')
        for name,msg in self.images.items():self.record('ready_'+name,msg)
        self.record('ready_joints',self.joint)
        config=load_scene_tf_config(self.launch['resolved']['scene_tf_config'])
        required={container_model_name(config)}
        if pedestal_enabled(config):required.add('robot_pedestal')
        wait_static_scene(lambda:self.read_scene('ready_scene'),required,deadline=time.monotonic()+45.)

    def read_scene(self,name):
        request=GetPlanningScene.Request()
        request.components.components=1023
        return self.call(self.scene,request,name)

    def observe_pose(self, name, *, require_uld_observation=True):
        poses = yaml.safe_load(Path(self.launch['resolved']['robot_poses_config']).read_text())['poses']
        poses.update(self.extra_views)
        target = poses[name]
        if not target['valid']:raise ValueError('INVALID_SITE_POSE:'+name)
        goal = MoveGroup.Goal()
        goal.request.group_name='elfin_arm';goal.request.num_planning_attempts=3
        goal.request.allowed_planning_time=10.
        goal.request.max_velocity_scaling_factor=.2;goal.request.max_acceleration_scaling_factor=.1
        goal.request.start_state.is_diff=True
        goal.request.goal_constraints=[Constraints(joint_constraints=[
            JointConstraint(joint_name=joint,position=float(value),tolerance_above=.01,
                            tolerance_below=.01,weight=1.) for joint,value in zip(JOINTS,target['values'])])]
        goal.planning_options.plan_only=False
        self.read_scene('before_observe_'+name)
        result = self.action(self.move,goal,'observe_'+name,90.)
        self.read_scene('after_observe_'+name)
        if result.result.error_code.val!=1:raise RuntimeError('OBSERVE_MOVEIT:'+str(result.result.error_code.val))
        # Wait for the existing stationary-depth adapter and O receipt, no guessed free-space.
        time.sleep(1.)
        validity = self.call(self.validity,GetValidity.Request(),'view_validity')
        # Pickup is outside ULD. Its view must provide measured detection, not
        # an unrelated ULD hit; initial empty authorization comes from O.
        if require_uld_observation and validity.last_integrated_stamp_ns<=0:
            raise RuntimeError('NO_INTEGRATED_SENSOR_VIEW')
        # Preserve this settled stage even if later motion ages it out of the
        # bounded failure sensor ring. Model bytes + joints retain camera TF.
        for key,msg in self.images.items():self.record('view_'+name+'_'+key,msg)
        self.record('view_'+name+'_joints',self.joint)

    def return_to_pick_observation(self, plan, *, pickup_present=True):
        """Existing MoveGroup view motion with actual O in its live scene.

        Only this diagnostic's generated O objects are applied/removed. Keep
        them on unresolved action/transport failure; never restore a whole scene.
        """
        validity=self.call(self.validity,GetValidity.Request(),'return_view_validity')
        captured=self.observer.capture(scene_revision=self.launch['scene_revision'],
            request_id='return-pick-view',expected_epoch=validity.epoch,
            expected_revision=None,deadline=time.monotonic()+15.,require_latest=False)
        baseline=self.read_scene('return_view_baseline').scene
        if any(obj.id.startswith(COLUMN_PREFIX) for obj in baseline.world.collision_objects):
            raise RuntimeError('OBSERVATION_OCC_NAMESPACE_ALREADY_OWNED')
        xyz,rpy=world_transform(load_scene_tf_config(self.launch['resolved']['scene_tf_config']),'container_link')
        container=Transform(tuple(xyz),tuple(Rotation.from_euler('xyz',rpy).as_quat()))
        snapshot=build_private_snapshot(captured.surface,baseline,self.launch['model_sha256'],container,
            volume=captured.volume,**self.launch['collision_policy'])
        private=deserialize_message(snapshot.scene_cdr,PlanningScene)
        objects=[obj for obj in private.world.collision_objects if obj.id.startswith(COLUMN_PREFIX)]
        pickup=CollisionObject(id='diagnostic-return-pickup',operation=CollisionObject.ADD)
        pickup.header.frame_id='world'
        if any(obj.id==pickup.id for obj in baseline.world.collision_objects):
            raise RuntimeError('OBSERVATION_PICKUP_NAMESPACE_ALREADY_OWNED')
        pickup.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,
            dimensions=list(plan.predicted_attachment.measured_dimensions_wdh))]
        pickup.primitive_poses=[plan.world_from_pick_box]
        if pickup_present:objects.append(pickup)
        diff=PlanningScene(is_diff=True);diff.robot_state.is_diff=True
        diff.world.collision_objects=objects
        applied=self.call(self.apply_scene,ApplyPlanningScene.Request(scene=diff),'return_view_apply_o')
        if not applied.success:raise RuntimeError('OBSERVATION_OCC_APPLY_REFUSED')
        self.observe_pose(self.launch['resolved']['observe_pose_name'],require_uld_observation=False)
        # observe_pose only returns on an acknowledged successful terminal action.
        remove=PlanningScene(is_diff=True);remove.robot_state.is_diff=True
        remove.world.collision_objects=[CollisionObject(id=obj.id,operation=CollisionObject.REMOVE)
                                        for obj in objects]
        if not self.call(self.apply_scene,ApplyPlanningScene.Request(scene=remove),'return_view_remove_o').success:
            raise RuntimeError('OBSERVATION_OCC_CLEANUP_REFUSED')

    def plan(self, *, task_mode=None):
        detection,domain = wait_observed_detection(
            lambda remaining:self.call(self.detect,DetectLuggage.Request(),'detect',timeout=remaining),
            lambda:self.domain,deadline=time.monotonic()+30.)
        self.record('top_domain',domain)
        for key,message in self.images.items():self.record('detection_'+key,message)
        validity = self.call(self.validity,GetValidity.Request(),'prepare_validity')
        # GetValidity records the validity transition; its map_revision is not
        # the latest repository identity. Pin the actual current snapshot until
        # prepare has acquired its independent lease of those exact map bytes.
        lease = self.call(self.snapshot,GetSnapshot.Request(expected_epoch=validity.epoch,
            exact_revision=False,max_age_ms=1000,allow_stale=False),'prepare_snapshot')
        if not lease.handle:raise RuntimeError('MAP_SNAPSHOT:'+lease.status)
        try:
            admit_response(lease,identity_record(lease.identity),lease.validity.generation)
            slots=()
            if task_mode is not None:
                from luggage_gazebo.observed_task_slots import observed_task_slots
                from luggage_planning.attachment_geometry_policy import load_attachment_geometry_policy
                captured=self.observer.capture(scene_revision=self.launch['scene_revision'],
                    request_id='explicit-'+task_mode,expected_epoch=lease.identity.epoch,
                    expected_revision=lease.identity.map_revision,deadline=time.monotonic()+15.,require_latest=False)
                slots,report=observed_task_slots(captured,tuple(domain.box_wdh),task_mode,
                    limits=load_attachment_geometry_policy(self.launch['resolved'].get('attachment_geometry_config','')),
                    margin=self.launch['collision_policy']['collision_margin_m'],
                    vertical_margin=self.launch['collision_policy']['vertical_collision_margin_m'])
                self.record('explicit_task_candidates',json.dumps(report,sort_keys=True).encode())
                if not slots:raise RuntimeError('NO_OBSERVED_'+task_mode.upper()+'_PLACEMENT')
            return self.prepare_from_observation(detection,domain,lease.identity,slots=slots)
        finally:
            self.call(self.release_snapshot,ReleaseSnapshot.Request(handle=lease.handle),'release_prepare_snapshot')

    def prepare_from_observation(self,detection,domain,identity,*,slots=()):
        joint = self.joint;self.record('prepare_joints',joint)
        positions = dict(zip(joint.name,joint.position))
        request = PrepareJointOperationV2.Request(request_id='gazebo-'+str(time.time_ns()),
            observation_id=domain.domain_sha256,detection=detection,generation=domain.generation,
            geometry_hash=identity.geometry_hash,expected_map_revision=identity.map_revision,
            expected_map_epoch=identity.epoch,expected_model_sha256=self.launch['model_sha256'],
            robot_state_stamp_valid=True,
            robot_state_stamp_ns=joint.header.stamp.sec*10**9+joint.header.stamp.nanosec,
            start_joints=[positions[name] for name in JOINTS],candidate_limit=8,
            budget_s=float(self.launch['resolved']['operation_prepare_budget_s']))
        from luggage_planning.place_service import pose_message
        request.slots=[pose_message(dict(contract_version=2,kind='pose',frame_id='container_link',length_unit='m',
            stamp_ns=0,time_basis='scene',scene_revision=self.launch['scene_revision'],request_id=request.request_id,
            reference='box_center',orientation_semantics='box_axis_pi',position=slot.translation,
            quaternion_xyzw=slot.quaternion_xyzw)) for slot in slots]
        prepared = self.call(self.prepare,request,'prepare',timeout=request.budget_s+10.)
        trace=self.folder/'prepare_trace.json'
        if trace.exists():trace=self.folder/('prepare_trace_'+request.request_id+'.json')
        trace.write_text(prepared.trace_json+'\n')
        if not prepared.success:raise RuntimeError('PREPARE:'+prepared.reason)
        return prepared.plan,detection,domain

    def run(self):
        try:
            self.stage('readiness',self.readiness)
            if not self.args.start:
                self.flush(outcome='READY_ONLY',capture_complete=True,replay_possible=True)
                return 0
            # Include the first sensor writes after initial-empty declaration;
            # action-only capture cannot explain O changes made during prepare.
            if getattr(self.args,'capture_operation',False):
                self.failure_capture.trigger()
            # --start is an explicit operator authorization for this diagnostic.
            def initialize():
                from luggage_mapping.observer import ObserverConfiguration
                owner=self.launch['map_owner']
                config=ObserverConfiguration.load(owner['geometry_file'],owner['config_file'])
                current=self.call(self.validity,GetValidity.Request(),'initial_validity')
                request=InitializeEmpty.Request(expected_epoch=current.epoch,
                    geometry_hash=config.descriptor()['geometry_hash'],config_digest=config.config_digest,
                    declaration_id='initial-empty-'+self.folder.name,declared_by='explicit_operator_start')
                result=self.call(self.initialize,request,'initialize_empty')
                if result.status!='OK':raise RuntimeError('INITIALIZE_EMPTY:'+result.status)
            self.stage('initialize_empty',initialize)
            views=self.args.views.split(',') if self.args.views else list(self.extra_views)
            for pose in views:
                self.stage('observe_'+pose,lambda p=pose:self.observe_pose(p))
            self.stage('return_pick_observe',lambda:self.observe_pose(
                self.launch['resolved']['observe_pose_name'],require_uld_observation=False))
            modes=('floor','floor','stack') if getattr(self.args,'three_box',False) else (None,)
            epoch=None
            for index,mode in enumerate(modes):
                if index:
                    self.rotate_capture()
                    self.stage('return_pick_box'+str(index+1),lambda:self.return_to_pick_observation(
                        previous_plan,pickup_present=False))
                plan,outcome=run_observed_cycle(self,task_mode=mode,prefix=('box'+str(index+1)+'_') if mode else '')
                source=outcome.final_map_source
                if epoch is None:epoch=source.epoch
                if source.epoch!=epoch:raise RuntimeError('PERSISTENT_O_EPOCH_CHANGED')
                self.completed_boxes.append(dict(index=index+1,task_mode=mode,plan_digest=plan.plan_digest,
                    map_epoch=source.epoch,map_revision=source.map_revision,success=True))
                capture_errors=self.capture_failure_state() or []
                if capture_errors:raise RuntimeError("CYCLE_BOUNDARY_CAPTURE_INCOMPLETE:"+",".join(capture_errors))
                if mode is not None:
                    final=self.call(self.finalize,FinalizeCurrentBox.Request(),'physics_finalize')
                    if not final.success:raise RuntimeError('PHYSICS_FINALIZE_REFUSED')
                previous_plan=plan
            if getattr(self,'failure_capture',None) is not None:
                self.failure_capture.close()
                if self.failure_capture.dropped:raise RuntimeError('OBSERVER_WINDOW_DROPPED_RECORDS')
            self.flush(outcome='THREE_BOX_PASS' if getattr(self.args,'three_box',False) else 'FULL_CYCLE_PASS',capture_complete=True,replay_possible=True)
            return 0
        except Exception as error:
            if (getattr(self,'failure_capture',None) is not None
                    and (str(error).endswith(('CURRENT_SOURCE_REFUSED','BOUND_NATIVE_ADMISSION_REQUIRED'))
                         or getattr(self.args,'three_box',False)
                         or (getattr(self.args,'capture_operation',False) and 'EXECUTE:' in str(error)))):
                self.failure_capture.trigger()
                self.stage('failure_stationary_capture',lambda:time.sleep(self.args.failure_hold_s))
                self.failure_capture.close()
            elif getattr(self,'failure_capture',None) is not None:
                self.failure_capture.close()
            capture_errors=self.capture_failure_state() or []
            if getattr(getattr(self,'failure_capture',None),'dropped',0):
                capture_errors.append('observer_window_dropped_records')
            for name,msg in self.sensor_diagnostics.items():self.record('failure_'+name,msg)
            if self.domain is not None:self.record('failure_top_domain',self.domain)
            for name,msg in self.images.items():self.record('failure_'+name,msg)
            if self.joint is not None:self.record('failure_joints',self.joint)
            self.flush(outcome='INCOMPLETE',reason=type(error).__name__+':'+str(error),
                       capture_complete=not capture_errors,replay_possible=not capture_errors,
                       latest_boundary_capture_errors=capture_errors)
            print(str(error),flush=True)
            return 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start',action='store_true',help='Explicitly authorize one simulation cycle')
    parser.add_argument('--run-dir',required=True)
    parser.add_argument('--evidence',required=True)
    parser.add_argument('--views',help='Optional supplemental site-pose names; no default initial ULD observation motion')
    parser.add_argument('--view-poses',help='Exact model/scene-bound diagnostic observation goals JSON')
    parser.add_argument('--failure-hold-s',type=float,default=0.,
        help='Diagnostic stationary observer capture after a terminal source refusal (0..60 s)')
    parser.add_argument('--prepick-replans',type=int,default=0,
        help='Explicit bounded new complete plans after terminal pre-pick map/native refusal (0..2)')
    parser.add_argument('--three-box',action='store_true',help='Same observed O epoch: floor, floor, stack; requires failure capture')
    parser.add_argument('--capture-operation',action='store_true',
        help='Persist source clouds/TF before map initialization, bounded to768MiB; requires failure hold')
    args = parser.parse_args()
    if not 0 <= args.failure_hold_s <= 60:parser.error('failure hold must be within 0..60 seconds')
    if not 0 <= args.prepick_replans <= 2:parser.error('prepick replans must be within 0..2')
    if args.capture_operation and args.failure_hold_s<=0:parser.error('operation capture requires failure hold')
    if args.three_box and (args.failure_hold_s<=0 or args.capture_operation):
        parser.error("three-box requires failure hold and uses bounded per-box exploration capture")
    rclpy.init()
    node = OperationSimDriver(args)
    executor = MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    thread = threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        result = node.run()
    finally:
        executor.shutdown();thread.join(timeout=5.);node.destroy_node();rclpy.shutdown()
    raise SystemExit(result)


if __name__=='__main__':
    main()
