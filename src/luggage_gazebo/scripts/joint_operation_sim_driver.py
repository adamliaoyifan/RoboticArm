#!/usr/bin/env python3
"""Explicitly authorized, observed full-operation Gazebo diagnostic client.

Spawn responses are physics-only and never supply detection/model/map inputs.
Planning consumes exclusively the detector and O owner. Every RPC/action boundary
is serialized with its ROS type so a refusal remains inspectable.
"""
import argparse
import hashlib
import json
from pathlib import Path
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState, Image, CameraInfo
from rosgraph_msgs.msg import Clock
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetPlanningScene
from luggage_msgs.action import ExecuteJointOperationV2
from luggage_msgs.msg import SuctionTopDomainV2
from luggage_msgs.srv import DetectLuggage, PrepareJointOperationV2, SpawnNextBox
from uld_occ_observer.srv import GetValidity, ResetObservation, GetSnapshot, ReleaseSnapshot
from luggage_mapping.observer import admit_response, identity_record
from controller_manager_msgs.srv import ListControllers
import yaml
from luggage_gazebo.operation_readiness import wait_active_controllers, wait_observed_detection


JOINTS = ['elfin_joint%d'%i for i in range(1,7)]


class OperationSimDriver(Node):
    def __init__(self, args):
        super().__init__('joint_operation_sim_driver', parameter_overrides=[])
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.args = args
        self.folder = Path(args.evidence)
        self.folder.mkdir(parents=True, exist_ok=False)
        self.launch = json.loads(Path(args.run_dir,'launch.json').read_text())
        self.records, self.stages, self.joint, self.domain = [], [], None, None
        self.images = {}
        self.clock_count = 0
        self.create_subscription(JointState,'/joint_states',lambda msg:setattr(self,'joint',msg),qos_profile_sensor_data)
        self.create_subscription(SuctionTopDomainV2,'/luggage/suction_top_domain_v2',
            lambda msg:setattr(self,'domain',msg),qos_profile_sensor_data)
        self.create_subscription(Clock,'/clock',lambda msg:setattr(self,'clock_count',self.clock_count+1),qos_profile_sensor_data)
        for key, cls, topic in [('rgb',Image,'/luggage/preprocessed/camera/color/image'),
                ('depth',Image,'/luggage/preprocessed/camera/depth/image'),
                ('info',CameraInfo,'/luggage/preprocessed/camera/depth/camera_info')]:
            self.create_subscription(cls,topic,lambda msg,k=key:self.images.__setitem__(k,msg),qos_profile_sensor_data)
        self.prepare = self.create_client(PrepareJointOperationV2,'/joint_operation/prepare')
        self.detect = self.create_client(DetectLuggage,'/luggage_detector/detect_luggage')
        self.validity = self.create_client(GetValidity,'/uld_occ_observer/get_validity')
        self.reset = self.create_client(ResetObservation,'/uld_occ_observer/reset_observation')
        self.snapshot = self.create_client(GetSnapshot,'/uld_occ_observer/get_snapshot')
        self.release_snapshot = self.create_client(ReleaseSnapshot,'/uld_occ_observer/release_snapshot')
        self.spawn = self.create_client(SpawnNextBox,'/pickup_box_spawner/spawn_next_box')
        self.execute = ActionClient(self,ExecuteJointOperationV2,'/joint_operation/execute')
        self.move = ActionClient(self,MoveGroup,'/move_action')
        self.controllers = self.create_client(ListControllers,'/controller_manager/list_controllers')
        self.scene = self.create_client(GetPlanningScene,'/get_planning_scene')

    def record(self, name, message):
        raw = serialize_message(message)
        if sum(row['bytes'] for row in self.records)+len(raw)>256*1024*1024:
            raise RuntimeError('DRIVER_CAPTURE_LIMIT')
        file = '%04d_%s.cdr'%(len(self.records),name)
        if name.startswith('physics_'):
            (self.folder/'reference').mkdir(exist_ok=True)
            file = 'reference/'+file
        (self.folder/file).write_bytes(raw)
        self.records.append(dict(file=file,ros_type=type(message).__module__+'.'+type(message).__name__,
            bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest(),wall_monotonic=time.monotonic()))
        self.flush()

    def flush(self, **result):
        (self.folder/'manifest.json').write_text(json.dumps(dict(records=self.records,
            stages=self.stages, **result),indent=2)+'\n')

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
        handle = self.future(client.send_goal_async(goal),10.,name+'_ADMISSION_TIMEOUT')
        if not handle.accepted:raise RuntimeError(name+'_REJECTED')
        try:
            wrapped = self.future(handle.get_result_async(),timeout,name+'_RESULT_TIMEOUT')
        except TimeoutError:
            self.future(handle.cancel_goal_async(),5.,name+'_CANCEL_TIMEOUT')
            raise
        self.record(name+'_result',wrapped)
        return wrapped

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
        scene_request = GetPlanningScene.Request()
        scene_request.components.components = 1023
        self.call(self.scene,scene_request,'ready_scene')

    def observe_pose(self, name):
        poses = yaml.safe_load(Path(self.launch['resolved']['robot_poses_config']).read_text())['poses']
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
        result = self.action(self.move,goal,'observe_'+name,90.)
        if result.result.error_code.val!=1:raise RuntimeError('OBSERVE_MOVEIT:'+str(result.result.error_code.val))
        # Wait for the existing stationary-depth adapter and O receipt, no guessed free-space.
        time.sleep(1.)
        validity = self.call(self.validity,GetValidity.Request(),'view_validity')
        if validity.last_integrated_stamp_ns<=0:raise RuntimeError('NO_INTEGRATED_SENSOR_VIEW')
        # Preserve this settled stage even if later motion ages it out of the
        # bounded failure sensor ring. Model bytes + joints retain camera TF.
        for key,msg in self.images.items():self.record('view_'+name+'_'+key,msg)
        self.record('view_'+name+'_joints',self.joint)

    def plan(self):
        detection,domain = wait_observed_detection(
            lambda remaining:self.call(self.detect,DetectLuggage.Request(),'detect',timeout=remaining),
            lambda:self.domain,deadline=time.monotonic()+30.)
        self.record('top_domain',domain)
        validity = self.call(self.validity,GetValidity.Request(),'prepare_validity')
        # GetValidity records the validity transition; its map_revision is not
        # the latest repository identity. Pin the actual current snapshot until
        # prepare has acquired its independent lease of those exact map bytes.
        lease = self.call(self.snapshot,GetSnapshot.Request(expected_epoch=validity.epoch,
            exact_revision=False,max_age_ms=1000,allow_stale=False),'prepare_snapshot')
        if not lease.handle:raise RuntimeError('MAP_SNAPSHOT:'+lease.status)
        try:
            admit_response(lease,identity_record(lease.identity),lease.validity.generation)
            return self.prepare_from_observation(detection,domain,lease.identity)
        finally:
            self.call(self.release_snapshot,ReleaseSnapshot.Request(handle=lease.handle),'release_prepare_snapshot')

    def prepare_from_observation(self,detection,domain,identity):
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
        prepared = self.call(self.prepare,request,'prepare',timeout=request.budget_s+10.)
        (self.folder/'prepare_trace.json').write_text(prepared.trace_json+'\n')
        if not prepared.success:raise RuntimeError('PREPARE:'+prepared.reason)
        return prepared.plan,detection,domain

    def run(self):
        try:
            self.stage('readiness',self.readiness)
            if not self.args.start:
                self.flush(outcome='READY_ONLY',capture_complete=True,replay_possible=True)
                return 0
            # --start is an explicit operator authorization for this diagnostic.
            def reset():
                current = self.call(self.validity,GetValidity.Request(),'reset_validity')
                result = self.call(self.reset,ResetObservation.Request(expected_epoch=current.epoch,
                    reset_id='gazebo-start-'+str(time.time_ns()),reason='explicit_simulation_start'),'reset')
                if result.status!='OK':raise RuntimeError('RESET:'+result.status)
            self.stage('reset',reset)
            for pose in self.args.views.split(','):
                self.stage('observe_'+pose,lambda p=pose:self.observe_pose(p))
            self.stage('return_pick_observe',lambda:self.observe_pose(self.launch['resolved']['observe_pose_name']))
            spawned = self.stage('physics_spawn',lambda:self.call(self.spawn,SpawnNextBox.Request(),'physics_spawn',30.))
            if not spawned.success:raise RuntimeError('SPAWN_FAILED')
            # Ignore response.box; only the simulator actuator owns that truth.
            plan,detection,domain = self.stage('prepare',self.plan)
            result = self.stage('execute',lambda:self.action(self.execute,
                ExecuteJointOperationV2.Goal(plan=plan,detection=detection,top_domain=domain,budget_s=300.),
                'execute',310.))
            if not result.result.success:raise RuntimeError('EXECUTE:'+result.result.outcome+':'+result.result.reason)
            self.flush(outcome='FULL_CYCLE_PASS',capture_complete=True,replay_possible=True)
            return 0
        except Exception as error:
            if self.domain is not None:self.record('failure_top_domain',self.domain)
            for name,msg in self.images.items():self.record('failure_'+name,msg)
            if self.joint is not None:self.record('failure_joints',self.joint)
            self.flush(outcome='INCOMPLETE',reason=type(error).__name__+':'+str(error),
                       capture_complete=True,replay_possible=True)
            print(str(error),flush=True)
            return 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start',action='store_true',help='Explicitly authorize one simulation cycle')
    parser.add_argument('--run-dir',required=True)
    parser.add_argument('--evidence',required=True)
    parser.add_argument('--views',default='portal_observe_sim',help='Comma separated calibrated/synthetic site-pose names')
    args = parser.parse_args()
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
