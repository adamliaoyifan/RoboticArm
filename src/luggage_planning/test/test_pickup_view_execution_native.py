"""Actual O/prepare/runtime/native/controller transports; synthetic actuator.

Declared missing-depth fixture and fixed wrist camera come from the view owner
gate. No Gazebo physics, new acquisition, association or hardware qualification.
"""
import copy
import importlib.util
import json
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest
from test_pickup_view_owner_native import view_owner_stack,camera_model
from test_joint_operation_endpoints_native import native_stack
from test_joint_operation_preparation_native import prepare_stack
from test_joint_operation_observer_native import observer_owner

pytestmark=[pytest.mark.skipif(not os.environ.get('JOINT_NATIVE_ENDPOINT_EXECUTABLE'),
    reason='explicit native view execution gate'),
    pytest.mark.parametrize('native_stack',[camera_model()],indirect=True,ids=['fixed-camera']),
    pytest.mark.parametrize('prepare_stack',['observer'],indirect=True)]


def until(predicate,timeout=5.):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate():return
        time.sleep(.005)
    assert predicate(),'bounded view execution condition missing'


def completed(future,timeout=25.):
    until(future.done,timeout)
    return future.result()


@pytest.fixture
def runtime_view(view_owner_stack,request):
    import rclpy
    from rclpy.action import ActionClient,ActionServer,CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.serialization import serialize_message
    from rclpy.time import Time
    from sensor_msgs.msg import JointState
    from luggage_msgs.msg import VacuumState
    from luggage_msgs.action import ExecutePickupViewV2
    from moveit_msgs.action import ExecuteTrajectory
    from moveit_msgs.srv import ApplyPlanningScene,GetMotionPlan
    from luggage_msgs.srv import VacuumCommandV2
    from luggage_planning.joint_operation.contracts import OPERATION_JOINTS
    s=view_owner_stack;f=s.f
    directory=f.directory/request.node.name;directory.mkdir()
    source=Path(__file__).parents[1]/'scripts/motion_planner_node.py'
    spec=importlib.util.spec_from_file_location('pickup_view_execution_host',source)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    params=dict(atlas_enabled=False,debug_capture=False,enable_joint_operation=True,
        use_sim_time=True,frame_scene_revision='scene',
        robot_description=camera_model()[0],
        robot_description_semantic=camera_model()[1],
        container_translation=[0.,0.,0.],container_quaternion_xyzw=[0.,0.,0.,1.],
        observer_geometry_file=str(f.observer.geometry),observer_config_file=str(f.observer.config),
        observer_max_age_ms=60000,geometric_slot_limit=2048,
        operation_state_file=str(directory/'journal.json'),operation_evidence_dir=str(directory/'execution'),
        settle_hold_time=.04,settle_timeout=1.)
    for name in ('scene_tf_config','container_collision_mesh'):
        params[name]=f.root.get_parameter(name).value
    node=module.MotionPlannerNode(context=f.owner.context,
        parameter_overrides=[Parameter(k,value=v) for k,v in params.items()])
    runtime=node._joint_operation
    state=SimpleNamespace(q=tuple(s.request.start_joints),goals=[],mode='success',vacuum=True,
        attached=False,vacuum_on=False,stop=threading.Event(),release=threading.Event(),
        error=None,cancelled=0,records=[],forbidden_calls=[])
    group=ReentrantCallbackGroup();capture_lock=threading.Lock()
    def record(name,message):
        raw=message if isinstance(message,bytes) else serialize_message(message)
        with capture_lock:
            file='%04d_'%len(state.records)+name
            (directory/file).write_bytes(raw);state.records.append(file)
    pub=node.create_publisher(JointState,'/joint_states',10)
    vacuum=node.create_publisher(VacuumState,'/vacuum/state',10)
    def sensors():
        began=time.monotonic()
        try:
            while not state.stop.wait(.01):
                stamp=12_000_000_000+int((time.monotonic()-began)*1e9)
                for owner in (node,f.root):
                    owner.get_clock().set_ros_time_override(Time(nanoseconds=stamp))
                message=JointState();message.header.stamp.sec,message.header.stamp.nanosec=divmod(stamp,10**9)
                message.name=list(OPERATION_JOINTS);message.position=list(state.q)
                pub.publish(message)
                if state.vacuum:
                    vacuum.publish(VacuumState(attached=state.attached,vacuum_on=state.vacuum_on))
        except Exception as error:state.error=str(error)
    def cancel(handle):
        state.cancelled+=1;record('cancel_requested.cdr',handle.request)
        return CancelResponse.ACCEPT
    def controller(handle):
        state.goals.append(copy.deepcopy(handle.request));record('controller_goal.cdr',handle.request)
        if state.mode=='hold':assert state.release.wait(5.),'release held controller'
        result=ExecuteTrajectory.Result()
        if handle.is_cancel_requested:
            handle.canceled();result.error_code.val=-7
        elif state.mode=='abort':
            handle.abort();result.error_code.val=-1
        else:
            state.q=tuple(handle.request.trajectory.joint_trajectory.points[-1].positions)
            until(lambda:node._joint_state is not None and tuple(node._joint_state.position)==state.q)
            handle.succeed();result.error_code.val=1
        record('controller_result.cdr',result)
        return result
    server=ActionServer(node,ExecuteTrajectory,'/execute_trajectory',controller,
        cancel_callback=cancel,callback_group=group)
    def forbidden(name):
        def callback(req,res):
            state.forbidden_calls.append(name);record('forbidden_'+name+'.cdr',req)
            return res
        return callback
    forbidden_services=[node.create_service(kind,path,forbidden(name),callback_group=group)
        for kind,path,name in ((VacuumCommandV2,'/vacuum/command_v2','vacuum'),
            (ApplyPlanningScene,'/apply_planning_scene','scene_write'),
            (GetMotionPlan,'/plan_kinematic_path','global_planner'))]
    client=ActionClient(node,ExecutePickupViewV2,'/joint_operation/execute_pickup_view',callback_group=group)
    executor=MultiThreadedExecutor(context=f.owner.context,num_threads=8);executor.add_node(node)
    worker=threading.Thread(target=executor.spin,daemon=True);worker.start()
    sensor=threading.Thread(target=sensors,daemon=True);sensor.start()
    goal=ExecutePickupViewV2.Goal(pickup_task_id='stationary-pickup',max_extra_view_motions=1,
        total_recovery_budget_s=30.,budget_s=20.)
    for field in goal.preparation.get_fields_and_field_types():
        setattr(goal.preparation,field,getattr(s.request,field))
    try:
        assert client.wait_for_server(timeout_sec=5.)
        for c in (runtime.pickup_view.ports.prepare_client,runtime.pickup_view.ports.check_client,
                  runtime.pickup_view.ports.native.client,runtime.ports.scene_client):
            assert c.wait_for_service(timeout_sec=5.)
        assert runtime.motion.client.wait_for_server(timeout_sec=5.)
        until(lambda:node._joint_state is not None and runtime.pickup_view.vacuum is not None)
        yield SimpleNamespace(s=s,f=f,node=node,runtime=runtime,state=state,goal=goal,client=client,
            record=record,directory=directory)
    finally:
        state.release.set();runtime.motion.cancel()
        until(lambda:not runtime.owner.motion_lock.locked(),5.)
        state.stop.set();sensor.join(timeout=3.)
        executor.shutdown(timeout_sec=3.);worker.join(timeout=3.)
        record('teardown.json',json.dumps(dict(sensor_joined=not sensor.is_alive(),
            executor_joined=not worker.is_alive(),sensor_error=state.error,
            goals=len(state.goals),cancels=state.cancelled,owner=runtime.owner.snapshot(),
            journal=runtime.journal.value,ledger=runtime.pickup_view.ledger.rows,
            forbidden_calls=state.forbidden_calls)).encode())
        assert not sensor.is_alive() and not worker.is_alive() and state.error is None
        assert not state.forbidden_calls
        client.destroy();server.destroy();node.destroy_node()


def invoke(f,goal=None):
    goal=copy.deepcopy(f.goal if goal is None else goal)
    f.record('action_goal.cdr',goal)
    handle=completed(f.client.send_goal_async(goal))
    assert handle.accepted,'view action not accepted'
    response=completed(handle.get_result_async())
    f.record('action_result.cdr',response)
    return response.result


def test_actual_o_view_dispatch_settle_and_retry_count(runtime_view):
    from rclpy.serialization import deserialize_message
    from luggage_msgs.msg import PickupViewPlanV2
    f=runtime_view
    result=invoke(f)
    assert result.success and result.submitted and result.motion_terminal,result.reason
    assert result.settled_stamp_valid and result.settled_stamp_ns>f.s.domain.stamp_ns
    assert result.dispatched_motions==1 and len(f.state.goals)==1
    assert f.state.goals[0].trajectory.joint_trajectory==result.plan.leg.trajectory.trajectory
    assert f.runtime.session is None and f.runtime.journal.value['active'] is None
    receipt = f.runtime.pickup_view.ledger.rows[f.goal.pickup_task_id]['settled_view']
    assert receipt['dispatched'] == 1 and receipt['settled_stamp_ns'] == result.settled_stamp_ns
    assert deserialize_message(bytes.fromhex(receipt['plan_cdr_hex']), PickupViewPlanV2) == result.plan
    retry=copy.deepcopy(f.goal);retry.preparation.request_id='another-prepare-same-box'
    result=invoke(f,retry)
    assert not result.success and not result.submitted and result.dispatched_motions==1
    assert 'MOTION_LIMIT' in result.reason and len(f.state.goals)==1


@pytest.mark.parametrize('fault',['loaded','vacuum_on','missing','zero'])
def test_no_dispatch_without_empty_state_or_budget(runtime_view,fault):
    f=runtime_view
    if fault=='loaded':f.state.attached=True
    if fault=='vacuum_on':f.state.vacuum_on=True
    if fault=='missing':f.state.vacuum=False
    if fault=='zero':f.goal.max_extra_view_motions=0
    f.goal.budget_s=.15
    result=invoke(f)
    assert not result.success and not result.submitted and not f.state.goals,result.reason
    assert result.dispatched_motions==0 and f.runtime.session is None


def test_native_collision_after_prepare_cannot_dispatch(runtime_view):
    from luggage_planning.joint_operation.scenes import _box
    from luggage_planning.placement_prior.contracts import Transform
    f=runtime_view;original=f.runtime.pickup_view.ports.prepare
    def prepare(*args,**kwargs):
        plan=original(*args,**kwargs)
        tool=f.f.root.model.urdf.tcp(f.state.q,f.f.root.model.path)
        f.f.state.scene.world.collision_objects.append(_box('new_measured_obstacle',(.02,)*3,
            Transform(tuple(tool[:3,3]),(0.,0.,0.,1.)),'world'))
        return plan
    f.runtime.pickup_view.ports.prepare=prepare
    try:
        result=invoke(f)
        assert not result.success and not result.submitted and not f.state.goals,result.reason
        assert 'NATIVE_CERTIFICATION_REFUSED' in result.reason,result.reason
        import gzip
        from rclpy.serialization import deserialize_message
        from luggage_msgs.srv import ValidateRouteV2
        rows=[r for r in f.runtime.records if 'view_certification_response' in r['file']]
        assert rows,'the collision case never reached the actual native certifier'
        reply=deserialize_message(gzip.decompress((f.runtime.directory/rows[-1]['file']).read_bytes()),ValidateRouteV2.Response)
        assert any(leg.reason=='COLLISION' and leg.contacts for leg in reply.leg_results)
        assert result.dispatched_motions==0
    finally:
        f.f.state.scene.world.collision_objects=[o for o in f.f.state.scene.world.collision_objects
                                                if o.id!='new_measured_obstacle']


def test_controller_abort_consumes_unit_and_releases_terminal_owner(runtime_view):
    f=runtime_view;f.state.mode='abort'
    result=invoke(f)
    assert not result.success and result.submitted and result.motion_terminal,result.reason
    assert result.dispatched_motions==1 and not result.settled_stamp_valid
    assert len(f.state.goals)==1 and f.runtime.session is None


def test_cancel_ack_keeps_owner_until_actual_late_terminal(runtime_view):
    f=runtime_view;f.state.mode='hold'
    handle=completed(f.client.send_goal_async(f.goal))
    assert handle.accepted
    until(lambda:len(f.state.goals)==1)
    completed(handle.cancel_goal_async())
    result=completed(handle.get_result_async()).result
    f.record('cancel_action_result.cdr',result)
    assert not result.success and result.submitted and not result.motion_terminal,result.reason
    assert result.dispatched_motions==1 and f.runtime.session is not None
    assert f.runtime.owner.motion_lock.locked()
    assert completed(f.client.send_goal_async(f.goal)).accepted is False
    f.state.release.set()
    until(lambda:f.runtime.session is None)
    assert not f.runtime.owner.motion_lock.locked() and f.runtime.journal.value['active'] is None


@pytest.mark.parametrize('fault',['scene_changed','invalid_certificate','stale_joints'])
def test_post_native_admission_faults_never_reach_controller(runtime_view,fault):
    from dataclasses import replace
    from luggage_planning.joint_operation.scenes import _box
    from luggage_planning.placement_prior.contracts import Transform
    f=runtime_view;ports=f.runtime.pickup_view.ports
    original=ports.native.validate
    entered=[]
    def validate(*args,**kwargs):
        request,response=original(*args,**kwargs)
        entered.append(True)
        if fault=='scene_changed':
            f.f.state.scene.world.collision_objects.append(_box('changed_world',(.02,)*3,
                Transform((10.,10.,10.),(0.,0.,0.,1.)),'world'))
        if fault=='invalid_certificate':
            response=copy.deepcopy(response);response.route_digest='a'*64
            f.record('injected_invalid_certificate.cdr',response)
        return request,response
    ports.native.validate=validate
    if fault=='stale_joints':
        read=f.runtime.ports.joints
        f.runtime.ports.joints=lambda:replace(read(),stamp_ns=0) if entered else read()
    try:
        result=invoke(f)
        assert entered and not result.success and not result.submitted and not f.state.goals,result.reason
        expected=dict(scene_changed='CURRENT_SOURCE_CHANGED',invalid_certificate='CERTIFICATION_IDENTITY_MISMATCH',
            stale_joints='STALE_OR_FUTURE_MEASURED_JOINTS')[fault]
        assert expected in result.reason,result.reason
        assert result.dispatched_motions==0 and f.runtime.session is None
    finally:
        f.f.state.scene.world.collision_objects=[o for o in f.f.state.scene.world.collision_objects if o.id!='changed_world']


def test_shared_motion_busy_cannot_prepare_or_dispatch(runtime_view):
    f=runtime_view
    f.runtime.owner.motion_lock.acquire()
    try:
        result=invoke(f)
        assert not result.success and not result.submitted and 'SHARED_MOTION_BUSY' in result.reason
        assert not f.state.goals and f.runtime.session is None
    finally:f.runtime.owner.motion_lock.release()


def test_cancellation_during_native_admission_sends_no_goal(runtime_view):
    f=runtime_view;entered=threading.Event();release=threading.Event()
    original=f.runtime.pickup_view.ports.native.validate
    def validate(*args,**kwargs):
        entered.set();assert release.wait(5.),'release admission fixture'
        return original(*args,**kwargs)
    f.runtime.pickup_view.ports.native.validate=validate
    handle=completed(f.client.send_goal_async(f.goal));assert handle.accepted
    try:
        assert entered.wait(5.),'native admission not reached'
        completed(handle.cancel_goal_async());release.set()
        result=completed(handle.get_result_async()).result
        f.record('pre_dispatch_cancel_result.cdr',result)
        assert not result.success and not result.submitted and result.dispatched_motions==0
        assert not f.state.goals and f.runtime.session is None
    finally:release.set()
