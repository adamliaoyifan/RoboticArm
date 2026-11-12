"""Actual native admission and ROS ExecuteTrajectory; synthetic controller only."""
from dataclasses import asdict, replace
from pathlib import Path
import hashlib
import json
import os
import threading
import time
from types import SimpleNamespace

import pytest

from test_joint_operation_revalidation_native import native_gate, issued, context, port
from luggage_planning.joint_operation.motion import MeasuredJointSample, StrictOperationMotionPort

pytestmark = pytest.mark.skipif(not os.environ.get('JOINT_NATIVE_CERTIFIER_EXECUTABLE'),
    reason='explicit isolated native/controller diagnostic gate')


@pytest.fixture(scope='module')
def controller(native_gate):
    import rclpy
    from moveit_msgs.action import ExecuteTrajectory
    from moveit_msgs.srv import GetMotionPlan
    from rclpy.action import ActionServer, GoalResponse, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.serialization import serialize_message
    gate = native_gate
    node = rclpy.create_node('operation_controller_fixture', context=gate.node.context)
    state = SimpleNamespace(mode='success', goals=[], cancelled=[], q=(), stamp_ns=0,
        accept_gate=threading.Event(), execution_gate=threading.Event(), results=[],planner_calls=0)
    state.accept_gate.set()
    records=[]
    def record(kind, raw):
        # Called by ROS worker threads and the main owner; keep names unique.
        with capture_lock:
            name='motion_%04d_%s.bin'%(len(records),kind)
            (gate.directory/name).write_bytes(raw)
            records.append({'file':name,'kind':kind,'sha256':hashlib.sha256(raw).hexdigest(),
                'case':os.environ.get('PYTEST_CURRENT_TEST')})
    capture_lock=threading.Lock()
    def sample():
        return MeasuredJointSample(state.q,state.stamp_ns,True)
    def update(q):
        state.q=tuple(q);state.stamp_ns=gate.node.get_clock().now().nanoseconds
    def goal(request):
        record('controller_received_goal',serialize_message(request))
        state.goals.append(request)
        assert state.accept_gate.wait(3.), 'test did not release late goal acceptance'
        return GoalResponse.REJECT if state.mode=='reject' else GoalResponse.ACCEPT
    def cancel(handle):
        state.cancelled.append(list(bytes(handle.goal_id.uuid)))
        return CancelResponse.ACCEPT
    def execute(handle):
        mode=state.mode
        if mode=='hold':
            assert state.execution_gate.wait(3.), 'test did not release held controller'
        result=ExecuteTrajectory.Result()
        if handle.is_cancel_requested:
            handle.canceled();result.error_code.val=-7
        elif mode=='abort':
            handle.abort();result.error_code.val=1
        else:
            endpoint=tuple(handle.request.trajectory.joint_trajectory.points[-1].positions)
            if mode!='old_stamp':
                update(tuple(v+(.2 if mode=='wrong_end' else 0.) for v in endpoint))
            else:
                state.q=endpoint
            handle.succeed();result.error_code.val=-1 if mode=='moveit_failure' else 1
        state.results.append(mode)
        return result
    server=ActionServer(node,ExecuteTrajectory,'/execute_trajectory',execute,
        goal_callback=goal,cancel_callback=cancel,callback_group=ReentrantCallbackGroup())
    def planner(request,response):
        state.planner_calls+=1
        response.motion_plan_response.error_code.val=-1
        return response
    planner_service=node.create_service(GetMotionPlan,'/plan_kinematic_path',planner)
    executor=rclpy.executors.MultiThreadedExecutor(num_threads=3,context=gate.node.context)
    executor.add_node(node)
    worker=threading.Thread(target=executor.spin,daemon=True);worker.start()
    arbiter=threading.Lock()
    transport=StrictOperationMotionPort(gate.node,motion_lock=arbiter,
        start_tolerance_rad=.01,end_tolerance_rad=.01,max_joint_age_ns=10**9,recorder=record)
    assert transport.client.wait_for_server(timeout_sec=5.)
    try:
        yield SimpleNamespace(state=state,port=transport,sample=sample,update=update,
            arbiter=arbiter,record=record,records=records)
    finally:
        state.accept_gate.set();state.execution_gate.set()
        transport.cancel()
        end=time.monotonic()+3.
        while time.monotonic()<end and arbiter.locked():
            transport.poll_terminal();time.sleep(.01)
        executor.shutdown(timeout_sec=3.);worker.join(timeout=3.)
        server.destroy();node.destroy_service(planner_service);node.destroy_node();transport.client.destroy()
        (gate.directory/'controller_manifest.json').write_text(json.dumps({
            'scope':'actual ROS action/native validation with synthetic joint/controller fixture; no hardware',
            'records':records,'goals':len(state.goals),'cancelled':state.cancelled,
            'results':state.results,'worker_joined':not worker.is_alive(),
            'motion_lock_released':not arbiter.locked(),
            'observed_plan_kinematic_path_calls':state.planner_calls},indent=2))
        assert not worker.is_alive()
        assert not arbiter.locked(), 'unresolved controller submission at teardown'
        assert state.planner_calls==0


@pytest.fixture
def invocation(native_gate,issued,controller,request):
    gate,c= native_gate,controller
    c.state.mode='success';c.state.accept_gate.set();c.state.execution_gate.clear()
    plan=issued[0]
    index=getattr(request,'param',2)
    c.update(plan.legs[index].knots[0].positions)
    arguments=context(gate,issued,index)
    native=port(gate)
    def validate(plan,index,**limits):
        values=dict(arguments);values.update(limits);values['next_leg_index']=index
        values['actual_joints']=c.state.q
        return native.validate(plan,**values)
    options=dict(expected_identity=plan.identity,expected_digest=plan.plan_digest,
        geometry=None if index<2 else plan.predicted_attachment,validate_current=validate,
        source_current=lambda *a:True,read_joints=c.sample,
        deadline=time.monotonic()+5.,cancelled=lambda:False)
    original=gate.directory/('motion_case_'+str(time.monotonic_ns())+'.json')
    original.write_text(json.dumps({'case':os.environ.get('PYTEST_CURRENT_TEST'),
        'original_plan_digest':plan.plan_digest,'index':index,
        'original_leg':asdict(plan.legs[index]),'controller':'synthetic','q':c.state.q},indent=2))
    def run(**changes):
        values=dict(options);values.update(changes)
        outcome=c.port.execute(plan,index,**values)
        c.record('outcome',json.dumps(asdict(outcome)).encode())
        return outcome
    yield SimpleNamespace(run=run,options=options,validate=validate,c=c,plan=plan,args=arguments,index=index)
    assert not c.arbiter.locked(), 'case must consume real terminal before next case'


@pytest.mark.parametrize('invocation',range(6),indirect=True)
def test_original_knots_reach_actual_action_unchanged(invocation):
    f=invocation;c=f.c;before=len(c.state.goals)
    result=f.run()
    assert result.success and result.terminal and result.submitted,result
    assert len(c.state.goals)==before+1
    trajectory=c.state.goals[-1].trajectory.joint_trajectory
    assert tuple(trajectory.joint_names)==f.plan.legs[f.index].joint_names
    assert len(trajectory.points)==len(f.plan.legs[f.index].knots)
    for point,knot in zip(trajectory.points,f.plan.legs[f.index].knots):
        assert tuple(point.positions)==knot.positions
        assert tuple(point.velocities)==knot.velocities
        assert tuple(point.accelerations)==knot.accelerations
        assert tuple(point.effort)==knot.effort
        assert point.time_from_start.sec*10**9+point.time_from_start.nanosec==knot.time_from_start_ns


@pytest.mark.parametrize('mode',['reject','abort','moveit_failure','wrong_end','old_stamp'])
def test_controller_terminal_requires_all_success_gates(invocation,mode):
    f=invocation;f.c.state.mode=mode
    result=f.run()
    assert not result.success and result.terminal and result.submitted,result
    assert not f.c.arbiter.locked()


@pytest.mark.parametrize('kind',['false','truthy','exception','changed'])
def test_current_source_refuses_before_action(invocation,kind):
    f=invocation;before=len(f.c.state.goals);calls=[]
    def source(*a):
        calls.append(1)
        if kind=='exception':raise ValueError('injected owner source failure')
        return (len(calls)==1) if kind=='changed' else (1 if kind=='truthy' else False)
    result=f.run(source_current=source)
    assert not result.success and result.terminal and not result.submitted,result
    assert len(f.c.state.goals)==before


@pytest.mark.parametrize('kind',['bool','negative','truthy','identity','index','digest','certificate','geometry','native_id','scene_hash','native_start'])
def test_native_receipt_binding_refuses_before_action(invocation,kind):
    f=invocation;before=len(f.c.state.goals)
    def validate(*a,**kw):
        value=f.validate(*a,**kw)
        assert value.receipt.accepted,value
        if kind=='bool':return True
        if kind=='native_id':return replace(value,native_validation_id='not native evidence')
        if kind=='scene_hash':return replace(value,current_scene_sha256='')
        fields={'negative':{'accepted':False},'truthy':{'accepted':1},
            'identity':{'identity':replace(f.plan.identity,box_id='another-box')},'index':{'next_leg_index':3},
            'digest':{'plan_digest':'x'},'certificate':{'certificate_id':'x'},
            'geometry':{'attachment_geometry_digest':'x'},
            'native_start':{'actual_joints':(9.,)*6}}
        return replace(value,receipt=replace(value.receipt,**fields[kind]))
    result=f.run(validate_current=validate)
    assert not result.success and result.terminal and not result.submitted,result
    assert len(f.c.state.goals)==before


@pytest.mark.parametrize('kind',['full_turn','stale','future','invalid','malformed'])
def test_actual_start_after_native_is_fresh_and_keeps_raw_winding(invocation,kind):
    f=invocation;before=len(f.c.state.goals)
    def sample():
        current=f.c.sample()
        if kind=='full_turn':return replace(current,positions=current.positions[:5]+(current.positions[5]+2*3.141592653589793,))
        if kind=='stale':return replace(current,stamp_ns=current.stamp_ns-2*10**9)
        if kind=='future':return replace(current,stamp_ns=current.stamp_ns+10*10**9)
        if kind=='invalid':return replace(current,stamp_valid=False)
        return replace(current,positions=(0.,))
    result=f.run(read_joints=sample)
    assert not result.success and result.terminal and not result.submitted,result
    assert len(f.c.state.goals)==before


def terminal(c):
    deadline=time.monotonic()+3.
    while time.monotonic()<deadline:
        result=c.port.poll_terminal()
        if result is not None:return result
        time.sleep(.01)
    pytest.fail('controller did not deliver a real terminal')


def test_late_acceptance_is_cancelled_and_still_owns_motion(invocation):
    f=invocation;c=f.c;c.state.mode='hold';c.state.accept_gate.clear()
    result=f.run(deadline=time.monotonic()+.25)
    assert result.submitted and not result.terminal and not result.success,result
    assert c.arbiter.locked()
    before=len(c.state.goals)
    blocked=f.run()
    assert blocked.reason=='MOTION_SUBMISSION_UNRESOLVED' and not blocked.submitted
    assert len(c.state.goals)==before
    cancels=len(c.state.cancelled)
    c.state.accept_gate.set()
    end=time.monotonic()+2.
    while len(c.state.cancelled)==cancels and time.monotonic()<end:time.sleep(.01)
    assert len(c.state.cancelled)==cancels+1
    assert c.arbiter.locked(), 'cancel acknowledgement is not terminal'
    c.state.execution_gate.set()
    done=terminal(c)
    assert done.terminal and not done.success and done.controller_status==5,done
    assert not c.arbiter.locked()


def test_cancel_during_execution_waits_for_controller_terminal(invocation):
    f=invocation;c=f.c;c.state.mode='hold'
    cancels=len(c.state.cancelled)
    result=f.run(deadline=time.monotonic()+.25)
    assert result.submitted and not result.terminal,result
    assert c.arbiter.locked()
    c.port.cancel()
    end=time.monotonic()+2.
    while len(c.state.cancelled)==cancels and time.monotonic()<end:time.sleep(.01)
    assert len(c.state.cancelled)==cancels+1
    c.state.execution_gate.set()
    done=terminal(c)
    assert not done.success and done.terminal and done.controller_status==5,done


def test_shared_owner_motion_lock_excludes_other_entry(invocation):
    f=invocation;before=len(f.c.state.goals)
    f.c.arbiter.acquire()
    try:
        result=f.run()
        assert result.reason=='SHARED_MOTION_BUSY' and not result.submitted,result
        assert len(f.c.state.goals)==before
    finally:
        f.c.arbiter.release()


@pytest.mark.parametrize('kind',['geometry','cancelled','deadline','wrong_digest','callback_exception'])
def test_preflight_refuses_without_controller_submission(invocation,kind):
    f=invocation;before=len(f.c.state.goals)
    def failing(*a,**kw):raise RuntimeError('injected native caller failure')
    changes={'geometry':{'geometry':None},'cancelled':{'cancelled':lambda:True},
        'deadline':{'deadline':time.monotonic()-1.},'wrong_digest':{'expected_digest':'wrong'},
        'callback_exception':{'validate_current':failing}}[kind]
    result=f.run(**changes)
    assert not result.success and result.terminal and not result.submitted,result
    assert len(f.c.state.goals)==before


@pytest.mark.parametrize('stage',['goal','acceptance','terminal'])
def test_capture_failure_cannot_grant_motion_success(invocation,stage,monkeypatch):
    f=invocation;original=f.c.port.recorder;before=len(f.c.state.goals)
    kind={'goal':'motion_goal','acceptance':'motion_acceptance','terminal':'motion_terminal'}[stage]
    def broken(name,raw):
        if name==kind:raise OSError('injected evidence sink failure')
        original(name,raw)
    monkeypatch.setattr(f.c.port,'recorder',broken)
    result=f.run()
    assert not result.success and result.terminal,result
    assert result.submitted is (stage!='goal')
    assert len(f.c.state.goals)==before+(stage!='goal')


@pytest.mark.parametrize('stage',['send','acceptance','result','nonterminal_result'])
def test_transport_uncertainty_holds_ownership_without_clear_shortcut(invocation,native_gate,stage,monkeypatch):
    # Boundary fault injection, not a claim of actual DDS loss. No controller
    # goal is transmitted; this intentionally stranded fixture lock can be
    # discarded only with its isolated owner, never cleared as a terminal goal.
    from concurrent.futures import Future
    from moveit_msgs.action import ExecuteTrajectory
    f=invocation;lock=threading.Lock();before=len(f.c.state.goals)
    transport=StrictOperationMotionPort(native_gate.node,motion_lock=lock,
        start_tolerance_rad=.01,end_tolerance_rad=.01,max_joint_age_ns=10**9,
        recorder=f.c.record)
    cancels=[]
    def submit(goal):
        if stage=='send':raise RuntimeError('injected uncertain send')
        acceptance=Future()
        if stage=='acceptance':
            acceptance.set_exception(RuntimeError('injected acceptance loss'))
        else:
            result=Future()
            if stage=='result':result.set_exception(RuntimeError('injected result loss'))
            else:
                value=ExecuteTrajectory.Impl.GetResultService.Response()
                value.status=2;result.set_result(value)
            def cancel():
                cancels.append(1)
                from action_msgs.srv import CancelGoal
                response=Future();response.set_result(CancelGoal.Response());return response
            acceptance.set_result(SimpleNamespace(accepted=True,
                goal_id=SimpleNamespace(uuid=bytes(range(16))),
                get_result_async=lambda:result,cancel_goal_async=cancel))
        return acceptance
    monkeypatch.setattr(transport.client,'send_goal_async',submit)
    try:
        result=transport.execute(f.plan,2,**f.options)
        f.c.record('injected_transport_outcome',json.dumps({'stage':stage,
            'outcome':asdict(result),'isolated_owner_lock_held':lock.locked()}).encode())
        assert not result.success and not result.terminal and result.submitted,result
        assert lock.locked() and transport.poll_terminal() is None
        blocked=transport.execute(f.plan,2,**f.options)
        assert blocked.reason=='MOTION_SUBMISSION_UNRESOLVED' and not blocked.submitted
        assert len(f.c.state.goals)==before
        if stage in ('result','nonterminal_result'):assert len(cancels)==1
    finally:
        transport.client.destroy()

@pytest.mark.parametrize('damage',(None,'path','attached','source'))
def test_post_release_view_requires_exact_native_unloaded_path_and_current_source(native_gate,issued,controller,damage):
    from rclpy.serialization import deserialize_message
    from moveit_msgs.msg import PlanningScene,AttachedCollisionObject
    from luggage_msgs.srv import ValidateRouteV2
    from luggage_planning.joint_operation.certification import _wire_leg
    c=controller;c.state.mode='success';c.state.accept_gate.set();c.state.execution_gate.set()
    leg=replace(issued[0].legs[0],name='post_place_explore',phase='retreat')
    c.update(leg.knots[0].positions);before=len(c.state.goals)
    scene=deserialize_message(issued[1].scenes.pick.scene_cdr,PlanningScene)
    wire=_wire_leg(leg);wire.load_state_index=0;wire.carrying=False
    request=ValidateRouteV2.Request(request_id=leg.trajectory_request_id,
        scene_revision=leg.trajectory_scene_revision,route_digest='view-path',scene_digest='view-scene',
        load_state_scenes=[scene],legs=[wire],max_sample_dt_ns=50_000_000.,max_sample_step_m=.01,
        joint_velocity_limits=[.314]*6,joint_acceleration_limits=[.314]*6,
        distance_stride=2**30,budget_s=3.)
    def validate(_):
        future=native_gate.port.client.call_async(request);end=time.monotonic()+4
        while not future.done() and time.monotonic()<end:time.sleep(.001)
        assert future.done();response=future.result()
        assert response.all_valid
        # Corrupt only after the real native response: an echo/verdict cannot
        # authorize a different path or attached state at controller dispatch.
        if damage=='path':request.legs[0].knots.points[-1].positions[0]+=.1
        if damage=='attached':request.load_state_scenes[0].robot_state.attached_collision_objects=[AttachedCollisionObject()]
        return request,response
    result=c.port.execute_observation(leg,model_sha256=native_gate.model.model_sha256,
        validate_current=validate,source_current=lambda:damage!='source',read_joints=c.sample,
        deadline=time.monotonic()+5.,cancelled=lambda:False)
    assert result.success==(damage is None),result
    assert result.terminal and not c.arbiter.locked()
    assert len(c.state.goals)==before+(damage is None)
