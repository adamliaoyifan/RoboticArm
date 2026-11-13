"""Same-goal bounded empty candidate search; scripted Action, durable boundaries."""
import copy,hashlib,importlib,json,os
from pathlib import Path
from types import SimpleNamespace
import pytest
from rclpy.serialization import serialize_message,deserialize_message
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints,JointConstraint,RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint,MultiDOFJointTrajectoryPoint
from luggage_gazebo.operation_motion import execute_observation_goal
from pickup_continuation_fixtures import case_archive


@pytest.fixture
def motion_case(case_archive):
    f=SimpleNamespace(archive=case_archive,clock=[0.],goals=[],deadlines=[],modes=[],records=[])
    f.goal=MoveGroup.Goal();f.goal.request.num_planning_attempts=3
    f.goal.request.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name='elfin_joint1',position=.3)])]
    def record(name,value):
        raw=value if isinstance(value,bytes) else serialize_message(value)
        case_archive.save(str(len(f.records))+'_'+name,raw);f.records.append(name)
    f.record=record
    def action(client,goal,name,remaining,*,deadline):
        f.goals.append(copy.deepcopy(goal));f.deadlines.append(deadline);record('goal',goal)
        mode=f.modes.pop(0) if f.modes else 'success';f.clock[0]+=1.
        if mode=='uncertain':
            record('action_uncertain',b'controlled transport timeout');raise TimeoutError('ACTION_UNCERTAIN')
        result=MoveGroup.Result();result.error_code.val=1 if mode in ('success','late') else -2
        status=4 if mode in ('success','late') else 6
        if mode=='execution':result.error_code.val=-4
        if mode=='partial':result.executed_trajectory.joint_trajectory.points=[JointTrajectoryPoint()]
        if mode=='multidof':result.executed_trajectory.multi_dof_joint_trajectory.points=[MultiDOFJointTrajectoryPoint()]
        if mode=='nonterminal':status=2
        if mode=='late':f.clock[0]=11.
        wrapped=MoveGroup.Impl.GetResultService.Response(status=status,result=result);record('result',wrapped)
        return wrapped
    f.host=SimpleNamespace(move='move',record=record,action=action)
    record('original_goal',f.goal)
    try:yield f
    finally:case_archive.save('motion_terminal',dict(clock=f.clock[0],deadlines=f.deadlines,
        calls=len(f.goals),remaining_modes=f.modes))


def invoke(f):
    try:return execute_observation_goal(f.host,f.goal,'observe_fixture',timeout=10.,clock=lambda:f.clock[0])
    except Exception as error:
        f.archive.save('motion_exception',dict(type=type(error).__name__,message=str(error)));raise


def test_first_valid_observation_candidate_uses_one_action(motion_case):
    f=motion_case;assert invoke(f).status==4 and len(f.goals)==1
    assert f.goal.request.num_planning_attempts==3 and f.goals[0].request.num_planning_attempts==1


def test_unexecuted_invalid_candidate_keeps_goal_and_original_deadline(motion_case):
    f=motion_case;f.modes=['invalid','success'];assert invoke(f).status==4
    assert len(f.goals)==2 and f.deadlines==[10.,10.]
    assert all(goal.request.goal_constraints==f.goal.request.goal_constraints for goal in f.goals)
    assert all(goal.request.num_planning_attempts==1 for goal in f.goals)


def test_all_invalid_observation_candidates_stop_at_original_attempt_count(motion_case):
    f=motion_case;f.modes=['invalid']*4
    with pytest.raises(RuntimeError,match='OBSERVE_MOVEIT:-2'):invoke(f)
    assert len(f.goals)==3 and f.modes==['invalid']


@pytest.mark.parametrize('mode',['execution','partial','multidof','nonterminal','uncertain','late'])
def test_execution_partial_uncertain_or_expired_observation_never_retries(motion_case,mode):
    f=motion_case;f.modes=[mode,'success']
    with pytest.raises((RuntimeError,TimeoutError)):invoke(f)
    assert len(f.goals)==1 and f.modes==['success']


def test_expired_recording_cannot_dispatch_another_observation_candidate(motion_case):
    f=motion_case;f.modes=['invalid','success'];normal=f.host.record
    def record(name,value):
        normal(name,value)
        if name=='observation_planning_attempt.json':f.clock[0]=11.
    f.host.record=record
    with pytest.raises(TimeoutError,match='OBSERVE_PLANNING_DEADLINE'):invoke(f)
    assert len(f.goals)==1


@pytest.mark.skipif(not os.environ.get('OBSERVATION_RETURN_REPLAY_ROOT'),reason='explicit full-model recorded candidate gate')
def test_recorded_full_model_rejection_then_native_valid_candidate_transport(case_archive):
    root=Path(os.environ['OBSERVATION_RETURN_REPLAY_ROOT']);rows=json.loads((root/'driver/manifest.json').read_text())['records']
    def read(prefix):
        row=next(r for r in rows if r['file'].startswith(prefix));raw=(root/'driver'/row['file']).read_bytes()
        assert hashlib.sha256(raw).hexdigest()==row['sha256'];module,kind=row['ros_type'].rsplit('.',1)
        return deserialize_message(raw,getattr(importlib.import_module(module),kind))
    goal=read('0108_');rejected=read('0109_');clock=[0.];calls=[]
    native=deserialize_message((root/'paired_timing01/seed_1/timed_0.cdr').read_bytes(),RobotTrajectory)
    report=json.loads((root/'paired_timing01/seed_1/result.json').read_text())
    assert report['timing'][0]['after_knot_valid']
    # Scripted execution acknowledgement only: these saved trajectories are not
    # current certified plans and are never sent to a controller by this test.
    success=MoveGroup.Impl.GetResultService.Response(status=4,result=MoveGroup.Result(
        planned_trajectory=native,executed_trajectory=native));success.result.error_code.val=1
    replies=iter((rejected,success))
    def record(name,value):case_archive.save(str(len(calls))+'_'+name,value if isinstance(value,bytes) else serialize_message(value))
    def action(client,request,name,remaining,*,deadline):
        calls.append(dict(remaining=remaining,deadline=deadline));record('goal',request)
        result=next(replies);record('result',result);clock[0]+=.25;return result
    case_archive.save('recorded_candidate_scope',dict(native_source=str(root),
        actuator='scripted acknowledgement; no native/controller execution in this transport fixture',
        native_proof='separate same-scene knot checks; continuous path and hardware qualification not established'))
    try:
        output=execute_observation_goal(SimpleNamespace(move='move',record=record,action=action),goal,
            'observe_recorded',timeout=90.,clock=lambda:clock[0])
        assert output.result.error_code.val==1 and len(calls)==2
        assert calls[1]['remaining']<calls[0]['remaining'] and calls[0]['deadline']==calls[1]['deadline']
    finally:case_archive.save('recorded_candidate_calls',calls)
