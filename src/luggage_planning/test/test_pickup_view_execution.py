"""Action lifecycle at scripted ports. Native/controller integration is separate."""
import copy
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest
from rclpy.serialization import serialize_message
from luggage_msgs.action import ExecutePickupViewV2
from luggage_msgs.srv import CheckPreparedPickupViewV2
from luggage_planning.joint_operation.motion import MotionOutcome,MeasuredJointSample
from luggage_planning.joint_operation.view_budget import PickupViewBudgetLedger
from luggage_planning.joint_operation.view_execution import PickupViewExecution
from luggage_planning.motion.motion_ownership import MotionOwnership
from test_pickup_view_owner import owner_case,prepare


@pytest.mark.parametrize('damage',['none','leg','expired','replaced','revoked'])
def test_independent_owner_receipt_is_exact_and_nonrenewing(owner_case,damage):
    f=owner_case;prepared=prepare(f);assert prepared.success
    request=CheckPreparedPickupViewV2.Request(plan=prepared.plan,budget_s=1.)
    if damage=='leg':request.plan.leg.trajectory.trajectory.points[-1].positions[0]+=.1
    if damage=='expired':f.owner.contexts.entries.clear()
    if damage=='replaced':f.owner.contexts.forget('task')
    if damage=='revoked':f.retained=False
    response=f.owner.check(request,CheckPreparedPickupViewV2.Response())
    f.node._record('check_view_request',request);f.node._record('check_view_response',response)
    assert response.current==(damage=='none'),response.reason
    if response.current:
        assert response.top_domain.box_id==f.domain.box_id
        assert bytes(response.checked_scene_cdr)==f.owner.contexts.issued('task',prepared.plan.route_digest).snapshot.scene_cdr
        assert response.probe==f.probe_message
    assert not f.node._active.locked()


@pytest.fixture
def lifecycle(tmp_path):
    view=PickupViewExecution.__new__(PickupViewExecution)
    view.action=ExecutePickupViewV2;view.owns_session=False;view.phase='IDLE';view.task_id=''
    view.dispatched_this_action=False;view.unexpected_load=False;view.load_checkpointed=False
    view.unexpected_load_sample=None
    view.ledger=PickupViewBudgetLedger(tmp_path/'budget.json')
    view.vacuum_lock=threading.Lock();view.vacuum=None
    owner=MotionOwnership()
    f=SimpleNamespace(directory=tmp_path,records=[],outcomes=[],prepares=[],settles=[],
        mode='success',empty=True,unresolved=False,terminal=None,fail_prepare=False,clock=12_000_000_000)
    def record(name,value):
        raw=value if isinstance(value,bytes) else serialize_message(value)
        (tmp_path/('%04d_'%len(f.records)+name)).write_bytes(raw);f.records.append(name)
    params=dict(settle_vel_tol=.01,settle_hold_time=.02,settle_timeout=.1)
    def joints():
        f.clock+=20_000_000
        return MeasuredJointSample((0.,)*6,f.clock,True)
    node=SimpleNamespace(get_parameter=lambda key:SimpleNamespace(value=params[key]),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=f.clock)))
    f.journal=[]
    parent=SimpleNamespace(node=node,owner=owner,record=record,_active=threading.Lock(),session=None,
        handle=None,_uncertain=False,evidence_root=tmp_path,journal=SimpleNamespace(write=f.journal.append),
        ports=SimpleNamespace(model=SimpleNamespace(model_sha256='m'*64),joints=joints,
            rpc=SimpleNamespace(unresolved=lambda:f.unresolved)))
    def execute(leg,**kwargs):
        if f.mode=='source_refused':return MotionOutcome(False,True,False,'CURRENT_SOURCE_REFUSED')
        if f.mode=='pre_cancel':return MotionOutcome(False,True,False,'CANCELLED_OR_DEADLINE')
        kwargs['on_dispatch']()
        if f.mode=='uncertain':
            owner.motion_lock.acquire();return MotionOutcome(False,False,True,'DEADLINE')
        return MotionOutcome(f.mode=='success',True,True,f.mode)
    parent.motion=SimpleNamespace(execute_pickup_view=execute,max_joint_age_ns=10**9,
        poll_terminal=lambda:f.terminal)
    def prepare_view(search,**kwargs):
        f.prepares.append(search.request_id)
        if f.fail_prepare:raise ValueError('SCRIPTED_PREPARE_FAILED')
        return view.action.Result().plan
    view.parent=parent;view.node=node
    view.ports=SimpleNamespace(prepare=prepare_view,leg=object(),validate=lambda *a,**kw:None,current=lambda d:True)
    view.empty=lambda:f.empty
    def handle(request_id='view1',limit=2):
        goal=ExecutePickupViewV2.Goal(pickup_task_id='stable-box',max_extra_view_motions=limit,
            total_recovery_budget_s=10.,budget_s=.3)
        goal.preparation.domain_sha256='d'*64;goal.preparation.request_id=request_id
        return SimpleNamespace(request=goal,is_cancel_requested=False,publish_feedback=lambda msg:record('feedback',msg),
            succeed=lambda:f.outcomes.append('succeed'),abort=lambda:f.outcomes.append('abort'),
            canceled=lambda:f.outcomes.append('canceled'))
    f.view,f.parent,f.handle=view,parent,handle
    return f


def test_terminal_motion_settles_and_retry_does_not_reset_count(lifecycle):
    f=lifecycle;result=f.view.execute(f.handle())
    assert result.success and result.submitted and result.motion_terminal and result.settled_stamp_valid
    assert result.dispatched_motions==1 and not f.parent._active.locked() and f.parent.session is None
    second=f.view.execute(f.handle('new-prepare-id'))
    assert second.success and second.dispatched_motions==2
    third=f.view.execute(f.handle('third-prepare-id'))
    assert not third.success and not third.submitted and third.dispatched_motions==2
    assert f.prepares==['view1','new-prepare-id']


@pytest.mark.parametrize('mode',['source_refused','pre_cancel','abort','uncertain'])
def test_failed_or_nonterminal_dispatch_counts_and_holds_correct_owner(lifecycle,mode):
    f=lifecycle;f.mode=mode
    result=f.view.execute(f.handle())
    submitted=mode in ('abort','uncertain')
    assert not result.success and result.submitted==submitted
    assert result.dispatched_motions==int(submitted)
    if mode=='uncertain':
        assert not result.motion_terminal and f.parent.session is not None and f.parent.owner.motion_lock.locked()
        f.view.poll_terminal();assert f.parent.session is not None
        f.parent.owner.motion_lock.release();f.terminal=MotionOutcome(False,True,True,'CANCELLED')
        f.view.poll_terminal();assert f.parent.session is None
    else:assert result.motion_terminal and f.parent.session is None


def test_prepare_failure_zero_limit_and_loaded_state_do_not_dispatch(lifecycle):
    f=lifecycle;f.fail_prepare=True
    result=f.view.execute(f.handle())
    assert not result.success and not result.submitted and result.dispatched_motions==0
    f.fail_prepare=False;f.empty=False
    f.view.vacuum=(time.monotonic()+1.,True,True)
    result=f.view.execute(f.handle('loaded'))
    assert not result.success and not result.submitted and 'LOADED' in result.reason
    assert f.prepares==['view1'] and f.parent.session is None


@pytest.mark.parametrize('fault',['same_stamp','moving','no_samples'])
def test_settle_requires_distinct_post_motion_measurements(lifecycle,fault):
    f=lifecycle;counter=[0]
    def joints():
        counter[0]+=1
        if fault=='no_samples':return MeasuredJointSample((0.,)*6,0,False)
        f.clock+=20_000_000
        stamp=12_020_000_000 if fault=='same_stamp' else f.clock
        return MeasuredJointSample((counter[0]*.1,)*6,stamp,True)
    f.parent.ports.joints=joints
    result=f.view.execute(f.handle())
    assert not result.success and result.submitted and result.motion_terminal and not result.settled_stamp_valid
    assert 'SETTLE_NOT_CONFIRMED' in result.reason


def test_same_timestamp_before_admission_is_not_fresh_empty_state(lifecycle):
    f=lifecycle;del f.view.empty
    f.view.began=time.monotonic()
    f.view.vacuum=(f.view.began-1.,False,False);assert not f.view.empty()
    f.view.vacuum=(time.monotonic(),False,False);assert f.view.empty()
    f.view.vacuum=(time.monotonic(),True,False);assert not f.view.empty()


def test_unexpected_attachment_after_dispatch_does_not_autorelease_task(lifecycle):
    from luggage_msgs.msg import VacuumState
    f=lifecycle;f.mode='uncertain'
    result=f.view.execute(f.handle())
    assert not result.motion_terminal and result.submitted
    f.view.on_vacuum(VacuumState(attached=True,vacuum_on=True))
    f.parent.owner.motion_lock.release();f.terminal=MotionOutcome(False,True,True,'CANCELLED')
    f.view.poll_terminal()
    assert f.parent.session is not None and f.view.unexpected_load
    assert f.journal[-1]['may_have_payload']
    # Receipt of a subsequent off sample cannot undo an unexplained attachment.
    f.view.on_vacuum(VacuumState());f.view.poll_terminal()
    assert f.parent.session is not None
