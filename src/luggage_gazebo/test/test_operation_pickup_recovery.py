"""Observed adapter lease, capture, deadline and exact acquisition boundaries."""
from dataclasses import asdict
import json
from types import SimpleNamespace
import pytest
from rclpy.serialization import serialize_message
from luggage_msgs.action import ExecutePickupViewV2
from luggage_msgs.msg import DetectedLuggage, SuctionTopDomainV2
from luggage_msgs.srv import DetectLuggage, PrepareJointOperationV2
from uld_occ_observer.srv import GetSnapshot, GetValidity, ReleaseSnapshot
from luggage_gazebo.operation_pickup import ObservedPickupPreparation
from luggage_gazebo.operation_readiness import wait_observed_detection
from luggage_planning.joint_operation.pickup_recovery import PickupPreparationAttempt
from pickup_continuation_fixtures import case_archive
from test_pickup_recovery_cycle import settings, attempt, domain_message
from test_operation_sim_driver import driver_module


def record_to(archive):
    count = [0]
    def record(name, value):
        raw = value if isinstance(value, bytes) else serialize_message(value)
        archive.save(f'{count[0]:04d}_{name}', raw); count[0] += 1
    return record


@pytest.mark.parametrize('count',[None,0,1])
def test_configuration_creates_only_needed_clients_without_sending(case_archive,monkeypatch,count):
    effects=[]; record=record_to(case_archive)
    monkeypatch.setattr('luggage_gazebo.operation_pickup.ActionClient',
        lambda *args:effects.append(('action_client',args[-1])) or 'action')
    host=SimpleNamespace(record=record,create_client=lambda kind,name:effects.append(('service_client',name)) or 'service')
    path=None
    if count is not None:
        path=case_archive.directory/'config.json'
        config=dict(schema='pickup_view_recovery_v1',**asdict(settings(count)))
        path.write_text(json.dumps(config))
    helper=ObservedPickupPreparation(host,path)
    case_archive.save('startup_effects',effects)
    assert len(effects)==(2 if count else 0)
    assert (helper.view_action is not None)==bool(count)


def test_prepare_releases_initial_lease_before_view_and_rearms_capture(case_archive,monkeypatch):
    events=[]; domain=domain_message('original',1); detection=DetectedLuggage(id='original',header=domain.header)
    first=attempt('original-request',detection,domain)
    lease=GetSnapshot.Response(status='OK',handle='lease')
    lease.identity.epoch=2; lease.validity.epoch=2; lease.validity.generation=1; lease.validity.state='VALID'
    record=record_to(case_archive)
    def call(client,request,name,**kwargs):
        record(name+'_request',request); events.append(name)
        if client=='validity':reply=GetValidity.Response(epoch=2)
        elif client=='snapshot':reply=lease
        elif client=='release':reply=ReleaseSnapshot.Response(status='OK')
        else:raise AssertionError(client)
        record(name+'_response',reply); return reply
    host=SimpleNamespace(validity='validity',snapshot='snapshot',release_snapshot='release',call=call,
        record=record,pickup_task_id='stable-box',pickup_capture=object(),pickup_capture_active=False)
    helper=ObservedPickupPreparation(host); helper.settings=settings(1); helper.view_action='view'
    helper.observe=lambda **kw:(detection,domain)
    helper.prepare_from_observation=lambda *args,**kwargs:first
    def action(client,goal,name,timeout,**kwargs):
        assert 'release_prepare_snapshot' in events and host.pickup_capture_active
        released=events.index('release_prepare_snapshot')
        assert all(name=='actual_view_after_release' for name in events[released+1:])
        assert goal.pickup_task_id=='stable-box'
        events.append('actual_view_after_release')
        record('actual_view_goal',goal)
        return ExecutePickupViewV2.Impl.GetResultService.Response(status=6,
            result=ExecutePickupViewV2.Result(motion_terminal=True))
    host.action=action
    try:
        with pytest.raises(RuntimeError,match='PREPARE:SEARCH_INCOMPLETE'):helper.plan(task_mode='stack')
    finally:case_archive.save('lease_view_sequence',events)
    assert events[:3]==['prepare_validity','prepare_snapshot','release_prepare_snapshot']
    assert events.count('actual_view_after_release')==2 # two distinct probes, no controller dispatch


def test_observe_passes_actual_settle_bound_and_archives_exact_new_window(case_archive,monkeypatch):
    stamp=2_000_000_000; domain=domain_message('later',3); detection=DetectedLuggage(id='later',header=domain.header)
    seen=[]; captures=[]
    def join(*args,**kwargs):seen.append(kwargs);return detection,domain
    monkeypatch.setattr('luggage_gazebo.operation_pickup.wait_observed_detection',join)
    def capture(folder,time_ns,**kwargs):
        captures.append((str(folder),time_ns,kwargs)); return dict(ok=True,stamp_ns=time_ns)
    host=SimpleNamespace(record=record_to(case_archive),detect='detect',domain=domain,call=lambda *a,**k:None,
        pickup_capture=SimpleNamespace(write=capture),pickup_capture_count=0,pickup_capture_required=('rgb','depth'),
        pickup_capture_active=True,images={'rgb':domain},folder=case_archive.directory)
    returned=ObservedPickupPreparation(host).observe(deadline=123.,after_stamp_ns=stamp)
    case_archive.save('join_parameters',seen);case_archive.save('capture_windows',captures)
    assert seen[0]['after_stamp_ns']==stamp and seen[0]['deadline']==123.
    assert returned==(detection,domain) and captures[0][1]==3_000_000_000
    assert host.pickup_capture_count==1 and not host.pickup_capture_active


@pytest.mark.parametrize('fresh',[False,True])
def test_acquisition_join_selects_exact_domain_among_multiple_detections(case_archive,fresh):
    domain=domain_message('target',3 if fresh else 2)
    other=DetectedLuggage(id='other',header=domain.header)
    target=DetectedLuggage(id='target',header=domain.header,height_valid=True,top_surface_valid=True,
        height_source=DetectedLuggage.HEIGHT_SOURCE_MEASURED_SUPPORT)
    response=DetectLuggage.Response(success=True,luggage=[other,target]); now=[0.]
    record=record_to(case_archive);record('detection_response',response);record('domain',domain)
    case_archive.save('source_bound',dict(after_stamp_ns=2_000_000_000,deadline=.2))
    try:
        if fresh:
            detected,observed=wait_observed_detection(lambda _:response,lambda:domain,deadline=.2,
                clock=lambda:now[0],pause=lambda dt:now.__setitem__(0,now[0]+dt),after_stamp_ns=2_000_000_000)
            assert detected.id=='target' and observed is domain
        else:
            with pytest.raises(TimeoutError,match='MATCHING_OBSERVATION'):
                wait_observed_detection(lambda _:response,lambda:domain,deadline=.2,
                    clock=lambda:now[0],pause=lambda dt:now.__setitem__(0,now[0]+dt),after_stamp_ns=2_000_000_000)
    finally:case_archive.save('join_terminal_clock',now)


@pytest.mark.parametrize('boundary',['service','action'])
@pytest.mark.parametrize('expiry',['ready','record','result',None])
def test_recovery_transport_deadlines_include_readiness_and_recording(case_archive,monkeypatch,boundary,expiry):
    module=driver_module(); now=[0.];effects=[]
    monkeypatch.setattr(module.time,'monotonic',lambda:now[0])
    real_record=record_to(case_archive)
    def record(name,value):
        real_record(name,value); effects.append(name)
        if expiry=='record' and name.endswith(('_request','_goal')):now[0]=2.
    def ready(**kwargs):
        effects.append(('ready',kwargs));
        if expiry=='ready':now[0]=2.
        return True
    def send(request,**kwargs):effects.append('dispatch');return 'pending'
    handle=SimpleNamespace(accepted=True,get_result_async=lambda:'result',cancel_goal_async=lambda:'cancel')
    result=ExecutePickupViewV2.Impl.GetResultService.Response(status=4)
    def future(pending,timeout,reason):
        effects.append(('wait',pending,timeout))
        if pending=='pending' and boundary=='action':return handle
        if expiry=='result':now[0]=2.
        return result if boundary=='action' else DetectLuggage.Response()
    host=SimpleNamespace(record=record,future=future,args=SimpleNamespace())
    client=SimpleNamespace(wait_for_service=ready,call_async=send,wait_for_server=ready,send_goal_async=send)
    call=(lambda:module.OperationSimDriver.call(host,client,DetectLuggage.Request(),'recovery',deadline=1.)) if boundary=='service' else (
        lambda:module.OperationSimDriver.action(host,client,ExecutePickupViewV2.Goal(),'pickup_extra_view',1.,deadline=1.))
    try:
        if expiry in ('ready','record') or (expiry=='result' and boundary=='service'):
            with pytest.raises(TimeoutError):call()
        else:call()
    finally:case_archive.save('deadline_effects',dict(clock=now[0],effects=effects))
    assert effects.count('dispatch')==(0 if expiry in ('ready','record') else 1)


def test_each_new_box_gets_one_task_key_preserved_across_prepare_retries(case_archive):
    from luggage_gazebo.operation_cycle import run_observed_cycle
    from luggage_msgs.msg import JointOperationPlanV2
    from luggage_msgs.srv import SpawnNextBox
    record=record_to(case_archive);keys=[]; replies=[]
    domain=SuctionTopDomainV2(instance_id='one-box')
    def plan(**kwargs):
        keys.append(driver.pickup_task_id)
        return JointOperationPlanV2(),DetectedLuggage(),domain
    def execute(*args):
        reply=SimpleNamespace(success=len(replies)%2==1,outcome='STOPPED',motion_terminal=True,
            may_have_payload=False,reason='CURRENT_SOURCE_REFUSED')
        replies.append(reply);return SimpleNamespace(result=reply)
    driver=SimpleNamespace(args=SimpleNamespace(prepick_replans=1),stage=lambda name,call:call(),
        call=lambda *args:SpawnNextBox.Response(success=True),spawn='spawn',plan=plan,execute_prepared=execute,
        return_to_pick_observation=lambda *args:None)
    try:
        run_observed_cycle(driver);run_observed_cycle(driver)
    finally:case_archive.save('task_keys',keys)
    assert keys[0]==keys[1] and keys[2]==keys[3] and keys[0]!=keys[2]


def test_same_box_prepare_retry_never_renews_recovery_or_fallback_time(case_archive,monkeypatch):
    from luggage_planning.joint_operation.pickup_recovery import PickupPreparationRecovery
    from luggage_msgs.msg import JointOperationPlanV2
    from luggage_msgs.srv import ReprepareAfterPickupViewV2
    from test_pickup_recovery_cycle import domain_message
    now=[0.]; goals=[]; deadlines=[]; count=[0]; record=record_to(case_archive)
    domain=domain_message('original',1); detection=DetectedLuggage(id='original',header=domain.header)
    host=SimpleNamespace(record=record,pickup_task_id='same-box',pickup_capture=None)
    helper=ObservedPickupPreparation(host); helper.settings=settings(2)
    monkeypatch.setattr('luggage_gazebo.operation_pickup.time.monotonic',lambda:now[0])
    def workflow(config,ports,**kwargs):
        kwargs['clock']=lambda:now[0]
        return PickupPreparationRecovery(config,ports,**kwargs)
    monkeypatch.setattr('luggage_gazebo.operation_pickup.PickupPreparationRecovery',workflow)
    helper.observe=lambda **kwargs:(detection,domain)
    def prepare(detected,observed,**kwargs):
        deadline=kwargs.get('deadline'); deadlines.append(deadline)
        return attempt('initial' if deadline is None else 'fallback',detected,observed,
            success=deadline is not None)
    helper.prepare_on_map=prepare
    def action(goal,deadline):
        goals.append(goal);record('retry_view_goal',goal);count[0]+=1
        result=ExecutePickupViewV2.Result(success=True,submitted=True,motion_terminal=True,
            settled_stamp_valid=True,settled_stamp_ns=2_000_000_000,dispatched_motions=count[0])
        search=goal.preparation
        result.plan.request_id,result.plan.task_request_id=search.request_id,search.task_request_id
        result.plan.domain_sha256,result.plan.query_id=search.domain_sha256,search.query_id
        return ExecutePickupViewV2.Impl.GetResultService.Response(status=4,result=result)
    helper.execute_view=action
    def continued(request,deadline):
        output=ReprepareAfterPickupViewV2.Response(associated=True,success=count[0]==1,
            reason='IK_FAILED',detection=detection,top_domain=domain,
            plan=JointOperationPlanV2(plan_digest='a'*64 if count[0]==1 else ''))
        record('retry_continue_response',output);return output
    helper.continue_after=continued
    try:
        helper.plan(task_mode='stack');now[0]=5.;helper.plan(task_mode='stack')
    finally:case_archive.save('retry_deadlines',dict(clock=now[0],map_deadlines=deadlines,
        goals_budget_s=[goal.budget_s for goal in goals]))
    assert len(goals)==2 and all(goal.pickup_task_id=='same-box' for goal in goals)
    assert goals[1].budget_s<=5. and deadlines[-1]<=10.
