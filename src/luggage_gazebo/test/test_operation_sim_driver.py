"""Exercise the diagnostic client's real snapshot/request/lease control flow."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
from sensor_msgs.msg import JointState
from luggage_msgs.msg import DetectedLuggage,SuctionTopDomainV2
from luggage_msgs.srv import PrepareJointOperationV2
from uld_occ_observer.srv import GetSnapshot,GetValidity,ReleaseSnapshot


def driver_module():
    path=Path(__file__).parents[1]/'scripts/joint_operation_sim_driver.py'
    spec=importlib.util.spec_from_file_location('operation_client_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('damage',[None,'model','scene','empty','shape','nan','invalid'])
def test_explicit_observation_views_are_model_bound_goals(tmp_path,damage):
    module=driver_module()
    launch=dict(model_sha256='model',scene_revision='scene')
    record=dict(schema='diagnostic_observation_poses_v1',**launch,
        poses={'measured_geometry_goal':dict(valid=True,values=[0.]*6)})
    if damage=='model':record['model_sha256']='other'
    elif damage=='scene':record['scene_revision']='other'
    elif damage=='empty':record['poses']={}
    elif damage=='shape':record['poses']['measured_geometry_goal']['values']=[0.]*5
    elif damage=='nan':record['poses']['measured_geometry_goal']['values'][0]=float('nan')
    elif damage=='invalid':record['poses']['measured_geometry_goal']['valid']=False
    path=tmp_path/'views.json';path.write_text(json.dumps(record))
    if damage:
        with pytest.raises(ValueError,match='OBSERVATION_VIEW'):module.load_diagnostic_views(path,launch)
    else:
        assert module.load_diagnostic_views(path,launch)==record['poses']
        assert module.load_diagnostic_views(None,launch)=={}


@pytest.mark.parametrize('success',[True,False])
def test_prepare_binds_repository_identity_and_releases_lease_on_refusal(monkeypatch,tmp_path,success):
    module=driver_module()
    detection=DetectedLuggage(id='measured')
    domain=SuctionTopDomainV2(generation=3,domain_sha256='observed')
    monkeypatch.setattr(module,'wait_observed_detection',lambda *a,**k:(detection,domain))
    lease=GetSnapshot.Response(status='OK',handle='2:7')
    lease.identity.epoch=2;lease.identity.map_revision=5;lease.identity.geometry_hash='current-geometry'
    lease.validity.epoch=2;lease.validity.generation=4;lease.validity.state='VALID'
    transitions=GetValidity.Response(epoch=2,map_revision=1,geometry_hash='old')
    requests=[]
    def call(client,request,name,**kwargs):
        requests.append((client,request))
        if client=='validity':return transitions
        if client=='snapshot':
            assert request.expected_epoch==2 and not request.exact_revision
            assert request.max_age_ms==1000 and not request.allow_stale
            return lease
        if client=='release':return ReleaseSnapshot.Response(status='OK')
        assert client=='prepare'
        assert request.expected_map_revision==5 and request.geometry_hash=='current-geometry'
        return PrepareJointOperationV2.Response(success=success,reason='diagnostic',trace_json='[]')
    probe=SimpleNamespace(detect='detect',domain=domain,validity='validity',snapshot='snapshot',
        release_snapshot='release',prepare='prepare',call=call,record=lambda *a:None,
        joint=JointState(name=module.JOINTS,position=[0.]*6),images={},
        launch={'model_sha256':'model','resolved':{'operation_prepare_budget_s':300.}},
        folder=tmp_path)
    probe.prepare_from_observation=lambda *a,**kw:module.OperationSimDriver.prepare_from_observation(probe,*a,**kw)
    if success:module.OperationSimDriver.plan(probe)
    else:
        with pytest.raises(RuntimeError,match='PREPARE:diagnostic'):
            module.OperationSimDriver.plan(probe)
    assert requests[-1][0]=='release' and requests[-1][1].handle=='2:7'


@pytest.mark.parametrize('replan',[False,True])
@pytest.mark.parametrize('capture',[False,True])
def test_explicit_start_declares_empty_without_mandatory_portal_motion(tmp_path,replan,capture):
    module=driver_module()
    from luggage_msgs.msg import JointOperationPlanV2
    from test_joint_operation_observer import configuration
    config,geometry,config_file=configuration(tmp_path)
    calls=[];poses=[];actions=[];capture_events=[]
    def call(client,request,name,*args):
        calls.append((client,request))
        if client=='validity':return SimpleNamespace(epoch=1)
        if client=='initialize':
            assert capture_events==(['trigger'] if capture else [])
            assert request.declared_by=='explicit_operator_start'
            assert request.geometry_hash==config.descriptor()['geometry_hash']
            return SimpleNamespace(status='OK')
        if client=='spawn':return SimpleNamespace(success=True)
        raise AssertionError(client)
    def action(*args):
        actions.append(args)
        refused=replan and len(actions)==1
        return SimpleNamespace(result=SimpleNamespace(success=not refused,outcome='STOPPED',
            motion_terminal=True,may_have_payload=False,reason='BOUND_NATIVE_ADMISSION_REQUIRED',
            final_map_source=SimpleNamespace(epoch=2,map_revision=len(actions))))
    probe=SimpleNamespace(args=SimpleNamespace(start=True,views=None,prepick_replans=int(replan),capture_operation=capture),
        failure_capture=SimpleNamespace(dropped=0,trigger=lambda:capture_events.append('trigger'),close=lambda:capture_events.append('close')),
        extra_views={},folder=tmp_path,completed_boxes=[],
        launch={'resolved':{'observe_pose_name':'pickup'},'map_owner':{
            'geometry_file':str(geometry),'config_file':str(config_file)}},
        validity='validity',initialize='initialize',spawn='spawn',execute='execute',call=call,
        readiness=lambda:None,observe_pose=lambda p,**kwargs:poses.append(p),
        return_to_pick_observation=lambda plan:poses.append('retry_pick'),
        stage=lambda name,f:f(),plan=lambda **kw:(JointOperationPlanV2(),DetectedLuggage(),SuctionTopDomainV2(instance_id='observed-instance')),
        action=action,execute_prepared=lambda plan,detection,domain,name:action('execute',
            module.ExecuteJointOperationV2.Goal(plan=plan,detection=detection,top_domain=domain),name,610.),
        flush=lambda **kwargs:None,capture_failure_state=lambda:None,domain=None,images={})
    assert module.OperationSimDriver.run(probe)==0
    assert poses==(['pickup','retry_pick'] if replan else ['pickup'])
    assert [c for c,_ in calls]==['validity','initialize','spawn']
    assert len(actions)==1+int(replan)
    assert all(a[1].top_domain.instance_id=='observed-instance' for a in actions)


@pytest.mark.parametrize('field,value',[
    (None,None),('success',True),('outcome','CARRY_FAULT'),('motion_terminal',False),
    ('may_have_payload',True),('reason','CONTROLLER_TIMEOUT')])
def test_preseal_replan_refuses_uncertain_or_carrying_results(field,value):
    module=driver_module()
    fields=dict(success=False,outcome='STOPPED',motion_terminal=True,
                may_have_payload=False,reason='BOUND_NATIVE_ADMISSION_REQUIRED')
    if field:fields[field]=value
    assert module.can_reprepare_before_pick(SimpleNamespace(**fields)) is (field is None)


@pytest.mark.parametrize('failure',[False,True])
def test_return_view_includes_actual_unknown_and_occupied_o_until_terminal(tmp_path,monkeypatch,failure):
    from test_joint_operation_observer import configuration,snapshot
    from moveit_msgs.msg import PlanningScene,CollisionObject
    module=driver_module();config,_,_=configuration(tmp_path)
    states=[1]*64;states[0]=2;states[63]=0
    captured=snapshot(config,states);effects=[]
    baseline=PlanningScene();baseline.world.collision_objects=[CollisionObject(id='unrelated-static')]
    def call(client,request,name):
        if client=='validity':return SimpleNamespace(epoch=1)
        assert client=='apply'
        effects.append(request.scene)
        return SimpleNamespace(success=True)
    def observe(name,**kwargs):
        assert len(effects)==1
        assert len(effects[0].world.collision_objects)==3
        assert all(o.operation==CollisionObject.ADD for o in effects[0].world.collision_objects)
        if failure:raise TimeoutError('unresolved motion')
    monkeypatch.setattr(module,'load_scene_tf_config',lambda _:None)
    monkeypatch.setattr(module,'world_transform',lambda *args:((0.,0.,0.),(0.,0.,0.)))
    probe=SimpleNamespace(validity='validity',apply_scene='apply',call=call,
        observer=SimpleNamespace(capture=lambda **kwargs:captured),
        launch={'scene_revision':'scene','model_sha256':'model',
            'resolved':{'scene_tf_config':'site','observe_pose_name':'pickup'},
            'collision_policy':{'collision_margin_m':.02,'vertical_collision_margin_m':0.}},
        read_scene=lambda _:SimpleNamespace(scene=baseline),observe_pose=observe)
    from luggage_msgs.msg import JointOperationPlanV2
    plan=JointOperationPlanV2()
    plan.predicted_attachment.measured_dimensions_wdh=[.2,.3,.4]
    if failure:
        with pytest.raises(TimeoutError):module.OperationSimDriver.return_to_pick_observation(probe,plan)
        assert len(effects)==1
    else:
        module.OperationSimDriver.return_to_pick_observation(probe,plan)
        assert len(effects)==2
        assert {o.id for o in effects[0].world.collision_objects}=={o.id for o in effects[1].world.collision_objects}
        assert all(o.operation==CollisionObject.REMOVE for o in effects[1].world.collision_objects)
    assert [o.id for o in baseline.world.collision_objects]==['unrelated-static']


@pytest.mark.parametrize('require_uld',[False,True])
def test_pickup_view_does_not_require_an_unrelated_uld_hit(tmp_path,monkeypatch,require_uld):
    module=driver_module()
    poses=tmp_path/'poses.yaml'
    poses.write_text('poses:\n  pickup:\n    valid: true\n    values: [0, 0, 0, 0, 0, 0]\n')
    monkeypatch.setattr(module.time,'sleep',lambda _:None)
    probe=SimpleNamespace(launch={'resolved':{'robot_poses_config':str(poses)}},extra_views={},
        move='move',validity='validity',images={},joint=JointState(),
        read_scene=lambda _:None,record=lambda *a:None,
        action=lambda *a:SimpleNamespace(result=SimpleNamespace(error_code=SimpleNamespace(val=1))),
        call=lambda *a:SimpleNamespace(last_integrated_stamp_ns=0))
    if require_uld:
        with pytest.raises(RuntimeError,match='NO_INTEGRATED_SENSOR_VIEW'):
            module.OperationSimDriver.observe_pose(probe,'pickup',require_uld_observation=True)
    else:
        module.OperationSimDriver.observe_pose(probe,'pickup',require_uld_observation=False)


@pytest.mark.parametrize('reason,expected',[
    ('EXECUTE:CARRY_FAULT:CURRENT_SOURCE_REFUSED',['trigger',25.,'close']),
    ('EXECUTE:STOPPED:BOUND_NATIVE_ADMISSION_REQUIRED',['trigger',25.,'close']),
    ('EXECUTE:STOPPED:CONTROLLER_TIMEOUT',['close'])])
def test_terminal_refusal_capture_runs_at_outer_operation_boundary(monkeypatch,reason,expected):
    module=driver_module();events=[]
    monkeypatch.setattr(module.time,'sleep',events.append)
    def fail():raise RuntimeError(reason)
    probe=SimpleNamespace(args=SimpleNamespace(failure_hold_s=25.),readiness=fail,
        stage=lambda name,fn:fn(),failure_capture=SimpleNamespace(
            trigger=lambda:events.append('trigger'),close=lambda:events.append('close')),
        sensor_diagnostics={},domain=None,images={},joint=None,flush=lambda **kwargs:None,
        capture_failure_state=lambda:events.append('snapshot'))
    assert module.OperationSimDriver.run(probe)==1
    assert events==expected+['snapshot']


def test_late_evidence_flush_preserves_terminal_outcome(tmp_path):
    import threading
    module=driver_module()
    owner=SimpleNamespace(_record_lock=threading.RLock(),folder=tmp_path,records=[],stages=[],completed_boxes=[])
    module.OperationSimDriver.flush(owner,outcome='INCOMPLETE',reason='captured_failure')
    owner.records.append(dict(file='late_lease_release',bytes=12))
    module.OperationSimDriver.flush(owner)
    result=json.loads((tmp_path/'manifest.json').read_text())
    assert result['outcome']=='INCOMPLETE' and result['reason']=='captured_failure'
    assert result['records'][0]['file']=='late_lease_release'


def test_operation_source_capture_starts_with_first_action_feedback():
    module=driver_module();events=[]
    handle=SimpleNamespace(accepted=True,get_result_async=lambda:'terminal')
    def send(goal,**options):
        callback=options['feedback_callback']
        callback(SimpleNamespace(feedback=SimpleNamespace(next_leg_index=0,stage="PREPARE")))
        return handle
    probe=SimpleNamespace(args=SimpleNamespace(capture_operation=True),
        failure_capture=SimpleNamespace(trigger=lambda:events.append('operation_capture')),
        record=lambda *a:None,future=lambda value,*a:value)
    client=SimpleNamespace(wait_for_server=lambda **k:True,send_goal_async=send)
    assert module.OperationSimDriver.action(probe,client,object(),'execute',10.)=='terminal'
    assert events==['operation_capture']


@pytest.mark.parametrize('budget,admitted',[(200.,False),(273.,False),(300.,True),(600.,True)])
def test_explicit_budget_cannot_be_shorter_than_retained_motion(budget,admitted):
    from luggage_msgs.msg import JointOperationPlanV2,JointOperationLegV2
    from trajectory_msgs.msg import JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    module=driver_module();plan=JointOperationPlanV2();events=[]
    for seconds in (10,3,3,253,2,2):
        leg=JointOperationLegV2();leg.trajectory.trajectory.points=[
            JointTrajectoryPoint(time_from_start=Duration(sec=seconds))];plan.legs.append(leg)
    owner=SimpleNamespace(launch={'resolved':{'operation_execution_budget_s':budget}},
        record=lambda *args:events.append(('evidence',args)),execute='execute',
        action=lambda client,goal,name,timeout:events.append(('motion',goal,timeout)))
    args=(owner,plan,DetectedLuggage(),SuctionTopDomainV2(),'execute')
    if admitted:
        module.OperationSimDriver.execute_prepared(*args)
        assert events[-1][0]=='motion' and events[-1][1].budget_s==budget
        assert events[-1][2]==budget+10.
        evidence=json.loads(events[0][1][1]);assert evidence['trajectory_time_s']==273.
    else:
        with pytest.raises(ValueError,match='TRAJECTORY_TIME_EXCEEDS'):
            module.OperationSimDriver.execute_prepared(*args)
        assert not events


def test_failure_scene_error_preserves_cause_and_still_captures_latest_o():
    module=driver_module();events=[]
    def scene(name):raise TimeoutError('scene_unavailable')
    def capture(**kwargs):
        assert kwargs['expected_epoch']==4 and not kwargs['require_latest']
        events.append('latest_o')
    owner=SimpleNamespace(read_scene=scene,observer=SimpleNamespace(capture=capture),
        launch={'scene_revision':'frozen'},validity='validity',
        call=lambda *args:SimpleNamespace(epoch=4),record=lambda *args:events.append(args))
    failures=module.OperationSimDriver.capture_failure_state(owner)
    assert failures==['planning_scene'] and events[-1]=='latest_o'
    assert 'scene_unavailable' in events[0][1].decode()


@pytest.mark.parametrize('changed_epoch',[False,True])
def test_three_cycles_keep_observed_map_and_finalize_before_next_spawn(tmp_path,monkeypatch,changed_epoch):
    module=driver_module()
    from test_joint_operation_observer import configuration
    from luggage_msgs.msg import JointOperationPlanV2
    config,geometry,config_file=configuration(tmp_path)
    events=[];outputs=[]
    def call(client,request,name,*args):
        events.append(client)
        if client=='validity':return SimpleNamespace(epoch=1)
        if client=='initialize':return SimpleNamespace(status='OK')
        if client=='finalize':return SimpleNamespace(success=True)
        raise AssertionError(client)
    def cycle(driver,*,task_mode,prefix):
        index=len(outputs);events.append(('cycle',task_mode))
        result=SimpleNamespace(final_map_source=SimpleNamespace(epoch=3 if changed_epoch and index else 2,map_revision=10+index))
        outputs.append(result)
        return JointOperationPlanV2(plan_digest='plan'+str(index)),result
    monkeypatch.setattr(module,'run_observed_cycle',cycle)
    probe=SimpleNamespace(args=SimpleNamespace(start=True,views=None,three_box=True,failure_hold_s=0.),
        failure_capture=None,extra_views={},folder=tmp_path,completed_boxes=[],stages=[],
        launch={'resolved':{'observe_pose_name':'pickup'},'map_owner':{'geometry_file':str(geometry),'config_file':str(config_file)}},
        validity='validity',initialize='initialize',finalize='finalize',call=call,readiness=lambda:None,
        observe_pose=lambda *a,**k:None,stage=lambda name,f:f(),rotate_capture=lambda:events.append('rotate'),
        return_to_pick_observation=lambda plan,**kw:events.append(('return',kw['pickup_present'])),
        flush=lambda **kw:events.append(('flush',kw)),capture_failure_state=lambda:[],
        sensor_diagnostics={},domain=None,images={},joint=None)
    assert module.OperationSimDriver.run(probe)==(1 if changed_epoch else 0)
    assert events.count('initialize')==1
    if changed_epoch:
        assert len(outputs)==2 and events[-1][1]['outcome']=='INCOMPLETE'
    else:
        assert [x for x in events if isinstance(x,tuple) and x[0]=='cycle']==[
            ('cycle','floor'),('cycle','floor'),('cycle','stack')]
        assert events.count('finalize')==3
        assert events.count(('return',False))==2
        assert events[-1][1]['outcome']=='THREE_BOX_PASS'
