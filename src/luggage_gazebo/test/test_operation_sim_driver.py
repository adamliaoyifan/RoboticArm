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
        joint=JointState(name=module.JOINTS,position=[0.]*6),
        launch={'model_sha256':'model','resolved':{'operation_prepare_budget_s':300.}},
        folder=tmp_path)
    probe.prepare_from_observation=lambda *a:module.OperationSimDriver.prepare_from_observation(probe,*a)
    if success:module.OperationSimDriver.plan(probe)
    else:
        with pytest.raises(RuntimeError,match='PREPARE:diagnostic'):
            module.OperationSimDriver.plan(probe)
    assert requests[-1][0]=='release' and requests[-1][1].handle=='2:7'


def test_explicit_start_declares_empty_without_mandatory_portal_motion(tmp_path):
    module=driver_module()
    from luggage_msgs.msg import JointOperationPlanV2
    from test_joint_operation_observer import configuration
    config,geometry,config_file=configuration(tmp_path)
    calls=[];poses=[]
    def call(client,request,name,*args):
        calls.append((client,request))
        if client=='validity':return SimpleNamespace(epoch=1)
        if client=='initialize':
            assert request.declared_by=='explicit_operator_start'
            assert request.geometry_hash==config.descriptor()['geometry_hash']
            return SimpleNamespace(status='OK')
        if client=='spawn':return SimpleNamespace(success=True)
        raise AssertionError(client)
    probe=SimpleNamespace(args=SimpleNamespace(start=True,views=None),extra_views={},folder=tmp_path,
        launch={'resolved':{'observe_pose_name':'pickup'},'map_owner':{
            'geometry_file':str(geometry),'config_file':str(config_file)}},
        validity='validity',initialize='initialize',spawn='spawn',execute='execute',call=call,
        readiness=lambda:None,observe_pose=lambda p,**kwargs:poses.append(p),
        stage=lambda name,f:f(),plan=lambda:(JointOperationPlanV2(),DetectedLuggage(),SuctionTopDomainV2()),
        action=lambda *a:SimpleNamespace(result=SimpleNamespace(success=True)),
        flush=lambda **kwargs:None,domain=None,images={})
    assert module.OperationSimDriver.run(probe)==0
    assert poses==['pickup']
    assert [c for c,_ in calls]==['validity','initialize','spawn']


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
    ('EXECUTE:STOPPED:CONTROLLER_TIMEOUT',[])])
def test_terminal_refusal_capture_runs_at_outer_operation_boundary(monkeypatch,reason,expected):
    module=driver_module();events=[]
    monkeypatch.setattr(module.time,'sleep',events.append)
    def fail():raise RuntimeError(reason)
    probe=SimpleNamespace(args=SimpleNamespace(failure_hold_s=25.),readiness=fail,
        stage=lambda name,fn:fn(),failure_capture=SimpleNamespace(
            trigger=lambda:events.append('trigger'),close=lambda:events.append('close')),
        sensor_diagnostics={},domain=None,images={},joint=None,flush=lambda **kwargs:None)
    assert module.OperationSimDriver.run(probe)==1
    assert events==expected
