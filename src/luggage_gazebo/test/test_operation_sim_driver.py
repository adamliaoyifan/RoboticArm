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
