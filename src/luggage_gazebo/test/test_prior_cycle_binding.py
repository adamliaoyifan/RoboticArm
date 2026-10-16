"""Measured prior binding in the existing full-cycle eval driver."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from motion_occ_eval_driver import MotionOccDriver
from luggage_msgs.msg import DetectedLuggage, SuctionCandidate


def measured_box():
    box=DetectedLuggage()
    box.id='box';box.header.frame_id='world';box.header.stamp.sec=1
    box.pose.orientation.w=1.;box.pose.position.z=.5
    box.width=.4;box.depth=.3;box.height=.2
    box.height_valid=True;box.top_surface_valid=True
    # Use the actual production constant rather than a guessed wire enum.
    from luggage_planning.pick_authorization import HEIGHT_SOURCE_MEASURED_SUPPORT
    box.height_source=HEIGHT_SOURCE_MEASURED_SUPPORT
    for rank,name in enumerate(('centre','offset'),1):
        candidate=SuctionCandidate()
        candidate.header=deepcopy(box.header);candidate.instance_id='box'
        candidate.generation=1;candidate.candidate_id=name;candidate.rank=rank
        candidate.contact_pose.orientation.w=1.
        candidate.contact_pose.position.z=.6;candidate.contact_pose.position.x=.02*(rank-1)
        candidate.model_hash='cup';candidate.model_version=1
        box.suction_candidates.append(candidate)
    return box


def driver(client):
    instance=object.__new__(MotionOccDriver)
    instance._placement_prior_client=client
    instance._frame_request_seq=0;instance._frame_scene_revision='scene'
    instance._typed=SimpleNamespace(set_placement_prior=lambda value:None)
    return instance


def test_adopted_candidate_preserves_measured_contact_and_rank():
    box=measured_box();original=deepcopy(box.suction_candidates[1])
    choice=SimpleNamespace(candidate_id='offset',slot_id='target')
    response=SimpleNamespace(elapsed_s=.1,cache_hit=True)
    instance=driver(SimpleNamespace(prepare=lambda *args:response,
        bind=lambda result,view,revision:(view,choice)))
    trial=SimpleNamespace(extras={})
    assert instance.on_detected_luggage(box,trial,None)==''
    assert box.suction_candidates==[original]
    assert instance._placement_prior_choice is choice
    assert trial.extras['placement_prior']['adopted']


def test_prior_miss_retains_original_candidates_and_records_fallback():
    box=measured_box();before=deepcopy(box)
    def missing(*args):raise ValueError('no confirmed snapshot')
    instance=driver(SimpleNamespace(prepare=missing))
    trial=SimpleNamespace(extras={})
    assert instance.on_detected_luggage(box,trial,None)==''
    assert box==before
    assert instance._placement_prior_choice is None
    assert not trial.extras['placement_prior']['adopted']


def test_acquisition_only_stops_at_detection_even_on_prior_miss():
    box=measured_box()
    def unavailable(*args):raise ValueError('no confirmed snapshot')
    instance=driver(SimpleNamespace(prepare=unavailable))
    instance._args=SimpleNamespace(prior_acquisition_only=True)
    trial=SimpleNamespace(extras={})
    assert instance.on_detected_luggage(box,trial,None)=='PRIOR_INPUT_ACQUISITION_COMPLETE'
    assert not trial.extras['placement_prior']['adopted']


def test_bootstrap_uses_certified_named_motion_before_observation_and_return():
    instance=object.__new__(MotionOccDriver);calls=[]
    instance._args=SimpleNamespace(goto_timeout=60.)
    instance._frame_scene_revision="test-scene"
    instance._goto_named_pose_v2=lambda name,**kw: calls.append(('goto',name)) or (True,'')
    instance._depth_observer=SimpleNamespace(observe_once=lambda name:calls.append(('observe',name)) or {'view':name})
    instance.goto_observe=lambda:calls.append(('return',None)) or (True,'',None)
    views=instance.bootstrap_depth_map()
    assert len(views)==3
    assert calls[0]==('goto','portal_observe_sim') and calls[-1]==('return',None)
    assert [kind for kind,_ in calls]==['goto','observe','observe','observe','return']


def test_bootstrap_refuses_observation_if_certified_route_fails():
    import pytest
    instance=object.__new__(MotionOccDriver);calls=[]
    instance._args=SimpleNamespace(goto_timeout=60.)
    instance._frame_scene_revision="test-scene"
    instance._goto_named_pose_v2=lambda *args,**kw:(False,'NO_ROUTE')
    instance._depth_observer=SimpleNamespace(observe_once=calls.append)
    with pytest.raises(RuntimeError,match='BOOTSTRAP_OBSERVE_ROUTE'):
        instance.bootstrap_depth_map()
    assert not calls


def test_raw_service_boundaries_include_named_motion_requests():
    from concurrent.futures import Future
    from luggage_msgs.srv import GoToRobotPoseV2
    instance=object.__new__(MotionOccDriver);captured=[]
    instance._service_boundary_recorder=lambda boundary,message:captured.append((boundary,deepcopy(message)))
    request=GoToRobotPoseV2.Request();request.pose_name='portal_observe_sim'
    response=GoToRobotPoseV2.Response();response.success=False;response.reason_code='NO_ROUTE'
    future=Future();future.set_result(response)
    client=SimpleNamespace(srv_name='/goto',wait_for_service=lambda **kw:True,call_async=lambda req:future)
    assert instance.call_srv(client,request,1.) is response
    assert captured==[('/goto:request',request),('/goto:response',response)]


def test_confirmed_attachment_survives_later_tcp_capture_and_clears_on_release():
    from pick_retreat_eval_driver import PickRetreatEvalDriver
    from luggage_msgs.srv import VacuumCommandV2
    from luggage_planning.typed_driver_adapter import grasp_record
    from luggage_planning.place_service import pose_message
    instance=object.__new__(PickRetreatEvalDriver)
    instance._frame_scene_revision='scene'
    instance._frame_box_identity=lambda:('box',1)
    instance._frame_request_id=lambda name:'attach'
    response=VacuumCommandV2.Response(success=True,has_measured_attachment=True)
    response.measured_grasp=pose_message(grasp_record('attach','scene',
        [.02,.03,.1],[1.,0.,0.,0.],stamp_ns=1000))
    response.attached_box_wdh=[.4,.3,.2]
    instance._typed=SimpleNamespace(vacuum_command=lambda **kw:response)
    instance._frame_capture_grasp=lambda box:setattr(instance,'_frame_grasp',None)
    assert instance.vacuum_command(True,measured_box())[0]
    frozen=deepcopy(instance._frame_grasp)
    # No TF buffer exists: a recapture would fail rather than quietly pass.
    PickRetreatEvalDriver._frame_capture_grasp(instance,measured_box())
    assert instance._frame_grasp==frozen
    assert instance._frame_grasp_stamp_ns==1000
    assert instance.vacuum_command(False,measured_box())[0]
    assert instance._frame_grasp is None
    assert not instance._frame_attachment_confirmed


def test_successful_vacuum_without_attachment_cannot_authorize_place():
    from pick_retreat_eval_driver import PickRetreatEvalDriver
    from luggage_msgs.srv import VacuumCommandV2
    instance=object.__new__(PickRetreatEvalDriver)
    instance._frame_scene_revision='scene'
    instance._frame_box_identity=lambda:('box',1)
    instance._frame_request_id=lambda name:'attach'
    instance._typed=SimpleNamespace(vacuum_command=lambda **kw:VacuumCommandV2.Response(success=True))
    instance._frame_capture_grasp=lambda box:None
    assert instance.vacuum_command(True,measured_box())==(False,'MEASURED_ATTACHMENT_MISSING')
    assert instance._frame_grasp is None


def test_release_pose_query_preserves_timeout_output(monkeypatch):
    import subprocess
    import place_smoke_driver as module
    instance=object.__new__(module.PlaceSmokeDriver);instance._box_model='box'
    monkeypatch.setattr(module,'_ign_model_bin',lambda:'ign')
    def timeout(*args,**kwargs):
        raise subprocess.TimeoutExpired(args[0],5.,output=b'waiting for world')
    monkeypatch.setattr(module.subprocess,'check_output',timeout)
    assert instance._gz_box_pose() is None
    assert instance._last_gz_pose_probe['error']=='TimeoutExpired'
    assert instance._last_gz_pose_probe['output']=='waiting for world'
    assert instance._last_gz_pose_probe['command']==['ign','model','-m','box','--pose']


def test_release_pose_query_preserves_unparseable_output(monkeypatch):
    import place_smoke_driver as module
    instance=object.__new__(module.PlaceSmokeDriver);instance._box_model='box'
    monkeypatch.setattr(module,'_ign_model_bin',lambda:'ign')
    monkeypatch.setattr(module.subprocess,'check_output',lambda *a,**kw:'world missing')
    assert instance._gz_box_pose() is None
    assert instance._last_gz_pose_probe['output']=='world missing'
    assert instance._last_gz_pose_probe['parsed'] is None


def test_readiness_failure_saves_the_frozen_frame_without_resampling(monkeypatch,tmp_path):
    import numpy as np
    import place_smoke_driver as module
    instance=object.__new__(module.PlaceSmokeDriver)
    instance._args=SimpleNamespace(dump_dir=str(tmp_path))
    decision={'image_stamp':12.3,'detection_count':0}
    calls=[]
    instance.freeze_failure_bundle=lambda record:(calls.append(('freeze',record)) or
        dict(capture_complete=False,missing=['cargo'],replay_possible=False))
    rgb=np.zeros((2,2,3),dtype=np.uint8);depth=np.ones((2,2),dtype=np.float32)
    instance.decoded_dump=lambda:(calls.append(('decode',None)) or
        ({'color':rgb},{'join_stamp':12.3},{'depth_m':depth}))
    def write(directory,fields,**kwargs):
        calls.append(('write',directory))
        assert kwargs['images']['color'] is rgb
        assert kwargs['arrays']['depth_m'] is depth
        assert kwargs['extras']['decision_record'] is decision
        assert kwargs['extras']['missing']==['cargo']
        return directory+'/trial'
    monkeypatch.setattr(module,'write_trial_dump',write)
    trial=module.PlaceTrial(index=0);trial.fail_code='YOLO_NOT_READY';trial.extras={}
    instance._capture_pick_readiness_failure(trial,decision)
    assert [item[0] for item in calls]==['freeze','decode','write']
    assert not trial.extras['perception_failure_capture']['capture_complete']
