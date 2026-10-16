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
