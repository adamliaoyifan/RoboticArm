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
