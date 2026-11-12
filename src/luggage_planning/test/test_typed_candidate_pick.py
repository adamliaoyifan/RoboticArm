"""Selected contact survives the real typed client/service/waypoint encoder."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from luggage_msgs.msg import DetectedLuggage, SuctionCandidate
from luggage_msgs.srv import BuildMotionSequenceV2
from luggage_planning.typed_driver_adapter import TypedDriverAdapter
from luggage_planning.place_service import handle_build_v2
from luggage_planning.ros_message_adapters import pick_from_detected


def request():
    box = DetectedLuggage()
    box.id = 'box'; box.header.frame_id = 'world'; box.header.stamp.sec = 1
    box.pose.position.x = .5; box.pose.position.y = .1; box.pose.position.z = .4
    box.pose.orientation.w = 1.
    box.width = .5; box.depth = .4; box.height = .32
    box.height_valid = True; box.height_source = 1; box.top_surface_valid = True
    box.top_surface_pose.position.z = .56; box.yaw_valid = True
    candidate = SuctionCandidate()
    candidate.header = box.header; candidate.instance_id = 'box'; candidate.generation = 1
    candidate.candidate_id = 'offset'; candidate.rank = 1
    candidate.model_hash = 'contact-model'; candidate.model_version = 1
    candidate.contact_pose.position.x = .532
    candidate.contact_pose.position.y = .1
    candidate.contact_pose.position.z = .56
    candidate.contact_pose.orientation.w = 1.
    box.suction_candidates = [candidate]
    fake = SimpleNamespace(_build=None, _call=lambda client, value, timeout: value)
    return TypedDriverAdapter.build_pick(fake, request_id='pick', scene_revision='scene', pick_message=box)


def test_contact_reaches_actual_encoder_and_certification(monkeypatch):
    path = Path(__file__).parents[1] / 'scripts/waypoint_generator_node.py'
    spec = importlib.util.spec_from_file_location('candidate_waypoint_node', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    import luggage_planning.routing.producer as producer
    from st09_contracts.fixtures import make_timed_trajectory
    captured = []; certified = []
    def records(waypoints, **kwargs):
        captured.extend(waypoints)
        return [producer._leg_record(name, make_timed_trajectory(),
                kwargs['request_id'], kwargs['scene_revision'], kwargs['stamp_ns'])
                for name, _, _ in waypoints], ('edges',)
    monkeypatch.setattr(producer, 'pick_leg_records', records)
    node = SimpleNamespace(_joints_live=[.1]*6,
        _pick_builder=lambda: dict(graph=None,edge_builder=None,refine=None,fk=None),
        _clearances=lambda: None,
        _certify_pick_legs=lambda *args, **kwargs: certified.append(args))
    wire = request()
    assert wire.suction_candidate_id == 'offset'
    response = handle_build_v2(wire, BuildMotionSequenceV2.Response(),
        pick=pick_from_detected(wire.pick),
        pick_encoder=module.WaypointGeneratorNode._pick_leg_encoder(node))
    assert response.success, response.message
    attach = next(p for name,p,q in captured if name=='attach')
    assert attach == pytest.approx((.532,.1,.56))
    assert attach[0] != wire.pick.pose.position.x
    assert certified
    assert all(segment.segment_type=='joint_path' for segment in response.motion_segments)


@pytest.mark.parametrize('candidate_id', ['', 'missing'])
def test_missing_selection_never_falls_back_to_center(candidate_id):
    wire = request(); wire.suction_candidate_id = candidate_id
    def forbidden(*args, **kwargs):
        pytest.fail('invalid candidate reached encoder')
    response = handle_build_v2(wire, BuildMotionSequenceV2.Response(),
        pick=pick_from_detected(wire.pick), pick_encoder=forbidden)
    assert not response.success
