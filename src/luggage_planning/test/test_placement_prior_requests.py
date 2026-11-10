"""Wire-level semantics: generated messages and existing strict frame admission."""
from dataclasses import replace
import json
import pytest
from geometry_msgs.msg import Pose
from luggage_msgs.srv import PreparePlacementPriorV2
from luggage_planning.place_service import pose_message, _admit_grasp
from luggage_planning.placement_prior_contracts import Snapshot, Transform
from luggage_planning.placement_prior_surface import ConfirmedSurface
from luggage_planning.placement_prior_requests import decode_request, encode_choices
from luggage_planning.placement_prior import compute_prior, CollisionVerdict

I=Transform((0,0,0),(0,0,0,1))


def slot_record():
    return dict(contract_version=2,kind='pose',frame_id='container_link',length_unit='m',
        stamp_ns=0,time_basis='scene',scene_revision='cal',request_id='prior',
        reference='box_center',orientation_semantics='box_axis_pi',position=[1.,2.,.5],
        quaternion_xyzw=[0.,0.,0.,1.])


def query():
    msg=PreparePlacementPriorV2.Request()
    msg.request_id='prior';msg.measured=False;msg.geometry_hash='hull';msg.expected_map_revision=3
    msg.template_box_wdh=[.4,.3,.2];grasp=Pose();grasp.position.z=.1;grasp.orientation.x=1.;grasp.orientation.w=0.
    msg.template_box_from_tcp=[grasp];msg.slots=[pose_message(slot_record())]
    msg.max_slots=3;msg.candidate_limit=32;msg.budget_s=2.
    return msg


def test_template_wire_roundtrip_and_existing_grasp_consumer():
    from rclpy.serialization import serialize_message,deserialize_message
    msg=deserialize_message(serialize_message(query()),PreparePlacementPriorV2.Request)
    snap=Snapshot('cal',3,'hull','model',I,b'occ',b'scene')
    confirmed=ConfirmedSurface(b'{}',b'occ',b'{}')
    request=decode_request(msg,snap,confirmed)
    assert not request.measured
    class Clear:
        def check(self,*args,**kwargs):return CollisionVerdict(True,'CLEAR')
    result=compute_prior(request,Clear())
    choices=encode_choices(result,request)
    grasp=_admit_grasp(choices[0].grasp,'prior','cal')
    assert grasp['position']==[0.,0.,.1]
    assert grasp['quaternion_xyzw']==[1.,0.,0.,0.]
    assert choices[0].slot.pose.position.x==1.
    assert result.adopt(request)==()  # hypothetical is never execution authority


def test_reject_mismatched_map_and_wrong_slot_coordinate_frame():
    snap=Snapshot('cal',3,'hull','model',I,b'occ',b'scene')
    confirmed=ConfirmedSurface(b'{}',b'occ',b'{}');msg=query()
    msg.expected_map_revision=2
    with pytest.raises(ValueError,match='snapshot'):decode_request(msg,snap,confirmed)
    msg.expected_map_revision=3;msg.slots[0].identity.frame_id='world'
    with pytest.raises(ValueError,match='FRAME'):decode_request(msg,snap,confirmed)


def confirmed_grid(*, floor=.25, unknown=False):
    from luggage_description.container_geometry import cuboid_from_inner_size
    descriptor = cuboid_from_inner_size((1., 1., 1.), floor_z=floor).descriptor()
    record = dict(kind='surface', frame_id='container_link', height_datum='container_origin',
        geometry_hash=descriptor['geometry_hash'], map_revision=3,
        floor_z_container=floor, ceiling_z_container=floor+1.,
        resolution_m=.1, origin_xy_container=[-.5, -.5],
        surface_z_container=[[None if unknown else floor]*10 for _ in range(10)],
        state=[['unknown' if unknown else 'free']*10 for _ in range(10)],
        confidence=[['unknown' if unknown else 'sensor']*10 for _ in range(10)])
    # Old message datum fields are deliberately wrong: only its hull metadata
    # may enter candidate generation, never its stale geometric grid values.
    metadata = dict(inner_size=[1.,1.,1.], geometry_descriptor=descriptor,
                    center_base=[99.,99.,99.], height=[[99.]*10 for _ in range(10)])
    return ConfirmedSurface(json.dumps(record).encode(), b'occ', json.dumps(metadata).encode())


def test_generated_slots_use_typed_height_datum():
    from luggage_planning.placement_prior_requests import enumerate_placements
    slots = enumerate_placements(confirmed_grid(), (.2,.2,.2), 3)
    assert 0 < len(slots) <= 3
    for slot in slots:
        x,y,z = slot.container_from_box.translation
        assert -.5 < x < .5 and -.5 < y < .5
        assert z == pytest.approx(.35)
    # The shared solver permits floor-prior candidates in unobserved cells.
    # These are geometric proposals only: the private collision scene treats
    # unknown columns as blocked before a prior choice can be admitted.
    unknown_slots = enumerate_placements(confirmed_grid(unknown=True), (.2,.2,.2), 3)
    assert all(slot.container_from_box.translation[2] == pytest.approx(.35)
               for slot in unknown_slots)


def test_generated_slots_refuse_missing_hull_and_shifted_grid():
    from luggage_planning.placement_prior_requests import enumerate_placements
    confirmed = confirmed_grid()
    with pytest.raises(ValueError, match='geometry_descriptor'):
        enumerate_placements(replace(confirmed, confirmation_json=b'{"inner_size":[1,1,1]}'),
                             (.2,.2,.2), 3)
    record = confirmed.record()
    record['origin_xy_container'][0] += .1
    with pytest.raises(ValueError, match='centered'):
        enumerate_placements(replace(confirmed, record_json=json.dumps(record).encode()),
                             (.2,.2,.2), 3)
