from dataclasses import replace
from types import SimpleNamespace
import math
import pytest
from luggage_planning.geometry.pose import Pose, Point, Quaternion
from luggage_planning.placement_prior.pick import candidates_for_prior, bind_choice_to_pick
from luggage_planning.placement_prior.contracts import Transform, PlacementCandidate
from luggage_planning.placement_prior.core import PriorChoice


def detection():
    from luggage_planning.pick.suction_pick_session import DetectionView
    candidate = SimpleNamespace(candidate_id='offset', rank=1, stamp=1.0,
        frame='world', instance_id='box', generation=1, model_hash='cup', model_version=1,
        contact=Pose(Point(1., 2.04, .6), Quaternion()))
    return DetectionView(stamp=1., frame='world', candidates=(candidate,),
        box_xyz=(1., 2., .5), box_quat=(0,0,math.sqrt(.5),math.sqrt(.5)),
        box_size=(.4,.3,.2), stamp_ns=1000000000,
        detection_yaw=math.pi/2, yaw_valid=True)


def test_uses_actual_attach_orientation_not_contact_normal():
    d = detection()
    grasp, = candidates_for_prior(d)
    assert grasp.box_from_tcp.translation == pytest.approx((.04,0,.1))
    # World TCP must face down although perception contact +Z faces up.
    tcp = Transform(d.box_xyz,d.box_quat).compose(grasp.box_from_tcp)
    from luggage_description.frame_transforms import rotate_vector
    assert rotate_vector(tcp.quaternion_xyzw,(0,0,1)) == pytest.approx((0,0,-1))
    assert tcp.translation == pytest.approx((1.,2.04,.6))
    slot=PlacementCandidate('slot',Transform((0,0,0),(0,0,0,1)))
    choice=PriorChoice(slot,grasp,slot.container_from_box.compose(grasp.box_from_tcp),())
    selected=bind_choice_to_pick(choice,d)
    assert selected.candidates[0] is d.candidates[0]
    assert d.candidates[0].rank == 1
    with pytest.raises(ValueError,match='no longer matches'):
        bind_choice_to_pick(choice,replace(d,stamp_ns=d.stamp_ns+1))


def test_reject_changed_candidate_identity():
    d=detection()
    d.candidates[0].frame='camera_link'
    with pytest.raises(ValueError,match='identity mismatch'):
        candidates_for_prior(d)


def test_directed_pi_pick_axes_and_rotated_place_preserve_offset_and_target():
    # Physical contact is 4 cm along the directed box +X axis. At pi yaw
    # it lies on world -X; do not canonicalize the box axes modulo pi.
    d = replace(detection(), box_quat=(0.,0.,1.,0.), detection_yaw=math.pi)
    d.candidates[0].contact.position.x = .96
    d.candidates[0].contact.position.y = 2.
    grasp, = candidates_for_prior(d)
    assert grasp.box_from_tcp.translation == pytest.approx((.04,0.,.1))
    target = Transform((3.,4.,.5),(0.,0.,math.sqrt(.5),math.sqrt(.5)))
    tcp = target.compose(grasp.box_from_tcp)
    assert tcp.translation == pytest.approx((3.,4.04,.6))
    recovered = tcp.compose(grasp.box_from_tcp.inverse())
    assert recovered.translation == pytest.approx((3.,4.,.5))
    assert recovered.quaternion_xyzw == pytest.approx(target.quaternion_xyzw)
