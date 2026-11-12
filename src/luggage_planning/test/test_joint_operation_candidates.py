"""Target-guided observed contacts joined to actual private endpoint inputs."""
from dataclasses import asdict, replace
import json
import math
import os
from pathlib import Path

import numpy as np
import pytest
from geometry_msgs.msg import Pose
from luggage_msgs.srv import CheckGraspLocalPaths
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.serialization import deserialize_message, serialize_message
from shape_msgs.msg import SolidPrimitive

from luggage_perception.directed_suction_quality import DirectedSuctionContext
from luggage_perception.directed_suction_service import DirectedSuctionService
from luggage_perception.ros_suction_domain import domain_to_message
from luggage_planning.joint_operation.candidates import (
    admit_pickup_association, candidate_for_contact, propose_contact,
)
from luggage_planning.joint_operation.contacts import contact_evidence, contact_query
from luggage_planning.joint_operation.endpoints import build_local_query
from luggage_planning.joint_operation.scenes import validate_phase_payloads
from luggage_planning.placement_prior.contracts import PlacementCandidate, Transform
from luggage_planning.routing.producer import anchor_plan
from luggage_planning.waypoint_generator import PLACE_RELEASE_GAP_M
from st09_contracts.payload_envelope import EnvelopeLimits
from test_directed_suction_service import request_and_context
from test_directed_suction_quality import context_inputs
from test_joint_operation_scenes import baseline


def snapshot():
    original = baseline()
    scene = deserialize_message(original.scene_cdr,PlanningScene)
    scene.world.collision_objects = [o for o in scene.world.collision_objects if o.id != 'pickup_box']
    return replace(original,scene_cdr=serialize_message(scene),
        world_from_container=Transform((.1,-.2,.3),(0.,0.,math.sin(.3),math.cos(.3))))


def placed_pose(yaw):
    return Transform((.8,-.8,.85),(0.,0.,math.sin(yaw/2),math.cos(yaw/2)))


@pytest.fixture
def capture(request):
    root = os.environ.get('JOINT_CANDIDATE_EVIDENCE')
    directory = Path(root)/request.node.name if root else None
    if directory: directory.mkdir(parents=True,exist_ok=False)
    def record(name,value):
        if directory:
            if isinstance(value,bytes): (directory/name).write_bytes(value)
            else: (directory/name).write_text(json.dumps(value,indent=2))
    return record


@pytest.mark.parametrize('yaw', [0.,math.pi/2,math.pi,-math.pi/2])
def test_target_hint_backprojects_to_fully_supported_eccentric_pick(capture,yaw):
    _,context = request_and_context()
    domain,model = context.domain,context._model
    proposal = propose_contact(domain,model,query_id='target',world_from_place_box=placed_pose(yaw),
        desired_world_from_tool=Transform((0.,-.8,1.),(1.,0.,0.,0.)),max_center_checks=64)
    capture('domain.cdr',serialize_message(domain_to_message(domain)))
    capture('proposal.json',asdict(proposal))
    assert proposal.reason == 'OBSERVED_FOOTPRINT_PROPOSAL'
    assert proposal.checked_centres == 1
    # Exact bounding contact can sit off the measurement grid, retaining bias.
    extent = max(abs(v) for v in proposal.center_uv_top_m)
    assert extent > .05
    request,_ = request_and_context()
    query = contact_query(domain,request.detection,[proposal.to_message()],model,budget_s=1.)
    service = DirectedSuctionService(clock=lambda:0.); service.replace_context(context)
    response = service.handle(query,service_response())
    capture('quality_request.cdr',serialize_message(query));capture('quality_response.cdr',serialize_message(response))
    evidence = contact_evidence(response,query,domain,model)
    assert evidence.complete and len(evidence.contacts) == 1, (evidence.reason,evidence.verdicts)
    assert evidence.contacts[0].query_uv == proposal.center_uv_top_m
    assert abs(proposal.center_uv_top_m[0]) < .35 and abs(proposal.center_uv_top_m[1]) < .225


def service_response():
    from luggage_msgs.srv import EvaluateDirectedSuctionV2
    return EvaluateDirectedSuctionV2.Response()


@pytest.mark.parametrize('yaw', [0.,math.pi/2,math.pi,-math.pi/2])
@pytest.mark.parametrize('tilt', [0.,.02])
def test_measured_quality_to_complete_candidate_and_endpoint_cdr(capture,yaw,tilt):
    request,_ = request_and_context()
    domain,hmap,model = context_inputs()
    if tilt:
        domain = replace(domain,v_axis_world=(0.,math.cos(tilt),math.sin(tilt)),
            normal_world=(0.,-math.sin(tilt),math.cos(tilt)))
        hmap.v_axis,hmap.normal = np.array(domain.v_axis_world),np.array(domain.normal_world)
        hmap.local_normal[:] = domain.normal_world
    context = DirectedSuctionContext(domain,hmap,model)
    query = contact_query(domain,request.detection,request.queries[:1],model,budget_s=1.)
    service = DirectedSuctionService(clock=lambda:0.);service.replace_context(context)
    response = service.handle(query,service_response())
    capture('domain.cdr',serialize_message(domain_to_message(domain)))
    capture('quality_request.cdr',serialize_message(query));capture('quality_response.cdr',serialize_message(response))
    evidence = contact_evidence(response,query,domain,model)
    assert evidence.complete and len(evidence.contacts) == 1
    source = snapshot();before = source.scene_cdr
    slot = PlacementCandidate('exact-slot',placed_pose(yaw))
    limits = EnvelopeLimits(5.,(.1,.1,.1),(.01,.02),.005)
    candidate = candidate_for_contact(source,domain,evidence.contacts[0],slot,limits,
        pickup_object_id='pickup_box',observation_id='observed',rank=0)
    assert source.scene_cdr == before
    expected_settled = source.world_from_container.compose(slot.container_from_box)
    assert candidate.world_from_settled_box == expected_settled
    up = np.array(source.world_from_container.compose(Transform((0.,0.,1.),(0.,0.,0.,1.))).translation)-source.world_from_container.translation
    assert np.array(candidate.world_from_release_box.translation)-expected_settled.translation == pytest.approx(up*PLACE_RELEASE_GAP_M)
    world_pick_box = Transform(domain.box_xyz_world,domain.box_quaternion_xyzw)
    G = world_pick_box.inverse().compose(candidate.world_from_pick_tool)
    expected_release_tool = candidate.world_from_release_box.compose(G)
    assert candidate.world_from_release_tool.translation == pytest.approx(expected_release_tool.translation,abs=1e-12)
    assert candidate.world_from_release_tool.quaternion_xyzw == pytest.approx(expected_release_tool.quaternion_xyzw,abs=1e-12)
    assert candidate.predicted_attachment.measured_dimensions_wdh == domain.box_wdh
    assert candidate.predicted_attachment.t_tool_box_translation == pytest.approx(G.inverse().translation)
    # Existing region-ladder geometry, without candidate -> RouteProducer dependency.
    ladder = anchor_plan(candidate.world_from_release_tool.translation,source.world_from_container.quaternion_xyzw,
                         domain.box_wdh[2])
    assert candidate.place_entry.translation == ladder.slot_approach
    assert candidate.place_retreat.translation == ladder.exit
    endpoint = build_local_query(candidate,service=CheckGraspLocalPaths,request_id='prepare',budget_s=1.)
    endpoint = deserialize_message(serialize_message(endpoint),CheckGraspLocalPaths.Request)
    capture('endpoint_request.cdr',serialize_message(endpoint))
    capture('source_scene.cdr',before)
    capture('candidate.json',dict(candidate_id=candidate.candidate_id,geometry=candidate.predicted_attachment.to_dict(),
        settled=asdict(candidate.world_from_settled_box),pick_tool=asdict(candidate.world_from_pick_tool),
        release_tool=asdict(candidate.world_from_release_tool)))
    for box,tool in ((endpoint.pick_box_world,candidate.world_from_pick_tool),
                     (endpoint.place_box_world,candidate.world_from_release_tool)):
        actual = pose_transform(box).compose(pose_transform(endpoint.box_from_tcp))
        assert actual.translation == pytest.approx(tool.translation,abs=1e-12)
        assert abs(sum(a*b for a,b in zip(actual.quaternion_xyzw,tool.quaternion_xyzw))) == pytest.approx(1.)
    decoded = []
    for phase in ('pick','carry','retreat'):
        raw = candidate.scenes.for_phase(phase).scene_cdr
        capture(phase+'_scene.cdr',raw)
        decoded.append(deserialize_message(raw,PlanningScene))
    validate_phase_payloads(decoded,pickup_object_id='pickup_box',geometry=candidate.predicted_attachment,
        world_from_pick_box=world_pick_box,world_from_settled_box=expected_settled)
    assert {obj.id for obj in endpoint.scene.world.collision_objects} == {'pickup_box_neighbor','container','robot_pedestal'}


def pose_transform(pose):
    p,q = pose.position,pose.orientation
    return Transform((p.x,p.y,p.z),(q.x,q.y,q.z,q.w))


def test_geometric_hole_and_cap_never_create_quality_permission(capture):
    _,context = request_and_context()
    domain = replace(context.domain,support=bytes(len(context.domain.support)))
    proposal = propose_contact(domain,context._model,query_id='hole',world_from_place_box=placed_pose(0.),
        desired_world_from_tool=Transform((.8,-.8,1.),(1.,0.,0.,0.)),max_center_checks=1)
    capture('domain.cdr',serialize_message(domain_to_message(domain)));capture('proposal.json',asdict(proposal))
    assert proposal.reason == 'CONTACT_CENTRE_SEARCH_INCOMPLETE' and proposal.checked_centres == 1
    with pytest.raises(ValueError): proposal.to_message()


def test_internal_unknown_cell_is_not_filled_to_satisfy_target(capture):
    _,context = request_and_context()
    support = bytearray(context.domain.support)
    support[70*90+45] = 0
    domain = replace(context.domain,support=bytes(support))
    proposal = propose_contact(domain,context._model,query_id='internal-hole',world_from_place_box=placed_pose(0.),
        desired_world_from_tool=Transform((.8,-.8,1.),(1.,0.,0.,0.)),max_center_checks=1)
    capture('domain.cdr',serialize_message(domain_to_message(domain)));capture('proposal.json',asdict(proposal))
    assert proposal.reason == 'CONTACT_CENTRE_SEARCH_INCOMPLETE'
    assert domain.support[70*90+45] == 0


def test_far_target_does_not_spend_contact_budget_on_impossible_edge_centres():
    domain,_,model=context_inputs()
    observed=domain.cells().copy()
    observed[:2,:]=False;observed[-2:,:]=False
    observed[:,:2]=False;observed[:,-2:]=False
    domain=replace(domain,support=observed.astype(np.uint8).tobytes())
    proposal=propose_contact(domain,model,query_id='far-hint',
        world_from_place_box=Transform(domain.box_xyz_world,domain.box_quaternion_xyzw),
        desired_world_from_tool=Transform((.9,.4,1.),(1.,0.,0.,0.)),max_center_checks=256)
    assert proposal.reason=='OBSERVED_FOOTPRINT_PROPOSAL'
    assert proposal.checked_centres<=256


def test_geometry_proposal_can_still_fail_real_surface_quality(capture):
    request,_ = request_and_context()
    domain,hmap,model = context_inputs()
    # Directly observed support is intact; a measured height discontinuity
    # inside the actual cup invalidates quality independently of containment.
    hmap.median_h[65:75,40:50] = .02
    hmap.smooth_h[65:75,40:50] = .02
    context = DirectedSuctionContext(domain,hmap,model)
    proposal = propose_contact(domain,model,query_id='rough',world_from_place_box=placed_pose(0.),
        desired_world_from_tool=Transform((.8,-.8,1.),(1.,0.,0.,0.)),max_center_checks=1)
    assert proposal.reason == 'OBSERVED_FOOTPRINT_PROPOSAL'
    query = contact_query(domain,request.detection,[proposal.to_message()],model,budget_s=1.)
    service = DirectedSuctionService(clock=lambda:0.);service.replace_context(context)
    response = service.handle(query,service_response())
    capture('domain.cdr',serialize_message(domain_to_message(domain)))
    capture('quality_request.cdr',serialize_message(query));capture('quality_response.cdr',serialize_message(response))
    capture('quality_heights.json',dict(patch_cells=[65,75,40,50],height_m=.02,origin='synthetic measured discontinuity'))
    evidence = contact_evidence(response,query,domain,model)
    assert evidence.complete and not evidence.contacts and evidence.verdicts[0][1] is False


def test_opposite_tool_hint_changes_directed_query_without_equating_it_to_J6():
    _,context = request_and_context()
    common = dict(query_id='direction',world_from_place_box=placed_pose(0.),max_center_checks=8)
    a = propose_contact(context.domain,context._model,
        desired_world_from_tool=Transform((.8,-.8,1.),(1.,0.,0.,0.)),**common)
    b = propose_contact(context.domain,context._model,
        desired_world_from_tool=Transform((.8,-.8,1.),(0.,1.,0.,0.)),**common)
    assert a.reason == b.reason == 'OBSERVED_FOOTPRINT_PROPOSAL'
    assert abs(math.cos(a.yaw_rad-b.yaw_rad)+1.) < 1e-12
    assert a.center_uv_top_m == pytest.approx(b.center_uv_top_m,abs=1e-12)


@pytest.mark.parametrize('damage', ['none','size','position','orientation','duplicate','frame','extra_shape'])
def test_pickup_association_never_deletes_same_id_unrelated_geometry(capture,damage):
    _,context = request_and_context();domain = context.domain
    source = snapshot();scene = deserialize_message(source.scene_cdr,PlanningScene)
    box = CollisionObject();box.id = 'pickup_box';box.header.frame_id = 'world';box.operation = CollisionObject.ADD
    box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,dimensions=list(domain.box_wdh))]
    pose = Pose();pose.position.x,pose.position.y,pose.position.z = domain.box_xyz_world
    pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w = domain.box_quaternion_xyzw
    box.primitive_poses = [pose]
    if damage == 'size': box.primitives[0].dimensions[0] += .01
    elif damage == 'position': box.primitive_poses[0].position.x += .01
    elif damage == 'orientation': box.primitive_poses[0].orientation.w = 0.;box.primitive_poses[0].orientation.z = 1.
    elif damage == 'frame': box.header.frame_id = 'base_link'
    elif damage == 'extra_shape': box.mesh_poses = [Pose()]
    scene.world.collision_objects.append(box)
    if damage == 'duplicate': scene.world.collision_objects.append(box)
    raw = serialize_message(scene)
    source = replace(source,scene_cdr=raw)
    capture('source_scene.cdr',raw)
    if damage == 'none': admit_pickup_association(source,domain,'pickup_box')
    else:
        with pytest.raises(ValueError,match='ASSOCIATION_CONFLICT'):
            admit_pickup_association(source,domain,'pickup_box')
    assert source.scene_cdr == raw
