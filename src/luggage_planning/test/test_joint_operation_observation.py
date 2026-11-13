"""Actual footprint/measurement gaps, separate from IK and quality refusal."""
from dataclasses import asdict, replace
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from luggage_perception.observed_contact_footprint import project_contact_footprint
from luggage_perception.pickup_view_evidence import PickupViewAcquisition
from luggage_planning.joint_operation.contact_observation import contact_matrix, unresolved_probe
from luggage_planning.joint_operation.pickup_observation import assess_pickup_observation
from luggage_planning.joint_operation.candidates import propose_contact
from luggage_planning.placement_prior.contracts import Transform
from test_directed_suction_quality import context_inputs


def measured_case(*, yaw=0., hole=True, shape=(240,320), region_source='instance_mask',
                  support=False, domain_shape=None, pose=None, fx=200.):
    domain,_,model = context_inputs()
    if domain_shape:
        domain=replace(domain,shape=domain_shape,origin_uv_top_m=tuple(-n*domain.cell_size_m/2 for n in domain_shape),
                       support=bytes([1 if support else 0])*math.prod(domain_shape))
    else:
        domain=replace(domain,support=bytes([1 if support else 0])*len(domain.support))
    matrix=np.diag([1.,-1.,-1.,1.]);matrix[:3,3]=(0.,0.,2.)
    depth=np.full(shape,1000.,dtype=np.float32);region=np.ones(shape,bool)
    if hole:depth[shape[0]//2+3,shape[1]//2+7]=0
    if pose is not None:
        rotation=pose[:3,:3];translation=pose[:3,3]
        basis=np.array([domain.u_axis_world,domain.v_axis_world,domain.normal_world]) @ rotation.T
        domain=replace(domain,u_axis_world=tuple(basis[0]),v_axis_world=tuple(basis[1]),
            normal_world=tuple(basis[2]),plane_offset_world_m=domain.plane_offset_world_m-np.dot(basis[2],translation),
            origin_uv_top_m=tuple(np.asarray(domain.origin_uv_top_m)+basis[:2]@translation),
            box_xyz_world=tuple(rotation@domain.box_xyz_world+translation),
            box_quaternion_xyzw=Transform.from_matrix(pose).quaternion_xyzw)
        matrix=pose@matrix
    uv=np.array([domain.u_axis_world,domain.v_axis_world]) @ (pose[:3,3] if pose is not None else np.zeros(3))
    vertices=project_contact_footprint(domain,model,contact_matrix(domain,uv,yaw))
    probe=unresolved_probe(domain,model,'placement:heading',uv,yaw,vertices)
    a=PickupViewAcquisition.capture(depth,region,SimpleNamespace(fx=fx,fy=fx,cx=(shape[1]-1)/2,cy=(shape[0]-1)/2),
        matrix,stamp_ns=domain.stamp_ns,optical_frame_id='actual_optical',instance_id=domain.instance_id,
        generation=domain.generation,region_source=region_source)
    return domain,model,probe,a.bind(domain)


@pytest.fixture
def capture(request):
    root=os.environ.get('JOINT_OBSERVATION_EVIDENCE')
    directory=Path(root)/request.node.name if root else None
    if directory:directory.mkdir(parents=True,exist_ok=False)
    def record(domain,view,probe,result):
        if directory:
            from rclpy.serialization import serialize_message
            from luggage_perception.ros_suction_domain import domain_to_message
            from luggage_perception.pickup_view_service import view_to_message
            (directory/'domain.cdr').write_bytes(serialize_message(domain_to_message(domain)))
            (directory/'view.cdr').write_bytes(serialize_message(view_to_message(view)))
            (directory/'probe.json').write_text(json.dumps(asdict(probe) if probe else None,indent=2))
            (directory/'result.json').write_text(json.dumps(asdict(result),indent=2))
    return record


@pytest.mark.parametrize('yaw',[0.,math.pi/2,math.pi,-math.pi/2])
@pytest.mark.parametrize('region_source',['instance_mask','detection_bbox'])
def test_off_centre_one_pixel_gap_is_not_hidden_by_valid_footprint_centre(capture,yaw,region_source):
    domain,model,probe,view=measured_case(yaw=yaw,region_source=region_source)
    before=(domain.digest(),view.digest(),domain.support,view.acquisition.depth_mm)
    result=assess_pickup_observation(domain,model,probe,view);capture(domain,view,probe,result)
    assert result.reason=='CONDITIONAL_OBSERVATION_NEED'
    assert result.missing_depth_pixels==1 and result.outside_image_pixels==0
    assert result.need.region_source==region_source
    assert result.need.gap_targets_world and result.need.view_sha256==view.digest()
    assert (domain.digest(),view.digest(),domain.support,view.acquisition.depth_mm)==before


@pytest.mark.parametrize('cause',['finite_depth','excluded_mask','all_support'])
def test_no_view_for_quality_like_refusal_mask_exclusion_or_full_support(capture,cause):
    domain,model,probe,view=measured_case(hole=cause!='finite_depth',support=cause=='all_support')
    if cause=='all_support':
        assert probe is None
        return
    if cause=='excluded_mask':
        a=view.acquisition;region=np.frombuffer(a.instance_region,dtype=bool).reshape(a.shape).copy()
        region[a.shape[0]//2+3,a.shape[1]//2+7]=False
        view=replace(a,instance_region=region.tobytes()).bind(domain)
    result=assess_pickup_observation(domain,model,probe,view);capture(domain,view,probe,result)
    assert result.reason=='NO_LOCALIZED_MISSING_MEASUREMENT'
    assert result.need is None and result.missing_depth_pixels==0
    assert bool(result.excluded_region_pixels)==(cause=='excluded_mask')


def test_fov_clipping_and_too_small_observed_grid_remain_unproven(capture):
    domain,model,_,view=measured_case(shape=(20,20),hole=False,domain_shape=(8,8))
    proposal=propose_contact(domain,model,query_id='deep',max_center_checks=8,
        world_from_place_box=Transform(domain.box_xyz_world,domain.box_quaternion_xyzw),
        desired_world_from_tool=Transform((0.,0.,1.),(1.,0.,0.,0.)))
    assert proposal.reason=='FOOTPRINT_LARGER_THAN_OBSERVED_GRID'
    assert not proposal.center_uv_top_m and proposal.checked_centres==0
    assert proposal.unresolved_probe is not None
    result=assess_pickup_observation(domain,model,proposal.unresolved_probe,view)
    capture(domain,view,proposal.unresolved_probe,result)
    assert result.need is not None and result.outside_image_pixels>0
    with pytest.raises(ValueError):proposal.to_message()


@pytest.mark.parametrize('kind',['internal_unknown','behind_camera','nan_depth'])
def test_actual_missing_support_and_camera_geometry_are_separate(capture,kind):
    domain,model,probe,view=measured_case(support=kind=='internal_unknown')
    if kind=='internal_unknown':
        cells=domain.cells().copy();cells[70,45]=False
        domain=replace(domain,support=cells.tobytes());view=view.acquisition.bind(domain)
        vertices=project_contact_footprint(domain,model,contact_matrix(domain,(0.,0.),0.))
        probe=unresolved_probe(domain,model,'internal',(0.,0.),0.,vertices)
        assert probe.unproven_cell_count==1
    elif kind=='behind_camera':
        matrix=np.eye(4);matrix[:3,3]=(0.,0.,2.)
        view=replace(view.acquisition,world_from_optical=tuple(matrix.ravel())).bind(domain)
    else:
        a=view.acquisition;depth=np.frombuffer(a.depth_mm,dtype=a.depth_dtype).reshape(a.shape).copy()
        depth[depth==0]=np.nan
        view=replace(a,depth_mm=depth.tobytes()).bind(domain)
    result=assess_pickup_observation(domain,model,probe,view);capture(domain,view,probe,result)
    if kind=='behind_camera':
        assert result.need is None and 'NOT_IN_FRONT_OF_CAMERA' in result.reason
    else:
        assert result.need is not None and result.missing_depth_pixels==1


@pytest.mark.parametrize('damage',['domain','model','vertices','unknown_count','view_stamp'])
def test_mismatched_hypotheses_and_sources_cannot_generate_information_need(capture,damage):
    domain,model,probe,view=measured_case()
    if damage=='domain':probe=replace(probe,domain_sha256='0'*64)
    elif damage=='model':probe=replace(probe,contact_model_hash='unrelated')
    elif damage=='vertices':probe=replace(probe,footprint_uv_top_m=tuple(x+.001 for x in probe.footprint_uv_top_m))
    elif damage=='unknown_count':probe=replace(probe,unproven_cell_count=1)
    else:view=replace(view,acquisition=replace(view.acquisition,stamp_ns=view.acquisition.stamp_ns-1))
    result=assess_pickup_observation(domain,model,probe,view);capture(domain,view,probe,result)
    assert result.need is None


def test_rigidly_transformed_measurement_keeps_gap_localization(capture):
    original=measured_case()
    angle=.4
    pose=np.eye(4);pose[:3,:3]=[[math.cos(angle),-math.sin(angle),0.],[math.sin(angle),math.cos(angle),0.],[0.,0.,1.]]
    pose[:3,3]=(.4,-.7,.2)
    domain,model,probe,view=measured_case(pose=pose)
    result=assess_pickup_observation(domain,model,probe,view);capture(domain,view,probe,result)
    prior=assess_pickup_observation(*original)
    assert result.need is not None and result.missing_depth_pixels==prior.missing_depth_pixels==1
    transformed=np.array(prior.need.gap_targets_world).reshape(-1,3)@pose[:3,:3].T+pose[:3,3]
    assert np.array(result.need.gap_targets_world).reshape(-1,3)==pytest.approx(transformed)


def test_failed_search_keeps_only_checked_best_probe_and_bound_target_wire(capture):
    from rclpy.serialization import serialize_message,deserialize_message
    from luggage_msgs.msg import SuctionSupportProbeV2
    from test_joint_operation_candidates import snapshot,placed_pose
    from luggage_planning.placement_prior.contracts import PlacementCandidate
    domain,model,_,view=measured_case()
    supported=domain.cells().copy();supported[68:73,43:48]=True
    domain=replace(domain,support=supported.tobytes());view=view.acquisition.bind(domain)
    proposal=propose_contact(domain,model,query_id='candidate',max_center_checks=3,
        world_from_place_box=placed_pose(0.),desired_world_from_tool=Transform((0.,-.8,1.),(1.,0.,0.,0.)))
    capture(domain,view,proposal.unresolved_probe,assess_pickup_observation(domain,model,proposal.unresolved_probe,view))
    assert proposal.checked_centres==3 and proposal.unresolved_probe is not None
    assert not proposal.center_uv_top_m
    placement=PlacementCandidate('slot',placed_pose(0.))
    message=proposal.unresolved_probe.to_message(placement,snapshot(),'request',proposal.checked_centres)
    decoded=deserialize_message(serialize_message(message),SuctionSupportProbeV2)
    assert decoded.domain_sha256==domain.digest() and decoded.placement_slot_id=='slot'
    assert (decoded.placement.pose.position.x,decoded.placement.pose.position.y,decoded.placement.pose.position.z)==placement.container_from_box.translation
    assert decoded.checked_centres==3 and not hasattr(decoded,'accepted')


def test_cancelled_core_drops_even_a_reused_response_probe():
    from luggage_msgs.srv import PrepareJointOperationV2
    from luggage_planning.joint_operation.preparation import JointOperationPreparation,PreparationInputs
    from luggage_planning.planning_contract.payload_envelope import EnvelopeLimits
    from luggage_planning.placement_prior.contracts import PlacementCandidate
    from test_joint_operation_candidates import snapshot
    domain,model,_,_=measured_case();source=snapshot()
    # Cancellation is a lifecycle outcome, not a request for extra observations.
    service=JointOperationPreparation(None,SimpleNamespace(reserve_s=.1),model,EnvelopeLimits(0.,(0.,0.,0.),(0.,0.),0.),
        max_center_checks=3,clock=lambda:0.)
    request=PrepareJointOperationV2.Request(request_id='cancelled',observation_id='observed',
        robot_state_stamp_valid=True,candidate_limit=1,budget_s=10.,geometry_hash=source.geometry_hash,
        expected_map_revision=source.map_revision,expected_model_sha256=source.model_hash,generation=domain.generation)
    # Matching detection first, before cancellation reaches the motion loop.
    request.detection.id=domain.box_id;request.detection.header.frame_id='world'
    request.detection.header.stamp.sec,request.detection.header.stamp.nanosec=divmod(domain.stamp_ns,10**9)
    request.detection.pose.position.x,request.detection.pose.position.y,request.detection.pose.position.z=domain.box_xyz_world
    request.detection.pose.orientation.w=1.
    request.detection.width,request.detection.depth,request.detection.height=domain.box_wdh
    request.detection.top_surface_valid=request.detection.height_valid=True
    request.detection.height_source=request.detection.HEIGHT_SOURCE_MEASURED_SUPPORT
    reused=PrepareJointOperationV2.Response()
    reused.unresolved_contacts=[measured_case()[2].to_message(
        PlacementCandidate('previous',Transform((0.,0.,0.),(0.,0.,0.,1.))),source,'previous',1)]
    response=service.prepare(request,reused,
        PreparationInputs(source,domain,(),Transform((0.,0.,1.),(1.,0.,0.,0.)),'pickup'),
        deadline=10.,revalidate=lambda *_:True,cancelled=lambda:True)
    assert response.reason=='CANCELLED_OR_DEADLINE'
    assert not response.success and not response.unresolved_contacts and not response.plan.legs
