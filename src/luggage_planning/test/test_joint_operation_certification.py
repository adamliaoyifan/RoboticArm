"""Reject incomplete native receipts and preserve controller interpolation."""
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest

from luggage_planning.joint_operation.certification import _valid_response, _wire_leg, EndpointModel
from luggage_planning.joint_operation.planner import timed_operation_leg
from test_joint_operation_contracts import example_plan
from test_snapshot_joint_path_native import robot_fixture


def fixture():
    request=NS(request_id='req',route_digest='route',scene_digest='scene',
               legs=[NS(edge_id='carry',n_samples=3)])
    response=NS(request_id='req',route_digest='route',scene_digest='scene',model_sha256='model',
        result=0,all_valid=True,detail='',leg_results=[NS(edge_id='carry',valid=True,reason='',
            n_samples=3,first_invalid_sample=3,min_distance_m=[.1,float('nan'),float('nan')],
            contacts=[],support_contacts=[])])
    return request,response


def test_native_receipt_requires_explicit_all_samples_and_identity():
    request,response=fixture()
    assert _valid_response(response,request,'model')==''


@pytest.mark.parametrize('field,value', [('request_id','stale'),('route_digest','stale'),
    ('scene_digest','stale'),('model_sha256','stale'),('result',3),('all_valid',False),('leg_results',[])])
def test_foreign_or_incomplete_native_response_refuses(field,value):
    request,response=fixture()
    setattr(response,field,value)
    assert _valid_response(response,request,'model')


@pytest.mark.parametrize('field,value', [('edge_id','other'),('valid',False),('reason','COLLISION'),
    ('n_samples',2),('first_invalid_sample',2),('min_distance_m',[.1]),
    ('min_distance_m',[float('nan')]*3),('min_distance_m',[.1,0.,float('nan')]),
    ('min_distance_m',[.1,-.01,float('nan')]),('contacts',[object()]),('support_contacts',[object()])])
def test_per_leg_missing_samples_contacts_and_bad_clearance_refuse(field,value):
    request,response=fixture()
    setattr(response.leg_results[0],field,value)
    assert _valid_response(response,request,'model')


def test_wire_uses_controller_quintic_samples_and_original_unmodified_knots():
    from luggage_planning.moveit_state_validity import sample_states,sample_grid
    from luggage_planning.joint_operation.certification import _trajectory
    source=timed_operation_leg('carry','carry',((0.,)*6,(.005,)*6,(.01,)*6),
        example_plan().identity,.2,.1)
    original=deepcopy(source)
    message=_wire_leg(source)
    assert source==original
    assert message.n_samples>len(source.knots)
    assert tuple(p.time_from_start.sec*10**9+p.time_from_start.nanosec for p in message.knots.points)==tuple(k.time_from_start_ns for k in source.knots)
    times,states=sample_grid(_trajectory(source)),sample_states(_trajectory(source))
    unique=[]
    for t,row in zip(times,states):
        if not unique or t>unique[-1][0]: unique.append((t,row))
    assert list(message.sample_times_ns)==[t for t,_ in unique]
    assert tuple(message.sample_positions_flat)==tuple(v for _,row in unique for v in row)


def test_fk_model_keeps_world_base_transform_and_refuses_unsupported_joint_type():
    from luggage_planning.placement_prior.contracts import Transform
    xml,srdf=robot_fixture()
    shifted=xml.replace('<child link="base"/>','<child link="base"/><origin xyz="1 2 3"/>',1)
    model=EndpointModel(shifted,srdf,position_tolerance_m=.002,orientation_tolerance_rad=.02)
    assert model.matches((0.,)*6,Transform((2.2,2.,3.),(0.,0.,0.,1.)))
    assert not model.matches((0.,)*6,Transform((1.2,0.,0.),(0.,0.,0.,1.)))
    with pytest.raises(ValueError,match='six-revolute'):
        EndpointModel(xml.replace('type="revolute"','type="continuous"',1),srdf,
            position_tolerance_m=.002,orientation_tolerance_rad=.02)
