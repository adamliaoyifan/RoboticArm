"""Owner provenance/retention behavior, not collision or association proof."""
from dataclasses import replace
from types import SimpleNamespace

import pytest
from luggage_msgs.srv import PrepareJointOperationV2

from luggage_perception.frozen_suction_observation import FrozenObservationBinding
from luggage_planning.joint_operation.view_context import PickupViewContext,PickupViewContextStore
from luggage_planning.placement_prior.contracts import PlacementCandidate,Transform
from test_pickup_view_motion import motion_inputs


def context_case():
    f=motion_inputs()
    from test_joint_operation_observation import measured_case
    domain,model,probe,view=measured_case()
    f.domain,f.contact_model,f.probe,f.view=domain,model,probe,view
    request=PrepareJointOperationV2.Request(request_id='task',observation_id='observation',
        generation=domain.generation,expected_model_sha256=f.model.model_sha256,
        geometry_hash=f.source.geometry_hash,expected_map_revision=7,expected_map_epoch=1,
        robot_state_stamp_valid=True,robot_state_stamp_ns=domain.stamp_ns,start_joints=list(f.start),
        candidate_limit=2,budget_s=10.)
    detection=request.detection;detection.id=domain.box_id;detection.header.frame_id=domain.frame_id
    detection.header.stamp.sec,detection.header.stamp.nanosec=divmod(domain.stamp_ns,10**9)
    detection.width,detection.depth,detection.height=domain.box_wdh
    detection.pose.position.x,detection.pose.position.y,detection.pose.position.z=domain.box_xyz_world
    (detection.pose.orientation.x,detection.pose.orientation.y,detection.pose.orientation.z,
        detection.pose.orientation.w)=domain.box_quaternion_xyzw
    detection.top_surface_valid=detection.height_valid=True
    detection.height_source=detection.HEIGHT_SOURCE_MEASURED_SUPPORT
    binding=FrozenObservationBinding(request.request_id,request.observation_id,'a'*32,domain.box_id,
        domain.generation,domain.stamp_ns,domain.contact_model_hash,domain.digest(),'b'*64,'c'*64)
    f.observation=SimpleNamespace(binding=binding)
    f.original=request
    f.placement=PlacementCandidate('slot',Transform((0.,0.,.6+domain.box_wdh[2]/2),(0.,0.,0.,1.)))
    f.probe_message=probe.to_message(f.placement,f.source,request.request_id,8)
    f.context=PickupViewContext.capture(request,domain,f.observation,[f.probe_message])
    return f


def store(**changes):
    settings=dict(max_contexts=2,max_bytes=8*1024*1024,lifetime_s=5.)
    settings.update(changes)
    return PickupViewContextStore(**settings)


def test_context_captures_messages_without_aliasing_and_requires_exact_query():
    f=context_case();f.original.detection.width+=1.;f.probe_message.query_id='changed'
    request=f.context.request();probe=f.context.probe(f.probe.query_id)
    assert request.detection.width==f.domain.box_wdh[0] and probe.query_id==f.probe.query_id
    request.detection.width+=2.;probe.yaw_rad+=1.
    assert f.context.request().detection.width==f.domain.box_wdh[0]
    assert f.context.probe(f.probe.query_id).yaw_rad==f.probe.yaw_rad
    assert f.context.probe('invented') is None


@pytest.mark.parametrize('damage',['domain','placement_request','duplicate','binding'])
def test_unbound_or_duplicate_probe_cannot_be_remembered(damage):
    f=context_case();probes=[f.probe_message]
    if damage=='domain':probes[0].domain_sha256='other'
    elif damage=='placement_request':probes[0].placement.identity.request_id='other'
    elif damage=='duplicate':probes*=2
    else:f.observation.binding=replace(f.observation.binding,domain_sha256='other')
    with pytest.raises(ValueError,match='VIEW_CONTEXT_'):
        PickupViewContext.capture(f.original,f.domain,f.observation,probes)


def test_lookups_duplicate_retention_and_new_receipts_do_not_renew_context_lifetime():
    f=context_case();now=[10.];s=store(clock=lambda:now[0]);s.remember(f.context)
    now[0]=14.;s.remember(f.context)
    assert s.lookup('task',f.domain.digest()) is f.context
    assert s.lookup('other',f.domain.digest()) is None and s.lookup('task','wrong') is None
    now[0]=15.;assert s.lookup('task',f.domain.digest()) is None


def test_count_byte_eviction_and_explicit_replacement_do_not_rebind_context():
    f=context_case();s=store(max_contexts=1);s.remember(f.context)
    with pytest.raises(ValueError,match='REBIND'):
        s.remember(replace(f.context,request_cdr=f.context.request_cdr+b'x'))
    other=replace(f.context,task_request_id='other');s.remember(other)
    assert s.lookup('task',f.domain.digest()) is None
    s.forget('other');s.remember(f.context)
    assert s.lookup('task',f.domain.digest()) is f.context
    tiny=store(max_bytes=f.context.size_bytes-1)
    with pytest.raises(ValueError,match='BYTE_LIMIT'):tiny.remember(f.context)
    bytes_store=store(max_bytes=f.context.size_bytes+10)
    bytes_store.remember(f.context);bytes_store.remember(other)
    assert bytes_store.lookup('task',f.domain.digest()) is None


@pytest.mark.parametrize('settings',[dict(max_contexts=0),dict(max_bytes=0),dict(lifetime_s=0.),dict(lifetime_s=float('nan'))])
def test_retention_limits_are_explicit_and_positive(settings):
    with pytest.raises(ValueError):store(**settings)
