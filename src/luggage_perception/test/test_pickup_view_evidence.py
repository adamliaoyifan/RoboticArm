"""Lossless source binding and optional camera retention, without new quality gates."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from luggage_perception.directed_suction_quality import DirectedSuctionContext
from luggage_perception.pickup_view_evidence import PickupViewAcquisition
from luggage_perception.suction_acquisition_history import SuctionAcquisitionHistory
from test_directed_suction_service import request_and_context


def view_context(*, stamp_ns=12_000_000_000, dtype='<f4', region_source='instance_mask'):
    request,original=request_and_context()
    domain=replace(original.domain,stamp_ns=stamp_ns)
    depth=np.full((8,12),1000,dtype=dtype);depth[2,3]=0
    if np.dtype(dtype).kind=='f':depth[4,5]=np.nan
    region=np.ones(depth.shape,bool);region[0,0]=False
    matrix=np.diag([1.,-1.,-1.,1.]);matrix[:3,3]=(.1,.2,1.5)
    a=PickupViewAcquisition.capture(depth,region,SimpleNamespace(fx=120.,fy=121.,cx=5.5,cy=3.5),matrix,
        stamp_ns=stamp_ns,optical_frame_id='measured_optical',instance_id=domain.instance_id,
        generation=domain.generation,region_source=region_source)
    context=DirectedSuctionContext(domain,original._height_map,original._model,pickup_view=a.bind(domain))
    request.domain_sha256=domain.digest()
    request.detection.header.stamp.sec,request.detection.header.stamp.nanosec=divmod(stamp_ns,10**9)
    return request,context,depth,region


@pytest.mark.parametrize('dtype',['<u2','>u2','<f4','<f8'])
@pytest.mark.parametrize('region_source',['instance_mask','detection_bbox'])
def test_capture_owns_samples_preserves_invalid_depth_and_exact_wire(dtype,region_source):
    from rclpy.serialization import serialize_message,deserialize_message
    from luggage_msgs.msg import PickupViewEvidenceV2
    from luggage_perception.pickup_view_service import view_to_message,view_from_message
    _,context,depth,region=view_context(dtype=dtype,region_source=region_source)
    view=context.pickup_view;before=view.digest()
    original_depth=depth.copy();original_region=region.copy()
    depth[:]=42;region[:]=True
    decoded=view_from_message(deserialize_message(serialize_message(view_to_message(view)),PickupViewEvidenceV2),context.domain)
    assert decoded==view and decoded.digest()==before
    a=decoded.acquisition
    assert a.stamp_ns==context.domain.stamp_ns and a.optical_frame_id=='measured_optical'
    assert a.region_source==region_source
    assert np.array_equal(np.frombuffer(a.depth_mm,dtype=a.depth_dtype).reshape(a.shape),original_depth,equal_nan=True)
    assert bytes(a.instance_region)==original_region.astype(np.uint8).tobytes()


@pytest.mark.parametrize('damage',['time','frame','tf','intrinsics','depth','mask','domain','generation'])
def test_changed_source_or_bytes_cannot_pass_wire_binding(damage):
    from luggage_perception.pickup_view_service import view_to_message,view_from_message
    _,context,_,_=view_context()
    message=view_to_message(context.pickup_view)
    if damage=='time':message.header.stamp.nanosec+=1
    elif damage=='frame':message.header.frame_id='another_camera'
    elif damage=='tf':message.world_from_optical[3]+=.01
    elif damage=='intrinsics':message.intrinsics_fx_fy_cx_cy[0]+=1.
    elif damage=='depth':message.depth_mm.data[0]^=1
    elif damage=='mask':message.instance_region.data[0]^=1
    elif damage=='domain':message.domain_sha256='0'*64
    else:message.generation+=1
    with pytest.raises(ValueError):view_from_message(message,context.domain)


def test_optional_view_pressure_preserves_original_quality_rows_and_expiry():
    _,old,_,_=view_context();_,new,_,_=view_context(stamp_ns=12_000_000_001)
    now=[0.]
    history=SuctionAcquisitionHistory(max_view_bytes=old.pickup_view.size_bytes,
        lifetime_s=1.,clock=lambda:now[0])
    history.remember(old.domain,context=old);history.remember(new.domain,context=new)
    retained=history.context_for_digest(old.domain_sha256)
    assert retained is not None and retained.samples_sha256==old.samples_sha256
    assert retained.pickup_view is None and retained.pickup_view_reason=='PICKUP_VIEW_RESOURCE_EVICTED'
    assert history.context_for_digest(new.domain_sha256).pickup_view==new.pickup_view
    assert history.diagnostics()['view_bytes']==new.pickup_view.size_bytes
    now[0]=.9;history.remember(new.domain,context=new)
    now[0]=1.
    assert history.context_for_digest(new.domain_sha256) is None
    assert history.diagnostics()['view_bytes']==0


def test_zero_camera_budget_and_view_only_conflict_preserve_quality():
    _,context,_,_=view_context()
    history=SuctionAcquisitionHistory(max_view_bytes=0)
    history.remember(context.domain,context=context)
    retained=history.context_for_digest(context.domain_sha256)
    assert retained.samples_sha256==context.samples_sha256 and retained.pickup_view is None
    assert history.diagnostics()['view_bytes']==0
    acquisition=context.pickup_view.acquisition
    changed=replace(acquisition,region_source='detection_bbox').bind(context.domain)
    other=DirectedSuctionContext(context.domain,context._height_map,context._model,pickup_view=changed)
    history.remember(context.domain,context=other)
    retained=history.context_for_digest(context.domain_sha256)
    assert retained is not None and retained.samples_sha256==context.samples_sha256
    assert retained.pickup_view is None and retained.pickup_view_reason=='PICKUP_VIEW_CONFLICTING_ACQUISITION'
    assert history.diagnostics()['last_view_conflict']['domain_sha256']==context.domain_sha256


def test_optional_camera_record_does_not_change_quality_digest_or_long_lived_retention():
    from luggage_perception.frozen_suction_observation import FrozenSuctionObservations
    _,context,_,_=view_context()
    ordinary=DirectedSuctionContext(context.domain,context._height_map,context._model)
    assert context.samples_sha256==ordinary.samples_sha256
    assert context.policy_sha256==ordinary.policy_sha256
    store=FrozenSuctionObservations(max_records=4,max_bytes=8*1024*1024,lifetime_s=10.)
    binding=store.retain('operation','observation',context)
    retained=store.lookup(binding)
    assert retained.samples_sha256==context.samples_sha256
    assert retained.pickup_view is None
