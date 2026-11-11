"""Generated bindings and actual ROS ports over frozen synthetic observations."""
import copy
from dataclasses import asdict,fields,replace
import hashlib
import json
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from luggage_msgs.srv import RetainSuctionObservationV2,CheckFrozenSuctionContactV2
from luggage_perception.frozen_suction_observation import FrozenSuctionObservations
from luggage_perception.frozen_suction_service import (
    FrozenSuctionService,binding_from_message,register_frozen_suction_services,
)
from test_directed_suction_service import request_and_context
from test_frozen_suction_observation import contact,relative


def retain_request(domain, detection):
    return RetainSuctionObservationV2.Request(request_id='operation',observation_id='view',
        domain_sha256=domain.digest(),detection=detection,generation=domain.generation)


def actual_request(binding,domain):
    return CheckFrozenSuctionContactV2.Request(binding=binding,attachment_geometry_sha256='a'*64,
        measured_box_wdh=list(domain.box_wdh),
        box_from_contact_frame=list(relative(domain,contact()).ravel()),budget_s=1.)


def capture_context(directory,context):
    directory.mkdir(parents=True,exist_ok=True)
    metadata=asdict(context.domain);metadata.pop('support')
    (directory/'observed_input.json').write_text(json.dumps({'domain':metadata,
        'model':context._model.as_dict(),'domain_sha256':context.domain_sha256,
        'samples_sha256':context.samples_sha256,'quality_policy_sha256':context.policy_sha256,
        'scope':'synthetic measured top plane; not RGB-D sensor qualification'},indent=2))
    np.savez_compressed(directory/'samples.npz',support=context.domain.cells(),**{
        f.name:getattr(context._height_map,f.name) for f in fields(context._height_map)
        if isinstance(getattr(context._height_map,f.name),np.ndarray)})


@pytest.fixture
def handler_case(request):
    from rclpy.serialization import serialize_message,deserialize_message
    original,context=request_and_context()
    current=SimpleNamespace(context=context,model=copy.deepcopy(context._model))
    store=FrozenSuctionObservations(max_records=3,max_bytes=8*1024**2,lifetime_s=10.,clock=lambda:0.)
    handler=FrozenSuctionService(store,context_provider=lambda:current.context,
        model_provider=lambda:current.model,clock=lambda:0.)
    counter=0
    root=None
    if os.environ.get('FROZEN_SUCTION_SERVICE_EVIDENCE_DIR'):
        root=Path(os.environ['FROZEN_SUCTION_SERVICE_EVIDENCE_DIR'])/request.node.name
        root.mkdir(parents=True,exist_ok=False);capture_context(root,context)
    def call(name,message,response=None):
        nonlocal counter
        service=RetainSuctionObservationV2 if name=='retain' else CheckFrozenSuctionContactV2
        raw=serialize_message(message)
        if root:(root/('%02d_%s_request.cdr'%(counter,name))).write_bytes(raw)
        result=getattr(handler,name)(deserialize_message(raw,service.Request),response or service.Response())
        encoded=serialize_message(result)
        if root:
            (root/('%02d_%s_response.cdr'%(counter,name))).write_bytes(encoded)
            (root/('%02d_current_model.json'%counter)).write_text(json.dumps(current.model.as_dict(),indent=2))
        counter+=1
        return deserialize_message(encoded,service.Response)
    return SimpleNamespace(current=current,handler=handler,context=context,original=original,call=call,
        retain_request=retain_request(context.domain,original.detection))


def test_generated_round_trip_and_new_frame_do_not_replace_frozen_samples(handler_case):
    f=handler_case;retained=f.call('retain',f.retain_request)
    assert retained.retained,retained.reason
    binding=binding_from_message(retained.binding)
    f.current.context=None
    result=f.call('check',actual_request(retained.binding,f.context.domain))
    assert result.checked and result.accepted,result.reason
    assert result.binding.header.stamp==retained.binding.header.stamp
    assert result.binding.samples_sha256==binding.samples_sha256
    assert np.array(result.world_from_contact_frame_at_observation).reshape(4,4)==pytest.approx(contact(),abs=1e-15)
    assert result.evidence_sha256


@pytest.mark.parametrize('kind',['request','observation','domain','generation','dimensions','no_context','model'])
def test_retain_requires_exact_current_measured_context(handler_case,kind):
    f=handler_case;r=f.retain_request
    if kind=='request':r.request_id=''
    elif kind=='observation':r.observation_id=''
    elif kind=='domain':r.domain_sha256='other'
    elif kind=='generation':r.generation+=1
    elif kind=='dimensions':r.detection.width+=.01
    elif kind=='no_context':f.current.context=None
    else:f.current.model.boundary_margin_m+=.001
    response=f.call('retain',r)
    assert not response.retained and not response.binding.binding_sha256


@pytest.mark.parametrize('kind',['binding','frame','box','generation','dimensions','model','revoked','budget','pose'])
def test_actual_identity_or_source_refuses_with_inactive_response(handler_case,kind):
    f=handler_case;saved=f.call('retain',f.retain_request);assert saved.retained
    r=actual_request(saved.binding,f.context.domain)
    previous=f.call('check',r);assert previous.accepted
    if kind=='binding':r.binding.binding_sha256='other'
    elif kind=='frame':r.binding.header.frame_id='camera_link'
    elif kind in ('box','generation'):
        from luggage_perception.frozen_suction_service import binding_to_message
        binding=binding_from_message(r.binding)
        r.binding=binding_to_message(replace(binding,**({'box_id':'other'} if kind=='box' else {'generation':binding.generation+1})))
    elif kind=='dimensions':r.measured_box_wdh[0]+=.01
    elif kind=='model':f.current.model.boundary_margin_m+=.001
    elif kind=='revoked':f.handler.store.revoke('operation','view')
    elif kind=='budget':r.budget_s=0.
    else:r.box_from_contact_frame[0]=2.
    result=f.call('check',r,previous)
    assert not result.checked and not result.accepted,result.reason
    assert not result.binding.binding_sha256 and not result.evidence_sha256
    assert not result.quality_metrics_json
    assert not np.any(result.world_from_contact_frame_at_observation)


@pytest.mark.parametrize('stage',['retain','check'])
def test_policy_change_during_owner_work_discards_positive(handler_case,stage,monkeypatch):
    f=handler_case
    if stage=='retain':
        original=f.handler.store.retain
        def changing(*a,**kw):
            value=original(*a,**kw);f.current.model.boundary_margin_m+=.001;return value
        monkeypatch.setattr(f.handler.store,'retain',changing)
        result=f.call('retain',f.retain_request)
        assert not result.retained and not result.binding.binding_sha256
    else:
        saved=f.call('retain',f.retain_request);assert saved.retained
        original=f.handler.store.check
        def changing(*a,**kw):
            value=original(*a,**kw);f.current.model.boundary_margin_m+=.001;return value
        monkeypatch.setattr(f.handler.store,'check',changing)
        result=f.call('check',actual_request(saved.binding,f.context.domain))
        assert not result.checked and not result.accepted and not result.evidence_sha256


@pytest.fixture(scope='module')
def actual_ros():
    if not os.environ.get('FROZEN_SUCTION_ROS_DOMAIN_ID'):
        pytest.skip('explicit isolated frozen-observation ROS RPC gate')
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from luggage_planning.joint_operation_actual_contact import FrozenActualContactPort
    from luggage_planning.joint_operation_contracts import OperationIdentity
    from luggage_planning.placement_prior_contracts import Transform
    from st09_contracts.attachment_geometry import freeze_geometry
    from st09_contracts.payload_envelope import EnvelopeLimits
    from scipy.spatial.transform import Rotation
    domain_id=int(os.environ['FROZEN_SUCTION_ROS_DOMAIN_ID'])
    directory=Path(os.environ['FROZEN_SUCTION_ROS_EVIDENCE_DIR']);directory.mkdir(parents=True,exist_ok=False)
    original,context=request_and_context();capture_context(directory,context)
    current=SimpleNamespace(context=context,model=copy.deepcopy(context._model))
    ctx=rclpy.context.Context();rclpy.init(context=ctx,domain_id=domain_id)
    server=rclpy.create_node('frozen_observation_owner_fixture',context=ctx)
    node=rclpy.create_node('actual_contact_consumer_fixture',context=ctx)
    handler=register_frozen_suction_services(server,context_provider=lambda:current.context,
        model_provider=lambda:current.model)
    records=[]
    def record(kind,raw):
        file='%03d_%s.cdr'%(len(records),kind);(directory/file).write_bytes(raw)
        records.append({'kind':kind,'file':file,'sha256':hashlib.sha256(raw).hexdigest(),
            'case':os.environ.get('PYTEST_CURRENT_TEST')})
    port=FrozenActualContactPort(node,recorder=record)
    executor=MultiThreadedExecutor(num_threads=2,context=ctx)
    executor.add_node(node);executor.add_node(server)
    worker=threading.Thread(target=executor.spin,daemon=True);worker.start()
    assert port.retain_client.wait_for_service(timeout_sec=3.) and port.check_client.wait_for_service(timeout_sec=3.)
    identity=OperationIdentity('operation','scene','fixture','hull','model',1,'view',
        context.domain.stamp_ns,15,context.domain.box_id,context.domain.generation,context.domain.contact_model_hash)
    tool_box=np.linalg.inv(relative(context.domain,contact()))
    def geometry(transform=tool_box):
        return freeze_geometry(context.domain.box_wdh,tuple(transform[:3,3]),
            tuple(Rotation.from_matrix(transform[:3,:3]).as_quat()),EnvelopeLimits(0.,(0.,0.,0.),(0.,0.),0.))
    try:
        yield SimpleNamespace(port=port,handler=handler,current=current,context=context,
            original=original,identity=identity,geometry=geometry,directory=directory,record=record,records=records)
    finally:
        executor.shutdown(timeout_sec=3.);worker.join(timeout=3.)
        port.retain_client.destroy();port.check_client.destroy()
        server.destroy_node();node.destroy_node();ctx.shutdown()
        (directory/'manifest.json').write_text(json.dumps({'domain_id':domain_id,'records':records,
            'worker_joined':not worker.is_alive(),'context_terminal':not ctx.ok(),
            'scope':'actual typed ROS RPC and detector registration helper with synthetic measured input; no actuator/Gazebo'},indent=2))
        assert not worker.is_alive() and not ctx.ok()


@pytest.fixture
def pinned(actual_ros):
    f=actual_ros;f.current.context=f.context;f.current.model=copy.deepcopy(f.context._model)
    f.handler.store.revoke('operation','view')
    saved=f.port.retain(f.context.domain,f.original.detection,request_id='operation',observation_id='view',
        deadline=time.monotonic()+2.,cancelled=lambda:False)
    assert saved.retained,saved.reason
    return saved.observation


def check(f,pinned,**changes):
    options=dict(identity=f.identity,geometry=f.geometry(),deadline=time.monotonic()+2.,cancelled=lambda:False)
    options.update(changes)
    result=f.port.check(pinned,**options)
    f.record('consumer_outcome',json.dumps({'checked':result.checked,'accepted':result.accepted,
        'reason':result.reason,'result':asdict(result.result) if result.result else None}).encode())
    return result


def test_actual_ros_freeze_and_changed_measured_contact_use_original_observation(actual_ros,pinned):
    f=actual_ros
    # No current camera frame can supply this result after retention.
    f.current.context=None
    result=check(f,pinned)
    assert result.checked and result.accepted,result.reason
    shifted=np.linalg.inv(relative(f.context.domain,contact((.04,-.02),.02)))
    moved=check(f,pinned,geometry=f.geometry(shifted))
    assert moved.checked and moved.accepted,moved.reason
    assert moved.result.box_from_contact_frame!=result.result.box_from_contact_frame
    assert moved.result.binding_sha256==result.result.binding_sha256


@pytest.mark.parametrize('kind',['request','observation','box','generation','stamp','contact_model','size','source_model','revoked'])
def test_actual_ros_rejects_identity_model_and_revocation(actual_ros,pinned,kind):
    f=actual_ros;changes={}
    if kind=='source_model':f.current.model.boundary_margin_m+=.001
    elif kind=='revoked':f.handler.store.revoke('operation','view')
    elif kind=='size':
        from st09_contracts.attachment_geometry import freeze_geometry
        g=f.geometry();changes['geometry']=freeze_geometry((.71,.45,.3),g.t_tool_box_translation,
            g.t_tool_box_quaternion_xyzw,g.conservative.limits)
    else:
        key={'request':'request_id','observation':'observation_id','box':'box_id',
            'generation':'generation','stamp':'observation_stamp_ns','contact_model':'contact_model_hash'}[kind]
        value=getattr(f.identity,key);changes['identity']=replace(f.identity,**{key:value+1 if type(value)is int else value+'other'})
    result=check(f,pinned,**changes)
    assert not result.accepted


@pytest.mark.parametrize('kind',['evidence','geometry','binding','pose','metrics'])
def test_actual_rpc_response_corruption_cannot_grant_positive(actual_ros,pinned,kind,monkeypatch):
    from rclpy.serialization import serialize_message
    f=actual_ros;original=f.port._call
    def damaged(*args,**kw):
        response=original(*args,**kw)
        assert response.checked and response.accepted
        if kind=='evidence':response.evidence_sha256='forged'
        elif kind=='geometry':response.attachment_geometry_sha256='b'*64
        elif kind=='binding':response.binding.binding_sha256='b'*64
        elif kind=='pose':response.world_from_contact_frame_at_observation[3]+=.001
        else:response.quality_metrics_json='{"valid_coverage": 1}'
        f.record('injected_corrupt_response',serialize_message(response))
        return response
    monkeypatch.setattr(f.port,'_call',damaged)
    result=check(f,pinned)
    assert not result.checked and not result.accepted


def test_timed_out_real_rpc_remains_owned_until_response(actual_ros,pinned,monkeypatch):
    from rclpy.serialization import serialize_message
    f=actual_ros;original=f.handler.store.check;entered=threading.Event();release=threading.Event()
    def delayed(*a,**kw):
        value=original(*a,**kw);entered.set()
        assert release.wait(3.), 'test did not release pending service'
        return value
    monkeypatch.setattr(f.handler.store,'check',delayed)
    first=check(f,pinned,deadline=time.monotonic()+.05)
    assert entered.wait(1.)
    try:
        assert not first.accepted
        before=len(f.records);second=check(f,pinned)
        assert not second.accepted and 'BACKEND_BUSY' in second.reason
        assert len(f.records)==before+1 # outcome only; no second RPC
        retained=f.port.retain(f.context.domain,f.original.detection,request_id='second',observation_id='view',
            deadline=time.monotonic()+1.,cancelled=lambda:False)
        assert not retained.retained and 'BACKEND_BUSY' in retained.reason
        assert len(f.records)==before+1 # no overlapping RPC on the other client
    finally:release.set()
    future=f.port._inflight[f.port.check_client];end=time.monotonic()+2.
    while not future.done() and time.monotonic()<end:time.sleep(.01)
    assert future.done()
    f.record('late_actual_contact_response',serialize_message(future.result()))
    assert not future.result().accepted
