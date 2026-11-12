"""O conversion and source ownership behaviors; native adoption is separate."""
from dataclasses import asdict, replace
import hashlib
import json
from types import SimpleNamespace
from concurrent.futures import Future
import time

import pytest
import yaml

from luggage_description.container_geometry import cuboid_from_inner_size
from luggage_planning.joint_operation.map_source import OperationMapSource
from luggage_planning.joint_operation.observer import (
    ObserverConfiguration, ObserverSnapshotPort, admit_response, assemble_snapshot,
)


def configuration(directory, size=(.4,.4,.4), floor=.6, resolution=.1):
    descriptor = cuboid_from_inner_size(size,floor_z=floor).descriptor()
    canonical = json.dumps(descriptor,sort_keys=True,separators=(',',':'))
    geometry, config = directory/'geometry.yaml', directory/'observer.yaml'
    geometry.write_text(yaml.safe_dump(dict(schema='uld_geometry_export_v1',descriptor_json=canonical,
        descriptor_sha256=hashlib.sha256(canonical.encode()).hexdigest())))
    config.write_text(yaml.safe_dump(dict(resolution_m=resolution,depth_z_min_m=.01,
                                        depth_z_max_m=6.,clock_domain='ros')))
    return ObserverConfiguration.load(geometry,config),geometry,config


def identity(config):
    return dict(epoch=1,map_revision=7,observed_revision=9,last_integrated_stamp_ns=12_000_000_000,
                geometry_hash=config.descriptor()['geometry_hash'],config_digest=config.config_digest,
                clock_domain='ros')


def columns_for(config, states):
    nx,ny,nz=config.dimensions_xyz
    result=[]
    for y in range(ny):
        for x in range(nx):
            values=[states[x+nx*(y+ny*z)] for z in range(nz)]
            occupied=2 in values
            result.append(dict(x=x,y=y,active_count=sum(v!=3 for v in values),
                known_count=sum(v in (1,2) for v in values),has_occupied=occupied,
                top_z=min(config.descriptor()['ceiling_z'],config.descriptor()['floor_z']+
                    (max(z for z,v in enumerate(values) if v==2)+1)*config.resolution_m) if occupied else 0.))
    return result


def snapshot(config, states, columns=None):
    return assemble_snapshot(config,identity(config),2,columns_for(config,states) if columns is None else columns,
                             states,scene_revision='scene',request_id='req')


def test_config_uses_exact_native_bytes_and_dimensions(tmp_path):
    config,geometry,path=configuration(tmp_path,size=(.4,.4,.7))
    assert config.config_digest==hashlib.sha256(geometry.read_bytes()+b'\n'+path.read_bytes()+
                                               b'\nprobability-profile-v1').hexdigest()
    import math
    assert config.dimensions_xyz==(4,4,math.ceil((1.3-.6)/.1))
    path.write_text(path.read_text()+'\n')
    assert ObserverConfiguration.load(geometry,path).config_digest!=config.config_digest


def test_x_fastest_unknown_occupied_and_inactive_are_preserved(tmp_path):
    from luggage_planning.placement_prior.volume import blocked_volume_runs
    config,_,_=configuration(tmp_path)
    states=[1]*64
    states[0]=0;states[1+4*(2+4*1)]=2;states[3+4*(3+4*3)]=3
    captured=snapshot(config,states)
    record=captured.surface.record()
    assert record['state'][0][0]=='unknown' and record['surface_z_container'][0][0] is None
    assert record['state'][1][2]=='occupied' and record['surface_z_container'][1][2]==pytest.approx(.8)
    assert captured.volume.active[-1] is False and captured.volume.states[-1]==0
    assert record['state'][3][3]=='free','all remaining active cells are observed free'
    boxes=blocked_volume_runs(captured.volume,captured.surface)
    assert len(boxes)==2
    assert sorted(box[0][2] for box in boxes)==pytest.approx([.65,.75])
    metadata=json.loads(captured.surface.confirmation_json)
    assert metadata['source']=='observer' and 'occupancy_confirmed' not in metadata
    assert metadata['observer_source']==asdict(captured.source)


def test_partial_known_column_is_not_promoted_to_free_volume(tmp_path):
    config,_,_=configuration(tmp_path)
    states=[0]*64;states[0]=1
    captured=snapshot(config,states)
    assert captured.surface.record()['state'][0][0]=='unknown'
    assert captured.volume.states.count(1)==1


def test_real_candidate_consumer_reads_immutable_observer_volume(tmp_path):
    from luggage_planning.placement_prior.requests import enumerate_placements
    config,_,_=configuration(tmp_path,size=(1.,1.,.4))
    count=__import__('math').prod(config.dimensions_xyz)
    captured=snapshot(config,[1]*count)
    candidates=enumerate_placements(captured.surface,(.2,.15,.1),32,volume=captured.volume,margin=.01)
    assert candidates and all(c.container_from_box.translation[2]==pytest.approx(.65) for c in candidates)


@pytest.mark.parametrize('damage',['missing_cell','invalid_state','missing_column','duplicate_column',
                                  'active_count','known_count','has_occupied','top_z'])
def test_conversion_rejects_incomplete_or_contradictory_views(tmp_path,damage):
    config,_,_=configuration(tmp_path)
    states=[1]*64;states[0]=2
    columns=columns_for(config,states)
    if damage=='missing_cell':states.pop()
    elif damage=='invalid_state':states[0]=4
    elif damage=='missing_column':columns.pop()
    elif damage=='duplicate_column':columns[-1]=columns[0]
    elif damage=='has_occupied':columns[0][damage]=False
    else:columns[0][damage]+=1
    with pytest.raises(ValueError,match='OBSERVER_'):snapshot(config,states,columns)


@pytest.mark.parametrize('field,value',[('epoch',2),('map_revision',8),('observed_revision',10),
    ('last_integrated_stamp_ns',13),('geometry_hash','different'),('config_digest','different'),
    ('clock_domain','other')])
def test_every_page_carries_the_entire_lease_identity(tmp_path,field,value):
    config,_,_=configuration(tmp_path)
    original=identity(config);changed=dict(original,**{field:value})
    page=SimpleNamespace(status='OK',identity=SimpleNamespace(**changed),
        validity=SimpleNamespace(state='VALID',epoch=1,generation=2))
    with pytest.raises(ValueError,match='IDENTITY_OR_VALIDITY'):admit_response(page,original,2)


@pytest.mark.parametrize('change,expected',[
    ({}, True),
    ({'observed_revision': 10, 'last_integrated_stamp_ns': 13_000_000_000}, True),
    ({'map_revision': 8}, False),
    ({'epoch': 2}, False),
    ({'geometry_hash': 'different'}, False),
    ({'config_digest': 'different'}, False),
    ({'clock_domain': 'other'}, False),
    ({'observed_revision': 8}, False),
    ({'last_integrated_stamp_ns': 11_000_000_000}, False),
    ({'generation': 3}, False),
    ({'state': 'INVALID'}, False),
    ({'stale': True}, False),
    ({'age_ns': 1_000_000_001}, False),
    ({'age_ns': -1}, False),
])
def test_current_checks_content_version_without_renewing_original_evidence(tmp_path,change,expected):
    from uld_occ_observer.srv import GetSnapshot
    config,_,_=configuration(tmp_path)
    source=snapshot(config,[1]*64).source
    original=asdict(source)
    latest=identity(config)
    latest.update({key:value for key,value in change.items() if key in latest})
    lease=SimpleNamespace(status='OK',handle='latest',stale=change.get('stale',False),
        age_ns=change.get('age_ns',0),identity=SimpleNamespace(**latest),
        validity=SimpleNamespace(state=change.get('state','VALID'),epoch=latest['epoch'],
            generation=change.get('generation',source.validity_generation)))
    port=object.__new__(ObserverSnapshotPort)
    port._capture_fault=None;port.max_age_ms=1000
    port.services={'get_snapshot':GetSnapshot}
    requests=[];released=[]
    port._get=lambda request,deadline:(requests.append(request) or lease)
    port._release=lambda response:released.append(response.handle)
    assert port.current(source,deadline=time.monotonic()+1.) is expected
    assert asdict(source)==original,'later sensor activity must not renew original evidence'
    assert released==['latest']
    assert not requests[0].exact_revision and not requests[0].allow_stale
    assert requests[0].max_age_ms==1000


def test_release_acknowledgement_is_awaited_within_original_deadline():
    import threading
    pending=Future()
    port=object.__new__(ObserverSnapshotPort)
    port._release_lock=threading.Lock();port._pending_releases=[pending];port._capture_fault=None
    entered=threading.Event()
    original=pending.add_done_callback
    def register(callback):
        original(callback);entered.set()
    pending.add_done_callback=register
    worker=threading.Thread(target=lambda:(entered.wait(1.),pending.set_result(SimpleNamespace(status='OK'))))
    worker.start()
    try:port._await_releases(time.monotonic()+1.)
    finally:worker.join(timeout=1.)
    assert entered.is_set() and pending.done() and not worker.is_alive()


@pytest.mark.parametrize('completed',[False,True])
def test_unacknowledged_release_cannot_allocate_another_lease(completed):
    import threading
    pending=Future()
    if completed:pending.set_result(SimpleNamespace(status='INVALID_HANDLE'))
    port=object.__new__(ObserverSnapshotPort)
    port._release_lock=threading.Lock();port._pending_releases=[pending];port._capture_fault=None
    with pytest.raises(RuntimeError,match='RELEASE_BACKEND_BUSY'):
        port._await_releases(time.monotonic())


def test_issuance_retains_map_source_and_refuses_rebinding(tmp_path):
    from test_joint_operation_issuance import plan,registry
    config,_,_=configuration(tmp_path);source=snapshot(config,[1]*64).source
    p=plan();source=replace(source,scene_revision=p.identity.scene_revision,
        geometry_hash=p.identity.geometry_hash,map_revision=p.identity.map_revision)
    store=registry();store.issue(p,map_source=source)
    assert store.map_source(p)==source
    for altered in (None,replace(source,epoch=2)):
        with pytest.raises(ValueError,match='REBIND_REFUSED'):store.issue(p,map_source=altered)
    from luggage_planning.joint_operation.map_source import map_source_from_message,map_source_to_message
    assert map_source_from_message(map_source_to_message(source))==source
    assert not map_source_to_message(None).bound


@pytest.mark.parametrize('damage',['cursor','empty','short_done','not_done','array_shape','validity','generation'])
def test_page_completion_is_checked_behaviorally(tmp_path,damage):
    from uld_occ_observer.srv import ReadSurface
    from uld_occ_observer.msg import SnapshotIdentity,ValidityStamp
    config,_,_=configuration(tmp_path)
    page=ReadSurface.Response(status='OK',identity=SnapshotIdentity(**identity(config)),
        validity=ValidityStamp(state='VALID',epoch=1,generation=2),next_cursor=16,done=True,
        x=[x for y in range(4) for x in range(4)],y=[y for y in range(4) for x in range(4)],
        active_count=[4]*16,known_count=[4]*16,has_occupied=[False]*16,top_z=[0.]*16)
    if damage=='cursor':page.next_cursor=15
    elif damage=='empty':
        page.x=[];page.y=[];page.active_count=[];page.known_count=[];page.has_occupied=[];page.top_z=[]
    elif damage=='short_done':page.next_cursor=15;page.x.pop();page.y.pop();page.active_count.pop();page.known_count.pop();page.has_occupied.pop();page.top_z.pop()
    elif damage=='not_done':page.done=False
    elif damage=='array_shape':page.x.pop()
    elif damage=='validity':page.validity.state='BUILDING'
    elif damage=='generation':page.validity.generation=3
    port=object.__new__(ObserverSnapshotPort)
    port.config=config;port.clients={'read_surface':object()};port.services={'read_surface':ReadSurface}
    port.call=lambda *a,**kw:page
    with pytest.raises(ValueError,match='OBSERVER_'):
        port._pages('read_surface','lease',16,identity(config),2,100.)


def test_timed_out_acquire_stays_single_flight_and_releases_late_handle_once():
    from luggage_planning.joint_operation.prepare_node import JointOperationPrepareNode
    pending=Future();sent=[];released=[];records=[]
    client=SimpleNamespace(service_is_ready=lambda:True,
        call_async=lambda request:(sent.append(request) or pending))
    # The real port keys by client object; use a small hashable transport fake.
    class Client:
        service_is_ready=staticmethod(client.service_is_ready)
        call_async=staticmethod(client.call_async)
    client=Client()
    owner=SimpleNamespace(_inflight={},_rpc_completions={},_record=lambda kind,value:records.append(kind),
        get_logger=lambda:SimpleNamespace(error=lambda text:pytest.fail(text)))
    with pytest.raises(TimeoutError):
        JointOperationPrepareNode._call(owner,client,'request',time.monotonic()+.02,'observer',
                                       late_response=lambda response:released.append(response.handle))
    with pytest.raises(RuntimeError,match='BACKEND_BUSY'):
        JointOperationPrepareNode._call(owner,client,'another',time.monotonic()+1.,'observer')
    assert sent==['request'] and not released
    pending.set_result(SimpleNamespace(handle='real-late-lease'))
    assert released==['real-late-lease'] and records[-1]=='observer_late_response'


def test_completed_response_cannot_reopen_acquire_before_late_cleanup_consumption():
    from luggage_planning.joint_operation.prepare_node import JointOperationPrepareNode
    import threading
    pending=Future();sent=[];entered=threading.Event();allow_cleanup=threading.Event()
    class Client:
        def service_is_ready(self):return True
        def call_async(self,request):sent.append(request);return pending
    client=Client()
    owner=SimpleNamespace(_inflight={},_rpc_completions={},_record=lambda *args:None,
        get_logger=lambda:SimpleNamespace(error=lambda text:pytest.fail(text)))
    def clean(response):
        entered.set()
        assert allow_cleanup.wait(1.)
    with pytest.raises(TimeoutError):
        JointOperationPrepareNode._call(owner,client,'first',time.monotonic()+.01,'acquire',late_response=clean)
    worker=threading.Thread(target=lambda:pending.set_result(SimpleNamespace(handle='first')))
    worker.start()
    try:
        assert entered.wait(1.) and pending.done()
        with pytest.raises(RuntimeError,match='BACKEND_BUSY'):
            JointOperationPrepareNode._call(owner,client,'second',time.monotonic()+.1,'acquire')
        assert sent==['first']
    finally:
        allow_cleanup.set();worker.join(timeout=1.)
    assert not worker.is_alive() and owner._rpc_completions[client].is_set()


def test_release_still_runs_when_its_capture_fails_and_blocks_new_capture():
    from uld_occ_observer.srv import ReleaseSnapshot
    import threading
    port=object.__new__(ObserverSnapshotPort)
    port._pending_releases=[];port._release_lock=threading.Lock();port._capture_fault=None
    port.services={'release_snapshot':ReleaseSnapshot}
    sent=[];future=Future()
    port.clients={'release_snapshot':SimpleNamespace(call_async=lambda request:(sent.append(request) or future))}
    def fail_capture(*args):raise OSError('evidence unavailable')
    port.recorder=fail_capture
    port._release(SimpleNamespace(handle='must-release'))
    assert sent[0].handle=='must-release'
    future.set_result(ReleaseSnapshot.Response(status='OK'))
    with pytest.raises(RuntimeError,match='RELEASE_CAPTURE_FAILED'):
        port.capture(scene_revision='scene',request_id='next',expected_epoch=1,expected_revision=7,
                     deadline=time.monotonic()+1.)
