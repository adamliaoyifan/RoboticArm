"""Actual O observer and prepare owners; synthetic sensors, native MoveIt.

The explicitly frozen ROS clock makes the fixture acquisition stationary in
time. No future-dated clouds, G publications, controller or Gazebo is used.
"""
import copy
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from types import SimpleNamespace

import pytest

from test_joint_operation_endpoints_native import native_stack
from test_joint_operation_preparation_native import prepare_stack
from test_joint_operation_observer import configuration
from test_joint_operation_issuance_native import check

pytestmark=pytest.mark.skipif(not os.environ.get('JOINT_NATIVE_ENDPOINT_EXECUTABLE'),
    reason='explicit actual O/native preparation gate')


@pytest.fixture(scope='module')
def observer_owner(native_stack):
    import numpy as np
    from builtin_interfaces.msg import Time
    from rosgraph_msgs.msg import Clock
    from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
    from std_msgs.msg import Header
    from rclpy.serialization import serialize_message
    from uld_occ_observer.srv import IntegrateObservation,GetSnapshot,ReleaseSnapshot,GetValidity,ResetObservation
    from ament_index_python.packages import get_package_prefix
    stack=native_stack;directory=stack.directory/'observer';directory.mkdir()
    config,geometry,config_file=configuration(directory,size=(1.6,1.6,1.),floor=.6,resolution=.1)
    executable=Path(get_package_prefix('uld_occ_observer'))/'lib/uld_occ_observer/observer_node'
    args=[str(executable),'--ros-args','-p','geometry_file:='+str(geometry),'-p',
        'config_file:='+str(config_file),'-p','use_sim_time:=true']
    (directory/'launch.json').write_text(json.dumps(dict(args=args,sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
        domain=175,scope='synthetic empty-volume depth rays, actual O owner; frozen ROS clock12s'),indent=2))
    log=(directory/'observer.log').open('wb')
    process=subprocess.Popen(args,env=dict(os.environ,ROS_DOMAIN_ID='175',ROS_LOCALHOST_ONLY='1'),stdout=log,stderr=subprocess.STDOUT)
    clock_pub=stack.node.create_publisher(Clock,'/clock',10)
    clients={name:stack.node.create_client(service,'/uld_occ_observer/'+name) for name,service in
        [('integrate_observation',IntegrateObservation),('get_snapshot',GetSnapshot),('release_snapshot',ReleaseSnapshot),
         ('get_validity',GetValidity),('reset_observation',ResetObservation)]}
    def call(name,request):
        stack.record('observer',name+'_request',serialize_message(request))
        future=clients[name].call_async(request);deadline=time.monotonic()+8.
        while not future.done() and time.monotonic()<deadline:time.sleep(.005)
        assert future.done(),'actual O RPC deadline: '+name
        response=future.result();stack.record('observer',name+'_response',serialize_message(response))
        return response
    try:
        for client in clients.values():assert client.wait_for_service(timeout_sec=10.)
        deadline=time.monotonic()+3.
        while clock_pub.get_subscription_count()==0 and time.monotonic()<deadline:time.sleep(.01)
        assert clock_pub.get_subscription_count()>0
        for _ in range(10):clock_pub.publish(Clock(clock=Time(sec=12)));time.sleep(.01)
        # A downward optical view of an empty measured volume: every ray ends
        # outside the hull below its floor. Dense end rays cover all active
        # cells; the native observer decides free/unknown without a floor prior.
        xy=np.arange(-4.,4.001,.04,dtype=np.float32)
        points=np.array([(x,-y,1.8) for x in xy for y in xy],dtype=np.float32)
        cloud=create_cloud_xyz32(Header(stamp=Time(sec=12),frame_id='optical'),points)
        request=IntegrateObservation.Request(sensor_id='fixture_depth',observation_id='empty-view',clock_domain='ros',
            geometry_hash=config.descriptor()['geometry_hash'],epoch=1,cloud=cloud)
        request.uld_from_optical.header=Header(stamp=Time(sec=12),frame_id='container_link')
        request.uld_from_optical.child_frame_id='optical'
        request.uld_from_optical.transform.translation.z=2.
        request.uld_from_optical.transform.rotation.x=1.
        request.uld_from_optical.transform.rotation.w=0.
        integrated=call('integrate_observation',request)
        assert integrated.receipt.status=='INTEGRATED',integrated.receipt.status
        lease=call('get_snapshot',GetSnapshot.Request(max_age_ms=1000))
        assert lease.status=='OK' and lease.validity.state=='VALID',str(lease)
        assert call('release_snapshot',ReleaseSnapshot.Request(handle=lease.handle)).status=='OK'
        state=SimpleNamespace(lease=lease,geometry=geometry,config=config_file,call=call,directory=directory,
                              stamp=12,epoch=lease.identity.epoch,sequence=0)
        def acquire():
            latest=call('get_snapshot',GetSnapshot.Request(expected_epoch=state.epoch,max_age_ms=1000))
            assert latest.status=='OK',str(latest)
            assert call('release_snapshot',ReleaseSnapshot.Request(handle=latest.handle)).status=='OK'
            state.lease=latest
            return latest
        def observe(*,partial=False):
            advance_clock(state.stamp+1)
            state.sequence+=1
            current=copy.deepcopy(request)
            current.observation_id='observed-'+str(state.sequence)
            current.epoch=state.epoch
            if partial:current.cloud=create_cloud_xyz32(current.cloud.header,points[len(points)//2:len(points)//2+1])
            current.cloud.header.stamp.sec=state.stamp
            current.uld_from_optical.header.stamp.sec=state.stamp
            response=call('integrate_observation',current)
            assert response.receipt.status=='INTEGRATED',response.receipt.status
            return acquire()
        def advance_clock(stamp):
            assert stamp>state.stamp
            state.stamp=stamp
            for _ in range(10):clock_pub.publish(Clock(clock=Time(sec=stamp)));time.sleep(.01)
        state.observe,state.acquire,state.advance_clock=observe,acquire,advance_clock
        yield state
    finally:
        for client in clients.values():stack.node.destroy_client(client)
        stack.node.destroy_publisher(clock_pub)
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=5.)
            except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5.)
        log.close()
        (directory/'teardown.json').write_text(json.dumps(dict(exit_code=process.returncode,worker_joined=True)))
        assert process.returncode==0,'O owner did not stop cleanly'


@pytest.mark.parametrize('prepare_stack',['observer'],indirect=True)
def test_actual_o_default_returns_complete_plan_without_g(prepare_stack):
    f=prepare_stack
    assert f.root.map_backend=='observer' and f.root.observer is not None
    assert not hasattr(f.root,'volume_client') and not hasattr(f.root,'map_parameters')
    request=copy.deepcopy(f.request);request.request_id='actual-o-full-prepare'
    response=f.call(request)
    assert response.success,response.reason+' '+response.trace_json
    assert len(response.plan.legs)==6 and len(response.plan.barriers)==2
    provenance=check(f,response.plan)
    assert provenance.current and provenance.map_source.bound
    assert provenance.map_source.producer=='observer'
    assert provenance.map_source.epoch==f.observer.lease.identity.epoch
    assert provenance.map_source.map_revision==f.observer.lease.identity.map_revision
    assert provenance.map_source.config_digest==f.observer.lease.identity.config_digest
    assert f.root.store.current() is None,'no G topics entered the default root'
    (f.directory/'o_positive.json').write_text(json.dumps(dict(plan_digest=response.plan.plan_digest,
        certificate=response.plan.certificate_id,legs=len(response.plan.legs),map_revision=provenance.map_source.map_revision,
        epoch=provenance.map_source.epoch,source='actual O services; actual native full operation certification'),indent=2))


@pytest.mark.parametrize('prepare_stack',['observer'],indirect=True)
@pytest.mark.parametrize('damage',['epoch','revision','geometry'])
def test_actual_o_wrong_identity_never_issues(prepare_stack,damage):
    f=prepare_stack;request=copy.deepcopy(f.request);request.request_id='wrong-o-'+damage
    if damage=='epoch':request.expected_map_epoch+=1
    elif damage=='revision':request.expected_map_revision+=1
    else:request.geometry_hash='different'
    response=f.call(request)
    assert not response.success and not response.plan.legs
    assert all(entry.plan.identity.request_id!=request.request_id for entry in f.root.issued._entries.values())


def refresh_request(fixture):
    fixture.request.expected_map_epoch=fixture.observer.lease.identity.epoch
    fixture.request.expected_map_revision=fixture.observer.lease.identity.map_revision


@pytest.mark.parametrize('prepare_stack',['observer'],indirect=True)
def test_current_guard_detects_changed_map_instead_of_transition_time_validity(prepare_stack):
    from uld_occ_observer.srv import GetValidity
    f=prepare_stack
    original=f.root.observer.capture(scene_revision='scene',request_id='guard',
        expected_epoch=f.request.expected_map_epoch,expected_revision=f.request.expected_map_revision,
        deadline=time.monotonic()+2.)
    f.observer.observe();refresh_request(f)
    transition=f.observer.call('get_validity',GetValidity.Request())
    assert transition.observed_revision==original.source.observed_revision
    assert f.observer.lease.identity.observed_revision>original.source.observed_revision
    assert f.observer.lease.identity.map_revision>original.source.map_revision
    assert f.root.observer.current(original.source,deadline=time.monotonic()+2.) is False


@pytest.mark.parametrize('prepare_stack',['observer'],indirect=True)
def test_changed_o_after_native_certification_requires_recertification_before_issuance(prepare_stack,monkeypatch):
    f=prepare_stack;f.state.scene_calls=0
    original=f.root.observed_contacts.retain
    changed=[]
    def advance(*args,**kwargs):
        result=original(*args,**kwargs)
        if kwargs.get('expected_observation') is not None and not changed:
            before=f.observer.lease.identity.map_revision
            latest=f.observer.observe()
            assert latest.identity.map_revision>before,'must exercise changed content'
            changed.append(latest.identity.map_revision)
        return result
    monkeypatch.setattr(f.root.observed_contacts,'retain',advance)
    try:
        request=copy.deepcopy(f.request);request.request_id='late-o-change'
        response=f.call(request)
        assert f.state.scene_calls==4,'initial certification plus a complete fresh-map certification'
        assert response.success and response.reason=='PLAN_READY',response.reason
        assert sum(row[0]=='map_recertification' for row in json.loads(response.trace_json))==1
        assert len(response.plan.legs)==6 and response.plan.map_revision==changed[0]
        provenance=check(f,response.plan)
        assert provenance.current and provenance.map_source.map_revision==changed[0]
    finally:
        refresh_request(f)


@pytest.mark.parametrize('prepare_stack',['observer'],indirect=True)
def test_real_owner_config_mismatch_is_refused(prepare_stack):
    f=prepare_stack;original=f.root.observer.config
    f.root.observer.config=replace(original,config_digest='a'*64)
    try:
        response=f.call(copy.deepcopy(f.request))
        assert not response.success and 'LEASE_CONFIGURATION' in response.reason and not response.plan.legs
    finally:f.root.observer.config=original


@pytest.mark.parametrize('prepare_stack',['observer'],indirect=True)
def test_actual_o_stale_acquisition_is_refused_without_relabelling(prepare_stack):
    f=prepare_stack;f.observer.advance_clock(f.observer.stamp+2)
    try:
        response=f.call(copy.deepcopy(f.request))
        assert not response.success and not response.plan.legs
        assert f.observer.lease.identity.last_integrated_stamp_ns < f.observer.stamp*10**9
    finally:
        f.observer.observe();refresh_request(f)


@pytest.mark.parametrize('prepare_stack',['observer'],indirect=True)
def test_reset_without_new_observation_cannot_prepare(prepare_stack):
    from uld_occ_observer.srv import ResetObservation
    f=prepare_stack
    response=f.observer.call('reset_observation',ResetObservation.Request(expected_epoch=f.observer.epoch,
        reset_id='operation-source-reset'))
    assert response.status=='OK',str(response)
    f.observer.epoch=response.epoch
    try:
        request=copy.deepcopy(f.request);request.expected_map_epoch=response.epoch
        response=f.call(request)
        assert not response.success and not response.plan.legs
    finally:
        f.observer.observe(partial=True);refresh_request(f)
    captured=f.root.observer.capture(scene_revision='scene',request_id='partial-real-o',
        expected_epoch=f.request.expected_map_epoch,expected_revision=f.request.expected_map_revision,
        deadline=time.monotonic()+2.)
    from luggage_planning.placement_prior.volume import blocked_volume_runs
    assert captured.volume.states.count(0)>0 and captured.volume.states.count(1)>0
    assert blocked_volume_runs(captured.volume,captured.surface),'unknown active volume must block motion'


@pytest.mark.parametrize('prepare_stack',['observer'],indirect=True)
def test_real_owner_repeated_unchanged_observation_keeps_original_evidence_current(prepare_stack):
    f=prepare_stack
    # Repeated identical rays eventually reach the owner's configured probability
    # clamp. Bound the workload; failure to obtain unchanged content is a failure
    # to exercise this case, rather than a vacuous successful liveness check.
    for _ in range(20):
        before=f.observer.lease.identity.map_revision
        latest=f.observer.observe()
        if latest.identity.map_revision==before:
            break
    else:
        pytest.fail('unchanged-content integration not observed in 20 identical-ray samples')
    refresh_request(f)
    original=f.root.observer.capture(scene_revision='scene',request_id='unchanged-real-o',
        expected_epoch=f.request.expected_map_epoch,expected_revision=f.request.expected_map_revision,
        deadline=time.monotonic()+2.)
    latest=f.observer.observe()
    assert latest.identity.map_revision==original.source.map_revision
    assert latest.identity.observed_revision>original.source.observed_revision
    assert latest.identity.last_integrated_stamp_ns>original.source.last_integrated_stamp_ns
    assert f.root.observer.current(original.source,deadline=time.monotonic()+2.) is True
    assert original.volume.source==original.source
    assert original.source.last_integrated_stamp_ns<latest.identity.last_integrated_stamp_ns
    refresh_request(f)
