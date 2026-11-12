import copy
import importlib.util
from pathlib import Path
import time

import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import Image, CameraInfo
from luggage_msgs.msg import CargoObservationPartitionV2
from luggage_msgs.srv import GetCargoObservationV2

from cargo_observation_case_capture import CaseCapture
from luggage_perception.cargo_observation_service import (
    CargoObservationBuffer, CargoObservationService, source_sha256,
)


@pytest.fixture
def case(request,tmp_path):
    return CaseCapture(request.node.nodeid,tmp_path)


def sources(stamp_ns=1_000_000_001,encoding='16UC1',bigendian=False):
    result = {}
    for kind in ('rgb','depth','mask'):
        image = Image();image.header.frame_id='camera_color_optical_frame'
        image.header.stamp.sec,image.header.stamp.nanosec=divmod(stamp_ns,1_000_000_000)
        image.height,image.width=2,3
        image.is_bigendian=bigendian
        if kind == 'rgb':
            image.encoding='rgb8';array=np.arange(18,dtype=np.uint8).reshape(2,3,3)
        elif kind == 'mask':
            image.encoding='mono16';array=np.array([[0,1,1],[2,2,0]],dtype='>u2' if bigendian else '<u2')
        else:
            image.encoding=encoding
            dt = ('>' if bigendian else '<')+('u2' if encoding=='16UC1' else 'f4')
            array=np.full((2,3),1000 if encoding=='16UC1' else 1.,dtype=dt)
        image.step=array.strides[0];image.data=array.tobytes()
        result[kind]=image
    info=CameraInfo();info.header=copy.deepcopy(result['depth'].header)
    info.height,info.width=2,3
    info.k=list(map(float,[2,0,1,0,2,0,0,0,1]));info.r=list(map(float,[1,0,0,0,1,0,0,0,1]))
    info.p=list(map(float,[2,0,1,0,0,2,0,0,0,0,1,0]))
    result['calibration']=info
    return result


def transform(stamp_ns=1_000_000_001):
    message=TransformStamped();message.header.frame_id='container_link'
    message.header.stamp.sec,message.header.stamp.nanosec=divmod(stamp_ns,1_000_000_000)
    message.child_frame_id='camera_color_optical_frame'
    message.transform.rotation.w=1.
    message.transform.translation.x=.25
    return message


def new_buffer():
    return CargoObservationBuffer(max_frames=2,max_bytes=100_000,max_pixels=100,max_age_ns=1_000_000_000)


def add(case,buffer,values):
    for kind,message in values.items():
        case.message('ingress_'+kind,message)
        buffer.add(kind,message)


def capture(case,buffer,stamp=1_000_000_001,tf=None,clock=None):
    port=CargoObservationService(buffer=buffer,sensor_id='test_rgbd',stride=1,
        now_ns=clock or (lambda:1_000_000_002),
        lookup_tf=lambda frame,time:tf or transform(time),recorder=case.message)
    request=GetCargoObservationV2.Request(request_id='capture-1',acquisition_stamp_ns=stamp)
    return port.handle(request,GetCargoObservationV2.Response())


@pytest.mark.parametrize('encoding,bigendian',[
    ('16UC1',False),('16UC1',True),('32FC1',False),('32FC1',True)])
def test_lossless_encodings_and_ros_roundtrip(case,encoding,bigendian):
    buffer=new_buffer();values=sources(encoding=encoding,bigendian=bigendian)
    add(case,buffer,values)
    response=capture(case,buffer)
    assert response.valid,response.reason
    observation=deserialize_message(serialize_message(response.observation),CargoObservationPartitionV2)
    xyz=np.frombuffer(bytes(observation.cloud.data),dtype='<f4').reshape(-1,3)
    np.testing.assert_array_equal(xyz,[[-.5,0,1],[0,0,1],[.5,0,1],[-.5,.5,1],[0,.5,1],[.5,.5,1]])
    assert list(observation.source_pixel_indices)==list(range(6))
    assert list(observation.frame_instance_labels)==[0,1,1,2,2,0]
    assert observation.container_from_optical.transform.translation.x==.25
    assert observation.binding_sha256==response.observation.binding_sha256
    expected_sources=[source_sha256(values[k]) for k in ('rgb','depth','mask','calibration')]
    assert list(observation.source_sha256)==expected_sources+[source_sha256(transform())]


def test_padded_rows_are_decoded_without_treating_padding_as_pixels(case):
    buffer=new_buffer();values=sources()
    for kind in ('rgb','depth','mask'):
        image=values[kind];raw=bytes(image.data);old_step=image.step
        image.step+=4
        image.data=b''.join(raw[y*old_step:(y+1)*old_step]+b'abcd' for y in range(image.height))
    add(case,buffer,values);response=capture(case,buffer)
    assert response.valid,response.reason
    assert list(response.observation.frame_instance_labels)==[0,1,1,2,2,0]


def test_source_mutation_and_response_mutation_do_not_change_frozen_inputs(case):
    buffer=new_buffer();values=sources();add(case,buffer,values)
    first=capture(case,buffer);assert first.valid
    values['depth'].data=bytes(len(values['depth'].data))
    original_hash=first.observation.binding_sha256
    first.observation.rgb.data=bytes(len(first.observation.rgb.data))
    second=capture(case,buffer)
    assert second.valid and second.observation.binding_sha256==original_hash
    assert bytes(second.observation.rgb.data)==bytes(range(18))


def test_semantically_identical_repeated_message_is_not_a_conflict(case):
    buffer=new_buffer();values=sources();add(case,buffer,values)
    for _ in range(3):
        add(case,buffer,copy.deepcopy(values))
    assert capture(case,buffer).valid


def test_conflicting_same_acquisition_never_silently_selects_latest(case):
    buffer=new_buffer();values=sources();add(case,buffer,values)
    values['rgb'].data[0]=99;add(case,buffer,{'rgb':values['rgb']})
    response=capture(case,buffer)
    assert not response.valid and response.reason=='CONFLICTING_ACQUISITION'


def test_evicted_conflict_cannot_reenter_as_an_unconflicted_frame(case):
    buffer=new_buffer();first=sources();add(case,buffer,first)
    first['rgb'].data[0]=99;add(case,buffer,{'rgb':first['rgb']})
    add(case,buffer,sources(1_000_000_002));add(case,buffer,sources(1_000_000_003))
    with pytest.raises(ValueError,match='OBSERVATION_EVICTED'):add(case,buffer,first)
    assert not capture(case,buffer).valid


@pytest.mark.parametrize('problem',[
    'missing_source','different_stamp','different_frame','bad_encoding','truncated_rows',
    'bad_dimensions','distortion','skew','roi','bad_tf_frame','bad_tf_stamp','bad_tf_quaternion'])
def test_incomplete_or_incompatible_sources_return_no_observation(case,problem):
    buffer=new_buffer();values=sources();tf=transform()
    if problem=='missing_source':del values['mask']
    if problem=='different_stamp':values['mask'].header.stamp.nanosec+=1
    if problem=='different_frame':values['mask'].header.frame_id='another_optical'
    if problem=='bad_encoding':values['depth'].encoding='8UC1'
    if problem=='truncated_rows':values['rgb'].data=bytes(1)
    if problem=='bad_dimensions':values['mask'].width=2
    if problem=='distortion':values['calibration'].d=[.1]
    if problem=='skew':values['calibration'].k[1]=.1
    if problem=='roi':values['calibration'].roi.x_offset=1
    if problem=='bad_tf_frame':tf.header.frame_id='world'
    if problem=='bad_tf_stamp':tf.header.stamp.nanosec+=1
    if problem=='bad_tf_quaternion':tf.transform.rotation.w=0.
    add(case,buffer,values);response=capture(case,buffer,tf=tf)
    assert not response.valid
    assert not response.observation.binding_sha256
    assert response.observation.cloud.width==0


@pytest.mark.parametrize('stamp,clock',[(0,1_000_000_002),(1_000_000_001,3_000_000_002),
                                      (1_000_000_001,1_000_000_000)])
def test_missing_expired_or_future_acquisition_is_not_accepted(case,stamp,clock):
    buffer=new_buffer();add(case,buffer,sources())
    response=capture(case,buffer,stamp=stamp,clock=lambda:clock)
    assert not response.valid


def test_age_is_checked_again_after_tf_lookup(case):
    buffer=new_buffer();add(case,buffer,sources())
    times=iter([1_000_000_002,3_000_000_002])
    response=capture(case,buffer,clock=lambda:next(times))
    assert not response.valid and response.reason=='OBSERVATION_EXPIRED_OR_FUTURE'


def test_source_byte_budget_and_pixel_budget_are_enforced(case):
    source=sources()['rgb']
    case.message('oversized_input',source)
    byte_buffer=CargoObservationBuffer(max_frames=2,max_bytes=1,max_pixels=100,max_age_ns=10)
    with pytest.raises(ValueError,match='BYTE_LIMIT'):byte_buffer.add('rgb',source)
    pixel_buffer=CargoObservationBuffer(max_frames=2,max_bytes=10000,max_pixels=5,max_age_ns=10)
    with pytest.raises(ValueError,match='SIZE_LIMIT'):pixel_buffer.add('rgb',source)


def test_total_byte_budget_evicts_old_frames_before_frame_count_limit(case):
    values=sources();total=sum(len(serialize_message(m)) for m in values.values())
    buffer=CargoObservationBuffer(max_frames=100,max_bytes=total+1,max_pixels=100,max_age_ns=10)
    add(case,buffer,values);add(case,buffer,sources(1_000_000_002))
    assert not capture(case,buffer).valid
    assert capture(case,buffer,stamp=1_000_000_002).valid


def test_conflicting_update_during_tf_lookup_revokes_captured_inputs(case):
    values=sources();buffer=new_buffer();add(case,buffer,values)
    def lookup(frame,stamp):
        changed=copy.deepcopy(values['rgb']);changed.data[0]=17
        add(case,buffer,{'rgb':changed})
        return transform(stamp)
    port=CargoObservationService(buffer=buffer,sensor_id='test_rgbd',stride=1,
        now_ns=lambda:1_000_000_002,lookup_tf=lookup,recorder=case.message)
    response=port.handle(GetCargoObservationV2.Request(request_id='race',
        acquisition_stamp_ns=1_000_000_001),GetCargoObservationV2.Response())
    assert not response.valid and response.reason=='CONFLICTING_ACQUISITION'
    assert not response.observation.binding_sha256


def test_actual_node_subscriptions_tf_and_service_roundtrip(case):
    """Real DDS/Image/TF/IDL boundary, isolated context, no robot/map ports."""
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from tf2_ros import TransformBroadcaster
    node_path=Path(__file__).parents[1]/'scripts/cargo_observation_node.py'
    spec=importlib.util.spec_from_file_location('cargo_observation_node',node_path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    context=Context();rclpy.init(context=context,domain_id=177)
    executor=SingleThreadedExecutor(context=context)
    owner=client_node=None
    try:
        owner=module.CargoObservationNode(context=context,
            parameter_overrides=[Parameter('max_age_ms',value=10000)])
        owner.port.recorder=case.message
        client_node=Node('cargo_observation_test',context=context)
        executor.add_node(owner);executor.add_node(client_node)
        publishers={kind:client_node.create_publisher(CameraInfo if kind=='calibration' else Image,
            topic,10) for kind,topic in (
                ('rgb','/luggage/preprocessed/camera/color/image'),
                ('depth','/luggage/preprocessed/camera/depth/image'),
                ('mask','/luggage/semantic/instance_mask'),
                ('calibration','/luggage/preprocessed/camera/depth/camera_info'))}
        broadcaster=TransformBroadcaster(client_node)
        stamp=client_node.get_clock().now().nanoseconds
        values=sources(stamp);tf=transform(stamp)
        for kind,message in values.items():case.message('published_'+kind,message)
        case.message('published_tf',tf)
        deadline=time.monotonic()+6.;ready=False
        while time.monotonic()<deadline:
            broadcaster.sendTransform(tf)
            for kind,publisher in publishers.items():publisher.publish(values[kind])
            executor.spin_once(timeout_sec=.02)
            try:
                owner.buffer.exact(stamp,owner.get_clock().now().nanoseconds)
                owner._lookup_tf(values['depth'].header.frame_id,stamp)
                ready=True;break
            except Exception:pass
        assert ready,'Real source/TF readiness did not occur'
        client=client_node.create_client(GetCargoObservationV2,'/cargo_observation/get_observation')
        assert client.wait_for_service(timeout_sec=1.)
        request=GetCargoObservationV2.Request(request_id='dds-capture',acquisition_stamp_ns=stamp)
        case.message('dds_request',request)
        future=client.call_async(request)
        executor.spin_until_future_complete(future,timeout_sec=3.)
        assert future.done(),'Actual service did not terminate'
        response=future.result();case.message('dds_response',response)
        assert response.valid,response.reason
        assert list(response.observation.frame_instance_labels)==[0,1,1,2,2,0]
        assert response.observation.container_from_optical.header.stamp==values['depth'].header.stamp
        assert not owner.count_publishers('/joint_trajectory_controller/joint_trajectory')
    finally:
        executor.shutdown()
        if client_node:client_node.destroy_node()
        if owner:owner.destroy_node()
        rclpy.shutdown(context=context)
        case.value('teardown',dict(executor_shutdown=True,nodes_destroyed=True,context_shutdown=True))
