"""Exercise actual source/installed sensor callback and exact ROS TF buffer."""
import copy
import importlib.util
import json
import math
import os
from pathlib import Path

from geometry_msgs.msg import TransformStamped
import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from pickup_continuation_fixtures import case_archive
from test_depth_dropout_fixture import config_record


@pytest.fixture
def actual_node(case_archive,monkeypatch):
    path=Path(os.environ.get('ELFIN_TEST_DEPTH_REPUBLISHER_SCRIPT',
        Path(__file__).parents[1]/'scripts/depth_image_republisher.py'))
    if os.environ.get('ELFIN_TEST_DEPTH_REPUBLISHER_SCRIPT'):
        assert '/install-production/' in str(path.resolve())
    spec=importlib.util.spec_from_file_location('actual_depth_fixture_adapter',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    nodes=[];callbacks=[]
    original=Node.create_subscription
    def subscribe(self,kind,topic,callback,*args,**kwargs):
        callbacks.append((topic,callback))
        return original(self,kind,topic,callback,*args,**kwargs)
    monkeypatch.setattr(Node,'create_subscription',subscribe)
    rclpy.init(domain_id=179)
    def create(enabled=True):
        root=case_archive.directory
        config=root/'config.json';config.write_text(json.dumps(config_record()))
        for key in ('ELFIN_SIM_DEPTH_DROPOUT_CONFIG','ELFIN_SIM_DEPTH_DROPOUT_EVIDENCE',
                    'ELFIN_SUCTION_INPUT_CAPTURE','ELFIN_AMBIGUITY_CAPTURE'):
            monkeypatch.delenv(key,raising=False)
        if enabled:
            monkeypatch.setenv('ELFIN_SIM_DEPTH_DROPOUT_CONFIG',str(config))
            monkeypatch.setenv('ELFIN_SIM_DEPTH_DROPOUT_EVIDENCE',str(root/'fixture'))
            monkeypatch.setenv('ELFIN_SUCTION_INPUT_CAPTURE',str(root/'suction_inputs'))
            monkeypatch.setenv('ELFIN_AMBIGUITY_CAPTURE',str(root/'detector_inputs'))
        node=module.DepthImageRepublisher();nodes.append(node)
        published=[];publish=node._pub.publish
        def observe(message):
            published.append(copy.deepcopy(message));publish(message)
        monkeypatch.setattr(node._pub,'publish',observe)
        if node._fixture is not None:node._fixture.publish=observe
        callback=next(cb for topic,cb in callbacks if topic=='/camera/depth/image_meters')
        return node,callback,published,callbacks
    try:
        yield create
    finally:
        for node in nodes:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def image(stamp):
    message=Image(height=16,width=24,encoding='32FC1',step=24*4,
        data=np.full((16,24),1.,dtype='<f4').tobytes())
    message.header.stamp.sec=stamp;message.header.frame_id='camera_depth_optical_frame'
    return message


def transform(stamp,angle=0.):
    value=TransformStamped();value.header.stamp.sec=stamp
    value.header.frame_id='world';value.child_frame_id='camera_depth_optical_frame'
    value.transform.rotation.y=math.sin(angle/2);value.transform.rotation.w=math.cos(angle/2)
    return value


def test_disabled_actual_callback_identical_without_tf_or_timer(actual_node,case_archive):
    node,callback,published,subscriptions=actual_node(False)
    message=image(1);callback(message)
    assert node._fixture is None and len(list(node.timers))==0
    assert [topic for topic,_ in subscriptions]==['/camera/depth/image_meters']
    assert len(published)==1 and published[0].header==message.header
    assert bytes(published[0].data)==np.full((16,24),1000,dtype='<u2').tobytes()
    case_archive.save('published',dict(stamp_ns=10**9,bytes=len(published[0].data)))


def test_actual_callback_waits_for_exact_tf_then_changes_only_declared_pixels(actual_node,case_archive):
    node,callback,published,_=actual_node()
    # Only a preceding transform is available: no latest/nearest substitution.
    node._fixture.buffer.set_transform(transform(1),'test')
    message=image(2);callback(message)
    assert not published and len(node._fixture.pending)==1
    node._fixture.buffer.set_transform(transform(2),'test');node._fixture.flush()
    assert len(published)==1 and published[0].header==message.header
    mm=np.frombuffer(bytes(published[0].data),dtype='<u2').reshape(16,24)
    yy,xx=np.indices(mm.shape);mask=(yy%8<2)&(xx%8<2)
    assert not mm[mask].any() and np.all(mm[~mask]==1000)
    node._fixture.buffer.set_transform(transform(3,.4),'test');callback(image(3))
    assert len(published)==2 and bytes(published[1].data)==np.full((16,24),1000,dtype='<u2').tobytes()
    np.savez_compressed(case_archive.directory/'published.npz',active=mm,
                        outside=np.frombuffer(bytes(published[1].data),dtype='<u2'))


@pytest.mark.parametrize('cause',['timeout','overflow','frame','zero_stamp'])
def test_missing_exact_tf_is_explicit_passthrough(actual_node,case_archive,cause):
    node,callback,published,_=actual_node();port=node._fixture
    message=image(1)
    if cause=='frame':message.header.frame_id='different_optical'
    if cause=='zero_stamp':message.header.stamp.sec=0
    callback(message)
    if cause=='timeout':
        msg,begin=port.pending.popleft();port.pending.append((msg,begin-1.));port.flush()
    elif cause=='overflow':
        for stamp in range(2,7):callback(image(stamp))
    assert published and bytes(published[0].data)==np.full((16,24),1000,dtype='<u2').tobytes()
    expected=dict(timeout='EXACT_TF_TIMEOUT',overflow='TF_QUEUE_OVERFLOW',
                  frame='OPTICAL_FRAME_MISMATCH',zero_stamp='ZERO_STAMP_NOT_EXACT')[cause]
    assert port.evidence.counts[expected]==1
    case_archive.save('counts',port.evidence.counts)


def test_diagnostic_capture_failure_does_not_change_injected_image(actual_node,case_archive,monkeypatch):
    node,callback,published,_=actual_node();port=node._fixture
    port.buffer.set_transform(transform(1),'test')
    def broken(*args,**kwargs):
        raise OSError('constructed capture failure')
    monkeypatch.setattr(port.evidence,'record',broken)
    callback(image(1))
    assert len(published)==1 and port.error_count==1
    depth=np.frombuffer(bytes(published[0].data),dtype='<u2').reshape(16,24)
    yy,xx=np.indices(depth.shape);mask=(yy%8<2)&(xx%8<2)
    assert not depth[mask].any() and np.all(depth[~mask]==1000)
    port.close()
    summary=json.loads((case_archive.directory/'fixture/summary.json').read_text())
    assert not summary['complete'] and any('capture failure' in s for s in summary['missing'])


def test_pending_teardown_after_context_shutdown_writes_summary_without_publishing(actual_node,case_archive):
    node,callback,published,_=actual_node()
    callback(image(1));assert len(node._fixture.pending)==1
    rclpy.shutdown();node.destroy_node()
    summary=json.loads((case_archive.directory/'fixture/summary.json').read_text())
    trace=[json.loads(line) for line in (case_archive.directory/'fixture/trace.jsonl').read_text().splitlines()]
    assert not published and summary['counts']['TEARDOWN_PENDING_TF']==1
    assert len(trace)==1 and not trace[0]['published']


def test_actual_adapter_pins_existing_detector_anomaly_without_selecting_a_pick(actual_node,case_archive):
    from luggage_perception.ambiguity_capture import FrameCapture
    node,callback,published,_=actual_node();port=node._fixture
    port.buffer.set_transform(transform(1),'test');callback(image(1))
    capture=FrameCapture(case_archive.directory/'detector_inputs')
    capture.consume(10**9,dict(instance_id='observed-box',generation=2),'anomaly',
        dict(detections=[],depth=np.full((16,24),1000,dtype='<u2')),{},now=0.)
    capture.flush()
    port.flush();port.close()
    summary=json.loads((case_archive.directory/'fixture/summary.json').read_text())
    assert not summary['selected'] and len(published)==1
    row=next(row for row in summary['diagnostics'] if row['kind']=='detector_anomaly')
    assert row['stamp_ns']==10**9 and row['published']
    arrays=np.load(case_archive.directory/'fixture'/row['file'])
    assert np.all(arrays['before_mm']==1000)
    assert arrays['after_mm'].tobytes()==bytes(published[0].data)
