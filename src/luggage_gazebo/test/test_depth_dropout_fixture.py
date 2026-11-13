"""Finite simulation workload conformance, independent of box/perception truth."""
from dataclasses import asdict
import hashlib
import json
import math

import numpy as np
import pytest

from luggage_gazebo.depth_dropout_fixture import DepthDropoutConfig, apply_dropout, optical_axis_world
from luggage_gazebo.depth_dropout_evidence import DepthDropoutEvidence
from pickup_continuation_fixtures import case_archive


def config_record():
    return dict(schema='simulation_depth_dropout_v1', model_hash='a'*64, scene_revision='b'*32,
        reference_observation='constructed-conformance-only', reference_input_sha256='c'*64,
        world_frame='world', optical_frame='camera_depth_optical_frame',
        reference_axis_world=[0.,0.,1.], pitch_pixels=8, square_side_pixels=2,
        active_axis_cone_rad=math.radians(10.))


def test_exact_pixel_loss_preserves_existing_zeros_and_other_depths(case_archive):
    config = DepthDropoutConfig.from_record(config_record())
    before = np.arange(16*24, dtype=np.uint16).reshape(16,24)
    after, effect = apply_dropout(before, config, [0.,0.,1.])
    np.savez_compressed(case_archive.directory/'arrays.npz', before=before, after=after)
    case_archive.save('config', asdict(config));case_archive.save('effect', effect)
    y,x = np.indices(before.shape)
    mask = (y%8<2)&(x%8<2)
    assert not after[mask].any()
    assert np.array_equal(after[~mask],before[~mask])
    assert effect['changed_pixels']==int(np.count_nonzero(before[mask]))
    assert np.array_equal(before,np.arange(16*24,dtype=np.uint16).reshape(16,24))


def test_axis_cone_ignores_roll_and_outside_is_byte_identical(case_archive):
    config = DepthDropoutConfig.from_record(config_record())
    before = np.full((16,24),1234,dtype='<u2')
    axes = [optical_axis_world(q) for q in ((0.,0.,0.,1.),(0.,0.,1.,0.),
                                             (0.,math.sin(.2),0.,math.cos(.2)))]
    results = [apply_dropout(before,config,axis) for axis in axes]
    case_archive.save('effects',[effect for _,effect in results])
    assert np.array_equal(results[0][0],results[1][0])
    assert results[0][1]['active'] and results[1][1]['active']
    assert not results[2][1]['active'] and results[2][0].tobytes()==before.tobytes()


@pytest.mark.parametrize('key,value', [('schema','unknown'),('world_frame',''),
    ('reference_axis_world',[0.,0.,2.]),('reference_axis_world',[float('nan'),0.,1.]),
    ('pitch_pixels',True),('square_side_pixels',8),('active_axis_cone_rad',0.),
    ('reference_input_sha256','not-a-hash')])
def test_invalid_explicit_configuration_is_rejected(case_archive,key,value):
    record = config_record();record[key]=value
    # Keep even non-finite invalid JSON as the exact input, not a cleaned substitute.
    (case_archive.directory/'input.json').write_text(json.dumps(record))
    with pytest.raises((ValueError,TypeError)):
        DepthDropoutConfig.from_record(record)


def event(stamp):
    return dict(status='saved',identity=['observed-box',2],stamp_ns=stamp,case='case-000')


def test_selected_event_pins_exact_original_and_injected_frame(case_archive):
    root=case_archive.directory;selection=root/'events.jsonl'
    capture=DepthDropoutEvidence(root/'fixture',selection)
    before=np.full((16,24),1000,dtype='<u2')
    after,effect=apply_dropout(before,DepthDropoutConfig.from_record(config_record()),[0.,0.,1.])
    capture.record(17,'camera_depth_optical_frame',before,after,effect)
    # A partial producer write is retried without losing or double-consuming it.
    line=json.dumps(event(17))+'\n';selection.write_text(line[:20]);capture.poll_selections()
    assert not capture.rows
    selection.write_text(line);capture.poll_selections()
    summary=capture.close();case_archive.save('summary',summary)
    assert summary['complete'] and len(summary['selected'])==1
    row=summary['selected'][0]
    arrays=np.load(root/'fixture'/row['file'])
    assert np.array_equal(arrays['before_mm'],before) and np.array_equal(arrays['after_mm'],after)
    assert row['input_sha256']==hashlib.sha256(before.tobytes()).hexdigest()
    assert row['output_sha256']==hashlib.sha256(after.tobytes()).hexdigest()


@pytest.mark.parametrize('damage',['ring_miss','trace_limit','partial_event','case_limit'])
def test_capture_loss_is_explicit_and_never_uses_nearest(case_archive,damage):
    root=case_archive.directory;selection=root/'events.jsonl'
    capture=DepthDropoutEvidence(root/'fixture',selection,
        trace_bytes=1 if damage=='trace_limit' else 10000,
        case_bytes=1 if damage=='case_limit' else 2*1024*1024)
    before=np.ones((2,2),dtype='<u2')
    capture.record(18,'camera_depth_optical_frame',before,before,dict(active=False,reason='OUTSIDE_CONE',changed_pixels=0))
    selection.write_text(json.dumps(event(17 if damage=='ring_miss' else 18))+ (' ' if damage=='partial_event' else '\n'))
    summary=capture.close();case_archive.save('summary',summary)
    assert not summary['complete']
    if damage=='ring_miss':
        assert not summary['selected'] and any('EXACT_FRAME_MISSING' in s for s in summary['missing'])


def test_unselected_teardown_tail_uses_shared_finite_payload_limit(case_archive):
    root=case_archive.directory
    capture=DepthDropoutEvidence(root/'fixture',root/'events.jsonl',selected_frames=2)
    for stamp in range(10):
        depth=np.full((2,2),stamp,dtype='<u2')
        capture.record(stamp,'camera_depth_optical_frame',depth,depth,
            dict(active=True,reason='ACTIVE_CONE',changed_pixels=0,published=True))
    summary=capture.close();case_archive.save('summary',summary)
    assert not summary['selected'] and summary['complete']
    assert [row['stamp_ns'] for row in summary['diagnostics']]==[9,8]
    assert len(list((root/'fixture').glob('*.npz')))==2


@pytest.mark.parametrize('damage',['none','evicted','partial','truncated','repeat','oversized'])
def test_existing_detector_anomaly_pins_only_its_exact_stamp(case_archive,damage):
    from luggage_gazebo.detector_anomaly_events import DetectorAnomalyEvents
    from luggage_perception.ambiguity_capture import FrameCapture
    root=case_archive.directory
    capture=DepthDropoutEvidence(root/'fixture',root/'events.jsonl',ring_frames=1)
    depth=np.full((2,2),1000,dtype='<u2');after=depth.copy();after[0,0]=0
    capture.record(17,'camera_depth_optical_frame',depth,after,
        dict(active=True,reason='ACTIVE_CONE',changed_pixels=1,published=True))
    detector=FrameCapture(root/'detector_inputs')
    detector.consume(17,dict(instance_id='observed-box',generation=2),'anomaly',
        dict(detections=[],depth=after),{},now=0.)
    detector.flush()
    path=next((root/'detector_inputs').glob('*/anomaly_*/frame.json'))
    watcher=DetectorAnomalyEvents(root/'detector_inputs',capture)
    if damage=='evicted':capture.record(18,'camera_depth_optical_frame',depth,after,
        dict(active=True,reason='ACTIVE_CONE',changed_pixels=1,published=True))
    if damage in ('partial','truncated'):
        original=path.read_bytes();path.write_bytes(original[:20]);watcher.poll()
        assert not capture.diagnostics
        if damage=='partial':path.write_bytes(original)
    if damage=='oversized':path.write_bytes(b' '*65537)
    watcher.poll()
    if damage=='repeat':watcher.poll()
    watcher.close();summary=capture.close();case_archive.save('summary',summary)
    anomalies=[row for row in summary['diagnostics'] if row['kind']=='detector_anomaly']
    if damage in ('evicted','truncated','oversized'):
        assert not summary['complete'] and not anomalies
    else:
        assert summary['complete'] and len(anomalies)==1 and anomalies[0]['stamp_ns']==17
        arrays=np.load(root/'fixture'/anomalies[0]['file'])
        assert np.array_equal(arrays['before_mm'],depth) and np.array_equal(arrays['after_mm'],after)


def test_selected_and_anomaly_references_share_one_exact_payload(case_archive):
    root=case_archive.directory;selection=root/'events.jsonl'
    capture=DepthDropoutEvidence(root/'fixture',selection,selected_frames=1)
    depth=np.ones((2,2),dtype='<u2')
    capture.record(17,'camera_depth_optical_frame',depth,depth,
        dict(active=False,reason='OUTSIDE_CONE',changed_pixels=0))
    capture.pin_diagnostic(dict(kind='detector_anomaly',stamp_ns=17))
    selection.write_text(json.dumps(event(17))+'\n');summary=capture.close()
    assert summary['complete'] and len(summary['selected'])==1
    assert summary['diagnostics'][0]['file']==summary['selected'][0]['file']
    assert len(list((root/'fixture').glob('*.npz')))==1


@pytest.mark.parametrize('limit',['epoch','payload'])
def test_detector_capture_resource_limits_are_observable(case_archive,limit):
    from luggage_gazebo.detector_anomaly_events import DetectorAnomalyEvents
    from luggage_perception.ambiguity_capture import FrameCapture
    root=case_archive.directory
    capture=DepthDropoutEvidence(root/'fixture',root/'events.jsonl',
        selected_frames=1 if limit=='payload' else 16)
    detector=FrameCapture(root/'detector_inputs')
    depth=np.ones((2,2),dtype='<u2')
    for stamp in range(1,6 if limit=='epoch' else 3):
        capture.record(stamp,'camera_depth_optical_frame',depth,depth,
            dict(active=True,reason='ACTIVE_CONE',changed_pixels=0))
        detector.consume(stamp,dict(instance_id='observed-'+str(stamp),generation=stamp),
            'anomaly',dict(depth=depth),{},now=0.)
    detector.flush();watcher=DetectorAnomalyEvents(root/'detector_inputs',capture)
    watcher.close();summary=capture.close();case_archive.save('summary',summary)
    assert not summary['complete']
    reason='DETECTOR_CAPTURE_EPOCH_LIMIT' if limit=='epoch' else 'FRAME_PAYLOAD_LIMIT'
    assert reason in summary['missing']
    assert len(list((root/'fixture').glob('*.npz'))) <= (16 if limit=='epoch' else 1)
