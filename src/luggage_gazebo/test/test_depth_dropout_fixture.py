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
        assert not summary['selected'] and any('EXACT_SELECTED_FRAME_MISSING' in s for s in summary['missing'])
