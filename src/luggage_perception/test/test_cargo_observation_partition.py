import copy

import numpy as np
import pytest

from cargo_observation_case_capture import CaseCapture
from luggage_perception.cargo_observation_partition import (
    AcquisitionKey, partition_aligned_observation,
)


@pytest.fixture
def case(request,tmp_path):
    return CaseCapture(request.node.nodeid,tmp_path)


def inputs():
    key = AcquisitionKey(1_000_000_001,'camera_color_optical_frame')
    return dict(sensor_id='test_rgbd',observation_id='measured-17',keys=(key,)*5,
        rgb=np.arange(18,dtype=np.uint8).reshape(2,3,3),
        depth_m=np.ones((2,3),dtype=np.float32),
        instance_mask=np.array([[0,1,1],[2,2,0]],dtype=np.uint16),
        intrinsics=np.array([[2.,0,1],[0,2,0],[0,0,1]]),distortion=[],
        container_from_optical=np.eye(4),stride=1,max_pixels=100,
        source_sha256=tuple(c*64 for c in 'abcde'))


def run(case, data):
    case.value('metadata',dict(sensor_id=data['sensor_id'],observation_id=data['observation_id'],
        keys=[vars(k) for k in data['keys']],stride=data['stride'],max_pixels=data['max_pixels'],
        source_sha256=data['source_sha256']))
    case.arrays(**{key:data[key] for key in ('rgb','depth_m','instance_mask','intrinsics',
        'distortion','container_from_optical')})
    try:
        result = partition_aligned_observation(**data)
    except Exception as error:
        case.value('result',dict(accepted=False,reason=str(error)))
        raise
    case.value('result',dict(accepted=True,binding=result.binding_sha256,
        points=result.points().tolist(),pixels=result.pixels().tolist(),labels=result.labels().tolist()))
    return result


def test_hand_deprojected_points_preserve_two_instances_and_background(case):
    result = run(case,inputs())
    np.testing.assert_array_equal(result.points(),[[-.5,0,1],[0,0,1],[.5,0,1],
                                                  [-.5,.5,1],[0,.5,1],[.5,.5,1]])
    assert result.pixels().tolist() == [0,1,2,3,4,5]
    assert result.labels().tolist() == [0,1,1,2,2,0]


def test_missing_depth_preserves_original_pixel_indices(case):
    data = inputs();data['depth_m'][0,1]=0;data['depth_m'][1,1]=np.nan
    result = run(case,data)
    assert result.pixels().tolist() == [0,2,3,5]
    assert result.labels().tolist() == [0,1,2,0]
    np.testing.assert_array_equal(result.points(),[[-.5,0,1],[.5,0,1],[-.5,.5,1],[.5,.5,1]])


def test_explicit_stride_does_not_reindex_original_pixels(case):
    data = inputs();data['stride']=2
    result = run(case,data)
    assert result.pixels().tolist() == [0,2]
    assert result.labels().tolist() == [0,1]


def test_issued_bytes_do_not_alias_mutable_caller_arrays(case):
    data = inputs();result = run(case,data)
    original = (result.xyz_optical_f32,result.instance_labels_u32,
                result.container_from_optical_f64,result.binding_sha256)
    data['depth_m'][:]=99;data['instance_mask'][:]=9;data['container_from_optical'][:]=0
    assert original == (result.xyz_optical_f32,result.instance_labels_u32,
                        result.container_from_optical_f64,result.binding_sha256)
    with pytest.raises(ValueError):
        result.points()[0,0]=10


@pytest.mark.parametrize('changed',['rgb','depth','mask','intrinsics','tf','stride','raw_source'])
def test_binding_changes_with_every_relevant_source(case,changed):
    before = inputs();after = copy.deepcopy(before)
    if changed == 'rgb':after['rgb'][0,0,0]+=1
    if changed == 'depth':after['depth_m'][0,0]+=1
    if changed == 'mask':after['instance_mask'][0,0]=17
    if changed == 'intrinsics':after['intrinsics'][0,0]=3
    if changed == 'tf':after['container_from_optical'][0,3]=.25
    if changed == 'stride':after['stride']=2
    if changed == 'raw_source':after['source_sha256']=('f'*64,)+before['source_sha256'][1:]
    assert run(case,before).binding_sha256 != run(case,after).binding_sha256


@pytest.mark.parametrize('problem',[
    'stamp','frame','rgb_shape','mask_shape','negative_label','large_label','bool_mask',
    'depth_units','empty_depth','stride','resource_limit','skew','focal','distortion',
    'invalid_rotation','invalid_tf_row','nonfinite_tf','missing_hash'])
def test_unqualified_observation_refuses(case,problem):
    data = inputs()
    if problem == 'stamp':data['keys']=(AcquisitionKey(2,'camera_color_optical_frame'),)+data['keys'][1:]
    if problem == 'frame':data['keys']=(AcquisitionKey(1_000_000_001,'other_frame'),)+data['keys'][1:]
    if problem == 'rgb_shape':data['rgb']=data['rgb'][:1]
    if problem == 'mask_shape':data['instance_mask']=data['instance_mask'][:1]
    if problem == 'negative_label':data['instance_mask']=np.full((2,3),-1,dtype=np.int32)
    if problem == 'large_label':data['instance_mask']=np.full((2,3),2**32,dtype=np.uint64)
    if problem == 'bool_mask':data['instance_mask']=np.ones((2,3),dtype=bool)
    if problem == 'depth_units':data['depth_m']=np.ones((2,3),dtype=np.uint16)
    if problem == 'empty_depth':data['depth_m'][:]=0
    if problem == 'stride':data['stride']=0
    if problem == 'resource_limit':data['max_pixels']=5
    if problem == 'skew':data['intrinsics'][0,1]=.1
    if problem == 'focal':data['intrinsics'][0,0]=np.nan
    if problem == 'distortion':data['distortion']=[.1]
    if problem == 'invalid_rotation':data['container_from_optical'][0,0]=2
    if problem == 'invalid_tf_row':data['container_from_optical'][3,0]=1
    if problem == 'nonfinite_tf':data['container_from_optical'][0,3]=np.nan
    if problem == 'missing_hash':data['source_sha256']=('',)*5
    with pytest.raises(ValueError):run(case,data)


@pytest.mark.parametrize('stamp,frame',[(0,'optical'),(-1,'optical'),(2**63,'optical'),(1,'')])
def test_acquisition_key_requires_real_stamp_and_frame(case,stamp,frame):
    case.value('invalid_key',dict(stamp=stamp,frame=frame))
    with pytest.raises(ValueError):AcquisitionKey(stamp,frame)
