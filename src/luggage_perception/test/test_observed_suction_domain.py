"""Whole-cell observed support binding; no interpolation or GT input."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from luggage_perception.observed_suction_domain import ObservedSuctionDomain
from luggage_perception.suction_top_domain import TopDomain


def record():
    cells=np.ones((40,30),np.uint8)
    cells[10,10]=0
    return ObservedSuctionDomain('box','instance',3,1794371300123456789,'world',1,'model',
        (0.,0.,.85),(0.,0.,0.,1.),(.8,.6,.3),(1.,0.,0.),(0.,1.,0.),(0.,0.,1.),
        -1.,(-.4,-.3),.02,(40,30),cells.tobytes())


def test_support_is_immutable_and_unknown_hole_is_not_filled():
    value=record()
    assert not value.cells().flags.writeable
    assert not value.cells()[10,10]
    assert value.cells().sum()==1199
    assert value.world_point((.1,.2))==(.1,.2,1.)
    with pytest.raises(ValueError): value.cells()[10,10]=True
    assert replace(value,support=bytes([1])*1200).digest()!=value.digest()


@pytest.mark.parametrize('field,value', [('frame_id','container_link'),('box_id',''),
    ('stamp_ns',-1),('stamp_ns',True),('generation',-1),('box_wdh',(1.,0.,1.)),
    ('contact_model_version',0),('stamp_ns',2**63),
    ('box_xyz_world',(0.,0.,float('nan'))),('box_quaternion_xyzw',(0.,0.,0.,0.)),
    ('normal_world',(0.,0.,-1.)),('v_axis_world',(1.,0.,0.)),('cell_size_m',0.),
    ('plane_offset_world_m',float('nan')),('shape',(40,29)),('shape',(1000001,1)),
    ('support',bytes([2])*1200),('origin_uv_top_m',(0.,float('inf')))])
def test_invalid_or_unbounded_domain_refuses(field,value):
    with pytest.raises(ValueError): replace(record(),**{field:value})


def detection_and_evaluation():
    from luggage_msgs.msg import DetectedLuggage
    value=record()
    detection=DetectedLuggage()
    detection.id=value.box_id
    detection.header.frame_id='world'
    detection.header.stamp.sec,detection.header.stamp.nanosec=divmod(value.stamp_ns,10**9)
    detection.pose.position.x,detection.pose.position.y,detection.pose.position.z=value.box_xyz_world
    detection.pose.orientation.w=1.
    detection.width,detection.depth,detection.height=value.box_wdh
    detection.top_surface_valid,detection.height_valid=True,True
    detection.height_source=detection.HEIGHT_SOURCE_MEASURED_SUPPORT
    stamp=float(detection.header.stamp.sec)+float(detection.header.stamp.nanosec)*1e-9
    domain=TopDomain(value.cells(),value.origin_uv_top_m,value.cell_size_m,stamp,'world',
        value.u_axis_world,value.v_axis_world,value.normal_world,value.plane_offset_world_m)
    evaluation=SimpleNamespace(ok=True,stamp=stamp,frame='world',instance_id='instance',generation=3,
        model_version=1,model_hash='model',observed_top_domain=domain)
    return detection,evaluation


def test_generated_ros_cdr_keeps_exact_nanoseconds_geometry_and_unknown_support():
    from rclpy.serialization import serialize_message,deserialize_message
    from luggage_msgs.msg import SuctionTopDomainV2
    from luggage_perception.ros_suction_domain import domain_to_message,domain_from_message,observed_domain_for_detection
    detection,evaluation=detection_and_evaluation()
    value=observed_domain_for_detection(detection,evaluation)
    assert value==record()
    message=domain_to_message(value)
    decoded=deserialize_message(serialize_message(message),SuctionTopDomainV2)
    assert domain_from_message(decoded)==value
    assert value.stamp_ns==1794371300123456789
    decoded.support[10*30+10]=1
    with pytest.raises(ValueError,match='digest mismatch'): domain_from_message(decoded)


@pytest.mark.parametrize('damage',['missing_domain','wrong_stamp','wrong_frame','configured_height','missing_height'])
def test_incomplete_or_unmeasured_detection_cannot_export_support(damage):
    from luggage_perception.ros_suction_domain import observed_domain_for_detection
    detection,evaluation=detection_and_evaluation()
    if damage=='missing_domain': evaluation.observed_top_domain=None
    elif damage=='wrong_stamp': evaluation.stamp-=1.
    elif damage=='wrong_frame': detection.header.frame_id='camera'
    elif damage=='configured_height': detection.height_source=detection.HEIGHT_SOURCE_CONFIGURED_SUPPORT
    else: detection.height_valid=False
    with pytest.raises(ValueError): observed_domain_for_detection(detection,evaluation)


def test_consumer_compares_independent_detection_generation_model_and_geometry():
    from luggage_perception.ros_suction_domain import require_matching_detection
    detection,_=detection_and_evaluation()
    value=record()
    require_matching_detection(value,detection,generation=3,contact_model_hash='model')
    for kwargs in ({'generation':4,'contact_model_hash':'model'},
                   {'generation':3,'contact_model_hash':'stale'}):
        with pytest.raises(ValueError): require_matching_detection(value,detection,**kwargs)
    detection.pose.position.x=.01
    with pytest.raises(ValueError): require_matching_detection(value,detection,generation=3,contact_model_hash='model')


def test_detector_publishes_exact_frame_domain_then_invalidates_mismatched_frame():
    import importlib.util
    from pathlib import Path
    from luggage_msgs.msg import DetectionFrame
    from luggage_perception.ros_suction_domain import domain_from_message
    path=Path(__file__).parents[1]/'scripts/luggage_detector_node.py'
    spec=importlib.util.spec_from_file_location('detector_observed_domain',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    detection,evaluation=detection_and_evaluation()
    outputs=[]
    fake=SimpleNamespace(_suction_top_domain_pub=SimpleNamespace(publish=outputs.append),
                         _last_suction_eval=evaluation)
    frame=DetectionFrame()
    frame.header=detection.header
    frame.box=detection
    frame.pca_valid=True
    frame.instance_id='instance'
    frame.generation=3
    module.LuggageDetector._publish_suction_top_domain(fake,frame)
    assert domain_from_message(outputs[-1])==record()
    frame.generation=4
    module.LuggageDetector._publish_suction_top_domain(fake,frame)
    assert not outputs[-1].valid and not outputs[-1].support
    assert outputs[-1].generation==4
    assert outputs[-1].reason.startswith('OBSERVED_TOP_DOMAIN_UNAVAILABLE:')
    # Disabling export does not change the legacy detection/candidate stream.
    fake._suction_top_domain_pub=None
    module.LuggageDetector._publish_suction_top_domain(fake,frame)
    assert len(outputs)==2


def test_real_depth_evaluator_export_preserves_quality_and_measured_holes():
    from luggage_perception.dynamic_top_surface import estimate_dynamic_top_surface
    from luggage_perception.instance_depth_component import isolate_depth_component
    from luggage_perception.suction_patch_evaluator import SuctionPatchEvaluator
    from luggage_perception.eval.dynamic_suction_renderer import (
        NadirRenderer,DEFAULT_CAMERA,box_polygon,pixel_bbox,
    )
    from test_suction_patch_evaluator import make_model
    renderer=NadirRenderer()
    polygon=box_polygon((.55,.40),(-1.,0.),0.)
    z=DEFAULT_CAMERA['platform_z']+.25
    depth=renderer.render([(z,polygon)],noise_sigma_mm=0.,missing_frac=0.,outlier_frac=0.,seed=0)
    bbox=pixel_bbox(renderer,polygon,z,expand_frac=.05)
    component=isolate_depth_component(depth,bbox=bbox)
    assert component.ok
    region=np.zeros(depth.shape,bool)
    x0,y0,x1,y1=bbox;region[y0:y1,x0:x1]=True
    intrinsics=SimpleNamespace(**{k:DEFAULT_CAMERA[k] for k in ('fx','fy','cx','cy')})
    transform=np.asarray(DEFAULT_CAMERA['optical_to_world'])
    top=estimate_dynamic_top_surface(depth,component.mask,intrinsics,transform,stamp=12.,
        frame='world',instance_region=region)
    assert top.reason=='ok'
    evaluator=SuctionPatchEvaluator(make_model(.18))
    kwargs=dict(instance_region=region,stamp=12.,frame_id='world',instance_id='instance',generation=3)
    evaluator.update(depth,intrinsics,transform,top,**kwargs)
    baseline=evaluator.copy_output()
    evaluator.update(depth,intrinsics,transform,top,capture_observed_domain=True,**kwargs)
    output=evaluator.copy_output()
    assert output.accepted==baseline.accepted and output.accepted
    assert output.observed_top_domain.valid_cells.any()
    assert baseline.observed_top_domain is None
    # A newly missing depth pixel remains missing even though the fitted plane
    # and box dimensions are unchanged; it cannot be converted into support.
    altered=depth.copy()
    ys,xs=np.nonzero(top.winning_patch_mask)
    index=np.argmin((xs-xs.mean())**2+(ys-ys.mean())**2)
    altered[ys[index],xs[index]]=0.
    evaluator.update(altered,intrinsics,transform,top,capture_observed_domain=True,**kwargs)
    changed=evaluator.copy_output().observed_top_domain
    assert changed.valid_cells.sum()<output.observed_top_domain.valid_cells.sum()
