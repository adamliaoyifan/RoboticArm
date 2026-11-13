"""Measured camera proposals need explicit limits and remain motion hypotheses."""
from dataclasses import asdict, replace
import json
import math
import os
from pathlib import Path

import numpy as np
import pytest

from luggage_planning.joint_operation.pickup_observation import assess_pickup_observation
from luggage_planning.joint_operation.view_proposals import PickupViewSampling,propose_pickup_views
from test_joint_operation_observation import measured_case


def sampling(**changes):
    values=dict(tilts_rad=(0.,.25,.5),azimuths_rad=(0.,math.pi/2,math.pi,3*math.pi/2),
                distance_scales=(.9,1.,1.2),min_optical_z_m=.1,max_optical_z_m=3.,max_view_proposals=7)
    values.update(changes)
    return PickupViewSampling(**values)


@pytest.mark.parametrize('limit',[1,3,12])
def test_proposals_use_measured_camera_model_are_distinct_and_respect_explicit_budget(request,limit):
    domain,model,probe,view=measured_case()
    need=assess_pickup_observation(domain,model,probe,view).need
    proposals=propose_pickup_views(domain,view,need,sampling(max_view_proposals=limit))
    assert len(proposals)==limit
    assert len({p.world_from_optical for p in proposals})==limit
    assert all(p.reason=='UNVALIDATED_CAMERA_HYPOTHESIS' and p.visible_gap_fraction>0 for p in proposals)
    assert all(p.domain_sha256==domain.digest() and p.view_sha256==view.digest() for p in proposals)
    assert all(p.camera_displacement_m>0 and p.projected_area_pixels>0 for p in proposals)
    assert not any(hasattr(p,'trajectory') or hasattr(p,'collision_free') for p in proposals)
    root=os.environ.get('JOINT_OBSERVATION_EVIDENCE')
    if root:
        directory=Path(root)/request.node.name;directory.mkdir(exist_ok=False)
        from rclpy.serialization import serialize_message
        from luggage_perception.ros_suction_domain import domain_to_message
        from luggage_perception.pickup_view_service import view_to_message
        (directory/'domain.cdr').write_bytes(serialize_message(domain_to_message(domain)))
        (directory/'view.cdr').write_bytes(serialize_message(view_to_message(view)))
        (directory/'need_and_sampling.json').write_text(json.dumps(dict(need=asdict(need),sampling=asdict(sampling(max_view_proposals=limit))),indent=2))
        (directory/'proposals.json').write_text(json.dumps([asdict(p) for p in proposals],indent=2))


@pytest.mark.parametrize('condition',['range','fov','unchanged','source'])
def test_no_useful_or_unbound_view_is_not_returned_as_a_camera_motion(condition):
    domain,model,probe,view=measured_case()
    need=assess_pickup_observation(domain,model,probe,view).need
    config=sampling()
    if condition=='range':config=sampling(min_optical_z_m=8.,max_optical_z_m=9.)
    elif condition=='fov':
        # Actual principal point far outside this acquired image: no gap visible.
        view=replace(view,acquisition=replace(view.acquisition,intrinsics_fx_fy_cx_cy=(200.,200.,100000.,100000.)))
        need=replace(need,view_sha256=view.digest())
    elif condition=='unchanged':config=sampling(tilts_rad=(0.,),azimuths_rad=(0.,),distance_scales=(1.,))
    else:
        need=replace(need,domain_sha256='0'*64)
        with pytest.raises(ValueError):propose_pickup_views(domain,view,need,config)
        return
    assert propose_pickup_views(domain,view,need,config)==()


@pytest.mark.parametrize('changes',[dict(tilts_rad=()),dict(tilts_rad=(math.pi/2,)),dict(distance_scales=(0.,)),
    dict(min_optical_z_m=0.),dict(max_optical_z_m=float('nan')),dict(max_view_proposals=0),
    dict(tilts_rad=(.2,)*4097)])
def test_no_implicit_or_unbounded_sampling(changes):
    with pytest.raises(ValueError):sampling(**changes)
