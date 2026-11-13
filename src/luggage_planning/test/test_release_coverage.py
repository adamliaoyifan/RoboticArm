"""Measured coverage remains strict; FOV ranking never inserts observations."""
from copy import deepcopy
from types import SimpleNamespace
import numpy as np
import pytest
from luggage_planning.joint_operation.release_coverage import release_coverage,projected_missing_columns
from luggage_planning.placement_prior.contracts import Transform
from luggage_planning.geometry.interior_view_scorer import CameraIntrinsics


def surface():
    return dict(resolution_m=.02,origin_xy_container=(-.02,-.02),
        state=[['occupied']*2 for _ in range(2)],surface_z_container=[[.65]*2 for _ in range(2)])


@pytest.mark.parametrize('damage',(None,'unknown','free','wrong_height','empty'))
def test_all_observed_top_columns_required_and_missing_points_are_measured(damage):
    s=surface();target=Transform((0.,0.,.60),(0.,0.,0.,1.));dimensions=(.04,.04,.10)
    if damage in ('unknown','free'):s['state'][1][1]=damage;s['surface_z_container'][1][1]=None
    if damage=='wrong_height':s['surface_z_container'][1][1]=.70
    if damage=='empty':target=Transform((1.,0.,.60),(0.,0.,0.,1.))
    before=deepcopy(s);result=release_coverage(s,target,dimensions)
    assert result['accepted']==(damage is None)
    assert result['expected_columns']==(0 if damage=='empty' else 4)
    assert len(result['missing_points_container'])==(0 if damage in (None,'empty') else 1)
    assert s==before


def test_projected_cell_count_uses_calibrated_forward_camera_axes():
    identity=Transform((0.,0.,0.),(0.,0.,0.,1.));k=CameraIntrinsics(100,100,100,100,50,50)
    points=[(0.,0.,1.),(.49,0.,1.),(.51,0.,1.),(0.,0.,-1.)]
    assert projected_missing_columns(points,identity,identity,k)==2
    down=Transform((0.,0.,2.),(1.,0.,0.,0.))
    assert projected_missing_columns(points[:1],identity,down,k)==1


def test_view_orientation_aims_missing_footprint_centre_in_camera_fov():
    from luggage_planning.joint_operation.release_coverage import camera_aimed_at_missing
    container=Transform((1.,2.,0.),(0.,0.,0.,1.))
    points=[(-.1,-.1,.6),(.1,.1,.6)]
    camera=camera_aimed_at_missing((0.,0.,1.2),points,container)
    assert projected_missing_columns(points,container,camera,CameraIntrinsics(100,100,50,50))==2


def test_explicit_observation_fraction_never_fills_missing_or_proves_stack():
    target=Transform((0.,0.,.60),(0.,0.,0.,1.));dimensions=(.04,.04,.10)
    s=surface();s['state'][1][1]='unknown';s['surface_z_container'][1][1]=None
    before=deepcopy(s)
    assert release_coverage(s,target,dimensions,min_observed_ratio=.6)['accepted']
    assert not release_coverage(s,target,dimensions)['accepted']
    s['state'][1][0]='free';s['surface_z_container'][1][0]=None
    assert not release_coverage(s,target,dimensions,min_observed_ratio=.6)['accepted']
    assert before['state'][1][1]=='unknown'
    assert release_coverage(before,target,dimensions,min_observed_ratio=.6)['missing_points_container']


@pytest.mark.parametrize('ratio',(0.,-1.,1.01,float('nan')))
def test_invalid_observation_fraction_is_not_accepted(ratio):
    with pytest.raises(ValueError,match='RELEASE_OBSERVATION_RATIO_REQUIRED'):
        release_coverage(surface(),Transform((0.,0.,.6),(0.,0.,0.,1.)),(.04,.04,.1),min_observed_ratio=ratio)
