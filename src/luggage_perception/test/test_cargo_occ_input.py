import math
import numpy as np
import pytest
from luggage_description.container_geometry import ContainerGeometry, Chamfer, contains_point
from luggage_perception.cargo_occ_input import prepare_occ_observation, OccInputPolicy
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.cargo_view_integration import apply_integrate
from luggage_perception.luggage_box_estimator import voxel_downsample


def make_mapper(res=.05,yaw=0.):
    return CargoVolumeMapper((1.,1.,1.),(0.,0.,.5),yaw,res,max_raycast_points=20,
        geometry_descriptor=ContainerGeometry(1,'container_link',1.,1.,0.,1.,Chamfer('positive_y',.2,.5,.3)).descriptor())


@pytest.mark.parametrize('res',[.01,.02,.05])
@pytest.mark.parametrize('yaw',[0.,math.radians(37),math.pi/2])
def test_crop_scalar_reference_yaw_and_readonly(res,yaw):
    m=make_mapper(res,yaw);rng=np.random.default_rng(20261023)
    points=rng.uniform([-.8,-.8,-.2],[.8,.8,1.2],(10000,3));before=points.copy()
    points.setflags(write=False)
    result=prepare_occ_observation(points,None,m._hull,res,OccInputPolicy(.05,tuple(m.center),yaw))
    inside=[]
    for p in points:
        x,y,z=m._world_to_local(*p)
        if p[2]>=.05 and contains_point(m._hull,(x,y,z+.5)):inside.append(p)
    np.testing.assert_array_equal(result.hit_points,voxel_downsample(inside,res))
    np.testing.assert_array_equal(points,before)
    assert not result.hit_points.flags.writeable and not result.ray_points.flags.writeable
    assert result.counts['inside_hull']==len(inside)


def test_chamfer_outside_point_cannot_steal_inside_downsample_representative():
    m=make_mapper(.05)
    # Same downsample cell, first point outside y=.2+z chamfer.
    points=np.array([[.01,.249,.01],[.01,.201,.01]])
    result=prepare_occ_observation(points,None,m._hull,.05,OccInputPolicy(None,tuple(m.center)))
    np.testing.assert_array_equal(result.hit_points,points[1:])
    np.testing.assert_array_equal(result.ray_points,points[:1])
    assert result.counts['inside_hull']==1


def test_exterior_endpoints_can_still_clear_interior():
    m=make_mapper()
    result=apply_integrate(m,np.array([[1.,0.,.5]]),(-1.,0.,.5))
    assert result['ok'] and len(result['observed_points'])==0
    assert result['preprocess_counts']['hit_downsampled']==0
    assert result['preprocess_counts']['ray_intersecting']==1
    assert 1 in m._grid and 2 not in m._grid


def test_miss_ray_never_steps_and_bad_origin_does_not_mutate():
    m=make_mapper();identity=m.map_revision
    result=apply_integrate(m,[[2.,2.,.5]],(1.,2.,.5))
    assert not result['ok'] and result['preprocess_counts']['samples_visited']==0
    with pytest.raises(ValueError):apply_integrate(m,[[0,0,.5]],(float('nan'),0,0))
    assert identity==m.map_revision


def legacy_ray(m,origin,hit):
    distance=math.dist(origin,hit)
    if distance<m.resolution:return set()
    steps=int(distance/(m.resolution*.5));result=set()
    for step in range(steps):
        t=float(step)/float(steps)
        point=tuple(a+(b-a)*t for a,b in zip(origin,hit))
        voxel=m._local_to_voxel(*m._world_to_local(*point))
        if voxel is not None:result.add(m._index(*voxel))
    return result


@pytest.mark.parametrize('res',[.01,.02,.05])
def test_ray_same_original_sample_phase(res):
    m=make_mapper(res);rng=np.random.default_rng(20261023)
    pairs=rng.uniform([-.8,-.8,-.2],[.8,.8,1.2],(1000,2,3))
    for origin,hit in pairs:
        assert m._raycast_indices(origin,hit)==legacy_ray(m,origin,hit)
    for origin,hit in [((-.8,.5,.5),(.8,.5,.5)),((0,0,.5),(0,0,.5)),
                       ((-.8,.6,.5),(.8,.6,.5)),((-.8,0,0),(.8,0,0))]:
        assert m._raycast_indices(origin,hit)==legacy_ray(m,origin,hit)


def test_summary_matches_full_stats_without_frontier_scan(monkeypatch):
    m=make_mapper();apply_integrate(m,[[0,0,.5]],None)
    expected=m.stats();expected.pop('frontier_count')
    def forbidden():raise AssertionError('response must not enumerate frontiers')
    monkeypatch.setattr(m,'_frontier_indices',forbidden)
    assert m.occupancy_summary()==expected
