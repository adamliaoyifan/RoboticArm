"""Physical mesh validation must not manufacture observed free space."""
from dataclasses import replace
import math
import pytest
from luggage_description.container_geometry import cuboid_from_inner_size
from luggage_perception.depth_physical_boundary import BoundaryHit, admit_physical_depth_ray
from luggage_perception.depth_view_rays import admit_depth_ray

HULL = cuboid_from_inner_size((1.,1.,1.))
ORIGIN = (-1.,0.,.5)
HIT = BoundaryHit(ORIGIN,(1.,0.,0.),1.52,'scene','mesh')


def admit(end=(.52,0.,.5), hit=HIT, band=.01, **identity):
    return admit_physical_depth_ray(HULL,ORIGIN,end,band,hit,
        **dict({'scene_revision':'scene','mesh_sha256':'mesh'},**identity))


def test_inset_usable_boundary_is_not_a_physical_wall():
    assert admit_depth_ray(HULL,ORIGIN,(.52,0.,.5),.01) is None
    ray = admit()
    assert ray.free_end == pytest.approx((.5,0.,.5))
    assert ray.obstacle is None


def test_band_is_subtracted_from_surface_not_inset_hull():
    ray = admit(band=.05)
    assert ray.free_end == pytest.approx((.47,0.,.5))
    assert ray.obstacle is None


def test_short_measured_obstacle_never_extrapolates_to_wall():
    ray = admit(end=(.1,0.,.5))
    assert ray.free_end == pytest.approx((.09,0.,.5))
    assert ray.obstacle == (.1,0.,.5)


def test_nearer_physical_hit_limits_noisy_return():
    ray = admit(end=(.525,0.,.5),band=.05)
    assert ray.free_end == pytest.approx((.47,0.,.5))


def test_return_through_physical_wall_is_refused():
    assert admit(end=(.55,0.,.5)) is None


def test_mesh_hit_before_usable_entry_is_refused():
    assert admit(end=(-.4,0.,.5),hit=replace(HIT,distance_m=.2)) is None


def test_missing_mesh_hit_cannot_create_ray():
    assert admit(hit=None) is None


@pytest.mark.parametrize('change',[{'scene_revision':'stale'},{'mesh_sha256':'changed'}])
def test_identity_mismatch_fails_closed(change):
    with pytest.raises(ValueError,match='identity mismatch'):
        admit(**change)


@pytest.mark.parametrize('change',[
    {'origin':(-.99,0.,.5)}, {'direction':(0.,1.,0.)},
    {'direction':(2.,0.,0.)}, {'distance_m':math.nan}, {'distance_m':-1.},
])
def test_unrelated_or_invalid_hit_fails_closed(change):
    with pytest.raises(ValueError):
        admit(hit=replace(HIT,**change))


def test_closed_face_entry_stays_forbidden():
    origin=(1.,0.,.5)
    hit=BoundaryHit(origin,(-1.,0.,0.),1.52,'scene','mesh')
    assert admit_physical_depth_ray(HULL,origin,(0.,0.,.5),.01,hit,
        scene_revision='scene',mesh_sha256='mesh') is None
