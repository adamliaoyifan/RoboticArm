import math
import pytest
from luggage_description.container_geometry import cuboid_from_inner_size
from luggage_perception.depth_view_rays import admit_depth_ray
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper, FREE, OCCUPIED, UNKNOWN


HULL=cuboid_from_inner_size((1.,1.,1.),floor_z=0.)


def state(mapper,point):
    voxel=mapper._local_to_voxel(*mapper._world_to_local(*point))
    return mapper.state_at(mapper._index(*voxel))


def test_wall_return_only_clears_measured_ray_and_leaves_terminal_band_unknown():
    mapper=CargoVolumeMapper((1.,1.,1.),(0.,0.,.5),0.,.1)
    for _ in range(5):
        result=mapper.integrate_depth_view([(.5,0.,.5)],(-1.,0.,.5))
    assert result['hits']==0 and result['free_voxels']>0
    assert state(mapper,(0.,0.,.5))==FREE
    assert state(mapper,(.45,0.,.5))==UNKNOWN
    assert state(mapper,(0.,.35,.5))==UNKNOWN
    assert all(row=='unknown' for column in mapper.surface_map_2d()['state'] for row in column)


def test_real_obstacle_is_occupied_and_space_behind_it_stays_unknown():
    mapper=CargoVolumeMapper((1.,1.,1.),(0.,0.,.5),0.,.1)
    for _ in range(5):
        result=mapper.integrate_depth_view([(0.,0.,.5)],(-1.,0.,.5))
    assert result['hits']==1
    assert state(mapper,(0.,0.,.5))==OCCUPIED
    assert state(mapper,(-.3,0.,.5))==FREE
    assert state(mapper,(.3,0.,.5))==UNKNOWN


@pytest.mark.parametrize('origin,end',[
    ((1.,0.,.5),(0.,0.,.5)),  # enters through rear wall
    ((0.,1.,.5),(0.,0.,.5)),  # enters through side wall
    ((-1.,0.,.5),(-.7,0.,.5)), # return before doorway
    ((-1.,0.,.5),(1.,0.,.5)), # implausibly beyond closed rear wall
    ((-1.,0.,.5),(float('nan'),0.,.5)),
])
def test_rejected_rays_leave_map_unchanged(origin,end):
    mapper=CargoVolumeMapper((1.,1.,1.),(0.,0.,.5),0.,.1)
    before=mapper.stats()['map_revision']
    result=mapper.integrate_depth_view([end],origin)
    assert result['rays']==0 and result['hits']==0
    assert mapper.stats()['map_revision']==before


def test_translated_rotated_mapper_gives_same_sensor_states():
    a=CargoVolumeMapper((1.,1.,1.),(0.,0.,.5),0.,.1)
    b=CargoVolumeMapper((1.,1.,1.),(2.,3.,1.5),math.pi/2,.1)
    for _ in range(5):
        a.integrate_depth_view([(.035,.035,.565)],(-1.03,.035,.565))
        b.integrate_depth_view([(1.965,3.035,1.565)],(1.965,1.97,1.565))
    assert a._grid==b._grid


def test_bad_band_rejected():
    with pytest.raises(ValueError):admit_depth_ray(HULL,(-1.,0.,.5),(.5,0.,.5),0.)


def test_covered_column_becomes_free_only_after_all_heights_observed():
    mapper=CargoVolumeMapper((1.,1.,1.),(0.,0.,.5),0.,.1)
    # Synthetic boundary test: ten different camera heights observe one row.
    # No production sensor provenance is claimed for these constructed rays.
    for iz in range(10):
        z=(iz+.5)*.1
        for _ in range(5):mapper.integrate_depth_view([(.5,.035,z)],(-1.03,.035,z))
        column=mapper.surface_map_2d()['state'][5][5]
        assert column==('free' if iz==9 else 'unknown')


def test_obstacle_just_inside_open_door_is_not_discarded():
    mapper=CargoVolumeMapper((1.,1.,1.),(0.,0.,.5),0.,.1)
    for _ in range(5):
        result=mapper.integrate_depth_view([(-.48,.035,.565)],(-1.03,.035,.565))
    assert result['hits']==1 and result['free_voxels']==0
    assert state(mapper,(-.48,.035,.565))==OCCUPIED
