"""Deployment spacing, immutable owner geometry and precision behaviour."""
import math
import numpy as np
import pytest
from luggage_description.cargo_grid_contract import CargoGridSpec, deployment_resolution, MAX_VOLUME_CELLS
from luggage_description.cargo_grid_parameters import default_resolution, declare_resolution
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper, SOURCE_SENSOR
from luggage_perception.cargo_volume_snapshot import copy_volume
from luggage_perception.occupancy_map_store import OccupancyMapStore, MapUpdate, UpdateKind


@pytest.mark.parametrize('value',[0,-.01,float('nan'),float('inf')])
def test_invalid_grid_rejected_before_allocation(value):
    with pytest.raises(ValueError): CargoGridSpec.create((1,1,1),value)


@pytest.mark.parametrize('size',[(0,1,1),(1,-1,1),(float('nan'),1,1),(1,1),(1,1,1,1)])
def test_invalid_dimensions(size):
    with pytest.raises(ValueError): CargoGridSpec.create(size,.01)


def test_deployment_default_and_bounded_full_size():
    assert default_resolution()==.01
    spec=CargoGridSpec.create((1.49,1.97,1.48),.01)
    assert spec.dimensions_xyz==(149,197,148)
    assert math.prod(spec.dimensions_xyz)==4344244 < MAX_VOLUME_CELLS
    with pytest.raises(ValueError,match='before allocation'):
        CargoGridSpec.create((1.49,1.97,1.48),.005)
    for value in (.004,.051,float('nan')):
        with pytest.raises(ValueError): deployment_resolution(value)


@pytest.mark.parametrize('resolution',[.01,.02,.05])
def test_same_owner_resolution_and_projection_copies(resolution):
    mapper=CargoVolumeMapper((.4,.4,.4),(0.,0.,.2),0.,resolution)
    store=OccupancyMapStore(mapper,epoch='test')
    assert store.snapshot().resolution==copy_volume(mapper)['resolution_m']==resolution
    before=store.identity
    with pytest.raises(AttributeError): mapper.resolution=.03
    assert store.identity==before
    projection=store.v1_surface_dict(); projection['height'][0][0]=99
    projection['state'][0][0]='occupied'
    projection['geometry_descriptor']['length']=99
    assert mapper.geometry_descriptor['length']==.4
    assert store.snapshot().height[0][0]==0
    assert store.snapshot().state[0][0]=='unknown'
    # Wrong expected identity cannot mutate a map.
    other=OccupancyMapStore(CargoVolumeMapper((.4,.4,.4),(0.,0.,.2),0.,resolution),epoch='other')
    result=store.apply_update(MapUpdate('bad',UpdateKind.ADD_BOX_V2,
        expected_snapshot=other.identity,geometry_hash=mapper.geometry_hash,
        box=((0.,0.,.1),(.2,.2,.2),0.),measured_source='verify_placed_box',
        verified_center=(0.,0.,.1),verify_tolerance_m=.005))
    assert result.status.value=='rejected'
    assert store.identity==before and not mapper.commit_ledger()


@pytest.mark.parametrize('resolution',[.01,.02,.05])
def test_node_startup_override_and_runtime_rejection(resolution):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    context=rclpy.Context(); rclpy.init(context=context,domain_id=37)
    node=Node('grid_parameter_test',context=context,
              parameter_overrides=[Parameter('resolution',value=resolution)])
    try:
        assert declare_resolution(node)==resolution
        result=node.set_parameters([Parameter('resolution',value=.03)])[0]
        assert not result.successful
        assert node.get_parameter('resolution').value==resolution
    finally:
        node.destroy_node();context.shutdown()


def test_projection_semantics_sensor_labels_and_inactive_cells():
    m=CargoVolumeMapper((.2,.2,.2),(0.,0.,.1),0.,.01,
                        hull_local_inside=lambda x,y,z:x<0)
    m.mark_occupied_voxel(0,0,2,SOURCE_SENSOR,label=8)
    m.mark_occupied_voxel(0,0,3,SOURCE_SENSOR,label=4)
    m.mark_occupied_voxel(0,0,4,SOURCE_SENSOR,label=4)
    m.mark_occupied_voxel(0,0,5,SOURCE_SENSOR,label=8)
    s=m.surface_map_2d()
    assert s['semantic_label'][0][0]==8  # tied counts keep first bottom-up label
    assert s['height'][0][0]==.06
    assert s['confidence'][0][0]=='sensor'
    assert s['state'][1][1]=='unknown'
    assert s['known_ratio'][0][0]==4/20
    m.mark_occupied_voxel(19,19,10,SOURCE_SENSOR,label=9)
    assert m.surface_map_2d()['height'][19][19]==0


def test_large_volume_export_not_truncated_and_isolated():
    # Exercises the formerly forbidden >1M cell range without a full ROS graph.
    m=CargoVolumeMapper((1.01,1.01,1.01),(0.,0.,.505),0.,.01)
    volume=copy_volume(m)
    assert len(volume['states'])==101**3
    assert all(type(v) is bool for v in volume['active'])
    m.mark_occupied_voxel(100,100,100)
    assert volume['states'][-1]==0 and copy_volume(m)['states'][-1]==2


def test_sparse_sensor_points_stay_unknown_at_finer_resolution():
    points=[(x,y,.199) for x in (-.075,-.025,.025,.075)
            for y in (-.075,-.025,.025,.075)]
    occupied=[]
    for r in (.05,.01):
        m=CargoVolumeMapper((.6,.6,.4),(0.,0.,.2),0.,r)
        for point in points:m.mark_occupied_world(*point)
        s=m.surface_map_2d()
        occupied.append(sum(v=='occupied' for row in s['state'] for v in row))
        assert all(v!='free' for row in s['state'] for v in row)
    assert occupied==[16,16]  # Smaller cells must not fabricate more observations.
