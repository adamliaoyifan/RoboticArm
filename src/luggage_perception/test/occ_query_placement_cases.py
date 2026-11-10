"""Real Store-generated v2 placement inputs for query-only regression.

The fallback case observes a high plateau around an off-centre floor hole.
No unknown column is assigned a made-up positive height. The plateau's
bounding rectangle spans the map, while floor cells are not plateaus, so
structural edge proposals cannot discover the hole; the grid remains legal.
"""
import itertools
import numpy as np


BOX = (.44,.33,.28)


def create_case(scene,resolution,boxes,fallback):
    from luggage_description.container_geometry import descriptor_from_scene_config
    from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
    from luggage_perception.occupancy_map_store import MapUpdate,OccupancyMapStore,UpdateKind
    hull = descriptor_from_scene_config(scene)
    mapper = CargoVolumeMapper((hull.length,hull.width,hull.ceiling_z-hull.floor_z),
        (0.,0.,(hull.ceiling_z+hull.floor_z)/2),0.,resolution,
        geometry_descriptor=hull.descriptor(),max_raycast_points=2000)
    store = OccupancyMapStore(mapper,epoch='placement-query-oracle',clock_ns=lambda:1_000_000_000)
    geometry = [((x,y,hull.floor_z+.13+.26*z),(.35,.53,.26),0.)
        for x,y in itertools.product((-.50,.50),(-.70,.285)) for z in range(5)]
    for i,box in enumerate(geometry[:boxes]):
        result = store.apply_update(MapUpdate(operation_id='box-'+str(i),kind=UpdateKind.ADD_BOX_V1,
            box=box,box_id='planning-box-'+str(i)))
        assert result.accepted,result.reason
    assert len(mapper._placed_boxes)==boxes
    points = np.empty((0,3))
    if fallback:
        # Exact native top at1.30m in all three resolutions. Sensor
        # observations use the normal Store path, including geometry shadow.
        z = hull.floor_z+(round(1.30/resolution)-.5)*resolution
        points = np.asarray([(x,y,z) for iy in range(mapper.ny) for ix in range(mapper.nx)
            # Avoid the world-origin downsampler's cell boundaries. For
            # this hull, native centres land on those boundaries at1cm;
            # roundoff would merge observations from adjacent columns.
            for x,y in [(-hull.length/2+(ix+.51)*resolution,-hull.width/2+(iy+.51)*resolution)]
            if not (-.05 <= x <= .55 and -.42 <= y <= 0.)])
        result = store.apply_update(MapUpdate(operation_id='plateau-observation',kind=UpdateKind.INTEGRATE_VIEW,
            expected_snapshot=store.identity,geometry_hash=mapper.geometry_hash,points=points,origin=None,
            schema_version=1,stamp_key=(1,0),acquisition_stamp_ns=1_000_000_000,settled=True))
        assert result.accepted,result.reason
    return store,geometry[:boxes],points
