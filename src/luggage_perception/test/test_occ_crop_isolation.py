"""Synthetic two-branch replay; these fixtures are not recorded camera frames."""
from collections import namedtuple
from dataclasses import fields
import hashlib
import numpy as np
import pytest
from builtin_interfaces.msg import Time
from luggage_perception.ros_message_adapters import cloud_msg_from_points, cloud_points_from_msg
from luggage_perception.cargo_view_integration import apply_integrate
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.dynamic_top_surface import estimate_dynamic_top_surface
from luggage_perception.occupancy_map_store import OccupancyMapStore, MapUpdate, UpdateKind


@pytest.mark.parametrize('scene_x',[-1.2,.5])
@pytest.mark.parametrize('frame',range(10))
def test_pick_top_and_shared_cloud_unchanged(scene_x,frame):
    intr=namedtuple('Intrinsics','fx fy cx cy')(100.,100.,50.,40.)
    depth=np.full((80,100),1000+frame,dtype=np.uint16)
    mask=np.zeros(depth.shape,dtype=bool);mask[12:68,16:84]=True
    transform=np.diag([1.,-1.,-1.,1.]);transform[:3,3]=[scene_x,0,1.5]
    yy,xx=np.nonzero(mask);distance=depth[mask]*.001
    points=np.column_stack((scene_x+(xx-50)*distance/100.,-(yy-40)*distance/100.,1.5-distance))
    stamp=Time(sec=frame+1,nanosec=17)
    message=cloud_msg_from_points(points,stamp,'container_link')
    payload=bytes(message.data);identity=(message.header.frame_id,message.header.stamp.sec,message.header.stamp.nanosec)
    baseline=estimate_dynamic_top_surface(depth,mask,intr,transform,stamp=frame+1,frame='container_link')
    assert baseline.reason=='ok'
    decoded=cloud_points_from_msg(message);copy=decoded.copy();decoded.setflags(write=False)
    mapper=CargoVolumeMapper((1.,1.,1.),(0,0,.5),0,.05,max_raycast_points=20)
    result=apply_integrate(mapper,decoded,None)
    if scene_x<0:assert result['preprocess_counts']['inside_hull']==0
    else:assert 0<result['preprocess_counts']['inside_hull']<len(decoded)
    after=estimate_dynamic_top_surface(depth,mask,intr,transform,stamp=frame+1,frame='container_link')
    def equal(a,b):
        if isinstance(a,np.ndarray):np.testing.assert_array_equal(a,b)
        elif isinstance(a,(tuple,list)):
            assert len(a)==len(b)
            for x,y in zip(a,b):equal(x,y)
        else:assert a==b
    for f in fields(baseline):equal(getattr(baseline,f.name),getattr(after,f.name))
    assert bytes(message.data)==payload
    assert (message.header.frame_id,message.header.stamp.sec,message.header.stamp.nanosec)==identity
    np.testing.assert_array_equal(decoded,copy)
    assert hashlib.sha256(payload).digest()==hashlib.sha256(bytes(message.data)).digest()


def test_policy_participates_in_operation_identity_and_bad_geometry_is_readonly():
    mapper=CargoVolumeMapper((1.,1.,1.),(0,0,.5),0,.05)
    owner=OccupancyMapStore(mapper,epoch='test')
    kwargs=dict(expected_snapshot=owner.identity,geometry_hash=mapper.geometry_hash,
                points=np.array([[0.,0.,.5]]),stamp_key=(1,0),acquisition_stamp_ns=1000000000,
                schema_version=1,settled=True)
    result=owner.apply_update(MapUpdate('a',UpdateKind.INTEGRATE_VIEW,minimum_z=.05,**kwargs))
    assert result.accepted
    identity=owner.identity
    changed=owner.apply_update(MapUpdate('a',UpdateKind.INTEGRATE_VIEW,minimum_z=.6,**kwargs))
    assert changed.reason=='OPERATION_ID_CONFLICT' and owner.identity==identity
    kwargs.update(geometry_hash='wrong',expected_snapshot=identity,stamp_key=(2,0))
    bad=owner.apply_update(MapUpdate('b',UpdateKind.INTEGRATE_VIEW,**kwargs))
    assert bad.reason=='GEOMETRY_HASH_MISMATCH' and owner.identity==identity
