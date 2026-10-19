import hashlib
import numpy as np
import pytest
from luggage_description.container_geometry import cuboid_from_inner_size
from luggage_perception.depth_boundary_mesh import StaticBoundaryMesh
from luggage_perception.local_depth_volume import LocalDepthVolume
from luggage_perception.voxel_log_odds import UNKNOWN,FREE,OCCUPIED


def volume(**kwargs):
    triangles=np.array([[[.52,-1.,-1.],[.52,1.,-1.],[.52,0.,2.]]],dtype='<f8')
    mesh=StaticBoundaryMesh(triangles,scene_revision='scene',expected_sha256=hashlib.sha256(triangles.tobytes()).hexdigest())
    return LocalDepthVolume(cuboid_from_inner_size((1.,1.,1.)),mesh,
                            (-.2,-.1,.4),(.5,.1,.6),.01,**kwargs)


def integrate(v,points,stamp=1,**kwargs):
    return v.integrate(points,(-1.,.005,.505),stamp_ns=stamp,
        scene_revision='scene',mesh_sha256=v.mesh.mesh_sha256,deadline=10.,clock=lambda:0.,**kwargs)


def test_geometry_alone_and_unobserved_cells_stay_unknown():
    v=volume();assert v.state_at((.3,.005,.505))==UNKNOWN
    integrate(v,[(.52,.005,.505)])
    assert v.state_at((.3,.005,.505))==FREE
    assert v.state_at((.3,.075,.505))==UNKNOWN
    assert v.snapshot()['confirmed'] is False


def test_nearby_obstacle_hit_wins_and_behind_remains_unknown():
    v=volume();integrate(v,[(.1,.005,.505),(.52,.005,.505)])
    assert v.state_at((.1,.005,.505))==OCCUPIED
    separate=volume();integrate(separate,[(.1,.005,.505)])
    assert separate.state_at((.3,.005,.505))==UNKNOWN


def test_duplicate_frame_and_identity_failure_do_not_mutate():
    v=volume();integrate(v,[(.52,.005,.505)]);before=v.snapshot()
    with pytest.raises(ValueError,match='stamp'):integrate(v,[(.52,.005,.505)])
    with pytest.raises(ValueError,match='identity'):
        v.integrate([(.52,.005,.505)],(-1.,0.,.5),stamp_ns=2,scene_revision='stale',
                    mesh_sha256=v.mesh.mesh_sha256,deadline=1.)
    assert v.snapshot()==before


def test_mid_frame_deadline_is_transactional():
    v=volume();before=v.snapshot();ticks=iter([0.]*15+[2.]*1000)
    with pytest.raises(TimeoutError):
        v.integrate([(.52,.005,.505)]*2,(-1.,.005,.505),stamp_ns=1,
          scene_revision='scene',mesh_sha256=v.mesh.mesh_sha256,deadline=1.,clock=lambda:next(ticks))
    assert v.snapshot()==before


def test_ray_budget_and_nonfinite_frame_refused():
    v=volume();before=v.snapshot()
    for points in [[(.52,0.,.5)]*2001,[(float('nan'),0.,.5)]]:
        with pytest.raises(ValueError):integrate(v,points)
    assert v.snapshot()==before


def test_roi_resource_and_hull_limits_are_checked_before_integration():
    v=volume()
    with pytest.raises(ValueError,match='budget'):
        LocalDepthVolume(v.hull,v.mesh,(-.2,-.1,.4),(.5,.1,.6),.0001)
    with pytest.raises(ValueError,match='inside'):
        LocalDepthVolume(v.hull,v.mesh,(-.2,-.1,.4),(.6,.1,.6),.01)
    with pytest.raises(ValueError,match='whole voxels'):
        LocalDepthVolume(v.hull,v.mesh,(-.2,-.1,.4),(.495,.1,.6),.01)
