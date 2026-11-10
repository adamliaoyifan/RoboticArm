"""Batch geometry matches the scalar authority, including chamfer EPS."""
import numpy as np
import pytest
from luggage_description.container_geometry import ContainerGeometry, Chamfer, contains_point, EPS
from luggage_description.container_geometry_batch import contains_points, segment_hull_intervals


def hull(chamfer=True):
    return ContainerGeometry(1,'container_link',1.,1.,0.,1.,
                             Chamfer('positive_y',.2,.5,.3) if chamfer else None)


@pytest.mark.parametrize('chamfer',[False,True])
def test_point_oracle_and_immutable_input(chamfer):
    h=hull(chamfer);rng=np.random.default_rng(20261023)
    points=rng.uniform([-.8,-.8,-.2],[.8,.8,1.2],(10000,3))
    boundary=np.array([(x,y,z) for x in (-.5,.5) for y in (-.5,.2,.5) for z in (0.,.3,1.)])
    points=np.concatenate([points,*[boundary+e for e in (0.,EPS,-EPS,2*EPS,-2*EPS)]])
    points.setflags(write=False)
    np.testing.assert_array_equal(contains_points(h,points),[contains_point(h,p) for p in points])
    assert not contains_points(h,[[np.nan,0,0],[0,np.inf,0]]).any()
    assert len(contains_points(h,[]))==0


def test_segment_parallel_tangent_zero_and_nonfinite():
    h=hull(False)
    ends=np.array([[.8,0,.5],[.8,.6,.5],[.5,.5,.5],[0,0,.5],[np.nan,0,0]])
    starts=np.array([[-.8,0,.5],[-.8,.6,.5],[.5,.5,.5],[0,0,.5],[0,0,0]])
    lo,hi,valid=segment_hull_intervals(h,starts,ends)
    assert valid.tolist()==[True,False,True,True,False]
    assert lo[0]==pytest.approx(.3/1.6) and hi[0]==pytest.approx(1.3/1.6)


def test_chamfer_segment_endpoints_are_on_hull():
    h=hull();rng=np.random.default_rng(20261023)
    a=rng.uniform([-.8,-.8,-.2],[.8,.8,1.2],(10000,3))
    b=rng.uniform([-.8,-.8,-.2],[.8,.8,1.2],(10000,3))
    lo,hi,valid=segment_hull_intervals(h,a,b)
    for t in (lo,hi,(lo+hi)*.5):
        assert contains_points(h,(a+(b-a)*t[:,None])[valid]).all()
