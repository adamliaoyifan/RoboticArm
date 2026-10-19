import hashlib
import numpy as np
import pytest
from luggage_perception.depth_boundary_mesh import StaticBoundaryMesh


def face(x):
    return [[x,-1.,-1.],[x,1.,-1.],[x,0.,1.]]


def make(values):
    triangles=np.array(values,dtype='<f8')
    return StaticBoundaryMesh(triangles,scene_revision='scene',
                              expected_sha256=hashlib.sha256(triangles.tobytes()).hexdigest())


def hit(mesh,origin=(0.,0.,0.),end=(1.,0.,0.)):
    return mesh.first_hit(origin,end,deadline=10.,clock=lambda:0.)


def test_tree_keeps_nearest_surface_and_binds_ray():
    mesh=make([face(x) for x in np.linspace(.765,2.,90)]+[face(-.2)])
    result=hit(mesh)
    assert result.distance_m==pytest.approx(.765)
    assert result.origin==(0.,0.,0.) and result.direction==(1.,0.,0.)
    assert result.scene_revision=='scene'


def test_input_mutation_cannot_change_frozen_mesh():
    triangles=np.array([face(.765)])
    digest=hashlib.sha256(triangles.astype('<f8').tobytes()).hexdigest()
    mesh=StaticBoundaryMesh(triangles,scene_revision='scene',expected_sha256=digest)
    triangles[:]=0.
    assert hit(mesh).distance_m==pytest.approx(.765)


@pytest.mark.parametrize('origin,end',[
    ((0.,0.,0.),(0.,1.,0.)),((0.,2.,0.),(1.,2.,0.)),((1.,0.,0.),(2.,0.,0.)),
])
def test_misses_are_not_hits(origin,end):
    assert hit(make([face(.765)]),origin,end) is None


def test_timeout_is_distinct_from_no_surface():
    with pytest.raises(TimeoutError):
        make([face(.765)]).first_hit((0,0,0),(1,0,0),deadline=1.,clock=lambda:1.)


def test_wrong_identity_and_oversized_inputs_refused():
    with pytest.raises(ValueError,match='identity mismatch'):
        StaticBoundaryMesh([face(.765)],scene_revision='scene',expected_sha256='wrong')
    with pytest.raises(ValueError,match='bounded'):
        make([face(.765)]*20001)
