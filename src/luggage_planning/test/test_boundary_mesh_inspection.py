"""Ray diagnostics must preserve thin/occluded surfaces and reject misses."""
import importlib.util
from pathlib import Path
import numpy as np
import pytest

path = Path(__file__).resolve().parents[3] / 'scripts/placement_prior/inspect_boundary_mesh.py'
spec = importlib.util.spec_from_file_location('boundary_mesh_inspection', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def face(x):
    return [[x,-1.,-1.],[x,1.,-1.],[x,0.,1.]]


def test_nearest_positive_surface_keeps_thin_separate_walls():
    triangles = np.array([face(.785),face(-.2),face(.765)])
    distances = module.intersections(triangles,np.zeros(3),np.array([1.,0.,0.]))
    assert distances.tolist() == pytest.approx([.765,.785])


@pytest.mark.parametrize('origin,direction',[
    ([0.,0.,0.],[0.,1.,0.]),  # parallel
    ([0.,2.,0.],[1.,0.,0.]),  # outside triangle
    ([1.,0.,0.],[1.,0.,0.]),  # behind ray
])
def test_misses_do_not_create_boundary(origin,direction):
    assert len(module.intersections(np.array([face(.765)]),np.array(origin),np.array(direction))) == 0
