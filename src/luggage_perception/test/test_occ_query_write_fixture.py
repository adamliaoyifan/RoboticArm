"""Actual workload classes, measured after the unchanged input preparation."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

from luggage_description.container_geometry import ContainerGeometry, Chamfer
from luggage_description.container_geometry_batch import contains_points
from luggage_perception.cargo_occ_input import OccInputPolicy, prepare_occ_observation


@pytest.mark.parametrize('resolution', [.01,.02,.05])
@pytest.mark.parametrize('count', [30000,100000])
@pytest.mark.parametrize('outside', [.5,.9])
@pytest.mark.parametrize('crossing', [0.,.5])
def test_raw_and_actually_selected_ray_classes(resolution,count,outside,crossing):
    path = Path(__file__).with_name('occ_query_write_fixture.py')
    spec = importlib.util.spec_from_file_location('write_workload',path)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    hull = ContainerGeometry(1,'container_link',1.49,1.97,.53,2.01,
        Chamfer('positive_y',.55,.985,.90))
    points, origin = module.make_points(hull,count,outside,crossing)
    assert len(points)==count
    assert abs((~contains_points(hull,points)).mean()-outside)<1e-12
    prepared = prepare_occ_observation(points,origin,hull,resolution,
        OccInputPolicy(.58,(0.,0.,1.27),0.))
    rays = prepared.ray_points
    if len(rays)>2000:rays=rays[::max(1,len(rays)//2000)]
    assert len(prepared.hit_points)==1000
    assert len(rays)==(1000 if crossing==0 else 2000)
    assert (~contains_points(hull,rays)).mean()==crossing
    assert not points.flags.writeable or not np.shares_memory(points,prepared.ray_points)
