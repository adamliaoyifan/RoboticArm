"""Actual workload classes, measured after the unchanged input preparation."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from luggage_description.container_geometry import ContainerGeometry, Chamfer
from luggage_description.container_geometry_batch import contains_points
from luggage_perception.cargo_occ_input import OccInputPolicy, prepare_occ_observation


def fixture_module():
    path = Path(__file__).with_name('occ_query_write_fixture.py')
    spec = importlib.util.spec_from_file_location('write_workload', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('resolution', [.01,.02,.05])
@pytest.mark.parametrize('count', [30000,100000])
@pytest.mark.parametrize('outside', [.5,.9])
@pytest.mark.parametrize('crossing', [0.,.5])
def test_raw_and_actually_selected_ray_classes(resolution,count,outside,crossing):
    module = fixture_module()
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


def test_index_observer_retains_nonempty_work_before_empty_refresh():
    from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
    from luggage_perception.occupancy_map_store import OccupancyMapStore, MapUpdate, UpdateKind

    stores = [OccupancyMapStore(CargoVolumeMapper((.2, .2, .2), (0, 0, .1), 0, .02),
                               epoch='observed', clock_ns=lambda: 1_000_000_000)
              for _ in range(2)]
    update = MapUpdate(operation_id='hit', kind=UpdateKind.INTEGRATE_VIEW,
        geometry_hash=stores[0].mapper.geometry_hash, expected_snapshot=stores[0].identity,
        points=[(0., 0., .1)], origin=None, stamp_key=(1, 0),
        acquisition_stamp_ns=1_000_000_000, schema_version=1, settled=True)
    reference = stores[0].apply_update(update)
    owner = stores[1]._query_indexes
    with fixture_module().capture_index_maintenance(owner) as stages:
        actual = stores[1].apply_update(update)
    assert actual.accepted and reference.accepted
    assert actual.snapshot == reference.snapshot
    assert stages[0]['changed_cells'] > 0
    assert stages[0]['work']['dirty_blocks'] > 0
    assert stages[-1]['changed_cells'] == stages[-1]['work']['dirty_blocks'] == 0
    assert owner.work['dirty_blocks'] == 0
    assert all(s['completed'] and s['end_ns'] >= s['begin_ns'] for s in stages)
    assert 'apply_delta' not in vars(owner)


def test_index_observer_restores_existing_override_on_failure():
    def failing(*args, **kwargs):
        raise RuntimeError('maintenance failed')
    owner = SimpleNamespace(apply_delta=failing)
    with pytest.raises(RuntimeError, match='maintenance failed'):
        with fixture_module().capture_index_maintenance(owner) as stages:
            owner.apply_delta(None, SimpleNamespace(indices=[3], rebuild=False))
    assert owner.apply_delta is failing
    assert len(stages) == 1 and not stages[0]['completed']
    assert stages[0]['end_ns'] >= stages[0]['begin_ns']


def test_index_observer_legacy_owner_is_explicitly_empty():
    with fixture_module().capture_index_maintenance(None) as stages:
        assert stages == []
