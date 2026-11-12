"""Exercise selected owner configuration, launch construction and real paging."""
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from luggage_mapping.configuration import owner_configuration
from luggage_mapping.observer import ObserverConfiguration, ObserverSnapshotPort

ROOT = Path(__file__).resolve().parents[3]
SCENE = ROOT/'src/luggage_description/config/scene_tf_deploy_update.yaml'
CONFIG = ROOT/'src/uld_occ_observer/config/observer.yaml'


@pytest.mark.parametrize('backend', ['observer', 'grid_g'])
@pytest.mark.parametrize('resolution', ['', '0.02'])
def test_one_owner_and_consumer_resolution(monkeypatch, tmp_path, backend, resolution):
    from luggage_planning import hardware_placement_launch as launch
    monkeypatch.setattr(launch, 'Node', lambda **kwargs: kwargs)
    owner = owner_configuration(backend=backend, scene=SCENE,
        resolution_override=resolution, observer_config=CONFIG, artifact_root=tmp_path)
    nodes = launch.placement_nodes(frame={}, scene=str(SCENE), atlas='', poses='', owner=owner)
    owners = [n for n in nodes if n['name'] in ('uld_occ_observer', 'cargo_volume_mapper')]
    assert len(owners) == 1
    assert owners[0]['name'] == ('uld_occ_observer' if backend == 'observer' else 'cargo_volume_mapper')
    params = next(n for n in nodes if n['name'] == 'placement_planner')['parameters'][0]
    expected = float(resolution or '.01')
    assert params['floor_prior_resolution'] == expected == owner.resolution_m
    assert params['volume_evidence_source'] == backend
    if backend == 'observer':
        source = owners[0]['parameters'][0]
        assert source['geometry_file'] == params['volume_observer_geometry_file']
        assert source['config_file'] == params['volume_observer_config_file']
        configuration = ObserverConfiguration.load(source['geometry_file'], source['config_file'])
        assert configuration.resolution_m == expected
        assert params['validity_backend'] == 'observer_o'
    else:
        assert owners[0]['parameters'][0]['resolution'] == expected
        assert params['volume_grid_g_cells'] is True
        assert params['validity_backend'] == 'grid_g'


def test_optical_input_is_explicit_and_never_semantic_cloud(monkeypatch, tmp_path):
    from luggage_planning import hardware_placement_launch as launch
    monkeypatch.setattr(launch, 'Node', lambda **kwargs: kwargs)
    owner = owner_configuration(backend='observer', scene=SCENE,
        observer_config=CONFIG, artifact_root=tmp_path)
    kwargs = dict(frame={}, scene=str(SCENE), atlas='', poses='', owner=owner,
                  observer_input_mode='subscription')
    with pytest.raises(ValueError, match='Optical cloud'):
        launch.placement_nodes(**kwargs)
    nodes = launch.placement_nodes(**kwargs, observer_cloud_topic='/measured/optical',
                                  observer_optical_frame='camera_optical')
    params = nodes[0]['parameters'][0]
    assert params['cloud_topic'] == '/measured/optical'
    assert params['optical_frame'] == 'camera_optical'


def test_invalid_owner_and_tampered_effective_configuration_reject(tmp_path):
    with pytest.raises(ValueError, match='occupancy_backend'):
        owner_configuration(backend='both', scene=SCENE)
    owner = owner_configuration(backend='observer', scene=SCENE,
        observer_config=CONFIG, artifact_root=tmp_path)
    Path(owner.observer_config_file).write_text('resolution_m: 0.10\n')
    with pytest.raises(ValueError, match='CONTENT_MISMATCH'):
        owner_configuration(backend='observer', scene=SCENE,
            observer_config=CONFIG, artifact_root=tmp_path)


def test_planning_compatibility_exports_are_same_classes():
    from luggage_mapping.source import OperationMapSource, OperationMapRefresh
    from luggage_planning.joint_operation.map_source import OperationMapSource as LegacySource
    from luggage_planning.joint_operation.map_source import OperationMapRefresh as LegacyRefresh
    from luggage_mapping.surface import ConfirmedSurface
    from luggage_planning.placement_prior.surface import ConfirmedSurface as LegacySurface
    from luggage_planning.joint_operation.observer import ObserverSnapshotPort as LegacyPort
    assert (LegacySource, LegacyRefresh, LegacySurface, LegacyPort) == (
        OperationMapSource, OperationMapRefresh, ConfirmedSurface, ObserverSnapshotPort)


def page_port(dimensions, *, damage=None):
    count = math.prod(dimensions)
    identity = dict(epoch=1, map_revision=7, observed_revision=8,
                    last_integrated_stamp_ns=1, geometry_hash='geometry',
                    config_digest='config', clock_domain='ros')
    cursors = []
    def call(client, request, deadline, stage):
        cursors.append(request.cursor)
        size = min(request.limit, count-request.cursor)
        response = NS(status='OK', identity=NS(**identity),
            validity=NS(state='VALID', epoch=1, generation=2),
            states=bytes(size), next_cursor=request.cursor+size,
            done=request.cursor+size == count)
        if damage == 'stalled':
            response.next_cursor = request.cursor
        if damage == 'mixed':
            response.identity.map_revision += 1
        if damage == 'short':
            response.states = response.states[:-1]
        return response
    port = object.__new__(ObserverSnapshotPort)
    port.config = NS(dimensions_xyz=dimensions)
    port.services = {'read_cells': NS(Request=lambda **kwargs: NS(**kwargs))}
    port.clients = {'read_cells': object()}
    port.call = call
    return port, identity, cursors


def test_full_one_cm_grid_reads_all_67_pages_without_magic_cap():
    port, identity, cursors = page_port((149, 197, 148))
    states = port._pages('read_cells', 'lease', 4344244, identity, 2, float('inf'))
    assert len(states) == 4344244 and len(cursors) == 67
    assert cursors[-1] == 66*65536
    assert set(states) == {0}, 'unobserved cells remain unknown'


@pytest.mark.parametrize('damage', ['stalled', 'mixed', 'short'])
def test_bad_pages_refuse(damage):
    port, identity, _ = page_port((4, 4, 4), damage=damage)
    with pytest.raises(ValueError, match='OBSERVER_'):
        port._pages('read_cells', 'lease', 64, identity, 2, float('inf'))


@pytest.mark.parametrize('lower_state', [1, 0, 2])
def test_compute_callback_consumes_shared_owner_capture(monkeypatch, tmp_path, lower_state):
    import importlib.util
    from test_joint_operation_observer import configuration, snapshot
    import luggage_mapping.observer as observer
    config, geometry, config_file = configuration(tmp_path)
    states = [1]*math.prod(config.dimensions_xyz)
    nx, ny, _ = config.dimensions_xyz
    states[:2*nx*ny] = [lower_state]*(2*nx*ny)
    captured = snapshot(config, states)
    calls = []
    class Port:
        def __init__(self, node, *, configuration, **kwargs):
            self.config = configuration
        def capture(self, **kwargs):
            calls.append(kwargs)
            return captured
    monkeypatch.setattr(observer, 'ObserverSnapshotPort', Port)
    path = ROOT/'src/luggage_packing/scripts/placement_planner_node.py'
    spec = importlib.util.spec_from_file_location('map_compute_callback', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    params = dict(volume_observer_geometry_file=str(geometry),
        volume_observer_config_file=str(config_file), volume_observer_resolution=0.,
        volume_operation_timeout_s=20., volume_service_timeout_s=2.,
        volume_observer_prefix='/uld_occ_observer', volume_max_age_ms=1000,
        hull_margin=-1.)
    admitted = []
    planner = NS(_admission=NS(validity=NS(epoch='1')), _frame_scene_revision='scene',
        get_parameter=lambda name: NS(value=params[name]),
        _admit_surface_map=lambda surface, source: admitted.append((surface, source)) or True)
    surface, verifier = module.PlacementPlannerNode._observer_snapshot_compute(planner)
    assert surface['resolution'] == config.resolution_m
    assert planner._observer_snapshot_port.config == config
    assert calls[0]['expected_epoch'] == 1 and calls[0]['expected_revision'] is None
    assert admitted[0][1] == 'observer_lease' and callable(verifier)
    # The solver emits container-absolute centers; the voxel origin is not
    # the legacy projection's floor-relative zero.
    candidate = dict(center_container=[0., 0., config.descriptor()['floor_z']+.1],
                     footprint=[.1, .1], size=[.1, .1, .2])
    assert (verifier(candidate) is None) == (lower_state == 1)
    upper = dict(candidate, center_container=[0., 0., config.descriptor()['floor_z']+.3])
    assert verifier(upper) is None
    params['volume_observer_resolution'] = .02
    with pytest.raises(RuntimeError, match='SOURCE_RESOLUTION_MISMATCH'):
        module.PlacementPlannerNode._observer_snapshot_compute(planner)


def test_ordinary_launch_callback_defaults_to_o(monkeypatch, tmp_path):
    from launch import LaunchContext
    import importlib.util
    from luggage_planning import hardware_placement_launch
    monkeypatch.setenv('ROS_HOME', str(tmp_path))
    spec = importlib.util.spec_from_file_location('map_owner_hardware_launch',
        ROOT/'src/luggage_planning/launch/hardware_pick.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, '_frame_runtime_params', lambda context: {})
    monkeypatch.setattr(hardware_placement_launch, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update(start_placement='true', scene_tf_config=str(SCENE),
        atlas_runtime_config='', robot_poses_config='', observer_config_file='')
    nodes = module._placement_service_nodes(context)
    assert nodes[0]['name'] == 'uld_occ_observer'
    assert all(node['name'] != 'cargo_volume_mapper' for node in nodes)
    assert nodes[1]['parameters'][0]['volume_evidence_source'] == 'observer'
