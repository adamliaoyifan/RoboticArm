"""Evaluate real launch callbacks and conditions without starting a process."""
import importlib.util
from pathlib import Path
from launch import LaunchContext
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import pytest


def launch_module():
    path=Path(__file__).resolve().parents[1]/'launch/hardware_pick.launch.py'
    spec=importlib.util.spec_from_file_location('hardware_plan_only_launch',path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('plan_only,executor_enabled,backend,execute', [
    ('true',False,'stub',False),('false',True,'hardware',True)])
def test_plan_only_gates_actual_node_parameters_and_executor_include(
        monkeypatch,plan_only,executor_enabled,backend,execute):
    module=launch_module()
    context=LaunchContext()
    context.launch_configurations.update(dict(plan_only=plan_only,start_executor='true',
        start_planning='true',use_moveit='true',frame_runtime='false',
        scene_tf_config='/offline/scene.yaml'))
    for action in module._execution_gate(context):
        action.execute(context)
    condition=IfCondition(LaunchConfiguration('start_executor'))
    assert condition.evaluate(context) is executor_enabled
    # Capture the resolved arguments passed to Node by production callbacks.
    monkeypatch.setattr(module,'Node',lambda **kwargs: kwargs)
    nodes=module._frame_nodes(context)
    vacuum=next(node for node in nodes if node['name']=='vacuum_controller')
    assert vacuum['parameters'][0]['backend']==backend
    move_group=module._move_group(context)[0]
    assert move_group['parameters'][-1]['allow_trajectory_execution'] is execute


@pytest.mark.parametrize('distro,plugin_key', [('humble','planning_plugin'),('jazzy','planning_plugins')])
def test_moveit_configuration_matches_installed_distro(monkeypatch,distro,plugin_key):
    module=launch_module()
    monkeypatch.setenv('ROS_DISTRO',distro)
    monkeypatch.setattr(module,'Node',lambda **kwargs: kwargs)
    context=LaunchContext()
    context.launch_configurations.update(dict(plan_only='true',start_planning='true',
        use_moveit='true',scene_tf_config='/offline/scene.yaml'))
    node=module._move_group(context)[0]
    ompl=node['parameters'][0]['ompl']
    assert plugin_key in ompl
    assert isinstance(ompl[plugin_key],list if distro=='jazzy' else str)
    assert node['parameters'][-1]['allow_trajectory_execution'] is False


def test_explicit_manifest_is_passed_to_strict_profile_gate(monkeypatch):
    from luggage_planning.deploy import deploy_profile
    module=launch_module()
    captured=[]
    def resolve(scene, atlas, **kwargs):
        captured.append((scene,atlas,kwargs))
        return dict(scene_tf_path=scene,atlas_runtime_config=atlas)
    monkeypatch.setattr(deploy_profile,'resolve_deploy_profile',resolve)
    context=LaunchContext()
    context.launch_configurations.update(dict(scene_tf_config='/frozen/scene.yaml',
        atlas_runtime_config='/frozen/atlas.yaml',deploy_asset_manifest='/frozen/manifest.yaml',
        start_planning='true'))
    module._deploy_profile_gate(context)
    assert captured == [('/frozen/scene.yaml','/frozen/atlas.yaml',
                        dict(manifest_arg='/frozen/manifest.yaml',check_atlas=True))]


@pytest.mark.parametrize('grid_resolution', ['', '0.01', '0.05'])
@pytest.mark.parametrize('enabled_by', ['start_placement','start_placement_prior'])
def test_placement_launch_starts_required_providers_with_shared_identity(monkeypatch, enabled_by, grid_resolution):
    from luggage_planning.deploy import hardware_placement_launch
    from luggage_description.runtime_graph import STACK_PEERS, endpoint_registry, resolve_peers
    module=launch_module()
    context=LaunchContext()
    context.launch_configurations["cargo_map_resolution_m"]=grid_resolution
    context.launch_configurations['occupancy_backend']='grid_g'
    context.launch_configurations['scene_tf_config']='/unused-for-grid-g.yaml'
    context.launch_configurations.update({enabled_by: 'true', 'start_planning':'true'})
    peers=resolve_peers('hardware_pick_driver', stack=STACK_PEERS)
    frame=dict(frame_runtime=True,frame_peers=','.join(peers),frame_identity_json='frozen')
    monkeypatch.setattr(module,'_frame_runtime_params',lambda ctx: frame)
    monkeypatch.setattr(hardware_placement_launch,'Node',lambda **kwargs: kwargs)
    nodes=module._placement_service_nodes(context)
    assert {node['name'] for node in nodes} == {
        'cargo_volume_mapper','placement_planner','trajectory_selector'}
    registry=endpoint_registry(peers)
    assert registry['/trajectory_selector/select_place_path_v2']['provider']=='trajectory_selector'
    for node in nodes:
        params=node['parameters'][0]
        assert params['frame_peers']==frame['frame_peers']
        assert params['frame_identity_json']=='frozen'
        assert params['use_sim_time'] is False
    assert nodes[0]['parameters'][0]['require_confirmed_map'] is True
    expected=float(grid_resolution) if grid_resolution else .01
    assert nodes[0]['parameters'][0]['resolution']==expected
    assert nodes[1]['parameters'][0]['floor_prior_resolution']==expected
    assert nodes[2]['parameters'][0]['corridor_planner']=='graph'


@pytest.mark.parametrize('missing', ['frame_runtime','start_planning','start_scene','use_moveit'])
def test_incomplete_placement_graph_is_rejected_before_identity_build(missing):
    module=launch_module()
    context=LaunchContext()
    context.launch_configurations.update(dict(start_placement_prior='true',
        frame_runtime='true',start_planning='true',start_scene='true',use_moveit='true'))
    context.launch_configurations[missing]='false'
    with pytest.raises(RuntimeError,match=missing):
        module._frame_runtime_params(context)


def test_certifier_receives_exact_planning_model(monkeypatch):
    module=launch_module()
    context=LaunchContext()
    context.launch_configurations['start_planning']='true'
    monkeypatch.setattr(module,'Node',lambda **kwargs: kwargs)
    monkeypatch.setattr(module,'_robot_description',lambda scene: {'robot_description':'exact-urdf'})
    monkeypatch.setattr(module,'_semantic_srdf',lambda: {'robot_description_semantic':'exact-srdf'})
    node=module._certification_nodes(context)[0]
    assert node['executable']=='validate_route_node'
    assert node['parameters']==[{'robot_description':'exact-urdf'},
        {'robot_description_semantic':'exact-srdf'},{'use_sim_time':False}]
    context.launch_configurations['start_planning']='false'
    assert module._certification_nodes(context)==[]
