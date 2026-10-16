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
