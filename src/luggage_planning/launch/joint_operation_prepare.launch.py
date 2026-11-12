"""Plan-only service with private native backends; external measured inputs required.

The parameter file must bind the same world-rooted URDF/SRDF and solver config
for every node, plus the root's scene_tf_config, scene revision/container transform/contact model.
The root generates ordered opening waypoints; callers provide measured box/map
inputs and placement candidates, never a separately hand-authored waypoint list.
No controller, vacuum, perception trigger or scene/map writer is started.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    parameters = [LaunchConfiguration('model_parameters'),{
        'max_preparation_budget_s':ParameterValue(LaunchConfiguration('max_preparation_budget_s'),value_type=float),
        'preparation_refresh_attempts':ParameterValue(LaunchConfiguration('preparation_refresh_attempts'),value_type=int),
    }]
    return LaunchDescription([
        DeclareLaunchArgument('model_parameters',description='Pinned native/root ROS parameter file'),
        DeclareLaunchArgument('start_native_backends',default_value='true'),
        DeclareLaunchArgument('max_preparation_budget_s',default_value='300.0',
            description='Diagnostic preparation watchdog; production latency target awaits full-model baseline'),
        DeclareLaunchArgument('preparation_refresh_attempts',default_value='2',
            description='Bounded latest O full-chain recertification attempts within the original deadline'),
        *[Node(package='luggage_planning',executable=executable,name=name,output='screen',
            parameters=parameters,condition=IfCondition(LaunchConfiguration('start_native_backends')))
          for executable,name in [('grasp_local_check_node','grasp_local_check'),
                                  ('snapshot_joint_path_node','snapshot_joint_path'),
                                  ('validate_route_node','validate_route_v2')]],
        Node(package='luggage_planning',executable='joint_operation_prepare_node.py',
             name='joint_operation_prepare',output='screen',parameters=parameters),
    ])
