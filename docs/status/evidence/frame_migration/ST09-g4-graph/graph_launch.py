"""ST09 graph campaign launch (generated). Production v2 chain, mock controllers."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from luggage_description.xacro_robot_with_scene_base import expand_and_patch


def _srdf():
    path = os.path.join(
        get_package_share_directory("luggage_description"),
        "config", "S20_with_camera.srdf")
    with open(path, "r", encoding="utf-8") as handle:
        xml = handle.read()
    return xml.replace('name="elfin_s20_with_camera"', 'name="S20"', 1)


def _urdf():
    xacro_path = os.path.join(
        get_package_share_directory("luggage_description"),
        "urdf", "elfin_s20_with_camera.urdf.xacro")
    urdf, _pose = expand_and_patch(
        xacro_path, config_path='/tmp/elfin-st09-g3/install/g4/luggage_description/share/luggage_description/config/scene_tf.yaml.example',
        xacro_args=["hardware_plugin:=mock_components/GenericSystem",
                    "use_gz_sim:=false"])
    return urdf


def _nodes(context):
    urdf = _urdf()
    frame = {'frame_runtime': True, 'frame_identity_json': '{"atlas_certification_sha256": "9964386c2f2d7d83adf8ab81b01e7d60b1ce626a3f5d1124e03baaa364c5354a", "atlas_sha256": "15a102c8fb5ca8d7346f809ada53f1fa3248d87700ad9e3a0b7915f3fadd2196", "calibration_revision": "graph-campaign", "contract_version": 2, "endpoints": {"/cargo_map/add_placed_box_v2": "luggage_msgs/srv/AddPlacedBoxV2", "/cargo_map/remove_placed_box_v2": "luggage_msgs/srv/RemovePlacedBoxV2", "/frame_contracts/v2/runtime_identity": "luggage_msgs/msg/RuntimeIdentityV2", "/luggage/cargo_map/surface_v2": "luggage_msgs/msg/SurfaceContractV2", "/motion_planner/plan_motion_v2": "luggage_msgs/action/PlanMotionV2", "/motion_planner/probe_motion_segment_v2": "luggage_msgs/srv/ProbeMotionSegmentV2", "/placement_planner/compute_placement_v2": "luggage_msgs/srv/ComputePlacementV2", "/scene_manager/add_scene_box_v2": "luggage_msgs/srv/AddSceneBoxV2", "/scene_manager/remove_scene_box_v2": "luggage_msgs/srv/RemoveSceneBoxV2", "/trajectory_selector/select_place_path_v2": "luggage_msgs/srv/SelectPlacePathV2", "/vacuum/command_v2": "luggage_msgs/srv/VacuumCommandV2", "/waypoint_generator/build_motion_sequence_v2": "luggage_msgs/srv/BuildMotionSequenceV2"}, "geometry_hash": "cb41f55f243786d50ab30997291083ab40467507abcf78a00e55da77ffc00856", "joint_names": ["elfin_joint1", "elfin_joint2", "elfin_joint3", "elfin_joint4", "elfin_joint5", "elfin_joint6"], "scene_revision": "b40c85ae6eb2eabc47d7da2ffb23b72c", "srdf_sha256": "99653626dedff09ca5fb4ecc9858154677fe832692a4550a073e5f4251ce4be3", "tool_link": "suction_contact_frame", "urdf_sha256": "3bf1f9b8149df2cc446282d5b58aafe68488b69a7727da28b499bdc5067453a8"}', 'frame_peers': 'placement_planner,waypoint_generator,trajectory_selector,motion_planner,vacuum_controller,scene_manager,cargo_volume_mapper,motion_occ_driver', 'frame_scene_revision': 'b40c85ae6eb2eabc47d7da2ffb23b72c'}
    scene_tf = '/tmp/elfin-st09-g3/install/g4/luggage_description/share/luggage_description/config/scene_tf.yaml.example'
    description = {"robot_description": ParameterValue(urdf, value_type=str)}
    semantic = {"robot_description_semantic": _srdf()}
    controllers = os.path.join(
        get_package_share_directory("elfin_control"),
        "config", "elfin_controllers.yaml")
    only = LaunchConfiguration("only").perform(context)
    wanted = [n.strip() for n in only.split(",") if n.strip()]

    def want(name):
        return not wanted or name in wanted

    nodes = []
    nodes.append(Node(
        package="luggage_description", executable="container_tf_publisher",
        name="container_tf_publisher", output="screen",
        parameters=[{"scene_tf_config": scene_tf, "republish_period": 0.0,
                     "frame_scene_revision": frame.get("frame_scene_revision", "")}]))
    nodes.append(Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="screen",
        parameters=[description],
        condition=_cond(want("robot_state_publisher"))))
    nodes.append(Node(
        package="controller_manager", executable="ros2_control_node",
        name="controller_manager", output="screen",
        parameters=[description, controllers],
        remappings=[("~/robot_description", "/robot_description")],
        condition=_cond(want("controller_manager"))))
    nodes.append(Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager",
                   "/controller_manager", "--controller-manager-timeout", "60"],
        output="screen",
        condition=_cond(want("controllers"))))
    nodes.append(Node(
        package="moveit_ros_move_group", executable="move_group",
        name="move_group", output="screen",
        parameters=[description, semantic,
                    {"publish_robot_description_semantic": True,
                      "allow_trajectory_execution": True,
                      "use_sim_time": False}],
        condition=_cond(want("move_group"))))
    nodes.append(Node(
        package="luggage_planning", executable="scene_manager_node.py",
        name="scene_manager", output="screen",
        parameters=[{"scene_tf_config": scene_tf, "world_frame": "world",
                     "pickup_object_id": "pickup_box", "auto_sync": False,
                     "use_sim_time": False, **frame}]))
    nodes.append(Node(
        package="luggage_planning", executable="waypoint_generator_node.py",
        name="waypoint_generator", output="screen",
        parameters=[{"scene_tf_config": scene_tf, "world_frame": "world",
                     "place_slot_frame": "elfin_base_link",
                     "use_sim_time": False, **frame}]))
    nodes.append(Node(
        package="luggage_planning", executable="trajectory_selector_node.py",
        name="trajectory_selector", output="screen",
        parameters=[{"scene_tf_config": scene_tf,
                     "use_sim_time": False, **frame}]))
    nodes.append(Node(
        package="luggage_planning", executable="motion_planner_node.py",
        name="motion_planner", output="screen",
        parameters=[{"scene_tf_config": scene_tf, "use_sim_time": False,
                     **frame}]))
    nodes.append(Node(
        package="luggage_perception", executable="cargo_volume_mapper_node.py",
        name="cargo_volume_mapper", output="screen",
        parameters=[{"cargo_cloud_topic": "/luggage/semantic/cargo_points_untracked",
                     "base_frame": "container_link", "cloud_buffer_size": 12,
                     "freshness_window_sec": 0.20, "use_sim_time": False,
                     **frame}]))
    nodes.append(Node(
        package="luggage_packing", executable="placement_planner_node.py",
        name="placement_planner", output="screen",
        parameters=[{"scene_tf_config": scene_tf,
                     "use_sim_time": False, **frame}]))
    nodes.append(Node(
        package="luggage_planning", executable="vacuum_controller_node.py",
        name="vacuum_controller", output="screen",
        parameters=[{"use_sim_time": False, **frame}]))
    # Controllers chain after the controller manager is up.
    from launch.actions import RegisterEventHandler
    from launch.event_handlers import OnProcessExit
    spawners = []
    arm = Node(
        package="controller_manager", executable="spawner",
        arguments=["elfin_arm_controller", "--controller-manager",
                   "/controller_manager", "--controller-manager-timeout", "60"],
        output="screen",
        condition=_cond(want("controllers")))
    return [n for n in nodes if n is not None], arm


def _cond(enabled):
    from launch.conditions import IfCondition
    return IfCondition("true" if enabled else "false")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("only", default_value=""),
        OpaqueFunction(function=_nodes_opaque),
    ])


def _nodes_opaque(context):
    nodes, arm = _nodes(context)
    return nodes + [arm]
