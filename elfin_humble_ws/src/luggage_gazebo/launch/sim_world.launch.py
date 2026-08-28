"""Gazebo Fortress simulation world for the luggage loading closed loop.

ROS 2 port of the noetic luggage_gazebo/sim_world.launch:

- starts Gazebo Fortress (airport_loading.sdf) through ros_gz_sim,
- publishes scene static TF (container_tf_publisher) and the scene-based
  robot_description (robot_state_publisher),
- spawns pedestal / pickup platform / container from scene_tf.yaml, then
  the S20 arm with D435 + suction panel welded through world_base
  (fixed_world:=true, model pose at the origin; scene pose is on the joint),
  joints at the named observe pose (Noetic spawn_at_observe),
- bridges /clock and activates the two controllers inside gz sim,
- starts MoveIt 2 ``move_group`` (IK / OMPL / cartesian) after the arm
  controller is active. Pose-target pick segments cannot run on raw FJT.

Backend switching stays inside the description: mock runs through
elfin_mvp_bringup/control.launch.py, Fortress runs through this file. Joint
goals still use FollowJointTrajectory; Cartesian / pose targets go through
``move_group``.
"""

import os

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

from luggage_description.scene_tf_config_utils import (
    gazebo_container_model,
    gazebo_container_spawn_pose,
    gazebo_pedestal_spawn_pose,
    gazebo_pickup_platform_spawn_pose,
    load_scene_tf_config,
    pedestal_enabled,
    pickup_platform_enabled,
)
from luggage_description.xacro_robot_with_scene_base import expand_and_patch

WORLD_NAME = "airport_loading"

# Same branches as luggage_description/config/robot_poses.yaml.example.
# Used only if that YAML is missing; gz_ros2_control must not silently spawn
# on a different wrap than MoveIt/controller.
_FALLBACK_OBSERVE = [3.5702, -1.3263, -1.0965, 3.9564, 1.6234, 0.4522]


def _csv_floats(text, default):
    parts = [p.strip() for p in str(text or "").split(",") if p.strip()]
    if len(parts) < 2:
        return [float(default[0]), float(default[1])]
    return [float(parts[0]), float(parts[1])]


def _csv_strings(text):
    parts = [p.strip() for p in str(text or "").split(",") if p.strip()]
    return parts if parts else [""]


def _check_renderer():
    """GPU hard gate: refuse to start Fortress on llvmpipe (see plan)."""
    ws_root = os.path.dirname(os.path.dirname(get_package_prefix("luggage_gazebo")))
    script = os.path.join(ws_root, "scripts", "check_gpu_renderer.sh")
    if not os.path.isfile(script):
        raise RuntimeError("GPU gate script not found: %s" % script)
    result = os.popen("bash %s 2>&1" % script).read()
    if "GPU hard gate passed" not in result:
        raise RuntimeError(
            "GPU hard gate FAILED (renderer is not NVIDIA):\n%s" % result
        )


def _resource_path() -> str:
    """GZ_SIM_RESOURCE_PATH entries for model:// resolution.

    libsdformat rewrites URDF ``package://<pkg>/...`` mesh URIs into
    ``model://<pkg>/...`` during URDF->SDF conversion, so each package's
    ``install/<pkg>/share`` directory must be on the resource path for both
    the arm meshes (elfin_description, luggage_gazebo) and the scene models.
    Pre-scaled suitcase models live under luggage_gazebo/models.
    """
    entries = []
    for pkg in ("elfin_description", "luggage_gazebo", "luggage_description"):
        entries.append(os.path.join(get_package_prefix(pkg), "share"))
    entries.append(
        os.path.join(get_package_share_directory("luggage_gazebo"), "models")
    )
    return ":".join(entries)


def _load_named_pose_joints(poses_path: str, pose_name: str):
    """Return 6 observe-style joint values from robot_poses YAML."""
    with open(poses_path, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    pose = config["poses"][pose_name]
    values = [float(v) for v in pose["values"]]
    if len(values) != 6:
        raise ValueError("pose '%s' must have 6 joint values" % pose_name)
    return values


def _initial_joint_xacro_args(context):
    """xacro mappings so gz_ros2_control starts at the named pose (or mock defaults)."""
    spawn = LaunchConfiguration("spawn_at_observe").perform(context).lower() == "true"
    if not spawn:
        return []
    poses_path = LaunchConfiguration("robot_poses_config").perform(context)
    pose_name = LaunchConfiguration("observe_pose_name").perform(context)
    try:
        values = _load_named_pose_joints(poses_path, pose_name)
    except Exception as exc:
        print(
            "WARN: failed to load pose '%s' from %s (%s); using fallback observe"
            % (pose_name, poses_path, exc)
        )
        values = list(_FALLBACK_OBSERVE)
    return ["initial_joint%d:=%.6f" % (i + 1, values[i]) for i in range(6)]


def _robot_description_param(scene_tf_config: str, initial_joint_args):
    """Gazebo+camera URDF. Shared by robot_state_publisher and move_group.

    MoveItConfigsBuilder would otherwise load the bare S20 xacro (no suction
    / camera). IK to suction_contact_frame needs this expanded model.
    """
    helper = os.path.join(
        get_package_prefix("luggage_description"),
        "lib",
        "luggage_description",
        "xacro_robot_with_scene_base",
    )
    xacro_path = os.path.join(
        get_package_share_directory("luggage_description"),
        "urdf",
        "elfin_s20_with_camera.urdf.xacro",
    )
    # The in-gz controller_manager parses robot_description from RSP, so the
    # hardware plugin must already be GazeboSimSystem here.
    return {
        "robot_description": ParameterValue(
            Command(
                [
                    helper,
                    " ",
                    xacro_path,
                    " ",
                    scene_tf_config,
                    " ",
                    "hardware_plugin:=gz_ros2_control/GazeboSimSystem",
                    " ",
                    "use_gz_sim:=true",
                ]
                + [part for arg in initial_joint_args for part in (" ", arg)]
            ),
            value_type=str,
        )
    }


def _robot_state_publisher_actions(scene_tf_config: str, robot_description):
    return [
        Node(
            package="luggage_description",
            executable="container_tf_publisher",
            name="container_tf_publisher",
            output="screen",
            parameters=[{"scene_tf_config": scene_tf_config, "republish_period": 0.0}],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
    ]


def _gazebo_semantic_description():
    """SRDF that disables suction/camera self-collisions.

    MoveItConfigsBuilder loads elfin_moveit_config/S20.srdf (bare arm). The
    Gazebo URDF adds suction_panel / camera, which FCL then treats as
    colliding with elfin_link6 and planning aborts with error 99999.
    luggage_description's S20_with_camera.srdf has those pairs; robot name
    must be S20 to match the patched URDF.
    """
    path = os.path.join(
        get_package_share_directory("luggage_description"),
        "config",
        "S20_with_camera.srdf",
    )
    with open(path, "r", encoding="utf-8") as handle:
        xml = handle.read()
    xml = xml.replace('name="elfin_s20_with_camera"', 'name="S20"', 1)
    return {"robot_description_semantic": xml}


def _move_group_node(robot_description):
    """MoveIt 2 move_group with sim time and the gazebo robot_description."""
    moveit_config = (
        MoveItConfigsBuilder("S20", package_name="elfin_moveit_config")
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    moveit_config.robot_description = robot_description
    return Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            _gazebo_semantic_description(),
            {
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "use_sim_time": True,
            },
        ],
        condition=IfCondition(LaunchConfiguration("use_moveit")),
    )


def _create_node(model_name: str, sdf_path: str, spawn: dict):
    """One ros_gz_sim create process per scene model.

    A single persistent service bridge for /world/<w>/create would die with
    whichever process owned it, so each model gets its own create node that
    brings up (and tears down) its bridge.
    """
    return Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world", WORLD_NAME,
            "-file", sdf_path,
            "-name", model_name,
            "-x", "%.6f" % float(spawn["x"]),
            "-y", "%.6f" % float(spawn["y"]),
            "-z", "%.6f" % float(spawn["z"]),
            "-R", "%.6f" % float(spawn["R"]),
            "-P", "%.6f" % float(spawn["P"]),
            "-Y", "%.6f" % float(spawn["Y"]),
        ],
    )


def _scene_model_actions(scene_tf_config: str):
    """Spawn pedestal / pickup platform / container from scene_tf.yaml."""
    config = load_scene_tf_config(scene_tf_config)
    gz_share = get_package_share_directory("luggage_gazebo")
    spawns = []
    if pedestal_enabled(config):
        spawns.append(("robot_pedestal", gazebo_pedestal_spawn_pose(config)))
    if pickup_platform_enabled(config):
        spawns.append(("pickup_platform", gazebo_pickup_platform_spawn_pose(config)))
    container_model = gazebo_container_model(config)
    spawns.append((container_model, gazebo_container_spawn_pose(config)))
    return [
        _create_node(
            model,
            os.path.join(gz_share, "models", model, "model.sdf"),
            spawn,
        )
        for model, spawn in spawns
    ]


def _robot_create_node(context, initial_joint_args):
    """Welded gz URDF: world_base carries the scene pose; spawn at the origin."""
    scene_tf_config = LaunchConfiguration("scene_tf_config").perform(context)
    xacro_path = os.path.join(
        get_package_share_directory("luggage_description"),
        "urdf",
        "elfin_s20_with_camera.urdf.xacro",
    )
    urdf_xml, _pose = expand_and_patch(
        xacro_path,
        config_path=scene_tf_config,
        xacro_args=[
            "fixed_world:=true",
            "use_gz_sim:=true",
            "hardware_plugin:=gz_ros2_control/GazeboSimSystem",
        ]
        + list(initial_joint_args),
    )
    urdf_file = os.path.join(os.environ.get("TMPDIR", "/tmp"), "elfin_s20_gazebo.urdf")
    with open(urdf_file, "w", encoding="utf-8") as handle:
        handle.write(urdf_xml)

    return Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world", WORLD_NAME,
            "-file", urdf_file,
            "-name", "S20",
            "-x", "0",
            "-y", "0",
            "-z", "0",
            "-R", "0",
            "-P", "0",
            "-Y", "0",
        ],
    )


def _spawn_scene_and_robot(context, initial_joint_args):
    """Spawn pedestal/platform/container first, then the welded arm (Noetic order)."""
    scene_tf_config = LaunchConfiguration("scene_tf_config").perform(context)
    scene_nodes = _scene_model_actions(scene_tf_config)
    robot = _robot_create_node(context, initial_joint_args)
    if not scene_nodes:
        return [robot]
    return scene_nodes + [
        RegisterEventHandler(
            OnProcessExit(target_action=scene_nodes[0], on_exit=[robot])
        )
    ]


def _launch_setup(context):
    _check_renderer()
    scene_tf_config = LaunchConfiguration("scene_tf_config").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    initial_joint_args = _initial_joint_xacro_args(context)

    world_path = os.path.join(
        get_package_share_directory("luggage_gazebo"), "worlds", "airport_loading.sdf"
    )
    # Must start running (-r). gz_ros2_control SwitchController needs a
    # nonzero sim period; a paused world times out JSB/arm activation.
    # Moving-link gravity is off in the URDF so observe does not collapse
    # before the arm controller claims the joints.
    gz_args = ("-r -v 3 " if gui else "-s -r -v 3 ") + world_path

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    # D435 streams -> ROS names the noetic stack consumed. The gz sensor
    # publishes /d435/{image,depth_image,camera_info,points}; rgbd_camera
    # publishes the point cloud itself, so no depth_image_proc is needed.
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        output="screen",
        arguments=[
            "/d435/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/d435/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/d435/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/d435/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        remappings=[
            ("/d435/image", "/camera/color/image_raw"),
            ("/d435/depth_image", "/camera/depth/image_meters"),
            ("/d435/camera_info", "/camera/depth/camera_info"),
            ("/d435/points", "/camera/depth/points"),
        ],
        parameters=[{"use_sim_time": True}],
    )

    depth_republisher = Node(
        package="luggage_gazebo",
        executable="depth_image_republisher.py",
        name="depth_image_republisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    preprocessor = Node(
        package="luggage_perception",
        executable="sensor_preprocessor_node.py",
        name="sensor_preprocessor",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("luggage_perception"),
                "config", "sensor_preprocessor.yaml",
            ),
            {"use_sim_time": True},
        ],
    )

    # Persistent ROS exposure of the gz world services: runtime box spawn
    # (pickup_box_spawner) and the M4 vacuum kinematic follow (set_pose).
    service_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="world_service_bridge",
        output="screen",
        arguments=[
            "/world/%s/create@ros_gz_interfaces/srv/SpawnEntity" % WORLD_NAME,
            "/world/%s/remove@ros_gz_interfaces/srv/DeleteEntity" % WORLD_NAME,
            "/world/%s/set_pose@ros_gz_interfaces/srv/SetEntityPose" % WORLD_NAME,
        ],
    )

    scene_viz = Node(
        package="luggage_gazebo",
        executable="scene_viz_node.py",
        name="scene_viz",
        output="screen",
        parameters=[{
            "scene_tf_config": scene_tf_config,
            "use_sim_time": True,
        }],
    )
    box_spawner = Node(
        package="luggage_gazebo",
        executable="pickup_box_spawner_node.py",
        output="screen",
        # No use_sim_time: the spawner needs no clock, and a 1 kHz /clock
        # subscription burns CPU in every rclpy node that asks for it.
        parameters=[{
            "scene_tf_config": scene_tf_config,
            "visual_kind": LaunchConfiguration("visual_kind").perform(context),
            "size_mode": LaunchConfiguration("size_mode").perform(context),
            "visual_settle_sec": (
                0.0 if LaunchConfiguration("visual_kind").perform(context)
                .strip().lower() == "mesh" else 2.0),
            "yaw_mode": LaunchConfiguration("yaw_mode").perform(context),
            "yaw_range": _csv_floats(
                LaunchConfiguration("yaw_range").perform(context), (0.0, 0.0)),
            "xy_jitter_range": _csv_floats(
                LaunchConfiguration("xy_jitter_range").perform(context),
                (0.0, 0.0)),
            "sequence_ids": _csv_strings(
                LaunchConfiguration("sequence_ids").perform(context)),
        }],
    )

    use_semantic = (
        LaunchConfiguration("use_semantic").perform(context).lower() == "true")

    detector = Node(
        package="luggage_perception",
        executable="luggage_detector_node.py",
        output="screen",
        parameters=[{
            "scene_tf_config": scene_tf_config,
            "use_semantic": use_semantic,
            "use_sim_time": True,
            "depth_topic": "/luggage/preprocessed/camera/depth/points",
            # The semantic chain (preprocessor 4-6 Hz + YOLO + point filter)
            # adds ~0.3-1.0 s of latency on top of the raw path the 1.0 s
            # default was sized for; measured stale ages peaked ~1.9 s.
            "cloud_max_age_sec": 2.5,
            "estimate_retry_count": 4,
            "estimate_retry_period_sec": 0.25,
            # No spawn GT on the real robot; keep sim on the same path so
            # DetectLuggage reports real misses instead of GetCurrentBox.
            "allow_gt_fallback": False,
            "evaluation_compare_gt": False,
            # SuitcaseViewWait kept but unused (0 = skip).
            "suitcase_update_timeout_sec": 0.0,
        }],
    )

    # Semantic chain (Todo 1): preprocessed RGB -> label mask -> cargo cloud.
    # Default off so the accepted raw-depth path stays the daily launch; the
    # eval driver turns it on explicitly (require_backend guards stub runs).
    semantic_config = os.path.join(
        get_package_share_directory("luggage_perception"),
        "config", "semantic_segmenter.yaml")
    require_backend = LaunchConfiguration(
        "semantic_require_backend").perform(context)
    segmenter = Node(
        package="luggage_perception",
        executable="semantic_segmenter_node.py",
        name="semantic_segmenter",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_semantic")),
        parameters=[semantic_config, {
            "use_sim_time": True,
            "require_backend": require_backend,
        }],
    )
    point_filter = Node(
        package="luggage_perception",
        executable="semantic_point_filter_node.py",
        name="semantic_point_filter",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_semantic")),
        parameters=[semantic_config, {"use_sim_time": True}],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--switch-timeout",
            "30",
        ],
        output="screen",
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "elfin_arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--switch-timeout",
            "30",
        ],
        output="screen",
    )
    # Pick/retreat shells (Todo 3). Default off so perception-only
    # debugging is unaffected; the closed-loop eval turns them on.
    poses_path = LaunchConfiguration("robot_poses_config").perform(context)
    motion_chain = [
        Node(
            package="luggage_planning",
            executable="waypoint_generator_node.py",
            name="waypoint_generator",
            output="screen",
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("use_motion")),
        ),
        Node(
            package="luggage_planning",
            executable="motion_planner_node.py",
            name="motion_planner",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "robot_poses_config": poses_path,
            }],
            condition=IfCondition(LaunchConfiguration("use_motion")),
        ),
    ]

    observe_hold = Node(
        package="luggage_gazebo",
        executable="observe_pose_hold.py",
        name="observe_pose_hold",
        output="screen",
        parameters=[{
            "spawn_at_observe": (
                LaunchConfiguration("spawn_at_observe").perform(context).lower()
                == "true"
            ),
            "robot_poses_config": LaunchConfiguration("robot_poses_config").perform(
                context
            ),
            "observe_pose_name": LaunchConfiguration("observe_pose_name").perform(
                context
            ),
        }],
    )

    robot_description = _robot_description_param(scene_tf_config, initial_joint_args)
    move_group = _move_group_node(robot_description)

    # Top-down orthographic RViz view (same viewpoint as the camera) plus
    # RobotModel/TF/point cloud/image panels for calibration checks.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(
            get_package_share_directory("luggage_gazebo"), "rviz", "sim_full.rviz")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return [
        gz_sim,
        clock_bridge,
        camera_bridge,
        depth_republisher,
        preprocessor,
        service_bridge,
        box_spawner,
        scene_viz,
        detector,
        segmenter,
        point_filter,
        *motion_chain,
        *_robot_state_publisher_actions(scene_tf_config, robot_description),
        *_spawn_scene_and_robot(context, initial_joint_args),
        jsb_spawner,
        RegisterEventHandler(
            OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=arm_spawner, on_exit=[observe_hold, move_group])
        ),
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scene_tf_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("luggage_description"),
                        "config",
                        "scene_tf.yaml.example",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "use_moveit",
                default_value="true",
                description="Start move_group after the arm controller (needed for IK / pose targets).",
            ),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "use_motion", default_value="false",
                description="Start waypoint_generator + motion_planner "
                            "(pick/retreat shells; move_group must also be "
                            "on via use_moveit)."),
            DeclareLaunchArgument(
                "use_semantic", default_value="false",
                description="Start the YOLO semantic chain (segmenter + point "
                            "filter) and feed the detector the cargo cloud."),
            DeclareLaunchArgument(
                "semantic_require_backend", default_value="",
                description="If set, segmenter startup fails unless "
                            "stats['backend'] starts with this prefix "
                            "(e.g. bbox_fill). Empty disables the guard."),
            DeclareLaunchArgument(
                "visual_kind", default_value="box",
                description="Pickup visual: box (primitive AABB) or mesh "
                            "(pre-scaled suitcase STL, visual=collision)."),
            DeclareLaunchArgument(
                "size_mode", default_value="catalog",
                description="Pickup size: catalog (small/medium/large) or "
                            "continuous (box visual only)."),
            DeclareLaunchArgument(
                "yaw_mode", default_value="",
                description="Override catalog yaw_mode (discrete/continuous)."),
            DeclareLaunchArgument(
                "yaw_range", default_value="0.0,0.0",
                description="Yaw range used when yaw_mode is continuous."),
            DeclareLaunchArgument(
                "xy_jitter_range", default_value="0.0,0.0",
                description="Pickup XY jitter half-widths in metres."),
            DeclareLaunchArgument(
                "sequence_ids", default_value="",
                description="Comma-separated catalog ids for SpawnNextBox "
                            "(e.g. carryon,standard). Empty = weighted random."),
            DeclareLaunchArgument(
                "spawn_at_observe",
                default_value="true",
                description="Spawn gz_ros2_control at the named observe pose.",
            ),
            DeclareLaunchArgument(
                "observe_pose_name",
                default_value="observe",
                description="Key in robot_poses YAML (observe or pickup_observe).",
            ),
            DeclareLaunchArgument(
                "robot_poses_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("luggage_description"),
                        "config",
                        "robot_poses.yaml.example",
                    ]
                ),
            ),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", _resource_path()),
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", _resource_path()),
            OpaqueFunction(function=_launch_setup),
        ]
    )
