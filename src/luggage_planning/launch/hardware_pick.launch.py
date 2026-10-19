"""实机代码 (site / real-cell). Not part of the Gazebo simulation stack.

Real-cell pick graph: D555 + CPS executor + detect + MoveIt + vacuum.
No Gazebo. Person on e-stop. One CPS client (jazzy_real).

  ros2 launch luggage_planning hardware_pick.launch.py
  # or: deployment_ws/scripts/hardware_pick.sh
  # no GPU: script injects semantic_device:=cpu

Then in another terminal:

  ros2 run luggage_planning hardware_pick_driver.py --detect-only
  ros2 run luggage_planning hardware_pick_driver.py
  ros2 run luggage_planning hardware_pick_driver.py --observe-pose current
      # default: stay at live joints (no sim pickup_observe)
"""

from __future__ import annotations

import os

import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _bool_text(value):
    return str(value).strip().lower() in ("true", "1", "yes", "on")


def _plan_only(context):
    return _bool_text(LaunchConfiguration("plan_only", default="false").perform(context))


def _execution_gate(context, *args, **kwargs):
    """Plan-only launch must not start the CPS hardware owner."""
    if _plan_only(context):
        return [SetLaunchConfiguration("start_executor", "false")]
    return []


_RAW_COLOR = "/camera/d555/color/image_raw"
_RAW_DEPTH = "/camera/d555/aligned_depth_to_color/image_raw"


def _preprocessor_node(context, *args, **kwargs):
    """Subscribe raw images unless use_compressed:=true."""
    if not _bool_text(LaunchConfiguration("start_perception").perform(context)):
        return []
    use_compressed = _bool_text(
        LaunchConfiguration("use_compressed").perform(context))
    perc_share = get_package_share_directory("luggage_perception")
    if use_compressed:
        color = _RAW_COLOR + "/compressed"
        depth = _RAW_DEPTH + "/compressed"
    else:
        color = _RAW_COLOR
        depth = _RAW_DEPTH
    return [
        Node(
            package="luggage_perception",
            executable="sensor_preprocessor_node.py",
            name="sensor_preprocessor",
            output="screen",
            parameters=[
                os.path.join(perc_share, "config", "sensor_preprocessor.yaml"),
                LaunchConfiguration("preprocessor_config"),
                {
                    "use_sim_time": False,
                    "input.use_compressed": use_compressed,
                    "input.color_image": color,
                    "input.depth_image": depth,
                },
            ],
        )
    ]


def _placement_enabled(context):
    return any(_bool_text(LaunchConfiguration(name, default="false").perform(context))
               for name in ("start_placement", "start_placement_prior"))


def _frame_runtime_params(context):
    """ST09 F21 identity/peer parameters for the hardware-plan-only chain.

    The identity is measured at launch resolution (scene bytes, expanded
    URDF, final SRDF, resolved atlas files). Fails closed on a partial
    enablement; off (default) leaves the graph byte-identical.
    """
    if _placement_enabled(context):
        for name in ("frame_runtime", "start_planning", "start_scene", "use_moveit"):
            if not _bool_text(LaunchConfiguration(name, default="false").perform(context)):
                raise RuntimeError("placement planning requires " + name)
    if not _bool_text(LaunchConfiguration("frame_runtime").perform(context)):
        return {}
    off = [name for name in ("start_planning", "start_scene")
           if not _bool_text(LaunchConfiguration(name).perform(context))]
    if off:
        raise RuntimeError(
            "frame_runtime requires the plan/scene chain; disabled here: %s "
            "(no mixed-v2 enablement)" % ", ".join(off))
    import json
    from luggage_description.runtime_graph import (
        HARDWARE_STACK_PEERS, STACK_PEERS, endpoint_registry, identity_endpoints, resolve_peers)
    from luggage_description.runtime_identity import build_identity
    from luggage_planning.atlas_runtime import (
        load_atlas_runtime_for_scene, resolve_atlas_files)
    from luggage_description.xacro_robot_with_scene_base import expand_and_patch
    scene = LaunchConfiguration("scene_tf_config").perform(context)
    driver = str(
        LaunchConfiguration("frame_driver_peer").perform(context)
        or "hardware_pick_driver")
    peers = resolve_peers(driver, stack=STACK_PEERS if _placement_enabled(context)
                          else HARDWARE_STACK_PEERS)
    xacro_path = os.path.join(
        get_package_share_directory("luggage_description"),
        "urdf", "elfin_s20_with_camera.urdf.xacro")
    # Same expansion the robot_description Command performs (hardware
    # plugin defaults; no gz arguments on the real cell).
    urdf_xml, _pose = expand_and_patch(xacro_path, config_path=scene,
                                       xacro_args=[])
    atlas_cfg = str(
        LaunchConfiguration("atlas_runtime_config").perform(context) or "")
    identity = build_identity(
        scene_tf_config=scene,
        calibration_revision=str(
            LaunchConfiguration("frame_calibration_revision").perform(context)
            or "d555_site"),
        urdf_text=urdf_xml,
        srdf_text=_semantic_srdf()["robot_description_semantic"],
        tool_link="suction_contact_frame",
        atlas_files=resolve_atlas_files(
            load_atlas_runtime_for_scene(scene, atlas_cfg or None)),
        endpoints=identity_endpoints(endpoint_registry(peers)))
    return {
        "frame_runtime": True,
        "frame_identity_json": json.dumps(identity, sort_keys=True),
        "frame_peers": ",".join(peers),
        "frame_scene_revision": identity["scene_revision"],
    }


def _frame_nodes(context, *args, **kwargs):
    """Planning-chain nodes with the frame runtime parameters attached."""
    frame = _frame_runtime_params(context)
    return [
        Node(
            package="luggage_planning",
            executable="scene_manager_node.py",
            name="scene_manager",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "scene_tf_config": LaunchConfiguration("scene_tf_config"),
                "world_frame": "world",
                "pickup_object_id": "pickup_box",
                "auto_sync": True,
                **frame,
            }],
            condition=start_planning_of(context),
        ),
        Node(
            package="luggage_planning",
            executable="waypoint_generator_node.py",
            name="waypoint_generator",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "scene_tf_config": LaunchConfiguration("scene_tf_config"),
                "world_frame": "world",
                "place_slot_frame": "elfin_base_link",
                "use_perception_approach": False,
                **frame,
            }],
            condition=start_planning_of(context),
        ),
        Node(
            package="luggage_planning",
            executable="motion_planner_node.py",
            name="motion_planner",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "scene_tf_config": LaunchConfiguration("scene_tf_config"),
                "atlas_runtime_config": LaunchConfiguration(
                    "atlas_runtime_config"),
                "robot_poses_config": LaunchConfiguration("robot_poses_config"),
                "default_observe_pose": LaunchConfiguration("observe_pose_name"),
                "named_pose_duration": 8.0,
                "named_pose_max_vel": 1.0,
                "velocity_scaling": 0.6,
                "acceleration_scaling": 0.6,
                "fjt_action": "/elfin_arm_controller/follow_joint_trajectory",
                **frame,
            }],
            condition=start_planning_of(context),
        ),
        Node(
            package="luggage_planning",
            executable="vacuum_controller_node.py",
            name="vacuum_controller",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "backend": "stub" if _plan_only(context) else "hardware",
                "seal_timeout_sec": 8.0,
                "follow_rate_hz": 10.0,
                **frame,
            }],
            condition=start_planning_of(context),
        ),
    ]


def _placement_prior_nodes(context, *args, **kwargs):
    if not _bool_text(LaunchConfiguration("start_placement_prior").perform(context)):
        return []
    if not _bool_text(LaunchConfiguration("frame_runtime").perform(context)):
        raise RuntimeError("placement prior requires the measured v2 frame runtime")
    frame = _frame_runtime_params(context)
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    launch_path = os.path.join(get_package_share_directory("luggage_planning"),
                               "launch", "placement_prior.launch.py")
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path),
        launch_arguments={
            "scene_tf_config": LaunchConfiguration("scene_tf_config").perform(context),
            "frame_scene_revision": frame["frame_scene_revision"],
        }.items())]


def _placement_service_nodes(context, *args, **kwargs):
    if not _placement_enabled(context):
        return []
    from luggage_planning.hardware_placement_launch import placement_nodes
    return placement_nodes(
        frame=_frame_runtime_params(context),
        scene=LaunchConfiguration("scene_tf_config"),
        atlas=LaunchConfiguration("atlas_runtime_config"),
        poses=LaunchConfiguration("robot_poses_config"))


def _certification_nodes(context, *args, **kwargs):
    if not _bool_text(LaunchConfiguration("start_planning").perform(context)):
        return []
    return [Node(package="luggage_planning", executable="validate_route_node",
                 name="validate_route_v2", output="screen", parameters=[
                     _robot_description(LaunchConfiguration("scene_tf_config")),
                     _semantic_srdf(), {"use_sim_time": False}])]


def start_planning_of(context):
    from launch.conditions import IfCondition
    return IfCondition(LaunchConfiguration("start_planning"))


def _deploy_profile_gate(context, *args, **kwargs):
    """ST09-PD: fail-closed deploy asset resolution at launch time.

    Empty arguments must select the installed deploy scene and
    atlas_runtime_deploy.yaml, never a simulation asset. Missing,
    ambiguous, sim-named, or hash-mismatched assets raise here so the
    hardware chain never starts on the wrong cell identity. The
    atlas/manifest identity checks run with the planning chain; a
    detect-only bringup still resolves the scene and config strictly.
    """
    from luggage_planning.deploy_profile import resolve_deploy_profile

    scene = LaunchConfiguration("scene_tf_config").perform(context)
    atlas_cfg = str(
        LaunchConfiguration("atlas_runtime_config").perform(context) or "")
    record = resolve_deploy_profile(
        scene, atlas_cfg,
        manifest_arg=LaunchConfiguration(
            "deploy_asset_manifest", default="").perform(context),
        check_atlas=_bool_text(
            LaunchConfiguration("start_planning").perform(context)))
    if "atlas_npz" not in record:
        return [
            LogInfo(msg=(
                "[hardware_pick] deploy profile: scene=%s atlas_cfg=%s "
                "(atlas identity not checked: start_planning=false)"
                % (record["scene_tf_path"], record["atlas_runtime_config"]))),
            SetLaunchConfiguration(
                "deploy_scene_resolved", record["scene_tf_path"]),
            SetLaunchConfiguration(
                "deploy_atlas_config_resolved",
                record["atlas_runtime_config"]),
        ]
    return [
        LogInfo(msg=(
            "[hardware_pick] deploy profile: scene=%s atlas_cfg=%s "
            "atlas=%s meta=%s urdf=%s manifest=%s scene_hash=%s urdf_hash=%s"
            % (record["scene_tf_path"], record["atlas_runtime_config"],
               record["atlas_npz"], record["atlas_meta"], record["urdf_path"],
               record.get("manifest_path", "-"), record["scene_tf_hash"],
               record["urdf_hash"]))),
        SetLaunchConfiguration(
            "deploy_scene_resolved", record["scene_tf_path"]),
        SetLaunchConfiguration(
            "deploy_atlas_config_resolved", record["atlas_runtime_config"]),
        SetLaunchConfiguration("deploy_atlas_npz", record["atlas_npz"]),
        SetLaunchConfiguration("deploy_atlas_meta", record["atlas_meta"]),
        SetLaunchConfiguration("deploy_urdf", record["urdf_path"]),
        SetLaunchConfiguration(
            "deploy_manifest", record.get("manifest_path", "")),
    ]


def _semantic_srdf():
    path = os.path.join(
        get_package_share_directory("luggage_description"),
        "config",
        "S20_with_camera.srdf",
    )
    with open(path, "r", encoding="utf-8") as handle:
        xml = handle.read()
    xml = xml.replace('name="elfin_s20_with_camera"', 'name="S20"', 1)
    return {"robot_description_semantic": xml}


def _robot_description(scene_tf_config):
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
    return {
        "robot_description": ParameterValue(
            Command([helper, " ", xacro_path, " ", scene_tf_config]),
            value_type=str,
        )
    }


def _move_group(context, *args, **kwargs):
    if not _bool_text(LaunchConfiguration("start_planning").perform(context)):
        return []
    if not _bool_text(LaunchConfiguration("use_moveit").perform(context)):
        return []
    try:
        from moveit_configs_utils import MoveItConfigsBuilder
    except ImportError:
        return [
            LogInfo(msg=(
                "[hardware_pick] MoveIt is not installed. Skipping move_group. "
                "Detect-only still works. For plan/pick: sudo apt install "
                "ros-jazzy-moveit-msgs ros-jazzy-moveit-configs-utils "
                "ros-jazzy-moveit-ros-move-group ros-jazzy-moveit-planners-ompl "
                "ros-jazzy-moveit-kinematics ros-jazzy-moveit-simple-controller-manager"
            ))
        ]
    scene = LaunchConfiguration("scene_tf_config").perform(context)
    robot_description = _robot_description(scene)
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
    params = dict(moveit_config.to_dict())
    # Humble sim yaml uses scalar plugin keys nested under ompl.
    # Jazzy move_group rejects those; overlay the site string-array file.
    if os.environ.get("ROS_DISTRO", "") == "jazzy":
        jazzy_ompl = os.path.join(
            get_package_share_directory("elfin_moveit_config"),
            "config",
            "ompl_planning.jazzy.yaml",
        )
        with open(jazzy_ompl, "r", encoding="utf-8") as handle:
            overlay = yaml.safe_load(handle) or {}
        params.pop("planning_plugin", None)
        params.pop("request_adapters", None)
        ompl = dict(params.get("ompl") or {})
        ompl.pop("planning_plugin", None)
        ompl.pop("request_adapters", None)
        ompl.update(overlay)
        params["ompl"] = ompl
    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[
                params,
                _semantic_srdf(),
                {
                    "publish_robot_description_semantic": True,
                    "allow_trajectory_execution": not _plan_only(context),
                    "use_sim_time": False,
                },
            ],
        )
    ]


def _launch_description():
    desc_share = get_package_share_directory("luggage_description")
    perc_share = get_package_share_directory("luggage_perception")
    exec_share = get_package_share_directory("elfin_trajectory_executor")
    planning_share = get_package_share_directory("luggage_planning")
    # ST09-PD: empty arguments select the installed deploy assets. Never
    # fall back to scene_tf.yaml[.example] -- that silent sim reuse is
    # exactly what the deploy profile gate rejects.
    default_scene = os.path.join(desc_share, "config", "scene_tf_deploy.yaml")
    default_atlas_cfg = os.path.join(
        planning_share, "config", "atlas_runtime_deploy.yaml")
    poses = os.path.join(desc_share, "config", "robot_poses.deploy.yaml")
    sim_poses = os.path.join(desc_share, "config", "robot_poses.yaml.example")
    sim_scene = os.path.join(desc_share, "config", "scene_tf.yaml.example")
    live_pp = os.path.join(perc_share, "config", "preprocessor_d555_live.yaml")
    semantic = os.path.join(perc_share, "config", "semantic_segmenter.yaml")
    site_pp = os.path.join(perc_share, "config", "preprocessor_d555_site.yaml")
    start_perception = IfCondition(LaunchConfiguration("start_perception"))
    start_planning = IfCondition(LaunchConfiguration("start_planning"))
    start_d555 = IfCondition(LaunchConfiguration("start_d555"))
    start_executor = IfCondition(LaunchConfiguration("start_executor"))
    start_scene = IfCondition(LaunchConfiguration("start_scene"))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scene_tf_config",
                default_value=default_scene,
                description=(
                    "Deploy scene file. The default is the installed "
                    "scene_tf_deploy.yaml; %s and scene_tf.yaml are "
                    "simulation assets and are rejected on this cell."
                    % os.path.basename(sim_scene)
                ),
            ),
            DeclareLaunchArgument(
                "atlas_runtime_config",
                default_value=default_atlas_cfg,
                description=(
                    "Reachability-atlas runtime YAML. Default is the "
                    "installed atlas_runtime_deploy.yaml (atlas identity "
                    "strict); atlas_runtime.yaml is the simulation cell "
                    "and is rejected on this cell."
                ),
            ),
            DeclareLaunchArgument(
                "robot_poses_config",
                default_value=poses,
                description=(
                    "Site pose file for GoToRobotPose. Default is "
                    "robot_poses.deploy.yaml (all entries valid:false, "
                    "observe_pose=current until the cell is measured). "
                    "Do not pass %s on this cell; those pickup_observe "
                    "joints are simulation." % sim_poses
                ),
            ),
            DeclareLaunchArgument(
                "observe_pose_name",
                default_value="current",
                description=(
                    "Default GoToRobotPose name when the goal is empty. "
                    "current/here = stay. pickup_observe requires measured "
                    "values in robot_poses_config."
                ),
            ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("use_moveit", default_value="true"),
            DeclareLaunchArgument(
                "start_perception",
                default_value="true",
                description=(
                    "Start the perception role: preprocessor, semantic "
                    "segmenter, semantic point filter, and luggage detector."
                ),
            ),
            DeclareLaunchArgument(
                "start_planning",
                default_value="true",
                description=(
                    "Start the planning role: scene manager, waypoint "
                    "generator, motion planner, vacuum controller, and "
                    "MoveIt when use_moveit is true."
                ),
            ),
            DeclareLaunchArgument("start_d555", default_value="true"),
            DeclareLaunchArgument("start_executor", default_value="true"),
            DeclareLaunchArgument(
                "deploy_asset_manifest", default_value="",
                description="Explicit frozen deployment manifest; all file hashes remain strictly checked."),
            DeclareLaunchArgument(
                "plan_only", default_value="false",
                description="Disable CPS startup and MoveIt execution; use a vacuum stub for offline interface checks."),
            DeclareLaunchArgument("start_placement_prior", default_value="false"),
            DeclareLaunchArgument("start_placement", default_value="false",
                description="Start typed OCC, placement and Select providers; also enabled by start_placement_prior."),
            DeclareLaunchArgument(
                "start_scene",
                default_value="true",
                description=(
                    "scene_hardware (RSP + container TF). Set false when "
                    "record_site already publishes the same tree."
                ),
            ),
            DeclareLaunchArgument(
                "preprocessor_config",
                default_value=site_pp,
                description=(
                    "实机 preprocessor. Default B is "
                    "preprocessor_d555_site.yaml (motion_gate off). "
                    "Profile A (motion_gate on) is preprocessor_config:=%s. "
                    "Do not use preprocessor_d555_replay.yaml for live pick "
                    "(use_sim_time true)." % live_pp
                ),
            ),
            DeclareLaunchArgument(
                "use_compressed",
                default_value="false",
                description=(
                    "false: preprocessor subscribes mapped raw "
                    "/camera/d555/.../image_raw. true: JPEG/PNG "
                    "image_raw/compressed (old bags and recorders only)."
                ),
            ),
            DeclareLaunchArgument(
                "publish_overlay",
                default_value="true",
                description=(
                    "Publish /luggage/semantic/overlay. semantic_segmenter.yaml "
                    "sets this false for sim-eval CPU hygiene; site detect "
                    "needs the image. Pass publish_overlay:=false to drop it."
                ),
            ),
            DeclareLaunchArgument("semantic_device", default_value="cuda"),
            DeclareLaunchArgument(
                "max_rate_hz",
                default_value="0.0",
                description=(
                    "YOLO cap. 0 follows the camera (~15 Hz). "
                    "A positive value drops frames closer than 1/max_rate_hz."
                ),
            ),
            DeclareLaunchArgument(
                "execution_backend",
                default_value="waypoint",
                description=(
                    "Real executor backend. Production pick stays waypoint; "
                    "servo_j is opt-in for hardware gates only."
                ),
            ),
            DeclareLaunchArgument(
                "servo_j_servo_time",
                default_value="0.02",
                description="ServoJ fixed-grid period (s). Used only when backend=servo_j.",
            ),
            DeclareLaunchArgument(
                "servo_j_lookahead_time",
                default_value="0.1",
                description="ServoJ lookahead (s). Used only when backend=servo_j.",
            ),
            DeclareLaunchArgument("robot_ip", default_value="192.168.0.10"),
            DeclareLaunchArgument("robot_port", default_value="10003"),
            DeclareLaunchArgument(
                "color_profile",
                default_value="640,360,15",
                description="D555 RGB profile. Do not use 896x504@30.",
            ),
            DeclareLaunchArgument(
                "depth_profile",
                default_value="640,360,15",
                description="D555 depth profile. Match colour.",
            ),
            LogInfo(
                msg=(
                    "[hardware_pick] No Gazebo. e-stop person required. "
                    "CPS via jazzy_real only. D555 colour+aligned depth. "
                    "Ctrl+C does not BlackOut. Observe pose default=current "
                    "(no sim pickup_observe swing)."
                )
            ),
            # ST09-PD: resolve the deploy profile before any node starts;
            # a missing/ambiguous/sim-named/hash-mismatched asset fails
            # the whole launch instead of starting on the wrong cell.
            OpaqueFunction(function=_deploy_profile_gate),
            OpaqueFunction(function=_execution_gate),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(exec_share, "launch", "jazzy_real.launch.py")
                ),
                launch_arguments={
                    "robot_ip": LaunchConfiguration("robot_ip"),
                    "robot_port": LaunchConfiguration("robot_port"),
                    # Huayan WayPoint min ~1 deg/s. 10/20 made MoveIt TOTG
                    # crawl and still hit 20070; site Gate 5 uses 30/60.
                    "default_velocity_deg": "30.0",
                    "max_velocity_deg": "60.0",
                    "command_acceleration_deg": "60.0",
                    "execution_backend": LaunchConfiguration("execution_backend"),
                    "servo_j_servo_time": LaunchConfiguration("servo_j_servo_time"),
                    "servo_j_lookahead_time": LaunchConfiguration(
                        "servo_j_lookahead_time"
                    ),
                }.items(),
                condition=start_executor,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(desc_share, "launch", "scene_hardware.launch.py")
                ),
                launch_arguments={
                    "scene_tf_config": LaunchConfiguration("scene_tf_config"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "use_sim_time": "false",
                }.items(),
                condition=start_scene,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(exec_share, "launch", "d555_rgbd.launch.py")
                ),
                launch_arguments={
                    "color_profile": LaunchConfiguration("color_profile"),
                    "depth_profile": LaunchConfiguration("depth_profile"),
                }.items(),
                condition=start_d555,
            ),
            OpaqueFunction(function=_preprocessor_node),
            Node(
                package="luggage_perception",
                executable="semantic_segmenter_node.py",
                name="semantic_segmenter",
                output="screen",
                parameters=[
                    semantic,
                    {
                        "use_sim_time": False,
                        "device": LaunchConfiguration("semantic_device"),
                        "self_body_camera_frame": "d555_color_optical_frame",
                        "require_backend": "yolo_world",
                        # yaml default true is a sim FP gate against the
                        # scene_tf pickup square. Site crop is YOLO bbox
                        # → depth, not that square.
                        "workspace_accept_enabled": False,
                        # After yaml: yaml publish_overlay is false (eval).
                        "publish_overlay": ParameterValue(
                            LaunchConfiguration("publish_overlay"),
                            value_type=bool,
                        ),
                        # 0 follows the 15 Hz camera. A 2 Hz cap drops frames
                        # even when inference is ~64 ms.
                        "max_rate_hz": ParameterValue(
                            LaunchConfiguration("max_rate_hz"),
                            value_type=float,
                        ),
                    },
                ],
                condition=start_perception,
            ),
            Node(
                package="luggage_perception",
                executable="semantic_point_filter_node.py",
                name="semantic_point_filter",
                output="screen",
                parameters=[semantic, {"use_sim_time": False}],
                condition=start_perception,
            ),
            Node(
                package="luggage_perception",
                executable="luggage_detector_node.py",
                name="luggage_detector",
                output="screen",
                parameters=[{
                    "scene_tf_config": LaunchConfiguration("scene_tf_config"),
                    "use_semantic": True,
                    "use_sim_time": False,
                    "depth_topic": "/luggage/preprocessed/camera/depth/image",
                    # CPU YOLO + D555 host-stamp lag: 8 s rejected still
                    # frames (DETECT_STALE_CLOUD). Site pick waits longer.
                    "cloud_max_age_sec": 30.0,
                    "estimate_retry_count": 6,
                    "estimate_retry_period_sec": 2.0,
                    "support_mode": "auto",
                    "platform_z": "",
                    "suitcase_update_timeout_sec": 0.0,
                    "crop_to_workspace": False,
                    # DYNAMIC-SUCTION ST-1: hardware top surface comes from
                    # the YOLO instance ROI + aligned-depth component; the
                    # cargo-cloud RANSAC/PCA legacy path stays the default
                    # everywhere else. scene_tf remains TF/collision/safety
                    # authority only.
                    "top_surface_mode": "dynamic",
                }],
                condition=start_perception,
            ),
            DeclareLaunchArgument(
                "frame_runtime",
                default_value="false",
                description=(
                    "ST09: typed v2 runtime chain on the plan/scene nodes "
                    "(identity heartbeats + readiness gating)."
                ),
            ),
            DeclareLaunchArgument(
                "frame_calibration_revision",
                default_value="d555_site",
                description="Calibration identity label for the runtime identity.",
            ),
            DeclareLaunchArgument(
                "frame_driver_peer",
                default_value="hardware_pick_driver",
                description="Driver node name joining the v2 graph as client.",
            ),
            OpaqueFunction(function=_frame_nodes),
            OpaqueFunction(function=_placement_service_nodes),
            OpaqueFunction(function=_certification_nodes),
            OpaqueFunction(function=_placement_prior_nodes),
            OpaqueFunction(function=_move_group),
        ]
    )


def generate_launch_description():
    from luggage_description.launch_profile import with_profile
    return with_profile(_launch_description())
