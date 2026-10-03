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
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
_PROFILE_SENTINEL = "__profile__"

# Same branches as luggage_description/config/robot_poses.yaml.example.
# Used only if that YAML is missing; gz_ros2_control must not silently spawn
# on a different wrap than MoveIt/controller.
_FALLBACK_OBSERVE = [3.5702, -1.3263, -1.0965, 3.9564, 1.6234, 0.4522]


def _default_launch_values():
    return {
        "scene_tf_config": os.path.join(
            get_package_share_directory("luggage_description"),
            "config",
            "scene_tf.yaml.example",
        ),
        "use_moveit": "true",
        "gui": "true",
        "use_rviz": "true",
        "use_perception": "true",
        "hull_margin": "0.0",
        "min_support_ratio": "0.6",
        "last_result_max_candidates": "24",
        "use_cargo_map": "false",
        "use_packing": "false",
        "use_vacuum": "false",
        "use_motion": "false",
        # Reachability-atlas runtime wiring (placement gate + motion IK
        # seeds). Empty = the nodes pick the atlas whose stored scene hash
        # matches scene_tf_config. atlas_runtime_config overrides that.
        # atlas_enabled forces the whole chain on/off.
        "atlas_runtime_config": "",
        "atlas_enabled": "",
        "named_pose_duration": "4.0",
        "named_pose_max_vel": "1.0",
        "velocity_scaling": "0.3",
        "acceleration_scaling": "0.3",
        "use_semantic": "false",
        "semantic_require_backend": "",
        "semantic_clip_vendor_dir": "",
        "semantic_model_name": "",
        "visual_kind": "mesh",
        "size_mode": "catalog",
        "yaw_mode": "",
        "yaw_range": "0.0,0.0",
        "xy_jitter_range": "0.0,0.0",
        "sequence_ids": "",
        "random_seed": "-1",
        "spawn_at_observe": "true",
        "observe_pose_name": "observe",
        # Seconds to wait for /controller_manager after the S20 create before
        # declaring the gz_ros2_control URDF fetch lost and shutting down.
        # Successful startups build it in about 2-4 s; 0 disables the gate.
        "startup_watchdog_sec": "45.0",
        "robot_poses_config": os.path.join(
            get_package_share_directory("luggage_description"),
            "config",
            "robot_poses.yaml.example",
        ),
        # ST09 F21: the typed v2 runtime chain. On = every peer node
        # publishes its measured RuntimeIdentityV2 and gates effects on
        # graph readiness; requires the full chain (no mixed enablement).
        "frame_runtime": "false",
        "frame_calibration_revision": "urdf-embedded-d435",
        "frame_driver_peer": "motion_occ_driver",
    }


def _resolved_launch_config(context):
    from luggage_description.runtime_profile import load_profile, resolve_profile, launch_text
    defaults = _default_launch_values()
    profile_path = LaunchConfiguration("profile_config").perform(context)
    overrides = {name: LaunchConfiguration(name).perform(context) for name in defaults}
    resolved = resolve_profile(defaults, load_profile(profile_path), overrides)
    return {name: launch_text(value) for name, value in resolved.items()}


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _semantic_asset_paths(cfg):
    """[ST09-I S5.7-6] Semantic assets resolve from the INSTALL overlay.

    Formal runs must not depend on the main workspace, the source tree,
    or the user cache: the CLIP vendor and the YOLO-World checkpoint
    are colcon-installed under install-st09/<pkg>/share and resolved
    through the package share (hashes recorded in the acceptance
    manifest). Explicit launch flags override; empty falls back to the
    share location — NEVER to a bare filename, whose ultralytics
    not-found fallback is a network download.
    """
    share = get_package_share_directory("luggage_perception")
    vendor = str(cfg.get("semantic_clip_vendor_dir") or "").strip() \
        or os.path.join(share, "vendor")
    model = str(cfg.get("semantic_model_name") or "").strip() \
        or os.path.join(share, "models", "yolov8s-world.pt")
    return vendor, model


def _require_semantic_assets(vendor, model):
    """Fail-closed named launch errors for absent semantic assets."""
    if not os.path.isdir(os.path.join(vendor, "clip_pkg")):
        raise RuntimeError(
            "SEMANTIC_VENDOR_MISSING: CLIP vendor incomplete at %s "
            "(need clip_pkg/; expected the colcon-installed package "
            "share copy)" % vendor)
    if not os.path.isfile(model):
        raise RuntimeError(
            "SEMANTIC_WEIGHTS_MISSING: YOLO-World checkpoint absent at "
            "%s (expected the colcon-installed package share copy; a "
            "bare filename would fall back to a network download)" % model)


def _bool_text(value):
    return "true" if _as_bool(value) else "false"


def _csv_floats(text, default):
    if isinstance(text, (list, tuple)):
        parts = list(text)
    else:
        parts = [p.strip() for p in str(text or "").split(",") if p.strip()]
    if len(parts) < 2:
        return [float(default[0]), float(default[1])]
    return [float(parts[0]), float(parts[1])]


def _csv_strings(text):
    if isinstance(text, (list, tuple)):
        parts = [str(p).strip() for p in text if str(p).strip()]
    else:
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


def _initial_joint_xacro_args(cfg):
    """xacro mappings so gz_ros2_control starts at the named pose (or mock defaults)."""
    spawn = _as_bool(cfg["spawn_at_observe"])
    if not spawn:
        return []
    poses_path = cfg["robot_poses_config"]
    pose_name = cfg["observe_pose_name"]
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


def _robot_state_publisher_actions(scene_tf_config: str, robot_description,
                                   frame_scene_revision=""):
    return [
        Node(
            package="luggage_description",
            executable="container_tf_publisher",
            name="container_tf_publisher",
            output="screen",
            parameters=[{"scene_tf_config": scene_tf_config,
                         "republish_period": 0.0,
                         "frame_scene_revision": frame_scene_revision}],
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


def _move_group_node(robot_description, use_moveit):
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
        condition=IfCondition(_bool_text(use_moveit)),
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


def _robot_create_node(scene_tf_config, initial_joint_args):
    """Welded gz URDF: world_base carries the scene pose; spawn at the origin."""
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


#: Logged immediately before the S20 create action. Startup gates must take
#: this marker as the authoritative stage boundary rather than running their
#: own parameter probe on a separate clock, which is how PF-R7 generation 9
#: expired a 2 s spawn deadline against a probe it did not own.
SPAWN_ROBOT_MARKER = "spawn_robot: create S20"


def _controller_manager_watchdog(timeout_sec):
    """Fail fast when gz_ros2_control never builds the controller manager.

    The plugin fetches the URDF with one non-retrying parameter call (see
    scripts/wait_controller_manager.py). When that call is lost the plugin
    blocks forever and the launch stays superficially alive while both
    spawners burn their 60 s timeouts. Converting that into a named shutdown
    within `timeout_sec` is what lets a caller simply relaunch.
    """
    return ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(
                get_package_prefix("luggage_gazebo"),
                "lib", "luggage_gazebo", "wait_controller_manager.py",
            ),
            "--timeout-sec", str(timeout_sec),
        ],
        name="wait_controller_manager",
        output="screen",
    )


def _spawn_scene_and_robot(scene_tf_config, initial_joint_args,
                           watchdog_sec=0.0):
    """Spawn pedestal/platform/container first, then the welded arm (Noetic order)."""
    scene_nodes = _scene_model_actions(scene_tf_config)
    robot = _robot_create_node(scene_tf_config, initial_joint_args)
    spawn_actions = [LogInfo(msg=SPAWN_ROBOT_MARKER), robot]

    watchdog = []
    if watchdog_sec > 0:
        cm_watchdog = _controller_manager_watchdog(watchdog_sec)
        watchdog = [
            RegisterEventHandler(
                OnProcessExit(target_action=robot, on_exit=[cm_watchdog])
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=cm_watchdog,
                    on_exit=_shutdown_unless_clean_exit,
                )
            ),
        ]

    if not scene_nodes:
        return spawn_actions + watchdog
    return scene_nodes + [
        RegisterEventHandler(
            OnProcessExit(target_action=scene_nodes[0], on_exit=spawn_actions)
        )
    ] + watchdog


def _shutdown_unless_clean_exit(event, context):
    """Shut the launch down when the watchdog reports a startup failure."""
    del context
    if getattr(event, "returncode", 0) == 0:
        return []
    return [
        LogInfo(msg="sim_world startup aborted: controller manager absent"),
        EmitEvent(event=Shutdown(reason="plugin_urdf_not_received")),
    ]


def _frame_runtime_params(cfg, scene_tf_config, initial_joint_args):
    """Identity + peer parameters for the v2 runtime chain, or ``{}``.

    The identity is measured at launch resolution time (scene bytes,
    expanded URDF, final SRDF, resolved atlas files) — never declared.
    Fails closed on a partial enablement: the v2 chain is all or nothing.
    """
    if not _as_bool(cfg["frame_runtime"]):
        return {}
    off = [name for name in ("use_motion", "use_cargo_map", "use_packing",
                             "use_vacuum") if not _as_bool(cfg[name])]
    if off:
        raise RuntimeError(
            "frame_runtime requires the full v2 chain; disabled here: %s "
            "(no mixed-v2 enablement)" % ", ".join(off))
    import json
    from luggage_description.runtime_graph import (
        endpoint_registry, identity_endpoints, resolve_peers)
    from luggage_description.runtime_identity import build_identity
    from luggage_planning.atlas_runtime import (
        load_atlas_runtime_for_scene, resolve_atlas_files)
    peers = resolve_peers(cfg["frame_driver_peer"])
    xacro_path = os.path.join(
        get_package_share_directory("luggage_description"),
        "urdf", "elfin_s20_with_camera.urdf.xacro")
    # Same expansion the robot_description Command performs for RSP and
    # move_group (scene pose on the joint, gz hardware plugin, named pose).
    urdf_xml, _pose = expand_and_patch(
        xacro_path, config_path=scene_tf_config,
        xacro_args=[
            "use_gz_sim:=true",
            "hardware_plugin:=gz_ros2_control/GazeboSimSystem",
        ] + list(initial_joint_args))
    identity = build_identity(
        scene_tf_config=scene_tf_config,
        calibration_revision=cfg["frame_calibration_revision"],
        urdf_text=urdf_xml,
        srdf_text=_gazebo_semantic_description()["robot_description_semantic"],
        tool_link="suction_contact_frame",
        atlas_files=resolve_atlas_files(load_atlas_runtime_for_scene(
            scene_tf_config, str(cfg["atlas_runtime_config"] or "") or None)),
        endpoints=identity_endpoints(endpoint_registry(peers)))
    return {
        "frame_runtime": True,
        "frame_identity_json": json.dumps(identity, sort_keys=True),
        "frame_peers": ",".join(peers),
        "frame_scene_revision": identity["scene_revision"],
    }


def _launch_setup(context):
    _check_renderer()
    cfg = _resolved_launch_config(context)
    scene_tf_config = cfg["scene_tf_config"]
    gui = _as_bool(cfg["gui"])
    initial_joint_args = _initial_joint_xacro_args(cfg)
    frame_params = _frame_runtime_params(cfg, scene_tf_config,
                                         initial_joint_args)

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
    use_perception_cond = IfCondition(_bool_text(cfg["use_perception"]))

    # D435 + Mid-360S -> ROS names the rest of the stack consumes.
    # Keep both in this one process: eval graph_error treats >3
    # parameter_bridge PIDs as a leftover dual-sim.
    # gpu_lidar publishes LaserScan on /livox/scan and the cloud on
    # /livox/scan/points; remap the cloud to the real-driver topic
    # /livox/lidar. Raster scan, no per-point times.
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        output="screen",
        arguments=[
            "/d435/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/d435/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/d435/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/livox/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        remappings=[
            ("/d435/image", "/camera/color/image_raw"),
            ("/d435/depth_image", "/camera/depth/image_meters"),
            ("/d435/camera_info", "/camera/depth/camera_info"),
            ("/livox/scan/points", "/livox/lidar"),
        ],
        parameters=[{"use_sim_time": True}],
        condition=use_perception_cond,
    )

    depth_republisher = Node(
        package="luggage_gazebo",
        executable="depth_image_republisher.py",
        name="depth_image_republisher",
        output="screen",
        parameters=[{"use_sim_time": True, "near_depth_m": 0.26}],
        condition=use_perception_cond,
    )
    camera_info_alias = Node(
        package="luggage_gazebo",
        executable="camera_info_alias.py",
        name="camera_info_alias",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=use_perception_cond,
    )

    # PF-R10: per-thread glibc arenas ratcheted transient geometry
    # allocations into unbounded RSS (measured +359 MiB/43 s on the
    # detector during cargo processing; offline 16-thread repro 76.6 vs
    # 50.1 MiB with the cap). One arena pair per node bounds it. The
    _arena_env = {"MALLOC_ARENA_MAX": "2"}
    # Filter/detector variant: their per-frame geometry buffers are large
    # and transient, so pinning glibc's mmap threshold (which otherwise
    # ratchets toward the size of every freed block, up to 32 MiB) routes
    # them through mmap/munmap and returns pages on free. The segmenter
    # must NOT get this: its CLIP buffers would churn through page
    # faults per frame (measured no_top growth, reverted 2026-09-09).
    _arena_env_mmap = {
        "MALLOC_ARENA_MAX": "2",
        "MALLOC_MMAP_THRESHOLD_": "131072",
        "MALLOC_TRIM_THRESHOLD_": "262144",
    }

    preprocessor = Node(
        package="luggage_perception",
        executable="sensor_preprocessor_node.py",
        name="sensor_preprocessor",
        output="screen",
        condition=use_perception_cond,
        additional_env=_arena_env,
        parameters=[
            os.path.join(
                get_package_share_directory("luggage_perception"),
                "config", "sensor_preprocessor.yaml",
            ),
            {"use_sim_time": True},
            # F02 observation admission rides the same scene revision the
            # runtime chain measured (no-op when frame_runtime is off).
            {"frame_scene_revision": frame_params.get("frame_scene_revision",
                                                      "")},
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
            "visual_kind": cfg["visual_kind"],
            "size_mode": cfg["size_mode"],
            "visual_settle_sec": 2.0,
            "yaw_mode": cfg["yaw_mode"],
            "yaw_range": _csv_floats(cfg["yaw_range"], (0.0, 0.0)),
            "xy_jitter_range": _csv_floats(
                cfg["xy_jitter_range"], (0.0, 0.0)),
            "sequence_ids": _csv_strings(cfg["sequence_ids"]),
            "random_seed": int(float(cfg["random_seed"])),
        }],
    )

    use_semantic = _as_bool(cfg["use_semantic"])

    detector = Node(
        package="luggage_perception",
        executable="luggage_detector_node.py",
        output="screen",
        # Place-only / planning-only stacks start with use_perception:=false
        # so acceptance can assert perception nodes are absent.
        condition=IfCondition(_bool_text(cfg["use_perception"])),
        # OPENBLAS 1: the support transform is a small-matrix matmul; the
        # full-core BLAS spin both burns CPU and fans allocations across
        # the OpenBLAS pool's threads.
        additional_env={**_arena_env_mmap, "OPENBLAS_NUM_THREADS": "1"},
        parameters=[{
            "scene_tf_config": scene_tf_config,
            "use_semantic": use_semantic,
            "use_sim_time": True,
            "depth_topic": "/luggage/preprocessed/camera/depth/image",
            # The semantic chain (preprocessor 4-6 Hz + YOLO + point filter)
            # adds ~0.3-1.0 s of latency on top of the raw path the 1.0 s
            # default was sized for; measured stale ages peaked ~1.9 s.
            "cloud_max_age_sec": 2.5,
            "estimate_retry_count": 4,
            "estimate_retry_period_sec": 0.25,
            # Platform-free height (E3): measured support only; no
            # configured platform Z (omitted is valid configuration).
            "support_mode": "auto",
            "platform_z": "",
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
    require_backend = cfg["semantic_require_backend"]
    # [ST09-I S5.7-6] assets come from the INSTALL overlay via the
    # package share (colcon-installed vendor + checkpoint; hash-recorded
    # in the acceptance manifest). No main-workspace path, no source
    # tree, no user cache, no bare filename (whose ultralytics fallback
    # is a network download) — an absent asset is a NAMED launch error.
    clip_vendor_dir, semantic_model_path = _semantic_asset_paths(cfg)
    if use_semantic:
        _require_semantic_assets(clip_vendor_dir, semantic_model_path)
    segmenter_env = dict(_arena_env)
    segmenter_env["YOLO_AUTOUPDATE"] = "false"
    segmenter_env["LUGGAGE_CLIP_VENDOR_DIR"] = clip_vendor_dir
    segmenter = Node(
        package="luggage_perception",
        executable="semantic_segmenter_node.py",
        name="semantic_segmenter",
        output="screen",
        additional_env=segmenter_env,
        condition=IfCondition(_bool_text(cfg["use_semantic"])),
        parameters=[semantic_config, {
            "use_sim_time": True,
            "require_backend": require_backend,
            "model_name": semantic_model_path,
        }],
    )
    point_filter = Node(
        package="luggage_perception",
        executable="semantic_point_filter_node.py",
        name="semantic_point_filter",
        output="screen",
        additional_env={**_arena_env_mmap, "OPENBLAS_NUM_THREADS": "1"},
        condition=IfCondition(_bool_text(cfg["use_semantic"])),
        parameters=[semantic_config, {
            "use_sim_time": True,
            "publish_untracked_cargo": True,
            "untracked_include_obstacle": True,
        }],
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
    poses_path = cfg["robot_poses_config"]
    # Atlas overrides only when explicitly set; empty keeps the nodes on
    # the installed atlas_runtime.yaml defaults.
    atlas_params = {}
    if cfg["atlas_runtime_config"]:
        atlas_params["atlas_runtime_config"] = cfg["atlas_runtime_config"]
    if cfg["atlas_enabled"]:
        atlas_params["atlas_enabled"] = _as_bool(cfg["atlas_enabled"])

    motion_chain = [
        Node(
            package="luggage_planning",
            executable="scene_manager_node.py",
            name="scene_manager",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "scene_tf_config": scene_tf_config,
                "world_frame": "world",
                "pickup_object_id": "pickup_box",
                **frame_params,
            }],
            condition=IfCondition(_bool_text(cfg["use_motion"])),
        ),
        Node(
            package="luggage_planning",
            executable="waypoint_generator_node.py",
            name="waypoint_generator",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "scene_tf_config": scene_tf_config,
                "world_frame": "world",
                "place_slot_frame": "elfin_base_link",
                **frame_params,
            }],
            condition=IfCondition(_bool_text(cfg["use_motion"])),
        ),
        Node(
            package="luggage_planning",
            executable="motion_planner_node.py",
            name="motion_planner",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "scene_tf_config": scene_tf_config,
                "robot_poses_config": poses_path,
                "named_pose_duration": float(cfg["named_pose_duration"]),
                "named_pose_max_vel": float(cfg["named_pose_max_vel"]),
                "velocity_scaling": float(cfg["velocity_scaling"]),
                "acceleration_scaling": float(cfg["acceleration_scaling"]),
                **frame_params,
            }, atlas_params],
            condition=IfCondition(_bool_text(cfg["use_motion"])),
        ),
        # SelectPlacePath provider (production path). The motion_occ driver
        # still selects in-process; this node runs alongside so the service
        # surface boots with the stack and stays covered by the smokes.
        Node(
            package="luggage_planning",
            executable="trajectory_selector_node.py",
            name="trajectory_selector",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "scene_tf_config": scene_tf_config,
                **frame_params,
            }, atlas_params],
            condition=IfCondition(_bool_text(cfg["use_motion"])),
        ),
    ]

    observe_hold = Node(
        package="luggage_gazebo",
        executable="observe_pose_hold.py",
        name="observe_pose_hold",
        output="screen",
        parameters=[{
            "spawn_at_observe": (
                _as_bool(cfg["spawn_at_observe"])
            ),
            "robot_poses_config": cfg["robot_poses_config"],
            "observe_pose_name": cfg["observe_pose_name"],
        }],
    )

    robot_description = _robot_description_param(scene_tf_config, initial_joint_args)
    move_group = _move_group_node(robot_description, cfg["use_moveit"])

    # Top-down orthographic RViz view (same viewpoint as the camera) plus
    # RobotModel/TF/point cloud/image panels for calibration checks.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(
            get_package_share_directory("luggage_gazebo"), "rviz", "sim_full.rviz")],
        condition=IfCondition(_bool_text(cfg["use_rviz"])),
    )

    return [
        gz_sim,
        clock_bridge,
        camera_bridge,
        depth_republisher,
        camera_info_alias,
        preprocessor,
        service_bridge,
        box_spawner,
        scene_viz,
        detector,
        segmenter,
        point_filter,
        *motion_chain,
        Node(
            package="luggage_perception",
            executable="cargo_volume_mapper_node.py",
            name="cargo_volume_mapper",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "cargo_cloud_topic": "/luggage/semantic/cargo_points_untracked",
                "base_frame": "container_link",
                "cloud_buffer_size": 12,
                "freshness_window_sec": 0.20,
                **frame_params,
            }],
            condition=IfCondition(_bool_text(cfg["use_cargo_map"])),
        ),
        Node(
            package="luggage_packing",
            executable="placement_planner_node.py",
            name="placement_planner",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "scene_tf_config": scene_tf_config,
                # 0 keeps the historical flush-wall hull gate; the
                # place-only profile configures 0.01 (>=10 mm clearance).
                "hull_margin": float(cfg["hull_margin"]),
                "min_support_ratio": float(cfg["min_support_ratio"]),
                "last_result_max_candidates": int(
                    float(cfg["last_result_max_candidates"])),
                **frame_params,
            }, atlas_params],
            condition=IfCondition(_bool_text(cfg["use_packing"])),
        ),
        Node(
            package="luggage_planning",
            executable="vacuum_controller_node.py",
            name="vacuum_controller",
            output="screen",
            parameters=[{"use_sim_time": True, **frame_params}],
            condition=IfCondition(_bool_text(cfg["use_vacuum"])),
        ),
        *_robot_state_publisher_actions(
            scene_tf_config, robot_description,
            frame_scene_revision=frame_params.get("frame_scene_revision",
                                                  "")),
        *_spawn_scene_and_robot(
            scene_tf_config, initial_joint_args,
            watchdog_sec=float(cfg["startup_watchdog_sec"]),
        ),
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
    def profile_arg(name, description=""):
        return DeclareLaunchArgument(
            name,
            default_value=_PROFILE_SENTINEL,
            description=(
                (description + " ") if description else ""
            ) + "Default is taken from profile_config, then built-in fallback.",
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile_config",
                default_value="",
                description="Optional grouped YAML profile. CLI launch args override profile values.",
            ),
            profile_arg(
                "scene_tf_config",
                "Scene TF YAML path.",
            ),
            profile_arg(
                "use_moveit",
                "Start move_group after the arm controller (needed for IK / pose targets).",
            ),
            profile_arg("gui", "Start Gazebo GUI."),
            profile_arg(
                "startup_watchdog_sec",
                "Seconds to wait for /controller_manager after the S20 create "
                "before aborting the launch; 0 disables the gate.",
            ),
            profile_arg("use_rviz", "Start RViz."),
            profile_arg(
                "use_cargo_map",
                "Start cargo_volume_mapper (geometry-commit "
                "occupancy grid + surface_2d)."),
            profile_arg(
                "use_packing",
                "Start placement_planner (ComputePlacement with "
                "aperture + corridor gates)."),
            profile_arg(
                "use_vacuum",
                "Start vacuum_controller (sim backend: gz "
                "kinematic follow + PlanningScene attach)."),
            profile_arg(
                "use_motion",
                "Start waypoint_generator + motion_planner "
                "(pick/retreat shells; move_group must also be "
                "on via use_moveit)."),
            profile_arg(
                "named_pose_duration",
                "Max GoToRobotPose FJT duration in seconds "
                "(was 8; actual time is min of this and "
                "max joint delta / named_pose_max_vel).",
            ),
            profile_arg(
                "named_pose_max_vel",
                "GoToRobotPose nominal joint speed in rad/s "
                "(joint_limits max is 1.57).",
            ),
            profile_arg(
                "velocity_scaling",
                "Motion planning trajectory velocity scaling factor "
                "(default 0.3).",
            ),
            profile_arg(
                "acceleration_scaling",
                "Motion planning trajectory acceleration scaling factor "
                "(default 0.3).",
            ),
            profile_arg(
                "use_semantic",
                "Start the YOLO semantic chain (segmenter + point "
                "filter) and feed the detector the cargo cloud."),
            profile_arg(
                "use_perception",
                "Start the perception stack (preprocessor, camera "
                "bridge, depth republisher, detector). Place-only eval "
                "profiles set false so perception nodes are absent."),
            profile_arg(
                "hull_margin",
                "Placement planner inward hull clearance in metres "
                "(default 0 = flush-wall, historical behavior)."),
            profile_arg(
                "min_support_ratio",
                "Placement planner stacking support ratio "
                "(default 0.6)."),
            profile_arg(
                "last_result_max_candidates",
                "Candidates kept in the placement last_result dump "
                "(default 24)."),
            profile_arg(
                "semantic_require_backend",
                "If set, segmenter startup fails unless "
                "stats['backend'] starts with this prefix "
                "(e.g. bbox_fill). Empty disables the guard."),
            profile_arg(
                "semantic_model_name",
                "YOLO-World checkpoint for the segmenter. Empty (the "
                "default) resolves the colcon-installed copy under the "
                "luggage_perception package share; an explicit path "
                "overrides. A bare filename must never be used: it "
                "resolves against the launch CWD and falls back to a "
                "network download."),
            profile_arg(
                "semantic_clip_vendor_dir",
                "Offline CLIP vendor dir (clip_pkg/ + deps/ + "
                "clip_models/) exported to the segmenter as "
                "LUGGAGE_CLIP_VENDOR_DIR. Empty (the default) resolves "
                "the colcon-installed copy under the "
                "luggage_perception package share."),
            profile_arg(
                "visual_kind",
                "Pickup visual: box (primitive AABB) or mesh "
                "(pre-scaled suitcase STL, visual=collision)."),
            profile_arg(
                "size_mode",
                "Pickup size: catalog (small/medium/large) or "
                "continuous (box visual only)."),
            profile_arg(
                "yaw_mode",
                "Override catalog yaw_mode (discrete/continuous)."),
            profile_arg(
                "yaw_range",
                "Yaw range used when yaw_mode is continuous."),
            profile_arg(
                "xy_jitter_range",
                "Pickup XY jitter half-widths in metres."),
            profile_arg(
                "sequence_ids",
                "Comma-separated catalog ids for SpawnNextBox "
                "(e.g. carryon,standard). Empty = weighted random."),
            profile_arg(
                "random_seed",
                "Pickup spawner RNG seed (-1 = entropy). Fixed seeds "
                "make catalog mass sampling deterministic."),
            profile_arg(
                "spawn_at_observe",
                "Spawn gz_ros2_control at the named observe pose.",
            ),
            profile_arg(
                "observe_pose_name",
                "Key in robot_poses YAML (observe or pickup_observe).",
            ),
            profile_arg(
                "robot_poses_config",
                "Robot named-pose YAML path.",
            ),
            profile_arg(
                "atlas_runtime_config",
                "Reachability-atlas runtime YAML path (empty = installed "
                "atlas_runtime.yaml).",
            ),
            profile_arg(
                "atlas_enabled",
                "Force the atlas chain on/off for placement and motion "
                "(empty = yaml default).",
            ),
            profile_arg(
                "frame_runtime",
                "Start the typed v2 runtime chain (identity heartbeats + "
                "readiness gating; requires the full stack on).",
            ),
            profile_arg(
                "frame_calibration_revision",
                "Calibration identity label stamped into the runtime "
                "identity.",
            ),
            profile_arg(
                "frame_driver_peer",
                "Driver node name that joins the v2 graph as the client "
                "peer.",
            ),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", _resource_path()),
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", _resource_path()),
            OpaqueFunction(function=_launch_setup),
        ]
    )
