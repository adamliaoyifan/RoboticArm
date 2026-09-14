"""Live D555 backend feeding the same canonical preprocessor used by sim.

The site driver remains raw-only because loading its compressed transport
plugin has crashed the D555 process.  Compression and acquisition-clock
mapping therefore live in the bounded backend adapter.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truthy(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _prefixed_remaps(camera_name, pairs):
    result = []
    for source, target in pairs:
        result.append((source, target))
        result.append((camera_name + "/" + source,
                       camera_name + "/" + target))
    return result


def _nodes(context):
    namespace = LaunchConfiguration("camera_namespace").perform(context)
    camera_name = LaunchConfiguration("camera_name").perform(context)
    serial = LaunchConfiguration("serial_no").perform(context)
    profile = LaunchConfiguration("profile").perform(context)
    library_path = LaunchConfiguration(
        "librealsense_library_path").perform(context).strip()
    start_preprocessor = _truthy(
        LaunchConfiguration("start_preprocessor").perform(context))

    driver_params = {
        "use_sim_time": False,
        "camera_name": camera_name,
        "serial_no": serial,
        "enable_color": True,
        "enable_depth": True,
        "enable_sync": True,
        "enable_rgbd": False,
        "align_depth.enable": True,
        "pointcloud.enable": False,
        "enable_infra": False,
        "enable_infra1": False,
        "enable_infra2": False,
        "enable_gyro": False,
        "enable_accel": False,
        "publish_tf": True,
        "tf_publish_rate": 0.0,
        "diagnostics_period": 0.0,
        "rgb_camera.color_format": "RGB8",
        "rgb_camera.color_profile": profile,
        "depth_module.depth_profile": profile,
        "image_transport.publisher.enable_pub_plugins": ["image_transport/raw"],
    }
    remaps = _prefixed_remaps(camera_name, [
        ("color/image_raw", "color/image_hw"),
        ("color/camera_info", "color/camera_info_hw"),
        ("aligned_depth_to_color/image_raw",
         "aligned_depth_to_color/image_hw"),
        ("aligned_depth_to_color/camera_info",
         "aligned_depth_to_color/camera_info_hw"),
    ])
    driver_kwargs = {}
    if library_path:
        current = [p for p in os.environ.get("LD_LIBRARY_PATH", "").split(
            os.pathsep) if p and p != library_path]
        effective = os.pathsep.join([library_path] + current)
        driver_kwargs["additional_env"] = {"LD_LIBRARY_PATH": effective}
        driver_kwargs["prefix"] = "env LD_LIBRARY_PATH=" + effective

    nodes = [
        LogInfo(msg=(
            "[d555] raw driver -> acquisition-clock/codec adapter -> "
            "shared canonical preprocessor; profile=" + profile)),
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            namespace=namespace,
            name=camera_name,
            output="screen",
            remappings=remaps,
            parameters=[driver_params],
            **driver_kwargs
        ),
        Node(
            package="luggage_perception",
            executable="d555_transport_adapter_node.py",
            name="d555_transport_adapter",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "camera_namespace": namespace,
                "camera_name": camera_name,
                "jpeg_quality": int(LaunchConfiguration(
                    "jpeg_quality").perform(context)),
                "publish_raw": _truthy(LaunchConfiguration(
                    "publish_raw").perform(context)),
                "require_clock_lock": True,
            }],
        ),
    ]
    if start_preprocessor:
        nodes.append(Node(
            package="luggage_perception",
            executable="sensor_preprocessor_node.py",
            name="sensor_preprocessor",
            output="screen",
            parameters=[LaunchConfiguration(
                "preprocessor_config").perform(context)],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("camera_namespace", default_value="camera"),
        DeclareLaunchArgument("camera_name", default_value="d555"),
        DeclareLaunchArgument("serial_no", default_value="419222302385"),
        DeclareLaunchArgument(
            "profile", default_value="640,360,15",
            description="Site-proven equal colour/depth profile."),
        DeclareLaunchArgument("jpeg_quality", default_value="80"),
        DeclareLaunchArgument("publish_raw", default_value="false"),
        DeclareLaunchArgument("start_preprocessor", default_value="true"),
        DeclareLaunchArgument(
            "preprocessor_config",
            default_value=os.path.join(
                os.path.dirname(os.path.dirname(__file__)),
                "config", "preprocessor_d555_live.yaml")),
        DeclareLaunchArgument(
            "librealsense_library_path",
            default_value="/lib/x86_64-linux-gnu",
            description="Site D555 DDS librealsense path; empty disables override."),
        OpaqueFunction(function=_nodes),
    ])
