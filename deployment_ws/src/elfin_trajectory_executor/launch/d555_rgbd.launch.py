"""D555 PoE: color + aligned depth only (apt librealsense DDS).

Intel's ROS wrapper is built against a librealsense without DDS. The apt
library in /lib/x86_64-linux-gnu enumerates the D555. This launch prepends
that path for this node only so Livox's SDK path is left alone.

Kept after d555_host_stamp (host clock):
  /camera/d555/color/image_raw/compressed + camera_info
  /camera/d555/aligned_depth_to_color/image_raw/compressed + camera_info
  /camera/d555/imu (DDS motion/sample restamped; also gyro/accel)

The wrapper stays on image_transport/raw. Forcing compressed-only plugins
(and PNG format params on 16UC1 aligned depth) SIGSEGV'd
realsense2_camera_node on this D555. JPEG/PNG for the bag is encoded in
d555_host_stamp from restamped *_hw Images.

Point cloud stays off: color+aligned depth already contain the same
geometry; replay can rebuild /camera/depth/points.

Canonical image topics are host-stamped by d555_host_stamp (one stamp
per color+aligned_depth pair). The driver publishes *_hw with the
unstable D555 HARDWARE_CLOCK mapping.

The wrapper still emits a few cheap extras while depth is on (raw depth,
metadata, extrinsics). Those are not in the lean bag regex.

Standalone:
  ros2 launch elfin_trajectory_executor d555_rgbd.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

APT_LIBREALSENSE = "/lib/x86_64-linux-gnu"


def _truthy(value: str) -> bool:
    return value.strip().lower() in ("true", "1", "yes", "on")


def _ld_library_path():
    """Apt librealsense 2.58.4 first. Jazzy's 2.58.1 has no DDS and wins via RPATH/ament."""
    cur = os.environ.get("LD_LIBRARY_PATH", "")
    parts = [p for p in cur.split(os.pathsep) if p]
    drop = {"/opt/ros/jazzy/lib/x86_64-linux-gnu"}
    parts = [p for p in parts if p not in drop]
    if APT_LIBREALSENSE in parts:
        parts.remove(APT_LIBREALSENSE)
    parts.insert(0, APT_LIBREALSENSE)
    return os.pathsep.join(parts)


def _driver_transport_params() -> dict:
    """Lock the wrapper on raw Image. Do not load compressed_pub here."""
    return {
        "image_transport.publisher.enable_pub_plugins": ["image_transport/raw"],
    }


def _prefixed_remaps(camera_name: str, pairs: list[tuple[str, str]]) -> list[tuple[str, str]]:
    """Match both relative and {camera_name}/... publisher names."""
    out = []
    for src, dst in pairs:
        out.append((src, dst))
        out.append((f"{camera_name}/{src}", f"{camera_name}/{dst}"))
    return out


def _imu_remaps(camera_name: str) -> list[tuple[str, str]]:
    """D555 DDS IMU is motion/sample, not USB gyro/accel."""
    return _prefixed_remaps(
        camera_name,
        [
            ("imu", "imu_hw"),
            ("gyro/sample", "gyro/sample_hw"),
            ("accel/sample", "accel/sample_hw"),
            ("motion/sample", "motion/sample_hw"),
        ],
    )


def _node(context, *args, **kwargs):
    ns = LaunchConfiguration("camera_namespace").perform(context)
    name = LaunchConfiguration("camera_name").perform(context)
    serial = LaunchConfiguration("serial_no").perform(context)
    pointcloud = _truthy(LaunchConfiguration("pointcloud_enable").perform(context))
    rgbd = _truthy(LaunchConfiguration("enable_rgbd").perform(context))
    infra = _truthy(LaunchConfiguration("enable_infra").perform(context))
    imu = _truthy(LaunchConfiguration("enable_imu").perform(context))
    color1 = _truthy(LaunchConfiguration("enable_color1").perform(context))
    compress = _truthy(LaunchConfiguration("compress").perform(context))
    jpeg_quality = int(LaunchConfiguration("jpeg_quality").perform(context))
    ld_path = _ld_library_path()
    driver_params = {
                    "use_sim_time": False,
                    "camera_name": name,
                    "serial_no": serial,
                    "enable_color": True,
                    "enable_color1": color1,
                    "enable_depth": True,
                    "enable_sync": True,
                    "enable_rgbd": rgbd,
                    "align_depth.enable": True,
                    "pointcloud.enable": pointcloud,
                    "enable_infra": False,
                    "enable_infra1": infra,
                    "enable_infra2": infra,
                    "enable_gyro": imu,
                    "enable_accel": imu,
                    "enable_motion": imu,
                    "enable_safety": False,
                    "enable_labeled_point_cloud": False,
                    "enable_occupancy": False,
                    "unite_imu_method": 2 if imu else 0,
                    "publish_tf": True,
                    "tf_publish_rate": 0.0,
                    "diagnostics_period": 0.0,
                    "rgb_camera.color_format": "RGB8",
                    # 896x504@30 dropped the DDS device on this cell.
                    "rgb_camera.color_profile": LaunchConfiguration(
                        "color_profile").perform(context),
                    "depth_module.depth_profile": LaunchConfiguration(
                        "depth_profile").perform(context),
                    "depth_module.infra_profile": LaunchConfiguration(
                        "depth_profile").perform(context),
                }
    driver_params.update(_driver_transport_params())
    return [
        LogInfo(
            msg=(
                "[d555] namespace=/%s/%s serial=%s color+aligned_depth "
                "pointcloud=%s rgbd=%s imu=%s color1=%s "
                "clock=host ROS (d555_host_stamp) compress=%s "
                "driver_image_transport=raw"
                % (
                    ns,
                    name,
                    serial or "(first DDS device)",
                    pointcloud,
                    rgbd,
                    imu,
                    color1,
                    compress,
                )
            )
        ),
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            namespace=ns,
            name=name,
            output="screen",
            emulate_tty=True,
            additional_env={"LD_LIBRARY_PATH": ld_path},
            prefix="env LD_LIBRARY_PATH=" + ld_path,
            remappings=_prefixed_remaps(
                name,
                [
                    ("color/image_raw", "color/image_hw"),
                    ("color/image_raw/compressed", "color/image_hw/compressed"),
                    ("color/camera_info", "color/camera_info_hw"),
                    (
                        "aligned_depth_to_color/image_raw",
                        "aligned_depth_to_color/image_hw",
                    ),
                    (
                        "aligned_depth_to_color/image_raw/compressed",
                        "aligned_depth_to_color/image_hw/compressed",
                    ),
                    (
                        "aligned_depth_to_color/camera_info",
                        "aligned_depth_to_color/camera_info_hw",
                    ),
                ],
            )
            + _imu_remaps(name),
            parameters=[driver_params],
        ),
        Node(
            package="elfin_trajectory_executor",
            executable="d555_host_stamp",
            name="d555_host_stamp",
            output="screen",
            parameters=[
                {
                    "use_sim_time": False,
                    "camera_namespace": ns,
                    "camera_name": name,
                    "enable_imu": imu,
                    "compress": compress,
                    "subscribe_compressed": False,
                    "publish_raw": _truthy(
                        LaunchConfiguration("publish_raw").perform(context)
                    ),
                    "jpeg_quality": jpeg_quality,
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_namespace", default_value="camera"),
            DeclareLaunchArgument("camera_name", default_value="d555"),
            DeclareLaunchArgument(
                "serial_no",
                default_value="419222302385",
                description="D555 PoE serial. Empty = first DDS device.",
            ),
            DeclareLaunchArgument(
                "pointcloud_enable",
                default_value="false",
                description="Colored point cloud. Off: replay rebuilds from depth.",
            ),
            DeclareLaunchArgument(
                "enable_rgbd",
                default_value="false",
                description="Combined RGBD topic.",
            ),
            DeclareLaunchArgument(
                "enable_infra",
                default_value="false",
                description="Infrared streams.",
            ),
            DeclareLaunchArgument(
                "enable_imu",
                default_value="true",
                description="D555 gyro/accel/motion (BMI088). Small compared with images.",
            ),
            DeclareLaunchArgument(
                "enable_color1",
                default_value="false",
                description="D555 second RGB + aligned_depth_to_color1.",
            ),
            DeclareLaunchArgument(
                "color_profile",
                default_value="640,360,15",
                description="rgb_camera.color_profile. 896x504@30 drops DDS.",
            ),
            DeclareLaunchArgument(
                "depth_profile",
                default_value="640,360,15",
                description="depth_module.depth_profile. Match colour.",
            ),
            DeclareLaunchArgument(
                "compress",
                default_value="true",
                description="d555_host_stamp JPEG/PNG on .../image_raw/compressed. Driver stays raw.",
            ),
            DeclareLaunchArgument(
                "publish_raw",
                default_value="false",
                description="Also publish uncompressed image_raw (doubles ROS traffic).",
            ),
            DeclareLaunchArgument(
                "jpeg_quality",
                default_value="80",
                description="JPEG quality 1-100 for color.",
            ),
            OpaqueFunction(function=_node),
        ]
    )
