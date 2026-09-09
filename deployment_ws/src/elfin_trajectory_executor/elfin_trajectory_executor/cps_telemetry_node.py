"""Read-only Huayan CPS telemetry for rosbag recording.

Connects to the controller without electrify / GrpEnable. Publishes:

- ``/joint_states`` — position (rad) and velocity (rad/s)
- ``/elfin/tcp_pose`` — actual TCP in ``elfin_base_link`` (metres)
- ``/elfin/cps_telemetry`` — JSON snapshot (deg, deg/s, TCP mm)

Do not run this together with ``jazzy_real.launch.py``: CPS TCP is one client.
"""

from __future__ import annotations

import json

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String

from .cps_parse import rpy_deg_to_quat_xyzw
from .huayan_interface import HuayanInterface

JOINT_NAMES = [
    "elfin_joint1",
    "elfin_joint2",
    "elfin_joint3",
    "elfin_joint4",
    "elfin_joint5",
    "elfin_joint6",
]


class CpsTelemetryNode(Node):
    def __init__(self) -> None:
        super().__init__("cps_telemetry")
        self.declare_parameter("robot_ip", "192.168.0.10")
        self.declare_parameter("robot_port", 10003)
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("tcp_frame_id", "elfin_base_link")

        ip = self.get_parameter("robot_ip").value
        port = int(self.get_parameter("robot_port").value)
        rate = float(self.get_parameter("rate_hz").value)
        self._tcp_frame = str(self.get_parameter("tcp_frame_id").value)

        self._iface = HuayanInterface(node=self, robot_ip=ip, robot_port=port)
        if not self._iface.connect(monitor_only=True):
            raise RuntimeError(
                "CPS monitor connect failed. Check ROBOT_IP, PYTHONPATH/HUAYAN_SDK, "
                "and that jazzy_real is not already holding the TCP port."
            )

        self._js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._tcp_pub = self.create_publisher(PoseStamped, "/elfin/tcp_pose", 10)
        self._json_pub = self.create_publisher(String, "/elfin/cps_telemetry", 10)
        period = 1.0 / max(rate, 1.0)
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"[cps_telemetry] monitor {ip}:{port} at {rate:.1f} Hz (no servo enable)"
        )

    def _tick(self) -> None:
        self._iface.refresh()
        now = self.get_clock().now().to_msg()
        js = JointState()
        js.header = Header(stamp=now)
        js.name = list(JOINT_NAMES)
        js.position = list(self._iface.current_positions)
        js.velocity = list(self._iface.current_velocities)
        js.effort = [0.0] * 6
        self._js_pub.publish(js)

        tcp = self._iface.current_tcp_mm_deg
        pose = PoseStamped()
        pose.header = Header(stamp=now, frame_id=self._tcp_frame)
        if len(tcp) >= 6:
            pose.pose.position.x = tcp[0] * 0.001
            pose.pose.position.y = tcp[1] * 0.001
            pose.pose.position.z = tcp[2] * 0.001
            qx, qy, qz, qw = rpy_deg_to_quat_xyzw(tcp[3], tcp[4], tcp[5])
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
        self._tcp_pub.publish(pose)

        payload = {
            "schema": "elfin_cps_telemetry/v1",
            "q_deg": list(self._iface.current_positions_deg),
            "qd_deg_s": list(self._iface.current_velocities_deg),
            "tcp_mm_deg": list(tcp),
        }
        self._json_pub.publish(String(data=json.dumps(payload)))

    def destroy_node(self) -> None:
        try:
            self._iface.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CpsTelemetryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
