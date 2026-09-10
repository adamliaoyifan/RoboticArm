"""Read-only Huayan CPS telemetry for rosbag recording.

Connects to the controller without electrify / GrpEnable. Each tick is one
complete CPS snapshot of the real machine (no held-over fields, no padded Hz):

- ``/joint_states`` — actual ACS rad, vel rad/s, effort=ampere (this tick)
- ``/elfin/joint_kinematics`` — actual + cmd ACS + finite-diff accel
- ``/elfin/tcp_pose`` — actual TCP from the same ``ReadActPos`` packet
- ``/elfin/cps_telemetry`` — JSON with CPS raw + sources + actual sample rate
- ``/elfin/cps_rate`` — ``[rate_hz_set, rate_hz_act, refresh_ms]``
- ``/vacuum/io`` ``/vacuum/di0`` ``/vacuum/do0`` ``/vacuum/do1`` — box DI/DO

Do not run this together with ``jazzy_real.launch.py``: CPS TCP is one client.
"""

from __future__ import annotations

import json
import time

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Header, String
from trajectory_msgs.msg import JointTrajectoryPoint

from .cps_parse import rpy_deg_to_quat_xyzw
from .huayan_interface import HuayanInterface
from .vacuum_io import (
    VacuumIoPublisher,
    apply_vacuum_io_params,
    declare_vacuum_io_params,
    snapshot as vacuum_snapshot,
)

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
        declare_vacuum_io_params(self)

        ip = self.get_parameter("robot_ip").value
        port = int(self.get_parameter("robot_port").value)
        rate = float(self.get_parameter("rate_hz").value)
        self._rate_hz_set = rate
        self._tcp_frame = str(self.get_parameter("tcp_frame_id").value)
        self._rate_hz_act = 0.0
        self._last_tick_mono = 0.0

        self._iface = HuayanInterface(node=self, robot_ip=ip, robot_port=port)
        apply_vacuum_io_params(self, self._iface)
        if not self._iface.connect(monitor_only=True):
            raise RuntimeError(
                "CPS monitor connect failed. Check ROBOT_IP, PYTHONPATH/HUAYAN_SDK, "
                "and that jazzy_real is not already holding the TCP port."
            )

        self._js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._kin_pub = self.create_publisher(
            JointTrajectoryControllerState, "/elfin/joint_kinematics", 10
        )
        self._tcp_pub = self.create_publisher(PoseStamped, "/elfin/tcp_pose", 10)
        self._json_pub = self.create_publisher(String, "/elfin/cps_telemetry", 10)
        self._rate_pub = self.create_publisher(Float64MultiArray, "/elfin/cps_rate", 10)
        self._vacuum_pub = VacuumIoPublisher(self)
        self._logged_nonzero = False
        period = 1.0 / max(rate, 1.0)
        self._timer = self.create_timer(period, self._tick)
        q0 = [round(v, 2) for v in self._iface.current_positions_deg]
        self.get_logger().info(
            f"[cps_telemetry] monitor {ip}:{port} poll_hz={rate:.1f} "
            f"(publishes only complete CPS snapshots; actual Hz on /elfin/cps_rate). "
            f"q_deg={q0}"
        )

    def _stamp_for_sample(self):
        now = self.get_clock().now()
        sample_mono = float(self._iface.last_sample_mono)
        if sample_mono <= 0.0:
            return now.to_msg()
        delay = max(0.0, time.monotonic() - sample_mono)
        return (now - Duration(seconds=delay)).to_msg()

    def _tick(self) -> None:
        try:
            self._iface.refresh()
        except (OSError, ConnectionError):
            return
        if not self._iface.last_acs_ok:
            return

        now_mono = time.monotonic()
        if self._last_tick_mono > 0.0:
            dt = now_mono - self._last_tick_mono
            if dt > 1e-4:
                inst = 1.0 / dt
                self._rate_hz_act = (
                    inst if self._rate_hz_act <= 0.0 else 0.8 * self._rate_hz_act + 0.2 * inst
                )
        self._last_tick_mono = now_mono
        now = self._stamp_for_sample()
        q = list(self._iface.current_positions)
        qd = list(self._iface.current_velocities)
        qdd = list(self._iface.current_accelerations)
        effort = list(self._iface.current_currents)
        q_cmd = list(self._iface.command_positions)

        js = JointState()
        js.header = Header(stamp=now)
        js.name = list(JOINT_NAMES)
        js.position = q
        js.velocity = qd
        if self._iface.last_cur_ok and len(effort) == 6:
            js.effort = effort
        self._js_pub.publish(js)

        kin = JointTrajectoryControllerState()
        kin.header = Header(stamp=now)
        kin.joint_names = list(JOINT_NAMES)
        kin.feedback = JointTrajectoryPoint(
            positions=q,
            velocities=qd,
            accelerations=qdd if self._iface.acc_source != "none" else [],
            effort=effort if self._iface.last_cur_ok and len(effort) == 6 else [],
        )
        if self._iface.last_cmd_ok and len(q_cmd) == 6:
            kin.reference = JointTrajectoryPoint(positions=q_cmd)
        kin.speed_scaling_factor = 1.0
        self._kin_pub.publish(kin)

        if not self._logged_nonzero and any(abs(v) > 1e-6 for v in q):
            self._logged_nonzero = True
            self.get_logger().info(
                "[cps_telemetry] /joint_states from CPS q_rad=%s qd_rad_s=%s"
                % ([round(v, 4) for v in q], [round(v, 4) for v in qd])
            )

        tcp = list(self._iface.current_tcp_mm_deg) if self._iface.last_tcp_ok else []
        if len(tcp) >= 6:
            pose = PoseStamped()
            pose.header = Header(stamp=now, frame_id=self._tcp_frame)
            pose.pose.position.x = tcp[0] * 0.001
            pose.pose.position.y = tcp[1] * 0.001
            pose.pose.position.z = tcp[2] * 0.001
            qx, qy, qz, qw = rpy_deg_to_quat_xyzw(tcp[3], tcp[4], tcp[5])
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            self._tcp_pub.publish(pose)

        cps_vel = self._iface.cps_velocities_deg
        payload = {
            "schema": "elfin_cps_telemetry/v4",
            "rate_hz_set": self._rate_hz_set,
            "rate_hz_act": round(self._rate_hz_act, 3),
            "refresh_ms": round(1000.0 * float(self._iface.last_refresh_dt_s), 2),
            "sample_ok": {
                "acs": self._iface.last_acs_ok,
                "tcp": self._iface.last_tcp_ok,
                "vel_cps": self._iface.last_vel_cps_ok,
                "cur": self._iface.last_cur_ok,
                "cmd": self._iface.last_cmd_ok,
                "io": self._iface.last_io_ok,
            },
            "q_deg": list(self._iface.current_positions_deg),
            "q_cmd_deg": list(self._iface.command_positions_deg),
            "qd_deg_s": list(self._iface.current_velocities_deg),
            "qd_cps_deg_s": cps_vel,
            "qdd_deg_s2": list(self._iface.current_accelerations_deg),
            "q_rad": q,
            "qd_rad_s": qd,
            "qdd_rad_s2": qdd,
            "i_a": effort if self._iface.last_cur_ok else [],
            "vel_source": self._iface.vel_source,
            "qdd_source": self._iface.acc_source,
            "tcp_mm_deg": tcp,
            "vacuum": vacuum_snapshot(self._iface),
        }
        self._json_pub.publish(String(data=json.dumps(payload)))
        rate_msg = Float64MultiArray()
        rate_msg.data = [
            float(self._rate_hz_set),
            float(self._rate_hz_act),
            float(payload["refresh_ms"]),
        ]
        self._rate_pub.publish(rate_msg)
        self._vacuum_pub.publish(self._iface)

    def destroy_node(self) -> None:
        try:
            self._timer.cancel()
        except Exception:
            pass
        try:
            self._iface.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = CpsTelemetryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
