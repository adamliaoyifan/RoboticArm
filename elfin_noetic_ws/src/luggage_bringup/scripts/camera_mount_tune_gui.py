#!/usr/bin/env python3
"""Tune RealSense mount via 6 Gazebo joints with live RGB preview."""

import math
import os
import sys
import threading
import xml.etree.ElementTree as ET

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import actionlib
import rospy
import rospkg
import yaml
from cv_bridge import CvBridge
from gazebo_msgs.srv import SetModelConfiguration
from moveit_msgs.msg import MoveGroupAction
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QClipboard, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)
from sensor_msgs.msg import Image, JointState
import tf
import tf.transformations as tft

from mount_config_utils import (
    build_mount_yaml,
    mount_dict_to_tune_joints,
    rotation_matrices_equivalent,
    tune_joints_to_fixed_mount,
)

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from luggage_msgs.srv import GoToRobotPose
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_CONTROLLER_ACTION = "/elfin_arm_controller/follow_joint_trajectory"

MOUNT_JOINT_NAMES = [
    "cam_mount_tx",
    "cam_mount_ty",
    "cam_mount_tz",
    "cam_mount_rx",
    "cam_mount_ry",
    "cam_mount_rz",
]

ARM_JOINT_NAMES = [
    "elfin_joint1",
    "elfin_joint2",
    "elfin_joint3",
    "elfin_joint4",
    "elfin_joint5",
    "elfin_joint6",
]

MOUNT_LABELS = [
    "tx (m)",
    "ty (m)",
    "tz (m)",
    "rx about X (rad)",
    "ry about Y (rad)",
    "rz about Z (rad)",
]

DEFAULT_MOUNT_PARENT = "elfin_link6"
DEFAULT_MOUNT_XYZ = [-0.017202, 0.129806, 0.101650]
DEFAULT_MOUNT_RPY = [-1.57079632679, -1.57079632679, 1.57079632679]

DEFAULT_PRISMATIC_LIMIT = 0.50

SLIDER_SCALE = 1000

TF_SAVE_CHILD_LINK = "camera_link"
TF_RPY_WARN_THRESHOLD = 0.05  # rad (~3 deg) per axis vs slider fold


def _pkg_config():
    return os.path.join(rospkg.RosPack().get_path("luggage_description"), "config")


def _default_mount_yaml():
    return os.path.join(_pkg_config(), "realsense_d435_mount.yaml.example")


def _mount_origin_xacro():
    return os.path.join(_pkg_config(), "camera_mount_origin.xacro")


def _move_arm_via_controller(joint_names, joint_positions, duration_sec=1.5):
    client = actionlib.SimpleActionClient(ARM_CONTROLLER_ACTION, FollowJointTrajectoryAction)
    if not client.wait_for_server(rospy.Duration(5.0)):
        return False
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = list(joint_names)
    point = JointTrajectoryPoint()
    point.positions = list(joint_positions)
    point.time_from_start = rospy.Duration(duration_sec)
    goal.trajectory.points = [point]
    client.send_goal(goal)
    if not client.wait_for_result(rospy.Duration(duration_sec + 10.0)):
        client.cancel_goal()
        return False
    return client.get_state() == actionlib.GoalStatus.SUCCEEDED


def _load_mount_config():
    path = rospy.get_param("~mount_config", _default_mount_yaml())
    with open(path, "r") as handle:
        data = yaml.safe_load(handle)
    mount = data.get("mount", data)
    parent = mount.get("parent_link", DEFAULT_MOUNT_PARENT)
    return mount_dict_to_tune_joints(mount), path, parent


def _load_observe_arm_config():
    path = rospy.get_param(
        "~robot_poses_config",
        os.path.join(_pkg_config(), "robot_poses.yaml.example"),
    )
    pose_name = rospy.get_param("~observe_pose_name", "observe")
    with open(path, "r") as handle:
        config = yaml.safe_load(handle)
    pose = config["poses"][pose_name]
    joints = list(pose["joints"])
    values = [float(v) for v in pose["values"]]
    return joints, values


def _sync_d435_mount_yaml(parent_link, mount_body):
    for name in ("realsense_d435.yaml", "realsense_d435.yaml.example"):
        d435_path = os.path.join(_pkg_config(), name)
        if not os.path.isfile(d435_path):
            continue
        with open(d435_path, "r") as handle:
            d435 = yaml.safe_load(handle)
        d435.setdefault("camera", {})["mount"] = {
            "parent_link": parent_link,
            "tune_joints": mount_body["mount"]["tune_joints"],
            "fixed": mount_body["mount"]["fixed"],
            "xyz": mount_body["mount"]["fixed"]["xyz"],
            "rpy": mount_body["mount"]["fixed"]["rpy"],
        }
        with open(d435_path, "w") as handle:
            yaml.safe_dump(d435, handle, default_flow_style=False, sort_keys=False)


def _parse_mount_joint_limits(joint_names):
    urdf_xml = rospy.get_param("/robot_description")
    root = ET.fromstring(urdf_xml)
    limits = {}
    for joint_name in joint_names:
        joint_el = root.find("joint[@name='%s']" % joint_name)
        if joint_el is None:
            default = DEFAULT_PRISMATIC_LIMIT if joint_name.endswith(("tx", "ty", "tz")) else math.pi
            limits[joint_name] = (-default, default)
            continue
        limit_el = joint_el.find("limit")
        if limit_el is None:
            default = DEFAULT_PRISMATIC_LIMIT if joint_name.endswith(("tx", "ty", "tz")) else math.pi
            limits[joint_name] = (-default, default)
            continue
        limits[joint_name] = (
            float(limit_el.get("lower", -math.pi)),
            float(limit_el.get("upper", math.pi)),
        )
    return limits


def _rad_to_slider(rad, lower, upper):
    clamped = max(lower, min(upper, rad))
    return int(round((clamped - lower) / (upper - lower) * SLIDER_SCALE))


def _slider_to_rad(value, lower, upper):
    return lower + (float(value) / SLIDER_SCALE) * (upper - lower)


def _format_yaml_export(parent_link, tune_joints):
    fixed_xyz, fixed_rpy = tune_joints_to_fixed_mount(*tune_joints)
    tx, ty, tz, rx, ry, rz = tune_joints
    tune_xyz_s = ", ".join("%.4f" % v for v in [tx, ty, tz])
    tune_rpy_s = ", ".join("%.4f" % v for v in [rx, ry, rz])
    fixed_xyz_s = ", ".join("%.4f" % v for v in fixed_xyz)
    fixed_rpy_s = ", ".join("%.4f" % v for v in fixed_rpy)
    return (
        "mount:\n"
        "  parent_link: %s\n"
        "  tune_joints:\n"
        "    xyz: [%s]\n"
        "    rx_ry_rz: [%s]\n"
        "  fixed:\n"
        "    xyz: [%s]\n"
        "    rpy: [%s]" % (parent_link, tune_xyz_s, tune_rpy_s, fixed_xyz_s, fixed_rpy_s)
    )


def _lookup_mount_from_tf(parent_link, child_link=TF_SAVE_CHILD_LINK, timeout_sec=2.0):
    """Return mount xyz/rpy from TF parent_link -> child_link (URDF fixed-joint convention)."""
    listener = tf.TransformListener()
    deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
    while rospy.Time.now() < deadline and not rospy.is_shutdown():
        try:
            if listener.canTransform(parent_link, child_link, rospy.Time(0)):
                translation, rotation = listener.lookupTransform(
                    parent_link, child_link, rospy.Time(0)
                )
                roll, pitch, yaw = tft.euler_from_quaternion(rotation, axes="sxyz")
                return (
                    [float(v) for v in translation],
                    [float(roll), float(pitch), float(yaw)],
                )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            pass
        rospy.sleep(0.05)
    return None


def _resolve_mount_for_save(parent_link, slider_xyz, slider_rpy):
    """Return TF-derived fixed mount; refuse to save if TF is unavailable."""
    tf_mount = _lookup_mount_from_tf(parent_link)
    if tf_mount is None:
        rospy.logerr("TF %s -> %s unavailable; refusing to save fixed mount", parent_link, TF_SAVE_CHILD_LINK)
        return None, None, "no TF"

    tf_xyz, tf_rpy = tf_mount
    slider_values = list(slider_xyz) + list(slider_rpy)
    if not rotation_matrices_equivalent(slider_values, tf_rpy, tolerance=TF_RPY_WARN_THRESHOLD):
        rospy.logwarn(
            "Slider rx/ry/rz %s differs from TF fixed rpy %s; saving TF fixed values",
            [round(v, 4) for v in slider_rpy],
            [round(v, 4) for v in tf_rpy],
        )
    return tf_xyz, tf_rpy, "TF"


def _write_mount_files(parent_link, tune_joints, fixed_xyz, fixed_rpy, yaml_path):
    xacro_path = _mount_origin_xacro()
    xacro_body = (
        "<?xml version=\"1.0\"?>\n"
        "<!-- D435 side mount. tune_joints -> fixed URDF mount. Saved by camera_mount_tune_gui.py -->\n"
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
        '  <xacro:property name="cam_mount_parent" value="%s"/>\n'
        '  <xacro:property name="cam_mount_xyz" value="%.6f %.6f %.6f"/>\n'
        '  <xacro:property name="cam_mount_rpy" value="%.8f %.8f %.8f"/>\n'
        "</robot>\n"
        % (
            parent_link,
            fixed_xyz[0],
            fixed_xyz[1],
            fixed_xyz[2],
            fixed_rpy[0],
            fixed_rpy[1],
            fixed_rpy[2],
        )
    )
    with open(xacro_path, "w") as handle:
        handle.write(xacro_body)

    yaml_body = build_mount_yaml(parent_link, tune_joints, fixed_xyz, fixed_rpy)
    yaml_body["meta"] = {"name": "camera_mount_intel_frame", "version": "1.1.0"}
    with open(yaml_path, "w") as handle:
        yaml.safe_dump(yaml_body, handle, default_flow_style=False, sort_keys=False)

    _sync_d435_mount_yaml(parent_link, yaml_body)


class CameraMountTuneWindow(QMainWindow):
    def __init__(self):
        super(CameraMountTuneWindow, self).__init__()
        self.setWindowTitle("Camera Mount Tune")
        self.resize(1100, 620)

        self._model_name = rospy.get_param("~gazebo_model_name", "S20")
        self._live_debounce_ms = int(rospy.get_param("~live_debounce_ms", 50))
        self._mount_values, self._yaml_path, self._parent_link = _load_mount_config()
        self._limits = _parse_mount_joint_limits(MOUNT_JOINT_NAMES)
        self._sliders = {}
        self._value_labels = {}
        self._updating_sliders = False
        self._bridge = CvBridge()
        self._latest_image = None

        self._image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self._on_image, queue_size=1
        )
        self._mount_joint_pub = rospy.Publisher(
            "/cam_mount_tune/joint_states", JointState, queue_size=1, latch=True
        )

        self._set_model_config = None
        gazebo_srv = rospy.get_param(
            "~gazebo_set_model_configuration_service", "/gazebo/set_model_configuration"
        )
        try:
            rospy.wait_for_service(gazebo_srv, timeout=30.0)
            self._set_model_config = rospy.ServiceProxy(gazebo_srv, SetModelConfiguration)
            rospy.loginfo("Live Gazebo preview via %s", gazebo_srv)
        except rospy.ROSException:
            rospy.logwarn("Gazebo set_model_configuration unavailable — live preview disabled")

        observe_srv = rospy.get_param("~go_to_robot_pose_service", "/motion_planner/go_to_robot_pose")
        require_motion = bool(rospy.get_param("~require_motion_planner", False))
        observe_timeout = 60.0 if require_motion else 2.0
        try:
            rospy.wait_for_service(observe_srv, timeout=observe_timeout)
            self._go_to_observe = rospy.ServiceProxy(observe_srv, GoToRobotPose)
        except rospy.ROSException:
            self._go_to_observe = None

        self._live_timer = QTimer(self)
        self._live_timer.setSingleShot(True)
        self._live_timer.timeout.connect(self._apply_live_mount)

        self._build_ui()
        self._load_mount_sliders(apply_live=False)

        # Init node sets observe+mount at launch; refresh mount in GUI once Gazebo is reachable.
        if self._set_model_config is not None:
            QTimer.singleShot(1500, self._apply_live_mount)

        if (
            rospy.get_param("~use_moveit_observe_on_start", False)
            and rospy.get_param("~move_to_observe_on_start", False)
            and self._go_to_observe
        ):
            self._schedule_observe_when_ready()

        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._refresh_image)
        self._refresh_timer.start(66)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)

        controls = QVBoxLayout()
        root.addLayout(controls, stretch=2)

        mount_box = QGroupBox("Mount joints on %s (cam_mount_tx … cam_mount_rz)" % self._parent_link)
        grid = QGridLayout(mount_box)
        for row, (joint_name, label) in enumerate(zip(MOUNT_JOINT_NAMES, MOUNT_LABELS)):
            lower, upper = self._limits[joint_name]
            grid.addWidget(QLabel(label), row, 0)
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(SLIDER_SCALE)
            slider.valueChanged.connect(self._make_slider_handler(joint_name))
            value_label = QLabel("")
            grid.addWidget(slider, row, 1)
            grid.addWidget(value_label, row, 2)
            self._sliders[joint_name] = slider
            self._value_labels[joint_name] = value_label
        controls.addWidget(mount_box)

        self._live_checkbox = QCheckBox("Live preview in Gazebo (SetModelConfiguration)")
        live_ok = self._set_model_config is not None
        self._live_checkbox.setChecked(live_ok)
        self._live_checkbox.setEnabled(live_ok)
        if not live_ok:
            self._live_checkbox.setToolTip("Requires sim_world_tune.launch (6 mount joints in URDF)")
        controls.addWidget(self._live_checkbox)

        btn_row = QHBoxLayout()
        observe_btn = QPushButton("Snap to Observe Pose (Gazebo)")
        observe_btn.clicked.connect(self._move_to_observe_and_apply)
        reset_btn = QPushButton("Reset to YAML")
        reset_btn.clicked.connect(self._reset_from_yaml)
        btn_row.addWidget(observe_btn)
        btn_row.addWidget(reset_btn)
        controls.addLayout(btn_row)

        export_box = QGroupBox("Export")
        export_layout = QVBoxLayout(export_box)
        self._export_label = QLabel("")
        self._export_label.setWordWrap(True)
        self._export_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        export_layout.addWidget(self._export_label)
        btns = QHBoxLayout()
        copy_btn = QPushButton("Copy")
        copy_btn.clicked.connect(self._copy_export)
        save_btn = QPushButton("Save to URDF config")
        save_btn.clicked.connect(self._save_export)
        save_btn.setToolTip("Writes camera_mount_origin.xacro + YAML; restart sim to apply permanently")
        btns.addWidget(copy_btn)
        btns.addWidget(save_btn)
        export_layout.addLayout(btns)
        controls.addWidget(export_box)

        self._status_label = QLabel(
            "Status: Arm starts at observe pose — drag mount sliders; RGB updates live."
        )
        self._status_label.setWordWrap(True)
        controls.addWidget(self._status_label)
        controls.addStretch(1)

        preview_box = QGroupBox("RGB preview (/camera/color/image_raw)")
        preview_layout = QVBoxLayout(preview_box)
        self._image_label = QLabel("Waiting for camera ...")
        self._image_label.setAlignment(Qt.AlignCenter)
        self._image_label.setMinimumSize(640, 480)
        self._image_label.setStyleSheet("background-color: #222; color: #ccc;")
        preview_layout.addWidget(self._image_label)
        root.addWidget(preview_box, stretch=3)

        self._update_export_text()

    def _format_value_text(self, joint_name, value):
        if joint_name.endswith(("tx", "ty", "tz")):
            return "%.4f m" % value
        return "%.4f rad (%.1f deg)" % (value, math.degrees(value))

    def _make_slider_handler(self, joint_name):
        def handler(_value):
            if self._updating_sliders:
                return
            rad = _slider_to_rad(self._sliders[joint_name].value(), *self._limits[joint_name])
            self._value_labels[joint_name].setText(self._format_value_text(joint_name, rad))
            self._update_export_text()
            self._schedule_live_apply()

        return handler

    def _set_slider_value(self, joint_name, value, apply_live=True):
        self._updating_sliders = True
        self._sliders[joint_name].setValue(
            _rad_to_slider(value, *self._limits[joint_name])
        )
        self._value_labels[joint_name].setText(self._format_value_text(joint_name, value))
        self._updating_sliders = False
        if apply_live:
            self._schedule_live_apply()

    def _load_mount_sliders(self, apply_live=True):
        values, _path, self._parent_link = _load_mount_config()
        for joint_name, value in zip(MOUNT_JOINT_NAMES, values):
            self._set_slider_value(joint_name, value, apply_live=False)
        self._update_export_text()
        if apply_live:
            self._apply_live_mount()

    def _current_mount(self):
        values = []
        for joint_name in MOUNT_JOINT_NAMES:
            slider_val = self._sliders[joint_name].value()
            values.append(_slider_to_rad(slider_val, *self._limits[joint_name]))
        xyz = values[:3]
        rpy = values[3:]
        return xyz, rpy, values

    def _schedule_live_apply(self):
        if self._live_checkbox.isChecked() and self._set_model_config is not None:
            self._live_timer.start(self._live_debounce_ms)

    def _publish_mount_joint_states(self, joint_values):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(MOUNT_JOINT_NAMES)
        msg.position = list(joint_values)
        self._mount_joint_pub.publish(msg)

    def _apply_live_mount(self):
        if self._set_model_config is None:
            return
        _, _, joint_values = self._current_mount()
        try:
            resp = self._set_model_config(
                model_name=self._model_name,
                urdf_param_name="robot_description",
                joint_names=MOUNT_JOINT_NAMES,
                joint_positions=joint_values,
            )
            if resp.success:
                self._publish_mount_joint_states(joint_values)
                self._status_label.setText("Status: Live mount preview updated")
            else:
                self._status_label.setText(
                    "Status: Gazebo rejected mount — %s" % resp.status_message
                )
        except rospy.ServiceException as exc:
            self._status_label.setText("Status: Live preview failed — %s" % exc)

    def _update_export_text(self):
        _xyz, _rpy, values = self._current_mount()
        self._export_label.setText(
            "# Updates config/camera_mount_origin.xacro on Save:\n"
            + _format_yaml_export(self._parent_link, values)
        )

    def _on_image(self, msg):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        self._latest_image = cv_image

    def _refresh_image(self):
        if self._latest_image is None:
            return
        rgb = self._latest_image[:, :, ::-1].copy()
        h, w, ch = rgb.shape
        qimage = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimage)
        self._image_label.setPixmap(
            pixmap.scaled(self._image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        )

    def _schedule_observe_when_ready(self):
        action_name = rospy.get_param("~move_group_action", "/move_group")
        timeout = float(rospy.get_param("~move_group_wait_timeout", 60.0))
        self._status_label.setText("Status: Waiting for move_group ...")

        def worker():
            client = actionlib.SimpleActionClient(action_name, MoveGroupAction)
            ready = client.wait_for_server(rospy.Duration(timeout))
            if ready:
                QTimer.singleShot(0, self._move_to_observe_via_moveit_and_apply)
            else:

                def notify_timeout():
                    self._status_label.setText(
                        "Status: move_group not ready in %.0fs — click Move to Observe Pose"
                        % timeout
                    )

                QTimer.singleShot(0, notify_timeout)

        threading.Thread(target=worker, daemon=True).start()

    def _snap_arm_observe_via_gazebo(self):
        try:
            arm_joints, arm_values = _load_observe_arm_config()
            return _move_arm_via_controller(arm_joints, arm_values, duration_sec=1.5)
        except (ValueError, rospy.ROSException):
            return False

    def _move_to_observe_via_moveit_and_apply(self):
        if self._go_to_observe is None:
            self._status_label.setText("Status: motion_planner not available")
            self._apply_live_mount()
            return
        try:
            resp = self._go_to_observe("observe")
            if resp.success:
                self._status_label.setText("Status: At observe pose (MoveIt) — tune mount here")
            else:
                self._status_label.setText("Status: Observe move failed — %s" % resp.message)
        except rospy.ServiceException as exc:
            self._status_label.setText("Status: go_to_robot_pose failed — %s" % exc)
        self._apply_live_mount()

    def _move_to_observe_and_apply(self):
        if self._snap_arm_observe_via_gazebo():
            self._status_label.setText("Status: At observe pose (Gazebo) — tune mount here")
            self._apply_live_mount()
            return
        if self._go_to_observe is None:
            self._status_label.setText("Status: Gazebo snap failed — motion_planner unavailable")
            self._apply_live_mount()
            return
        try:
            resp = self._go_to_observe("observe")
            if resp.success:
                self._status_label.setText("Status: At observe pose (MoveIt) — tune mount here")
            else:
                self._status_label.setText("Status: Observe move failed — %s" % resp.message)
        except rospy.ServiceException as exc:
            self._status_label.setText("Status: go_to_robot_pose failed — %s" % exc)
        self._apply_live_mount()

    def _reset_from_yaml(self):
        self._load_mount_sliders(apply_live=True)
        self._status_label.setText("Status: Reset mount from YAML")

    def _copy_export(self):
        _xyz, _rpy, values = self._current_mount()
        QApplication.clipboard().setText(
            _format_yaml_export(self._parent_link, values), QClipboard.Clipboard
        )
        self._status_label.setText("Status: Copied mount YAML to clipboard")

    def _save_export(self):
        slider_xyz, slider_rpy, slider_values = self._current_mount()
        fixed_xyz, fixed_rpy, source = _resolve_mount_for_save(
            self._parent_link, slider_xyz, slider_rpy
        )
        if fixed_xyz is None or fixed_rpy is None:
            QMessageBox.warning(
                self,
                "Save failed",
                "TF %s -> %s is unavailable; cannot write production fixed mount."
                % (self._parent_link, TF_SAVE_CHILD_LINK),
            )
            self._status_label.setText("Status: Save failed — TF unavailable")
            return
        try:
            _write_mount_files(
                self._parent_link,
                slider_values,
                fixed_xyz,
                fixed_rpy,
                self._yaml_path,
            )
        except OSError as exc:
            QMessageBox.warning(self, "Save failed", str(exc))
            return
        self._status_label.setText(
            "Status: Saved (%s) tune_joints + fixed URDF to %s — restart sim_full"
            % (source, self._yaml_path)
        )


def main():
    rospy.init_node("camera_mount_tune_gui")
    if os.environ.get("LIBGL_ALWAYS_SOFTWARE", "0") == "1":
        QApplication.setAttribute(Qt.AA_UseSoftwareOpenGL, True)
    app = QApplication(sys.argv)
    window = CameraMountTuneWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
