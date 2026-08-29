#!/usr/bin/env python3
"""Unified EOF mount stack tune GUI: three-layer TF calibration in one window.

Controls all three TF segments simultaneously:
  1. EOF (elfin_end_link) -> suction_panel
  2. suction_panel -> eef_mount_adapter (arm_realsense connector)
  3. eef_mount_adapter -> camera_link

Each segment has independent xyz/rpy sliders, Reset/Save buttons.
Live preview sets all 18 tune joints at once via Gazebo SetModelConfiguration.
"""

import math
import os
import sys
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
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QClipboard, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
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
    tune_joints_to_fixed_mount,
)

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_CONTROLLER_ACTION_DEFAULT = "/S20/elfin_arm_controller/follow_joint_trajectory"

SLIDER_SCALE = 1000
DEFAULT_PRISMATIC_LIMIT = 0.30


LAYER_DEFS = [
    {
        "key": "suction",
        "title": "Layer 1: EOF -> Suction Panel",
        "parent_link": "elfin_end_link",
        "child_link": "suction_panel",
        "joint_names": [
            "suction_flange_tx", "suction_flange_ty", "suction_flange_tz",
            "suction_flange_rx", "suction_flange_ry", "suction_flange_rz",
        ],
        "topic": "/suction_flange_tune/joint_states",
        "yaml_file": "suction_flange.yaml.example",
        "xacro_file": "suction_flange_origin.xacro",
        "xacro_props": ("suction_flange_parent", "suction_flange_xyz", "suction_flange_rpy"),
    },
    {
        "key": "adapter",
        "title": "Layer 2: Suction Panel -> arm_realsense Connector",
        "parent_link": "suction_panel",
        "child_link": "eef_mount_adapter",
        "joint_names": [
            "adapter_mount_tx", "adapter_mount_ty", "adapter_mount_tz",
            "adapter_mount_rx", "adapter_mount_ry", "adapter_mount_rz",
        ],
        "topic": "/adapter_mount_tune/joint_states",
        "yaml_file": "eef_mount_adapter.yaml.example",
        "xacro_file": "eef_mount_adapter_origin.xacro",
        "xacro_props": ("adapter_mount_parent", "adapter_mount_xyz", "adapter_mount_rpy"),
    },
    {
        "key": "camera",
        "title": "Layer 3: arm_realsense Connector -> Camera",
        "parent_link": "eef_mount_adapter",
        "child_link": "camera_link",
        "joint_names": [
            "cam_mount_tx", "cam_mount_ty", "cam_mount_tz",
            "cam_mount_rx", "cam_mount_ry", "cam_mount_rz",
        ],
        "topic": "/cam_mount_tune/joint_states",
        "yaml_file": "realsense_d435_mount.yaml.example",
        "xacro_file": "camera_mount_origin.xacro",
        "xacro_props": ("cam_mount_parent", "cam_mount_xyz", "cam_mount_rpy"),
    },
]

AXIS_LABELS = ["tx (m)", "ty (m)", "tz (m)", "rx (rad)", "ry (rad)", "rz (rad)"]

ALL_JOINT_NAMES = []
for ld in LAYER_DEFS:
    ALL_JOINT_NAMES.extend(ld["joint_names"])


def _pkg_config():
    return os.path.join(rospkg.RosPack().get_path("luggage_description"), "config")


def _rad_to_slider(rad, lower, upper):
    clamped = max(lower, min(upper, rad))
    return int(round((clamped - lower) / (upper - lower) * SLIDER_SCALE))


def _slider_to_rad(value, lower, upper):
    return lower + (float(value) / SLIDER_SCALE) * (upper - lower)


def _parse_joint_limits(joint_names):
    try:
        urdf_xml = rospy.get_param("/robot_description")
    except KeyError:
        return {n: (-DEFAULT_PRISMATIC_LIMIT if n.endswith(("tx", "ty", "tz")) else -math.pi,
                     DEFAULT_PRISMATIC_LIMIT if n.endswith(("tx", "ty", "tz")) else math.pi)
                for n in joint_names}
    root = ET.fromstring(urdf_xml)
    limits = {}
    for jn in joint_names:
        joint_el = root.find("joint[@name='%s']" % jn)
        default = DEFAULT_PRISMATIC_LIMIT if jn.endswith(("tx", "ty", "tz")) else math.pi
        if joint_el is None or joint_el.find("limit") is None:
            limits[jn] = (-default, default)
        else:
            lim = joint_el.find("limit")
            limits[jn] = (float(lim.get("lower", -default)), float(lim.get("upper", default)))
    return limits


def _load_layer_config(layer_def):
    config_dir = _pkg_config()
    yaml_path = os.path.join(config_dir, layer_def["yaml_file"])
    try:
        with open(yaml_path, "r") as h:
            data = yaml.safe_load(h) or {}
    except (IOError, OSError):
        return [0.0] * 6, yaml_path

    mount = data.get("mount", data)
    parent = mount.get("parent_link", layer_def["parent_link"])

    if "tune_joints" in mount:
        try:
            vals = mount_dict_to_tune_joints(mount)
            return vals, yaml_path
        except (KeyError, ValueError, TypeError):
            pass

    fixed = mount.get("fixed", {})
    xyz = [float(v) for v in fixed.get("xyz", [0, 0, 0])]
    rpy = [float(v) for v in fixed.get("rpy", [0, 0, 0])]
    return xyz + rpy, yaml_path


def _lookup_tf(parent, child, timeout_sec=2.0):
    listener = tf.TransformListener()
    deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
    while rospy.Time.now() < deadline and not rospy.is_shutdown():
        try:
            if listener.canTransform(parent, child, rospy.Time(0)):
                trans, rot = listener.lookupTransform(parent, child, rospy.Time(0))
                r, p, y = tft.euler_from_quaternion(rot, axes="sxyz")
                return [float(v) for v in trans], [float(r), float(p), float(y)]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        rospy.sleep(0.05)
    return None


def _write_xacro(xacro_path, parent_link, fixed_xyz, fixed_rpy, prop_parent, prop_xyz, prop_rpy, saved_by):
    body = (
        '<?xml version="1.0"?>\n'
        "<!-- Saved by %s -->\n"
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
        '  <xacro:property name="%s" value="%s"/>\n'
        '  <xacro:property name="%s" value="%.6f %.6f %.6f"/>\n'
        '  <xacro:property name="%s" value="%.8f %.8f %.8f"/>\n'
        "</robot>\n"
        % (
            saved_by,
            prop_parent, parent_link,
            prop_xyz, fixed_xyz[0], fixed_xyz[1], fixed_xyz[2],
            prop_rpy, fixed_rpy[0], fixed_rpy[1], fixed_rpy[2],
        )
    )
    with open(xacro_path, "w") as h:
        h.write(body)


def _write_yaml(yaml_path, parent_link, child_link, fixed_xyz, fixed_rpy):
    try:
        with open(yaml_path, "r") as h:
            data = yaml.safe_load(h) or {}
    except (IOError, OSError):
        data = {}
    data.setdefault("mount", {}).update({
        "parent_link": parent_link,
        "child_link": child_link,
        "fixed": {
            "xyz": [round(v, 6) for v in fixed_xyz],
            "rpy": [round(v, 8) for v in fixed_rpy],
        },
    })
    with open(yaml_path, "w") as h:
        yaml.safe_dump(data, h, default_flow_style=False, sort_keys=False)


def _sync_d435_yaml(parent_link, fixed_xyz, fixed_rpy, tune_joints):
    config_dir = _pkg_config()
    mount_body = build_mount_yaml(parent_link, tune_joints, fixed_xyz, fixed_rpy)
    for name in ("realsense_d435.yaml", "realsense_d435.yaml.example"):
        path = os.path.join(config_dir, name)
        if not os.path.isfile(path):
            continue
        try:
            with open(path, "r") as h:
                d435 = yaml.safe_load(h) or {}
            d435.setdefault("camera", {})["mount"] = {
                "parent_link": parent_link,
                "tune_joints": mount_body["mount"]["tune_joints"],
                "fixed": mount_body["mount"]["fixed"],
                "xyz": mount_body["mount"]["fixed"]["xyz"],
                "rpy": mount_body["mount"]["fixed"]["rpy"],
            }
            with open(path, "w") as h:
                yaml.safe_dump(d435, h, default_flow_style=False, sort_keys=False)
        except (IOError, OSError):
            pass


def _move_arm_via_controller(joint_names, joint_positions, duration_sec=1.5):
    action = rospy.get_param("~arm_controller_action", ARM_CONTROLLER_ACTION_DEFAULT)
    client = actionlib.SimpleActionClient(action, FollowJointTrajectoryAction)
    if not client.wait_for_server(rospy.Duration(5.0)):
        return False
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = list(joint_names)
    pt = JointTrajectoryPoint()
    pt.positions = list(joint_positions)
    pt.time_from_start = rospy.Duration(duration_sec)
    goal.trajectory.points = [pt]
    client.send_goal(goal)
    if not client.wait_for_result(rospy.Duration(duration_sec + 10.0)):
        client.cancel_goal()
        return False
    return client.get_state() == actionlib.GoalStatus.SUCCEEDED


class EofMountStackTuneWindow(QMainWindow):
    def __init__(self):
        super(EofMountStackTuneWindow, self).__init__()
        self.setWindowTitle("EOF Mount Stack Tune — 3-Layer TF Calibration")
        self.resize(1200, 800)

        self._model_name = rospy.get_param("~gazebo_model_name", "S20")
        self._live_debounce_ms = int(rospy.get_param("~live_debounce_ms", 50))
        self._bridge = CvBridge()
        self._latest_image = None
        self._updating_sliders = False

        self._limits = _parse_joint_limits(ALL_JOINT_NAMES)
        self._sliders = {}
        self._value_labels = {}
        self._layer_configs = {}

        self._joint_pubs = {}
        for ld in LAYER_DEFS:
            self._joint_pubs[ld["key"]] = rospy.Publisher(
                ld["topic"], JointState, queue_size=1, latch=True
            )

        self._image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self._on_image, queue_size=1
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
            rospy.logwarn("Gazebo set_model_configuration unavailable")

        self._live_timer = QTimer(self)
        self._live_timer.setSingleShot(True)
        self._live_timer.timeout.connect(self._apply_all_live)

        self._build_ui()
        self._load_all_from_yaml(apply_live=False)

        if self._set_model_config is not None:
            QTimer.singleShot(1500, self._apply_all_live)

        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._refresh_image)
        self._refresh_timer.start(66)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        controls = QVBoxLayout(scroll_widget)
        scroll.setWidget(scroll_widget)
        root.addWidget(scroll, stretch=3)

        for ld in LAYER_DEFS:
            box = QGroupBox(ld["title"])
            box_layout = QVBoxLayout(box)
            grid = QGridLayout()
            for row, (jn, label) in enumerate(zip(ld["joint_names"], AXIS_LABELS)):
                grid.addWidget(QLabel(label), row, 0)
                slider = QSlider(Qt.Horizontal)
                slider.setMinimum(0)
                slider.setMaximum(SLIDER_SCALE)
                slider.valueChanged.connect(self._make_slider_handler(jn))
                vlabel = QLabel("")
                grid.addWidget(slider, row, 1)
                grid.addWidget(vlabel, row, 2)
                self._sliders[jn] = slider
                self._value_labels[jn] = vlabel
            box_layout.addLayout(grid)

            btn_row = QHBoxLayout()
            reset_btn = QPushButton("Reset to YAML")
            reset_btn.clicked.connect(self._make_reset_handler(ld))
            zero_btn = QPushButton("Zero")
            zero_btn.clicked.connect(self._make_zero_handler(ld))
            save_btn = QPushButton("Save")
            save_btn.clicked.connect(self._make_save_handler(ld))
            save_btn.setToolTip("Save this layer's TF to xacro + YAML config")
            btn_row.addWidget(reset_btn)
            btn_row.addWidget(zero_btn)
            btn_row.addWidget(save_btn)
            box_layout.addLayout(btn_row)
            controls.addWidget(box)

        global_row = QHBoxLayout()
        observe_btn = QPushButton("Snap to Observe Pose")
        observe_btn.clicked.connect(self._move_to_observe)
        save_all_btn = QPushButton("Save All Layers")
        save_all_btn.clicked.connect(self._save_all)
        reset_all_btn = QPushButton("Reset All from YAML")
        reset_all_btn.clicked.connect(lambda: self._load_all_from_yaml(apply_live=True))
        global_row.addWidget(observe_btn)
        global_row.addWidget(reset_all_btn)
        global_row.addWidget(save_all_btn)
        controls.addLayout(global_row)

        self._status_label = QLabel("Status: Ready — drag sliders to calibrate mount transforms.")
        self._status_label.setWordWrap(True)
        controls.addWidget(self._status_label)
        controls.addStretch(1)

        preview_box = QGroupBox("RGB preview (/camera/color/image_raw)")
        preview_layout = QVBoxLayout(preview_box)
        self._image_label = QLabel("Waiting for camera ...")
        self._image_label.setAlignment(Qt.AlignCenter)
        self._image_label.setMinimumSize(480, 360)
        self._image_label.setStyleSheet("background-color: #222; color: #ccc;")
        preview_layout.addWidget(self._image_label)
        root.addWidget(preview_box, stretch=2)

    def _format_value(self, jn, val):
        if jn.endswith(("tx", "ty", "tz")):
            return "%.4f m" % val
        return "%.4f rad (%.1f deg)" % (val, math.degrees(val))

    def _make_slider_handler(self, jn):
        def handler(_):
            if self._updating_sliders:
                return
            val = _slider_to_rad(self._sliders[jn].value(), *self._limits[jn])
            self._value_labels[jn].setText(self._format_value(jn, val))
            self._schedule_live()
        return handler

    def _set_slider(self, jn, val):
        self._updating_sliders = True
        self._sliders[jn].setValue(_rad_to_slider(val, *self._limits[jn]))
        self._value_labels[jn].setText(self._format_value(jn, val))
        self._updating_sliders = False

    def _get_layer_values(self, ld):
        return [_slider_to_rad(self._sliders[jn].value(), *self._limits[jn])
                for jn in ld["joint_names"]]

    def _load_all_from_yaml(self, apply_live=True):
        for ld in LAYER_DEFS:
            vals, yaml_path = _load_layer_config(ld)
            self._layer_configs[ld["key"]] = {"yaml_path": yaml_path}
            for jn, v in zip(ld["joint_names"], vals):
                self._set_slider(jn, v)
        if apply_live:
            self._apply_all_live()
        self._status_label.setText("Status: Loaded all layers from YAML")

    def _make_reset_handler(self, ld):
        def handler():
            vals, yaml_path = _load_layer_config(ld)
            self._layer_configs[ld["key"]] = {"yaml_path": yaml_path}
            for jn, v in zip(ld["joint_names"], vals):
                self._set_slider(jn, v)
            self._apply_all_live()
            self._status_label.setText("Status: Reset %s from YAML" % ld["title"])
        return handler

    def _make_zero_handler(self, ld):
        def handler():
            for jn in ld["joint_names"]:
                self._set_slider(jn, 0.0)
            self._apply_all_live()
            self._status_label.setText("Status: Zeroed %s" % ld["title"])
        return handler

    def _schedule_live(self):
        if self._set_model_config is not None:
            self._live_timer.start(self._live_debounce_ms)

    def _apply_all_live(self):
        if self._set_model_config is None:
            return
        all_names = []
        all_values = []
        for ld in LAYER_DEFS:
            vals = self._get_layer_values(ld)
            all_names.extend(ld["joint_names"])
            all_values.extend(vals)
        try:
            resp = self._set_model_config(
                model_name=self._model_name,
                urdf_param_name="robot_description",
                joint_names=all_names,
                joint_positions=all_values,
            )
            if resp.success:
                for ld in LAYER_DEFS:
                    vals = self._get_layer_values(ld)
                    msg = JointState()
                    msg.header.stamp = rospy.Time.now()
                    msg.name = list(ld["joint_names"])
                    msg.position = vals
                    self._joint_pubs[ld["key"]].publish(msg)
                self._status_label.setText("Status: Live preview updated (18 joints)")
            else:
                self._status_label.setText("Status: Gazebo rejected — %s" % resp.status_message)
        except rospy.ServiceException as exc:
            self._status_label.setText("Status: Live preview failed — %s" % exc)

    def _save_layer(self, ld):
        parent = ld["parent_link"]
        child = ld["child_link"]
        tf_result = _lookup_tf(parent, child)
        if tf_result is None:
            return False, "TF %s -> %s unavailable" % (parent, child)

        fixed_xyz, fixed_rpy = tf_result
        config_dir = _pkg_config()
        yaml_path = os.path.join(config_dir, ld["yaml_file"])
        xacro_path = os.path.join(config_dir, ld["xacro_file"])
        prop_parent, prop_xyz, prop_rpy = ld["xacro_props"]

        _write_xacro(xacro_path, parent, fixed_xyz, fixed_rpy,
                      prop_parent, prop_xyz, prop_rpy,
                      "eof_mount_stack_tune_gui.py")
        _write_yaml(yaml_path, parent, child, fixed_xyz, fixed_rpy)

        if ld["key"] == "camera":
            tune_vals = self._get_layer_values(ld)
            _sync_d435_yaml(parent, fixed_xyz, fixed_rpy, tune_vals)

        return True, "Saved %s -> %s" % (parent, child)

    def _make_save_handler(self, ld):
        def handler():
            ok, msg = self._save_layer(ld)
            if ok:
                self._status_label.setText("Status: %s — restart sim to apply" % msg)
            else:
                QMessageBox.warning(self, "Save failed", msg)
                self._status_label.setText("Status: Save failed — %s" % msg)
        return handler

    def _save_all(self):
        results = []
        for ld in LAYER_DEFS:
            ok, msg = self._save_layer(ld)
            results.append((ld["title"], ok, msg))
        failed = [r for r in results if not r[1]]
        if failed:
            msgs = "\n".join("%s: %s" % (t, m) for t, _, m in failed)
            QMessageBox.warning(self, "Save All — partial failure", msgs)
            self._status_label.setText("Status: Save All partial failure")
        else:
            self._status_label.setText(
                "Status: All 3 layers saved — restart sim to apply permanently"
            )

    def _move_to_observe(self):
        try:
            path = rospy.get_param(
                "~robot_poses_config",
                os.path.join(_pkg_config(), "robot_poses.yaml.example"),
            )
            with open(path, "r") as h:
                config = yaml.safe_load(h)
            pose = config["poses"]["observe"]
            joints = list(pose["joints"])
            values = [float(v) for v in pose["values"]]
            if _move_arm_via_controller(joints, values):
                self._status_label.setText("Status: At observe pose — tune mounts here")
            else:
                self._status_label.setText("Status: Could not reach observe pose")
        except Exception as exc:
            self._status_label.setText("Status: Observe pose failed — %s" % exc)
        self._apply_all_live()

    def _on_image(self, msg):
        try:
            self._latest_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            pass

    def _refresh_image(self):
        if self._latest_image is None:
            return
        rgb = self._latest_image[:, :, ::-1].copy()
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg)
        self._image_label.setPixmap(
            pix.scaled(self._image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        )


def main():
    rospy.init_node("eof_mount_stack_tune_gui")
    if os.environ.get("LIBGL_ALWAYS_SOFTWARE", "0") == "1":
        QApplication.setAttribute(Qt.AA_UseSoftwareOpenGL, True)
    app = QApplication(sys.argv)
    window = EofMountStackTuneWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
