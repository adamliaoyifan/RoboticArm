#!/usr/bin/env python3
"""PyQt GUI to tune observe pose joints with live RGB camera preview."""

import math
import os
import sys
import threading
import xml.etree.ElementTree as ET

import rospy
import rospkg
import yaml
from cv_bridge import CvBridge
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
from gazebo_msgs.srv import SetModelConfiguration
from sensor_msgs.msg import Image, JointState

from luggage_msgs.srv import GoToJointValues

JOINT_NAMES = [
    "elfin_joint1",
    "elfin_joint2",
    "elfin_joint3",
    "elfin_joint4",
    "elfin_joint5",
    "elfin_joint6",
]

SLIDER_SCALE = 1000


def _default_poses_path():
    return os.path.join(
        rospkg.RosPack().get_path("luggage_description"),
        "config",
        "robot_poses.yaml.example",
    )


def parse_joint_limits(joint_names):
    """Read lower/upper limits from /robot_description."""
    urdf_xml = rospy.get_param("/robot_description")
    root = ET.fromstring(urdf_xml)
    limits = {}
    for joint_name in joint_names:
        joint_el = root.find("joint[@name='%s']" % joint_name)
        if joint_el is None:
            limits[joint_name] = (-math.pi, math.pi)
            continue
        limit_el = joint_el.find("limit")
        if limit_el is None:
            limits[joint_name] = (-math.pi, math.pi)
            continue
        limits[joint_name] = (
            float(limit_el.get("lower", -math.pi)),
            float(limit_el.get("upper", math.pi)),
        )
    return limits


def rad_to_slider(rad, lower, upper):
    clamped = max(lower, min(upper, rad))
    return int(round((clamped - lower) / (upper - lower) * SLIDER_SCALE))


def slider_to_rad(value, lower, upper):
    return lower + (float(value) / SLIDER_SCALE) * (upper - lower)


def format_values_yaml(values):
    formatted = ", ".join("%.4f" % v for v in values)
    return "    values: [%s]" % formatted


class PoseTuneWindow(QMainWindow):
    def __init__(self):
        super(PoseTuneWindow, self).__init__()
        self.setWindowTitle("Observe Pose Tune")
        self.resize(1100, 680)

        self._limits = parse_joint_limits(JOINT_NAMES)
        self._sliders = {}
        self._value_labels = {}
        self._updating_sliders = False
        self._latest_image = None
        self._image_lock = threading.Lock()
        self._bridge = CvBridge()
        self._moving = False
        self._gazebo_model = rospy.get_param("~gazebo_model_name", "S20")
        self._live_debounce_ms = int(rospy.get_param("~live_debounce_ms", 40))

        self._joint_states = {}
        self._joint_sub = rospy.Subscriber("/joint_states", JointState, self._on_joint_states, queue_size=1)
        self._image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self._on_image, queue_size=1
        )

        service_name = rospy.get_param("~go_to_joint_values_service", "/motion_planner/go_to_joint_values")
        rospy.loginfo("Waiting for %s ...", service_name)
        rospy.wait_for_service(service_name, timeout=120.0)
        self._go_to_joints = rospy.ServiceProxy(service_name, GoToJointValues)

        self._set_model_config = None
        self._init_gazebo_live_preview()

        self._live_timer = QTimer(self)
        self._live_timer.setSingleShot(True)
        self._live_timer.timeout.connect(self._apply_live_joints)

        self._build_ui()
        self._load_observe_yaml(apply_live=True)

        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._refresh_image)
        self._refresh_timer.start(66)

    def _init_gazebo_live_preview(self):
        gazebo_srv = rospy.get_param(
            "~gazebo_set_model_configuration_service", "/gazebo/set_model_configuration"
        )
        try:
            rospy.wait_for_service(gazebo_srv, timeout=30.0)
            self._set_model_config = rospy.ServiceProxy(gazebo_srv, SetModelConfiguration)
            rospy.loginfo("Live Gazebo preview enabled via %s", gazebo_srv)
        except rospy.ROSException:
            self._set_model_config = None
            rospy.logwarn("Gazebo set_model_configuration unavailable — sliders won't move sim")

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)

        controls = QVBoxLayout()
        root.addLayout(controls, stretch=2)

        sliders_box = QGroupBox("Joint angles")
        sliders_layout = QGridLayout(sliders_box)
        for row, joint_name in enumerate(JOINT_NAMES):
            lower, upper = self._limits[joint_name]
            label = QLabel(joint_name)
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(SLIDER_SCALE)
            slider.valueChanged.connect(self._make_slider_handler(joint_name))
            value_label = QLabel(self._format_joint_text(joint_name, 0.0))
            sliders_layout.addWidget(label, row, 0)
            sliders_layout.addWidget(slider, row, 1)
            sliders_layout.addWidget(value_label, row, 2)
            self._sliders[joint_name] = slider
            self._value_labels[joint_name] = value_label
            self._set_slider_value(joint_name, 0.0, apply_live=False)
        controls.addWidget(sliders_box)

        self._live_checkbox = QCheckBox("Live preview in Gazebo (drag sliders — bypasses MoveIt IK)")
        self._live_checkbox.setChecked(False)
        self._live_checkbox.setEnabled(self._set_model_config is not None)
        if self._set_model_config is None:
            self._live_checkbox.setToolTip("Gazebo service not available")
        controls.addWidget(self._live_checkbox)

        btn_row1 = QHBoxLayout()
        read_btn = QPushButton("Read Joints (after MoveIt Execute)")
        read_btn.clicked.connect(self._read_current)
        read_btn.setToolTip("Read /joint_states into sliders after you Plan & Execute in RViz")
        load_btn = QPushButton("Load Observe YAML")
        load_btn.clicked.connect(lambda: self._load_observe_yaml(apply_live=True))
        home_btn = QPushButton("Home")
        home_btn.clicked.connect(self._set_home)
        btn_row1.addWidget(read_btn)
        btn_row1.addWidget(load_btn)
        btn_row1.addWidget(home_btn)
        controls.addLayout(btn_row1)

        self._execute_btn = QPushButton("Execute Move (MoveIt)")
        self._execute_btn.clicked.connect(self._execute_move)
        self._execute_btn.setToolTip("Collision-aware planned move; use after live tuning")
        controls.addWidget(self._execute_btn)

        export_box = QGroupBox("Export")
        export_layout = QVBoxLayout(export_box)
        self._export_label = QLabel("")
        self._export_label.setWordWrap(True)
        self._export_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        export_layout.addWidget(self._export_label)
        export_btns = QHBoxLayout()
        copy_btn = QPushButton("Copy")
        copy_btn.clicked.connect(self._copy_export)
        save_btn = QPushButton("Save")
        save_btn.clicked.connect(self._save_export)
        export_btns.addWidget(copy_btn)
        export_btns.addWidget(save_btn)
        export_layout.addLayout(export_btns)
        controls.addWidget(export_box)

        self._status_label = QLabel(
            "Status: Use RViz MoveIt marker → Plan → Execute, then Read Joints here"
        )
        controls.addWidget(self._status_label)
        controls.addStretch(1)

        preview_box = QGroupBox("RGB Camera Preview (/camera/color/image_raw)")
        preview_layout = QVBoxLayout(preview_box)
        self._image_label = QLabel("Waiting for camera ...")
        self._image_label.setAlignment(Qt.AlignCenter)
        self._image_label.setMinimumSize(640, 480)
        self._image_label.setStyleSheet("background-color: #222; color: #ccc;")
        preview_layout.addWidget(self._image_label)
        root.addWidget(preview_box, stretch=3)

        self._update_export_text()

    def _make_slider_handler(self, joint_name):
        def handler(value):
            if self._updating_sliders:
                return
            rad = slider_to_rad(value, *self._limits[joint_name])
            self._value_labels[joint_name].setText(self._format_joint_text(joint_name, rad))
            self._update_export_text()
            self._schedule_live_apply()

        return handler

    @staticmethod
    def _format_joint_text(joint_name, rad):
        return "%.4f rad (%.1f deg)" % (rad, math.degrees(rad))

    def _set_slider_value(self, joint_name, rad, apply_live=True):
        lower, upper = self._limits[joint_name]
        self._updating_sliders = True
        self._sliders[joint_name].setValue(rad_to_slider(rad, lower, upper))
        self._value_labels[joint_name].setText(self._format_joint_text(joint_name, rad))
        self._updating_sliders = False
        if apply_live:
            self._schedule_live_apply()

    def _current_values(self):
        values = []
        for joint_name in JOINT_NAMES:
            slider_val = self._sliders[joint_name].value()
            values.append(slider_to_rad(slider_val, *self._limits[joint_name]))
        return values

    def _schedule_live_apply(self):
        if not self._live_checkbox.isChecked() or self._set_model_config is None:
            return
        self._live_timer.start(self._live_debounce_ms)

    def _apply_live_joints(self):
        if not self._live_checkbox.isChecked() or self._set_model_config is None:
            return
        values = self._current_values()
        try:
            resp = self._set_model_config(
                model_name=self._gazebo_model,
                urdf_param_name="robot_description",
                joint_names=JOINT_NAMES,
                joint_positions=values,
            )
            if resp.success:
                self._status_label.setText("Status: Live preview updated")
            else:
                self._status_label.setText("Status: Gazebo rejected pose — %s" % resp.status_message)
        except rospy.ServiceException as exc:
            self._status_label.setText("Status: Live preview failed — %s" % exc)

    def _update_export_text(self):
        values = self._current_values()
        snippet = format_values_yaml(values)
        self._export_label.setText(
            "# Paste into robot_poses.yaml under poses.observe:\n%s" % snippet
        )

    def _on_joint_states(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self._joint_states[name] = pos

    def _on_image(self, msg):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "cv_bridge error: %s", exc)
            return
        with self._image_lock:
            self._latest_image = cv_image

    def _refresh_image(self):
        with self._image_lock:
            cv_image = None if self._latest_image is None else self._latest_image.copy()
        if cv_image is None:
            return
        rgb = cv_image[:, :, ::-1].copy()
        height, width, channel = rgb.shape
        bytes_per_line = channel * width
        qimage = QImage(rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimage)
        self._image_label.setPixmap(
            pixmap.scaled(self._image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        )

    def _read_current(self):
        if not self._joint_states:
            self._status_label.setText("Status: No /joint_states yet")
            return
        for joint_name in JOINT_NAMES:
            if joint_name not in self._joint_states:
                self._status_label.setText("Status: Missing %s in joint_states" % joint_name)
                return
            self._set_slider_value(joint_name, self._joint_states[joint_name], apply_live=False)
        self._update_export_text()
        self._status_label.setText("Status: Loaded current joint states")

    def _load_observe_yaml(self, apply_live=True):
        path = rospy.get_param("~robot_poses_config", _default_poses_path())
        try:
            with open(path, "r") as handle:
                config = yaml.safe_load(handle)
            values = config["poses"]["observe"]["values"]
        except Exception as exc:
            self._status_label.setText("Status: Failed to load YAML: %s" % exc)
            return
        if len(values) != len(JOINT_NAMES):
            self._status_label.setText("Status: observe.values length mismatch")
            return
        self._updating_sliders = True
        for joint_name, value in zip(JOINT_NAMES, values):
            lower, upper = self._limits[joint_name]
            rad = float(value)
            self._sliders[joint_name].setValue(rad_to_slider(rad, lower, upper))
            self._value_labels[joint_name].setText(self._format_joint_text(joint_name, rad))
        self._updating_sliders = False
        self._update_export_text()
        if apply_live:
            self._apply_live_joints()
        self._status_label.setText("Status: Loaded observe pose from YAML")

    def _set_home(self):
        self._updating_sliders = True
        for joint_name in JOINT_NAMES:
            lower, upper = self._limits[joint_name]
            self._sliders[joint_name].setValue(rad_to_slider(0.0, lower, upper))
            self._value_labels[joint_name].setText(self._format_joint_text(joint_name, 0.0))
        self._updating_sliders = False
        self._update_export_text()
        self._apply_live_joints()
        self._status_label.setText("Status: Sliders set to home (0)")

    def _execute_move(self):
        if self._moving:
            return
        values = self._current_values()
        self._moving = True
        self._execute_btn.setEnabled(False)
        self._status_label.setText("Status: MoveIt planning ...")

        def worker():
            try:
                resp = self._go_to_joints(values, JOINT_NAMES)
                status = resp.message
                if resp.success and resp.already_there:
                    status = "Already at target"
            except rospy.ServiceException as exc:
                status = "Service call failed: %s" % exc
                resp = None
            self._finish_move(status, resp.success if resp else False)

        threading.Thread(target=worker, daemon=True).start()

    def _finish_move(self, status, success):
        def update_ui():
            self._moving = False
            self._execute_btn.setEnabled(True)
            prefix = "Status: OK — " if success else "Status: Failed — "
            self._status_label.setText(prefix + status)
            self._read_current()

        QTimer.singleShot(0, update_ui)

    def _copy_export(self):
        clipboard = QApplication.clipboard()
        clipboard.setText(format_values_yaml(self._current_values()), QClipboard.Clipboard)
        self._status_label.setText("Status: Copied values line to clipboard")

    def _save_export(self):
        values = self._current_values()
        path = os.path.expanduser("~/observe_pose_tuned.yaml")
        content = (
            "# Tuned observe pose — paste values into robot_poses.yaml\n"
            "joints:\n"
            + "\n".join("  - %s" % n for n in JOINT_NAMES)
            + "\n"
            + format_values_yaml(values)
            + "\n"
        )
        try:
            with open(path, "w") as handle:
                handle.write(content)
        except OSError as exc:
            QMessageBox.warning(self, "Save failed", str(exc))
            return
        self._status_label.setText("Status: Saved to %s" % path)


def main():
    rospy.init_node("pose_tune_gui")
    app = QApplication(sys.argv)
    window = PoseTuneWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
