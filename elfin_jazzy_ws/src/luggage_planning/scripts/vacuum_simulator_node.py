#!/usr/bin/env python3
"""Vacuum pickup simulation: logical stub or Gazebo kinematic follow attach."""

from __future__ import division

import math

import rospy
import rospkg
import tf2_ros
import os 
import sys
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Pose, Twist
from std_srvs.srv import Trigger, TriggerRequest

from luggage_msgs.srv import VacuumCommand, VacuumCommandResponse

PLANNING_ROOT = rospkg.RosPack().get_path("luggage_planning")
_scripts = os.path.join(PLANNING_ROOT, "scripts")
if _scripts in sys.path:
    sys.path.remove(_scripts)
sys.path.insert(0, _scripts)

from vacuum_attach_utils import (  # noqa: E402
    compose_transform,
    contact_ok,
    invert_transform,
    lists_to_pose,
    pose_to_lists,
)
from vacuum_retention import retention_metrics  # noqa: E402


class VacuumSimulator:
    def __init__(self):
        self._attached = False
        self._mode = rospy.get_param("~mode", "stub")
        self._robot_frame = rospy.get_param("~robot_frame", "elfin_base_link")
        self._robot_link = rospy.get_param("~robot_link", "suction_panel")
        self._suction_frame = rospy.get_param(
            "~suction_frame", "suction_contact_frame")
        self._box_link = rospy.get_param("~box_link", "suitcase_link")
        self._world_frame = rospy.get_param("~world_frame", "world")
        self._box_param = rospy.get_param("~box_param", "/luggage/current_box")
        self._attached_param = rospy.get_param("~attached_param", "/luggage/vacuum/attached")
        self._contact_margin = float(rospy.get_param("~contact_margin", 0.08))
        self._follow_rate = float(rospy.get_param("~follow_rate", 50.0))
        self._pressure_kpa = float(rospy.get_param(
            "~retention/pressure_kpa", 70.0))
        self._effective_area_m2 = float(rospy.get_param(
            "~retention/effective_area_m2", 0.012))
        self._seal_efficiency = float(rospy.get_param(
            "~retention/seal_efficiency", 0.80))
        self._friction_coefficient = float(rospy.get_param(
            "~retention/friction_coefficient", 0.60))
        self._minimum_margin = float(rospy.get_param(
            "~retention/minimum_retention_margin", 2.0))
        self._max_suction_tilt_deg = float(rospy.get_param(
            "~retention/max_suction_tilt_deg", 5.0))
        self._max_linear_accel = float(rospy.get_param(
            "~retention/max_linear_accel_mps2", 2.0))
        self._max_angular_accel = float(rospy.get_param(
            "~retention/max_angular_accel_radps2", 1.0))
        self._pressure_ready_kpa = float(rospy.get_param(
            "~retention/pressure_ready_threshold_kpa", 60.0))
        self._rated_payload_kg = float(rospy.get_param(
            "~retention/rated_payload_kg", 23.0))
        # Upper-bound factor on the payload mass used for the retention gate;
        # covers density jitter and size estimation error.
        self._mass_safety_factor = float(rospy.get_param(
            "~retention/mass_safety_factor", 1.25))
        self._attach_scene_service = rospy.get_param(
            "~attach_scene_service", "/scene_manager/attach_pickup_box"
        )
        self._detach_scene_service = rospy.get_param(
            "~detach_scene_service", "/scene_manager/detach_pickup_box"
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._follow_timer = None
        self._box_model = None
        self._box_size = None
        self._offset_in_panel = None
        self._box_mass_kg = None
        rospy.set_param("/luggage/vacuum/events", [])
        rospy.set_param("/luggage/vacuum/retention_fault", "")

        self._get_model_state = None
        self._set_model_state = None
        if self._mode == "gazebo_follow":
            rospy.wait_for_service("/gazebo/get_model_state", timeout=30.0)
            rospy.wait_for_service("/gazebo/set_model_state", timeout=30.0)
            self._get_model_state = rospy.ServiceProxy(
                "/gazebo/get_model_state", GetModelState
            )
            self._set_model_state = rospy.ServiceProxy(
                "/gazebo/set_model_state", SetModelState
            )

        self._publish_attached(False)

    def _scene_proxy(self, service_name, cached_attr):
        proxy = getattr(self, cached_attr, None)
        if proxy is not None:
            return proxy
        rospy.wait_for_service(service_name, timeout=5.0)
        proxy = rospy.ServiceProxy(service_name, Trigger)
        setattr(self, cached_attr, proxy)
        return proxy

    def _publish_attached(self, attached):
        self._attached = bool(attached)
        rospy.set_param(self._attached_param, self._attached)

    def _current_box_info(self):
        data = rospy.get_param(self._box_param, {})
        if not data:
            return None, None
        model_name = data.get("id", "")
        pose_data = data.get("pose", {})
        position = pose_data.get("position", {})
        orientation = pose_data.get("orientation", {})
        size = [
            float(data.get("width", 0.0)),
            float(data.get("depth", 0.0)),
            float(data.get("height", 0.0)),
        ]
        pose = Pose()
        pose.position.x = float(position.get("x", 0.0))
        pose.position.y = float(position.get("y", 0.0))
        pose.position.z = float(position.get("z", 0.0))
        pose.orientation.x = float(orientation.get("x", 0.0))
        pose.orientation.y = float(orientation.get("y", 0.0))
        pose.orientation.z = float(orientation.get("z", 0.0))
        pose.orientation.w = float(orientation.get("w", 1.0))
        return model_name, {
            "pose": pose,
            "size": size,
            "yaw": float(data.get("yaw", 0.0)),
            "mass_kg": float(data.get("mass_kg", 0.0)),
        }

    def _lookup_panel_in_world(self):
        transform = self._tf_buffer.lookup_transform(
            self._world_frame,
            self._robot_link,
            rospy.Time(0),
            rospy.Duration(0.5),
        )
        return pose_to_lists(transform.transform.translation, transform.transform.rotation)

    def _lookup_suction_quaternion_world(self):
        transform = self._tf_buffer.lookup_transform(
            self._world_frame,
            self._suction_frame,
            rospy.Time(0),
            rospy.Duration(0.5),
        )
        _translation, quaternion = pose_to_lists(
            transform.transform.translation,
            transform.transform.rotation)
        return quaternion

    def _lookup_panel_in_box(self, box_pose_world):
        panel_t, panel_q = self._lookup_panel_in_world()
        box_t, box_q = pose_to_lists(box_pose_world.position, box_pose_world.orientation)
        box_inv_t, box_inv_q = invert_transform(box_t, box_q)
        return compose_transform(box_inv_t, box_inv_q, panel_t, panel_q)

    def _get_box_pose_world(self, model_name):
        resp = self._get_model_state(model_name, self._world_frame)
        if not resp.success:
            raise RuntimeError("get_model_state failed for %s: %s" % (model_name, resp.status_message))
        return resp.pose

    def _verify_contact(self, model_name, box_size):
        panel_t, _panel_q = self._lookup_panel_in_world()
        box_pose = self._get_box_pose_world(model_name)
        box_t, _box_q = pose_to_lists(box_pose.position, box_pose.orientation)
        if not contact_ok(panel_t, box_t, box_size, self._contact_margin):
            dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(panel_t, box_t)))
            raise RuntimeError(
                "suction panel not in contact with %s (distance=%.3fm, margin=%.3fm)"
                % (model_name, dist, self._contact_margin)
            )

    def _update_box_param_pose(self, pose, yaw=None):
        data = rospy.get_param(self._box_param, {})
        if not data:
            return
        data["pose"] = {
            "position": {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
            },
            "orientation": {
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w,
            },
        }
        if yaw is not None:
            data["yaw"] = float(yaw)
        rospy.set_param(self._box_param, data)

    @staticmethod
    def _suction_tilt_deg(quaternion):
        x, y, z, w = quaternion
        panel_z_world_z = 1.0 - 2.0 * (x * x + y * y)
        cosine = max(-1.0, min(1.0, -panel_z_world_z))
        return math.degrees(math.acos(cosine))

    def _planning_mass_kg(self, box_info):
        """Mass the retention gate should be evaluated against.

        In simulation the exact mass is on the parameter, but the robot does not
        know a bag's density; sizes are now sampled continuously and density is
        jittered, so the safe number is an upper bound rather than the nominal
        one. ``mass_safety_factor`` covers the density jitter plus the size
        estimation error, capped at the rated payload.

        On real hardware this is where a check-in weight would be used instead,
        since airline baggage is weighed before it reaches the loader.
        """
        mass_kg = float(box_info.get("mass_kg", 0.0))
        if mass_kg <= 0.0:
            return mass_kg
        return min(
            self._rated_payload_kg, mass_kg * self._mass_safety_factor)

    def _retention_for(self, panel_q, mass_kg, box_size):
        tilt = self._suction_tilt_deg(panel_q)
        radius = 0.5 * math.hypot(box_size[0], box_size[1])
        return retention_metrics(
            mass_kg=mass_kg,
            tilt_deg=tilt,
            pressure_kpa=self._pressure_kpa,
            effective_area_m2=self._effective_area_m2,
            seal_efficiency=self._seal_efficiency,
            friction_coefficient=self._friction_coefficient,
            linear_accel_mps2=self._max_linear_accel,
            angular_accel_radps2=self._max_angular_accel,
            payload_radius_m=radius,
        )

    def _record_event(self, event, metrics=None, message=""):
        events = rospy.get_param("/luggage/vacuum/events", [])
        record = {
            "stamp": rospy.Time.now().to_sec(),
            "event": event,
            "model": self._box_model or "",
            "message": message,
        }
        if metrics is not None:
            record["retention"] = metrics
            rospy.set_param("/luggage/vacuum/retention", metrics)
        events.append(record)
        rospy.set_param("/luggage/vacuum/events", events[-100:])

    def _simulate_detach(self, reason, metrics):
        rospy.logerr("Vacuum retention fault: %s", reason)
        self._record_event("RETENTION_FAULT", metrics, reason)
        rospy.set_param("/luggage/vacuum/retention_fault", reason)
        self._stop_follow()
        try:
            response = self._scene_proxy(
                self._detach_scene_service, "_detach_scene")(
                    TriggerRequest())
            if not response.success:
                rospy.logwarn("MoveIt detach after retention fault: %s",
                              response.message)
        except rospy.ServiceException as exc:
            rospy.logwarn("MoveIt fault detach service failed: %s", exc)
        self._publish_attached(False)
        self._box_model = None
        self._box_size = None
        self._box_mass_kg = None
        self._offset_in_panel = None

    def _follow_tick(self, _event):
        if not self._attached or not self._box_model or self._offset_in_panel is None:
            return
        try:
            panel_t, panel_q = self._lookup_panel_in_world()
            metrics = self._retention_for(
                self._lookup_suction_quaternion_world(),
                self._box_mass_kg, self._box_size)
            if (
                    metrics["tilt_deg"] > self._max_suction_tilt_deg
                    or metrics["margin"] < self._minimum_margin):
                self._simulate_detach(
                    "tilt=%.2fdeg margin=%.2f" % (
                        metrics["tilt_deg"], metrics["margin"]),
                    metrics)
                return
            rospy.set_param("/luggage/vacuum/retention", metrics)
            box_t, box_q = compose_transform(panel_t, panel_q, self._offset_in_panel[0], self._offset_in_panel[1])
            pose = lists_to_pose(box_t, box_q)
            state = ModelState()
            state.model_name = self._box_model
            state.pose = pose
            state.twist = Twist()
            state.reference_frame = self._world_frame
            resp = self._set_model_state(state)
            if not resp.success:
                rospy.logwarn_throttle(2.0, "set_model_state failed: %s", resp.status_message)
                return
            self._update_box_param_pose(pose)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as exc:
            rospy.logwarn_throttle(2.0, "vacuum follow TF failed: %s", exc)

    def _start_follow(self):
        if self._follow_timer is not None:
            self._follow_timer.shutdown()
        period = 1.0 / max(self._follow_rate, 1.0)
        self._follow_timer = rospy.Timer(rospy.Duration(period), self._follow_tick)

    def _stop_follow(self):
        if self._follow_timer is not None:
            self._follow_timer.shutdown()
            self._follow_timer = None

    def _attach_gazebo(self):
        model_name, box_info = self._current_box_info()
        if not model_name or box_info is None:
            raise RuntimeError("no current pickup box to attach")

        self._verify_contact(model_name, box_info["size"])
        box_pose = self._get_box_pose_world(model_name)
        panel_t, panel_q = self._lookup_panel_in_world()
        mass_kg = self._planning_mass_kg(box_info)
        if mass_kg <= 0.0:
            raise RuntimeError("payload mass_kg is missing or non-positive")
        if mass_kg > self._rated_payload_kg:
            raise RuntimeError(
                "payload %.2fkg exceeds rated %.2fkg"
                % (mass_kg, self._rated_payload_kg))
        if self._pressure_kpa < self._pressure_ready_kpa:
            raise RuntimeError(
                "vacuum pressure %.1fkPa below ready threshold %.1fkPa"
                % (self._pressure_kpa, self._pressure_ready_kpa))
        metrics = self._retention_for(
            self._lookup_suction_quaternion_world(),
            mass_kg, box_info["size"])
        if (
                metrics["tilt_deg"] > self._max_suction_tilt_deg
                or metrics["margin"] < self._minimum_margin):
            raise RuntimeError(
                "retention gate failed tilt=%.2fdeg margin=%.2f"
                % (metrics["tilt_deg"], metrics["margin"]))
        box_t, box_q = pose_to_lists(box_pose.position, box_pose.orientation)
        panel_inv_t, panel_inv_q = invert_transform(panel_t, panel_q)
        offset_t, offset_q = compose_transform(panel_inv_t, panel_inv_q, box_t, box_q)

        scene_resp = self._scene_proxy(self._attach_scene_service, "_attach_scene")(
            TriggerRequest()
        )
        if not scene_resp.success:
            raise RuntimeError("MoveIt attach failed: %s" % scene_resp.message)

        self._box_model = model_name
        self._box_size = box_info["size"]
        self._box_mass_kg = mass_kg
        self._offset_in_panel = (offset_t, offset_q)
        self._start_follow()
        self._follow_tick(None)
        self._publish_attached(True)
        rospy.set_param("/luggage/vacuum/retention_fault", "")
        self._record_event("ATTACHED", metrics)
        rospy.loginfo(
            "VacuumSimulator attached %s to %s::%s (gazebo_follow)",
            model_name,
            self._robot_frame,
            self._robot_link,
        )

    def _detach_gazebo(self):
        self._stop_follow()
        self._record_event("DETACHED")
        if self._mode == "gazebo_follow":
            try:
                scene_resp = self._scene_proxy(
                    self._detach_scene_service, "_detach_scene"
                )(TriggerRequest())
                if not scene_resp.success:
                    rospy.logwarn("MoveIt detach failed: %s", scene_resp.message)
            except rospy.ServiceException as exc:
                rospy.logwarn("MoveIt detach service failed: %s", exc)
        self._box_model = None
        self._box_size = None
        self._box_mass_kg = None
        self._offset_in_panel = None
        self._publish_attached(False)
        rospy.loginfo("VacuumSimulator detached pickup box (gazebo_follow)")

    def handle(self, req):
        self._record_event("REQUEST_ON" if req.enable else "REQUEST_OFF")
        if req.enable:
            if self._attached:
                return VacuumCommandResponse(success=True, message="vacuum already ON")
            try:
                if self._mode == "gazebo_follow":
                    self._attach_gazebo()
                else:
                    self._publish_attached(True)
                    rospy.loginfo(
                        "VacuumSimulator stub attach (robot=%s::%s box_link=%s)",
                        self._robot_frame,
                        self._robot_link,
                        self._box_link,
                    )
                return VacuumCommandResponse(success=True, message="vacuum ON")
            except Exception as exc:
                rospy.logerr("Vacuum attach failed: %s", exc)
                self._record_event("ATTACH_FAILED", message=str(exc))
                self._publish_attached(False)
                return VacuumCommandResponse(success=False, message=str(exc))

        if not self._attached:
            return VacuumCommandResponse(success=True, message="vacuum already OFF")

        try:
            if self._mode == "gazebo_follow":
                self._detach_gazebo()
            else:
                self._publish_attached(False)
                rospy.loginfo("VacuumSimulator stub detach")
            return VacuumCommandResponse(success=True, message="vacuum OFF")
        except Exception as exc:
            rospy.logerr("Vacuum detach failed: %s", exc)
            return VacuumCommandResponse(success=False, message=str(exc))



# Log level must be chosen before init_node, so it cannot come from a private
# param; log_level_utils reads the LUGGAGE_LOG_LEVEL environment variable.
import os as _os
import sys as _sys
import rospkg as _rospkg
_DESC = _os.path.join(
    _rospkg.RosPack().get_path("luggage_description"), "scripts")
if _DESC not in _sys.path:
    _sys.path.insert(0, _DESC)
from log_level_utils import resolve_log_level  # noqa: E402

def main():
    rospy.init_node("vacuum_simulator", log_level=resolve_log_level())
    sim = VacuumSimulator()
    rospy.Service("~vacuum_command", VacuumCommand, sim.handle)
    rospy.loginfo(
        "vacuum_simulator ready (mode=%s, attached_param=%s)",
        sim._mode,
        sim._attached_param,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
