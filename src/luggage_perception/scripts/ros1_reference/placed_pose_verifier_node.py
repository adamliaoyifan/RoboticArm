#!/usr/bin/env python3
"""RGBD placed-box pose verifier for production and simulation."""
import math
import os
import sys
import threading
import time

import numpy as np
import rospy
import rospkg
import tf2_ros
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from tf.transformations import quaternion_from_euler, quaternion_matrix

from luggage_msgs.srv import VerifyPlacedBox, VerifyPlacedBoxResponse

SCRIPTS_DIR = os.path.join(
    rospkg.RosPack().get_path("luggage_perception"), "scripts")
if SCRIPTS_DIR in sys.path:
    sys.path.remove(SCRIPTS_DIR)
sys.path.insert(0, SCRIPTS_DIR)
from luggage_box_estimator import _refine_rectangle


def _rpy_from_quaternion(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = (
        math.copysign(math.pi * 0.5, sinp)
        if abs(sinp) >= 1.0 else math.asin(sinp))
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return roll, pitch, math.atan2(siny, cosy)


def _angle_error_pi(actual, expected):
    delta = actual - expected
    return math.atan2(math.sin(2.0 * delta), math.cos(2.0 * delta)) * 0.5


class PlacedPoseVerifier(object):
    def __init__(self):
        self._base_frame = rospy.get_param("~base_frame", "elfin_base_link")
        self._cloud_topic = rospy.get_param(
            "~cloud_topic", "/camera/depth/points_robot_filtered")
        self._fresh_sec = float(rospy.get_param("~fresh_sec", 3.0))
        self._wait_sec = float(rospy.get_param("~wait_sec", 10.0))
        self._drift_wait_sec = float(rospy.get_param(
            "~drift_wait_sec", 3.0))
        self._min_points = int(rospy.get_param("~min_points", 100))
        self._xy_tolerance = float(rospy.get_param("~xy_tolerance", 0.04))
        self._z_tolerance = float(rospy.get_param("~z_tolerance", 0.03))
        self._yaw_tolerance = math.radians(float(rospy.get_param(
            "~yaw_tolerance_deg", 5.0)))
        self._rp_tolerance = math.radians(float(rospy.get_param(
            "~roll_pitch_tolerance_deg", 2.0)))
        self._drift_tolerance = float(rospy.get_param(
            "~drift_tolerance", 0.02))
        self._allow_latest_tf = bool(rospy.get_param(
            "~allow_stable_latest_tf_fallback", True))
        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf)
        self._condition = threading.Condition()
        self._latest = None
        rospy.Subscriber(
            self._cloud_topic, PointCloud2, self._on_cloud, queue_size=1)
        rospy.Service("~verify", VerifyPlacedBox, self._handle)

    def _on_cloud(self, cloud):
        with self._condition:
            self._latest = cloud
            self._condition.notify_all()

    def _fresh_cloud(self, after_stamp=None):
        deadline = time.time() + self._wait_sec
        with self._condition:
            while not rospy.is_shutdown():
                cloud = self._latest
                if cloud is not None:
                    age = max(
                        0.0, (rospy.Time.now() - cloud.header.stamp).to_sec())
                    newer = (
                        after_stamp is None
                        or cloud.header.stamp > after_stamp)
                    if age <= self._fresh_sec and newer:
                        return cloud
                remaining = deadline - time.time()
                if remaining <= 0.0:
                    return None
                self._condition.wait(timeout=min(0.1, remaining))
        return None

    def _cloud_in_base(self, cloud):
        try:
            transform = self._tf.lookup_transform(
                self._base_frame, cloud.header.frame_id,
                cloud.header.stamp, rospy.Duration(0.5))
        except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            if not self._allow_latest_tf:
                return None
            transform = self._tf.lookup_transform(
                self._base_frame, cloud.header.frame_id,
                rospy.Time(0), rospy.Duration(0.5))
        raw = np.asarray(list(point_cloud2.read_points(
            cloud, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=np.float64)
        if raw.size == 0:
            return None
        q = transform.transform.rotation
        matrix = quaternion_matrix([q.x, q.y, q.z, q.w])
        translation = transform.transform.translation
        return (
            np.dot(raw[:, :3], matrix[:3, :3].T)
            + np.asarray([translation.x, translation.y, translation.z]))

    def _estimate(self, cloud, planned):
        points = self._cloud_in_base(cloud)
        if points is None:
            return None, "cloud transform unavailable"
        _roll, _pitch, planned_yaw = _rpy_from_quaternion(
            planned.place_pose.orientation)
        center = planned.place_pose.position
        dx = points[:, 0] - center.x
        dy = points[:, 1] - center.y
        local_x = math.cos(-planned_yaw) * dx - math.sin(-planned_yaw) * dy
        local_y = math.sin(-planned_yaw) * dx + math.cos(-planned_yaw) * dy
        expected_top = center.z + planned.height * 0.5
        mask = (
            (np.abs(local_x) <= planned.width * 0.55)
            & (np.abs(local_y) <= planned.depth * 0.55)
            & (np.abs(points[:, 2] - expected_top) <= 0.12))
        roi = points[mask]
        if roi.shape[0] < self._min_points:
            return None, "insufficient ROI points: %d" % roi.shape[0]
        bins = np.round(roi[:, 2] / 0.005).astype(np.int64)
        values, counts = np.unique(bins, return_counts=True)
        dominant_z = values[np.argmax(counts)] * 0.005
        top = roi[np.abs(roi[:, 2] - dominant_z) <= 0.012]
        if top.shape[0] < self._min_points:
            return None, "insufficient top-plane points: %d" % top.shape[0]
        design = np.column_stack(
            (top[:, 0], top[:, 1], np.ones(top.shape[0])))
        a, b, c = np.linalg.lstsq(design, top[:, 2], rcond=None)[0]
        normal = np.asarray([-a, -b, 1.0])
        normal /= np.linalg.norm(normal)
        roll = math.atan2(normal[1], normal[2])
        pitch = math.atan2(-normal[0], normal[2])
        # Placement policy restricts goal yaw to container axes. Refine around
        # that registered goal instead of a density-biased PCA seed; the final
        # yaw delta remains a hard gate, so this cannot hide a rotated box.
        measured_yaw, extent0, extent1, rectangle_center = (
            _refine_rectangle(
                top[:, :2], planned_yaw,
                search_deg=15.0, step_deg=0.25))
        yaw_delta = _angle_error_pi(measured_yaw, planned_yaw)
        measured_yaw = planned_yaw + yaw_delta
        q = quaternion_from_euler(roll, pitch, measured_yaw)
        pose = planned.place_pose.__class__()
        pose.position.x = float(rectangle_center[0])
        pose.position.y = float(rectangle_center[1])
        pose.position.z = float(np.median(top[:, 2]) - planned.height * 0.5)
        pose.orientation.x, pose.orientation.y = float(q[0]), float(q[1])
        pose.orientation.z, pose.orientation.w = float(q[2]), float(q[3])
        # Top-face extents along the refined rectangle axes, plus height above
        # the support surface. The support elevation is known geometry -- the
        # container floor or the top of an already committed box -- exactly as
        # the platform height is at pickup, so measuring against it gives a
        # real height rather than echoing the planned one back.
        support_z = (
            planned.place_pose.position.z - planned.height * 0.5)
        measured = (
            float(extent0), float(extent1),
            float(np.median(top[:, 2]) - support_z))
        return (pose, roll, pitch, abs(yaw_delta), top.shape[0],
                measured), ""

    def _handle(self, request):
        request_stamp = rospy.Time.now()
        first_cloud = self._fresh_cloud(request_stamp)
        if first_cloud is None:
            return VerifyPlacedBoxResponse(
                success=False, message="RGBD_STALE_CLOUD")
        first, error = self._estimate(first_cloud, request.planned)
        if first is None:
            return VerifyPlacedBoxResponse(
                success=False, message="RGBD_POSE_FAILED: %s" % error)
        rospy.sleep(self._drift_wait_sec)
        second_cloud = self._fresh_cloud(first_cloud.header.stamp)
        if second_cloud is None:
            return VerifyPlacedBoxResponse(
                success=False, message="RGBD_DRIFT_CLOUD_MISSING")
        second, error = self._estimate(second_cloud, request.planned)
        if second is None:
            return VerifyPlacedBoxResponse(
                success=False, message="RGBD_POSE_FAILED: %s" % error)
        first_pose = first[0]
        pose, roll, pitch, yaw_error, point_count, measured = second
        planned = request.planned.place_pose.position
        xy_error = math.hypot(
            pose.position.x - planned.x, pose.position.y - planned.y)
        z_error = abs(pose.position.z - planned.z)
        drift = math.sqrt(
            (pose.position.x - first_pose.position.x) ** 2
            + (pose.position.y - first_pose.position.y) ** 2
            + (pose.position.z - first_pose.position.z) ** 2)
        failures = []
        if xy_error > self._xy_tolerance:
            failures.append("xy")
        if z_error > self._z_tolerance:
            failures.append("z")
        if yaw_error > self._yaw_tolerance:
            failures.append("yaw")
        if max(abs(roll), abs(pitch)) > self._rp_tolerance:
            failures.append("roll_pitch")
        if drift > self._drift_tolerance:
            failures.append("drift")
        message = (
            "rgbd points=%d xy=%.3f z=%.3f yaw=%.3f rp=(%.3f,%.3f) "
            "drift=%.3f extents=(%.3f,%.3f,%.3f)" % (
                point_count, xy_error, z_error, yaw_error,
                roll, pitch, drift,
                measured[0], measured[1], measured[2]))
        if failures:
            message = "RGBD_POSE_GATE[%s]: %s" % (
                ",".join(failures), message)
        return VerifyPlacedBoxResponse(
            success=not failures,
            message=message,
            actual_pose=pose,
            xy_error=xy_error,
            z_error=z_error,
            yaw_error=yaw_error,
            roll=roll,
            pitch=pitch,
            drift=drift,
            point_count=point_count,
            measured_width=measured[0],
            measured_depth=measured[1],
            measured_height=measured[2],
        )



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
    rospy.init_node("placed_pose_verifier", log_level=resolve_log_level())
    PlacedPoseVerifier()
    rospy.loginfo("placed_pose_verifier ready")
    rospy.spin()


if __name__ == "__main__":
    main()
