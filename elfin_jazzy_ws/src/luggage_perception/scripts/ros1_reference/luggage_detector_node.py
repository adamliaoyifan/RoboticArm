#!/usr/bin/env python3
"""Luggage detector: estimate pickup-box pose from semantic point cloud.

Subscribes to the semantic cargo cloud (or raw depth as fallback),
runs RANSAC + PCA box fitting, transforms the result into world frame,
and serves the same ``~detect_luggage`` service as the previous stub.
"""

from __future__ import division

import os
import sys
import threading
import json

import numpy as np
import rospy
import rospkg
import tf2_ros
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from luggage_msgs.msg import DetectedLuggage
from luggage_msgs.srv import DetectLuggage, DetectLuggageResponse, GetCurrentBox

DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
PERC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_perception"), "scripts")
for _p in (DESC_SCRIPTS, PERC_SCRIPTS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from box_catalog_utils import box_catalog_entries, load_box_catalog  # noqa: E402
from luggage_box_estimator import estimate_box  # noqa: E402
from scene_tf_config_utils import (  # noqa: E402
    load_scene_tf_config,
    pickup_source_in_world,
    resolve_scene_tf_config_path,
)


def _transform_points_to_world(tf_buffer, points, source_frame, target_frame, stamp):
    """Rigid-body transform an (N,3) array from *source_frame* to *target_frame*."""
    try:
        tf_msg = tf_buffer.lookup_transform(target_frame, source_frame, stamp, rospy.Duration(0.5))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as exc:
        rospy.logwarn_throttle(5.0, "TF lookup %s->%s failed: %s", source_frame, target_frame, exc)
        return None

    t = tf_msg.transform.translation
    r = tf_msg.transform.rotation
    # Rotation matrix from quaternion.
    qx, qy, qz, qw = r.x, r.y, r.z, r.w
    rot = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ])
    trans = np.array([t.x, t.y, t.z])
    return points.dot(rot.T) + trans


class LuggageDetector:
    def __init__(self):
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        scene_cfg_path = rospy.get_param(
            "~scene_tf_config",
            rospy.get_param("/luggage/scene_tf_config", resolve_scene_tf_config_path()),
        )
        scene_config = load_scene_tf_config(scene_cfg_path)
        self._source_xyz, _ = pickup_source_in_world(scene_config)
        self._platform_z = self._source_xyz[2]

        catalog_config = load_box_catalog(scene_config=scene_config)
        self._catalog_entries = box_catalog_entries(catalog_config)

        self._world_frame = rospy.get_param("~world_frame", "world")
        self._use_semantic = rospy.get_param("~use_semantic", True)
        self._roi_margin = float(rospy.get_param("~roi_margin", 0.5))
        self._cloud_max_age = float(rospy.get_param("~cloud_max_age_sec", 1.0))
        self._catalog_tol = float(rospy.get_param(
            "~catalog_match_tolerance", 0.08))
        self._catalog_snap = bool(rospy.get_param(
            "~catalog_snap_enabled", False))
        self._min_points = int(rospy.get_param("~min_points", 50))
        self._min_confidence = float(rospy.get_param(
            "~min_confidence", 0.70))
        self._min_height_above_platform = float(rospy.get_param(
            "~min_height_above_platform", 0.03))
        self._allow_gt_fallback = bool(rospy.get_param(
            "~allow_gt_fallback", True))
        self._evaluation_compare_gt = bool(rospy.get_param(
            "~evaluation_compare_gt", False))
        self._last_failure_reason = "not_run"
        # Latched so a consumer that starts later (the detection overlay, a
        # debug rostopic echo) sees the last detection instead of nothing
        # until the next one.
        self._diag_pub = rospy.Publisher(
            "~diagnostics_json", String, queue_size=10, latch=True)

        self._cloud_lock = threading.Lock()
        self._latest_cloud = None
        self._latest_stamp = None
        self._latest_frame = None

        # Fallback service (spawner GT).
        self._current_box_service = rospy.get_param(
            "~current_box_service", "/pickup_box_spawner/get_current_box"
        )

        if self._use_semantic:
            topic = rospy.get_param("~cargo_cloud_topic", "/luggage/semantic/cargo_points")
        else:
            topic = rospy.get_param("~depth_topic", "/camera/depth/points")
        rospy.Subscriber(topic, PointCloud2, self._cloud_cb, queue_size=1)
        rospy.loginfo("luggage_detector subscribing to %s", topic)

    def _cloud_cb(self, msg):
        with self._cloud_lock:
            self._latest_cloud = msg
            self._latest_stamp = msg.header.stamp
            self._latest_frame = msg.header.frame_id

    # ------------------------------------------------------------------
    # GT fallback (identical to the old stub)
    # ------------------------------------------------------------------

    def _gt_fallback(self):
        """Return DetectedLuggage from the spawner / param (stub path)."""
        try:
            rospy.wait_for_service(self._current_box_service, timeout=0.2)
            resp = rospy.ServiceProxy(self._current_box_service, GetCurrentBox)()
            if resp.success:
                return resp.box
        except Exception:
            pass

        data = rospy.get_param("/luggage/current_box", {})
        if not data:
            return None
        pos = data.get("pose", {}).get("position", {})
        ori = data.get("pose", {}).get("orientation", {})
        return DetectedLuggage(
            id=data.get("id", "current_box"),
            width=float(data.get("width", 0.70)),
            height=float(data.get("height", 0.28)),
            depth=float(data.get("depth", 0.45)),
            pose=Pose(
                position=Point(
                    x=float(pos.get("x", 0.0)),
                    y=float(pos.get("y", 0.0)),
                    z=float(pos.get("z", 0.0)),
                ),
                orientation=Quaternion(
                    x=float(ori.get("x", 0.0)),
                    y=float(ori.get("y", 0.0)),
                    z=float(ori.get("z", 0.0)),
                    w=float(ori.get("w", 1.0)),
                ),
            ),
        )

    # ------------------------------------------------------------------
    # Perception path
    # ------------------------------------------------------------------

    def _estimate_from_cloud(self):
        """Run the full perception pipeline.

        Returns (DetectedLuggage, confidence) or (None, 0.0).
        """
        with self._cloud_lock:
            cloud_msg = self._latest_cloud
            stamp = self._latest_stamp
            frame = self._latest_frame

        if cloud_msg is None:
            self._last_failure_reason = "DETECT_NO_CLOUD"
            rospy.logwarn_throttle(5.0, "luggage_detector: no point cloud received yet")
            return None, 0.0

        age = (rospy.Time.now() - stamp).to_sec()
        if age > self._cloud_max_age:
            self._last_failure_reason = "DETECT_STALE_CLOUD"
            rospy.logwarn_throttle(5.0, "luggage_detector: cloud too old (%.2fs)", age)
            return None, 0.0

        # Extract xyz array from the PointCloud2.
        pts_gen = pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
        pts_camera = np.array(list(pts_gen), dtype=np.float64)
        if len(pts_camera) < self._min_points:
            self._last_failure_reason = "DETECT_TOO_FEW_POINTS"
            rospy.logwarn_throttle(5.0, "luggage_detector: too few points (%d)", len(pts_camera))
            return None, 0.0

        # Transform points into world frame so ROI and platform_z make sense.
        pts_world = _transform_points_to_world(
            self._tf_buffer, pts_camera, frame, self._world_frame, stamp,
        )
        if pts_world is None:
            self._last_failure_reason = "DETECT_TF_FAILED"
            return None, 0.0

        roi_xy = (self._source_xyz[0], self._source_xyz[1])

        est = estimate_box(
            pts_world,
            roi_center_xy=roi_xy,
            roi_margin=self._roi_margin,
            platform_z=self._platform_z,
            # Catalog snapping rounds the measurement to one of three shapes.
            # With continuously sized boxes that discards the answer instead of
            # cleaning it up, so it is off unless a run replays catalog sizes.
            catalog_entries=(
                self._catalog_entries if self._catalog_snap else None),
            catalog_tolerance=self._catalog_tol,
            min_points=self._min_points,
            min_height_above_platform=self._min_height_above_platform,
        )
        if est is None:
            self._last_failure_reason = "DETECT_ESTIMATION_FAILED"
            rospy.logwarn_throttle(5.0, "luggage_detector: box estimation failed")
            return None, 0.0
        if est.confidence < self._min_confidence:
            self._last_failure_reason = "DETECT_LOW_CONFIDENCE"
            rospy.logwarn_throttle(
                5.0,
                "luggage_detector: confidence %.2f below %.2f",
                est.confidence, self._min_confidence,
            )
            return None, est.confidence

        box_id = est.matched_catalog_id or "detected_box"

        rospy.loginfo(
            "luggage_detector: estimated box '%s' at (%.3f, %.3f, %.3f) "
            "size=(%.3f, %.3f, %.3f) conf=%.2f",
            box_id,
            est.center_xyz[0], est.center_xyz[1], est.center_xyz[2],
            est.width, est.depth, est.height, est.confidence,
        )

        self._last_failure_reason = "ok"
        return DetectedLuggage(
            id=box_id,
            width=est.width,
            depth=est.depth,
            height=est.height,
            yaw_valid=bool(est.yaw_valid),
            aspect_ratio=float(est.aspect_ratio),
            pose=Pose(
                position=Point(
                    x=float(est.center_xyz[0]),
                    y=float(est.center_xyz[1]),
                    z=float(est.center_xyz[2]),
                ),
                orientation=Quaternion(
                    x=float(est.quaternion_xyzw[0]),
                    y=float(est.quaternion_xyzw[1]),
                    z=float(est.quaternion_xyzw[2]),
                    w=float(est.quaternion_xyzw[3]),
                ),
            ),
        ), est.confidence

    def _publish_diagnostics(self, source, success, confidence, reason,
                             detected=None, gt=None):
        record = {
            "stamp": rospy.Time.now().to_sec(),
            "source": source,
            "success": bool(success),
            "confidence": float(confidence),
            "reason": str(reason),
            "allow_gt_fallback": self._allow_gt_fallback,
        }
        if detected is not None:
            record["detected"] = {
                "id": detected.id,
                "position": [
                    detected.pose.position.x,
                    detected.pose.position.y,
                    detected.pose.position.z,
                ],
                "orientation": [
                    detected.pose.orientation.x,
                    detected.pose.orientation.y,
                    detected.pose.orientation.z,
                    detected.pose.orientation.w,
                ],
                "size": [
                    detected.width, detected.depth, detected.height],
            }
        if gt is not None and detected is not None:
            record["gt_delta"] = [
                detected.pose.position.x - gt.pose.position.x,
                detected.pose.position.y - gt.pose.position.y,
                detected.pose.position.z - gt.pose.position.z,
            ]
            detected_yaw = 2.0 * math.atan2(
                detected.pose.orientation.z,
                detected.pose.orientation.w)
            gt_yaw = 2.0 * math.atan2(
                gt.pose.orientation.z, gt.pose.orientation.w)
            record["gt_yaw_delta"] = math.atan2(
                math.sin(2.0 * (detected_yaw - gt_yaw)),
                math.cos(2.0 * (detected_yaw - gt_yaw))) * 0.5
        rospy.set_param("/luggage/perception/detection/latest", record)
        self._diag_pub.publish(String(data=json.dumps(record, sort_keys=True)))

    # ------------------------------------------------------------------
    # Service handler
    # ------------------------------------------------------------------

    def handle(self, _req):
        detected, confidence = self._estimate_from_cloud()

        if detected is not None:
            gt = self._gt_fallback() if self._evaluation_compare_gt else None
            if gt is not None:
                dx = detected.pose.position.x - gt.pose.position.x
                dy = detected.pose.position.y - gt.pose.position.y
                dz = detected.pose.position.z - gt.pose.position.z
                rospy.loginfo(
                    "luggage_detector: perception vs GT delta "
                    "dx=%.4f dy=%.4f dz=%.4f",
                    dx, dy, dz,
                )
            self._publish_diagnostics(
                "perception", True, confidence, "ok", detected, gt)
            return DetectLuggageResponse(
                luggage=[detected], success=True,
                message="perception estimate (conf=%.2f)" % confidence,
            )

        if self._allow_gt_fallback:
            rospy.logwarn(
                "luggage_detector: perception failed — falling back to GT")
            gt = self._gt_fallback()
            if gt is not None:
                self._publish_diagnostics(
                    "gt_fallback", True, confidence,
                    self._last_failure_reason, gt, None)
                return DetectLuggageResponse(
                    luggage=[gt], success=True,
                    message="gt fallback (perception unavailable)",
                )

        rospy.logwarn(
            "luggage_detector: strict perception failed (%s)",
            self._last_failure_reason)
        self._publish_diagnostics(
            "perception", False, confidence, self._last_failure_reason)
        return DetectLuggageResponse(
            luggage=[], success=False,
            message=self._last_failure_reason,
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
    rospy.init_node("luggage_detector", log_level=resolve_log_level())
    detector = LuggageDetector()
    rospy.Service("~detect_luggage", DetectLuggage, detector.handle)
    rospy.loginfo(
        "luggage_detector ready (perception mode, semantic=%s)",
        detector._use_semantic,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
