#!/usr/bin/env python3
"""ROS node: pickup-box detection overlay.

Draws what the detector decided, not what the camera is currently seeing.
``luggage_detector`` estimates the pickup box once per ``DetectPickupBox``
attempt and publishes the outcome (success or failure) as a diagnostics
record; this node listens for that record and republishes it in two forms:

  /luggage/debug/detect_overlay   colour image with the OBB + centre drawn
  /luggage/debug/detected_box     world-frame MarkerArray for the 3-D view

Both are event-driven: one frame per detection, not one per camera frame. The
detection is a discrete decision, and re-rendering it at camera rate would
both cost a second full-rate image stream and suggest a continuous tracker
that does not exist.

Nothing here feeds the control path -- the orchestrator still uses the
``detect_luggage`` service response. A failed detection publishes a cleared
frame and deletes the markers, because staying silent would leave the last
successful overlay latched in RViz and read as "still detected".
"""

from __future__ import division

import json
import os
import sys
import threading

import numpy as np
import rospy
import rospkg
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

SCRIPTS_DIR = os.path.join(
    rospkg.RosPack().get_path("luggage_perception"), "scripts")
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from detect_overlay import (  # noqa: E402
    OBB_EDGES,
    draw_detection_overlay,
    draw_failure_overlay,
    format_detection_label,
    obb_corners_world,
    parse_detection_record,
    project_detection,
    rotation_from_quaternion,
    source_color_bgr,
)

MARKER_NS = "detected_box"
LATEST_DETECTION_PARAM = "/luggage/perception/detection/latest"


def _bgr_to_color_rgba(color_bgr, alpha=1.0):
    b, g, r = color_bgr
    return ColorRGBA(r=r / 255.0, g=g / 255.0, b=b / 255.0, a=alpha)


class DetectOverlayNode(object):
    def __init__(self):
        self._world_frame = rospy.get_param("~world_frame", "world")
        self._hold_sec = float(rospy.get_param("~hold_sec", 10.0))
        self._tf_timeout = float(rospy.get_param("~tf_timeout_sec", 0.5))
        # Display-only jump gate, off by default. Detections are sparse (a few
        # per box), so there is nothing to de-jitter yet; the knob exists for
        # the day a continuous preview is added.
        self._jump_gate_m = float(rospy.get_param("~jump_gate_m", 0.0))

        detection_topic = rospy.get_param(
            "~detection_topic", "/luggage_detector/diagnostics_json")
        color_image_topic = rospy.get_param(
            "~color_image_topic", "/camera/color/image_raw")
        color_info_topic = rospy.get_param(
            "~color_info_topic", "/camera/color/camera_info")

        self._bridge = CvBridge()
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._lock = threading.Lock()
        self._latest_image = None
        self._latest_info = None
        self._last_success_stamp = None
        self._last_position = None

        self._overlay_pub = rospy.Publisher(
            "/luggage/debug/detect_overlay", Image, queue_size=1, latch=True)
        self._marker_pub = rospy.Publisher(
            "/luggage/debug/detected_box", MarkerArray, queue_size=1,
            latch=True)

        rospy.Subscriber(
            color_image_topic, Image, self._on_image, queue_size=1,
            buff_size=2 ** 24)
        rospy.Subscriber(
            color_info_topic, CameraInfo, self._on_camera_info, queue_size=1)
        rospy.Subscriber(
            detection_topic, String, self._on_detection, queue_size=5)

        if self._hold_sec > 0.0:
            rospy.Timer(rospy.Duration(1.0), self._on_hold_timer)

        # A latched detector publication predates this node on a restart, and
        # the param survives even that, so recover the last state instead of
        # showing nothing until the next detection.
        self._replay_latest_param()

        rospy.loginfo(
            "detect_overlay ready: detection=%s image=%s info=%s hold=%.1fs",
            detection_topic, color_image_topic, color_info_topic,
            self._hold_sec)

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _on_image(self, msg):
        with self._lock:
            self._latest_image = msg

    def _on_camera_info(self, msg):
        with self._lock:
            self._latest_info = msg

    def _on_detection(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError as exc:
            rospy.logwarn_throttle(
                5.0, "detect_overlay: bad diagnostics payload: %s", exc)
            return
        self._handle_record(payload)

    def _replay_latest_param(self):
        record = rospy.get_param(LATEST_DETECTION_PARAM, None)
        if isinstance(record, dict):
            self._handle_record(record)

    def _handle_record(self, payload):
        record = parse_detection_record(payload)
        if record is None:
            return

        if not record.success:
            self._clear(record.reason)
            return

        if self._suppressed_by_jump_gate(record):
            return

        self._publish_success(record)

    def _suppressed_by_jump_gate(self, record):
        if self._jump_gate_m <= 0.0:
            return False
        with self._lock:
            previous = self._last_position
        if previous is None:
            return False
        moved = float(np.linalg.norm(record.position - previous))
        if moved <= self._jump_gate_m:
            return False
        rospy.logwarn_throttle(
            5.0, "detect_overlay: holding previous box, moved %.3fm > %.3fm",
            moved, self._jump_gate_m)
        return True

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _publish_success(self, record):
        with self._lock:
            self._last_success_stamp = rospy.Time.now()
            self._last_position = np.array(record.position, copy=True)

        self._publish_markers(record)
        self._publish_overlay(record)

    def _publish_overlay(self, record):
        with self._lock:
            image_msg = self._latest_image
            info_msg = self._latest_info

        if image_msg is None or info_msg is None:
            rospy.logwarn_throttle(
                10.0,
                "detect_overlay: no colour image/camera_info yet; "
                "publishing markers only")
            return

        transform = self._lookup_transform(info_msg.header.frame_id)
        if transform is None:
            return

        rotation, translation = transform
        intrinsics = (info_msg.K[0], info_msg.K[4], info_msg.K[2], info_msg.K[5])
        centre_uv, corner_uv, corner_valid, _ = project_detection(
            record.position, record.quat, record.size,
            rotation, translation, intrinsics)

        try:
            bgr = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except Exception as exc:  # cv_bridge raises its own error type
            rospy.logwarn_throttle(
                5.0, "detect_overlay: cv_bridge decode failed: %s", exc)
            return

        overlay = draw_detection_overlay(
            bgr, centre_uv, corner_uv, corner_valid,
            format_detection_label(record), source_color_bgr(record.source))
        self._publish_image(overlay, image_msg.header)

    def _publish_markers(self, record):
        corners = obb_corners_world(record.position, record.quat, record.size)
        color = _bgr_to_color_rgba(source_color_bgr(record.source))
        stamp = rospy.Time.now()
        array = MarkerArray()

        centre = Marker()
        centre.header.frame_id = self._world_frame
        centre.header.stamp = stamp
        centre.ns = MARKER_NS
        centre.id = 0
        centre.type = Marker.SPHERE
        centre.action = Marker.ADD
        centre.pose.position.x = float(record.position[0])
        centre.pose.position.y = float(record.position[1])
        centre.pose.position.z = float(record.position[2])
        centre.pose.orientation.w = 1.0
        centre.scale.x = centre.scale.y = centre.scale.z = 0.04
        centre.color = color
        array.markers.append(centre)

        wireframe = Marker()
        wireframe.header.frame_id = self._world_frame
        wireframe.header.stamp = stamp
        wireframe.ns = MARKER_NS
        wireframe.id = 1
        wireframe.type = Marker.LINE_LIST
        wireframe.action = Marker.ADD
        wireframe.pose.orientation.w = 1.0
        wireframe.scale.x = 0.01
        wireframe.color = color
        for a, b in OBB_EDGES:
            for index in (a, b):
                wireframe.points.append(Point(
                    x=float(corners[index][0]),
                    y=float(corners[index][1]),
                    z=float(corners[index][2])))
        array.markers.append(wireframe)

        label = Marker()
        label.header.frame_id = self._world_frame
        label.header.stamp = stamp
        label.ns = MARKER_NS
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = float(record.position[0])
        label.pose.position.y = float(record.position[1])
        label.pose.position.z = float(record.position[2] + record.size[2] * 0.5 + 0.08)
        label.pose.orientation.w = 1.0
        label.scale.z = 0.05
        label.color = color
        label.text = format_detection_label(record)
        array.markers.append(label)

        self._marker_pub.publish(array)

    def _clear(self, reason):
        """Delete the markers and show why there is no box.

        Published unconditionally: a reported failure is worth seeing even
        when nothing was on screen, and the callers are self-limiting -- a
        detection failure arrives at most once per detect attempt, and the
        hold timer stops firing as soon as it clears the last success.
        """
        with self._lock:
            image_msg = self._latest_image
            self._last_success_stamp = None
            self._last_position = None

        delete_all = Marker()
        delete_all.header.frame_id = self._world_frame
        delete_all.header.stamp = rospy.Time.now()
        delete_all.ns = MARKER_NS
        delete_all.action = Marker.DELETEALL
        self._marker_pub.publish(MarkerArray(markers=[delete_all]))

        if image_msg is None:
            return
        try:
            bgr = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except Exception as exc:  # cv_bridge raises its own error type
            rospy.logwarn_throttle(
                5.0, "detect_overlay: cv_bridge decode failed: %s", exc)
            return
        self._publish_image(draw_failure_overlay(bgr, reason), image_msg.header)

    def _on_hold_timer(self, _event):
        with self._lock:
            stamp = self._last_success_stamp
        if stamp is None:
            return
        if (rospy.Time.now() - stamp).to_sec() < self._hold_sec:
            return
        self._clear("stale (no detection for %.0fs)" % self._hold_sec)

    def _publish_image(self, bgr, header):
        try:
            msg = self._bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
        except Exception as exc:  # cv_bridge raises its own error type
            rospy.logwarn_throttle(
                5.0, "detect_overlay: cv_bridge encode failed: %s", exc)
            return
        msg.header = header
        self._overlay_pub.publish(msg)

    def _lookup_transform(self, camera_frame):
        if not camera_frame:
            rospy.logwarn_throttle(
                10.0, "detect_overlay: camera_info has no frame_id")
            return None
        try:
            # Latest TF rather than the image stamp: the arm holds
            # pickup_observe through the whole detection, so the two agree,
            # and an exact-stamp lookup would drop the overlay entirely the
            # moment TF is a few ms behind.
            tf_msg = self._tf_buffer.lookup_transform(
                camera_frame, self._world_frame, rospy.Time(0),
                rospy.Duration(self._tf_timeout))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn_throttle(
                5.0, "detect_overlay: TF %s->%s failed: %s",
                self._world_frame, camera_frame, exc)
            return None

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        rotation = rotation_from_quaternion([q.x, q.y, q.z, q.w])
        return rotation, np.array([t.x, t.y, t.z], dtype=np.float64)


# Log level must be chosen before init_node, so it cannot come from a private
# param; log_level_utils reads the LUGGAGE_LOG_LEVEL environment variable.
_DESC = os.path.join(
    rospkg.RosPack().get_path("luggage_description"), "scripts")
if _DESC not in sys.path:
    sys.path.insert(0, _DESC)
from log_level_utils import resolve_log_level  # noqa: E402


def main():
    rospy.init_node("detect_overlay", log_level=resolve_log_level())
    DetectOverlayNode()
    rospy.spin()


if __name__ == "__main__":
    main()
