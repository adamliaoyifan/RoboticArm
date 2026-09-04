#!/usr/bin/env python3
"""Publish the end-effector's ACTUAL executed trajectory as an RViz LINE_STRIP.

Why this exists
---------------
The motion_planner only publishes target-point markers (where the arm *should*
go) and MoveIt's /move_group/display_planned_path shows the *planned* path. The
_ExecutionJointLogger logs joint desired/actual/error to rosout as text, but
there was no RViz visualization of where the EE *actually* moved. That makes it
hard to see plan-vs-execution drift, settle oscillation, or unexpected detours.

This node samples the EE link pose in the base frame at a fixed rate while the
arm is moving, appends points to a persistent LINE_STRIP, and republishes it.
The trace is reset on demand (service) and capped to a max length so it does
not grow without bound during a long run.

The trace is latched off (empty) at start so RViz shows the line appear as the
arm moves. A min inter-point distance avoids a dense point cloud when the arm
sits still.
"""

from __future__ import division

import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker


def _dist2(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2


class EETrajectoryTrace:
    def __init__(self):
        self._base_frame = rospy.get_param("~base_frame", "elfin_base_link")
        self._ee_link = rospy.get_param("~ee_link", "suction_contact_frame")
        self._rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        # Min distance (m) between consecutive trace points; below this the
        # arm is treated as stationary and the point is skipped.
        self._min_step = float(rospy.get_param("~min_step", 0.005))
        # Cap on stored points (rolling window). 0 = unlimited.
        self._max_points = int(rospy.get_param("~max_points", 4000))
        self._line_width = float(rospy.get_param("~line_width", 0.015))
        self._topic = rospy.get_param("~topic", "/luggage/debug/ee_trace")

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._pub = rospy.Publisher(self._topic, Marker, queue_size=1, latch=True)
        self._lock = threading.Lock()
        self._points = []  # list of (x, y, z)
        self._last = None

        rospy.Service("~reset_trace", Trigger, self._handle_reset)
        self._publish_empty()  # show an (empty) trace in RViz immediately

        period = rospy.Duration(1.0 / max(0.1, self._rate_hz))
        self._timer = rospy.Timer(period, self._on_tick)
        rospy.loginfo(
            "ee_trajectory_trace: ee=%s base=%s rate=%.1fHz topic=%s",
            self._ee_link, self._base_frame, self._rate_hz, self._topic,
        )

    def _handle_reset(self, _req):
        with self._lock:
            self._points = []
            self._last = None
        self._publish_empty()
        return TriggerResponse(success=True, message="ee trace cleared")

    def _publish_empty(self):
        m = self._empty_marker()
        self._pub.publish(m)

    def _empty_marker(self):
        m = Marker()
        m.header.frame_id = self._base_frame
        m.header.stamp = rospy.Time.now()
        m.ns = "ee_trace"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation = Quaternion(w=1.0)
        m.scale.x = self._line_width
        m.color.r = 0.0
        m.color.g = 0.8
        m.color.b = 1.0
        m.color.a = 0.8
        return m

    def _on_tick(self, _event):
        # base->ee transform (point of ee in base)
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame, self._ee_link, rospy.Time(0), rospy.Duration(0.05)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return
        t = tf.transform.translation
        xyz = (t.x, t.y, t.z)
        with self._lock:
            if self._last is not None and _dist2(xyz, self._last) < self._min_step ** 2:
                return  # stationary — skip
            self._points.append(xyz)
            if self._max_points > 0 and len(self._points) > self._max_points:
                del self._points[: len(self._points) - self._max_points]
            self._last = xyz
            pts = list(self._points)
        m = self._empty_marker()
        m.header.stamp = rospy.Time.now()
        m.points = [Point(x=p[0], y=p[1], z=p[2]) for p in pts]
        self._pub.publish(m)



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
    rospy.init_node("ee_trajectory_trace", log_level=resolve_log_level())
    EETrajectoryTrace()
    rospy.spin()


if __name__ == "__main__":
    main()
