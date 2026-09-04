#!/usr/bin/env python3
"""Publish scene relation markers from pose_tune draft scene_tf (preview only)."""

from __future__ import division

import math
import os
import sys

import rospy
import rospkg
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
if DESC_SCRIPTS not in sys.path:
    sys.path.insert(0, DESC_SCRIPTS)

from scene_tf_config_utils import (  # noqa: E402
    container_in_base_link,
    container_opening_in_base_link,
    container_outer_dimensions,
    robot_base_in_world,
    urdf_world_base_pose,
)
from pose_tune_draft_utils import DRAFT_PARAM, get_draft  # noqa: E402


def _rpy_to_quaternion(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return {
        "x": sr * cp * cy - cr * sp * sy,
        "y": cr * sp * cy + sr * cp * sy,
        "z": cr * cp * sy - sr * sp * cy,
        "w": cr * cp * cy + sr * sp * sy,
    }


def _rotate_vec(q, vec):
    x, y, z = vec
    qx, qy, qz, qw = q["x"], q["y"], q["z"], q["w"]
    ix = qw * x + qy * z - qz * y
    iy = qw * y + qz * x - qx * z
    iz = qw * z + qx * y - qy * x
    iw = -qx * x - qy * y - qz * z
    return [
        ix * qw + iw * -qx + iy * -qz - iz * -qy,
        iy * qw + iw * -qy + iz * -qx - ix * -qz,
        iz * qw + iw * -qz + ix * -qy - iy * -qx,
    ]


class PoseTuneSceneVizNode:
    def __init__(self):
        self._base_frame = rospy.get_param("~base_frame", "elfin_base_link")
        self._marker_pub = rospy.Publisher(
            "/luggage/pose_tune/scene_markers", MarkerArray, queue_size=1, latch=True
        )
        if not rospy.has_param(DRAFT_PARAM):
            from pose_tune_draft_utils import init_draft_from_production

            init_draft_from_production(rospy)
        rospy.Service("~refresh", Trigger, self._handle_refresh)
        rospy.Timer(rospy.Duration(1.0), self._on_timer, oneshot=False)
        self.publish()

    def _on_timer(self, _event):
        self.publish()

    def _handle_refresh(self, _req):
        self.publish()
        return TriggerResponse(success=True, message="scene markers refreshed")

    def publish(self):
        draft = get_draft(rospy)
        opening_xyz, _opening_rpy = container_opening_in_base_link(draft)
        base_xyz, base_rpy = container_in_base_link(draft)
        outer_l, outer_w, outer_h = container_outer_dimensions(draft)

        horiz = math.sqrt(opening_xyz[0] ** 2 + opening_xyz[1] ** 2)
        delta_z = opening_xyz[2]
        yaw_to_opening = math.atan2(opening_xyz[1], opening_xyz[0])

        planning_base, _ = robot_base_in_world(draft)
        urdf_base, _ = urdf_world_base_pose(draft)
        base_delta = math.sqrt(
            (planning_base[0] - urdf_base[0]) ** 2
            + (planning_base[1] - urdf_base[1]) ** 2
            + (planning_base[2] - urdf_base[2]) ** 2
        )

        markers = MarkerArray()
        stamp = rospy.Time.now()
        mid = 0

        def add(marker):
            nonlocal mid
            marker.header.frame_id = self._base_frame
            marker.header.stamp = stamp
            marker.id = mid
            mid += 1
            markers.markers.append(marker)

        arrow = Marker()
        arrow.type = Marker.ARROW
        arrow.ns = "scene_relation"
        arrow.action = Marker.ADD
        arrow.points = [Point(0.0, 0.0, 0.0), Point(opening_xyz[0], opening_xyz[1], opening_xyz[2])]
        arrow.scale.x = 0.03
        arrow.scale.y = 0.06
        arrow.scale.z = 0.06
        arrow.color = ColorRGBA(0.2, 0.8, 1.0, 0.95)
        add(arrow)

        text = Marker()
        text.type = Marker.TEXT_VIEW_FACING
        text.ns = "scene_relation"
        text.action = Marker.ADD
        text.pose.position.x = opening_xyz[0] * 0.5
        text.pose.position.y = opening_xyz[1] * 0.5
        text.pose.position.z = opening_xyz[2] * 0.5 + 0.15
        text.scale.z = 0.08
        text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        text.text = (
            "horiz=%.2fm dz=%.2fm yaw=%.1fdeg\npreview planning math"
            % (horiz, delta_z, math.degrees(yaw_to_opening))
        )
        if base_delta > 0.01:
            text.text += "\nURDF base delta=%.3fm (informative)" % base_delta
        add(text)

        wire = Marker()
        wire.type = Marker.LINE_LIST
        wire.ns = "container_wireframe"
        wire.action = Marker.ADD
        wire.scale.x = 0.015
        wire.color = ColorRGBA(0.7, 0.7, 0.7, 0.8)
        half_l, half_w, half_h = outer_l * 0.5, outer_w * 0.5, outer_h * 0.5
        corners_local = [
            [-half_l, -half_w, 0.0],
            [half_l, -half_w, 0.0],
            [half_l, half_w, 0.0],
            [-half_l, half_w, 0.0],
            [-half_l, -half_w, outer_h],
            [half_l, -half_w, outer_h],
            [half_l, half_w, outer_h],
            [-half_l, half_w, outer_h],
        ]
        rot = _rpy_to_quaternion(base_rpy)
        edges = (
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        )
        for i, j in edges:
            pi = _rotate_vec(rot, corners_local[i])
            pj = _rotate_vec(rot, corners_local[j])
            wire.points.append(
                Point(base_xyz[0] + pi[0], base_xyz[1] + pi[1], base_xyz[2] + pi[2])
            )
            wire.points.append(
                Point(base_xyz[0] + pj[0], base_xyz[1] + pj[1], base_xyz[2] + pj[2])
            )
        add(wire)

        delete_old = Marker()
        delete_old.action = Marker.DELETEALL
        markers.markers.insert(0, delete_old)
        self._marker_pub.publish(markers)


def main():
    rospy.init_node("pose_tune_scene_viz")
    PoseTuneSceneVizNode()
    rospy.spin()


if __name__ == "__main__":
    main()
