#!/usr/bin/env python3
"""Filter the current pickup box out of the depth cloud before the MoveIt
octomap sees it (the "semantic/geometric relaxation" the pickup needs).

Why this exists
---------------
In active loading the pickup box is added to the planning scene as a NAMED
collision object ``current_pickup_box`` with an AllowedCollisionMatrix entry
allowing ``suction_panel`` to contact it (scene_manager_node). That ACM is
how the cup is allowed to touch the box.

But the RealSense depth camera ALSO inserts the box surface into the MoveIt
octomap. The whole octomap is one collision object named ``<octomap>``, so
the suction_panel<->current_pickup_box ACM cannot relax it — the cup collides
with the octomap's box voxels and cannot descend to contact (Cartesian stops
at ~84%, OMPL goal is GOAL_IN_COLLISION against <octomap>).

This node subscribes to the raw depth cloud, drops points inside the known
box OBB (read from /luggage/current_box, world frame; transformed to the
cloud frame via TF), and republishes a filtered cloud. Point the MoveIt
octomap sensor at the filtered topic (sensors_d435_pointcloud.yaml
``point_cloud_topic``) so the box never enters the octomap, while container
walls and any other obstacles remain fully represented.

The box itself keeps its correct collision representation via the named
``current_pickup_box`` object + ACM (contact allowed). No ML is needed — the
box is ground-truth known in active loading.
"""

from __future__ import division

import threading

import numpy as np
import rospy
import tf2_ros
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


def _quat_to_rotmat(qx, qy, qz, qw):
    """Rotation matrix R such that v_world = R @ v_local (local->world)."""
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


class PickupBoxPointCloudFilter:
    def __init__(self):
        self._input_topic = rospy.get_param("~input_cloud", "/camera/depth/points")
        self._output_topic = rospy.get_param(
            "~output_cloud", "/camera/depth/points_filtered"
        )
        self._box_param = rospy.get_param("~box_param", "/luggage/current_box")
        self._world_frame = rospy.get_param("~world_frame", "world")
        # Inflation of the box OBB so residual edge voxels near the surface
        # are also removed (the cup must reach right onto the box top).
        self._padding = float(rospy.get_param("~box_padding", 0.02))
        # When set, only the top-most `top_crop` meters (in box-local +Z) are
        # filtered — useful if the whole box should still be seen as an
        # obstacle below the grasp surface. Default: filter the whole box.
        self._top_crop = float(rospy.get_param("~box_top_crop", 0.0))
        self._allow_latest_tf_fallback = bool(
            rospy.get_param("~allow_latest_tf_fallback", False)
        )
        # On TF failure: False (default) = fail-closed (drop frame, don't
        # publish) so box/self points don't leak into the octomap; True =
        # passthrough the raw cloud (old behavior, keeps octomap updating).
        self._tf_fail_passthrough = bool(
            rospy.get_param("~tf_fail_passthrough", False)
        )
        self._drop_log_interval = float(rospy.get_param("~drop_log_interval", 5.0))

        self._lock = threading.Lock()
        # (center np(3), R_box world<-box np(3,3), half np(3))
        self._box = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._pub = rospy.Publisher(self._output_topic, PointCloud2, queue_size=1)
        self._sub = rospy.Subscriber(
            self._input_topic, PointCloud2, self._on_cloud, queue_size=1
        )
        rospy.Timer(rospy.Duration(0.5), self._poll_box)
        self._poll_box(None)

        rospy.loginfo(
            "pickup_box_pointcloud_filter: %s -> %s (box_param=%s padding=%.3f)",
            self._input_topic,
            self._output_topic,
            self._box_param,
            self._padding,
        )

    def _poll_box(self, _event):
        if not rospy.has_param(self._box_param):
            with self._lock:
                self._box = None
            return
        try:
            data = rospy.get_param(self._box_param) or {}
            pos = data["pose"]["position"]
            ori = data["pose"]["orientation"]
            w = float(data["width"])
            d = float(data["depth"])
            h = float(data["height"])
        except (KeyError, TypeError, ValueError):
            return
        center = np.array([pos["x"], pos["y"], pos["z"]], dtype=np.float64)
        r_box = _quat_to_rotmat(
            float(ori["x"]), float(ori["y"]), float(ori["z"]), float(ori["w"])
        )
        half = np.array([w, d, h], dtype=np.float64) * 0.5 + self._padding
        with self._lock:
            self._box = (center, r_box, half)

    def _lookup_world_from_cloud(self, cloud_frame, stamp):
        """Return (R 3x3, t 3) for P_world = R @ P_cloud + t, or None."""
        stamps = [stamp]
        if self._allow_latest_tf_fallback:
            stamps.append(rospy.Time(0))
        for t in stamps:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._world_frame, cloud_frame, t, rospy.Duration(0.1)
                )
                tr = tf.transform.translation
                rr = tf.transform.rotation
                return _quat_to_rotmat(rr.x, rr.y, rr.z, rr.w), np.array(
                    [tr.x, tr.y, tr.z], dtype=np.float64
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                continue
        return None

    def _on_cloud(self, msg):
        box = self._current_box()
        if box is None:
            self._pub.publish(msg)  # no current box: pass through unchanged
            return
        center, r_box, half = box

        # cloud points in their own frame
        pts = np.array(
            list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=np.float64,
        )
        if pts.size == 0:
            self._pub.publish(msg)
            return

        transform = self._lookup_world_from_cloud(
            msg.header.frame_id, msg.header.stamp
        )
        if transform is None:
            # TF unavailable: fail-closed by default (drop the frame) so box
            # and robot-self points don't leak into the octomap. Set
            # ~tf_fail_passthrough=true to keep the old passthrough behavior.
            if self._tf_fail_passthrough:
                rospy.logwarn_throttle(5.0,
                    "pickup_box_filter: TF %s->%s unavailable, passthrough",
                    msg.header.frame_id, self._world_frame)
                self._pub.publish(msg)
            else:
                rospy.logwarn_throttle(5.0,
                    "pickup_box_filter: TF %s->%s unavailable, dropping frame",
                    msg.header.frame_id, self._world_frame)
            return
        r_wc, t_wc = transform

        # P_world = R_wc @ P_cloud + t
        p_world = pts @ r_wc.T + t_wc
        # into box-local: v_local = R_box^T @ (P_world - center)  == (P_world-center) @ R_box
        p_local = (p_world - center) @ r_box
        inside = (np.abs(p_local) <= half).all(axis=1)
        if self._top_crop > 0.0:
            # Only crop points whose box-local Z is within top_crop of the top.
            inside = inside & (p_local[:, 2] >= (half[2] - self._top_crop))
        keep = ~inside
        dropped = int(np.count_nonzero(inside))

        if dropped == 0:
            self._pub.publish(msg)
            return

        filtered = pts[keep]
        out = pc2.create_cloud_xyz32(msg.header, filtered.tolist())
        self._pub.publish(out)

        if self._drop_log_interval > 0.0:
            # Per-frame filter accounting: only useful while debugging the cloud
            # pipeline, so it stays out of the run log unless debug is enabled.
            rospy.logdebug_throttle(
                self._drop_log_interval,
                "pickup_box_filter: dropped %d/%d box points (%.1f%%)",
                dropped, len(pts), 100.0 * dropped / max(1, len(pts)),
            )

    def _current_box(self):
        with self._lock:
            return self._box



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
    rospy.init_node("pickup_box_pointcloud_filter", log_level=resolve_log_level())
    PickupBoxPointCloudFilter()
    rospy.spin()


if __name__ == "__main__":
    main()
