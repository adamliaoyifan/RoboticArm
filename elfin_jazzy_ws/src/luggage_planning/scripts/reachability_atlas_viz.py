#!/usr/bin/env python3
"""RViz visualization for the reachability atlas.

Loads a precomputed atlas and publishes a latched MarkerArray showing
reachable (green), marginal (yellow), unreachable (red), and unknown (gray)
cells. Schema-v2 atlases can optionally be restricted to cells connected to
the container opening.

Usage:
    rosrun luggage_planning reachability_atlas_viz.py \
        _atlas_npz:=$(rospack find luggage_planning)/data/reachability_atlas/s20_container_collision_aware.npz \
        _atlas_meta:=$(rospack find luggage_planning)/data/reachability_atlas/s20_container_collision_aware.yaml
"""

from __future__ import division

import math
import os
import sys

import numpy as np
import rospy
import rospkg
import yaml
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

PLAN_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_planning"), "scripts")
if PLAN_SCRIPTS not in sys.path:
    sys.path.insert(0, PLAN_SCRIPTS)

# Load reachability_atlas from source (bypass any stale catkin wrapper in devel/lib).
import importlib.util as _ilu
_spec = _ilu.spec_from_file_location(
    "reachability_atlas", os.path.join(PLAN_SCRIPTS, "reachability_atlas.py"))
_ra = _ilu.module_from_spec(_spec)
_spec.loader.exec_module(_ra)
ReachabilityAtlas = _ra.ReachabilityAtlas
UNKNOWN = getattr(_ra, "UNKNOWN", 0)
UNREACHABLE = getattr(_ra, "UNREACHABLE", 1)
MARGINAL = getattr(_ra, "MARGINAL", 2)
REACHABLE = getattr(_ra, "REACHABLE", 3)


def _rpy_to_matrix(rpy):
    cr, sr = math.cos(rpy[0]), math.sin(rpy[0])
    cp, sp = math.cos(rpy[1]), math.sin(rpy[1])
    cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


class ReachabilityAtlasViz:
    def __init__(self):
        npz = rospy.get_param("~atlas_npz", "")
        meta = rospy.get_param("~atlas_meta", "")
        if not npz or not meta:
            # Model-aware default: s20_container_*.npz / s30_container_*.npz
            # based on /robot_name (set by active_loading from arm_model).
            _rn = str(rospy.get_param("/robot_name", "elfin_s20_with_camera"))
            _p = _rn[len("elfin_"):] if _rn.startswith("elfin_") else _rn
            _p = _p[:-len("_with_camera")] if _p.endswith("_with_camera") else _p
            _p = _p or "s20"
            default_dir = os.path.join(
                rospkg.RosPack().get_path("luggage_planning"),
                "data", "reachability_atlas")
            npz = os.path.join(default_dir, "%s_container_collision_aware.npz" % _p)
            meta = os.path.join(default_dir, "%s_container_collision_aware.yaml" % _p)

        self._atlas = ReachabilityAtlas.load(npz, meta)
        rospy.loginfo("Loaded atlas: %s", self._atlas.stats())

        self._show_reachable = bool(rospy.get_param("~show_reachable", True))
        self._show_marginal = bool(rospy.get_param("~show_marginal", True))
        self._show_unreachable = bool(rospy.get_param("~show_unreachable", True))
        self._show_unknown = bool(rospy.get_param("~show_unknown", True))
        self._opening_connected_only = bool(
            rospy.get_param("~opening_connected_only", False))
        self._marginal_threshold = float(rospy.get_param("~marginal_joint_margin", 0.1))
        self._yaw_filter = int(rospy.get_param("~yaw_filter", -1))  # -1=all
        self._base_frame = rospy.get_param("~base_frame", "elfin_base_link")

        schema_version = int(self._atlas.meta.get("schema_version", 1))
        self._has_opening_connectivity = schema_version >= 2
        if self._opening_connected_only and not self._has_opening_connectivity:
            rospy.logwarn(
                "opening_connected_only requested for a legacy atlas; "
                "legacy cells have no opening-connectivity evidence")

        # Container transform from atlas metadata.
        container = self._atlas.meta.get("container", {})
        self._container_xyz = [float(v) for v in container.get("base_xyz", [0, 0, 0])]
        self._container_rpy = [float(v) for v in container.get("base_rpy", [0, 0, 0])]
        self._container_R = _rpy_to_matrix(self._container_rpy)

        self._pub = rospy.Publisher(
            "/luggage/debug/reachability_atlas", MarkerArray,
            queue_size=1, latch=True)

        # Publish once (latched).
        rospy.sleep(0.5)
        self._publish()
        rospy.loginfo("Reachability atlas viz published (latched).")

    def _container_to_base(self, x, y, z):
        R = self._container_R
        return (
            self._container_xyz[0] + R[0][0] * x + R[0][1] * y + R[0][2] * z,
            self._container_xyz[1] + R[1][0] * x + R[1][1] * y + R[1][2] * z,
            self._container_xyz[2] + R[2][0] * x + R[2][1] * y + R[2][2] * z,
        )

    def _cell_state(self, x, y, z, yaw):
        """Return (status, opening_connected) through the public atlas API."""
        if hasattr(self._atlas, "query"):
            result = self._atlas.query(
                x, y, z, yaw, yaw_tolerance=float("inf"))
            return int(result.status), bool(result.opening_connected)

        # Compatibility with an older ReachabilityAtlas implementation.
        reachable, _ = self._atlas.is_reachable(x, y, z, yaw)
        if not reachable:
            return UNREACHABLE, False
        if self._atlas.is_marginal(
                x, y, z, yaw, threshold=self._marginal_threshold):
            return MARGINAL, False
        return REACHABLE, False

    def _publish(self):
        nx, ny, nz, nyaw = self._atlas.grid_size
        res = self._atlas.resolution
        origin = self._atlas.meta["grid"]["origin"]
        yaw_bins = self._atlas.yaw_bins
        # Collect points by category.
        green_pts, yellow_pts, red_pts, gray_pts = [], [], [], []

        for ix in range(nx):
            for iy in range(ny):
                for iz in range(nz):
                    for iyaw in range(nyaw):
                        if self._yaw_filter >= 0 and iyaw != self._yaw_filter:
                            continue
                        x = origin[0] + (ix + 0.5) * res
                        y = origin[1] + (iy + 0.5) * res
                        z = origin[2] + (iz + 0.5) * res
                        status, opening_connected = self._cell_state(
                            x, y, z, yaw_bins[iyaw])
                        if self._opening_connected_only and not opening_connected:
                            continue

                        point = self._container_to_base(x, y, z)
                        if status == REACHABLE and self._show_reachable:
                            green_pts.append(point)
                        elif status == MARGINAL and self._show_marginal:
                            yellow_pts.append(point)
                        elif status == UNREACHABLE and self._show_unreachable:
                            red_pts.append(point)
                        elif status == UNKNOWN and self._show_unknown:
                            gray_pts.append(point)

        ma = MarkerArray()
        stamp = rospy.Time.now()
        sphere_diam = res * 0.5

        for ns, pts, color in (
            ("atlas_reachable", green_pts, ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.6)),
            ("atlas_marginal", yellow_pts, ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.5)),
            ("atlas_unreachable", red_pts, ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3)),
            ("atlas_unknown", gray_pts, ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.3)),
        ):
            if not pts:
                continue
            m = Marker()
            m.header.frame_id = self._base_frame
            m.header.stamp = stamp
            m.ns = ns
            m.id = 0
            m.type = Marker.SPHERE_LIST
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = sphere_diam
            m.scale.y = sphere_diam
            m.scale.z = sphere_diam
            m.color = color
            m.lifetime = rospy.Duration(0)
            for px, py, pz in pts:
                m.points.append(Point(x=px, y=py, z=pz))
            ma.markers.append(m)

        self._pub.publish(ma)
        rospy.loginfo(
            "Atlas viz: %d reachable, %d marginal, %d unreachable, %d unknown",
            len(green_pts), len(yellow_pts), len(red_pts), len(gray_pts))


def main():
    rospy.init_node("reachability_atlas_viz")
    ReachabilityAtlasViz()
    rospy.spin()


if __name__ == "__main__":
    main()
