#!/usr/bin/env python3
"""RViz visualization for layout atlas union results.

Loads union.npz + atlas meta and publishes a latched MarkerArray showing:
- Green: cells covered by at least one base stop (X/Y/Z union).
- Red: blind cells (not covered by any stop).
- Yellow: marginal (covered but low confidence).

Also publishes a text marker with the summary decision and the base-movement
envelope (X/Y/Z ranges).

Usage:
    rosrun luggage_planning layout_atlas_viz.py \
        _union_npz:=$(rospack find luggage_planning)/data/layout_atlas/union.npz \
        _summary_yaml:=$(rospack find luggage_planning)/data/layout_atlas/summary.yaml \
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


def _rpy_to_matrix(rpy):
    cr, sr = math.cos(rpy[0]), math.sin(rpy[0])
    cp, sp = math.cos(rpy[1]), math.sin(rpy[1])
    cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


REACHABLE = 3
MARGINAL = 2
UNREACHABLE = 1
UNKNOWN = 0


class LayoutAtlasViz:
    def __init__(self):
        # Model-specific defaults: data/layout_atlas_<prefix>/ and
        # <prefix>_container_collision_aware.yaml, so S20/S30 auto-load the
        # right files. /robot_name comes from active_loading (arm_model).
        import layout_atlas as la
        prefix = la.model_prefix_from_robot_name(
            rospy.get_param("/robot_name", "elfin_s20_with_camera"))
        _data = os.path.join(rospkg.RosPack().get_path("luggage_planning"), "data")

        # Load union data.
        union_path = rospy.get_param(
            "~union_npz",
            os.path.join(_data, "layout_atlas_%s" % prefix, "union.npz"))
        self._union = dict(np.load(union_path, allow_pickle=False))

        # Load atlas meta (for grid + container transform). Resolution order:
        #   1. ~atlas_meta param
        #   2. the sweep's own meta.yaml (self-contained)
        #   3. the model's reachability atlas meta
        #   4. s20 atlas meta (fallback: grid/container describe container_link,
        #      which is arm-independent, so this is valid for any arm)
        meta_param = rospy.get_param("~atlas_meta", "")
        if meta_param:
            meta_path = meta_param
        else:
            candidates = [
                os.path.join(_data, "layout_atlas_%s" % prefix, "meta.yaml"),
                os.path.join(_data, "reachability_atlas",
                             "%s_container_collision_aware.yaml" % prefix),
                os.path.join(_data, "reachability_atlas",
                             "s20_container_collision_aware.yaml"),
            ]
            meta_path = next((c for c in candidates if os.path.exists(c)),
                             candidates[0])
            rospy.loginfo("Layout atlas viz: using atlas meta %s", meta_path)
        with open(meta_path, "r") as f:
            self._meta = yaml.safe_load(f)

        # Load summary (for decision text).
        summary_path = rospy.get_param(
            "~summary_yaml",
            os.path.join(_data, "layout_atlas_%s" % prefix, "summary.yaml"))
        try:
            with open(summary_path, "r") as f:
                self._summary = yaml.safe_load(f)
        except Exception:
            self._summary = None

        # Grid params.
        grid = self._meta["grid"]
        self._res = float(grid["resolution_xyz"])
        self._origin = [float(v) for v in grid["origin"]]
        self._nx, self._ny, self._nz = grid["size"]
        self._nyaw = len(grid["yaw_bins"])

        # Container transform (baseline Y=0 for visualization).
        container = self._meta.get("container", {})
        self._container_xyz = [float(v) for v in container.get("base_xyz", [0, 0, 0])]
        self._container_rpy = [float(v) for v in container.get("base_rpy", [0, 0, 0])]
        self._container_R = _rpy_to_matrix(self._container_rpy)
        self._base_frame = rospy.get_param("~base_frame", "elfin_base_link")

        # Display options.
        self._show_blind = bool(rospy.get_param("~show_blind", True))
        self._yaw_filter = int(rospy.get_param("~yaw_filter", -1))

        self._pub = rospy.Publisher(
            "/luggage/debug/layout_atlas", MarkerArray,
            queue_size=1, latch=True)

        rospy.sleep(0.5)
        self._publish()
        rospy.loginfo("Layout atlas viz published (latched).")

    def _container_to_base(self, x, y, z):
        R = self._container_R
        return (
            self._container_xyz[0] + R[0][0] * x + R[0][1] * y + R[0][2] * z,
            self._container_xyz[1] + R[1][0] * x + R[1][1] * y + R[1][2] * z,
            self._container_xyz[2] + R[2][0] * x + R[2][1] * y + R[2][2] * z,
        )

    def _publish(self):
        union_status = self._union["union_status"]
        coverage_count = self._union.get("coverage_count")
        # preferred_base_xyz is the 3-axis form; fall back to legacy preferred_base_y.
        preferred_xyz = self._union.get("preferred_base_xyz")

        green_pts, red_pts, yellow_pts = [], [], []
        for ix in range(self._nx):
            for iy in range(self._ny):
                for iz in range(self._nz):
                    for iyaw in range(self._nyaw):
                        if self._yaw_filter >= 0 and iyaw != self._yaw_filter:
                            continue
                        x = self._origin[0] + (ix + 0.5) * self._res
                        y = self._origin[1] + (iy + 0.5) * self._res
                        z = self._origin[2] + (iz + 0.5) * self._res
                        bx, by, bz = self._container_to_base(x, y, z)

                        status = int(union_status[ix, iy, iz, iyaw])
                        if status >= REACHABLE:
                            # Check if marginal (covered by only 1 stop).
                            count = int(coverage_count[ix, iy, iz, iyaw]) \
                                if coverage_count is not None else 2
                            if count <= 1:
                                yellow_pts.append((bx, by, bz))
                            else:
                                green_pts.append((bx, by, bz))
                        elif self._show_blind:
                            red_pts.append((bx, by, bz))

        ma = MarkerArray()
        stamp = rospy.Time.now()
        diam = self._res * 0.5

        for ns, pts, color in (
            ("union_covered", green_pts, ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.6)),
            ("union_blind", red_pts, ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.25)),
            ("union_marginal", yellow_pts, ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.5)),
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
            m.scale.x = diam
            m.scale.y = diam
            m.scale.z = diam
            m.color = color
            m.lifetime = rospy.Duration(0)
            for px, py, pz in pts:
                m.points.append(Point(x=px, y=py, z=pz))
            ma.markers.append(m)

        # Summary text marker.
        if self._summary:
            decision = self._summary.get("decision", {})
            best = self._summary.get("best_fixed", {})
            union = self._summary.get("union", {})
            envelope = self._summary.get("base_movement_envelope", {}) or {}

            best_off = best.get("offset", [0.0, best.get("base_y", 0.0), 0.0])
            best_y = best_off[1] if len(best_off) > 1 else best.get("base_y", 0.0)

            text = "Layout Atlas (X/Y/Z): %s\n" % decision.get("recommendation", "?")
            text += "Best fixed off=%s coverage=%.1f%%\n" % (
                [round(float(v), 2) for v in best_off],
                best.get("score", {}).get("coverage_rate", 0) * 100)
            text += "Union=%.1f%% (%d stops, %s)\n" % (
                union.get("coverage_rate", 0) * 100,
                len(self._summary.get("selected_stops", [])),
                union.get("mode", "?"))
            text += "Blind=%s cells\n" % union.get("remaining_blind", "?")

            def _fmt(axis):
                e = envelope.get(axis, {}) or {}
                if e.get("min") is None:
                    return "%s: n/a" % axis.upper()
                return "%s[%.2f,%.2f]" % (axis.upper(), e["min"], e["max"])

            text += "Base range: %s %s %s" % (
                _fmt("x"), _fmt("y"), _fmt("z"))
            if "reason" in decision:
                text += "\n%s" % decision["reason"]

            tm = Marker()
            tm.header.frame_id = self._base_frame
            tm.header.stamp = stamp
            tm.ns = "layout_summary"
            tm.id = 0
            tm.type = Marker.TEXT_VIEW_FACING
            tm.action = Marker.ADD
            tm.pose.position = Point(x=0.0, y=0.0, z=2.5)
            tm.pose.orientation.w = 1.0
            tm.scale.z = 0.12
            tm.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            tm.text = text
            tm.lifetime = rospy.Duration(0)
            ma.markers.append(tm)

        self._pub.publish(ma)
        rospy.loginfo("Union: %d covered, %d blind, %d marginal",
                      len(green_pts), len(red_pts), len(yellow_pts))


def main():
    rospy.init_node("layout_atlas_viz")
    LayoutAtlasViz()
    rospy.spin()


if __name__ == "__main__":
    main()
