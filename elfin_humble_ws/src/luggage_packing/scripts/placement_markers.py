#!/usr/bin/env python3
"""RViz MarkerArray builders for placement candidates.

Visualizes the placement candidate list as:
  - a footprint rectangle (LINE_LIST) per candidate, green=feasible / red=rejected
  - a sphere at each candidate center
  - a text label with score + failure reason

Kept ROS-message only (no node logic) so it can be reused by any node.
"""

from __future__ import division

import math

MARKER_NS_FOOTPRINT = "placement_footprint"
MARKER_NS_CENTER = "placement_center"
MARKER_NS_TEXT = "placement_text"


def _color(r, g, b, a=1.0):
    from std_msgs.msg import ColorRGBA

    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def _footprint_corners(center, yaw, footprint, z):
    fl, fw = footprint
    hx, hy = fl * 0.5, fw * 0.5
    local = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    corners = []
    for lx, ly in local:
        corners.append((
            center[0] + cos_y * lx - sin_y * ly,
            center[1] + sin_y * lx + cos_y * ly,
            z,
        ))
    return corners


def _delete_all(ns):
    from visualization_msgs.msg import Marker

    marker = Marker()
    marker.ns = ns
    marker.action = Marker.DELETEALL
    return marker


def build_candidate_markers(candidates, frame_id, stamp):
    from geometry_msgs.msg import Point, Quaternion
    from visualization_msgs.msg import Marker, MarkerArray

    markers = MarkerArray()
    for ns in (MARKER_NS_FOOTPRINT, MARKER_NS_CENTER, MARKER_NS_TEXT):
        markers.markers.append(_delete_all(ns))

    for idx, cand in enumerate(candidates):
        feasible = bool(cand.get("feasible", False))
        color = _color(0.1, 0.9, 0.2, 0.9) if feasible else _color(0.95, 0.2, 0.1, 0.8)
        center = cand["center_base"]
        # Footprint drawn at the box base (center_z - height/2).
        base_z = center[2] - cand["size"][2] * 0.5
        corners = _footprint_corners(
            center, cand.get("yaw", 0.0), cand.get("footprint", cand["size"][:2]), base_z
        )

        line = Marker()
        line.header.frame_id = frame_id
        line.header.stamp = stamp
        line.ns = MARKER_NS_FOOTPRINT
        line.id = idx
        line.type = Marker.LINE_LIST
        line.action = Marker.ADD
        line.pose.orientation = Quaternion(w=1.0)
        line.scale.x = 0.028
        line.color = color
        for i in range(4):
            a = corners[i]
            b = corners[(i + 1) % 4]
            line.points.append(Point(x=a[0], y=a[1], z=a[2]))
            line.points.append(Point(x=b[0], y=b[1], z=b[2]))
        markers.markers.append(line)

        sphere = Marker()
        sphere.header.frame_id = frame_id
        sphere.header.stamp = stamp
        sphere.ns = MARKER_NS_CENTER
        sphere.id = idx
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = Point(x=center[0], y=center[1], z=center[2])
        sphere.pose.orientation = Quaternion(w=1.0)
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.08
        sphere.color = color
        markers.markers.append(sphere)

        text = Marker()
        text.header.frame_id = frame_id
        text.header.stamp = stamp
        text.ns = MARKER_NS_TEXT
        text.id = idx
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position = Point(
            x=center[0], y=center[1], z=center[2] + cand["size"][2] * 0.5 + 0.10
        )
        text.pose.orientation = Quaternion(w=1.0)
        text.scale.z = 0.09
        text.color = _color(1.0, 1.0, 1.0, 0.95)
        reach = cand.get("reachability_score", -1.0)
        reach_str = "" if reach is None or reach < 0 else " reach=%.2f" % reach
        text.text = "#%d %s s=%.2f%s\n%s" % (
            idx,
            "OK" if feasible else "X",
            cand.get("score", 0.0),
            reach_str,
            cand.get("reason", ""),
        )
        markers.markers.append(text)

    return markers
