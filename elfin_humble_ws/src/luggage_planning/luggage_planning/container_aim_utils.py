#!/usr/bin/env python3
"""Geometry helpers for aiming camera_depth_optical_frame at the container opening."""

from __future__ import division

import math


def _normalize(vec):
    norm = math.sqrt(sum(v * v for v in vec))
    if norm < 1e-9:
        raise ValueError("zero-length vector")
    return [v / norm for v in vec]


def _cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def look_at_quaternion(eye, target, up=(0.0, 0.0, 1.0)):
    """Return quaternion (x,y,z,w) for optical frame: +Z from eye toward target."""
    z_axis = _normalize([target[i] - eye[i] for i in range(3)])
    up_proj = up[:]
    if abs(_dot(z_axis, up_proj)) > 0.95:
        up_proj = [0.0, 1.0, 0.0]
    x_axis = _normalize(_cross(up_proj, z_axis))
    y_axis = _cross(z_axis, x_axis)

    rot = [
        [x_axis[0], y_axis[0], z_axis[0]],
        [x_axis[1], y_axis[1], z_axis[1]],
        [x_axis[2], y_axis[2], z_axis[2]],
    ]
    trace = rot[0][0] + rot[1][1] + rot[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rot[2][1] - rot[1][2]) / s
        y = (rot[0][2] - rot[2][0]) / s
        z = (rot[1][0] - rot[0][1]) / s
    elif rot[0][0] > rot[1][1] and rot[0][0] > rot[2][2]:
        s = math.sqrt(1.0 + rot[0][0] - rot[1][1] - rot[2][2]) * 2.0
        w = (rot[2][1] - rot[1][2]) / s
        x = 0.25 * s
        y = (rot[0][1] + rot[1][0]) / s
        z = (rot[0][2] + rot[2][0]) / s
    elif rot[1][1] > rot[2][2]:
        s = math.sqrt(1.0 + rot[1][1] - rot[0][0] - rot[2][2]) * 2.0
        w = (rot[0][2] - rot[2][0]) / s
        x = (rot[0][1] + rot[1][0]) / s
        y = 0.25 * s
        z = (rot[1][2] + rot[2][1]) / s
    else:
        s = math.sqrt(1.0 + rot[2][2] - rot[0][0] - rot[1][1]) * 2.0
        w = (rot[1][0] - rot[0][1]) / s
        x = (rot[0][2] + rot[2][0]) / s
        y = (rot[1][2] + rot[2][1]) / s
        z = 0.25 * s
    return [x, y, z, w]


def optical_pose_look_at(eye, target, frame_id="elfin_base_link", up=(0.0, 0.0, 1.0)):
    """Build PoseStamped for camera_depth_optical_frame (position fixed at eye)."""
    from geometry_msgs.msg import PoseStamped, Point, Quaternion

    q = look_at_quaternion(eye, target, up=up)
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position = Point(x=eye[0], y=eye[1], z=eye[2])
    pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    return pose


def view_axis_alignment_error(eye, target, optical_z_axis):
    desired = _normalize([target[i] - eye[i] for i in range(3)])
    return math.degrees(
        math.acos(max(-1.0, min(1.0, _dot(desired, optical_z_axis))))
    )


def joint1_seed(current_joints, opening_xy, link6_xy):
    """Analytic joint1 delta to pan wrist toward opening in base XY plane."""
    dx = opening_xy[0] - link6_xy[0]
    dy = opening_xy[1] - link6_xy[1]
    desired = math.atan2(dy, dx)
    current = float(current_joints[0])
    delta = desired - current
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    seed = list(current_joints)
    seed[0] = current + delta
    return seed


def joint_delta_norm(a, b):
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def pick_closest_joint_solution(candidates, reference):
    best = None
    best_score = None
    for joints in candidates:
        if joints is None:
            continue
        score = joint_delta_norm(joints, reference)
        if best_score is None or score < best_score:
            best = list(joints)
            best_score = score
    return best, best_score


def build_joint_seeds(current, observe, opening_xy, link6_xy):
    """Return ordered IK seed joint vectors (unique)."""
    seeds = []
    seen = set()

    def add(values):
        key = tuple(round(v, 4) for v in values)
        if key not in seen:
            seen.add(key)
            seeds.append(list(values))

    add(current)
    if observe is not None:
        add(observe)
    add(joint1_seed(current, opening_xy, link6_xy))
    if observe is not None:
        add(joint1_seed(observe, opening_xy, link6_xy))
    return seeds
