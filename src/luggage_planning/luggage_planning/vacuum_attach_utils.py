"""Pure helpers for vacuum pickup attach checks and pose composition."""

from __future__ import division

import math


def _quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def _quat_rotate(q, vector):
    x, y, z, w = q
    q_vec = [x, y, z]
    uv = [
        q_vec[1] * vector[2] - q_vec[2] * vector[1],
        q_vec[2] * vector[0] - q_vec[0] * vector[2],
        q_vec[0] * vector[1] - q_vec[1] * vector[0],
    ]
    uuv = [
        q_vec[1] * uv[2] - q_vec[2] * uv[1],
        q_vec[2] * uv[0] - q_vec[0] * uv[2],
        q_vec[0] * uv[1] - q_vec[1] * uv[0],
    ]
    scale = 2.0 * w
    return [
        vector[0] + scale * uv[0] + 2.0 * uuv[0],
        vector[1] + scale * uv[1] + 2.0 * uuv[1],
        vector[2] + scale * uv[2] + 2.0 * uuv[2],
    ]


def _quat_conjugate(q):
    return [-q[0], -q[1], -q[2], q[3]]


def pose_to_lists(position, orientation):
    return (
        [position.x, position.y, position.z],
        [orientation.x, orientation.y, orientation.z, orientation.w],
    )


def lists_to_pose(translation, quaternion):
    from geometry_msgs.msg import Pose, Point, Quaternion

    pose = Pose()
    pose.position = Point(x=translation[0], y=translation[1], z=translation[2])
    pose.orientation = Quaternion(
        x=quaternion[0],
        y=quaternion[1],
        z=quaternion[2],
        w=quaternion[3],
    )
    return pose


def compose_transform(parent_t, parent_q, child_t, child_q):
    """Return world pose of child given parent pose and child pose in parent frame."""
    rotated = _quat_rotate(parent_q, child_t)
    world_t = [parent_t[i] + rotated[i] for i in range(3)]
    world_q = _quat_multiply(parent_q, child_q)
    return world_t, world_q


def invert_transform(translation, quaternion):
    inv_q = _quat_conjugate(quaternion)
    inv_t = _quat_rotate(inv_q, [-translation[0], -translation[1], -translation[2]])
    return inv_t, inv_q


def contact_distance(panel_xyz, box_xyz, box_size, extra_margin=0.05):
    """Euclidean distance between the suction panel and the box center."""
    del box_size, extra_margin
    dx = panel_xyz[0] - box_xyz[0]
    dy = panel_xyz[1] - box_xyz[1]
    dz = panel_xyz[2] - box_xyz[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def contact_ok(panel_xyz, box_xyz, box_size, extra_margin=0.05):
    """True when the panel is close enough to the box for vacuum attach."""
    half_diag = 0.5 * math.sqrt(
        float(box_size[0]) ** 2
        + float(box_size[1]) ** 2
        + float(box_size[2]) ** 2
    )
    return contact_distance(panel_xyz, box_xyz, box_size, extra_margin) <= (
        half_diag + float(extra_margin)
    )


def top_face_gap(panel_xyz, box_xyz, box_size):
    """Signed distance from the panel to the box top face (world +Z)."""
    return float(panel_xyz[2]) - (
        float(box_xyz[2]) + 0.5 * float(box_size[2]))


def top_face_contact_ok(panel_xyz, box_xyz, box_size,
                        xy_margin=0.05, gap_min=-0.01, gap_max=0.05):
    """True when the panel is over the top face and within the gap band.

    Returns ``(ok, gap)``. XY is axis-aligned in world (spawned boxes sit
    yaw-aligned on the platform). ``gap`` is panel_z minus the top face.
    """
    dx = abs(float(panel_xyz[0]) - float(box_xyz[0]))
    dy = abs(float(panel_xyz[1]) - float(box_xyz[1]))
    in_footprint = (
        dx <= 0.5 * float(box_size[0]) + float(xy_margin)
        and dy <= 0.5 * float(box_size[1]) + float(xy_margin))
    gap = top_face_gap(panel_xyz, box_xyz, box_size)
    ok = in_footprint and float(gap_min) <= gap <= float(gap_max)
    return ok, gap
