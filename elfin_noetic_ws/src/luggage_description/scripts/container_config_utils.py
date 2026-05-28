#!/usr/bin/env python3
"""Load container.yaml and compute static transforms."""

from __future__ import division

import math
import os

import rospkg
import yaml


def default_config_path():
    return os.path.join(
        rospkg.RosPack().get_path("luggage_description"),
        "config",
        "container.yaml.example",
    )


def load_container_config(path=None):
    path = path or default_config_path()
    with open(path, "r") as handle:
        return yaml.safe_load(handle)


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _mat_vec(mat, vec):
    return [
        mat[0][0] * vec[0] + mat[0][1] * vec[1] + mat[0][2] * vec[2],
        mat[1][0] * vec[0] + mat[1][1] * vec[1] + mat[1][2] * vec[2],
        mat[2][0] * vec[0] + mat[2][1] * vec[1] + mat[2][2] * vec[2],
    ]


def _matmul(a, b):
    return [
        [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def _invert_transform(xyz, rpy):
    rot = _rpy_matrix(*rpy)
    inv_rot = [
        [rot[0][0], rot[1][0], rot[2][0]],
        [rot[0][1], rot[1][1], rot[2][1]],
        [rot[0][2], rot[1][2], rot[2][2]],
    ]
    t_inv = _mat_vec(inv_rot, [-xyz[0], -xyz[1], -xyz[2]])
    inv_rpy = _matrix_to_rpy(inv_rot)
    return t_inv, inv_rpy


def _matrix_to_rpy(rot):
    sy = math.sqrt(rot[0][0] * rot[0][0] + rot[1][0] * rot[1][0])
    if sy > 1e-6:
        roll = math.atan2(rot[2][1], rot[2][2])
        pitch = math.atan2(-rot[2][0], sy)
        yaw = math.atan2(rot[1][0], rot[0][0])
    else:
        roll = math.atan2(-rot[1][2], rot[1][1])
        pitch = math.atan2(-rot[2][0], sy)
        yaw = 0.0
    return [roll, pitch, yaw]


def _compose(xyz_a, rpy_a, xyz_b, rpy_b):
    rot_a = _rpy_matrix(*rpy_a)
    rot_b = _rpy_matrix(*rpy_b)
    rot = _matmul(rot_a, rot_b)
    t = _mat_vec(rot_a, xyz_b)
    t = [t[0] + xyz_a[0], t[1] + xyz_a[1], t[2] + xyz_a[2]]
    return t, _matrix_to_rpy(rot)


def origin_in_world(config):
    origin = config.get("origin", {})
    return (
        [float(v) for v in origin.get("xyz", [0.0, 0.0, 0.0])],
        [float(v) for v in origin.get("rpy", [0.0, 0.0, 0.0])],
    )


def base_in_world(config):
    base = config.get("base_in_world", {})
    return (
        [float(v) for v in base.get("xyz", [0.0, 0.0, 0.0])],
        [float(v) for v in base.get("rpy", [0.0, 0.0, 0.0])],
    )


def opening_in_container(config):
    opening = config.get("opening", {})
    frame = opening.get("frame", {})
    return (
        [float(v) for v in frame.get("xyz", [0.0, 1.0, 1.0])],
        [float(v) for v in frame.get("rpy", [0.0, 0.0, 0.0])],
    )


def container_in_base_link(config):
    """Return xyz/rpy of container_link expressed in elfin_base_link."""
    world_t, world_r = origin_in_world(config)
    base_t, base_r = base_in_world(config)
    base_inv_t, base_inv_r = _invert_transform(base_t, base_r)
    return _compose(base_inv_t, base_inv_r, world_t, world_r)


def opening_in_base_link(config):
    """Return xyz/rpy of container_opening_frame in elfin_base_link."""
    base_c_t, base_c_r = container_in_base_link(config)
    open_t, open_r = opening_in_container(config)
    return _compose(base_c_t, base_c_r, open_t, open_r)


def opening_target_point(config):
    """Opening center as [x,y,z] in elfin_base_link."""
    xyz, _ = opening_in_base_link(config)
    return xyz


def spawn_pose_from_config(config):
    """Gazebo spawn args for airport_container (world frame)."""
    xyz, rpy = origin_in_world(config)
    return {
        "x": xyz[0],
        "y": xyz[1],
        "z": xyz[2],
        "R": rpy[0],
        "P": rpy[1],
        "Y": rpy[2],
    }


def outer_box_center_in_container(config):
    """Center of outer collision box in container_link frame."""
    outer = config.get("outer", {})
    height = float(outer.get("height", 2.2))
    return [0.0, 0.0, height * 0.5]


def outer_dimensions(config):
    outer = config.get("outer", {})
    return (
        float(outer.get("length", 2.4)),
        float(outer.get("width", 2.0)),
        float(outer.get("height", 2.2)),
    )


def inner_dimensions(config):
    inner = config.get("inner", {})
    return (
        float(inner.get("length", 2.3)),
        float(inner.get("width", 1.9)),
        float(inner.get("height", 2.1)),
    )
