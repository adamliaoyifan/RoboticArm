#!/usr/bin/env python3
"""Convert tune-chain mount joints to production fixed-joint xyz/rpy."""

from __future__ import division

import math


def _euler_xyz_from_matrix(rot):
    """URDF fixed-joint rpy (extrinsic roll-pitch-yaw about X, Y, Z)."""
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


def _rot_to_matrix(rotation):
    """SciPy >=1.4 uses as_matrix(); older ROS Noetic builds use as_dcm()."""
    if hasattr(rotation, "as_matrix"):
        return rotation.as_matrix()
    if hasattr(rotation, "as_dcm"):
        return rotation.as_dcm()
    raise AttributeError("Rotation object has neither as_matrix() nor as_dcm()")


try:
    from scipy.spatial.transform import Rotation as Rot

    def _rotation_matrix(rx, ry, rz):
        # URDF tune chain cam_mount_rx -> ry -> rz equals fixed-joint rpy (extrinsic xyz).
        return (
            _rot_to_matrix(Rot.from_euler("z", rz))
            @ _rot_to_matrix(Rot.from_euler("y", ry))
            @ _rot_to_matrix(Rot.from_euler("x", rx))
        )

    def _matrix_to_rpy(rot):
        # Manual decomposition avoids SciPy gimbal-lock warnings at pitch ±90°.
        return _euler_xyz_from_matrix(rot)

except ImportError:

    def _rot_axis(angle, axis):
        c, s = math.cos(angle), math.sin(angle)
        if axis == "x":
            return [[1, 0, 0], [0, c, -s], [0, s, c]]
        if axis == "y":
            return [[c, 0, s], [0, 1, 0], [-s, 0, c]]
        return [[c, -s, 0], [s, c, 0], [0, 0, 1]]

    def _matmul(a, b):
        return [
            [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)
        ]

    def _rotation_matrix(rx, ry, rz):
        return _matmul(_rot_axis(rz, "z"), _matmul(_rot_axis(ry, "y"), _rot_axis(rx, "x")))

    def _matrix_to_rpy(rot):
        return _euler_xyz_from_matrix(rot)


def tune_joints_to_fixed_mount(tx, ty, tz, rx, ry, rz):
    """Map cam_mount_tx..rz joint values to URDF fixed-joint xyz/rpy."""
    rot = _rotation_matrix(float(rx), float(ry), float(rz))
    xyz = [float(tx), float(ty), float(tz)]
    rpy = _matrix_to_rpy(rot)
    return xyz, rpy


def _fixed_rpy_matrix(roll, pitch, yaw):
    """Return the rotation matrix for URDF fixed-joint rpy."""
    return _rotation_matrix(float(roll), float(pitch), float(yaw))


def _max_matrix_delta(a, b):
    return max(abs(float(a[i][j]) - float(b[i][j])) for i in range(3) for j in range(3))


def rotation_matrices_equivalent(tune_joints, fixed_rpy, tolerance=1e-4):
    """Compare tune-chain rx/ry/rz with production fixed-joint rpy by rotation matrix."""
    _, _, _, rx, ry, rz = [float(v) for v in tune_joints]
    roll, pitch, yaw = [float(v) for v in fixed_rpy]
    tune_rot = _rotation_matrix(rx, ry, rz)
    fixed_rot = _fixed_rpy_matrix(roll, pitch, yaw)
    return _max_matrix_delta(tune_rot, fixed_rot) <= tolerance


def mount_dict_to_tune_joints(mount):
    """Return [tx, ty, tz, rx, ry, rz] from mount yaml section."""
    if mount is None:
        raise ValueError("mount section missing")

    if "tune_joints" in mount:
        joints = mount["tune_joints"]
        if isinstance(joints, dict):
            keys = ["tx", "ty", "tz", "rx", "ry", "rz"]
            return [float(joints[k]) for k in keys]
        return [float(v) for v in joints]

    xyz = [float(v) for v in mount["xyz"]]
    rpy = [float(v) for v in mount["rpy"]]
    return xyz + rpy


def mount_dict_to_fixed(mount):
    """Return production fixed-joint xyz/rpy from mount yaml section."""
    if mount is None:
        raise ValueError("mount section missing")

    if "fixed" in mount:
        fixed = mount["fixed"]
        return (
            [float(v) for v in fixed["xyz"]],
            [float(v) for v in fixed["rpy"]],
        )

    joints = mount_dict_to_tune_joints(mount)
    return tune_joints_to_fixed_mount(*joints)


def build_mount_yaml(parent_link, tune_joints, fixed_xyz=None, fixed_rpy=None):
    """Build normalized mount yaml dict with tune + fixed sections."""
    tx, ty, tz, rx, ry, rz = [float(v) for v in tune_joints]
    if fixed_xyz is None or fixed_rpy is None:
        fixed_xyz, fixed_rpy = tune_joints_to_fixed_mount(tx, ty, tz, rx, ry, rz)

    return {
        "mount": {
            "parent_link": parent_link,
            "tune_joints": {
                "tx": tx,
                "ty": ty,
                "tz": tz,
                "rx": rx,
                "ry": ry,
                "rz": rz,
            },
            "fixed": {
                "xyz": [float(v) for v in fixed_xyz],
                "rpy": [float(v) for v in fixed_rpy],
            },
            "xyz": [float(v) for v in fixed_xyz],
            "rpy": [float(v) for v in fixed_rpy],
        }
    }
