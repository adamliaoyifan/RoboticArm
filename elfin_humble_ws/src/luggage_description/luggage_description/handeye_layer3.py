"""Fold eye-in-hand X into Layer 3 (eef_mount_adapter -> camera_link).

OpenCV calibrateHandEye returns T_end_optical (camera points into
elfin_end_link). Production URDF only stores cam_mount_* on
eef_mount_adapter. Layers 1-2 stay CAD; d555_link -> color optical stays
the RealSense driver TF.
"""

from __future__ import division

import json
import math
import os
import re

import numpy as np

# D555 SN 419222302385, FW 7.56.37776.6014, driver /tf_static 2026-09-11.
D555_LINK_TO_COLOR_FRAME_T = (-0.000490440, -0.058778100, 0.000001840)
D555_LINK_TO_COLOR_FRAME_Q_XYZW = (
    -0.000543107,
    0.000145948,
    0.002769204,
    0.999996006,
)
# optical = ROS camera optical (Z forward, X right, Y down)
COLOR_FRAME_TO_OPTICAL_Q_XYZW = (-0.5, 0.5, -0.5, 0.5)

_CONFIG = os.path.join(os.path.dirname(__file__), "..", "config")


def rpy_to_R(roll, pitch, yaw):
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    return rz.dot(ry).dot(rx)


def R_to_rpy(rot):
    sy = math.sqrt(rot[0, 0] * rot[0, 0] + rot[1, 0] * rot[1, 0])
    if sy > 1e-6:
        roll = math.atan2(rot[2, 1], rot[2, 2])
        pitch = math.atan2(-rot[2, 0], sy)
        yaw = math.atan2(rot[1, 0], rot[0, 0])
    else:
        roll = math.atan2(-rot[1, 2], rot[1, 1])
        pitch = math.atan2(-rot[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def T_xyz_rpy(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(*rpy)
    T[:3, 3] = np.asarray(xyz, dtype=np.float64)
    return T


def T_t_q(t, q_xyzw):
    x, y, z, w = [float(v) for v in q_xyzw]
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    R = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=np.float64)
    return T


def orthonormalize(R):
    u, _s, vt = np.linalg.svd(np.asarray(R, dtype=np.float64))
    Rn = u.dot(vt)
    if np.linalg.det(Rn) < 0:
        u[:, -1] *= -1.0
        Rn = u.dot(vt)
    return Rn


def parse_xacro_xyz_rpy(path, xyz_name, rpy_name):
    text = open(path, encoding="utf-8").read()

    def grab(name):
        match = re.search(
            r'name="%s" value="([^"]+)"' % re.escape(name), text)
        if not match:
            raise ValueError("missing %s in %s" % (name, path))
        return [float(v) for v in match.group(1).split()]

    return grab(xyz_name), grab(rpy_name)


def T_end_adapter(config_dir=None):
    config_dir = config_dir or _CONFIG
    xyz1, rpy1 = parse_xacro_xyz_rpy(
        os.path.join(config_dir, "suction_flange_origin.xacro"),
        "suction_flange_xyz",
        "suction_flange_rpy",
    )
    xyz2, rpy2 = parse_xacro_xyz_rpy(
        os.path.join(config_dir, "eef_mount_adapter_origin.xacro"),
        "adapter_mount_xyz",
        "adapter_mount_rpy",
    )
    return T_xyz_rpy(xyz1, rpy1).dot(T_xyz_rpy(xyz2, rpy2))


def T_d555_optical():
    return T_t_q(D555_LINK_TO_COLOR_FRAME_T, D555_LINK_TO_COLOR_FRAME_Q_XYZW).dot(
        T_t_q((0.0, 0.0, 0.0), COLOR_FRAME_TO_OPTICAL_Q_XYZW)
    )


def average_handeye(methods):
    Rs = []
    ts = []
    for name in ("TSAI", "PARK", "DANIILIDIS"):
        blob = methods[name]
        Rs.append(np.asarray(blob["R"], dtype=np.float64))
        ts.append(np.asarray(blob["t_m"], dtype=np.float64).reshape(3))
    T = np.eye(4)
    T[:3, :3] = orthonormalize(np.mean(Rs, axis=0))
    T[:3, 3] = np.mean(ts, axis=0)
    return T


def layer3_from_end_optical(T_end_optical, config_dir=None):
    T_adp = T_end_adapter(config_dir)
    T_opt = T_d555_optical()
    return np.linalg.inv(T_adp).dot(T_end_optical).dot(np.linalg.inv(T_opt))


def layer3_xyz_rpy(T_adapter_camera):
    xyz = [float(v) for v in T_adapter_camera[:3, 3]]
    rpy = [float(v) for v in R_to_rpy(T_adapter_camera[:3, :3])]
    return xyz, rpy


def load_handeye_json(path):
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def camera_mount_xacro_text(xyz, rpy, note):
    return (
        '<?xml version="1.0"?>\n'
        "<!-- %s -->\n"
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
        '  <xacro:property name="cam_mount_parent" value="eef_mount_adapter"/>\n'
        '  <xacro:property name="cam_mount_xyz" value="%.6f %.6f %.6f"/>\n'
        '  <xacro:property name="cam_mount_rpy" value="%.8f %.8f %.8f"/>\n'
        "</robot>\n"
        % (note, xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2])
    )


def freeze_from_handeye_json(path, config_dir=None):
    blob = load_handeye_json(path)
    T_x = average_handeye(blob["methods"])
    T3 = layer3_from_end_optical(T_x, config_dir)
    xyz, rpy = layer3_xyz_rpy(T3)
    return {
        "xyz": xyz,
        "rpy": rpy,
        "T_end_optical": T_x.tolist(),
        "T_adapter_camera": T3.tolist(),
        "used_poses": blob.get("used_poses"),
        "n": blob.get("n"),
    }
