#!/usr/bin/env python3
"""Verify D435 mount: tune joints vs production fixed mount, and observe-pose constraints."""

import math
import os
import re
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from mount_config_utils import (
    mount_dict_to_fixed,
    mount_dict_to_tune_joints,
    rotation_matrices_equivalent,
    tune_joints_to_fixed_mount,
)

try:
    import yaml
except ImportError:
    yaml = None

try:
    import tf.transformations as tft

    def _euler_matrix(roll, pitch, yaw):
        return tft.euler_matrix(roll, pitch, yaw, axes="rzyx")

    def _apply_rot(rot, vec3):
        return (rot[:3, :3] @ vec3).tolist()

except ImportError:

    def _rot_axis(angle, axis):
        c, s = math.cos(angle), math.sin(angle)
        if axis == "x":
            return [[1, 0, 0], [0, c, -s], [0, s, c]]
        if axis == "y":
            return [[c, 0, s], [0, 1, 0], [-s, 0, c]]
        return [[c, -s, 0], [s, c, 0], [0, 0, 1]]

    def _matmul(a, b):
        return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

    def _euler_matrix(roll, pitch, yaw):
        return _matmul(
            _rot_axis(yaw, "z"),
            _matmul(_rot_axis(pitch, "y"), _rot_axis(roll, "x")),
        )

    def _apply_rot(rot, vec3):
        if hasattr(rot, "__array__"):
            rot = rot[:3, :3]
        v = vec3
        return [
            rot[0][0] * v[0] + rot[0][1] * v[1] + rot[0][2] * v[2],
            rot[1][0] * v[0] + rot[1][1] * v[1] + rot[1][2] * v[2],
            rot[2][0] * v[0] + rot[2][1] * v[1] + rot[2][2] * v[2],
        ]


# S20 joint static origins (xyz, rpy) + revolute Z — matches elfin_description S20.urdf.xacro
_ARM_JOINT_ORIGINS = [
    ([0.0, 0.0, 0.171], [0.0, 0.0, 0.0]),
    ([0.0, -0.2295, 0.0], [math.pi / 2, 0.0, 0.0]),
    ([-0.85, 0.0, -0.1885], [0.0, 0.0, math.pi]),
    ([0.712, 0.0, 0.0], [0.0, 0.0, 0.0]),
    ([0.0, 0.0, 0.138], [-math.pi / 2, 0.0, 0.0]),
    ([0.0, 0.0, 0.138], [math.pi / 2, 0.0, 0.0]),
]

OPTICAL_IN_CAMERA_RPY = (-math.pi / 2, 0.0, -math.pi / 2)  # optical +Z = camera +X

LENS_DOWN_DOT = -0.95
LONG_EDGE_Z_MAX = 0.1


def _parse_xacro_mount(path):
    text = open(path, "r").read()
    parent = re.search(r'cam_mount_parent" value="([^"]+)"', text)
    xyz = re.search(r'cam_mount_xyz" value="([^"]+)"', text)
    rpy = re.search(r'cam_mount_rpy" value="([^"]+)"', text)
    if not (parent and xyz and rpy):
        raise ValueError("missing mount properties in %s" % path)
    return (
        parent.group(1),
        [float(v) for v in xyz.group(1).split()],
        [float(v) for v in rpy.group(1).split()],
    )


def _load_observe_joints():
    cfg_dir = os.path.join(
        os.path.dirname(__file__), "..", "..", "luggage_description", "config"
    )
    path = os.path.join(cfg_dir, "robot_poses.yaml.example")
    if yaml is None:
        return [-2.7130, -1.3263, -1.0965, 3.9564, -4.6598, 0.4522]
    with open(path, "r") as handle:
        return [float(v) for v in yaml.safe_load(handle)["poses"]["observe"]["values"]]


def _fk_rotation_world_link6(joint_values):
    rot = _euler_matrix(0.0, 0.0, 0.0)
    if hasattr(rot, "__array__"):
        rot = rot[:3, :3]
    for (xyz, rpy), q in zip(_ARM_JOINT_ORIGINS, joint_values):
        r0 = _euler_matrix(rpy[0], rpy[1], rpy[2])
        rz = _euler_matrix(0.0, 0.0, q)
        if hasattr(r0, "__array__"):
            joint_rot = (r0 @ rz)[:3, :3]
            rot = rot @ joint_rot
        else:
            rot = _matmul(rot, _matmul(r0, rz))
    return rot


def _rotation_from_rpy(roll, pitch, yaw):
    rot = _euler_matrix(roll, pitch, yaw)
    return rot[:3, :3] if hasattr(rot, "__array__") else rot


def _check_body_frame():
    """Optical +Z aligns with camera +X (Intel D435 camera_link forward)."""
    r_opt = _rotation_from_rpy(*OPTICAL_IN_CAMERA_RPY)
    opt_z_in_cam = _apply_rot(r_opt, [0.0, 0.0, 1.0])
    ok = (
        abs(opt_z_in_cam[0] - 1.0) < 0.01
        and abs(opt_z_in_cam[1]) < 0.01
        and abs(opt_z_in_cam[2]) < 0.01
    )
    return ok, opt_z_in_cam


def _check_observe_pose(mount_rpy):
    """At observe FK: lens/camera +X ~ world -Z, camera +Y long edge horizontal."""
    r_w6 = _fk_rotation_world_link6(_load_observe_joints())
    r_mount = _rotation_from_rpy(*mount_rpy)
    if hasattr(r_w6, "__array__"):
        r_wc = r_w6 @ r_mount
    else:
        r_wc = _matmul(r_w6, r_mount)

    lens_world = _apply_rot(r_wc, [1.0, 0.0, 0.0])
    long_world = _apply_rot(r_wc, [0.0, 1.0, 0.0])

    lens_down = lens_world[0] ** 2 + lens_world[1] ** 2 + (lens_world[2] + 1.0) ** 2
    long_horiz = abs(long_world[2])

    ok = lens_world[2] < LENS_DOWN_DOT and long_horiz < LONG_EDGE_Z_MAX
    return ok, lens_world, long_world, lens_down


def _load_mount_yaml():
    cfg_dir = os.path.join(os.path.dirname(__file__), "..", "..", "luggage_description", "config")
    path = os.path.join(cfg_dir, "realsense_d435_mount.yaml.example")
    if yaml is None:
        return None
    with open(path, "r") as handle:
        return yaml.safe_load(handle).get("mount", {})


def _check_tune_vs_fixed(mount):
    tune = mount_dict_to_tune_joints(mount)
    fixed_xyz, fixed_rpy = mount_dict_to_fixed(mount)
    computed_xyz, computed_rpy = tune_joints_to_fixed_mount(*tune)
    xyz_ok = all(abs(a - b) < 1e-6 for a, b in zip(fixed_xyz, computed_xyz))
    rot_ok = rotation_matrices_equivalent(tune, fixed_rpy, tolerance=1e-4)
    return xyz_ok and rot_ok, tune, fixed_xyz, fixed_rpy, computed_xyz, computed_rpy


def main():
    pkg = os.path.join(os.path.dirname(__file__), "..", "..", "luggage_description", "config")
    xacro_path = os.path.join(pkg, "camera_mount_origin.xacro")
    parent, xyz, rpy = _parse_xacro_mount(xacro_path)
    mount = _load_mount_yaml()

    print("=== D435 mount verify ===")
    print("parent:", parent)

    if mount is not None:
        ok_sync, tune, fixed_xyz, fixed_rpy, computed_xyz, computed_rpy = _check_tune_vs_fixed(mount)
        print("\nTune joints (camera_mount_tune):", [round(v, 6) for v in tune])
        print("Fixed URDF (sim_full): xyz", [round(v, 6) for v in fixed_xyz], "rpy", [round(v, 6) for v in fixed_rpy])
        if ok_sync:
            print("OK: tune_joints and fixed mount are equivalent")
        else:
            print("FAIL: tune_joints do not match fixed mount — run sync_camera_mount_config.py or Save in tune GUI")
            print("  suggested fixed xyz:", [round(v, 6) for v in computed_xyz])
            print("  suggested fixed rpy:", [round(v, 6) for v in computed_rpy])
            sys.exit(1)

        if not all(abs(a - b) < 1e-6 for a, b in zip(xyz, fixed_xyz)):
            print("FAIL: camera_mount_origin.xacro xyz differs from yaml fixed.xyz")
            sys.exit(1)
        if not rotation_matrices_equivalent(tune, rpy, tolerance=1e-4):
            print("FAIL: camera_mount_origin.xacro rpy differs from yaml fixed.rpy")
            sys.exit(1)
        print("OK: camera_mount_origin.xacro matches yaml fixed mount")

    print("\nProduction xacro xyz:", xyz)
    print("Production xacro rpy:", rpy)

    ok_body, opt_z_cam = _check_body_frame()
    print("\nBody frame (optical +Z in camera_link):", [round(v, 4) for v in opt_z_cam])
    if ok_body:
        print("OK: optical +Z = camera +X (Intel D435 forward axis)")
    else:
        print("FAIL: expected optical +Z ~ camera +X [1,0,0]")
        sys.exit(1)

    ok_obs, lens_w, long_w, cost = _check_observe_pose(rpy)
    print("\nObserve FK (world frame):")
    print("  lens direction (camera +X):", [round(v, 4) for v in lens_w])
    print("  long axis (camera +Y):", [round(v, 4) for v in long_w])
    print("  cost:", round(cost, 6))
    if ok_obs:
        print("OK: lens ~ [0,0,-1], long edge horizontal (|long_z| < %.2f)" % LONG_EDGE_Z_MAX)
    else:
        print(
            "FAIL: at observe need lens_z < %.2f and |long_z| < %.2f"
            % (LENS_DOWN_DOT, LONG_EDGE_Z_MAX)
        )
        sys.exit(1)

    print("\nAll checks passed.")


if __name__ == "__main__":
    main()
