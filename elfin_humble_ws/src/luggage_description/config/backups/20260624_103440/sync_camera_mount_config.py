#!/usr/bin/env python3
"""Regenerate camera_mount_origin.xacro from mount yaml (tune joints -> fixed URDF)."""

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rospkg
import yaml

from mount_config_utils import build_mount_yaml, mount_dict_to_fixed, mount_dict_to_tune_joints


def _config_dir():
    return os.path.join(rospkg.RosPack().get_path("luggage_description"), "config")


def _write_xacro(parent_link, fixed_xyz, fixed_rpy):
    path = os.path.join(_config_dir(), "camera_mount_origin.xacro")
    body = (
        '<?xml version="1.0"?>\n'
        "<!-- D435 side mount on elfin_link6. fixed.* from tune_joints via sync_camera_mount_config.py -->\n"
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
        '  <xacro:property name="cam_mount_parent" value="%s"/>\n'
        '  <xacro:property name="cam_mount_xyz" value="%.6f %.6f %.6f"/>\n'
        '  <xacro:property name="cam_mount_rpy" value="%.8f %.8f %.8f"/>\n'
        "</robot>\n"
        % (
            parent_link,
            fixed_xyz[0],
            fixed_xyz[1],
            fixed_xyz[2],
            fixed_rpy[0],
            fixed_rpy[1],
            fixed_rpy[2],
        )
    )
    with open(path, "w") as handle:
        handle.write(body)
    return path


def _sync_d435_yaml(parent_link, mount_body):
    path = os.path.join(_config_dir(), "realsense_d435.yaml")
    with open(path, "r") as handle:
        config = yaml.safe_load(handle) or {}
    config.setdefault("camera", {})["mount"] = {
        "parent_link": parent_link,
        "tune_joints": mount_body["mount"]["tune_joints"],
        "fixed": mount_body["mount"]["fixed"],
        "xyz": mount_body["mount"]["fixed"]["xyz"],
        "rpy": mount_body["mount"]["fixed"]["rpy"],
    }
    with open(path, "w") as handle:
        yaml.safe_dump(config, handle, default_flow_style=False, sort_keys=False)


def main():
    mount_yaml = os.path.join(_config_dir(), "realsense_d435_mount.yaml.example")
    with open(mount_yaml, "r") as handle:
        data = yaml.safe_load(handle) or {}
    mount = data.get("mount", {})
    parent = mount.get("parent_link", "elfin_link6")

    tune_joints = mount_dict_to_tune_joints(mount)
    fixed_xyz, fixed_rpy = mount_dict_to_fixed(mount)
    mount_body = build_mount_yaml(parent, tune_joints, fixed_xyz, fixed_rpy)
    mount_body["meta"] = data.get("meta", {"name": "camera_mount_intel_frame", "version": "1.1.0"})

    with open(mount_yaml, "w") as handle:
        yaml.safe_dump(mount_body, handle, default_flow_style=False, sort_keys=False)

    xacro_path = _write_xacro(parent, fixed_xyz, fixed_rpy)
    _sync_d435_yaml(parent, mount_body)

    print("Synced mount config:")
    print("  tune_joints:", tune_joints)
    print("  fixed xyz:", [round(v, 6) for v in fixed_xyz])
    print("  fixed rpy:", [round(v, 8) for v in fixed_rpy])
    print("  wrote:", mount_yaml)
    print("  wrote:", xacro_path)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("ERROR:", exc, file=sys.stderr)
        sys.exit(1)
