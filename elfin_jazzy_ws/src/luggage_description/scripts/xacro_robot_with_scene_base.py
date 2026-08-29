#!/usr/bin/env python3
"""Run xacro and patch URDF world_base pose from scene_tf."""

from __future__ import annotations

import os
import subprocess
import sys
import xml.etree.ElementTree as ET

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from scene_tf_config_utils import (  # noqa: E402
    default_scene_tf_config_path,
    load_scene_tf_config,
    urdf_world_base_pose,
)


def _parse_xyz(text):
    parts = [float(v) for v in (text or "0 0 0").split()]
    while len(parts) < 3:
        parts.append(0.0)
    return parts[:3]


def _format_xyz(xyz):
    return "%g %g %g" % (xyz[0], xyz[1], xyz[2])


def _patch_world_base(root, base_xyz, base_rpy):
    for joint in root.findall("joint"):
        if joint.get("name") != "world_base":
            continue
        origin = joint.find("origin")
        if origin is None:
            origin = ET.SubElement(joint, "origin")
        origin.set("xyz", _format_xyz(base_xyz))
        origin.set("rpy", _format_xyz(base_rpy))
        break


def main():
    if len(sys.argv) < 2:
        sys.stderr.write("usage: xacro_robot_with_scene_base.py XACRO [scene_tf.yaml]\n")
        return 1

    xacro_path = sys.argv[1]
    config_path = sys.argv[2] if len(sys.argv) > 2 else default_scene_tf_config_path()
    config = load_scene_tf_config(config_path)
    base_xyz, base_rpy = urdf_world_base_pose(config)

    urdf_xml = subprocess.check_output(
        ["xacro", xacro_path],
        text=True,
    )
    root = ET.fromstring(urdf_xml)
    _patch_world_base(root, base_xyz, base_rpy)

    sys.stdout.write(ET.tostring(root, encoding="unicode"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
