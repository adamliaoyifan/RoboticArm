#!/usr/bin/env python3
"""Run xacro and patch URDF world_base pose from scene_tf."""

from __future__ import annotations

import subprocess
import sys
import xml.etree.ElementTree as ET

from luggage_description.scene_tf_config_utils import (
    default_scene_tf_config_path,
    load_scene_tf_config,
    urdf_world_base_pose,
)


def format_xyz(xyz):
    return "%g %g %g" % (float(xyz[0]), float(xyz[1]), float(xyz[2]))


def _local_tag(element):
    return element.tag.split("}", 1)[-1]


def nonfixed_joint_names(root):
    """Joint names that need /joint_states (Humble RSP ignores them otherwise)."""
    names = []
    for element in root.iter():
        if _local_tag(element) != "joint":
            continue
        joint_type = (element.get("type") or "fixed").lower()
        if joint_type in ("fixed", "unknown"):
            continue
        name = element.get("name")
        if name:
            names.append(name)
    return names


def patch_world_base(root, base_xyz, base_rpy):
    """Write ``world_base`` origin from scene_tf. Mutates ``root``."""
    for joint in root.findall("joint"):
        if joint.get("name") != "world_base":
            continue
        origin = joint.find("origin")
        if origin is None:
            origin = ET.SubElement(joint, "origin")
        origin.set("xyz", format_xyz(base_xyz))
        origin.set("rpy", format_xyz(base_rpy))
        return True
    return False


def expand_and_patch(xacro_path, config_path=None, xacro_args=None):
    """Return URDF XML with ``world_base`` matching ``urdf_world_base_pose``.

    ``xacro_args`` is a list of ``key:=value`` mappings forwarded verbatim to
    xacro (e.g. ``hardware_plugin:=gz_ros2_control/GazeboSimSystem``). When
    the expanded URDF has no ``world_base`` joint (``fixed_world:=false``),
    patching is skipped and the scene base pose is still returned so a caller
    can spawn a floating model at that pose. sim_world keeps ``fixed_world:=true``
    and spawns at the origin.
    """
    config = load_scene_tf_config(config_path or default_scene_tf_config_path())
    base_xyz, base_rpy = urdf_world_base_pose(config)
    urdf_xml = subprocess.check_output(
        ["xacro", xacro_path] + list(xacro_args or []), text=True
    )
    root = ET.fromstring(urdf_xml)
    patch_world_base(root, base_xyz, base_rpy)
    return ET.tostring(root, encoding="unicode"), (base_xyz, base_rpy)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if len(argv) < 1:
        sys.stderr.write(
            "usage: xacro_robot_with_scene_base XACRO [scene_tf.yaml] [key:=value ...]\n")
        return 1
    config_path = None
    xacro_args = []
    for extra in argv[1:]:
        if ":=" in extra:
            xacro_args.append(extra)
        elif config_path is None:
            config_path = extra
        else:
            sys.stderr.write("unexpected argument: %s\n" % extra)
            return 1
    xml, _pose = expand_and_patch(argv[0], config_path, xacro_args)
    sys.stdout.write(xml)
    return 0


if __name__ == "__main__":
    sys.exit(main())
