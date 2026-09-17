"""Named joint poses for GoToRobotPose (ROS-free).

``current`` / ``here`` means stay at the live /joint_states. Site YAML must
not copy simulation ``pickup_observe`` values from
``robot_poses.yaml.example``.
"""

from __future__ import division

import os

import yaml

JOINTS = [
    "elfin_joint1",
    "elfin_joint2",
    "elfin_joint3",
    "elfin_joint4",
    "elfin_joint5",
    "elfin_joint6",
]

CURRENT_POSE_NAMES = frozenset(("current", "here"))


def is_current_pose_name(name):
    return str(name or "").strip().lower() in CURRENT_POSE_NAMES


def load_poses_config(path):
    if not path:
        raise ValueError("robot_poses_config path is empty")
    with open(path, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}
    if not isinstance(config, dict):
        raise ValueError("robot_poses_config is not a mapping: %s" % path)
    return config


def default_observe_pose_name(config, override=""):
    name = str(override or "").strip()
    if name:
        return name
    defaults = config.get("defaults") or {}
    name = str(defaults.get("observe_pose") or "current").strip()
    return name or "current"


def resolve_pose_name(requested, config, param_override=""):
    name = str(requested or "").strip()
    if name:
        return name
    return default_observe_pose_name(config, override=param_override)


def named_pose_joints(config, pose_name):
    """Return 6 joint radians, or None when pose_name means current.

    Raises KeyError if the named pose is missing, ValueError if values
    are the wrong length.
    """
    if is_current_pose_name(pose_name):
        return None
    poses = config.get("poses") or {}
    if pose_name not in poses:
        raise KeyError(
            "pose %r not in robot_poses (defaults.observe_pose=%s). "
            "Use current to stay, or fill poses.%s.values from a measured "
            "dump."
            % (pose_name,
               default_observe_pose_name(config),
               pose_name))
    values = [float(v) for v in poses[pose_name]["values"]]
    if len(values) != len(JOINTS):
        raise ValueError(
            "pose %r has %d values, need %d"
            % (pose_name, len(values), len(JOINTS)))
    return values


def plan_goto_joints(pose_name, config, current_joints):
    """Joints to command, or None to stay (no FJT).

    ``current`` / ``here`` requires live ``current_joints``.
    """
    if is_current_pose_name(pose_name):
        if current_joints is None:
            raise RuntimeError(
                "pose %r needs /joint_states" % pose_name)
        if len(current_joints) != len(JOINTS):
            raise ValueError(
                "current joints length %d, need %d"
                % (len(current_joints), len(JOINTS)))
        return None
    return named_pose_joints(config, pose_name)


def format_pose_yaml(pose_name, values, observe_default=None):
    """YAML snippet to paste under poses.<name>."""
    nums = ", ".join("%.6f" % float(v) for v in values)
    lines = [
        "  %s:" % pose_name,
        "    group: elfin_arm",
        "    joints:",
    ]
    for joint in JOINTS:
        lines.append("      - %s" % joint)
    lines.append("    values: [%s]" % nums)
    lines.append("    tolerance: 0.15")
    if observe_default:
        lines.extend([
            "",
            "defaults:",
            "  observe_pose: %s" % observe_default,
        ])
    return "\n".join(lines) + "\n"


def site_poses_share_path():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory("luggage_description"),
        "config",
        "robot_poses.site.yaml",
    )
