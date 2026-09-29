"""Planning-side v2 adapters, opt-in and separate from all existing v1 consumers."""
from luggage_description.frame_contracts import ContractError
from luggage_description.frame_diagnostics import at_boundary
from luggage_description.ros_frame_bindings.poses import (
    pose_from_message, pose_to_message, waypoints_from_message, waypoints_to_message,
)
from luggage_description.ros_frame_bindings.joints import joints_from_message, joints_to_message


PROFILES = ('slot', 'map_tcp', 'motion', 'base_tool')


def _profile(profile):
    if profile not in PROFILES:
        raise ContractError('UNKNOWN_PROFILE', 'planning_profile', PROFILES, profile)


def planning_pose_to_message(record, profile, context):
    at_boundary(record, context, lambda: _profile(profile))
    return pose_to_message(record, profile, context)


def planning_pose_from_message(message, profile, context):
    at_boundary({}, context, lambda: _profile(profile))
    return pose_from_message(message, profile, context)


def cartesian_path_to_message(record, profile, context):
    at_boundary(record, context, lambda: _profile(profile))
    return waypoints_to_message(record, profile, context)


def cartesian_path_from_message(message, profile, context):
    at_boundary({}, context, lambda: _profile(profile))
    return waypoints_from_message(message, profile, context)


def joint_path_to_message(record, context):
    return joints_to_message(record, context)


def joint_path_from_message(message, context):
    return joints_from_message(message, context)
