"""Perception-side v2 message boundary glue; algorithm records remain ROS-free.

Callers supply measured identities and an independent BoundaryContext. These
functions do not obtain TF, infer stamps, publish messages, or mutate a map.
"""
from luggage_description.ros_frame_bindings.poses import pose_from_message, pose_to_message
from luggage_description.ros_frame_bindings.surfaces import surface_from_message, surface_to_message


def detection_to_message(record, context):
    return pose_to_message(record, 'detection', context)


def detection_from_message(message, context):
    return pose_from_message(message, 'detection', context)


def contact_to_message(record, context):
    return pose_to_message(record, 'contact', context)


def contact_from_message(message, context):
    return pose_from_message(message, 'contact', context)


def measured_surface_to_message(record, context):
    return surface_to_message(record, context)


def measured_surface_from_message(message, context):
    return surface_from_message(message, context)
