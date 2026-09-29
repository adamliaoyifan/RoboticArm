"""Complete Cartesian pose/path adapters using generated ROS messages."""
from copy import deepcopy
from dataclasses import replace

from luggage_msgs.msg import PoseContractV2, WaypointArrayV2
from luggage_description.frame_contracts import require_equal, validate_pose
from luggage_description.frame_diagnostics import at_boundary, validate_context
from luggage_description.frame_sequence_contracts import validate_waypoints
from .identity import (IDENTITY_FIELDS, identity_from_message, identity_to_message,
                       reject_extra_fields, require_message)

POSE_FIELDS = (*IDENTITY_FIELDS, 'stamp_ns', 'kind', 'reference',
               'orientation_semantics', 'position', 'quaternion_xyzw')
PATH_FIELDS = (*IDENTITY_FIELDS, 'stamp_ns', 'kind', 'geometry_hash', 'map_revision', 'waypoints')


def pose_to_message(record, profile, context):
    def encode():
        validate_pose(record, profile, context.scene_revision)
        validate_context(record, context)
        reject_extra_fields(record, POSE_FIELDS)
        message = PoseContractV2()
        message.identity = identity_to_message(record, context.scene_revision)
        message.reference = record['reference']
        message.orientation_semantics = record['orientation_semantics']
        for name, value in zip(('x', 'y', 'z'), record['position']):
            setattr(message.pose.position, name, float(value))
        for name, value in zip(('x', 'y', 'z', 'w'), record['quaternion_xyzw']):
            setattr(message.pose.orientation, name, float(value))
        return message
    return at_boundary(record, context, encode)


def pose_from_message(message, profile, context):
    record = {}
    def decode():
        require_message(message, PoseContractV2)
        record.update(identity_from_message(message.identity))
        record.update(kind='pose', reference=message.reference,
                      orientation_semantics=message.orientation_semantics,
                      position=[getattr(message.pose.position, n) for n in ('x', 'y', 'z')],
                      quaternion_xyzw=[getattr(message.pose.orientation, n) for n in ('x', 'y', 'z', 'w')])
        validate_pose(record, profile, context.scene_revision)
        validate_context(record, context)
        return deepcopy(record)
    return at_boundary(record, context, decode)


def _validate_path(record, profile, context):
    validate_waypoints(record, profile, context.scene_revision,
                       context.geometry_hash, context.map_revision)
    validate_context(record, context, spatial=False)
    require_equal(record, 'frame_id', context.frame_id, 'FRAME_CONTRACT_MISMATCH')
    reject_extra_fields(record, PATH_FIELDS)


def waypoints_to_message(record, profile, context):
    def encode():
        _validate_path(record, profile, context)
        message = WaypointArrayV2()
        message.identity = identity_to_message(record, context.scene_revision)
        message.geometry_hash = record['geometry_hash']
        message.map_revision = record['map_revision']
        message.waypoints = [pose_to_message(p, profile, replace(context, time_basis=p['time_basis']))
                             for p in record['waypoints']]
        return message
    return at_boundary(record, context, encode)


def waypoints_from_message(message, profile, context):
    record = {}
    def decode():
        require_message(message, WaypointArrayV2)
        record.update(identity_from_message(message.identity))
        record.update(kind='waypoints', geometry_hash=message.geometry_hash,
                      map_revision=message.map_revision)
        record['waypoints'] = [pose_from_message(p, profile, replace(context, time_basis=p.identity.time_basis))
                               for p in message.waypoints]
        _validate_path(record, profile, context)
        return deepcopy(record)
    return at_boundary(record, context, decode)
