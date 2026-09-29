"""Named joint trajectory adapters; Cartesian labels are rejected, never inferred."""
from copy import deepcopy

from luggage_msgs.msg import JointTrajectoryContractV2
from trajectory_msgs.msg import JointTrajectoryPoint
from luggage_description.frame_contracts import ContractError
from luggage_description.frame_diagnostics import at_boundary, validate_context
from luggage_description.frame_sequence_contracts import validate_joint_trajectory
from .identity import reject_extra_fields, require_message

SCALARS = ('contract_version', 'request_id', 'scene_revision', 'time_basis', 'angle_unit')
POINT_FIELDS = ('positions', 'velocities', 'accelerations', 'effort')


def _validate(record, context):
    validate_joint_trajectory(record, context.scene_revision)
    validate_context(record, context, spatial=False)
    if context.frame_id or context.reference:
        raise ContractError('JOINT_SPACE_LABEL_MISMATCH', 'context', 'empty Cartesian labels', context)
    reject_extra_fields(record, (*SCALARS, 'stamp_ns', 'kind', 'joint_names', 'points'))
    for point in record['points']:
        reject_extra_fields(point, (*POINT_FIELDS, 'time_from_start_ns'))


def joints_to_message(record, context):
    def encode():
        _validate(record, context)
        message = JointTrajectoryContractV2()
        for name in SCALARS:
            setattr(message, name, record[name])
        message.stamp_valid = True
        message.stamp_ns = record['stamp_ns']
        message.trajectory.joint_names = list(record['joint_names'])
        points = []
        for point in record['points']:
            out = JointTrajectoryPoint()
            for field in POINT_FIELDS:
                setattr(out, field, [float(v) for v in point[field]])
            out.time_from_start.sec, out.time_from_start.nanosec = divmod(point['time_from_start_ns'], 10**9)
            points.append(out)
        message.trajectory.points = points
        return message
    return at_boundary(record, context, encode)


def joints_from_message(message, context):
    record = {}
    def decode():
        require_message(message, JointTrajectoryContractV2)
        record.update({name: getattr(message, name) for name in SCALARS})
        record.update(kind='joint_trajectory', stamp_ns=message.stamp_ns if message.stamp_valid else None)
        header = message.trajectory.header
        if header.frame_id or header.stamp.sec or header.stamp.nanosec:
            raise ContractError('JOINT_SPACE_LABEL_MISMATCH', 'trajectory.header', 'empty frame and zero unused stamp', repr(header))
        record['joint_names'] = list(message.trajectory.joint_names)
        record['points'] = []
        for point in message.trajectory.points:
            if point.time_from_start.nanosec >= 10**9:
                raise ContractError('INVALID_TRAJECTORY_TIME', 'nanosec', '<1e9', point.time_from_start.nanosec)
            data = {field: list(getattr(point, field)) for field in POINT_FIELDS}
            data['time_from_start_ns'] = point.time_from_start.sec * 10**9 + point.time_from_start.nanosec
            record['points'].append(data)
        _validate(record, context)
        return deepcopy(record)
    return at_boundary(record, context, decode)
