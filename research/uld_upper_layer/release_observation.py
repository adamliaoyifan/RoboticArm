"""Normalize same-step carriage telemetry; all output poses are container absolute.

The tool link is the source suction_contact_frame, unlike the arm's link6.
Joint displacement is in metres, ordered container X/Z. This input cannot be
passed off as the six-joint arm observation or a reachability certificate.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation

from physical_observation import ObservedBody, pose_matrix, read_payload_contacts


@dataclass(frozen=True)
class ReleaseObservation:
    stamp_s: float
    joint_displacements_xz_m: tuple
    container_T_box: tuple
    container_T_tool: tuple
    lower: tuple
    contacts: tuple
    attached: bool
    constraint_present: bool
    attachment_create_count: int
    attachment_detach_count: int
    processed_commands: int
    tool_T_box_at_attach: tuple


def checked_pose(row):
    try:
        quaternion = np.array([row['orientation'][k] for k in ('x', 'y', 'z', 'w')])
        position = np.array([row['position'][k] for k in ('x', 'y', 'z')])
    except (KeyError, TypeError) as exc:
        raise ValueError('RELEASE_POSE_INCOMPLETE') from exc
    if not np.isfinite(np.r_[quaternion, position]).all() or abs(np.linalg.norm(quaternion) - 1.) > 1e-8:
        raise ValueError('RELEASE_POSE_INVALID')
    return pose_matrix(row)


class ReleaseObservationReader:
    def __init__(self, world_T_container, lower_dimensions, payload_id):
        frame = np.asarray(world_T_container, dtype=float).reshape(4, 4).copy()
        if (not np.isfinite(frame).all() or not np.allclose(frame[3], [0, 0, 0, 1]) or
                not np.allclose(frame[:3, :3].T @ frame[:3, :3], np.eye(3)) or
                not np.isclose(np.linalg.det(frame[:3, :3]), 1.)):
            raise ValueError('RELEASE_FRAME_INVALID')
        self._container_T_world = np.linalg.inv(frame)
        self._lower = {name: tuple(size) for name, size in lower_dimensions.items()}
        if not self._lower or any(len(size) != 3 or not np.isfinite(size).all() or
                min(size) <= 0 for size in self._lower.values()):
            raise ValueError('RELEASE_LOWER_DIMENSIONS_INVALID')
        self._payload_id = payload_id

    def read(self, native):
        flags = ('ready', 'attached', 'joint_positions_complete', 'contacts_component_available',
                 'attachment_joint_component_present', 'payload_pose_command_component_present')
        if any(type(native.get(name)) is not bool for name in flags):
            raise ValueError('RELEASE_NATIVE_FLAGS_INVALID')
        if not native['ready'] or not native['joint_positions_complete'] or not native['contacts_component_available']:
            raise ValueError('RELEASE_NATIVE_INCOMPLETE')
        if native['joint_names'] != ['release_x', 'release_z']:
            raise ValueError('RELEASE_JOINT_NAMES_INVALID')
        q = tuple(native['q']); stamp = native['sim_time_s']
        if (len(q) != 2 or any(isinstance(v, bool) for v in q) or isinstance(stamp, bool) or
                not np.isfinite(q).all() or not np.isfinite(stamp) or stamp < 0):
            raise ValueError('RELEASE_NATIVE_JOINTS_OR_STAMP_INVALID')
        for name in ('pose_command_count', 'attachment_create_count', 'attachment_detach_count', 'processed_commands'):
            value = native.get(name)
            if type(value) is not int or value < 0:
                raise ValueError('RELEASE_NATIVE_COUNT_INVALID:' + name)
        if native['pose_command_count'] or native['payload_pose_command_component_present']:
            raise ValueError('RELEASE_PAYLOAD_COMMAND_FORBIDDEN')
        poses = native['observed_model_poses_world']
        if set(poses) != set(self._lower):
            raise ValueError('RELEASE_LOWER_SET_CHANGED')
        lower = []
        for name, size in self._lower.items():
            if poses[name].get('valid') is not True:
                raise ValueError('RELEASE_LOWER_POSE_INVALID:' + name)
            frame = self._container_T_world @ checked_pose(poses[name]['pose'])
            lower.append(ObservedBody(name, size, tuple(frame.ravel()), stamp))
        box = self._container_T_world @ checked_pose(native['payload_pose_world'])
        tool = self._container_T_world @ checked_pose(native['link_pose_world'])
        relation = np.eye(4)
        xyz = np.asarray(native['link_T_payload_xyz']); quat = np.asarray(native['link_T_payload_quat_xyzw'])
        if xyz.shape != (3,) or quat.shape != (4,) or not np.isfinite(np.r_[xyz, quat]).all() or abs(np.linalg.norm(quat) - 1.) > 1e-8:
            raise ValueError('RELEASE_ATTACHMENT_RELATION_INVALID')
        relation[:3, 3] = xyz; relation[:3, :3] = Rotation.from_quat(quat).as_matrix()
        contacts = read_payload_contacts(native['payload_contacts'], self._payload_id, self._container_T_world)
        return ReleaseObservation(float(stamp), q, tuple(box.ravel()), tuple(tool.ravel()),
            tuple(lower), contacts, native['attached'], native['attachment_joint_component_present'],
            native['attachment_create_count'], native['attachment_detach_count'],
            native['processed_commands'], tuple(relation.ravel()))
