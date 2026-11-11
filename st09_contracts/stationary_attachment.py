"""Provenance for a stationary pre-seal box and a later confirmed TCP sample.

This value is produced only after actual vacuum and frozen-footprint admission.
It preserves both clocks; it is neither a new observation nor motion permission.
The ordinary attachment path still requires exactly simultaneous measurements.
"""
from dataclasses import asdict, dataclass

from st09_contracts._base import ContractViolation, digest_of, nonempty_str, stamp_ns


def measured_pose_digest(pose):
    return digest_of(pose.frame, pose.stamp_ns, pose.position, pose.quaternion_xyzw)


@dataclass(frozen=True)
class StationaryAttachmentEvidence:
    request_id: str
    box_id: str
    generation: int
    scene_revision: str
    box_acquisition_stamp_ns: int
    tcp_confirmation_stamp_ns: int
    box_pose_sha256: str
    tcp_pose_sha256: str
    frozen_binding_sha256: str
    actual_footprint_sha256: str
    vacuum_request_sha256: str
    vacuum_response_sha256: str
    geometry_sha256: str

    def __post_init__(self):
        for name in ('request_id', 'box_id', 'scene_revision'):
            nonempty_str(getattr(self, name), name)
        if type(self.generation) is not int or self.generation <= 0:
            raise ContractViolation('STATIONARY_ATTACHMENT_GENERATION_INVALID')
        for name in ('box_acquisition_stamp_ns', 'tcp_confirmation_stamp_ns'):
            value = getattr(self, name)
            stamp_ns(value, name)
            if value <= 0:
                raise ContractViolation('STATIONARY_ATTACHMENT_STAMP_INVALID')
        if self.tcp_confirmation_stamp_ns < self.box_acquisition_stamp_ns:
            raise ContractViolation('STATIONARY_ATTACHMENT_STAMP_ORDER_INVALID')
        for name in ('box_pose_sha256', 'tcp_pose_sha256', 'frozen_binding_sha256',
                     'actual_footprint_sha256', 'vacuum_request_sha256',
                     'vacuum_response_sha256', 'geometry_sha256'):
            value = getattr(self, name)
            if (not isinstance(value, str) or len(value) != 64
                    or any(c not in '0123456789abcdef' for c in value)):
                raise ContractViolation('STATIONARY_ATTACHMENT_DIGEST_INVALID:' + name)

    def digest(self):
        return digest_of(asdict(self))
