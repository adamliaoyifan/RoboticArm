"""Immutable measured-map provenance retained by the preparation owner.

This describes the original planning input. It is not permission to execute
against an older map; the consumer must check a fresh source and native path.
"""
from dataclasses import dataclass, fields
import re


@dataclass(frozen=True)
class OperationMapSource:
    scene_revision: str
    geometry_hash: str
    epoch: int
    map_revision: int
    observed_revision: int
    last_integrated_stamp_ns: int
    config_digest: str
    clock_domain: str
    validity_generation: int
    content_sha256: str

    def __post_init__(self):
        if (not self.scene_revision or not self.geometry_hash or self.clock_domain != 'ros'
                or any(type(v) is not int or not 0 < v < 2**64 for v in
                       (self.epoch, self.map_revision, self.observed_revision, self.validity_generation))
                or type(self.last_integrated_stamp_ns) is not int
                or not 0 < self.last_integrated_stamp_ns < 2**63
                or any(not re.fullmatch('[0-9a-f]{64}', v)
                       for v in (self.config_digest, self.content_sha256))):
            raise ValueError('INVALID_OPERATION_OBSERVER_PROVENANCE')


def map_source_to_message(source):
    from luggage_msgs.msg import OperationMapSourceV2
    result = OperationMapSourceV2()
    if source is not None:
        if type(source) is not OperationMapSource:
            raise ValueError('TYPED_OPERATION_MAP_SOURCE_REQUIRED')
        result.bound, result.producer = True, 'observer'
        for field in fields(source):
            setattr(result, field.name, getattr(source, field.name))
    return result


def map_source_from_message(message):
    if not message.bound or message.producer != 'observer':
        raise ValueError('BOUND_OBSERVER_SOURCE_REQUIRED')
    return OperationMapSource(**{f.name: getattr(message, f.name) for f in fields(OperationMapSource)})
