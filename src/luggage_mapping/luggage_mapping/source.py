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
    initialization_id: str = ""
    declared_by: str = ""
    initialization_stamp_ns: int = 0

    def __post_init__(self):
        initialized = (bool(self.initialization_id) and bool(self.declared_by)
                       and type(self.initialization_stamp_ns) is int
                       and 0 < self.initialization_stamp_ns < 2**63)
        if (bool(self.initialization_id or self.declared_by or self.initialization_stamp_ns) != initialized):
            raise ValueError('INVALID_OPERATION_INITIAL_CONDITION')
        if (type(self.observed_revision) is not int or self.observed_revision < 0
                or self.observed_revision >= 2**64
                or (self.observed_revision == 0 and not initialized)):
            raise ValueError('INVALID_OPERATION_OBSERVER_PROVENANCE')
        if (not self.scene_revision or not self.geometry_hash or self.clock_domain != 'ros'
                or any(type(v) is not int or not 0 < v < 2**64 for v in
                       (self.epoch, self.map_revision, self.validity_generation))
                or type(self.last_integrated_stamp_ns) is not int
                or not (0 <= self.last_integrated_stamp_ns < 2**63)
                or (self.last_integrated_stamp_ns == 0 and not initialized)
                or ((self.observed_revision == 0) != (self.last_integrated_stamp_ns == 0))
                or any(not re.fullmatch('[0-9a-f]{64}', v)
                       for v in (self.config_digest, self.content_sha256))):
            raise ValueError('INVALID_OPERATION_OBSERVER_PROVENANCE')


@dataclass(frozen=True)
class OperationMapRefresh:
    """Original issuance plus a separately captured current O source.

    This permits native checking, never execution by itself. The surface hash
    joins the complete immutable O surface/volume capture to its private scene.
    """
    original: OperationMapSource
    current: OperationMapSource
    surface_record_sha256: str

    def admits(self, identity, snapshot):
        if (type(self.original) is not OperationMapSource
                or type(self.current) is not OperationMapSource):
            return False
        a, b = self.original, self.current
        fixed = ('scene_revision', 'geometry_hash', 'epoch', 'config_digest',
                 'clock_domain', 'validity_generation', 'initialization_id', 'declared_by', 'initialization_stamp_ns')
        return (all(getattr(a, k) == getattr(b, k) for k in fixed)
            and (a.scene_revision, a.geometry_hash, a.map_revision) ==
                (identity.scene_revision, identity.geometry_hash, identity.map_revision)
            and (b.scene_revision, b.geometry_hash, b.map_revision) ==
                (snapshot.scene_revision, snapshot.geometry_hash, snapshot.map_revision)
            and b.map_revision >= a.map_revision and b.observed_revision >= a.observed_revision
            and b.last_integrated_stamp_ns >= a.last_integrated_stamp_ns
            and bool(re.fullmatch('[0-9a-f]{64}', self.surface_record_sha256))
            and snapshot.surface_content_hash == self.surface_record_sha256)


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
