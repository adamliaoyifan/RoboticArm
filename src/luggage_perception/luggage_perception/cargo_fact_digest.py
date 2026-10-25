"""Schema-2 canonical evidence hashes owned only by QueryIndexOwner.

Each leaf binds 64 consecutive native-order evidence records. A block binds
the ordered leaf hashes, active mask and exact geometry memberships. This
is an index identity; it never replaces the legacy Store content_digest.
"""
from dataclasses import dataclass
import hashlib

import numpy as np

from luggage_perception.cargo_voxel_delta import EVIDENCE_DTYPE


RECORDS_PER_LEAF = 64
LEAF_BYTES = RECORDS_PER_LEAF * EVIDENCE_DTYPE.itemsize
BLOCK_DOMAIN = b'cargo-map-block-v2\0'


@dataclass(frozen=True)
class CanonicalFactDigest:
    record_count: int
    leaves: bytes

    @classmethod
    def build(cls, facts):
        view = memoryview(facts)
        leaves = b''.join(
            hashlib.sha256(view[start:start + LEAF_BYTES]).digest()
            for start in range(0, len(view), LEAF_BYTES))
        return cls(len(view) // EVIDENCE_DTYPE.itemsize, leaves)

    def updated(self, facts, offsets):
        if not len(offsets):
            return self
        leaves = bytearray(self.leaves)
        view = memoryview(facts)
        for leaf in np.unique(offsets // RECORDS_PER_LEAF):
            leaf = int(leaf)
            start = leaf * LEAF_BYTES
            leaves[leaf * 32:(leaf + 1) * 32] = hashlib.sha256(
                view[start:start + LEAF_BYTES]).digest()
        return CanonicalFactDigest(self.record_count, bytes(leaves))

    def block_digest(self, active, box_members):
        digest = hashlib.sha256(
            BLOCK_DOMAIN + self.record_count.to_bytes(8, 'little') + self.leaves + active)
        for identifier in sorted(box_members):
            encoded = identifier.encode('utf-8')
            digest.update(len(encoded).to_bytes(4, 'little'))
            digest.update(encoded)
            members = box_members[identifier]
            digest.update(len(members).to_bytes(4, 'little'))
            digest.update(members)
        return digest.hexdigest()

    def byte_size(self):
        return 128 + len(self.leaves)
