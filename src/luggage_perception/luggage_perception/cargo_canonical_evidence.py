"""Writer-private evidence blocks; never retained by a read generation.

QueryIndexOwner alone initializes, patches and rehashes this derived store.
Rows use the existing compact native order inside each block. Prefix offsets
enable batch lookup without padding at partial block ends.
The mapper's primary arrays and lists remain unchanged.
"""
import numpy as np

from luggage_perception.cargo_fact_digest import CanonicalFactDigest
from luggage_perception.cargo_map_query import BLOCK_EDGE
from luggage_perception.cargo_voxel_delta import EVIDENCE_DTYPE


RECORD_BYTES = np.dtype(('V', EVIDENCE_DTYPE.itemsize))


class CanonicalEvidenceBlocks:
    def __init__(self, shape):
        self._shape = tuple(shape)
        self._block_shape = tuple((n + BLOCK_EDGE - 1)//BLOCK_EDGE for n in shape)
        count = int(np.prod(self._block_shape))
        nx, ny, nz = shape
        self._lengths = np.asarray([
            min(BLOCK_EDGE, nx-x)*min(BLOCK_EDGE, ny-y)*min(BLOCK_EDGE, nz-z)
            for z in range(0, nz, BLOCK_EDGE)
            for y in range(0, ny, BLOCK_EDGE)
            for x in range(0, nx, BLOCK_EDGE)], dtype=np.int64)
        self._starts = np.r_[0, np.cumsum(self._lengths)]
        self._records = np.empty(int(self._starts[-1]), dtype=RECORD_BYTES)
        self._initialized = np.zeros(count, dtype=bool)
        self._initialized_count = 0
        self._digests = [None]*count
        self._generation = object()

    def initialize_block(self, block_id, evidence):
        if self._initialized[block_id]:
            raise RuntimeError('canonical block already initialized')
        if evidence.dtype != EVIDENCE_DTYPE or len(evidence) != self._lengths[block_id]:
            raise ValueError('invalid canonical block records')
        self._records[self._starts[block_id]:self._starts[block_id+1]] = evidence.view(RECORD_BYTES)
        self._initialized[block_id] = True
        self._initialized_count += 1
        self._digests[block_id] = CanonicalFactDigest.build(self.block_bytes(block_id))
        self._generation = object()
        return self._digests[block_id]

    def block_bytes(self, block_id):
        records = self._records[self._starts[block_id]:self._starts[block_id+1]]
        return memoryview(records).cast('B').toreadonly()

    def _locations(self, indices):
        nx, ny, nz = self._shape
        if np.any(indices < 0) or np.any(indices >= nx*ny*nz):
            raise IndexError('canonical evidence outside grid')
        x, y, z = indices % nx, indices//nx % ny, indices//(nx*ny)
        bx, by, _ = self._block_shape
        ids = x//BLOCK_EDGE + bx*(y//BLOCK_EDGE + by*(z//BLOCK_EDGE))
        sx = np.minimum(BLOCK_EDGE, nx - x//BLOCK_EDGE*BLOCK_EDGE)
        sy = np.minimum(BLOCK_EDGE, ny - y//BLOCK_EDGE*BLOCK_EDGE)
        offsets = (x % BLOCK_EDGE
                   + sx*(y % BLOCK_EDGE + sy*(z % BLOCK_EDGE)))
        return ids, offsets

    def bind_prior_reader(self):
        """Bind first-old facts until the next derived evidence mutation.

        Store finalizes its delta before QueryIndexOwner applies it. A stale
        binding is an index fault, rather than silently reading new facts.
        Every lookup returns a detached copy, never a writable owner alias.
        """
        if self._initialized_count != len(self._lengths):
            raise RuntimeError('canonical evidence initialization incomplete')
        generation = self._generation

        def read(indices):
            if self._generation is not generation:
                raise RuntimeError('STALE_PRIOR_EVIDENCE')
            ids, offsets = self._locations(indices)
            return self._records[self._starts[ids] + offsets].view(EVIDENCE_DTYPE)
        return read

    def patch(self, indices, evidence):
        if evidence.dtype != EVIDENCE_DTYPE or len(indices) != len(evidence):
            raise ValueError('invalid canonical patch records')
        ids, offsets = self._locations(indices)
        if len(indices):
            self._records[self._starts[ids] + offsets] = evidence.view(RECORD_BYTES)
            self._generation = object()
        return ids, offsets

    def rehash(self, block_id, offsets):
        self._digests[block_id] = self._digests[block_id].updated(
            self.block_bytes(block_id), offsets)
        return self._digests[block_id]

    def byte_size(self):
        # Charge the complete private allocation and conservative metadata.
        return (1024 + self._records.nbytes + 64*len(self._lengths)
                + self._starts.nbytes + self._lengths.nbytes + self._initialized.nbytes
                + sum(d.byte_size() for d in self._digests if d is not None))
