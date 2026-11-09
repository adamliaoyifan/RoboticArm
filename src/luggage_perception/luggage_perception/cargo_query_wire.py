"""G4-A wire codec: content-addressed immutable objects + manifests.

Implements the A0 freeze (docs/plans/G4A_WIRE_DTO_A0_20261026.md) of the
G4 protocol (OCC_QUERY_G4_AB_PROTOCOL_20261026.md sections 2-3):

- Objects are content-addressed (sha256 of exact payload bytes); arrays
  carry a self-describing binary header (magic, dtype, shape); JSON
  objects are canonical (sorted keys, tight separators, UTF-8).
- A manifest is the COMPLETE reference table for one generation: FULL
  references every object; DELTA references only new objects and points
  unchanged entries at content addresses that already exist (never
  re-written). Receivers must validate the base before applying a DELTA.
- No pickle/eval/arbitrary deserialization; every length is validated
  against a bound BEFORE allocation; dtype/shape whitelists are strict.
- wire payload checksums cover every transmitted member INCLUDING
  frontier (fact_digest is identity-only and never a transport
  equivalence basis).

Pure module: no ROS imports, no cross-request mutable state.
"""

from __future__ import annotations

import hashlib
import json
import math
import struct
import uuid

import numpy as np

# v2 (2026-10-28, Q-20261028-1): provenance now carries the summary
# legs occupancy_confirmed / change_event / pending_changes -- v1
# streams are named-rejected (UNKNOWN_SCHEMA), recovery via a new
# FULL from a v2 writer; no default-value compatibility shims.
WIRE_SCHEMA_VERSION = 2
MAX_OBJECT_BYTES = 4 * 1024 * 1024
MAX_MANIFEST_BYTES = 8 * 1024 * 1024
MAX_DIMS = 3
MAX_DIM = 1 << 24

_ARRAY_MAGIC = b'CQA1'
_JSON_MAGIC = b'CQJ1'
_ALLOWED_DTYPES = {
    'u1': np.dtype('u1'), 'i4': np.dtype('i4'), 'i8': np.dtype('i8'),
    'f8': np.dtype('f8'), 'u2': np.dtype('u2'), '<u2': np.dtype('<u2'),
}


class WireError(ValueError):
    """Named transport-format failure; never falls back to guessing."""

    def __init__(self, reason, detail=''):
        super().__init__(reason)
        self.reason = reason
        self.detail = detail


def canonical_json_bytes(value) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(',', ':'),
                      ensure_ascii=False, allow_nan=False).encode('utf-8')


def sha256_hex(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


# ------------------------------------------------------------------
# Object payloads (the bytes that get content-addressed)


def _dtype_short(array) -> str:
    if array.dtype == np.dtype('bool'):
        return 'u1'
    for short, dtype in _ALLOWED_DTYPES.items():
        if array.dtype == dtype:
            return short
    raise WireError('UNSUPPORTED_DTYPE', array.dtype.str)


def encode_array_object(array) -> bytes:
    array = np.ascontiguousarray(array)
    if array.dtype == np.dtype('bool'):
        array = array.view(np.uint8)
    name = _dtype_short(array)
    shape = array.shape
    if len(shape) > MAX_DIMS or any(d > MAX_DIM for d in shape):
        raise WireError('INVALID_SHAPE', str(shape))
    header = (_ARRAY_MAGIC + name.ljust(4).encode('ascii')
              + struct.pack('<B', len(shape))
              + b''.join(struct.pack('<I', int(d)) for d in shape))
    return header + array.tobytes(order='C')


def decode_array_object(payload: bytes):
    if len(payload) < len(_ARRAY_MAGIC) + 5:
        raise WireError('TRUNCATED_OBJECT', 'array header')
    if payload[:len(_ARRAY_MAGIC)] != _ARRAY_MAGIC:
        raise WireError('BAD_MAGIC', 'array')
    offset = len(_ARRAY_MAGIC)
    name = payload[offset:offset + 4].decode('ascii', 'strict').strip()
    dtype = _ALLOWED_DTYPES.get(name)
    if dtype is None:
        raise WireError('UNSUPPORTED_DTYPE', name)
    offset += 4
    (ndim,) = struct.unpack_from('<B', payload, offset)
    offset += 1
    if ndim > MAX_DIMS:
        raise WireError('INVALID_NDIMS', str(ndim))
    dims = []
    for _ in range(ndim):
        (dim,) = struct.unpack_from('<I', payload, offset)
        offset += 4
        if dim > MAX_DIM:
            raise WireError('INVALID_DIM', str(dim))
        dims.append(int(dim))
    expected = int(np.prod(dims, dtype=np.int64)) if dims else 1
    if len(payload) - offset != expected * dtype.itemsize:
        raise WireError('LENGTH_MISMATCH',
                        '%d != %d' % (len(payload) - offset,
                                      expected * dtype.itemsize))
    return np.frombuffer(payload, dtype=dtype, count=expected,
                         offset=offset).reshape(dims)


def encode_json_object(value) -> bytes:
    return _JSON_MAGIC + canonical_json_bytes(value)


def decode_json_object(payload: bytes):
    if payload[:len(_JSON_MAGIC)] != _JSON_MAGIC:
        raise WireError('BAD_MAGIC', 'json')
    try:
        value = json.loads(payload[len(_JSON_MAGIC):].decode('utf-8'))
    except (ValueError, UnicodeDecodeError) as error:
        raise WireError('BAD_JSON', str(error)) from None
    return value


# ------------------------------------------------------------------
# Manifest


def manifest_checksum(manifest: dict) -> str:
    body = {key: value for key, value in manifest.items()
            if key != 'checksum'}
    return sha256_hex(canonical_json_bytes(body))


def validate_manifest(manifest, expected_schema=WIRE_SCHEMA_VERSION):
    if not isinstance(manifest, dict):
        raise WireError('BAD_MANIFEST', 'not an object')
    if manifest.get('wire_schema_version') != expected_schema:
        raise WireError('UNKNOWN_SCHEMA',
                        str(manifest.get('wire_schema_version')))
    required = ('producer_incarnation', 'map_epoch', 'seq', 'kind',
                'target_identity', 'geometry_hash', 'grid',
                'commit_monotonic_ns', 'entries', 'checksum')
    for key in required:
        if key not in manifest:
            raise WireError('MISSING_MANIFEST_FIELD', key)
    if manifest['kind'] not in ('FULL', 'DELTA'):
        raise WireError('BAD_KIND', str(manifest['kind']))
    if manifest['kind'] == 'DELTA':
        if (not isinstance(manifest.get('base_identity'), (list, tuple))
                or not isinstance(manifest.get('base_manifest_sha'), str)):
            raise WireError('MISSING_BASE', 'delta without base legs')
    if manifest.get('checksum') != manifest_checksum(manifest):
        raise WireError('BAD_MANIFEST_CHECKSUM')
    for name, entry in manifest['entries'].items():
        if not isinstance(entry, dict) or 'sha256' not in entry:
            raise WireError('BAD_ENTRY', name)
        for key in ('type', 'bytes'):
            if key not in entry:
                raise WireError('MISSING_ENTRY_FIELD', '%s.%s' % (name, key))
        if not 0 <= entry['bytes'] <= MAX_OBJECT_BYTES * 4:
            raise WireError('ENTRY_TOO_LARGE', name)
    try:
        uuid.UUID(str(manifest['producer_incarnation']))
        int(manifest['seq'])
    except (ValueError, TypeError):
        raise WireError('BAD_MANIFEST_IDENTITY') from None
    return True


def new_producer_incarnation() -> str:
    return str(uuid.uuid4())
