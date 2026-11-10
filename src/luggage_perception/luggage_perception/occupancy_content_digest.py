"""Bounded-allocation implementation of the unchanged legacy map digest.

The compatibility recipe is SHA256 of ``repr(part).encode('utf-8') + NUL``
for each part, in order (``st09_contracts._base.digest_of``). Only the large
columns tuple is streamed. Its exact Python tuple representation, including
the empty/singleton cases, remains part of the legacy identity contract.
This module reads frozen projections; it neither owns nor changes map facts.
"""

import hashlib
from itertools import islice


# Limit temporary column records, without changing GC thresholds/scheduling.
_COLUMN_CHUNK_SIZE = 512


def norm_height(value):
    """Legacy height normalization: six decimals, positive zero, None kept."""
    if value is None:
        return None
    return round(float(value), 6) + 0.0


def _update_part(digest, part):
    digest.update(repr(part).encode("utf-8"))
    digest.update(b"\0")


def _update_tuple_part(digest, values, chunk_size=_COLUMN_CHUNK_SIZE):
    """Feed exactly repr(tuple(values)) + NUL, retaining one bounded chunk."""
    if chunk_size < 1:
        raise ValueError("chunk_size must be positive")
    iterator = iter(values)
    count = 0
    digest.update(b"(")
    while True:
        chunk = tuple(islice(iterator, chunk_size))
        if not chunk:
            break
        if count:
            digest.update(b", ")
        representation = repr(chunk)
        # Strip only the outer tuple syntax; nested record syntax is data.
        body = representation[1:-2] if len(chunk) == 1 else representation[1:-1]
        digest.update(body.encode("utf-8"))
        count += len(chunk)
    if count == 1:
        digest.update(b",")
    digest.update(b")\0")


def content_digest(snapshot, columns_state=None):
    """Hash the same geometry, columns, ledger and confirmation as legacy.

    Revision/epoch/stamps/request IDs/commit order and ledger provenance are
    excluded exactly as before. Height rounding is digest-only and does not
    change the precise heights exposed by either surface API.

    ``columns_state`` is an optional ONE-slot memo dict the caller owns.
    When the caller keys it on the frozen surface projection's identity
    (replaced on every real column change) plus the geometry parts, an
    unchanged projection resumes from a cached COPY of the hashlib
    state taken right after the columns part -- bit-exact by
    construction (SHA256 state copies are exact; the skipped work is
    regenerating 29k Python tuples + repr, ~12-33ms at 1cm).
    """
    digest = hashlib.sha256()
    for part in (
        snapshot.geometry_hash, snapshot.nx, snapshot.ny,
        snapshot.resolution, snapshot.inner_size,
    ):
        _update_part(digest, part)
    cache_key = None
    if columns_state is not None:
        cache_key = id(columns_state.get('projection')) if (
            columns_state.get('projection') is not None) else None
    resume = (columns_state or {}).get('digest_after_columns') \
        if cache_key is not None else None
    if resume is not None:
        digest = resume.copy()
    else:
        columns = (
            (snapshot.state[ix][iy], snapshot.confidence[ix][iy],
             norm_height(snapshot.height[ix][iy]))
            for ix in range(snapshot.nx) for iy in range(snapshot.ny)
        )
        _update_tuple_part(digest, columns)
        if columns_state is not None:
            columns_state['digest_after_columns'] = digest.copy()
    ledger_geometry = tuple(sorted(
        (center, size, yaw)
        for center, size, yaw, _source, _anchor in snapshot.ledger
    ))
    _update_part(digest, ledger_geometry)
    _update_part(digest, (
        snapshot.occupancy_confirmed, snapshot.change_event,
        snapshot.pending_changes,
    ))
    return digest.hexdigest()
