"""Byte compatibility with the registered legacy identity recipe."""

from dataclasses import replace
import hashlib
from types import SimpleNamespace

import pytest
from luggage_planning.planning_contract._base import digest_of

from luggage_perception.occupancy_content_digest import (
    _update_tuple_part,
    content_digest,
    norm_height,
)
from luggage_perception.occupancy_map_store import OccupancyMapStore
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper


class _BytesSink:
    def __init__(self):
        self.bytes = bytearray()

    def update(self, value):
        self.bytes.extend(value)


@pytest.mark.parametrize("count", [0, 1, 2, 511, 512, 513, 1024, 1025])
@pytest.mark.parametrize("chunk_size", [1, 2, 3, 511, 512, 513])
def test_streamed_tuple_has_exact_legacy_bytes(count, chunk_size):
    # Nested tuples and punctuation must not be mistaken for outer delimiters.
    records = tuple(
        (index, "箱),\n'\\", (None, -0.0, True, float("inf"), float("nan")))
        for index in range(count)
    )
    sink = _BytesSink()
    _update_tuple_part(sink, iter(records), chunk_size)
    assert sink.bytes == repr(records).encode("utf-8") + b"\0"


def _legacy_digest(snapshot):
    # Frozen b33 recipe. Keep height normalization independent of the helper
    # under test so an accidental rounding change cannot pass this oracle.
    columns = tuple(
        (snapshot.state[ix][iy], snapshot.confidence[ix][iy],
         None if snapshot.height[ix][iy] is None else
         round(float(snapshot.height[ix][iy]), 6) + 0.0)
        for ix in range(snapshot.nx) for iy in range(snapshot.ny)
    )
    ledger = tuple(sorted(
        (center, size, yaw)
        for center, size, yaw, _source, _anchor in snapshot.ledger
    ))
    return digest_of(
        snapshot.geometry_hash, snapshot.nx, snapshot.ny,
        snapshot.resolution, snapshot.inner_size, columns, ledger,
        (snapshot.occupancy_confirmed, snapshot.change_event,
         snapshot.pending_changes),
    )


@pytest.mark.parametrize("shape", [(0, 0), (1, 1), (2, 1), (3, 171), (149, 197)])
def test_complete_digest_matches_independent_legacy_recipe(shape):
    nx, ny = shape
    heights = (None, -0.0, -0.0000001, .12345649, .12345651, 1.7)
    snapshot = SimpleNamespace(
        geometry_hash="几何\0identity", nx=nx, ny=ny, resolution=.01,
        inner_size=(1.49, 1.97, 1.48),
        state=tuple(tuple((ix + iy) % 4 for iy in range(ny)) for ix in range(nx)),
        confidence=tuple(tuple((ix + iy) % 100 / 100.0 for iy in range(ny))
                         for ix in range(nx)),
        height=tuple(tuple(heights[(ix + iy) % len(heights)] for iy in range(ny))
                     for ix in range(nx)),
        ledger=(((.3, .2, .1), (.1, .2, .3), .5, "verify", None),
                ((.1, .1, .1), (.2, .3, .4), -.2, "measured", (1, 2, 3))),
        occupancy_confirmed=False, change_event="box_removed",
        pending_changes=(("箱\\", 123),),
    )
    assert content_digest(snapshot) == _legacy_digest(snapshot)


def test_real_frozen_snapshot_retains_digest_inclusions_and_exclusions():
    mapper = CargoVolumeMapper(
        inner_size=[1.0, 1.0, 1.0], center_base=[0.0, 0.0, .5],
        yaw=0.0, resolution=.1,
    )
    store = OccupancyMapStore(mapper, epoch="digest-test")
    original = store.snapshot()
    assert content_digest(original) == original.identity.content_digest
    assert content_digest(original) == _legacy_digest(original)
    provenance_only = replace(original, acquisition_stamp_ns=998,
                              map_source="another-source", change_token=7)
    assert content_digest(provenance_only) == content_digest(original)
    changed = replace(original, occupancy_confirmed=not original.occupancy_confirmed)
    assert content_digest(changed) != content_digest(original)
    assert content_digest(changed) == _legacy_digest(changed)


def test_height_zero_and_quantization_are_unchanged():
    assert norm_height(None) is None
    assert repr(norm_height(-0.0)) == "0.0"
    assert norm_height(.12345649) == .123456
    assert norm_height(.12345651) == .123457


def test_tuple_chunk_size_must_be_positive():
    with pytest.raises(ValueError, match="positive"):
        _update_tuple_part(hashlib.sha256(), (), 0)
