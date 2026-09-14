"""Deterministic tests for D555 acquisition-clock mapping."""

from luggage_perception.device_clock import DeviceClockMapper


def test_equal_device_stamps_map_identically_across_receipt_times():
    mapper = DeviceClockMapper(min_samples=1)
    first = mapper.map(1000000000, 11000000000)
    duplicate = mapper.map(1000000000, 11050000000)
    assert duplicate.mapped_host_ns == first.mapped_host_ns
    assert duplicate.samples == 1


def test_warmup_identity_cannot_become_publishable_when_replayed_late():
    mapper = DeviceClockMapper(min_samples=3)
    warming = mapper.map(100, 1000)
    mapper.map(200, 1100)
    assert mapper.map(300, 1200).quality == "locked"
    duplicate = mapper.map(100, 2000)
    assert warming.quality == "warming"
    assert duplicate.quality == "warming"


def test_device_intervals_are_preserved_when_receipt_jitter_grows():
    mapper = DeviceClockMapper(min_samples=1)
    first = mapper.map(1000000000, 11000000000)
    second = mapper.map(1033333333, 11100000000)
    assert second.mapped_host_ns - first.mapped_host_ns == 33333333
    assert second.estimated_delay_ns > first.estimated_delay_ns


def test_offset_correction_is_bounded_and_mapping_remains_monotonic():
    mapper = DeviceClockMapper(min_samples=1, max_offset_slew_ns=1000000)
    first = mapper.map(1000000000, 11000000000)
    second = mapper.map(1030000000, 10030000000)
    assert first.mapped_host_ns < second.mapped_host_ns
    assert mapper.offset_ns == 9999000000


def test_device_rollback_starts_warmup_epoch_and_clears_identity_cache():
    mapper = DeviceClockMapper(min_samples=2, rollback_ns=100)
    mapper.map(1000, 10000)
    locked = mapper.map(1200, 10200)
    assert locked.quality == "locked"
    reset = mapper.map(10, 20000)
    assert reset.epoch == 1
    assert reset.quality == "warming"
    assert mapper.resets == 1


def test_cache_is_bounded():
    mapper = DeviceClockMapper(min_samples=1, cache_size=3)
    for value in range(10):
        mapper.map(value, value + 100)
    assert mapper.diagnostics()["cache_entries"] == 3
