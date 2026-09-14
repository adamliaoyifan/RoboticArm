"""ROS-free device-acquisition to host-clock mapping.

The D555 driver exposes a monotonic hardware clock.  Receipt time is not an
acquisition timestamp, but it is an observation of the clock offset plus a
non-negative transport delay.  This mapper preserves device-clock intervals
and follows only the low-delay envelope, with bounded offset slew.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass


@dataclass(frozen=True)
class ClockMapping:
    device_ns: int
    mapped_host_ns: int
    receipt_host_ns: int
    epoch: int
    samples: int
    quality: str
    estimated_delay_ns: int


class DeviceClockMapper:
    """Map one monotonic device clock into the host ROS-time domain.

    Duplicate device stamps are cached so independently arriving colour and
    aligned-depth messages receive bit-identical mapped stamps.  A substantial
    device rollback starts a new epoch and re-enters warm-up.
    """

    def __init__(self, min_samples=5, rollback_ns=500000000,
                 max_offset_slew_ns=1000000, cache_size=256):
        self.min_samples = max(1, int(min_samples))
        self.rollback_ns = max(1, int(rollback_ns))
        self.max_offset_slew_ns = max(0, int(max_offset_slew_ns))
        self.cache_size = max(2, int(cache_size))
        self.epoch = 0
        self.samples = 0
        self.resets = 0
        self._offset_ns = None
        self._last_device_ns = None
        self._last_mapped_ns = None
        self._cache = OrderedDict()

    @property
    def quality(self):
        return "locked" if self.samples >= self.min_samples else "warming"

    @property
    def offset_ns(self):
        return self._offset_ns

    def _reset(self):
        self.epoch += 1
        self.resets += 1
        self.samples = 0
        self._offset_ns = None
        self._last_device_ns = None
        self._last_mapped_ns = None
        self._cache.clear()

    def map(self, device_ns, receipt_host_ns):
        device_ns = int(device_ns)
        receipt_host_ns = int(receipt_host_ns)
        cached = self._cache.get(device_ns)
        if cached is not None:
            self._cache.move_to_end(device_ns)
            cached_mapped_ns, cached_samples, cached_quality = cached
            return ClockMapping(
                device_ns, cached_mapped_ns, receipt_host_ns, self.epoch,
                cached_samples, cached_quality,
                max(0, receipt_host_ns - cached_mapped_ns))

        if (self._last_device_ns is not None
                and device_ns < self._last_device_ns - self.rollback_ns):
            self._reset()

        candidate_offset = receipt_host_ns - device_ns
        advances_clock = (self._last_device_ns is None
                          or device_ns > self._last_device_ns)
        if self._offset_ns is None:
            self._offset_ns = candidate_offset
        elif advances_clock and candidate_offset < self._offset_ns:
            correction = min(
                self._offset_ns - candidate_offset,
                self.max_offset_slew_ns,
            )
            self._offset_ns -= correction

        mapped_ns = device_ns + self._offset_ns
        if (advances_clock and self._last_mapped_ns is not None
                and mapped_ns <= self._last_mapped_ns):
            mapped_ns = self._last_mapped_ns + 1

        if advances_clock:
            self.samples += 1
            self._last_device_ns = device_ns
            self._last_mapped_ns = mapped_ns
        self._cache[device_ns] = (mapped_ns, self.samples, self.quality)
        self._cache.move_to_end(device_ns)
        while len(self._cache) > self.cache_size:
            self._cache.popitem(last=False)

        return ClockMapping(
            device_ns, mapped_ns, receipt_host_ns, self.epoch, self.samples,
            self.quality, max(0, receipt_host_ns - mapped_ns))

    def diagnostics(self):
        return {
            "epoch": self.epoch,
            "samples": self.samples,
            "quality": self.quality,
            "offset_ns": self._offset_ns,
            "resets": self.resets,
            "cache_entries": len(self._cache),
        }
