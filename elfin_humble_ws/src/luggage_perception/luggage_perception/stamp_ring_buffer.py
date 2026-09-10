#!/usr/bin/env python3
"""Bounded, stamp-sorted ring buffer (no ROS).

PF-R9 g2 payload identity: keys are exact integer nanoseconds derived from
``(sec, nanosec)`` headers (or rounded once from a float stamp). Float
equality is never the payload identity. Counters expose capacity eviction,
horizon eviction, same-stamp replacement, and rollback flushes.
"""

from __future__ import division

import bisect

NS = 1000000000


def _ns_from_float(stamp):
    return int(round(float(stamp) * NS))


class StampRingBuffer(object):
    """Capacity- and horizon-bounded storage keyed by integer nanosecond
    stamps.

    Items are kept sorted by stamp. Slightly out-of-order inserts (within
    ``rollback_sec`` of the latest stamp) are spliced in place. A jump
    backward larger than ``rollback_sec`` is treated as a clock rollback
    and flushes the buffer (``rollback_count`` increments; callers derive
    their epoch handling from it).
    """

    def __init__(self, maxlen, horizon_sec=None, rollback_sec=0.25):
        if int(maxlen) < 1:
            raise ValueError("maxlen must be >= 1")
        self.maxlen = int(maxlen)
        self.horizon_sec = None if horizon_sec is None else float(horizon_sec)
        self.rollback_sec = float(rollback_sec)
        self._stamps = []          # integer nanoseconds, sorted
        self._items = []
        # named counters (PF-R9 g2 diagnostics)
        self.evicted_capacity = 0
        self.evicted_horizon = 0
        self.replaced = 0
        self.rollback_count = 0

    def __len__(self):
        return len(self._stamps)

    def clear(self):
        self._stamps = []
        self._items = []

    def latest(self):
        if not self._stamps:
            return None
        return self._stamps[-1] / float(NS), self._items[-1]

    def latest_ns(self):
        if not self._stamps:
            return None
        return self._stamps[-1], self._items[-1]

    def oldest(self):
        if not self._stamps:
            return None
        return self._stamps[0] / float(NS), self._items[0]

    def stamps(self):
        return [s / float(NS) for s in self._stamps]

    def stamps_ns(self):
        return list(self._stamps)

    def items(self):
        return [(s / float(NS), item) for s, item in zip(self._stamps, self._items)]

    def items_ns(self):
        return list(zip(self._stamps, self._items))

    def insert_ns(self, stamp_ns, item):
        """Insert at an exact integer nanosecond stamp."""
        stamp_ns = int(stamp_ns)
        if stamp_ns <= 0:
            return False
        rolled_back = False
        if self._stamps:
            latest = self._stamps[-1]
            if (latest - stamp_ns) > int(round(self.rollback_sec * NS)):
                self.clear()
                self.rollback_count += 1
                rolled_back = True
        index = bisect.bisect_left(self._stamps, stamp_ns)
        if index < len(self._stamps) and self._stamps[index] == stamp_ns:
            self._items[index] = item
            self.replaced += 1
        else:
            self._stamps.insert(index, stamp_ns)
            self._items.insert(index, item)
        overflow = len(self._stamps) - self.maxlen
        if overflow > 0:
            del self._stamps[:overflow]
            del self._items[:overflow]
            self.evicted_capacity += overflow
        return not rolled_back or True  # inserted either way

    def insert(self, stamp, item):
        """Float-seconds wrapper over insert_ns (single rounding)."""
        stamp = float(stamp)
        if not _finite_positive(stamp):
            return False
        return self.insert_ns(_ns_from_float(stamp), item)

    def prune_ns(self, now_ns):
        if self.horizon_sec is None or not self._stamps:
            return
        cutoff = int(now_ns) - int(round(self.horizon_sec * NS))
        drop = 0
        for stamp in self._stamps:
            if stamp >= cutoff:
                break
            drop += 1
        if drop:
            del self._stamps[:drop]
            del self._items[:drop]
            self.evicted_horizon += drop

    def prune(self, now):
        if self.horizon_sec is None or not self._stamps:
            return
        self.prune_ns(_ns_from_float(now))

    def nearest_ns(self, stamp_ns, max_dt_ns):
        """Exact-key nearest lookup in the integer domain."""
        if not self._stamps:
            return None
        stamp_ns = int(stamp_ns)
        max_dt_ns = int(max_dt_ns)
        index = bisect.bisect_left(self._stamps, stamp_ns)
        candidates = []
        if index < len(self._stamps):
            candidates.append(index)
        if index > 0:
            candidates.append(index - 1)
        best = None
        best_dt = None
        best_stamp = None
        for i in candidates:
            dt = abs(self._stamps[i] - stamp_ns)
            if dt > max_dt_ns:
                continue
            if (best is None or dt < best_dt
                    or (dt == best_dt and self._stamps[i] < best_stamp)):
                best = i
                best_dt = dt
                best_stamp = self._stamps[i]
        if best is None:
            return None
        return self._stamps[best], self._items[best]

    def nearest(self, stamp, max_dt):
        if not self._stamps:
            return None
        hit = self.nearest_ns(
            _ns_from_float(stamp), int(round(float(max_dt) * NS)))
        if hit is None:
            return None
        return hit[0] / float(NS), hit[1]


def _finite_positive(stamp):
    try:
        value = float(stamp)
    except (TypeError, ValueError):
        return False
    return value > 0.0 and value == value and value != float("inf")
