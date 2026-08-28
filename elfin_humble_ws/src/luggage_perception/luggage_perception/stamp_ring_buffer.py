#!/usr/bin/env python3
"""Bounded, stamp-sorted ring buffer (no ROS)."""

from __future__ import division

import bisect


class StampRingBuffer(object):
    """Capacity- and horizon-bounded storage keyed by monotonically used stamps.

    Items are kept sorted by stamp. Slightly out-of-order inserts (within
    ``rollback_sec`` of the latest stamp) are spliced in place. A jump
    backward larger than ``rollback_sec`` is treated as a clock rollback
    and flushes the buffer.
    """

    def __init__(self, maxlen, horizon_sec=None, rollback_sec=0.25):
        if int(maxlen) < 1:
            raise ValueError("maxlen must be >= 1")
        self.maxlen = int(maxlen)
        self.horizon_sec = None if horizon_sec is None else float(horizon_sec)
        self.rollback_sec = float(rollback_sec)
        self._stamps = []
        self._items = []

    def __len__(self):
        return len(self._stamps)

    def clear(self):
        self._stamps = []
        self._items = []

    def latest(self):
        if not self._stamps:
            return None
        return self._stamps[-1], self._items[-1]

    def oldest(self):
        if not self._stamps:
            return None
        return self._stamps[0], self._items[0]

    def stamps(self):
        return list(self._stamps)

    def insert(self, stamp, item):
        """Insert ``item`` at ``stamp``. Returns False if the stamp is rejected."""
        stamp = float(stamp)
        if not _finite_positive(stamp):
            return False
        if self._stamps:
            latest = self._stamps[-1]
            if stamp < latest and (latest - stamp) > self.rollback_sec:
                self.clear()
        index = bisect.bisect_left(self._stamps, stamp)
        if index < len(self._stamps) and self._stamps[index] == stamp:
            self._items[index] = item
        else:
            self._stamps.insert(index, stamp)
            self._items.insert(index, item)
        self._trim_capacity()
        return True

    def prune(self, now):
        if self.horizon_sec is None or not self._stamps:
            return
        now = float(now)
        cutoff = now - self.horizon_sec
        drop = 0
        for stamp in self._stamps:
            if stamp >= cutoff:
                break
            drop += 1
        if drop:
            del self._stamps[:drop]
            del self._items[:drop]

    def nearest(self, stamp, max_dt):
        """Return ``(stamp, item)`` closest to ``stamp`` within ``max_dt``.

        Ties pick the earlier stamp. Returns None when empty or all outside
        the window.
        """
        if not self._stamps:
            return None
        stamp = float(stamp)
        max_dt = float(max_dt)
        index = bisect.bisect_left(self._stamps, stamp)
        candidates = []
        if index < len(self._stamps):
            candidates.append(index)
        if index > 0:
            candidates.append(index - 1)
        best = None
        best_dt = None
        best_stamp = None
        for i in candidates:
            dt = abs(self._stamps[i] - stamp)
            if dt > max_dt + 1e-12:
                continue
            if (
                best is None
                or dt < best_dt
                or (dt == best_dt and self._stamps[i] < best_stamp)
            ):
                best = i
                best_dt = dt
                best_stamp = self._stamps[i]
        if best is None:
            return None
        return self._stamps[best], self._items[best]

    def _trim_capacity(self):
        overflow = len(self._stamps) - self.maxlen
        if overflow > 0:
            del self._stamps[:overflow]
            del self._items[:overflow]


def _finite_positive(stamp):
    try:
        value = float(stamp)
    except (TypeError, ValueError):
        return False
    return value > 0.0 and value == value and value != float("inf")
