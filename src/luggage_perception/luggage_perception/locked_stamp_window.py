#!/usr/bin/env python3
"""Thread-safe stamp window wrapping StampRingBuffer (no ROS)."""

from __future__ import division

import copy
import threading

from luggage_perception.stamp_ring_buffer import StampRingBuffer


class LockedStampWindow(object):
    """maxlen-bounded stamp buffer. Gets return deep copies."""

    def __init__(self, maxlen=10, horizon_sec=None, rollback_sec=0.25):
        self._lock = threading.Lock()
        self._buf = StampRingBuffer(
            maxlen, horizon_sec=horizon_sec, rollback_sec=rollback_sec)

    def __len__(self):
        with self._lock:
            return len(self._buf)

    def clear(self):
        with self._lock:
            self._buf.clear()

    def push(self, stamp, item):
        with self._lock:
            return self._buf.insert(stamp, item)

    def latest(self):
        with self._lock:
            hit = self._buf.latest()
            if hit is None:
                return None
            return hit[0], copy.deepcopy(hit[1])

    def nearest(self, stamp, max_dt):
        with self._lock:
            hit = self._buf.nearest(stamp, max_dt)
            if hit is None:
                return None
            return hit[0], copy.deepcopy(hit[1])

    def snapshot(self):
        with self._lock:
            return [
                (stamp, copy.deepcopy(item))
                for stamp, item in self._buf.items()
            ]
