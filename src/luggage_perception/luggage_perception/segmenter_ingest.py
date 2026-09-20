#!/usr/bin/env python3
"""Assemble one segmenter frame from preprocessed RGB plus a task epoch.

ROS-free. The camera preprocessor already stamp-joins RGBD; this module does
not pair streams. ``/luggage/current_box`` is an optional latched task epoch
(sim spawn id + generation; omitted on hardware). Ingest copies the latest
epoch onto the RGB stamp at assemble time. Algorithm code reads the returned
``FrameObservation``, not a live latch.
"""

from __future__ import division

from dataclasses import dataclass

from luggage_perception.stamp_ring_buffer import StampRingBuffer


@dataclass(frozen=True)
class FrameObservation(object):
    """One ingest snapshot. ``rgb`` is a shared reference, never pixel-copied."""

    stamp: float
    frame_id: str
    generation: int
    instance_id: str
    rgb: object = None


class SegmenterIngest(object):
    """Latest-at-ingest epoch + bounded RGB stamp ring."""

    def __init__(self, maxlen=15, horizon_sec=1.0):
        self._generation = 0
        self._instance_id = ""
        self._ring = StampRingBuffer(
            maxlen=int(maxlen), horizon_sec=float(horizon_sec))

    @property
    def generation(self):
        return int(self._generation)

    @property
    def instance_id(self):
        return str(self._instance_id)

    def note_epoch(self, instance_id, generation):
        """Latch a current_box epoch. True when either field changed."""
        generation = int(generation or 0)
        instance_id = str(instance_id or "")
        if (generation == self._generation
                and instance_id == self._instance_id):
            return False
        self._generation = generation
        self._instance_id = instance_id
        return True

    def assemble(self, rgb_frame):
        """Bind the current epoch to this RGB stamp and remember the snapshot.

        Previously assembled observations are not rewritten when the epoch
        later changes. Ring eviction drops the buffer's reference only.
        """
        stamp = float(getattr(rgb_frame, "stamp", 0.0) or 0.0)
        frame_id = str(getattr(rgb_frame, "frame_id", "") or "")
        obs = FrameObservation(
            stamp=stamp,
            frame_id=frame_id,
            generation=int(self._generation),
            instance_id=str(self._instance_id),
            rgb=rgb_frame,
        )
        if stamp > 0.0:
            self._ring.insert(stamp, obs)
            self._ring.prune(stamp)
        return obs
