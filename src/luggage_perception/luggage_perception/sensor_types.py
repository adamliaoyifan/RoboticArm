#!/usr/bin/env python3
"""Frozen per-stream frames and the internal SyncedObservation (no ROS).

PF-R9 g2 payload contract: ``RgbFrame``/``DepthFrame`` own an **opaque
immutable payload reference** (the source ROS ``array.array('B')``).
``copy()`` shares that reference — it never copies pixel bytes. Algorithm
code reads pixels only through ``view()``, a read-only NumPy view built
with ``np.frombuffer`` over a read-only memoryview; unwrapping or
replacing the mutable ROS data array from algorithm code is impossible
and ``view.setflags(write=True)`` raises. Only the ROS adapter/node layer
may unwrap a payload (``ros_data()``) to republish it by identity.
"""

from __future__ import division

from dataclasses import dataclass, field, replace

import numpy as np


def _copy_array(value):
    if value is None:
        return None
    return np.array(value, copy=True)


def _copy_info(info):
    if info is None:
        return None
    return info.copy()


class OpaquePayload(object):
    """Immutable-by-convention owner of a source message byte buffer.

    ``obj`` is the received ``array.array('B')`` (or any bytes-like object).
    The algorithm layer never touches ``obj`` directly: it gets read-only
    NumPy views. The adapter layer may read ``ros_data()`` once more to
    assign the same object into an output ``Image.data`` — payload
    identity survives receive -> buffer -> pair -> copy-out -> queue ->
    republish with zero Python-owned full-frame materialisations.
    """

    __slots__ = ("_obj", "_nbytes", "origin")

    def __init__(self, obj, origin="ros"):
        self._obj = obj
        self._nbytes = len(obj)
        self.origin = str(origin)

    def nbytes(self):
        return int(self._nbytes)

    def ros_data(self):
        """Adapter-layer unwrap for identity republish. Not for algorithms."""
        return self._obj

    def readonly_memory(self):
        return memoryview(self._obj)

    def __len__(self):
        return self._nbytes


def _readonly_view(payload, dtype, shape, strides):
    """Read-only strided view over an OpaquePayload. Never copies."""
    base = np.frombuffer(payload.readonly_memory(), dtype=dtype)
    base.setflags(write=False)
    view = np.lib.stride_tricks.as_strided(base, shape=shape, strides=strides)
    view.setflags(write=False)
    return view


def _legacy_view(array):
    arr = np.asarray(array)
    try:
        arr.setflags(write=False)
    except ValueError:
        pass
    return arr


@dataclass(frozen=True)
class CameraInfoFrame(object):
    """Intrinsics plus the rectification/projection the source published.

    ``rectification`` (9) and ``projection`` (12) are carried verbatim so an
    encoder can rebuild an equivalent CameraInfo without consulting the
    original message. Empty means "derive from fx/fy/cx/cy".
    """

    stamp: float
    frame_id: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion_model: str = "plumb_bob"
    distortion_coeffs: tuple = ()
    rectification: tuple = ()
    projection: tuple = ()
    binning_x: int = 0
    binning_y: int = 0
    encoding_hint: str = ""

    def copy(self):
        return replace(
            self,
            distortion_coeffs=tuple(self.distortion_coeffs),
            rectification=tuple(self.rectification),
            projection=tuple(self.projection),
        )


class _ImageViewMixin(object):
    """Shared geometry + payload view logic for RgbFrame/DepthFrame."""

    _COLOR_CHANNELS = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4,
                       "mono8": 1, "8UC1": 1}

    def _channels(self):
        return self._COLOR_CHANNELS.get(str(self.encoding), 3)

    def _declared_bytes(self):
        raise NotImplementedError

    def view(self):
        """Read-only NumPy view of the pixels (no copy, no astype)."""
        if self.payload is not None:
            declared = self._declared_bytes()
            if declared is None or self.payload.nbytes() < declared:
                return None  # truncated buffer: caller counts it by name
            return self._build_view()
        return _legacy_view(self._raw)

    def _build_view(self):
        raise NotImplementedError


class RgbFrame(_ImageViewMixin, object):
    """One colour image. ``image`` is the legacy direct-array constructor."""

    __slots__ = ("stamp", "frame_id", "encoding", "payload", "height",
                 "width", "step", "is_bigendian", "stamp_key", "_raw")

    def __init__(self, stamp, frame_id, image=None, encoding="rgb8",
                 payload=None, height=0, width=0, step=0, is_bigendian=0,
                 stamp_key=None):
        self.stamp = float(stamp)
        self.frame_id = str(frame_id)
        self.encoding = str(encoding)
        self.payload = payload
        self.height = int(height or 0)
        self.width = int(width or 0)
        self.step = int(step or 0)
        self.is_bigendian = int(is_bigendian)
        self.stamp_key = stamp_key
        self._raw = image

    @property
    def image(self):
        return self.view()

    def _declared_bytes(self):
        if self.height <= 0 or self.width <= 0:
            return None
        channels = self._channels()
        step = self.step or (self.width * channels)
        return int(self.height) * int(step)

    def _build_view(self):
        channels = self._channels()
        step = int(self.step or (self.width * channels))
        shape = (int(self.height), int(self.width), channels) \
            if channels > 1 else (int(self.height), int(self.width))
        strides = (step, channels, 1) if channels > 1 else (step, 1)
        return _readonly_view(self.payload, np.dtype("u1"), shape, strides)

    def copy(self):
        """Share the payload reference (PF-R9 g2): no pixel copy."""
        return RgbFrame(
            self.stamp, self.frame_id, image=self._raw,
            encoding=self.encoding,
            payload=self.payload, height=self.height, width=self.width,
            step=self.step, is_bigendian=self.is_bigendian,
            stamp_key=self.stamp_key)


class DepthFrame(_ImageViewMixin, object):
    """One aligned depth image, ``16UC1`` millimetres by contract."""

    __slots__ = ("stamp", "frame_id", "encoding", "units", "payload",
                 "height", "width", "step", "is_bigendian", "stamp_key",
                 "_raw")

    _DEPTH_CHANNELS = {"16UC1": 1, "mono16": 1, "8UC1": 1}

    def __init__(self, stamp, frame_id, depth=None, units="millimetres",
                 encoding="16UC1", payload=None, height=0, width=0, step=0,
                 is_bigendian=0, stamp_key=None):
        self.stamp = float(stamp)
        self.frame_id = str(frame_id)
        self.encoding = str(encoding)
        self.units = str(units)
        self.payload = payload
        self.height = int(height or 0)
        self.width = int(width or 0)
        self.step = int(step or 0)
        self.is_bigendian = int(is_bigendian)
        self.stamp_key = stamp_key
        self._raw = depth

    @property
    def depth(self):
        return self.view()

    def _channels(self):
        return self._DEPTH_CHANNELS.get(str(self.encoding), 1)

    def _declared_bytes(self):
        if self.height <= 0 or self.width <= 0:
            return None
        return int(self.height) * int(self.step or (self.width * 2))

    def _build_view(self):
        dtype = np.dtype(">u2" if self.is_bigendian else "<u2")
        step = int(self.step or (self.width * 2))
        return _readonly_view(
            self.payload, dtype, (int(self.height), int(self.width)),
            (step, 2))

    def copy(self):
        """Share the payload reference (PF-R9 g2): no pixel copy."""
        return DepthFrame(
            self.stamp, self.frame_id, depth=self._raw, units=self.units,
            encoding=self.encoding, payload=self.payload,
            height=self.height, width=self.width, step=self.step,
            is_bigendian=self.is_bigendian, stamp_key=self.stamp_key)


@dataclass(frozen=True)
class CameraCloud(object):
    """Sparse semantic output cloud (cargo/obstacle products). The
    transported camera cloud is no longer a pipeline product (PF-R9 g2);
    this struct remains for generated point products only."""

    stamp: float
    frame_id: str
    data_frame: str
    points: object
    dropped_nonfinite: int = 0

    def copy(self):
        return replace(self, points=_copy_array(self.points))


@dataclass(frozen=True)
class JointSample(object):
    stamp: float
    joint_names: tuple
    positions: object
    velocities: object = None

    def copy(self):
        return replace(
            self,
            joint_names=tuple(self.joint_names),
            positions=_copy_array(self.positions),
            velocities=_copy_array(self.velocities),
        )


@dataclass(frozen=True)
class LidarScan(object):
    stamp_start: float
    stamp_end: float
    frame_id: str
    points: object
    point_times: object = None

    def copy(self):
        return replace(
            self,
            points=_copy_array(self.points),
            point_times=_copy_array(self.point_times),
        )


@dataclass(frozen=True)
class ImuSample(object):
    stamp: float
    frame_id: str
    angular_velocity: object
    linear_acceleration: object

    def copy(self):
        return replace(
            self,
            angular_velocity=_copy_array(self.angular_velocity),
            linear_acceleration=_copy_array(self.linear_acceleration),
        )


@dataclass(frozen=True)
class ObservationFlags(object):
    rgb_ok: bool = False
    depth_ok: bool = False
    color_info_ok: bool = False
    depth_info_ok: bool = False
    cloud_ok: bool = False
    lidar_ok: bool = False
    deskewed: bool = False
    stale: bool = False
    motion_too_large: bool = False
    geometry_ok: bool = False

    def copy(self):
        return replace(self)

    def as_dict(self):
        return {
            "rgb_ok": self.rgb_ok,
            "depth_ok": self.depth_ok,
            "color_info_ok": self.color_info_ok,
            "depth_info_ok": self.depth_info_ok,
            "cloud_ok": self.cloud_ok,
            "lidar_ok": self.lidar_ok,
            "deskewed": self.deskewed,
            "stale": self.stale,
            "motion_too_large": self.motion_too_large,
            "geometry_ok": self.geometry_ok,
        }


@dataclass(frozen=True)
class SyncedObservation(object):
    primary_stamp: float
    primary_source: str
    rgb: object = None
    depth: object = None
    color_info: object = None
    depth_info: object = None
    lidar_points: object = None
    frame_id: str = ""
    lidar_dt: float = -1.0
    depth_dt: float = -1.0
    rgb_stamp: float = 0.0
    depth_stamp: float = 0.0
    stamp_key: tuple = None
    motion_score: float = 0.0
    units: str = "millimetres"
    flags: ObservationFlags = field(default_factory=ObservationFlags)
    rejection_reason: str = ""

    def copy(self):
        """Copy-out boundary: frames share payloads, metadata is fresh."""
        return replace(
            self,
            rgb=None if self.rgb is None else self.rgb.copy(),
            depth=None if self.depth is None else self.depth.copy(),
            color_info=_copy_info(self.color_info),
            depth_info=_copy_info(self.depth_info),
            lidar_points=_copy_array(self.lidar_points),
            flags=self.flags.copy(),
        )
