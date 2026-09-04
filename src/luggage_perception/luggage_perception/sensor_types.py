#!/usr/bin/env python3
"""Frozen per-stream frames and the internal SyncedObservation (no ROS)."""

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


@dataclass(frozen=True)
class RgbFrame(object):
    stamp: float
    frame_id: str
    image: object
    encoding: str = "rgb8"

    def copy(self):
        return replace(self, image=_copy_array(self.image))


@dataclass(frozen=True)
class DepthFrame(object):
    stamp: float
    frame_id: str
    depth: object
    units: str
    encoding: str = "16UC1"

    def copy(self):
        return replace(self, depth=_copy_array(self.depth))


@dataclass(frozen=True)
class CameraCloud(object):
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
    camera_points: object = None
    lidar_points: object = None
    frame_id: str = ""
    data_frame: str = ""
    lidar_dt: float = -1.0
    depth_dt: float = -1.0
    cloud_dt: float = -1.0
    rgb_stamp: float = 0.0
    depth_stamp: float = 0.0
    cloud_stamp: float = 0.0
    motion_score: float = 0.0
    dropped_nonfinite: int = 0
    units: str = "millimetres"
    flags: ObservationFlags = field(default_factory=ObservationFlags)
    rejection_reason: str = ""

    def copy(self):
        return replace(
            self,
            rgb=None if self.rgb is None else self.rgb.copy(),
            depth=None if self.depth is None else self.depth.copy(),
            color_info=_copy_info(self.color_info),
            depth_info=_copy_info(self.depth_info),
            camera_points=_copy_array(self.camera_points),
            lidar_points=_copy_array(self.lidar_points),
            flags=self.flags.copy(),
        )
