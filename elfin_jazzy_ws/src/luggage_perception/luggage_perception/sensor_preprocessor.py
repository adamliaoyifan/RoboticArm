#!/usr/bin/env python3
"""Camera-triggered sensor pairing, filtering, and motion gating (no ROS)."""

from __future__ import division

import numpy as np

from luggage_perception.motion_stability_filter import MotionStabilityGate
from luggage_perception.sensor_types import (
    CameraCloud,
    CameraInfoFrame,
    DepthFrame,
    ImuSample,
    LidarScan,
    ObservationFlags,
    RgbFrame,
    SyncedObservation,
)
from luggage_perception.stamp_ring_buffer import StampRingBuffer, _finite_positive


ALLOWED_DEPTH_ENCODINGS = ("16UC1", "mono16")
ALLOWED_DEPTH_UNITS = ("millimetres", "mm")


class SensorPreprocessor(object):
    """RGB-primary pairing of D435 streams with a joint-state motion gate.

    Mid-360 / IMU buffers exist so later deskew can reuse the same object, but
    lidar is never attached to an observation until deskew is implemented.
    """

    def __init__(
        self,
        camera_maxlen=10,
        camera_horizon_sec=0.35,
        camera_slop_sec=0.020,
        camera_info_max_age_sec=1.0,
        joint_horizon_sec=1.0,
        joint_maxlen=50,
        lidar_maxlen=4,
        lidar_horizon_sec=0.40,
        imu_maxlen=200,
        imu_horizon_sec=1.0,
        output_cloud_frame="camera_depth_optical_frame",
        enable_lidar_output=False,
        motion_gate=None,
        joint_names=None,
        stale_sec=0.15,
        rollback_sec=0.25,
    ):
        if enable_lidar_output:
            raise ValueError(
                "lidar output requires per-point deskew; leave "
                "enable_lidar_output=False until that milestone"
            )
        self.camera_slop_sec = float(camera_slop_sec)
        self.camera_info_max_age_sec = float(camera_info_max_age_sec)
        self.output_cloud_frame = str(output_cloud_frame)
        self.enable_lidar_output = False
        self.stale_sec = float(stale_sec)
        self._rgb = StampRingBuffer(
            camera_maxlen, camera_horizon_sec, rollback_sec)
        self._depth = StampRingBuffer(
            camera_maxlen, camera_horizon_sec, rollback_sec)
        self._cloud = StampRingBuffer(
            camera_maxlen, camera_horizon_sec, rollback_sec)
        self._color_info = StampRingBuffer(
            camera_maxlen, camera_horizon_sec, rollback_sec)
        self._depth_info = StampRingBuffer(
            camera_maxlen, camera_horizon_sec, rollback_sec)
        self._joints = StampRingBuffer(
            joint_maxlen, joint_horizon_sec, rollback_sec)
        self._lidar = StampRingBuffer(
            lidar_maxlen, lidar_horizon_sec, rollback_sec)
        self._imu = StampRingBuffer(
            imu_maxlen, imu_horizon_sec, rollback_sec)
        self._gate = motion_gate or MotionStabilityGate(
            joint_names=list(joint_names or []),
        )
        self._emitted = set()
        self._output = None
        self._last_rejection = ""
        self._last_dropped_nonfinite = 0

    def copy_output(self):
        if self._output is None:
            return None
        return self._output.copy()

    def last_rejection_reason(self):
        return self._last_rejection

    def buffer_occupancy(self):
        return {
            "rgb": len(self._rgb),
            "depth": len(self._depth),
            "cloud": len(self._cloud),
            "color_info": len(self._color_info),
            "depth_info": len(self._depth_info),
            "joints": len(self._joints),
            "lidar": len(self._lidar),
            "imu": len(self._imu),
        }

    def diagnostics(self, now=None):
        occupancy = self.buffer_occupancy()
        flags = (
            self._output.flags.as_dict()
            if self._output is not None
            else ObservationFlags().as_dict()
        )
        return {
            "schema": "luggage.preprocessed.status.v1",
            "buffers": occupancy,
            "flags": flags,
            "last_rejection": self._last_rejection,
            "dropped_nonfinite": self._last_dropped_nonfinite,
            "output_cloud_frame": self.output_cloud_frame,
            "motion_gate": self._gate.diagnostics(now=now),
            "primary_stamp": (
                self._output.primary_stamp if self._output is not None else 0.0
            ),
            "depth_dt": self._output.depth_dt if self._output is not None else -1.0,
            "cloud_dt": self._output.cloud_dt if self._output is not None else -1.0,
            "lidar_dt": -1.0,
            "units": "millimetres",
        }

    def update_rgb(self, frame):
        if not isinstance(frame, RgbFrame) or not _finite_positive(frame.stamp):
            self._last_rejection = "invalid_rgb_stamp"
            return None
        if frame.image is None or np.asarray(frame.image).size == 0:
            self._last_rejection = "invalid_rgb_image"
            return None
        self._rgb.insert(frame.stamp, frame.copy())
        return self._try_emit(frame.stamp)

    def update_depth(self, frame):
        if not isinstance(frame, DepthFrame) or not _finite_positive(frame.stamp):
            self._last_rejection = "invalid_depth_stamp"
            return None
        if str(frame.encoding) not in ALLOWED_DEPTH_ENCODINGS:
            self._last_rejection = "unexpected_depth_encoding"
            return None
        if str(frame.units) not in ALLOWED_DEPTH_UNITS:
            self._last_rejection = "unexpected_depth_units"
            return None
        depth = np.asarray(frame.depth)
        if depth.ndim != 2 or depth.size == 0:
            self._last_rejection = "invalid_depth_dimensions"
            return None
        stored = replace_depth(frame, depth)
        self._depth.insert(frame.stamp, stored)
        return self._try_emit(frame.stamp)

    def update_camera_info(self, frame, slots=("depth", "color")):
        if not isinstance(frame, CameraInfoFrame) or not _finite_positive(frame.stamp):
            self._last_rejection = "invalid_camera_info_stamp"
            return None
        if int(frame.width) <= 0 or int(frame.height) <= 0:
            self._last_rejection = "invalid_camera_info_dimensions"
            return None
        copied = frame.copy()
        for slot in slots:
            if slot == "color":
                self._color_info.insert(frame.stamp, copied)
            elif slot == "depth":
                self._depth_info.insert(frame.stamp, copied)
            else:
                raise ValueError("unknown camera_info slot %r" % slot)
        return self._try_emit(frame.stamp)

    def update_camera_cloud(self, cloud, point_transform=None):
        if not isinstance(cloud, CameraCloud) or not _finite_positive(cloud.stamp):
            self._last_rejection = "invalid_cloud_stamp"
            return None
        points = np.asarray(cloud.points, dtype=np.float64).reshape(-1, 3)
        finite = np.isfinite(points).all(axis=1)
        dropped = int((~finite).sum())
        filtered = points[finite]
        if point_transform is not None:
            filtered = transform_points(filtered, point_transform)
        stored = CameraCloud(
            stamp=cloud.stamp,
            frame_id=self.output_cloud_frame,
            data_frame=cloud.data_frame,
            points=filtered,
            dropped_nonfinite=dropped,
        )
        self._cloud.insert(cloud.stamp, stored)
        self._last_dropped_nonfinite = dropped
        return self._try_emit(cloud.stamp)

    def update_joint_state(self, sample):
        if sample is None or not _finite_positive(sample.stamp):
            self._last_rejection = "invalid_joint_stamp"
            return None
        self._joints.insert(sample.stamp, sample.copy())
        velocities = None
        if sample.velocities is not None:
            velocities = np.asarray(sample.velocities).tolist()
        self._gate.update(
            list(sample.joint_names),
            np.asarray(sample.positions).tolist(),
            velocities,
            stamp=sample.stamp,
            now=sample.stamp,
        )
        return self._try_emit(sample.stamp)

    def update_lidar(self, scan):
        if not isinstance(scan, LidarScan) or not _finite_positive(scan.stamp_end):
            self._last_rejection = "invalid_lidar_stamp"
            return None
        self._lidar.insert(scan.stamp_end, scan.copy())
        return self._try_emit(scan.stamp_end)

    def update_imu(self, sample):
        if not isinstance(sample, ImuSample) or not _finite_positive(sample.stamp):
            self._last_rejection = "invalid_imu_stamp"
            return None
        self._imu.insert(sample.stamp, sample.copy())
        return self._try_emit(sample.stamp)

    def _try_emit(self, hint_stamp):
        now_hint = self._now_hint(hint_stamp)
        self._prune_all(now_hint)
        self._prune_emitted(now_hint)
        emitted = None
        for stamp in list(self._rgb.stamps()):
            if stamp in self._emitted:
                continue
            observation = self._build_if_ready(stamp, now_hint)
            if observation is None:
                continue
            self._emitted.add(stamp)
            self._output = observation
            emitted = observation
            break
        return None if emitted is None else emitted.copy()

    def _build_if_ready(self, rgb_stamp, now_hint):
        rgb_hit = self._rgb.nearest(rgb_stamp, 0.0)
        if rgb_hit is None:
            return None
        rgb = rgb_hit[1]
        depth_hit = self._depth.nearest(rgb_stamp, self.camera_slop_sec)
        cloud_hit = self._cloud.nearest(rgb_stamp, self.camera_slop_sec)
        if not self._window_ready(rgb_stamp, depth_hit, cloud_hit, now_hint):
            return None

        color_info = self._nearest_info(self._color_info, rgb_stamp)
        depth_info = self._nearest_info(self._depth_info, rgb_stamp)
        if color_info is None:
            color_info = depth_info
        if depth_info is None:
            depth_info = color_info

        depth = None if depth_hit is None else depth_hit[1]
        cloud = None if cloud_hit is None else cloud_hit[1]
        depth_dt = -1.0 if depth is None else abs(depth.stamp - rgb_stamp)
        cloud_dt = -1.0 if cloud is None else abs(cloud.stamp - rgb_stamp)

        gate_now = rgb_stamp
        gate_state = self._gate.state(now=gate_now)
        geometry_ok = bool(self._gate.accepts_cloud(rgb_stamp, now=gate_now))
        motion_too_large = gate_state in ("moving", "settling")
        motion_score = float(self._gate.peak_excursion or 0.0)
        if gate_state in ("unknown", "stale") or not self._joints:
            geometry_ok = False

        camera_points = None
        dropped = 0
        data_frame = ""
        if cloud is not None and geometry_ok:
            camera_points = np.array(cloud.points, copy=True)
            dropped = int(cloud.dropped_nonfinite)
            data_frame = cloud.data_frame
        elif cloud is not None:
            dropped = int(cloud.dropped_nonfinite)
            data_frame = cloud.data_frame

        flags = ObservationFlags(
            rgb_ok=True,
            depth_ok=depth is not None,
            color_info_ok=color_info is not None,
            depth_info_ok=depth_info is not None,
            cloud_ok=cloud is not None and geometry_ok,
            lidar_ok=False,
            deskewed=False,
            stale=(now_hint - rgb_stamp) > self.stale_sec,
            motion_too_large=motion_too_large,
            geometry_ok=geometry_ok,
        )
        return SyncedObservation(
            primary_stamp=rgb_stamp,
            primary_source="camera",
            rgb=rgb.copy(),
            depth=None if depth is None else depth.copy(),
            color_info=None if color_info is None else color_info.copy(),
            depth_info=None if depth_info is None else depth_info.copy(),
            camera_points=camera_points,
            lidar_points=None,
            frame_id=self.output_cloud_frame,
            data_frame=data_frame,
            lidar_dt=-1.0,
            depth_dt=depth_dt,
            cloud_dt=cloud_dt,
            rgb_stamp=rgb_stamp,
            depth_stamp=0.0 if depth is None else depth.stamp,
            cloud_stamp=0.0 if cloud is None else cloud.stamp,
            motion_score=motion_score,
            dropped_nonfinite=dropped,
            units="millimetres",
            flags=flags,
            rejection_reason=self._last_rejection,
        )

    def _window_ready(self, rgb_stamp, depth_hit, cloud_hit, now_hint):
        depth_ready = (
            depth_hit is not None
            or self._past_slop(self._depth, rgb_stamp)
            or (self._has_newer_rgb(rgb_stamp) and len(self._depth) == 0)
        )
        cloud_ready = (
            cloud_hit is not None
            or self._past_slop(self._cloud, rgb_stamp)
            or (now_hint - rgb_stamp) >= self.camera_slop_sec
        )
        return depth_ready and cloud_ready

    def _has_newer_rgb(self, rgb_stamp):
        latest = self._rgb.latest()
        return latest is not None and latest[0] > rgb_stamp

    def _past_slop(self, buffer, rgb_stamp):
        latest = buffer.latest()
        return latest is not None and latest[0] > rgb_stamp + self.camera_slop_sec

    def _nearest_info(self, buffer, stamp):
        hit = buffer.nearest(stamp, self.camera_info_max_age_sec)
        if hit is None:
            return None
        return hit[1]

    def _now_hint(self, hint_stamp):
        candidates = [float(hint_stamp)]
        for buf in (
            self._rgb, self._depth, self._cloud, self._joints,
            self._color_info, self._depth_info, self._lidar, self._imu,
        ):
            latest = buf.latest()
            if latest is not None:
                candidates.append(latest[0])
        return max(candidates)

    def _prune_all(self, now_hint):
        for buf in (
            self._rgb, self._depth, self._cloud, self._joints,
            self._color_info, self._depth_info, self._lidar, self._imu,
        ):
            buf.prune(now_hint)

    def _prune_emitted(self, now_hint):
        horizon = self._rgb.horizon_sec or 1.0
        cutoff = now_hint - horizon
        self._emitted = {stamp for stamp in self._emitted if stamp >= cutoff}


def transform_points(points, matrix):
    """Apply a 4x4 transform to (N,3) row vectors: p' = R p + t."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if pts.size == 0:
        return pts
    matrix = np.asarray(matrix, dtype=np.float64).reshape(4, 4)
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    return pts.dot(rotation.T) + translation


def replace_depth(frame, depth_array):
    return DepthFrame(
        stamp=frame.stamp,
        frame_id=frame.frame_id,
        depth=np.array(depth_array, copy=True),
        units=frame.units,
        encoding=frame.encoding,
    )
