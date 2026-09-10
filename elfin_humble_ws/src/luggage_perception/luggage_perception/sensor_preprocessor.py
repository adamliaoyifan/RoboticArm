#!/usr/bin/env python3
"""Camera-triggered depth-primary sensor pairing (no ROS).

PF-R9 g2 contract: the canonical acquisition is colour image + colour-
aligned 16UC1 depth + colour CameraInfo + aligned-depth CameraInfo, all
under one exact primary stamp, one width/height, and one truthful colour
optical frame whose K/P describe the colour pixel grid. Aligned depth is
the **mandatory** pair gate: an acquisition without it is never emitted.

The preprocessor never transports a camera point cloud. Frames own opaque
immutable payload references (PF-R9 g2): buffering, pairing, copy-out,
and emit-queueing share the payload; nothing on the receive->republish
path materialises pixel bytes.

Camera buffers are fixed at 15 entries / 1.0 second on the canonical
camera stream clock. Joint, IMU, and lidar timestamps never age camera
payloads. A backward jump beyond ``rollback_sec`` flushes every camera and
accepted index, increments ``camera_epoch`` and ``rollback_events``.
"""

from __future__ import division

import numpy as np

from luggage_perception.motion_stability_filter import MotionStabilityGate
from luggage_perception.sensor_types import (
    CameraInfoFrame,
    DepthFrame,
    ImuSample,
    LidarScan,
    ObservationFlags,
    RgbFrame,
    SyncedObservation,
)
from luggage_perception.stamp_ring_buffer import StampRingBuffer, _ns_from_float

NS = 1000000000

ALLOWED_DEPTH_ENCODINGS = ("16UC1", "mono16")
ALLOWED_DEPTH_UNITS = ("millimetres", "mm")


def _stamp_int_parts(seconds):
    """Float seconds -> exact (sec, nanosec) for the status payload."""
    if seconds <= 0.0:
        return 0, 0
    whole = int(seconds)
    nanosec = int(round((seconds - whole) * 1e9))
    if nanosec >= 1000000000:
        whole += 1
        nanosec -= 1000000000
    return whole, nanosec


def _frame_ns(frame):
    """Exact integer nanosecond key of a frame (header pair preferred)."""
    key = getattr(frame, "stamp_key", None)
    if key:
        return int(key[0]) * NS + int(key[1])
    return _ns_from_float(frame.stamp)


class SensorPreprocessor(object):
    """RGB-primary, depth-mandatory pairing with a joint-state motion gate.

    Mid-360 / IMU buffers exist so later deskew can reuse the same object, but
    lidar is never attached to an observation until deskew is implemented.
    """

    def __init__(
        self,
        camera_maxlen=15,
        camera_horizon_sec=1.0,
        camera_slop_sec=0.005,
        camera_pair_tolerance_sec=None,
        camera_wait_deadline_sec=0.060,
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
        intrinsics_eps=1e-6,
    ):
        if enable_lidar_output:
            raise ValueError(
                "lidar output requires per-point deskew; leave "
                "enable_lidar_output=False until that milestone"
            )
        # PF-R9 g2: pairing tolerance (|depth stamp - rgb stamp|; the
        # aligned streams are co-stamped by the sensor) and wait deadline
        # (bounded on the canonical camera clock, never on joints).
        self.camera_slop_sec = float(camera_slop_sec)
        self.camera_pair_tolerance_sec = (
            float(camera_pair_tolerance_sec)
            if camera_pair_tolerance_sec is not None
            else self.camera_slop_sec)
        self.camera_wait_deadline_sec = float(camera_wait_deadline_sec)
        self.camera_info_max_age_sec = float(camera_info_max_age_sec)
        self.output_cloud_frame = str(output_cloud_frame)
        self.enable_lidar_output = False
        self.stale_sec = float(stale_sec)
        self.intrinsics_eps = float(intrinsics_eps)
        self._rgb = StampRingBuffer(
            camera_maxlen, camera_horizon_sec, rollback_sec)
        self._depth = StampRingBuffer(
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
        self._emitted_ns = set()
        self._output = None
        self._last_rejection = ""
        self._last_geometry_ok_stamp = 0.0
        # PF-R9 g2 named diagnostics counters.
        self.camera_epoch = 0
        self.rollback_events = 0
        self.depth_wait_skips = 0
        self.info_wait_skips = 0
        self.acquisition_mismatches = 0
        self.exact_lookup_misses = 0
        self.payload_materialisations = 0
        self.payload_bytes_received = 0
        self._camera_rollback_marks = {
            name: getattr(self, "_" + name).rollback_count
            for name in ("rgb", "depth", "color_info", "depth_info")
        }

    # ------------------------------------------------------------------
    # Public accessors
    # ------------------------------------------------------------------

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
            "color_info": len(self._color_info),
            "depth_info": len(self._depth_info),
            "joints": len(self._joints),
            "lidar": len(self._lidar),
            "imu": len(self._imu),
        }

    def payload_bytes_buffered(self):
        total = 0
        for buf in (self._rgb, self._depth):
            for _, item in buf.items_ns():
                payload = getattr(item, "payload", None)
                if payload is not None:
                    total += payload.nbytes()
        return total

    def eviction_counters(self):
        out = {}
        for name in ("rgb", "depth", "color_info", "depth_info",
                     "joints", "lidar", "imu"):
            buf = getattr(self, "_" + name)
            out[name] = {
                "capacity": buf.evicted_capacity,
                "horizon": buf.evicted_horizon,
                "replaced": buf.replaced,
                "rollback": buf.rollback_count,
            }
        return out

    def diagnostics(self, now=None):
        occupancy = self.buffer_occupancy()
        flags = (
            self._output.flags.as_dict()
            if self._output is not None
            else ObservationFlags().as_dict()
        )
        primary = (
            self._output.primary_stamp if self._output is not None else 0.0)
        primary_sec, primary_nanosec = _stamp_int_parts(primary)
        return {
            "schema": "luggage.preprocessed.status.v2",
            "camera_pair_tolerance_sec": self.camera_pair_tolerance_sec,
            "camera_wait_deadline_sec": self.camera_wait_deadline_sec,
            "camera_epoch": self.camera_epoch,
            "rollback_events": self.rollback_events,
            "depth_wait_skips": self.depth_wait_skips,
            "info_wait_skips": self.info_wait_skips,
            "acquisition_mismatches": self.acquisition_mismatches,
            "exact_lookup_misses": self.exact_lookup_misses,
            "payload_materialisations": self.payload_materialisations,
            "payload_bytes_buffered": self.payload_bytes_buffered(),
            "payload_bytes_received": self.payload_bytes_received,
            "buffers": occupancy,
            "evictions": self.eviction_counters(),
            "flags": flags,
            "last_rejection": self._last_rejection,
            "output_cloud_frame": self.output_cloud_frame,
            "motion_gate": self._gate.diagnostics(now=now),
            "primary_stamp": primary,
            "primary_stamp_sec": primary_sec,
            "primary_stamp_nanosec": primary_nanosec,
            "last_geometry_ok_stamp": self._last_geometry_ok_stamp,
            "depth_dt": self._output.depth_dt if self._output is not None else -1.0,
            "units": "millimetres",
        }

    # ------------------------------------------------------------------
    # Stream updates
    # ------------------------------------------------------------------

    def update_rgb(self, frame):
        if not isinstance(frame, RgbFrame) or frame.stamp <= 0.0:
            self._last_rejection = "invalid_rgb_stamp"
            return None
        payload = getattr(frame, "payload", None)
        if (payload is None and frame._raw is None) or frame.view() is None:
            self._last_rejection = "invalid_rgb_image"
            return None
        if payload is not None:
            self.payload_bytes_received += payload.nbytes()
        self._rgb.insert_ns(_frame_ns(frame), frame)
        self._note_camera_rollback("rgb")
        return self._try_emit()

    def update_depth(self, frame):
        if not isinstance(frame, DepthFrame) or frame.stamp <= 0.0:
            self._last_rejection = "invalid_depth_stamp"
            return None
        if str(frame.encoding) not in ALLOWED_DEPTH_ENCODINGS:
            self._last_rejection = "unexpected_depth_encoding"
            return None
        if str(frame.units) not in ALLOWED_DEPTH_UNITS:
            self._last_rejection = "unexpected_depth_units"
            return None
        payload = getattr(frame, "payload", None)
        if payload is None:
            if frame._raw is None or np.asarray(frame._raw).size == 0:
                self._last_rejection = "invalid_depth_dimensions"
                return None
        elif frame.view() is None:
            self._last_rejection = "invalid_depth_dimensions"
            return None
        if payload is not None:
            self.payload_bytes_received += payload.nbytes()
        self._depth.insert_ns(_frame_ns(frame), frame)
        self._note_camera_rollback("depth")
        return self._try_emit()

    def update_camera_info(self, frame, slots=("depth", "color")):
        if not isinstance(frame, CameraInfoFrame) or frame.stamp <= 0.0:
            self._last_rejection = "invalid_camera_info_stamp"
            return None
        if int(frame.width) <= 0 or int(frame.height) <= 0:
            self._last_rejection = "invalid_camera_info_dimensions"
            return None
        copied = frame.copy()
        key = _frame_ns(frame)
        for slot in slots:
            if slot == "color":
                self._color_info.insert_ns(key, copied)
                self._note_camera_rollback("color_info")
            elif slot == "depth":
                self._depth_info.insert_ns(key, copied)
                self._note_camera_rollback("depth_info")
            else:
                raise ValueError("unknown camera_info slot %r" % slot)
        return self._try_emit()

    def update_joint_state(self, sample):
        if sample is None or sample.stamp <= 0.0:
            self._last_rejection = "invalid_joint_stamp"
            return None
        self._joints.insert_ns(_frame_ns(sample), sample)
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
        return self._try_emit()

    def update_lidar(self, scan):
        if not isinstance(scan, LidarScan) or scan.stamp_end <= 0.0:
            self._last_rejection = "invalid_lidar_stamp"
            return None
        self._lidar.insert_ns(_ns_from_float(scan.stamp_end), scan)
        return self._try_emit()

    def update_imu(self, sample):
        if not isinstance(sample, ImuSample) or sample.stamp <= 0.0:
            self._last_rejection = "invalid_imu_stamp"
            return None
        self._imu.insert_ns(_frame_ns(sample), sample)
        return self._try_emit()

    # ------------------------------------------------------------------
    # Pairing and emission
    # ------------------------------------------------------------------

    def _camera_now_ns(self):
        latest = None
        for buf in (self._rgb, self._depth, self._color_info,
                    self._depth_info):
            hit = buf.latest_ns()
            if hit is not None and (latest is None or hit[0] > latest):
                latest = hit[0]
        return latest

    def _past_deadline_ns(self, rgb_ns):
        latest = self._rgb.latest_ns()
        if latest is None:
            return False
        return (latest[0] - rgb_ns) >= int(
            round(self.camera_wait_deadline_sec * NS))

    def _try_emit(self):
        camera_now = self._camera_now_ns()
        if camera_now is not None:
            # Camera buffers age on the canonical camera clock only.
            for buf in (self._rgb, self._depth, self._color_info,
                        self._depth_info):
                buf.prune_ns(camera_now)
            horizon_ns = int(round(
                (self._rgb.horizon_sec or 1.0) * NS))
            self._emitted_ns = {
                ns for ns in self._emitted_ns
                if ns >= camera_now - horizon_ns}
        for buf in (self._joints, self._lidar, self._imu):
            latest = buf.latest_ns()
            if latest is not None:
                buf.prune_ns(latest[0])
        tolerance_ns = int(round(self.camera_pair_tolerance_sec * NS))
        for rgb_ns in list(self._rgb.stamps_ns()):
            if rgb_ns in self._emitted_ns:
                continue
            depth_hit = self._depth.nearest_ns(rgb_ns, tolerance_ns)
            if depth_hit is None:
                if self._past_deadline_ns(rgb_ns):
                    self._last_rejection = "depth_wait_timeout"
                    self.depth_wait_skips += 1
                    self._emitted_ns.add(rgb_ns)
                else:
                    self.exact_lookup_misses += 1
                continue
            observation = self._build_if_ready(rgb_ns, depth_hit)
            if observation is None:
                continue
            self._emitted_ns.add(rgb_ns)
            self._output = observation
            if observation.flags.geometry_ok:
                self._last_geometry_ok_stamp = float(
                    observation.primary_stamp)
            return observation.copy()
        return None

    def _nearest_info(self, buffer, stamp_ns):
        hit = buffer.nearest_ns(
            stamp_ns, int(round(self.camera_info_max_age_sec * NS)))
        if hit is None:
            return None
        return hit[1]

    def _build_if_ready(self, rgb_ns, depth_hit):
        rgb_hit = self._rgb.nearest_ns(rgb_ns, 0)
        if rgb_hit is None:
            return None
        rgb = rgb_hit[1]
        depth_ns, depth = depth_hit

        color_info = self._nearest_info(self._color_info, rgb_ns)
        depth_info = self._nearest_info(self._depth_info, rgb_ns)
        if color_info is None:
            color_info = depth_info
        if depth_info is None:
            depth_info = color_info
        if color_info is None or depth_info is None:
            # Incomplete acquisition: wait, then fail closed past the
            # deadline. Camera infos never produce a partial emission.
            if self._past_deadline_ns(rgb_ns):
                self._last_rejection = "missing_camera_info"
                self.info_wait_skips += 1
                self._emitted_ns.add(rgb_ns)
            return None

        mismatch = self._acquisition_mismatch(rgb, depth, color_info,
                                              depth_info)
        if mismatch:
            self._last_rejection = mismatch
            self.acquisition_mismatches += 1
            self._emitted_ns.add(rgb_ns)
            return None

        depth_dt = abs(depth_ns - rgb_ns) / float(NS)

        gate_state = self._gate.state(now=rgb.stamp)
        geometry_ok = bool(self._gate.accepts_cloud(rgb.stamp, now=rgb.stamp))
        motion_too_large = gate_state in ("moving", "settling")
        motion_score = float(self._gate.peak_excursion or 0.0)
        if gate_state != "disabled" and (
                gate_state in ("unknown", "stale") or not self._joints):
            geometry_ok = False

        flags = ObservationFlags(
            rgb_ok=True,
            depth_ok=True,
            color_info_ok=True,
            depth_info_ok=True,
            cloud_ok=False,
            lidar_ok=False,
            deskewed=False,
            stale=((self._camera_now_ns() - rgb_ns) / float(NS)
                   > self.stale_sec) if self._camera_now_ns() else False,
            motion_too_large=motion_too_large,
            geometry_ok=geometry_ok,
        )
        return SyncedObservation(
            primary_stamp=rgb.stamp,
            primary_source="camera",
            rgb=rgb.copy(),
            depth=depth.copy(),
            color_info=color_info.copy(),
            depth_info=depth_info.copy(),
            lidar_points=None,
            frame_id=rgb.frame_id,
            lidar_dt=-1.0,
            depth_dt=depth_dt,
            rgb_stamp=rgb.stamp,
            depth_stamp=depth.stamp,
            stamp_key=(rgb_ns // NS, rgb_ns % NS),
            motion_score=motion_score,
            units="millimetres",
            flags=flags,
            rejection_reason=self._last_rejection,
        )

    def _acquisition_mismatch(self, rgb, depth, color_info, depth_info):
        """Canonical-set consistency check (fail-closed, named counter)."""
        rgb_view = rgb.view()
        depth_view = depth.view()
        if rgb_view is None or depth_view is None:
            return "acquisition_view_invalid"
        if rgb_view.shape[:2] != depth_view.shape[:2]:
            return "acquisition_dimension_mismatch"
        height, width = rgb_view.shape[:2]
        rgb_frame_id = str(rgb.frame_id or "")
        if str(depth.frame_id or "") != rgb_frame_id:
            return "acquisition_frame_mismatch"
        for info in (color_info, depth_info):
            if int(info.width) != int(width) or int(info.height) != int(height):
                return "acquisition_info_dimension_mismatch"
        if not self._intrinsics_match(color_info, depth_info):
            return "acquisition_intrinsics_mismatch"
        return ""

    @staticmethod
    def _intrinsics_match(a, b, eps=1e-6):
        return (abs(a.fx - b.fx) <= eps and abs(a.fy - b.fy) <= eps
                and abs(a.cx - b.cx) <= eps and abs(a.cy - b.cy) <= eps)

    def _note_camera_rollback(self, name):
        buf = getattr(self, "_" + name)
        if buf.rollback_count > self._camera_rollback_marks[name]:
            # One rollback flushes every camera and accepted index and
            # bumps the local epoch; the node clears pending output. The
            # buffer that rolled back already cleared itself and holds the
            # first frame of the new timeline — it is kept.
            self.camera_epoch += 1
            self.rollback_events += 1
            for other_name in ("rgb", "depth", "color_info", "depth_info"):
                if other_name != name:
                    getattr(self, "_" + other_name).clear()
            self._camera_rollback_marks = {
                n: getattr(self, "_" + n).rollback_count
                for n in self._camera_rollback_marks
            }
            self._emitted_ns = set()


def transform_points(points, matrix, dtype=None):
    """Apply a 4x4 transform to (N,3) row vectors: p' = R p + t.

    BLAS-free on purpose: the matmul form dispatches (307k,3)x(3,3) to
    OpenBLAS, whose worker threads busy-spin and pin the calling node at
    >400% CPU (PF-R9 measured). The ufunc column form is single-threaded
    and equally fast at this shape.
    """
    dtype = np.dtype(dtype) if dtype is not None else np.float64
    pts = np.asarray(points, dtype=dtype).reshape(-1, 3)
    if pts.size == 0:
        return pts
    matrix = np.asarray(matrix, dtype=dtype).reshape(4, 4)
    r = matrix[:3, :3]
    t = matrix[:3, 3]
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    out = np.empty_like(pts)
    out[:, 0] = x * r[0, 0] + y * r[0, 1] + z * r[0, 2] + t[0]
    out[:, 1] = x * r[1, 0] + y * r[1, 1] + z * r[1, 2] + t[1]
    out[:, 2] = x * r[2, 0] + y * r[2, 1] + z * r[2, 2] + t[2]
    return out
