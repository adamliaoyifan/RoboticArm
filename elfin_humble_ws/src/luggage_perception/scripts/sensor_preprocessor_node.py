#!/usr/bin/env python3
"""Thin ROS 2 node around SensorPreprocessor.

Subscribes to canonical D435 topics plus /joint_states, pairs them, and
publishes a synchronised set of standard messages. SyncedObservation stays
inside the algorithm class; message conversion lives in
luggage_perception.ros_message_adapters.
"""

from __future__ import division

# Cap BLAS/OpenMP worker threads BEFORE numpy (transitively) loads: any
# residual gemm in this process would otherwise fan out over every core
# and busy-spin (PF-R9 B5 root cause; the hot path itself now avoids BLAS
# entirely — see sensor_preprocessor.transform_points).
import os as _os
for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS",
             "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    _os.environ.setdefault(_var, "1")

import json

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from std_msgs.msg import String
import tf2_ros

from luggage_perception import ros_message_adapters as adapters
from luggage_perception.motion_stability_filter import MotionStabilityGate
from luggage_perception.sensor_preprocessor import SensorPreprocessor


class SensorPreprocessorNode(Node):

    def __init__(self):
        super().__init__("sensor_preprocessor")
        self._declare_params()
        if bool(self.get_parameter("enable_lidar_output").value):
            raise RuntimeError(
                "enable_lidar_output is true, but per-point lidar deskew is "
                "not implemented; refusing to publish uncompensated geometry"
            )

        joint_names = list(
            self.get_parameter("motion_gate.joint_names").value or []
        )
        self._core = SensorPreprocessor(
            camera_maxlen=int(self.get_parameter("camera_maxlen").value),
            camera_horizon_sec=float(
                self.get_parameter("camera_horizon_sec").value),
            camera_slop_sec=float(self.get_parameter("camera_slop_sec").value),
            camera_pair_tolerance_sec=float(
                self.get_parameter("camera_pair_tolerance_sec").value),
            camera_wait_deadline_sec=float(
                self.get_parameter("camera_wait_deadline_sec").value),
            camera_emit_rgb_only=bool(
                self.get_parameter("camera_emit_rgb_only").value),
            camera_info_max_age_sec=float(
                self.get_parameter("camera_info_max_age_sec").value),
            joint_horizon_sec=float(
                self.get_parameter("joint_horizon_sec").value),
            output_cloud_frame=str(
                self.get_parameter("output_cloud_frame").value),
            stale_sec=float(self.get_parameter("stale_sec").value),
            motion_gate=MotionStabilityGate(
                joint_names=joint_names,
                velocity_threshold=float(
                    self.get_parameter("motion_gate.velocity_threshold").value),
                settle_time_sec=float(
                    self.get_parameter("motion_gate.settle_time_sec").value),
                joint_state_timeout_sec=float(
                    self.get_parameter(
                        "motion_gate.joint_state_timeout_sec").value),
                enabled=bool(self.get_parameter("motion_gate.enabled").value),
            ),
            joint_names=joint_names,
        )
        self._input_cloud_data_frame = str(
            self.get_parameter("input_cloud_data_frame").value).strip()
        self._output_cloud_frame = str(
            self.get_parameter("output_cloud_frame").value)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_warn_stamp = 0.0
        self._cb_counts = {
            "rgb": 0, "depth": 0, "cloud": 0, "info": 0, "joints": 0,
        }
        # PF-R9: per-stage cloud-callback wall timing (ms), last 32
        # samples, surfaced on /luggage/preprocessed/status for B3/B4
        # evidence and PF-R10 C2.
        import collections as _collections
        self._cloud_stage_ms = _collections.deque(maxlen=32)
        self._cloud_decimation_stride = max(
            1, int(self.get_parameter("cloud_decimation_stride").value))
        self._last_cloud_points_in = 0
        self._last_cloud_points_out = 0

        # gz ros_gz_bridge currently offers RELIABLE on these topics.
        # BEST_EFFORT subscriptions connect but barely deliver on this RMW
        # (recorded behaviour; the B1 probe re-measures both). The
        # reliability stays configurable so the measured choice lands in
        # the config, not the code.
        reliability = (
            ReliabilityPolicy.BEST_EFFORT
            if str(self.get_parameter("cloud_input_qos_reliability").value)
            == "best_effort"
            else ReliabilityPolicy.RELIABLE)
        input_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        cloud_qos = QoSProfile(
            depth=5,
            reliability=reliability,
            history=HistoryPolicy.KEEP_LAST,
        )
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        joint_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub_color = self.create_publisher(
            Image, self.get_parameter("output.color_image").value, sensor_qos)
        self._pub_color_info = self.create_publisher(
            CameraInfo, self.get_parameter("output.color_info").value, sensor_qos)
        self._pub_depth = self.create_publisher(
            Image, self.get_parameter("output.depth_image").value, sensor_qos)
        self._pub_depth_info = self.create_publisher(
            CameraInfo, self.get_parameter("output.depth_info").value, sensor_qos)
        self._pub_cloud = self.create_publisher(
            PointCloud2, self.get_parameter("output.camera_points").value, sensor_qos)
        self._pub_status = self.create_publisher(
            String, self.get_parameter("output.status").value, status_qos)

        # PF-R9 B5 note: a MultiThreadedExecutor with per-entity groups
        # was tried and abandoned on this stack — the 1 Hz status timer
        # and the parameter services were never serviced and cloud
        # dispatch latency stayed ~70 ms regardless of thread count (see
        # main() for the publisher-thread design that replaced it).
        self.create_subscription(
            Image, self.get_parameter("input.color_image").value,
            self._on_color, input_qos)
        self.create_subscription(
            Image, self.get_parameter("input.depth_image").value,
            self._on_depth, input_qos)
        self.create_subscription(
            CameraInfo, self.get_parameter("input.camera_info").value,
            self._on_camera_info, input_qos)
        color_info_topic = str(
            self.get_parameter("input.color_camera_info").value).strip()
        if color_info_topic:
            self.create_subscription(
                CameraInfo, color_info_topic, self._on_color_info, input_qos)
        self.create_subscription(
            PointCloud2, self.get_parameter("input.camera_points").value,
            self._on_cloud, cloud_qos)
        self.create_subscription(
            JointState, self.get_parameter("input.joint_states").value,
            self._on_joints, joint_qos)

        self.get_logger().info(
            "sensor_preprocessor ready: data_frame=%r output_frame=%s "
            "pair_tolerance=%.3fs wait_deadline=%.3fs cloud_qos=%s "
            "decimation=%d"
            % (self._input_cloud_data_frame or "<header>",
               self._output_cloud_frame,
               self._core.camera_pair_tolerance_sec,
               self._core.camera_wait_deadline_sec,
               self.get_parameter("cloud_input_qos_reliability").value,
               self._cloud_decimation_stride)
        )
        # PF-R9 B5: publishing (0.9-3.7 MB cloud messages) inside the
        # sensor callbacks blocked them (measured publish p95 233 ms at
        # full size); the blocked callback then dropped incoming messages
        # and capped emission at a few Hz. Sensor callbacks hand
        # observations to this bounded queue; the daemon publisher thread
        # in main() drains it, so a slow publish never head-of-line-blocks
        # the sensor path.
        self._emit_queue = _collections.deque(maxlen=4)
        self._emit_drops = 0

    def _declare_params(self):
        defaults = {
            "input.color_image": "/camera/color/image_raw",
            "input.depth_image": "/camera/depth/image_raw",
            "input.camera_info": "/camera/depth/camera_info",
            "input.color_camera_info": "",
            "input.camera_points": "/camera/depth/points",
            "input.joint_states": "/joint_states",
            "input.lidar": "/livox/lidar",
            "input.imu": "/livox/imu",
            "output.color_image": "/luggage/preprocessed/camera/color/image",
            "output.color_info": "/luggage/preprocessed/camera/color/camera_info",
            "output.depth_image": "/luggage/preprocessed/camera/depth/image",
            "output.depth_info": "/luggage/preprocessed/camera/depth/camera_info",
            "output.camera_points": "/luggage/preprocessed/camera/depth/points",
            "output.status": "/luggage/preprocessed/status",
            "input_cloud_data_frame": "camera_link",
            "output_cloud_frame": "camera_depth_optical_frame",
            "enable_lidar_output": False,
            "enable_imu": False,
            "camera_slop_sec": 0.020,
            # PF-R9 B2 split: pairing tolerance (~exact; gz co-stamps) and
            # wait deadline (RGB-stream clock, set from the B1 receipt-lag
            # p95; never advanced by /joint_states).
            "camera_pair_tolerance_sec": 0.005,
            "camera_wait_deadline_sec": 0.20,
            "camera_emit_rgb_only": False,
            "cloud_input_qos_reliability": "reliable",
            "cloud_decimation_stride": 2,
            "camera_info_max_age_sec": 1.0,
            "camera_maxlen": 10,
            "camera_horizon_sec": 0.35,
            "joint_horizon_sec": 1.0,
            "stale_sec": 0.15,
            "motion_gate.joint_names": [
                "elfin_joint1", "elfin_joint2", "elfin_joint3",
                "elfin_joint4", "elfin_joint5", "elfin_joint6",
            ],
            "motion_gate.velocity_threshold": 0.02,
            "motion_gate.settle_time_sec": 0.5,
            "motion_gate.joint_state_timeout_sec": 1.0,
            "motion_gate.enabled": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _on_color(self, msg):
        self._cb_counts["rgb"] += 1
        frame = adapters.rgb_frame_from_msg(msg)
        if frame is None:
            self._warn_throttled(
                "dropping color image with encoding %s" % msg.encoding)
            return
        self._handle(self._core.update_rgb(frame))

    def _on_depth(self, msg):
        self._cb_counts["depth"] += 1
        frame = adapters.depth_frame_from_msg(msg)
        if frame is None:
            self._warn_throttled(
                "dropping depth image with encoding %s" % msg.encoding)
            return
        self._handle(self._core.update_depth(frame))

    def _on_camera_info(self, msg):
        self._cb_counts["info"] += 1
        frame = adapters.camera_info_frame_from_msg(msg)
        self._handle(self._core.update_camera_info(frame, slots=("depth", "color")))

    def _on_color_info(self, msg):
        frame = adapters.camera_info_frame_from_msg(msg)
        self._handle(self._core.update_camera_info(frame, slots=("color",)))

    def _on_cloud(self, msg):
        import time as _time
        t0 = _time.monotonic()
        self._cb_counts["cloud"] += 1
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        data_frame = self._input_cloud_data_frame or msg.header.frame_id
        t1 = _time.monotonic()
        matrix = self._lookup_cloud_transform(data_frame, stamp)
        t2 = _time.monotonic()
        if matrix is None:
            self._note_cloud_stage(t0, t1, t2, None, None)
            return
        cloud = self._decode_cloud(msg, data_frame)
        t3 = _time.monotonic()
        if cloud is None:
            self._warn_throttled("dropping cloud with unsupported point layout")
            self._note_cloud_stage(t0, t1, t2, t3, None)
            return
        if self._cloud_decimation_stride > 1:
            cloud = self._decimate_cloud(cloud, msg)
        observation = self._core.update_camera_cloud(
            cloud, point_transform=matrix)
        t4 = _time.monotonic()
        self._handle(observation)
        t5 = _time.monotonic()
        self._note_cloud_stage(t0, t1, t2, t3, t4, t5)

    @staticmethod
    def _decode_cloud(msg, data_frame):
        """Fast path for the sensor's float32-XYZ-first cloud layout.

        The general adapter path (per-field gather + float64 astype)
        measured 21 ms per 307k cloud under load. When x/y/z are FLOAT32
        at offsets 0/4/8 — true for the gz points layout (point_step 24
        with rgb at 16) — a structured view plus one float32 stack is
        ~2 ms. Anything else falls back to the adapter.
        """
        if not msg.is_bigendian and msg.point_step >= 12:
            fields = {field.name: field for field in msg.fields}
            if ({"x", "y", "z"} <= set(fields)
                    and all(
                        fields[n].datatype == 7  # PointField.FLOAT32
                        and fields[n].count == 1
                        and fields[n].offset == 4 * i
                        for i, n in enumerate(("x", "y", "z")))):
                from luggage_perception.sensor_types import CameraCloud as _CC
                raw = np.frombuffer(msg.data, dtype=np.uint8)
                n = min(int(msg.width) * int(msg.height),
                        raw.size // int(msg.point_step))
                if n <= 0:
                    return adapters.camera_cloud_from_msg(msg, data_frame)
                structured = np.ndarray(
                    shape=(n,),
                    dtype=np.dtype({
                        "names": ["x", "y", "z"],
                        "formats": ["<f4", "<f4", "<f4"],
                        "offsets": [0, 4, 8],
                        "itemsize": int(msg.point_step)}),
                    buffer=raw[:n * int(msg.point_step)],
                    order="C")
                return _CC(
                    stamp=adapters.stamp_to_sec(msg.header.stamp),
                    frame_id=msg.header.frame_id,
                    data_frame=data_frame,
                    points=np.stack(
                        (structured["x"], structured["y"], structured["z"]),
                        axis=1),
                )
        return adapters.camera_cloud_from_msg(msg, data_frame)

    def _decimate_cloud(self, cloud, msg):
        """PF-R9 B5 (optional lever, reviews recommendation E).

        Stride-subsample the organized cloud (rows and columns) before it
        enters the pipeline: a 3.7 MB 307k-point publish was the measured
        bottleneck (publish p95 233 ms); stride 2 gives a 0.9 MB 77k-point
        cloud. The filter's (u,v) reprojection indexes the full-resolution
        mask, so no downstream change. Density gates: cargo points drop
        ~4x (measured ~29.5k -> ~7.4k, still far above min_points 50 /
        min_top_points 80); geometry non-regression is carried by the
        PF-R10 C1 bars. Unorganized clouds fall back to flat [::stride].
        """
        stride = int(self._cloud_decimation_stride)
        pts = np.asarray(cloud.points)
        if stride <= 1 or pts.ndim != 2 or pts.shape[0] < stride * stride:
            return cloud
        height, width = int(msg.height), int(msg.width)
        if (height > 1 and width > 1
                and height * width == pts.shape[0]):
            organized = pts.reshape(height, width, 3)
            decimated = organized[::stride, ::stride, :].reshape(-1, 3)
        else:
            decimated = pts[::stride]
        self._last_cloud_points_in = int(pts.shape[0])
        self._last_cloud_points_out = int(decimated.shape[0])
        from luggage_perception.sensor_types import CameraCloud as _CC
        return _CC(
            stamp=cloud.stamp,
            frame_id=cloud.frame_id,
            data_frame=cloud.data_frame,
            points=decimated,
            dropped_nonfinite=cloud.dropped_nonfinite,
        )

    def _note_cloud_stage(self, t0, t1, t2, t3, t4, t5=None):
        rec = {
            "pre": (t1 - t0) * 1000.0,
            "tf_lookup_ms": (t2 - t1) * 1000.0,
        }
        if t3 is not None:
            rec["decode_ms"] = (t3 - t2) * 1000.0
        if t4 is not None:
            rec["core_ms"] = (t4 - t3) * 1000.0
        if t5 is not None:
            # handle = enqueue; the actual publish runs on the daemon
            # publisher thread and is deliberately off this path.
            rec["handle_ms"] = (t5 - t4) * 1000.0
            rec["total_ms"] = (t5 - t0) * 1000.0
        self._cloud_stage_ms.append(rec)

    def _on_joints(self, msg):
        self._cb_counts["joints"] += 1
        sample = adapters.joint_sample_from_msg(
            msg, fallback_stamp_sec=self.get_clock().now().nanoseconds * 1e-9)
        if sample is None:
            return
        self._handle(self._core.update_joint_state(sample))

    def _lookup_cloud_transform(self, data_frame, stamp):
        if data_frame == self._output_cloud_frame:
            return np.eye(4, dtype=np.float64)
        timeout = rclpy.duration.Duration(seconds=0.0)
        try:
            if not self._tf_buffer.can_transform(
                    self._output_cloud_frame, data_frame, stamp, timeout):
                self._warn_throttled(
                    "TF %s <- %s not available at stamp %s"
                    % (self._output_cloud_frame, data_frame, stamp))
                return None
            tf_msg = self._tf_buffer.lookup_transform(
                self._output_cloud_frame, data_frame, stamp, timeout,
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self._warn_throttled(
                "TF %s <- %s failed: %s" % (
                    self._output_cloud_frame, data_frame, exc))
            return None
        return adapters.transform_matrix(tf_msg)

    def _warn_throttled(self, message, period_sec=2.0):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._tf_warn_stamp <= period_sec:
            return
        self._tf_warn_stamp = now
        self.get_logger().warning(message)

    def _handle(self, observation):
        if observation is None:
            return
        if len(self._emit_queue) >= self._emit_queue.maxlen:
            self._emit_drops += 1
        self._emit_queue.append(observation)

    def _publish_observation(self, obs):
        stamp = adapters.sec_to_stamp(obs.primary_stamp)
        if obs.rgb is not None and obs.flags.rgb_ok:
            self._pub_color.publish(
                adapters.image_msg_from_frame(obs.rgb, stamp))
        # PF-R9 B5: the 0.6 MB depth image has no consumer on the online
        # path (the filter consumes the cloud; the segmenter the RGB).
        # Serializing it per emission for zero readers cost a quarter of
        # the publish cycle; publish it only when someone listens.
        if (obs.depth is not None and obs.flags.depth_ok
                and self._pub_depth.get_subscription_count() > 0):
            self._pub_depth.publish(
                adapters.depth_msg_from_frame(obs.depth, stamp))
        if obs.color_info is not None and obs.flags.color_info_ok:
            self._pub_color_info.publish(
                adapters.camera_info_msg_from_frame(obs.color_info, stamp))
        if obs.depth_info is not None and obs.flags.depth_info_ok:
            self._pub_depth_info.publish(
                adapters.camera_info_msg_from_frame(obs.depth_info, stamp))
        if obs.camera_points is not None:
            self._pub_cloud.publish(adapters.cloud_msg_from_points(
                obs.camera_points, stamp, obs.frame_id))

    def _on_status_timer(self):
        payload = self._core.diagnostics()
        payload["callbacks"] = dict(self._cb_counts)
        payload["cloud_decimation_stride"] = self._cloud_decimation_stride
        payload["last_cloud_points_in"] = self._last_cloud_points_in
        payload["last_cloud_points_out"] = self._last_cloud_points_out
        payload["emit_queue_drops"] = self._emit_drops
        if self._cloud_stage_ms:
            import numpy as _np
            stage = {}
            for key in ("tf_lookup_ms", "decode_ms", "core_ms",
                        "handle_ms", "total_ms"):
                values = [rec[key] for rec in self._cloud_stage_ms
                          if key in rec]
                if values:
                    stage[key] = {
                        "p50": float(_np.percentile(values, 50)),
                        "p95": float(_np.percentile(values, 95)),
                        "max": float(max(values)),
                    }
            payload["cloud_stage_ms"] = stage
        self._publish_status_payload(payload)

    def _publish_status(self, obs):
        payload = self._core.diagnostics(now=obs.primary_stamp)
        payload["callbacks"] = dict(self._cb_counts)
        payload["flags"] = obs.flags.as_dict()
        payload["data_frame"] = obs.data_frame
        payload["frame_id"] = obs.frame_id
        payload["dropped_nonfinite"] = obs.dropped_nonfinite
        payload["depth_dt"] = obs.depth_dt
        payload["cloud_dt"] = obs.cloud_dt
        self._publish_status_payload(payload)

    def _publish_status_payload(self, payload):
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self._pub_status.publish(msg)


def main():
    rclpy.init()
    node = SensorPreprocessorNode()
    # Single-threaded spin: the PF-R9 B5 experiments with a
    # MultiThreadedExecutor (cloud/status/emit groups, 2-3 threads) showed
    # pathological scheduling on this stack — the 1 Hz status timer and
    # the parameter services were never serviced, and cloud-callback
    # dispatch latency stayed ~70 ms regardless of thread count. The
    # measured per-callback costs after the core fixes (float32, BLAS-free
    # transform, decimation, no double copy) are a few ms, so one thread
    # suffices; publishing stays on a plain daemon thread below so a slow
    # large-message publish cannot head-of-line-block sensor callbacks.
    import threading
    stop = threading.Event()

    def publisher_loop():
        import time as _time
        last_status = 0.0
        while not stop.is_set():
            try:
                observation = node._emit_queue.popleft()
            except IndexError:
                _time.sleep(0.002)
                continue
            node._publish_observation(observation)
            node._publish_status(observation)
            now = _time.monotonic()
            if now - last_status >= 1.0:
                last_status = now
                try:
                    node._on_status_timer()
                except Exception:  # noqa: BLE001 diagnostics must not kill
                    pass

    thread = threading.Thread(target=publisher_loop, daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        thread.join(timeout=2.0)
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
