#!/usr/bin/env python3
"""Thin ROS 2 node around SensorPreprocessor (depth-primary, PF-R9 g2).

Subscribes the canonical colour-aligned RGBD set (raw Image or compressed
JPEG/PNG) plus /joint_states, pairs them by exact payload stamp, and
republishes the accepted acquisition under its primary stamp. Raw input
keeps the zero-copy payload-view path; compressed D555 input is decoded once
at this boundary. The camera point cloud is not subscribed, waited for,
transformed, or published; depth-primary consumers unproject locally.
"""

from __future__ import division

# Cap BLAS/OpenMP worker threads BEFORE numpy (transitively) loads: any
# residual gemm in this process would otherwise fan out over every core
# and busy-spin (PF-R9 measured; the hot path avoids BLAS entirely — see
# sensor_preprocessor.transform_points).
import os as _os
for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS",
             "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    _os.environ.setdefault(_var, "1")

import collections as _collections
import json
import time as _time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState
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

        self._cb_counts = {
            "rgb": 0, "depth": 0, "info": 0, "joints": 0,
        }
        self._use_compressed = bool(
            self.get_parameter("input.use_compressed").value)
        # PF-R9 g2: complete-path stage timing (D1 instrument, retained as
        # diagnostics): receive->view, core, emit-queue wait, output
        # construction, per-product publish.
        self._d1 = _collections.defaultdict(
            lambda: _collections.deque(maxlen=256))
        self._d1_bytes = _collections.Counter()
        # Bounded emit queue; the daemon publisher thread in main() drains
        # it so a slow large-message publish never head-of-line-blocks the
        # sensor callbacks.
        self._emit_queue = _collections.deque(maxlen=4)
        self._emit_drops = 0
        self._seen_epoch = 0

        camera_reliability = (
            ReliabilityPolicy.BEST_EFFORT
            if str(self.get_parameter("camera_input_qos_reliability").value)
            == "best_effort"
            else ReliabilityPolicy.RELIABLE)
        input_qos = QoSProfile(
            depth=5,
            reliability=camera_reliability,
            history=HistoryPolicy.KEEP_LAST,
        )
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Decoded RGB is a product stream (RViz/rqt + perception), not the
        # D555 driver. RViz Image defaults to RELIABLE and will not connect
        # to BEST_EFFORT; BEST_EFFORT readers still match this writer.
        color_out_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        info_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
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
            Image, self.get_parameter("output.color_image").value, color_out_qos)
        self._pub_color_info = self.create_publisher(
            CameraInfo, self.get_parameter("output.color_info").value, info_qos)
        # Aligned depth is the mandatory geometry input: publish RELIABLE
        # so exact-stamp consumers (detector) never lose a frame. BEST_
        # EFFORT subscribers stay compatible.
        depth_pub_qos = QoSProfile(
            depth=15, reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST)
        self._pub_depth = self.create_publisher(
            Image, self.get_parameter("output.depth_image").value,
            depth_pub_qos)
        self._pub_depth_info = self.create_publisher(
            CameraInfo, self.get_parameter("output.depth_info").value, info_qos)
        self._pub_status = self.create_publisher(
            String, self.get_parameter("output.status").value, status_qos)

        color_type = CompressedImage if self._use_compressed else Image
        depth_type = CompressedImage if self._use_compressed else Image
        self.create_subscription(
            color_type, self.get_parameter("input.color_image").value,
            self._on_color, input_qos)
        self.create_subscription(
            depth_type, self.get_parameter("input.depth_image").value,
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
            JointState, self.get_parameter("input.joint_states").value,
            self._on_joints, joint_qos)

        self.get_logger().info(
            "sensor_preprocessor ready (depth-primary): frame=%s "
            "pair_tolerance=%.3fs wait_deadline=%.3fs maxlen=%d/%.1fs "
            "compressed=%s camera_qos=%s"
            % (self.get_parameter("output_cloud_frame").value,
               self._core.camera_pair_tolerance_sec,
               self._core.camera_wait_deadline_sec,
               int(self.get_parameter("camera_maxlen").value),
               float(self.get_parameter("camera_horizon_sec").value),
               self._use_compressed,
               self.get_parameter("camera_input_qos_reliability").value)
        )

    def _declare_params(self):
        defaults = {
            "input.color_image": "/camera/color/image_raw",
            "input.depth_image": "/camera/depth/image_raw",
            "input.camera_info": "/camera/depth/camera_info",
            "input.color_camera_info": "",
            "input.joint_states": "/joint_states",
            "input.use_compressed": False,
            "output.color_image": "/luggage/preprocessed/camera/color/image",
            "output.color_info": "/luggage/preprocessed/camera/color/camera_info",
            "output.depth_image": "/luggage/preprocessed/camera/depth/image",
            "output.depth_info": "/luggage/preprocessed/camera/depth/camera_info",
            "output.status": "/luggage/preprocessed/status",
            "output_cloud_frame": "camera_depth_optical_frame",
            "enable_lidar_output": False,
            "enable_imu": False,
            # PF-R9 g2 split: pairing tolerance (~exact; aligned streams
            # are co-stamped) and wait deadline (canonical camera clock,
            # never advanced by /joint_states).
            "camera_slop_sec": 0.005,
            "camera_pair_tolerance_sec": 0.005,
            "camera_wait_deadline_sec": 0.060,
            "camera_info_max_age_sec": 1.0,
            "camera_input_qos_reliability": "reliable",
            # PF-R9 g2 fixed camera cache contract.
            "camera_maxlen": 15,
            "camera_horizon_sec": 1.0,
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

    # ------------------------------------------------------------------
    # Sensor callbacks (payload-owning, no materialisation)
    # ------------------------------------------------------------------

    def _d1_note(self, label, dt_sec):
        self._d1[label].append(dt_sec * 1000.0)

    def _on_color(self, msg):
        t0 = _time.monotonic()
        self._cb_counts["rgb"] += 1
        if self._use_compressed:
            frame = adapters.rgb_frame_from_compressed_msg(msg)
            hint = "format %s" % getattr(msg, "format", "")
        else:
            frame = adapters.rgb_frame_from_msg(msg)
            hint = "encoding %s" % getattr(msg, "encoding", "")
        t1 = _time.monotonic()
        self._d1_note("rgb_view_ms", t1 - t0)
        if frame is None:
            self._warn_throttled(
                "dropping colour image with unsupported/truncated layout "
                "(%s)" % hint)
            return
        observation = self._core.update_rgb(frame)
        t2 = _time.monotonic()
        self._d1_note("rgb_core_ms", t2 - t1)
        self._handle(observation)

    def _on_depth(self, msg):
        t0 = _time.monotonic()
        self._cb_counts["depth"] += 1
        if self._use_compressed:
            frame = adapters.depth_frame_from_compressed_msg(msg)
            hint = "format %s" % getattr(msg, "format", "")
        else:
            frame = adapters.depth_frame_from_msg(msg)
            hint = "encoding %s" % getattr(msg, "encoding", "")
        t1 = _time.monotonic()
        self._d1_note("depth_view_ms", t1 - t0)
        if frame is None:
            self._warn_throttled(
                "dropping depth image with unsupported/truncated layout "
                "(%s)" % hint)
            return
        observation = self._core.update_depth(frame)
        t2 = _time.monotonic()
        self._d1_note("depth_core_ms", t2 - t1)
        self._handle(observation)

    def _on_camera_info(self, msg):
        self._cb_counts["info"] += 1
        frame = adapters.camera_info_frame_from_msg(msg)
        if frame is None:
            return
        self._handle(self._core.update_camera_info(
            frame, slots=("depth", "color")))

    def _on_color_info(self, msg):
        frame = adapters.camera_info_frame_from_msg(msg)
        if frame is None:
            return
        self._handle(self._core.update_camera_info(frame, slots=("color",)))

    def _on_joints(self, msg):
        self._cb_counts["joints"] += 1
        sample = adapters.joint_sample_from_msg(
            msg, fallback_stamp_sec=self.get_clock().now().nanoseconds * 1e-9)
        if sample is None:
            return
        self._handle(self._core.update_joint_state(sample))

    def _warn_throttled(self, message, period_sec=2.0):
        now = self.get_clock().now().nanoseconds * 1e-9
        last = getattr(self, "_warn_stamp", 0.0)
        if now - last <= period_sec:
            return
        self._warn_stamp = now
        self.get_logger().warning(message)

    # ------------------------------------------------------------------
    # Emit queue (publisher thread drains it; see main)
    # ------------------------------------------------------------------

    def _handle(self, observation):
        if observation is None:
            return
        if self._core.camera_epoch != self._seen_epoch:
            # Rollback epoch: drop every pending pre-rollback output.
            self._emit_queue.clear()
            self._seen_epoch = self._core.camera_epoch
        if len(self._emit_queue) >= self._emit_queue.maxlen:
            self._emit_drops += 1
        self._emit_queue.append((_time.monotonic(), observation))

    def _publish_observation(self, obs):
        """Identity republish: headers/metadata rewritten, payloads shared."""
        stamp = adapters.sec_to_stamp(obs.primary_stamp)
        b0 = _time.monotonic()
        msgs = []
        if obs.rgb is not None and obs.flags.rgb_ok:
            out = adapters.image_msg_from_frame(obs.rgb, stamp)
            self._d1_bytes["color"] += len(out.data)
            msgs.append((self._pub_color, out, "color"))
        # Publish-on-demand stays as idle/debug hygiene only (no D3
        # credit): in the accepted graph colour and depth both have
        # subscribers.
        if (obs.depth is not None and obs.flags.depth_ok
                and self._pub_depth.get_subscription_count() > 0):
            out = adapters.depth_msg_from_frame(obs.depth, stamp)
            self._d1_bytes["depth"] += len(out.data)
            msgs.append((self._pub_depth, out, "depth"))
        if obs.color_info is not None and obs.flags.color_info_ok:
            msgs.append((self._pub_color_info,
                         adapters.camera_info_msg_from_frame(
                             obs.color_info, stamp), "info"))
        if obs.depth_info is not None and obs.flags.depth_info_ok:
            msgs.append((self._pub_depth_info,
                         adapters.camera_info_msg_from_frame(
                             obs.depth_info, stamp), "info"))
        b1 = _time.monotonic()
        self._d1_note("build_total_ms", b1 - b0)
        for pub, msg, kind in msgs:
            p0 = _time.monotonic()
            pub.publish(msg)
            p1 = _time.monotonic()
            self._d1_note("pub_%s_ms" % kind, p1 - p0)

    def _on_status_timer(self):
        payload = self._core.diagnostics()
        payload["callbacks"] = dict(self._cb_counts)
        payload["emit_queue_drops"] = self._emit_drops
        payload["emit_queue_depth"] = len(self._emit_queue)
        if self._d1:
            import numpy as _np
            d1 = {}
            for label, values in sorted(self._d1.items()):
                if values:
                    d1[label] = {
                        "n": len(values),
                        "p50": float(_np.percentile(values, 50)),
                        "p95": float(_np.percentile(values, 95)),
                        "max": float(max(values)),
                    }
            payload["d1_stage_ms"] = d1
            payload["d1_bytes_total"] = dict(self._d1_bytes)
        self._publish_status_payload(payload)

    def _publish_status(self, obs):
        payload = self._core.diagnostics(now=obs.primary_stamp)
        payload["callbacks"] = dict(self._cb_counts)
        payload["flags"] = obs.flags.as_dict()
        payload["emit_queue_depth"] = len(self._emit_queue)
        payload["depth_dt"] = obs.depth_dt
        self._publish_status_payload(payload)

    def _publish_status_payload(self, payload):
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self._pub_status.publish(msg)



def _start_malloc_trim_timer(interval_sec=10.0):
    """PF-R10 C2: release freed arena tails to the OS on a low-rate timer."""
    import ctypes
    import threading
    try:
        libc = ctypes.CDLL("libc.so.6")
        libc.malloc_trim.argtypes = [ctypes.c_size_t]
        trim = libc.malloc_trim
    except Exception:
        return None
    stop = threading.Event()

    def _tick():
        while not stop.wait(interval_sec):
            try:
                trim(0)
            except Exception:
                return

    threading.Thread(target=_tick, daemon=True).start()
    return stop


def main():
    rclpy.init()
    node = SensorPreprocessorNode()
    node._trim_stop = _start_malloc_trim_timer()
    # Single-threaded spin + daemon publisher thread (PF-R9 measured form:
    # a MultiThreadedExecutor starved timers/services on this stack).
    import threading
    stop = threading.Event()

    def publisher_loop():
        last_status = 0.0
        while not stop.is_set():
            try:
                enqueued_at, observation = node._emit_queue.popleft()
            except IndexError:
                _time.sleep(0.002)
                continue
            node._d1_note("queue_wait_ms", _time.monotonic() - enqueued_at)
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
