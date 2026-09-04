#!/usr/bin/env python3
"""Luggage detector: estimate pickup-box pose from the depth point cloud.

ROS 2 port of the noetic luggage_detector_node. Subscribes to the semantic
cargo cloud (or raw depth as fallback), runs RANSAC + PCA box fitting
(``luggage_box_estimator.estimate_box``), transforms the result into world
frame, and serves ``detect_luggage``.

Differences from the ROS 1 node, per the migration plan section 9:
- no ``rospy.set_param`` state: the detection record is published on
  ``~/diagnostics_json`` and ``/luggage/perception/detection/latest``
  (transient local, replaces the latched param);
- the GT fallback box comes from the ``pickup_box_spawner`` service
  (``allow_gt_fallback`` default false: no spawn GT on the real robot);
- ``DetectLuggage`` waits until RGB leaves the pre-spawn suitcase view
  (``/luggage/current_box`` id change) so PCA is not run on a stale GPU
  frame. Timeout is ``DETECT_SUITCASE_NOT_UPDATED``, not GT fallback.
- ``DetectLuggage`` refuses cargo whose ``stats_json.generation`` does not
  match the current box (``DETECT_STALE_INSTANCE``). It does not wait for
  preprocessor ``geometry_ok``; the cargo tracker keeps the latest
  associated cloud.
- YOLO + cargo are exact-stamp joined and PCA runs every joined frame on
  ``/luggage/perception/detection_frame``. ``DetectLuggage`` reads that
  window (generation gated) instead of fitting again.
"""

from __future__ import division

import json
import threading
import time

import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)

from geometry_msgs.msg import Point, Pose, Quaternion
from luggage_msgs.msg import DetectedLuggage, DetectionFrame, YoloDetections
from luggage_msgs.srv import DetectLuggage, GetCurrentBox
from luggage_perception import ros_message_adapters as adapters
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import String
import tf2_ros

from luggage_description.box_catalog_utils import (
    box_catalog_entries,
    load_box_catalog,
)
from luggage_description.scene_tf_config_utils import (
    load_scene_tf_config,
    pickup_source_in_world,
    resolve_scene_tf_config_path,
)
from luggage_perception.luggage_box_estimator import estimate_box
from luggage_perception.detection_temporal_gate import (
    SuitcaseViewWait,
    should_retry_estimate,
)
from luggage_perception.cargo_instance_tracker import parse_current_box_payload
from luggage_perception.detection_frame_join import (
    ExactStampJoin,
    empty_cargo_pca_fields,
    pca_fields_from_estimate,
    pca_fields_from_failure,
    stamp_key,
)
from luggage_perception.locked_stamp_window import LockedStampWindow
from luggage_perception.motion_stability_filter import (
    detection_replay_fields,
)


def _transform_points_to_world(tf_buffer, points, source_frame, target_frame, stamp):
    """Rigid-body transform an (N,3) array from *source_frame* to *target_frame*."""
    try:
        tf_msg = tf_buffer.lookup_transform(
            target_frame, source_frame, stamp, rclpy.duration.Duration(seconds=0.5)
        )
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as exc:
        return None, str(exc)

    t = tf_msg.transform.translation
    r = tf_msg.transform.rotation
    qx, qy, qz, qw = r.x, r.y, r.z, r.w
    rot = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ])
    trans = np.array([t.x, t.y, t.z])
    return points.dot(rot.T) + trans, None


class LuggageDetector(Node):

    def __init__(self):
        super().__init__("luggage_detector")
        self._group = ReentrantCallbackGroup()

        self.declare_parameter("scene_tf_config", "")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("use_semantic", False)
        self.declare_parameter("roi_margin", 0.5)
        self.declare_parameter("cloud_max_age_sec", 1.0)
        self.declare_parameter("catalog_match_tolerance", 0.08)
        self.declare_parameter("catalog_snap_enabled", False)
        self.declare_parameter("min_points", 50)
        self.declare_parameter("min_confidence", 0.70)
        self.declare_parameter("min_height_above_platform", 0.03)
        self.declare_parameter("allow_gt_fallback", False)
        self.declare_parameter("evaluation_compare_gt", False)
        self.declare_parameter(
            "current_box_service", "/pickup_box_spawner/get_current_box")
        self.declare_parameter(
            "cargo_cloud_topic", "/luggage/semantic/cargo_points")
        self.declare_parameter(
            "depth_topic", "/luggage/preprocessed/camera/depth/points")
        # Empty means use the cloud header frame_id. The preprocessor publishes
        # optical-frame points; override only if a consumer still sees raw gz
        # clouds labelled optical but stored in camera_link.
        self.declare_parameter("cloud_data_frame", "")
        # Uniform-grid downsample inside estimate_box, applied after the
        # ROI/platform band crop. 0.01 m is statistically equivalent for
        # plane/rectangle estimation while shrinking the fitting stages'
        # input ~20x (cargo clouds ~110k -> ~5k points); 0 disables.
        # Supported range <= 0.02 m (confidence is count-based; see the
        # estimator docstring).
        self.declare_parameter("voxel_size", 0.01)
        self.declare_parameter("estimate_retry_count", 4)
        self.declare_parameter("estimate_retry_period_sec", 0.25)
        # RGB MAD wait before PCA. Kept in tree but off: meas-vs-image
        # overlay alignment is the check that matters. 0 disables.
        self.declare_parameter("suitcase_update_timeout_sec", 0.0)
        self.declare_parameter("suitcase_update_mad", 10.0)
        self.declare_parameter("suitcase_stable_mad", 4.0)
        self.declare_parameter("suitcase_stable_frames", 2)
        self.declare_parameter(
            "color_topic", "/luggage/preprocessed/camera/color/image")
        self.declare_parameter("current_box_topic", "/luggage/current_box")
        self.declare_parameter(
            "preprocessor_status_topic", "/luggage/preprocessed/status")
        self.declare_parameter(
            "filter_stats_topic", "/semantic_point_filter/stats_json")
        self.declare_parameter(
            "yolo_topic", "/luggage/semantic/yolo_detections")
        self.declare_parameter(
            "detection_frame_topic", "/luggage/perception/detection_frame")
        self.declare_parameter("join_buffer_maxlen", 10)

        scene_cfg_path = self.get_parameter("scene_tf_config").value
        if not scene_cfg_path:
            scene_cfg_path = resolve_scene_tf_config_path()
        scene_config = load_scene_tf_config(scene_cfg_path)
        self._source_xyz, _ = pickup_source_in_world(scene_config)
        self._platform_z = self._source_xyz[2]

        catalog_config = load_box_catalog(scene_config=scene_config)
        self._catalog_entries = box_catalog_entries(catalog_config)

        self._world_frame = self.get_parameter("world_frame").value
        self._use_semantic = bool(self.get_parameter("use_semantic").value)
        self._cloud_data_frame = self.get_parameter("cloud_data_frame").value
        self._voxel_size = float(self.get_parameter("voxel_size").value)
        self._estimate_retry_count = max(
            0, int(self.get_parameter("estimate_retry_count").value))
        self._estimate_retry_period = max(
            0.0, float(self.get_parameter("estimate_retry_period_sec").value))
        self._suitcase_update_timeout = max(
            0.0, float(self.get_parameter("suitcase_update_timeout_sec").value))
        self._view_wait = SuitcaseViewWait(
            update_mad=float(self.get_parameter("suitcase_update_mad").value),
            stable_mad=float(self.get_parameter("suitcase_stable_mad").value),
            stable_frames=int(self.get_parameter("suitcase_stable_frames").value),
        )
        self._view_lock = threading.Lock()
        self._latest_rgb = None
        self._timing = {}
        self._roi_margin = float(self.get_parameter("roi_margin").value)
        self._cloud_max_age = float(self.get_parameter("cloud_max_age_sec").value)
        self._catalog_tol = float(
            self.get_parameter("catalog_match_tolerance").value)
        self._catalog_snap = bool(
            self.get_parameter("catalog_snap_enabled").value)
        self._min_points = int(self.get_parameter("min_points").value)
        self._min_confidence = float(self.get_parameter("min_confidence").value)
        self._min_height_above_platform = float(
            self.get_parameter("min_height_above_platform").value)
        self._allow_gt_fallback = bool(
            self.get_parameter("allow_gt_fallback").value)
        self._evaluation_compare_gt = bool(
            self.get_parameter("evaluation_compare_gt").value)
        self._last_failure_reason = "not_run"
        self._last_cloud_stamp_sec = None
        self._status = {"payload": None}
        self._filter_stats = None
        self._box_epoch_seen = False
        self._box_id = ""
        self._box_generation = 0
        self._join = ExactStampJoin(
            maxlen=max(1, int(self.get_parameter("join_buffer_maxlen").value)))
        self._frame_window = LockedStampWindow(
            maxlen=max(1, int(self.get_parameter("join_buffer_maxlen").value)))
        self._frame_seq = 0
        self._frame_lock = threading.Lock()

        # Transient local replaces the ROS 1 latched diagnostics publisher.
        transient = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._diag_pub = self.create_publisher(
            String, "~/diagnostics_json", transient)
        self._latest_pub = self.create_publisher(
            String, "/luggage/perception/detection/latest", transient)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._cloud_lock = threading.Lock()
        self._latest_cloud = None
        self._latest_stamp = None
        self._latest_frame = None

        stream_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        topic = (
            self.get_parameter("cargo_cloud_topic").value
            if self._use_semantic
            else self.get_parameter("depth_topic").value
        )
        self.create_subscription(
            PointCloud2, topic, self._cloud_cb, stream_qos,
            callback_group=self._group)
        self.create_subscription(
            YoloDetections, self.get_parameter("yolo_topic").value,
            self._yolo_cb, stream_qos, callback_group=self._group)
        self._frame_pub = self.create_publisher(
            DetectionFrame,
            self.get_parameter("detection_frame_topic").value,
            stream_qos)
        self.get_logger().info("luggage_detector subscribing to %s" % topic)

        image_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            Image, self.get_parameter("color_topic").value,
            self._rgb_cb, image_qos, callback_group=self._group)
        self.create_subscription(
            String, self.get_parameter("current_box_topic").value,
            self._on_current_box, transient, callback_group=self._group)
        self.create_subscription(
            String, self.get_parameter("preprocessor_status_topic").value,
            self._on_status, transient, callback_group=self._group)
        self.create_subscription(
            String, self.get_parameter("filter_stats_topic").value,
            self._on_filter_stats, transient, callback_group=self._group)

        box_service = self.get_parameter("current_box_service").value
        self._current_box_cli = self.create_client(GetCurrentBox, box_service)

        self.create_service(
            DetectLuggage, "/luggage_detector/detect_luggage", self.handle_detect,
            callback_group=self._group)
        self.get_logger().info(
            "luggage_detector ready (perception mode, semantic=%s, "
            "retries=%d period=%.2fs suitcase_wait=%.1fs)"
            % (self._use_semantic, self._estimate_retry_count,
               self._estimate_retry_period, self._suitcase_update_timeout))

    def _cloud_cb(self, msg):
        with self._cloud_lock:
            self._latest_cloud = msg
            self._latest_stamp = msg.header.stamp
            self._latest_frame = msg.header.frame_id
        key = stamp_key(msg.header.stamp)
        if key is None:
            return
        if self._use_semantic:
            pair = self._join.push_right(key, msg)
            if pair is not None:
                self._emit_joined(pair[0], pair[1])
            return
        self._emit_joined(self._empty_yolo_for_cloud(msg), msg)

    def _yolo_cb(self, msg):
        key = stamp_key(msg.header.stamp)
        if key is None:
            return
        pair = self._join.push_left(key, msg)
        if pair is not None:
            self._emit_joined(pair[0], pair[1])

    def _empty_yolo_for_cloud(self, cloud_msg):
        msg = YoloDetections()
        msg.header = cloud_msg.header
        msg.generation = int(self._box_generation)
        msg.instance_id = str(self._box_id)
        return msg

    def _rgb_cb(self, msg):
        image = adapters.image_array_from_msg(msg)
        if image is None:
            return
        with self._view_lock:
            self._latest_rgb = image

    def _on_current_box(self, msg):
        box_id, generation = parse_current_box_payload(msg.data)
        self._box_epoch_seen = True
        self._box_id = box_id
        self._box_generation = generation
        self._join.clear()
        self._frame_window.clear()
        with self._view_lock:
            rgb = self._latest_rgb
            changed = self._view_wait.note_box_id(box_id, rgb)
        if changed and box_id:
            self.get_logger().info(
                "luggage_detector: waiting for suitcase RGB update (%s)"
                % box_id)

    def _on_filter_stats(self, msg):
        if not msg.data:
            return
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(data, dict):
            self._filter_stats = data

    def _on_status(self, msg):
        self._status["payload"] = msg.data

    def _status_data(self):
        payload = self._status.get("payload")
        if not payload:
            return None
        try:
            data = json.loads(payload)
        except (TypeError, ValueError):
            return None
        return data if isinstance(data, dict) else None

    def _instance_gate_reason(self):
        """None when DetectLuggage may run. Skip when no current_box (real robot)."""
        if not self._use_semantic or not self._box_epoch_seen:
            return None
        if not self._box_id:
            return "DETECT_STALE_INSTANCE"
        stats = self._filter_stats
        if not stats:
            return "DETECT_STALE_INSTANCE"
        try:
            gen = int(stats.get("generation") or 0)
            raw = stats.get("last_cargo_n_points")
            if raw is None:
                raw = stats.get("n_points", 0)
            n_points = int(raw if raw is not None else 0)
        except (TypeError, ValueError):
            return "DETECT_STALE_INSTANCE"
        if gen != int(self._box_generation):
            return "DETECT_STALE_INSTANCE"
        if n_points <= 0:
            return "DETECT_NO_CLOUD"
        return None

    def _wait_instance_ready(self):
        """Block until tracker generation matches current_box, or timeout."""
        if not self._use_semantic or not self._box_epoch_seen:
            return True
        timeout = max(
            0.5,
            (1.0 + float(self._estimate_retry_count))
            * float(self._estimate_retry_period))
        deadline = time.time() + timeout
        while time.time() < deadline:
            reason = self._instance_gate_reason()
            if reason is None:
                return True
            self._last_failure_reason = reason
            time.sleep(0.02)
        if self._instance_gate_reason() is None:
            return True
        return False

    def _wait_suitcase_view(self):
        """Block until RGB leaves the pre-spawn view, or timeout.

        Returns True when detect may run. Timeout is a hard miss: the pixels
        are still the previous suitcase, so GT fallback would hide that.
        """
        if self._suitcase_update_timeout <= 0.0:
            return True
        with self._view_lock:
            if not self._view_wait.pending:
                return True
            if self._latest_rgb is None:
                # Raw-depth launches may have no preprocessor RGB.
                return True
        deadline = time.time() + self._suitcase_update_timeout
        while time.time() < deadline:
            with self._view_lock:
                rgb = self._latest_rgb
                self._view_wait.set_baseline_if_missing(rgb)
                ready = self._view_wait.observe(rgb)
            if ready:
                prev = self._cloud_stamp_key()
                remaining = deadline - time.time()
                self._wait_newer_cloud(prev, max(0.0, min(0.5, remaining)))
                return True
            time.sleep(0.02)
        return False

    def _cloud_stamp_key(self):
        with self._cloud_lock:
            stamp = self._latest_stamp
        if stamp is None:
            return None
        return (int(stamp.sec), int(stamp.nanosec))

    def _wait_newer_cloud(self, prev_key, timeout):
        """Block until cargo/depth stamp changes, or *timeout* elapses."""
        if timeout <= 0.0:
            return False
        deadline = time.time() + float(timeout)
        while time.time() < deadline:
            time.sleep(0.02)
            key = self._cloud_stamp_key()
            if key is not None and key != prev_key:
                return True
        return False

    def _estimate_with_retries(self):
        attempts = 1 + int(self._estimate_retry_count)
        detected, confidence = None, 0.0
        for attempt in range(attempts):
            prev_key = self._cloud_stamp_key()
            detected, confidence = self._estimate_from_cloud()
            if detected is not None:
                if attempt:
                    self.get_logger().info(
                        "luggage_detector: estimate ok on retry %d/%d"
                        % (attempt, self._estimate_retry_count))
                return detected, confidence
            if attempt + 1 >= attempts:
                break
            if not should_retry_estimate(self._last_failure_reason):
                break
            self._wait_newer_cloud(prev_key, self._estimate_retry_period)
        return detected, confidence

    # ------------------------------------------------------------------
    # GT fallback
    # ------------------------------------------------------------------

    def _gt_fallback(self):
        """Return DetectedLuggage from the spawner service, or None."""
        if not self._current_box_cli.wait_for_service(timeout_sec=0.2):
            return None
        event = threading.Event()
        future = self._current_box_cli.call_async(GetCurrentBox.Request())

        def _done(_fut):
            event.set()

        future.add_done_callback(_done)
        if not event.wait(timeout=2.0):
            return None
        resp = future.result()
        if resp is not None and resp.success:
            return resp.box
        return None

    # ------------------------------------------------------------------
    # Perception path
    # ------------------------------------------------------------------

    def _warn_throttled(self, msg):
        # Replaces rospy.logwarn_throttle; 5 s window on the node clock.
        now = self.get_clock().now().nanoseconds
        last = getattr(self, "_last_warn_ns", 0)
        if now - last > 5e9:
            self.get_logger().warning(msg)
            self._last_warn_ns = now

    def _pca_source_label(self, n_points):
        if int(n_points) <= 0:
            return "empty"
        stats = self._filter_stats or {}
        source = str(stats.get("source") or "")
        if source in ("measure", "hold_track", "empty"):
            return source
        return "measure"

    def _detected_from_estimate(self, est):
        return DetectedLuggage(
            id=est.matched_catalog_id or "detected_box",
            width=est.width,
            depth=est.depth,
            height=est.height,
            yaw_valid=bool(est.yaw_valid),
            aspect_ratio=float(est.aspect_ratio),
            pose=Pose(
                position=Point(
                    x=float(est.center_xyz[0]),
                    y=float(est.center_xyz[1]),
                    z=float(est.center_xyz[2]),
                ),
                orientation=Quaternion(
                    x=float(est.quaternion_xyzw[0]),
                    y=float(est.quaternion_xyzw[1]),
                    z=float(est.quaternion_xyzw[2]),
                    w=float(est.quaternion_xyzw[3]),
                ),
            ),
        )

    def _pca_from_cloud_msg(self, cloud_msg):
        """Fit a world-frame box from one cargo/depth cloud.

        Returns ``(pca_fields, DetectedLuggage or None)``. Always returns
        fields so the stream can publish ``pca_valid=false`` frames.
        """
        stamp = cloud_msg.header.stamp
        frame = cloud_msg.header.frame_id
        stamp_time = rclpy.time.Time.from_msg(stamp)
        _t0 = time.monotonic()
        pts_camera = adapters.cloud_points_from_msg(cloud_msg)
        if pts_camera is None:
            return pca_fields_from_failure(
                "DETECT_CLOUD_DECODE_FAILED", 0, "empty"), None
        pts_camera = pts_camera[np.isfinite(pts_camera).all(axis=1)]
        self._timing["read_ms"] = (time.monotonic() - _t0) * 1000.0
        n_points = int(len(pts_camera))
        source = self._pca_source_label(n_points)
        if n_points <= 0:
            return empty_cargo_pca_fields(0), None
        if n_points < self._min_points:
            return pca_fields_from_failure(
                "DETECT_TOO_FEW_POINTS", n_points, source), None

        _t0 = time.monotonic()
        source_frame = self._cloud_data_frame or frame
        pts_world, tf_err = _transform_points_to_world(
            self._tf_buffer, pts_camera, source_frame,
            self._world_frame, stamp_time,
        )
        if pts_world is None:
            return pca_fields_from_failure(
                "DETECT_TF_FAILED", n_points, source), None
        self._timing["tf_ms"] = (time.monotonic() - _t0) * 1000.0
        centroid = tuple(float(v) for v in pts_world.mean(axis=0))

        est = estimate_box(
            pts_world,
            roi_center_xy=(self._source_xyz[0], self._source_xyz[1]),
            roi_margin=self._roi_margin,
            platform_z=self._platform_z,
            catalog_entries=(
                self._catalog_entries if self._catalog_snap else None),
            catalog_tolerance=self._catalog_tol,
            min_points=self._min_points,
            min_height_above_platform=self._min_height_above_platform,
            voxel_size=self._voxel_size,
            timing=self._timing,
        )
        if est is None:
            return pca_fields_from_failure(
                "DETECT_ESTIMATION_FAILED", n_points, source, centroid), None
        if est.confidence < self._min_confidence:
            fields = pca_fields_from_failure(
                "DETECT_LOW_CONFIDENCE", n_points, source,
                tuple(float(v) for v in est.center_xyz))
            fields["pca_confidence"] = float(est.confidence)
            return fields, None
        return pca_fields_from_estimate(est, n_points, source), (
            self._detected_from_estimate(est))

    def _make_detection_frame(self, yolo_msg, cloud_msg, fields, box):
        msg = DetectionFrame()
        msg.header.stamp = cloud_msg.header.stamp
        msg.header.frame_id = self._world_frame
        msg.yolo_optical_frame = (
            yolo_msg.header.frame_id or cloud_msg.header.frame_id)
        with self._frame_lock:
            msg.frame_seq = self._frame_seq
            self._frame_seq += 1
        msg.generation = int(yolo_msg.generation)
        msg.instance_id = str(yolo_msg.instance_id)
        msg.yolo = list(yolo_msg.detections)
        msg.pca_valid = bool(fields["pca_valid"])
        msg.pca_reason = str(fields["pca_reason"])
        msg.pca_source = str(fields["pca_source"])
        msg.pca_confidence = float(fields["pca_confidence"])
        msg.n_cargo_points = int(fields["n_cargo_points"])
        cx, cy, cz = fields["centroid"]
        msg.centroid = Point(x=float(cx), y=float(cy), z=float(cz))
        if box is not None and fields["pca_valid"]:
            msg.box = box
        return msg

    def _emit_joined(self, yolo_msg, cloud_msg):
        fields, box = self._pca_from_cloud_msg(cloud_msg)
        if not fields["pca_valid"]:
            reason = fields["pca_reason"]
            if reason in (
                    "DETECT_CLOUD_DECODE_FAILED",
                    "DETECT_TF_FAILED",
                    "DETECT_TOO_FEW_POINTS"):
                self._warn_throttled(
                    "luggage_detector: stream %s (n=%d)"
                    % (reason, fields["n_cargo_points"]))
        frame = self._make_detection_frame(yolo_msg, cloud_msg, fields, box)
        self._frame_pub.publish(frame)
        self._frame_window.push(
            adapters.stamp_to_sec(cloud_msg.header.stamp), frame)

    def _wait_newer_frame(self, prev_stamp, timeout):
        if timeout <= 0.0:
            return False
        deadline = time.time() + float(timeout)
        while time.time() < deadline:
            time.sleep(0.02)
            hit = self._frame_window.latest()
            if hit is not None and hit[0] != prev_stamp:
                return True
        return False

    def _frame_to_detect(self, frame):
        stamp_time = rclpy.time.Time.from_msg(frame.header.stamp)
        age = (self.get_clock().now() - stamp_time).nanoseconds / 1e9
        self._last_cloud_stamp_sec = (
            float(frame.header.stamp.sec)
            + 1e-9 * float(frame.header.stamp.nanosec))
        if age > self._cloud_max_age:
            self._last_failure_reason = "DETECT_STALE_CLOUD"
            return None, 0.0
        if self._use_semantic and self._box_epoch_seen:
            if int(frame.generation) != int(self._box_generation):
                self._last_failure_reason = "DETECT_STALE_INSTANCE"
                return None, 0.0
        if not frame.pca_valid:
            self._last_failure_reason = str(
                frame.pca_reason or "DETECT_ESTIMATION_FAILED")
            return None, float(frame.pca_confidence)
        self._last_failure_reason = "ok"
        return frame.box, float(frame.pca_confidence)

    def _detect_from_window_with_retries(self):
        attempts = 1 + int(self._estimate_retry_count)
        detected, confidence = None, 0.0
        prev_stamp = None
        for attempt in range(attempts):
            hit = self._frame_window.latest()
            if hit is None:
                self._last_failure_reason = "DETECT_NO_CLOUD"
            else:
                prev_stamp, frame = hit
                detected, confidence = self._frame_to_detect(frame)
                if detected is not None:
                    if attempt:
                        self.get_logger().info(
                            "luggage_detector: window ok on retry %d/%d"
                            % (attempt, self._estimate_retry_count))
                    return detected, confidence
            if attempt + 1 >= attempts:
                break
            if not should_retry_estimate(self._last_failure_reason):
                break
            self._wait_newer_frame(prev_stamp, self._estimate_retry_period)
        return detected, confidence

    def _estimate_from_cloud(self):
        """Run the full perception pipeline on the latest cloud.

        Used when the DetectionFrame window is empty (raw-depth path).
        """
        with self._cloud_lock:
            cloud_msg = self._latest_cloud
            stamp = self._latest_stamp
        self._last_cloud_stamp_sec = None
        if stamp is not None:
            self._last_cloud_stamp_sec = (
                float(stamp.sec) + 1e-9 * float(stamp.nanosec))

        if cloud_msg is None:
            self._last_failure_reason = "DETECT_NO_CLOUD"
            self._warn_throttled("luggage_detector: no point cloud received yet")
            return None, 0.0

        stamp_time = rclpy.time.Time.from_msg(stamp)
        age = (self.get_clock().now() - stamp_time).nanoseconds / 1e9
        if age > self._cloud_max_age:
            self._last_failure_reason = "DETECT_STALE_CLOUD"
            self._warn_throttled(
                "luggage_detector: cloud too old (%.2fs)" % age)
            return None, 0.0

        fields, box = self._pca_from_cloud_msg(cloud_msg)
        self._last_failure_reason = (
            "ok" if fields["pca_valid"] else fields["pca_reason"])
        if fields["pca_valid"]:
            t = self._timing
            self.get_logger().info(
                "detect timing: read=%.1fms tf=%.1fms voxel=%.1fms(%d->%d) "
                "ransac=%.1fms refine=%.1fms"
                % (t.get("read_ms", -1), t.get("tf_ms", -1),
                   t.get("voxel_ms", 0.0),
                   t.get("voxel_from", -1), t.get("voxel_to", -1),
                   t.get("ransac_ms", -1), t.get("refine_ms", -1)))
            self.get_logger().info(
                "luggage_detector: estimated box '%s' at (%.3f, %.3f, %.3f) "
                "size=(%.3f, %.3f, %.3f) conf=%.2f yaw_valid=%s aspect=%.2f"
                % (box.id, box.pose.position.x, box.pose.position.y,
                   box.pose.position.z, box.width, box.depth, box.height,
                   fields["pca_confidence"], box.yaw_valid, box.aspect_ratio))
            return box, fields["pca_confidence"]
        self._warn_throttled(
            "luggage_detector: %s" % self._last_failure_reason)
        return None, fields["pca_confidence"]

    def _publish_diagnostics(self, source, success, confidence, reason,
                             detected=None, gt=None):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        ctype = self.get_clock().clock_type
        clock_name = ctype.name if hasattr(ctype, "name") else str(ctype)
        record = {
            "stamp": now_sec,
            "source": source,
            "success": bool(success),
            "confidence": float(confidence),
            "reason": str(reason),
            "allow_gt_fallback": self._allow_gt_fallback,
        }
        record.update(detection_replay_fields(
            self._status_data(),
            self._last_cloud_stamp_sec,
            now_sec,
            clock_name,
        ))
        record["box_generation"] = int(self._box_generation)
        record["box_id"] = str(self._box_id)
        stats = self._filter_stats or {}
        record["cargo_generation"] = int(stats.get("generation") or 0)
        record["cargo_n_points"] = int(
            stats.get("last_cargo_n_points", stats.get("n_points", 0)) or 0)
        record["cargo_source"] = str(stats.get("source") or "")
        if detected is not None:
            record["detected"] = {
                "id": detected.id,
                "position": [
                    detected.pose.position.x,
                    detected.pose.position.y,
                    detected.pose.position.z,
                ],
                "orientation": [
                    detected.pose.orientation.x,
                    detected.pose.orientation.y,
                    detected.pose.orientation.z,
                    detected.pose.orientation.w,
                ],
                "size": [detected.width, detected.depth, detected.height],
                "yaw_valid": bool(getattr(detected, "yaw_valid", False)),
                "aspect_ratio": float(getattr(detected, "aspect_ratio", 0.0)),
            }
        if gt is not None and detected is not None:
            record["gt_delta"] = [
                detected.pose.position.x - gt.pose.position.x,
                detected.pose.position.y - gt.pose.position.y,
                detected.pose.position.z - gt.pose.position.z,
            ]
        payload = String(data=json.dumps(record, sort_keys=True))
        self._diag_pub.publish(payload)
        self._latest_pub.publish(payload)

    # ------------------------------------------------------------------
    # Service handler
    # ------------------------------------------------------------------

    def handle_detect(self, _req, _response):
        response = DetectLuggage.Response()
        try:
            if not self._wait_suitcase_view():
                self._last_failure_reason = "DETECT_SUITCASE_NOT_UPDATED"
                self._publish_diagnostics(
                    "perception", False, 0.0, self._last_failure_reason)
                response.luggage = []
                response.success = False
                response.message = self._last_failure_reason
                self.get_logger().warning(
                    "luggage_detector: suitcase RGB did not update "
                    "within %.1fs" % self._suitcase_update_timeout)
                return response
            if not self._wait_instance_ready():
                reason = self._last_failure_reason or "DETECT_STALE_INSTANCE"
                self._last_failure_reason = reason
                self._publish_diagnostics(
                    "perception", False, 0.0, self._last_failure_reason)
                response.luggage = []
                response.success = False
                response.message = self._last_failure_reason
                self.get_logger().warning(
                    "luggage_detector: cargo instance not ready (%s)"
                    % self._last_failure_reason)
                return response
            detected, confidence = self._detect_from_window_with_retries()
            if detected is None and not self._use_semantic:
                detected, confidence = self._estimate_with_retries()
        except Exception as exc:  # noqa: BLE001 - service boundary
            self.get_logger().error("detect_luggage handler failed: %s" % exc)
            response.luggage = []
            response.success = False
            response.message = "DETECT_HANDLER_ERROR: %s" % exc
            return response

        if detected is not None:
            gt = self._gt_fallback() if self._evaluation_compare_gt else None
            if gt is not None:
                self.get_logger().info(
                    "luggage_detector: perception vs GT delta dx=%.4f dy=%.4f dz=%.4f"
                    % (detected.pose.position.x - gt.pose.position.x,
                       detected.pose.position.y - gt.pose.position.y,
                       detected.pose.position.z - gt.pose.position.z))
            self._publish_diagnostics(
                "perception", True, confidence, "ok", detected, gt)
            response.luggage = [detected]
            response.success = True
            response.message = "perception estimate (conf=%.2f)" % confidence
            return response

        if self._allow_gt_fallback:
            self.get_logger().warning(
                "luggage_detector: perception failed - falling back to GT")
            gt = self._gt_fallback()
            if gt is not None:
                self._publish_diagnostics(
                    "gt_fallback", True, confidence,
                    self._last_failure_reason, gt, None)
                response.luggage = [gt]
                response.success = True
                response.message = "gt fallback (perception unavailable)"
                return response

        self.get_logger().warning(
            "luggage_detector: strict perception failed (%s)"
            % self._last_failure_reason)
        self._publish_diagnostics(
            "perception", False, confidence, self._last_failure_reason)
        response.luggage = []
        response.success = False
        response.message = self._last_failure_reason
        return response


def main(argv=None):
    rclpy.init(args=argv)
    node = LuggageDetector()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
