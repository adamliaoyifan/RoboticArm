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
from luggage_msgs.msg import DetectedLuggage
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

        # Sensor QoS contract (plan section 10): best effort, depth 1.
        sensor_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        topic = (
            self.get_parameter("cargo_cloud_topic").value
            if self._use_semantic
            else self.get_parameter("depth_topic").value
        )
        self.create_subscription(
            PointCloud2, topic, self._cloud_cb, sensor_qos,
            callback_group=self._group)
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

    def _rgb_cb(self, msg):
        image = adapters.image_array_from_msg(msg)
        if image is None:
            return
        with self._view_lock:
            self._latest_rgb = image

    def _on_current_box(self, msg):
        box_id = ""
        if msg.data:
            try:
                data = json.loads(msg.data)
            except (TypeError, ValueError):
                data = None
            if isinstance(data, dict):
                box_id = str(data.get("id") or data.get("model_name") or "")
        with self._view_lock:
            rgb = self._latest_rgb
            changed = self._view_wait.note_box_id(box_id, rgb)
        if changed and box_id:
            self.get_logger().info(
                "luggage_detector: waiting for suitcase RGB update (%s)"
                % box_id)

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

    def _estimate_from_cloud(self):
        """Run the full perception pipeline.

        Returns (DetectedLuggage, confidence) or (None, 0.0).
        """
        with self._cloud_lock:
            cloud_msg = self._latest_cloud
            stamp = self._latest_stamp
            frame = self._latest_frame
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

        # Vectorized decode via the shared adapter. The previous
        # sensor_msgs_py generator unpacked ~110k points one-by-one into
        # Python objects (about a second per call, GIL-held, starving the
        # cloud subscription - the back-to-back DETECT_STALE_CLOUD bug);
        # see docs/status/todo1_semantic_chain.md (GIL section).
        _t0 = time.monotonic()
        pts_camera = adapters.cloud_points_from_msg(cloud_msg)
        if pts_camera is None:
            self._last_failure_reason = "DETECT_CLOUD_DECODE_FAILED"
            self._warn_throttled(
                "luggage_detector: dropping cloud with unsupported layout")
            return None, 0.0
        # cloud_points_from_msg does not drop non-finite values (the old
        # read_points path only skipped NaN anyway). Far-plane misses
        # arrive as inf and silently break RANSAC/PCA. See
        # docs/status/m2_perception_occlusion_problem.md (Bug 2).
        pts_camera = pts_camera[np.isfinite(pts_camera).all(axis=1)]
        self._timing["read_ms"] = (time.monotonic() - _t0) * 1000.0
        if len(pts_camera) < self._min_points:
            self._last_failure_reason = "DETECT_TOO_FEW_POINTS"
            self._warn_throttled(
                "luggage_detector: too few points (%d)" % len(pts_camera))
            return None, 0.0

        _t0 = time.monotonic()
        source_frame = self._cloud_data_frame or frame
        pts_world, tf_err = _transform_points_to_world(
            self._tf_buffer, pts_camera, source_frame,
            self._world_frame, stamp_time,
        )
        if pts_world is None:
            self._last_failure_reason = "DETECT_TF_FAILED"
            self._warn_throttled(
                "luggage_detector: TF %s->%s failed: %s"
                % (source_frame, self._world_frame, tf_err))
            return None, 0.0
        self._timing["tf_ms"] = (time.monotonic() - _t0) * 1000.0

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
            self._last_failure_reason = "DETECT_ESTIMATION_FAILED"
            self._warn_throttled("luggage_detector: box estimation failed")
            return None, 0.0
        if est.confidence < self._min_confidence:
            self._last_failure_reason = "DETECT_LOW_CONFIDENCE"
            self._warn_throttled(
                "luggage_detector: confidence %.2f below %.2f"
                % (est.confidence, self._min_confidence))
            return None, est.confidence

        box_id = est.matched_catalog_id or "detected_box"
        t = self._timing
        self.get_logger().info(
            "detect timing: read=%.1fms tf=%.1fms voxel=%.1fms(%d->%d) "
            "ransac=%.1fms refine=%.1fms"
            % (t.get("read_ms", -1), t.get("tf_ms", -1), t.get("voxel_ms", 0.0),
               t.get("voxel_from", -1), t.get("voxel_to", -1),
               t.get("ransac_ms", -1), t.get("refine_ms", -1)))
        self.get_logger().info(
            "luggage_detector: estimated box '%s' at (%.3f, %.3f, %.3f) "
            "size=(%.3f, %.3f, %.3f) conf=%.2f yaw_valid=%s aspect=%.2f"
            % (box_id, est.center_xyz[0], est.center_xyz[1], est.center_xyz[2],
               est.width, est.depth, est.height, est.confidence,
               est.yaw_valid, est.aspect_ratio))
        self._last_failure_reason = "ok"
        return DetectedLuggage(
            id=box_id,
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
        ), est.confidence

    def _publish_diagnostics(self, source, success, confidence, reason,
                             detected=None, gt=None):
        record = {
            "stamp": self.get_clock().now().nanoseconds / 1e9,
            "source": source,
            "success": bool(success),
            "confidence": float(confidence),
            "reason": str(reason),
            "allow_gt_fallback": self._allow_gt_fallback,
        }
        if self._last_cloud_stamp_sec is not None:
            record["cloud_stamp"] = float(self._last_cloud_stamp_sec)
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
