#!/usr/bin/env python3
"""RGB -> semantic label mask node (ROS 2 Humble port).

Thin node shell around ``SemanticSegmenter`` (see
docs/architecture/perception_architecture.md): subscribes the preprocessed
RGB stream, runs the segmenter, publishes the label mask.

The suction-panel silhouette is a fixed image-space mask (camera bolted to
the panel). The node builds it from the collision meshes plus the mount TF
(or the URDF chain in yaml) and live camera_info; the algorithm only paints
the boolean mask.

Subscribes the *preprocessed* color image so the mask inherits the RGB
``primary_stamp`` and the downstream point filter can join on exact stamps
with the preprocessed cloud.

The mask/instance-mask/overlay outputs are BEST_EFFORT (sensor-stream
contract); ``~/stats_json`` is transient-local so a late consumer sees the
last record.
"""

from __future__ import division

import json
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from ament_index_python.packages import get_package_share_directory

from luggage_perception import ros_message_adapters as adapters
from luggage_perception.detect_overlay import (
    draw_timestamp_banner,
    timestamp_banner_lines,
)
from luggage_perception.semantic_segmenter import (
    build_segmenter,
    draw_detections_overlay,
)
from luggage_perception.wrist_self_body import (
    matrix_from_translation_quaternion,
    optical_from_panel_matrix,
    panel_mask_from_meshes,
)


class SemanticSegmenterNode(Node):

    def __init__(self):
        super().__init__("semantic_segmenter")
        defaults = {
            "backend": "stub",
            "model_name": "yolov8s-world.pt",
            "device": "cuda",
            "confidence_threshold": 0.25,
            "sam2.checkpoint": "facebook/sam2-hiera-small",
            "sam2.model_type": "sam2_hiera_s",
            "prompts": [""],
            "class_mapping_labels": [-1],
            # Non-empty and not a prefix of stats["backend"] -> RuntimeError
            # at startup, so an eval never silently runs the stub fallback.
            "require_backend": "",
            # 0 = unlimited. YOLO on CPU is slower than the 4-6 Hz input;
            # unlimited processing would pile callbacks up.
            "max_rate_hz": 0.0,
            "publish_overlay": True,
            "temporal_window_frames": 5,
            "temporal_min_positive_ratio": 0.5,
            "temporal_scene_change_mad": 10.0,
            "temporal_bbox_iou_reset": 0.3,
            "self_body_row_start_frac": 0.0,
            "self_body_from_mesh": False,
            "self_body_dilate_px": 0,
            "self_body_mesh_package": "luggage_gazebo",
            "self_body_mesh_dir": "models/suction_panel/collision",
            "self_body_meshes": [""],
            "self_body_mesh_origin": [0.0, 0.0, 0.0],
            "self_body_camera_frame": "camera_depth_optical_frame",
            "self_body_panel_frame": "suction_panel",
            "self_body_adapter_xyz": [0.0, 0.0, 0.0],
            "self_body_adapter_rpy": [0.0, 0.0, 0.0],
            "self_body_camera_xyz": [0.0, 0.0, 0.0],
            "self_body_camera_rpy": [0.0, 0.0, 0.0],
            "input.color_image": "/luggage/preprocessed/camera/color/image",
            "input.camera_info": "/luggage/preprocessed/camera/color/camera_info",
            "output.mask": "/luggage/semantic/mask",
            "output.overlay": "/luggage/semantic/overlay",
            "output.instance_mask": "/luggage/semantic/instance_mask",
            "output.stats": "~/stats_json",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        prompts = [str(p) for p in self.get_parameter("prompts").value]
        labels = [int(v) for v in self.get_parameter("class_mapping_labels").value]
        if labels and len(labels) != len(prompts):
            raise RuntimeError(
                "class_mapping_labels length %d != prompts length %d"
                % (len(labels), len(prompts)))
        class_mapping = dict(zip(prompts, labels)) if labels else None

        config = {
            "backend": self.get_parameter("backend").value,
            "model_name": self._resolve_model(
                self.get_parameter("model_name").value),
            "prompts": prompts,
            "class_mapping": class_mapping,
            "confidence_threshold": float(
                self.get_parameter("confidence_threshold").value),
            "device": self.get_parameter("device").value,
            "temporal_window_frames": int(
                self.get_parameter("temporal_window_frames").value),
            "temporal_min_positive_ratio": float(
                self.get_parameter("temporal_min_positive_ratio").value),
            "temporal_scene_change_mad": float(
                self.get_parameter("temporal_scene_change_mad").value),
            "temporal_bbox_iou_reset": float(
                self.get_parameter("temporal_bbox_iou_reset").value),
            "sam2": {
                "checkpoint": self.get_parameter("sam2.checkpoint").value,
                "model_type": self.get_parameter("sam2.model_type").value,
            },
            "self_body_row_start_frac": float(
                self.get_parameter("self_body_row_start_frac").value),
        }
        self._segmenter = build_segmenter(config)
        backend = self._segmenter.last_stats["backend"]
        require = str(self.get_parameter("require_backend").value)
        if require and not backend.startswith(require):
            raise RuntimeError(
                "semantic backend %r does not match require_backend %r"
                % (backend, require))
        self._max_interval = (
            0.0 if float(self.get_parameter("max_rate_hz").value) <= 0.0
            else 1.0 / float(self.get_parameter("max_rate_hz").value))
        self._last_process_sec = 0.0
        self._overlay_ok = bool(self.get_parameter("publish_overlay").value)
        self._overlay_missing_warned = False
        self._drop_count = 0
        self._camera_info = None
        self._self_body_source = "none"
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        sensor_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        mask_qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        stats_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._mask_pub = self.create_publisher(
            Image, self.get_parameter("output.mask").value, mask_qos)
        self._overlay_pub = self.create_publisher(
            Image, self.get_parameter("output.overlay").value, mask_qos)
        self._instance_pub = self.create_publisher(
            Image, self.get_parameter("output.instance_mask").value, mask_qos)
        self._stats_pub = self.create_publisher(
            String, self.get_parameter("output.stats").value, stats_qos)

        self.create_subscription(
            Image, self.get_parameter("input.color_image").value,
            self._on_image, sensor_qos)
        self.create_subscription(
            CameraInfo, self.get_parameter("input.camera_info").value,
            self._on_camera_info, sensor_qos)
        self._rebuild_self_body_mask()

        self.get_logger().info(
            "semantic_segmenter ready (backend=%s, prompts=%d, self_body=%s, "
            "temporal_window=%s)"
            % (backend, len(prompts), self._self_body_source,
               getattr(self._segmenter.temporal_gate, "window_size", 0)))

    @staticmethod
    def _resolve_model(model_name):
        """Bare filenames resolve against the package share models dir so a
        missing local file never triggers ultralytics' network download."""
        import os
        if os.path.isfile(model_name):
            return model_name
        candidate = os.path.join(
            get_package_share_directory("luggage_perception"),
            "models", os.path.basename(model_name))
        if os.path.isfile(candidate):
            return candidate
        return model_name

    def _on_camera_info(self, msg):
        self._camera_info = msg
        self._rebuild_self_body_mask()

    def _mesh_paths(self):
        names = [str(n) for n in self.get_parameter("self_body_meshes").value
                 if str(n)]
        pkg = str(self.get_parameter("self_body_mesh_package").value)
        rel = str(self.get_parameter("self_body_mesh_dir").value)
        try:
            root = os.path.join(get_package_share_directory(pkg), rel)
        except Exception:
            return []
        return [os.path.join(root, name) for name in names
                if os.path.isfile(os.path.join(root, name))]

    def _panel_to_optical(self):
        cam = str(self.get_parameter("self_body_camera_frame").value)
        panel = str(self.get_parameter("self_body_panel_frame").value)
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                cam, panel, rclpy.time.Time())
            trans = tf_msg.transform.translation
            rot = tf_msg.transform.rotation
            return matrix_from_translation_quaternion(
                (trans.x, trans.y, trans.z),
                (rot.x, rot.y, rot.z, rot.w)), "tf"
        except TransformException:
            pass
        xyz_a = list(self.get_parameter("self_body_adapter_xyz").value)
        rpy_a = list(self.get_parameter("self_body_adapter_rpy").value)
        xyz_c = list(self.get_parameter("self_body_camera_xyz").value)
        rpy_c = list(self.get_parameter("self_body_camera_rpy").value)
        if len(xyz_a) != 3 or len(xyz_c) != 3:
            return None, None
        return optical_from_panel_matrix(xyz_a, rpy_a, xyz_c, rpy_c), "urdf_chain"

    def _rebuild_self_body_mask(self):
        if not bool(self.get_parameter("self_body_from_mesh").value):
            return
        paths = self._mesh_paths()
        if not paths:
            return
        info = self._camera_info
        if info is not None and info.width > 0 and info.height > 0:
            fx, fy = float(info.k[0]), float(info.k[4])
            cx, cy = float(info.k[2]), float(info.k[5])
            width, height = int(info.width), int(info.height)
        else:
            fx = fy = 337.22194822727283
            cx, cy = 320.0, 240.0
            width, height = 640, 480
        t_mat, source = self._panel_to_optical()
        if t_mat is None:
            return
        key = (width, height, round(fx, 3), round(fy, 3), source)
        if (key == getattr(self, "_self_body_key", None)
                and self._segmenter.self_body_mask is not None):
            return
        origin = [float(v) for v in
                  self.get_parameter("self_body_mesh_origin").value]
        dilate = int(self.get_parameter("self_body_dilate_px").value)
        try:
            mask = panel_mask_from_meshes(
                paths, origin, t_mat, fx, fy, cx, cy, width, height, dilate)
        except (IOError, ValueError, OSError) as exc:
            self._warn_throttled("self-body mesh mask failed: %s" % exc)
            return
        if not mask.any():
            self._warn_throttled("self-body mesh mask is empty")
            return
        prev = self._self_body_source
        self._segmenter.self_body_mask = mask
        self._self_body_source = source
        self._self_body_key = key
        if prev != source:
            self.get_logger().info(
                "suction-panel mask %d px source=%s" % (int(mask.sum()), source))

    # ------------------------------------------------------------------

    def _on_image(self, msg):
        recv_wall = time.time()
        frame = adapters.rgb_frame_from_msg(msg)
        if frame is None:
            self._warn_throttled(
                "dropping color image with encoding %s" % msg.encoding)
            return
        if (self._max_interval > 0.0
                and frame.stamp - self._last_process_sec < self._max_interval):
            self._drop_count += 1
            return
        self._last_process_sec = frame.stamp
        if self._segmenter.self_body_mask is None:
            self._rebuild_self_body_mask()

        t0 = time.monotonic()
        self._segmenter.update(frame.image, frame.stamp, frame.frame_id)
        out = self._segmenter.copy_output()
        stamp = adapters.sec_to_stamp(out.stamp)
        pub_ms = (time.monotonic() - t0) * 1000.0

        self._mask_pub.publish(adapters.mask_msg_from_array(
            out.label_map, stamp, out.frame_id))
        if out.instance_map is not None:
            self._instance_pub.publish(adapters.instance_mask_msg_from_array(
                out.instance_map, stamp, out.frame_id))
        self._publish_overlay(frame.image, out, stamp, pub_ms=pub_ms)
        self._publish_stats(out, recv_wall=recv_wall)

    def _publish_overlay(self, rgb_image, out, stamp, pub_ms=None):
        if not self._overlay_ok:
            return
        try:
            bgr = draw_detections_overlay(rgb_image, list(out.detections))
            now_sec = self.get_clock().now().nanoseconds / 1e9
            infer_ms = (out.stats or {}).get("inference_ms")
            lines, meta = timestamp_banner_lines(
                out.stamp, out.stamp, dump_stamp=now_sec,
                infer_ms=infer_ms, pub_ms=pub_ms)
            bgr = draw_timestamp_banner(
                bgr, lines, aligned=bool(meta.get("aligned")), is_bgr=True)
        except ImportError:
            if not self._overlay_missing_warned:
                self.get_logger().warning(
                    "cv2 unavailable: overlay disabled")
                self._overlay_missing_warned = True
            self._overlay_ok = False
            return
        overlay = Image()
        overlay.header.stamp = stamp
        overlay.header.frame_id = out.frame_id
        overlay.height = int(bgr.shape[0])
        overlay.width = int(bgr.shape[1])
        overlay.encoding = "bgr8"
        overlay.is_bigendian = 0
        overlay.step = overlay.width * 3
        overlay.data = bgr.tobytes()
        self._overlay_pub.publish(overlay)

    def _publish_stats(self, out, recv_wall=None):
        record = dict(out.stats)
        record["stamp"] = out.stamp
        record["image_stamp"] = out.stamp
        record["frame_id"] = out.frame_id
        record["detect_sim_stamp"] = (
            self.get_clock().now().nanoseconds / 1e9)
        record["detect_wall_sec"] = time.time()
        if recv_wall is not None:
            record["recv_wall_sec"] = float(recv_wall)
        record["rate_limited_drops"] = self._drop_count
        record["self_body_source"] = self._self_body_source
        if self._segmenter.self_body_mask is not None:
            record["self_body_pixels"] = int(self._segmenter.self_body_mask.sum())
        self._stats_pub.publish(String(data=json.dumps(record, sort_keys=True)))

    def _warn_throttled(self, text):
        now = self.get_clock().now().nanoseconds
        last = getattr(self, "_warn_ns", 0)
        if now - last > 5e9:
            self.get_logger().warning(text)
            self._warn_ns = now


def main(argv=None):
    rclpy.init(args=argv)
    node = SemanticSegmenterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
