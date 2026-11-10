#!/usr/bin/env python3
"""Record one complete synchronized RGB/depth/info/TF/detection bundle.

B1's "one complete RGBD/TF/detection input": the same stamp on color,
depth, camera_info and the YOLO detection message, with the TF transform
chain effective at that stamp. Saved serialized + meta for offline
twice-replay through the production association/authorization chain.

usage: b1_capture.py <out_dir>
"""
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, CameraInfo
from luggage_msgs.msg import YoloDetections

TOLERANCE_S = 0.02


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class BundleRecorder(Node):
    def __init__(self, out_dir):
        super().__init__("g3_b1_capture")
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)
        self.color = None
        self.depth = None
        self.info = None
        self.yolo = None
        self.tf_frames = []
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, "/camera/color/image_raw",
                                 self._on_color, best_effort)
        self.create_subscription(Image, "/camera/depth/image_raw",
                                 self._on_depth, best_effort)
        self.create_subscription(CameraInfo, "/camera/color/camera_info",
                                 self._on_info, best_effort)
        self.create_subscription(YoloDetections, "/luggage/semantic/yolo_detections",
                                 self._on_yolo, best_effort)
        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_color(self, msg):
        self.color = msg

    def _on_depth(self, msg):
        self.depth = msg

    def _on_info(self, msg):
        self.info = msg

    def _on_yolo(self, msg):
        self.yolo = msg

    def complete(self, require_detection=True):
        if self.color is None or self.depth is None or self.info is None or self.yolo is None:
            return False
        stamps = [stamp_sec(self.color.header.stamp),
                  stamp_sec(self.depth.header.stamp),
                  stamp_sec(self.info.header.stamp),
                  stamp_sec(self.yolo.header.stamp)]
        if max(stamps) - min(stamps) > TOLERANCE_S:
            return False
        if require_detection and not self.yolo.detections:
            return False
        return True

    def save(self):
        frame_id = self.color.header.frame_id
        stamp = self.color.header.stamp
        # TF chain effective at the joined stamp: the frames the
        # deprojection/association chain resolves at runtime.
        tf_records = {}
        for target in ("world", "elfin_base_link"):
            for source in (frame_id, "suction_contact_frame"):
                if target == source:
                    continue
                try:
                    tf = self.tf_buffer.lookup_transform(
                        target, source, stamp,
                        timeout=rclpy.duration.Duration(seconds=0.0))
                    tf_records["%s<-%s" % (target, source)] = {
                        "translation": [tf.transform.translation.x,
                                        tf.transform.translation.y,
                                        tf.transform.translation.z],
                        "rotation": [tf.transform.rotation.x,
                                     tf.transform.rotation.y,
                                     tf.transform.rotation.z,
                                     tf.transform.rotation.w],
                    }
                except Exception as exc:  # noqa: BLE001 - record the miss
                    tf_records["%s<-%s" % (target, source)] = {"error": str(exc)[:120]}
        payloads = {
            "color.bin": (self.color, "sensor_msgs/msg/Image"),
            "depth.bin": (self.depth, "sensor_msgs/msg/Image"),
            "camera_info.bin": (self.info, "sensor_msgs/msg/CameraInfo"),
            "yolo.bin": (self.yolo, "luggage_msgs/msg/YoloDetections"),
        }
        for name, (msg, msg_type) in payloads.items():
            with open(os.path.join(self.out_dir, name), "wb") as handle:
                handle.write(serialize_message(msg))
        meta = {
            "tolerance_s": TOLERANCE_S,
            "stamps": {
                "color": stamp_sec(self.color.header.stamp),
                "depth": stamp_sec(self.depth.header.stamp),
                "camera_info": stamp_sec(self.info.header.stamp),
                "yolo": stamp_sec(self.yolo.header.stamp),
            },
            "frame_id": frame_id,
            "color": {"width": self.color.width, "height": self.color.height,
                      "encoding": self.color.encoding},
            "depth": {"width": self.depth.width, "height": self.depth.height,
                      "encoding": self.depth.encoding},
            "n_detections": len(self.yolo.detections),
            "detections": [{
                "label": str(d.label), "confidence": float(d.confidence),
                "has_mask": bool(getattr(d, "mask_array", None) is not None
                                 and len(d.mask_array) > 0),
            } for d in self.yolo.detections],
            "tf": tf_records,
        }
        with open(os.path.join(self.out_dir, "meta.json"), "w") as handle:
            json.dump(meta, handle, indent=1)
            handle.write("\n")
        return meta


def main():
    out_dir = sys.argv[1]
    rclpy.init()
    node = BundleRecorder(out_dir)
    deadline = time.time() + 60.0
    saved = False
    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.complete(require_detection=True):
                meta = node.save()
                print(json.dumps({"saved": True, "detection_frame": True,
                                  "stamps": meta["stamps"],
                                  "n_detections": meta["n_detections"]}))
                saved = True
                break
        if not saved and node.complete(require_detection=False):
            meta = node.save()
            print(json.dumps({"saved": True, "detection_frame": False,
                              "stamps": meta["stamps"],
                              "n_detections": meta["n_detections"]}))
            saved = True
        if not saved:
            print(json.dumps({"saved": False,
                              "color": node.color is not None,
                              "depth": node.depth is not None,
                              "info": node.info is not None,
                              "yolo": node.yolo is not None}))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if saved else 1


if __name__ == "__main__":
    sys.exit(main())
