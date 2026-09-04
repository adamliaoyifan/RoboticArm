#!/usr/bin/env python3
"""Dump a few aligned color PNG + depth.npy frames from the live D435.

Gate 3 evidence. Do not reuse docs/status/evidence/m2_occlusion simulation npy.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def _image_to_array(msg: Image):
    if msg.encoding in ("rgb8", "bgr8"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == "bgr8":
            arr = arr[:, :, ::-1].copy()
        return arr
    if msg.encoding == "16UC1":
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        return depth.astype(np.float32) / 1000.0
    if msg.encoding == "32FC1":
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    raise RuntimeError("unsupported encoding %s" % msg.encoding)


class Dumper(Node):
    def __init__(self, out_dir: str, count: int):
        super().__init__("dump_camera_frames")
        self._out = out_dir
        self._need = int(count)
        self._color = None
        self._depth = None
        self._saved = 0
        os.makedirs(out_dir, exist_ok=True)
        self.create_subscription(Image, "/camera/color/image_raw", self._on_color, 10)
        self.create_subscription(Image, "/camera/camera/color/image_raw", self._on_color, 10)
        self.create_subscription(Image, "/camera/depth/image_raw", self._on_depth, 10)
        self.create_subscription(
            Image, "/camera/aligned_depth_to_color/image_raw", self._on_depth, 10
        )
        self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw", self._on_depth, 10
        )
        self.create_subscription(
            Image, "/camera/camera/depth/image_raw", self._on_depth, 10
        )

    def _on_color(self, msg):
        self._color = msg
        self._try_save()

    def _on_depth(self, msg):
        self._depth = msg
        self._try_save()

    def _try_save(self):
        if self._saved >= self._need or self._color is None or self._depth is None:
            return
        if self._color.header.stamp != self._depth.header.stamp:
            # Keep going; next pair may match. Still dump latest pair after timeout
            # in main() if needed.
            return
        self._write(self._color, self._depth)
        self._color = None
        self._depth = None

    def flush_latest(self):
        if self._saved >= self._need:
            return
        if self._color is None or self._depth is None:
            return
        self._write(self._color, self._depth)

    def _write(self, color_msg, depth_msg):
        idx = self._saved
        color = _image_to_array(color_msg)
        depth = _image_to_array(depth_msg)
        np.save(os.path.join(self._out, "frame_%02d_depth.npy" % idx), depth)
        try:
            from PIL import Image as PILImage  # noqa: PLC0415
            PILImage.fromarray(color).save(
                os.path.join(self._out, "frame_%02d_color.png" % idx)
            )
        except ImportError:
            np.save(os.path.join(self._out, "frame_%02d_color.npy" % idx), color)
        self.get_logger().info("saved frame %d to %s" % (idx, self._out))
        self._saved += 1

    @property
    def done(self):
        return self._saved >= self._need


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out",
        default=os.path.expanduser("~/robotarm_site_frames"),
        help="directory for color png + depth npy",
    )
    parser.add_argument("--count", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args(argv)

    rclpy.init(args=[])
    node = Dumper(args.out, args.count)
    deadline = time.time() + args.timeout
    try:
        while rclpy.ok() and not node.done and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.done:
            node.flush_latest()
        if not node.done:
            node.get_logger().error(
                "only saved %s/%s frames (is the D435 publishing?)"
                % (node._saved, args.count)
            )
            return 1
        print("OK: %s frames in %s" % (args.count, args.out))
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
