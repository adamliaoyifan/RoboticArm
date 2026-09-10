import unittest

import numpy as np
from sensor_msgs.msg import Image

from elfin_trajectory_executor.rgbd_codec import (
    COLOR_JPEG_FORMAT,
    DEPTH_PNG_FORMAT,
    canonicalize_color_compressed,
    canonicalize_depth_compressed,
    color_to_jpeg,
    depth_to_png,
    jpeg_to_color,
    png_to_depth,
)


def _rgb8(h=48, w=64, val=40):
    msg = Image()
    msg.height = h
    msg.width = w
    msg.encoding = "rgb8"
    msg.step = w * 3
    rgb = np.zeros((h, w, 3), dtype=np.uint8)
    rgb[:, :, 0] = val
    rgb[:, :, 1] = val + 10
    rgb[:, :, 2] = val + 20
    rgb[0, 0] = (255, 0, 0)
    msg.data = rgb.tobytes()
    msg.header.stamp.sec = 3
    msg.header.stamp.nanosec = 7
    msg.header.frame_id = "d555_color_optical_frame"
    return msg, rgb


def _depth(h=48, w=64):
    msg = Image()
    msg.height = h
    msg.width = w
    msg.encoding = "16UC1"
    msg.step = w * 2
    depth = np.arange(h * w, dtype=np.uint16).reshape((h, w)) * 17
    msg.data = depth.tobytes()
    msg.header.stamp.sec = 3
    msg.header.stamp.nanosec = 7
    msg.header.frame_id = "d555_color_optical_frame"
    return msg, depth


class RgbdCodecTest(unittest.TestCase):
    def test_jpeg_keeps_stamp_and_size(self):
        raw, _ = _rgb8()
        jpg = color_to_jpeg(raw, quality=90)
        self.assertEqual(jpg.format, "jpeg")
        self.assertLess(len(jpg.data), len(raw.data))
        self.assertEqual(jpg.header.stamp.sec, 3)
        back = jpeg_to_color(jpg)
        self.assertEqual(back.encoding, "rgb8")
        self.assertEqual(back.height, 48)
        self.assertEqual(back.width, 64)
        self.assertEqual(back.header.stamp.nanosec, 7)

    def test_png_depth_roundtrip_exact(self):
        raw, depth = _depth()
        png = depth_to_png(raw)
        self.assertIn("png", png.format)
        self.assertLess(len(png.data), len(raw.data))
        back = png_to_depth(png)
        got = np.frombuffer(bytes(back.data), dtype=np.uint16).reshape(48, 64)
        np.testing.assert_array_equal(got, depth)
        self.assertEqual(back.header.frame_id, "d555_color_optical_frame")

    def test_canonicalize_driver_jpeg_and_png_labels(self):
        raw, _ = _rgb8()
        jpg = color_to_jpeg(raw, quality=80)
        jpg.format = "rgb8; jpeg compressed"
        out = canonicalize_color_compressed(jpg)
        self.assertEqual(out.format, COLOR_JPEG_FORMAT)
        back = jpeg_to_color(out)
        self.assertEqual(back.height, 48)
        png = depth_to_png(_depth()[0])
        png.format = "16UC1; png compressed"
        out_d = canonicalize_depth_compressed(png)
        self.assertEqual(out_d.format, DEPTH_PNG_FORMAT)

    def test_canonicalize_rejects_jpeg_depth(self):
        raw, _ = _rgb8()
        jpg = color_to_jpeg(raw)
        with self.assertRaises(ValueError):
            canonicalize_depth_compressed(jpg)
