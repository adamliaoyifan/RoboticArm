"""Transport codec tests derived from origin/ros2_humble d56b914."""

import numpy as np
import pytest

pytest.importorskip("sensor_msgs")
cv2 = pytest.importorskip("cv2")

from sensor_msgs.msg import Image  # noqa: E402

from luggage_perception import rgbd_codec  # noqa: E402


def _image(array, encoding):
    msg = Image()
    msg.header.frame_id = "d555_color_optical_frame"
    msg.header.stamp.sec = 7
    msg.header.stamp.nanosec = 11
    msg.height = int(array.shape[0])
    msg.width = int(array.shape[1])
    msg.encoding = encoding
    msg.step = int(array.strides[0])
    msg.data = array.tobytes()
    return msg


def test_color_is_jpeg_and_preserves_header():
    rgb = np.zeros((12, 16, 3), dtype=np.uint8)
    rgb[:, :, 1] = 180
    out = rgbd_codec.color_to_jpeg(_image(rgb, "rgb8"), quality=80)
    assert out.format == "jpeg"
    assert out.header.frame_id == "d555_color_optical_frame"
    assert out.header.stamp.sec == 7
    decoded = cv2.imdecode(np.frombuffer(out.data, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == rgb.shape


def test_depth_png_round_trip_is_bit_exact():
    depth = np.array([[0, 1, 500], [65000, 42, 9999]], dtype=np.uint16)
    out = rgbd_codec.depth_to_png(_image(depth, "16UC1"))
    assert out.format == "16UC1; png"
    decoded = cv2.imdecode(
        np.frombuffer(out.data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    np.testing.assert_array_equal(decoded, depth)


def test_depth_rejects_lossy_or_wrong_source_encoding():
    with pytest.raises(ValueError):
        rgbd_codec.depth_to_png(_image(
            np.zeros((2, 2), dtype=np.uint8), "mono8"))
