"""Convert Livox CustomMsg (compact) to PointCloud2 (XYZRTLT, packed)."""

from __future__ import annotations

import struct
from typing import Iterable

from sensor_msgs.msg import PointCloud2, PointField

# Matches livox_ros_driver2 LivoxPointXyzrtlt (#pragma pack(1)).
_POINT_STEP = 26  # 3*f32 + f32 + u8 + u8 + f64
_PACK = struct.Struct("<ffffBBd")


def custom_points_to_cloud(header, points: Iterable, _timebase: int = 0) -> PointCloud2:
    """Build the same PointCloud2 layout the Livox driver uses for xfer_format=0.

    ``point.timestamp`` is the per-point offset_time as float64 nanoseconds,
    same as the driver (not timebase + offset).
    """
    blob = bytearray()
    n = 0
    for p in points:
        blob.extend(
            _PACK.pack(
                float(p.x),
                float(p.y),
                float(p.z),
                float(p.reflectivity),
                int(p.tag) & 0xFF,
                int(p.line) & 0xFF,
                float(p.offset_time),
            )
        )
        n += 1
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.is_bigendian = False
    msg.is_dense = True
    msg.point_step = _POINT_STEP
    msg.row_step = _POINT_STEP * n
    msg.data = bytes(blob)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="tag", offset=16, datatype=PointField.UINT8, count=1),
        PointField(name="line", offset=17, datatype=PointField.UINT8, count=1),
        PointField(name="timestamp", offset=18, datatype=PointField.FLOAT64, count=1),
    ]
    return msg
