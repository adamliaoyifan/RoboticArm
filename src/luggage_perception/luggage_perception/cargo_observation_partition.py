"""Exact-acquisition point/pixel partitions, without physical cargo identity.

Frame-local segmentation labels are evidence, never ownership or free space.
There is no hole filling, nearest-stamp matching, box prior, or ROS dependency.
"""
from dataclasses import dataclass
import hashlib
import json
import math

import numpy as np


@dataclass(frozen=True)
class AcquisitionKey:
    stamp_ns: int
    optical_frame: str

    def __post_init__(self):
        if (type(self.stamp_ns) is not int or not 0 < self.stamp_ns < 2**63
                or not isinstance(self.optical_frame, str) or not self.optical_frame.strip()):
            raise ValueError('INVALID_ACQUISITION_KEY')


@dataclass(frozen=True)
class CargoObservationPartition:
    """Immutable little-endian Float32 XYZ and Uint32 pixel/label buffers."""
    key: AcquisitionKey
    sensor_id: str
    observation_id: str
    shape_hw: tuple
    stride: int
    xyz_optical_f32: bytes
    source_pixels_u32: bytes
    instance_labels_u32: bytes
    container_from_optical_f64: bytes
    source_sha256: tuple
    binding_sha256: str

    def points(self):
        return np.frombuffer(self.xyz_optical_f32, dtype='<f4').reshape(-1, 3)

    def pixels(self):
        return np.frombuffer(self.source_pixels_u32, dtype='<u4')

    def labels(self):
        return np.frombuffer(self.instance_labels_u32, dtype='<u4')


def _positive_int(value, reason):
    if type(value) is not int or value <= 0:
        raise ValueError(reason)


def partition_aligned_observation(*, sensor_id, observation_id, keys,
        rgb, depth_m, instance_mask, intrinsics, distortion,
        container_from_optical, stride, max_pixels, source_sha256):
    """Deproject the actual aligned depth once and retain every sampled return.

    ``keys`` are RGB/depth/mask/calibration/TF acquisition keys. ``depth_m``
    has explicit metre units; the ROS boundary converts the declared encoding.
    Hashes bind the five complete serialized source messages, including raw
    depth encoding and calibration. Numerical matrix tolerances only check
    algebraic validity; they are not geometric association tolerances.
    """
    if (len(keys) != 5 or any(type(k) is not AcquisitionKey for k in keys)
            or any(k != keys[0] for k in keys)):
        raise ValueError('EXACT_ACQUISITION_REQUIRED')
    if any(not isinstance(s, str) or not s.strip() for s in (sensor_id, observation_id)):
        raise ValueError('SENSOR_OBSERVATION_ID_REQUIRED')
    _positive_int(stride, 'POSITIVE_STRIDE_REQUIRED')
    _positive_int(max_pixels, 'POSITIVE_RESOURCE_BOUND_REQUIRED')
    hashes = tuple(source_sha256)
    if (len(hashes) != 5 or any(not isinstance(h, str) or len(h) != 64
            or any(c not in '0123456789abcdef' for c in h) for h in hashes)):
        raise ValueError('COMPLETE_SOURCE_HASHES_REQUIRED')
    depth, colour, mask = map(np.asarray, (depth_m, rgb, instance_mask))
    if (depth.ndim != 2 or not depth.size or depth.size > max_pixels
            or depth.size >= 2**32 or depth.dtype.kind != 'f'):
        raise ValueError('BOUNDED_METRE_DEPTH_REQUIRED')
    if (colour.shape != (*depth.shape, 3) or colour.dtype != np.uint8
            or mask.shape != depth.shape or mask.dtype.kind not in 'ui'
            or np.any(mask < 0) or np.any(mask > 2**32-1)):
        raise ValueError('ALIGNED_IMAGE_MASK_REQUIRED')
    k = np.asarray(intrinsics, dtype=np.float64)
    d = np.asarray(distortion, dtype=np.float64)
    transform = np.asarray(container_from_optical, dtype=np.float64)
    if (k.shape != (3, 3) or not np.all(np.isfinite(k))
            or k[0,0] <= 0 or k[1,1] <= 0
            or not np.array_equal(k[[0,1,2,2,2],[1,0,0,1,2]], [0,0,0,0,1])
            or d.ndim != 1 or not np.all(np.isfinite(d)) or np.any(d != 0)):
        raise ValueError('RECTIFIED_PINHOLE_REQUIRED')
    if (transform.shape != (4,4) or not np.all(np.isfinite(transform))
            or not np.array_equal(transform[3], [0,0,0,1])
            or not np.allclose(transform[:3,:3].T @ transform[:3,:3], np.eye(3),
                               atol=1e-7, rtol=0)
            or not math.isclose(np.linalg.det(transform[:3,:3]), 1., abs_tol=1e-7)):
        raise ValueError('RIGID_ACQUISITION_TF_REQUIRED')
    vv, uu = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[vv,uu].astype(np.float32)
    valid = np.isfinite(z) & (z > 0)
    if not np.any(valid):
        raise ValueError('NO_VALID_DEPTH')
    u, v, z = uu[valid], vv[valid], z[valid]
    x = (u.astype(np.float32)-np.float32(k[0,2]))*z/np.float32(k[0,0])
    y = (v.astype(np.float32)-np.float32(k[1,2]))*z/np.float32(k[1,1])
    points = np.column_stack((x,y,z)).astype('<f4')
    if not np.all(np.isfinite(points)):
        raise ValueError('NONFINITE_DEPROJECTION')
    pixel_bytes = np.asarray(v*depth.shape[1]+u, dtype='<u4').tobytes()
    label_bytes = np.asarray(mask[v,u], dtype='<u4').tobytes()
    point_bytes = points.tobytes()
    transform_bytes = np.asarray(transform,dtype='<f8').tobytes()
    metadata = dict(sensor_id=sensor_id, observation_id=observation_id,
        stamp_ns=keys[0].stamp_ns, optical_frame=keys[0].optical_frame,
        shape_hw=depth.shape, stride=stride, source_sha256=hashes)
    digest = hashlib.sha256(b'cargo-observation-partition-v2\0')
    digest.update(json.dumps(metadata,sort_keys=True,separators=(',',':')).encode())
    # Hash actual values too: supplied source hashes cannot hide array mutation.
    for array in (colour, depth, mask, k):
        digest.update(array.dtype.str.encode()+b'\0')
        digest.update(np.ascontiguousarray(array).tobytes())
    for payload in (point_bytes,pixel_bytes,label_bytes,transform_bytes):
        digest.update(len(payload).to_bytes(8,'little'))
        digest.update(payload)
    return CargoObservationPartition(keys[0],sensor_id,observation_id,
        tuple(depth.shape),stride,point_bytes,pixel_bytes,label_bytes,
        transform_bytes,hashes,digest.hexdigest())
