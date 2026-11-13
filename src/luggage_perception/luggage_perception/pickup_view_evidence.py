"""Immutable measured camera inputs for pre-pick observation decisions.

This record cannot admit suction, fill missing depth or certify a view motion.
The region may be a detector bbox; its explicit source prevents consumers from
mistaking it for a measured instance contour. Depth values remain millimetres.
"""
from dataclasses import dataclass
import hashlib
import json
import math

import numpy as np


MAX_VIEW_BYTES = 8*1024*1024  # Transport resource limit, not sensor accuracy.
DEPTH_ENCODINGS = {'uint16':'16UC1', 'float32':'32FC1', 'float64':'64FC1'}


@dataclass(frozen=True)
class PickupViewAcquisition:
    stamp_ns: int
    optical_frame_id: str
    instance_id: str
    generation: int
    shape: tuple
    depth_dtype: str
    depth_mm: bytes
    instance_region: bytes
    region_source: str
    intrinsics_fx_fy_cx_cy: tuple
    world_from_optical: tuple

    def __post_init__(self):
        if (type(self.stamp_ns) is not int or not 0 <= self.stamp_ns < 2**63
                or self.stamp_ns//10**9 > 2**31-1
                or type(self.generation) is not int or not 0 <= self.generation < 2**64
                or not isinstance(self.optical_frame_id,str) or not self.optical_frame_id.strip()
                or not isinstance(self.instance_id,str) or not self.instance_id.strip()
                or self.region_source not in ('instance_mask','detection_bbox')):
            raise ValueError('PICKUP_VIEW_ACQUISITION_METADATA_INVALID')
        shape=tuple(self.shape)
        dtype=np.dtype(self.depth_dtype)
        if (len(shape)!=2 or any(type(n) is not int or n<1 for n in shape)
                or dtype.name not in DEPTH_ENCODINGS):
            raise ValueError('PICKUP_VIEW_IMAGE_LAYOUT_INVALID')
        depth,region=bytes(self.depth_mm),bytes(self.instance_region)
        pixels=math.prod(shape)
        if (pixels*(dtype.itemsize+1)>MAX_VIEW_BYTES
                or len(depth)!=pixels*dtype.itemsize or len(region)!=pixels
                or any(v not in (0,1) for v in region)):
            raise ValueError('PICKUP_VIEW_IMAGE_BYTES_INVALID_OR_OVER_BUDGET')
        intrinsics=tuple(float(v) for v in self.intrinsics_fx_fy_cx_cy)
        matrix=np.asarray(self.world_from_optical,dtype=float)
        if (len(intrinsics)!=4 or not all(math.isfinite(v) for v in intrinsics)
                or min(intrinsics[:2])<=0 or matrix.shape!=(16,) or not np.isfinite(matrix).all()):
            raise ValueError('PICKUP_VIEW_CAMERA_GEOMETRY_INVALID')
        matrix=matrix.reshape(4,4)
        if (not np.allclose(matrix[3],[0,0,0,1],atol=1e-9,rtol=0)
                or not np.allclose(matrix[:3,:3].T@matrix[:3,:3],np.eye(3),atol=1e-7,rtol=0)
                or np.linalg.det(matrix[:3,:3])<0):
            raise ValueError('PICKUP_VIEW_CAMERA_TRANSFORM_INVALID')
        for name,value in (('shape',shape),('depth_dtype',dtype.str),('depth_mm',depth),
                           ('instance_region',region),('intrinsics_fx_fy_cx_cy',intrinsics),
                           ('world_from_optical',tuple(float(v) for v in matrix.ravel()))):
            object.__setattr__(self,name,value)

    @classmethod
    def capture(cls, depth_mm, instance_region, intrinsics, world_from_optical, *,
                stamp_ns, optical_frame_id, instance_id, generation, region_source):
        depth,region=np.asarray(depth_mm),np.asarray(instance_region)
        if depth.ndim!=2 or region.shape!=depth.shape or region.dtype!=np.bool_:
            raise ValueError('PICKUP_VIEW_SAME_IMAGE_BOOLEAN_REGION_REQUIRED')
        # Bound before copying either image. Original NaN/zero values are kept.
        if depth.nbytes+region.size>MAX_VIEW_BYTES:
            raise ValueError('PICKUP_VIEW_IMAGE_BYTES_INVALID_OR_OVER_BUDGET')
        return cls(stamp_ns,optical_frame_id,instance_id,generation,tuple(depth.shape),
            depth.dtype.str,depth.tobytes(),region.astype(np.uint8,copy=False).tobytes(),region_source,
            tuple(float(getattr(intrinsics,name)) for name in ('fx','fy','cx','cy')),
            tuple(np.asarray(world_from_optical,dtype=float).ravel()))

    @property
    def size_bytes(self):
        return len(self.depth_mm)+len(self.instance_region)+1024

    def metadata(self):
        return {name:getattr(self,name) for name in self.__dataclass_fields__
                if name not in ('depth_mm','instance_region')}

    def bind(self, domain):
        if ((self.stamp_ns,self.instance_id,self.generation) !=
                (domain.stamp_ns,domain.instance_id,domain.generation)):
            raise ValueError('PICKUP_VIEW_DOMAIN_ACQUISITION_MISMATCH')
        return PickupViewEvidence(self,domain.digest(),domain.box_id)


@dataclass(frozen=True)
class PickupViewEvidence:
    acquisition: PickupViewAcquisition
    domain_sha256: str
    box_id: str

    def __post_init__(self):
        if (type(self.acquisition) is not PickupViewAcquisition
                or len(self.domain_sha256)!=64
                or any(c not in '0123456789abcdef' for c in self.domain_sha256)
                or not self.box_id.strip()):
            raise ValueError('PICKUP_VIEW_DOMAIN_BINDING_INVALID')

    @property
    def size_bytes(self):
        return self.acquisition.size_bytes

    def digest(self):
        a=self.acquisition
        metadata=dict(schema='pickup_view_evidence_v2',domain_sha256=self.domain_sha256,
                      box_id=self.box_id,acquisition=a.metadata())
        encoded=json.dumps(metadata,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
        return hashlib.sha256(b'pickup-view-evidence-v2\0'+encoded+b'\0'
            +a.depth_mm+b'\0'+a.instance_region).hexdigest()
