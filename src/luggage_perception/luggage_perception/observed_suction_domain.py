"""Immutable directly observed top support, shared by perception and planning.

All positions/basis vectors refer to world. UV grid coordinates use the named
top-plane basis. A one authorizes the entire cell; zero never authorizes contact.
This record carries measurement, not inferred support or execution permission.
"""
from dataclasses import dataclass
import hashlib
import json
import math

import numpy as np


# Wire/resource bound (one MB of support), not a geometric filling threshold.
MAX_DOMAIN_CELLS = 1_000_000


def _finite(values, size, name):
    result = tuple(float(v) for v in values)
    if len(result) != size or not all(math.isfinite(v) for v in result):
        raise ValueError('invalid '+name)
    return result


@dataclass(frozen=True)
class ObservedSuctionDomain:
    box_id: str
    instance_id: str
    generation: int
    stamp_ns: int
    frame_id: str
    contact_model_version: int
    contact_model_hash: str
    box_xyz_world: tuple
    box_quaternion_xyzw: tuple
    box_wdh: tuple
    u_axis_world: tuple
    v_axis_world: tuple
    normal_world: tuple
    plane_offset_world_m: float
    origin_uv_top_m: tuple
    cell_size_m: float
    shape: tuple
    support: bytes

    def __post_init__(self):
        for name in ('box_id','instance_id','contact_model_hash'):
            if not isinstance(getattr(self,name),str) or not getattr(self,name).strip():
                raise ValueError('missing '+name)
        if self.frame_id != 'world':
            raise ValueError('world-frame observed domain required')
        for name,bits in (('generation',64),('stamp_ns',64),('contact_model_version',32)):
            value = getattr(self,name)
            if type(value) is not int or not 0 <= value < 2**bits:
                raise ValueError('invalid '+name)
        if self.contact_model_version == 0 or self.stamp_ns//10**9 > 2**31-1:
            raise ValueError('positive model version and representable ROS acquisition time required')
        for name,size in (('box_xyz_world',3),('box_quaternion_xyzw',4),('box_wdh',3),
                          ('u_axis_world',3),('v_axis_world',3),('normal_world',3),
                          ('origin_uv_top_m',2)):
            object.__setattr__(self,name,_finite(getattr(self,name),size,name))
        if min(self.box_wdh) <= 0 or abs(sum(v*v for v in self.box_quaternion_xyzw)-1.) > 1e-6:
            raise ValueError('measured box size and unit quaternion required')
        basis = np.asarray((self.u_axis_world,self.v_axis_world,self.normal_world))
        if (not np.allclose(basis@basis.T,np.eye(3),atol=1e-7,rtol=0)
                or np.linalg.det(basis) < 0 or self.normal_world[2] <= 0):
            raise ValueError('right-handed upward top-plane basis required')
        for name in ('plane_offset_world_m','cell_size_m'):
            value = float(getattr(self,name))
            if not math.isfinite(value):
                raise ValueError('finite '+name+' required')
            object.__setattr__(self,name,value)
        shape = tuple(self.shape)
        if (len(shape) != 2 or any(type(v) is not int or v <= 0 for v in shape)
                or math.prod(shape) > MAX_DOMAIN_CELLS or self.cell_size_m <= 0):
            raise ValueError('bounded positive observed grid required')
        support = bytes(self.support)
        if len(support) != math.prod(shape) or any(v not in (0,1) for v in support):
            raise ValueError('observed grid shape/support mismatch')
        object.__setattr__(self,'shape',shape)
        object.__setattr__(self,'support',support)

    def digest(self):
        metadata = {name:getattr(self,name) for name in self.__dataclass_fields__ if name != 'support'}
        encoded = json.dumps(metadata,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
        return hashlib.sha256(b'observed-suction-domain-v2\0'+encoded+b'\0'+self.support).hexdigest()

    def cells(self):
        """Read-only view of immutable support bytes, without an array copy."""
        return np.frombuffer(self.support,dtype=np.bool_).reshape(self.shape)

    def world_point(self, uv):
        u,v = _finite(uv,2,'top-plane point')
        return tuple(u*a+v*b-self.plane_offset_world_m*n
                     for a,b,n in zip(self.u_axis_world,self.v_axis_world,self.normal_world))
