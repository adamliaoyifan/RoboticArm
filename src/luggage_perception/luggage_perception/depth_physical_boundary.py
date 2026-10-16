"""Identity-bound physical boundary admission, not yet wired to the map owner.

Only a first positive intersection from the authoritative static collision mesh
may supply BoundaryHit. Mesh rays alone are never observations: a measured
return, matching ray and matching scene/model identities are also required.
"""
from dataclasses import dataclass
import math

from luggage_perception.depth_view_rays import _admit_depth_ray


@dataclass(frozen=True)
class BoundaryHit:
    origin: tuple
    direction: tuple
    distance_m: float
    scene_revision: str
    mesh_sha256: str


def admit_physical_depth_ray(hull, origin, measured_end, terminal_band_m, hit, *,
                             scene_revision, mesh_sha256):
    """Refuse missing/mismatched mesh evidence; never extrapolate measured rays."""
    if hit is None:
        return None
    if (not scene_revision or not mesh_sha256 or hit.scene_revision != scene_revision
            or hit.mesh_sha256 != mesh_sha256):
        raise ValueError('physical boundary scene/model identity mismatch')
    origin = tuple(float(v) for v in origin)
    end = tuple(float(v) for v in measured_end)
    if len(origin) != 3 or len(end) != 3 or not all(map(math.isfinite, origin+end)):
        return None
    if len(hit.origin) != 3 or len(hit.direction) != 3:
        raise ValueError('physical boundary ray shape mismatch')
    values = (*hit.origin,*hit.direction,hit.distance_m)
    if not all(map(math.isfinite,values)) or hit.distance_m <= 0:
        raise ValueError('invalid physical boundary intersection')
    direction = tuple(b-a for a,b in zip(origin,end))
    length = math.sqrt(sum(v*v for v in direction))
    if length <= 0:
        return None
    direction = tuple(v/length for v in direction)
    if (any(abs(a-b)>1e-9 for a,b in zip(origin,hit.origin)) or
            any(abs(a-b)>1e-9 for a,b in zip(direction,hit.direction))):
        raise ValueError('physical boundary belongs to a different ray')
    return _admit_depth_ray(hull,origin,end,terminal_band_m,hit.distance_m)
