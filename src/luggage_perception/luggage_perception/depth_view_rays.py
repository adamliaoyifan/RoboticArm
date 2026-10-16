"""Conservative depth rays in the authoritative container coordinate frame.

Only rays entering through the -X aperture may observe the interior. Their
terminal band is left unobserved unless the measured return is clearly inside
all closed faces, in which case it is an obstacle hit. The mapper uses one voxel diagonal as its terminal band;
it is not permission to clear a near-wall obstacle. Missing returns make no ray.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DepthRay:
    free_end: tuple
    obstacle: tuple | None


def closed_planes(hull):
    planes=[((1.,0.,0.),hull.half_x),((0.,-1.,0.),hull.half_y),
            ((0.,1.,0.),hull.half_y),((0.,0.,-1.),-hull.floor_z),
            ((0.,0.,1.),hull.ceiling_z)]
    if hull.chamfer:
        c=hull.chamfer
        slope=(c.wall_y-c.floor_y)/(c.wall_z-hull.floor_z)
        planes.append(((0.,1.,-slope),c.floor_y-slope*hull.floor_z))
    return planes


def admit_depth_ray(hull, origin, measured_end, terminal_band_m):
    """Return admitted free-ray prefix and optional hit, or None.

    All coordinates are container-frame metres. Reject returns beyond the first
    model wall by more than the terminal band, and entry through a closed face.
    Never extrapolate a short return to the wall behind it.
    """
    return _admit_depth_ray(hull, origin, measured_end, terminal_band_m)


def _admit_depth_ray(hull, origin, measured_end, terminal_band_m, physical_hit_distance_m=None):
    """Shared clipping kernel; physical distance is validated by its boundary adapter."""
    origin=tuple(float(v) for v in origin);end=tuple(float(v) for v in measured_end)
    if len(origin)!=3 or len(end)!=3 or not all(map(math.isfinite,origin+end)):
        return None
    band=float(terminal_band_m)
    if not math.isfinite(band) or band<=0:
        raise ValueError('positive finite terminal band required')
    direction=tuple(b-a for a,b in zip(origin,end));length=math.sqrt(sum(v*v for v in direction))
    if length<=band:return None
    planes=closed_planes(hull)
    all_planes=[((-1.,0.,0.),hull.half_x),*planes]
    near,far=0.,float('inf');entry=None
    dot=lambda a,b:sum(x*y for x,y in zip(a,b))
    for index,(normal,bound) in enumerate(all_planes):
        numerator=bound-dot(normal,origin);denominator=dot(normal,direction)
        if abs(denominator)<1e-12:
            if numerator < -1e-9:return None
            continue
        crossing=numerator/denominator
        if denominator<0 and crossing>near:
            near=crossing;entry=index
        elif denominator>0:
            far=min(far,crossing)
    if near>far or far<=0 or near>=1 or (entry is not None and entry!=0):return None
    if physical_hit_distance_m is None:
        if length*(1.-far)>band:return None
    else:
        # The first physical surface must not precede usable-volume entry.
        # A measured return behind that surface cannot clear through it.
        if physical_hit_distance_m <= near*length or length-physical_hit_distance_m > band:
            return None
    interior=all((bound-dot(normal,end))/math.sqrt(dot(normal,normal))>band
                 for normal,bound in planes)
    obstacle=end if interior and end[0]>=-hull.half_x else None
    if physical_hit_distance_m is None:
        stop=min(1.,far)-band/length
    else:
        # Truncate at the nearer measured/physical surface BEFORE clipping to
        # usable volume. The inset semantic boundary is not a measured wall.
        stop=min((min(length,physical_hit_distance_m)-band)/length,far)
    if stop<=near:
        # A return just inside the open door is still an obstacle, even
        # when no full voxel before it is justified as free.
        return DepthRay(origin,obstacle) if obstacle is not None else None
    free_end=tuple(a+stop*d for a,d in zip(origin,direction))
    return DepthRay(free_end,obstacle)


def integrate_depth_acquisition(mapper, points, origin):
    """Owner callback result with the existing settled-view result vocabulary."""
    import time
    import numpy as np
    started=time.monotonic()
    points=np.asarray(points,dtype=float).reshape((-1,3))
    finite=points[np.isfinite(points).all(axis=1)]
    before=int(mapper.stats()['map_revision'])
    detail=mapper.integrate_depth_view(finite,origin)
    after=int(mapper.stats()['map_revision'])
    return dict(ok=after>before,reason_code='' if after>before else 'VIEW_EMPTY',
                n_finite=len(finite),n_kept=detail['rays'],
                latency_sec=time.monotonic()-started,revision_before=before,revision_after=after,
                observed_points=np.asarray(detail['observed_points'],dtype=float).reshape((-1,3)),
                depth_detail={key:value for key,value in detail.items() if key!='observed_points'})
