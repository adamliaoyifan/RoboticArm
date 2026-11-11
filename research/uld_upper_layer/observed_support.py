"""Near-contact geometric overlap and anti-tip hull from measured rigid poses.

This is not a pressure distribution or a claim that every point carries load.
Native discrete contacts and their hull are reported separately. Faces are
clipped in 3-D before projection, preserving small observed tilts and gaps.
"""
from dataclasses import dataclass
import numpy as np
from shapely.geometry import Polygon,Point,MultiPoint
from shapely.ops import unary_union


@dataclass(frozen=True)
class ObservedSupport:
    near_contact_area_m2: float
    projected_footprint_area_m2: float
    near_contact_area_ratio: float
    near_contact_wkt: str
    anti_tip_hull_wkt: str
    com_margin_m: float
    lower_body_ids: tuple
    native_lower_body_ids: tuple
    native_contact_points_container: tuple
    native_contact_hull_wkt: str
    native_max_depth_m: float
    pressure_area_status: str = 'unavailable_without_contact_pressure_model'


def clip_halfspace(points,axis,bound,keep_less):
    """Sutherland-Hodgman clipping of a convex 3-D face."""
    if not points:return []
    result=[];previous=np.asarray(points[-1]);last=(previous[axis]-bound)*(1 if keep_less else -1)
    for current in points:
        current=np.asarray(current);value=(current[axis]-bound)*(1 if keep_less else -1)
        if (value<=0)!=(last<=0):
            result.append(previous+(current-previous)*last/(last-value))
        if value<=0:result.append(current)
        previous=current;last=value
    return result


def measure_observed_support(sample,dimensions,com_local,gap_m,penetration_m):
    if min(gap_m,penetration_m)<0:raise ValueError('INVALID_SUPPORT_GAP_LIMIT')
    box=np.asarray(sample.container_T_box).reshape(4,4);half=np.asarray(dimensions)/2
    bottom_sign=-1 if box[2,2]>=0 else 1
    face=np.array([[u*half[0],v*half[1],bottom_sign*half[2]] for u,v in ((-1,-1),(1,-1),(1,1),(-1,1))])
    face=face@box[:3,:3].T+box[:3,3]
    footprint=Polygon(face[:,:2])
    if footprint.area<=0:raise ValueError('SUPPORT_FOOTPRINT_DEGENERATE')
    pieces=[];ids=[]
    for lower in sample.lower:
        transform=np.asarray(lower.container_T_body).reshape(4,4);inverse=np.linalg.inv(transform)
        points=list(face@inverse[:3,:3].T+inverse[:3,3]);h=np.array(lower.dimensions)/2
        # The source aligned lower fixture uses its material +Z as top. A
        # flipped body cannot be silently relabelled as an available support.
        if transform[2,2]<=0:continue
        for axis,low,high in ((0,-h[0],h[0]),(1,-h[1],h[1]),(2,h[2]-penetration_m,h[2]+gap_m)):
            points=clip_halfspace(points,axis,low,False)
            points=clip_halfspace(points,axis,high,True)
        if len(points)<3:continue
        points=np.asarray(points)@transform[:3,:3].T+transform[:3,3]
        polygon=Polygon(points[:,:2])
        if polygon.area>0:pieces.append(polygon);ids.append(lower.name)
    overlap=unary_union(pieces);hull=overlap.convex_hull
    com=box@np.r_[com_local,1.];point=Point(com[:2])
    margin=(-float(np.linalg.norm(dimensions)) if hull.is_empty else
            hull.boundary.distance(point) if hull.covers(point) else -hull.distance(point))
    names=tuple(body.name for body in sample.lower)
    native=tuple(c.point_container for c in sample.contacts if any(c.other_collision.startswith(name+'::') for name in names)
                 and c.normal_toward_payload_container[2]>0)
    native_hull=MultiPoint([p[:2] for p in native]).convex_hull
    depths=[c.depth_m for c in sample.contacts if any(c.other_collision.startswith(name+'::') for name in names)]
    native_ids=tuple(sorted(name for name in names if any(c.other_collision.startswith(name+'::') for c in sample.contacts)))
    return ObservedSupport(overlap.area,footprint.area,overlap.area/footprint.area,overlap.wkt,hull.wkt,
        margin,tuple(sorted(ids)),native_ids,native,native_hull.wkt,max(depths,default=0.))
