"""Contact area and gravity-only support polygon, for rigid coplanar boxes."""
import numpy as np
from shapely.geometry import Polygon, Point, box
from shapely.ops import unary_union
from planning_types import SupportMetrics


def measure_support(box_pose, dimensions, patches, config, com_local=(0., 0., 0.)):
    transform = np.asarray(box_pose, dtype=float).reshape(4, 4)
    size = np.asarray(dimensions)
    # V1 accepts horizontal boxes, including a 180-degree inversion.
    if abs(abs(transform[2, 2]) - 1.) > 1e-7:
        raise ValueError('NON_COPLANAR_BOX_UNSUPPORTED')
    offsets = np.array([[-1,-1], [1,-1], [1,1], [-1,1]]) * size[:2]/2
    corners = offsets @ transform[:2,:2].T + transform[:2,3]
    footprint = Polygon(corners)
    bottom = transform[2,3] - size[2]/2
    contacts, names = [], []
    for patch in patches:
        if abs(patch.surface_z_container - bottom) <= config.contact_tolerance_m:
            intersection = footprint.intersection(box(*patch.bounds_xy))
            if intersection.area > 0:
                contacts.append(intersection); names.append(patch.source_id)
    contact = unary_union(contacts)
    hull = contact.convex_hull
    ratio = contact.area/footprint.area
    com = (transform @ np.r_[com_local, 1.])[:2]
    point = Point(com)
    if hull.is_empty:
        margin = -float(np.linalg.norm(size[:2]))
    else:
        margin = hull.boundary.distance(point) if hull.covers(point) else -hull.distance(point)
    reason = ('SUPPORT_AREA' if ratio + 1e-10 < config.support_area_ratio_min else
              'COM_SUPPORT_MARGIN' if margin + 1e-10 < config.support_com_margin_m else 'SUPPORTED')
    return SupportMetrics(contact.area, ratio, contact.wkt, hull.wkt, margin,
                          tuple(sorted(names)), reason == 'SUPPORTED', reason)
