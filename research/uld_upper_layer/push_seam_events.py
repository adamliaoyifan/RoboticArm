"""Locate support-edge crossings of a fixed-orientation straight box push.

This is a topology/event report, not a collision or stability certificate.
All coordinates are container-absolute; progress is distance in metres. The
continuous overlap interval uses translating-polygon SAT, so a narrow seam is
not lost between configured sweep samples. SurfacePatch is horizontal and
rectangular by contract; arbitrary/tilted observed supports need another model.
"""
import numpy as np
from shapely.geometry import Polygon


def trace_push_seams(start_pose, final_pose, dimensions, patches):
    """Return each source patch's footprint overlap interval and edge events.

    Positive-area overlap is distinguished from grazing. Heights are reported
    even for patches outside nominal contact tolerance: a raised next patch
    must not disappear merely because it cannot qualify as current support.
    Consumers bind this report to snapshot/config/plan identity themselves.
    """
    start = np.asarray(start_pose, dtype=float).reshape(4, 4)
    final = np.asarray(final_pose, dtype=float).reshape(4, 4)
    size = np.asarray(dimensions, dtype=float)
    if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
        raise ValueError('INVALID_SEAM_BOX_SIZE')
    for pose in (start, final):
        if (not np.isfinite(pose).all() or
                not np.allclose(pose[3], (0, 0, 0, 1), atol=1e-10, rtol=0) or
                not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-10, rtol=0) or
                not np.isclose(np.linalg.det(pose[:3, :3]), 1., atol=1e-10, rtol=0)):
            raise ValueError('INVALID_SEAM_POSE')
    if (not np.allclose(start[:3, :3], final[:3, :3], atol=1e-10, rtol=0) or
            abs(abs(start[2, 2]) - 1.) > 1e-10 or
            abs(final[2, 3] - start[2, 3]) > 1e-10):
        raise ValueError('SEAM_MODEL_REQUIRES_HORIZONTAL_FIXED_ORIENTATION_PUSH')
    delta = final[:2, 3] - start[:2, 3]
    length = float(np.linalg.norm(delta))
    if length <= 0:
        raise ValueError('EMPTY_SEAM_PUSH')
    direction = delta / length
    local = np.array([[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]]) * size[:2] / 2
    footprint = local @ start[:2, :2].T + start[:2, 3]
    axes = [np.array([1., 0.]), np.array([0., 1.]),
            start[:2, 0], start[:2, 1]]
    rows, events, ids = [], [], set()
    for patch in patches:
        if patch.source_id in ids:
            raise ValueError('DUPLICATE_SEAM_SOURCE_ID')
        ids.add(patch.source_id)
        bounds = np.asarray(patch.bounds_xy, dtype=float)
        if (bounds.shape != (4,) or not np.isfinite(bounds).all() or
                np.any(bounds[2:] <= bounds[:2]) or not np.isfinite(patch.surface_z_container)):
            raise ValueError('INVALID_SEAM_PATCH')
        x0, y0, x1, y1 = bounds
        rectangle = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
        enter, leave = -float('inf'), float('inf')
        enter_axis = leave_axis = None
        overlaps = True
        for axis in axes:
            moving = footprint @ axis
            fixed = rectangle @ axis
            speed = float(direction @ axis)
            if abs(speed) <= np.finfo(float).eps * 8:
                if moving.max() <= fixed.min() or fixed.max() <= moving.min():
                    overlaps = False
                    break
                continue
            a = float((fixed.min() - moving.max()) / speed)
            b = float((fixed.max() - moving.min()) / speed)
            low, high = min(a, b), max(a, b)
            if low > enter:
                enter, enter_axis = low, axis.tolist()
            if high < leave:
                leave, leave_axis = high, axis.tolist()
        if not overlaps or max(enter, 0.) >= min(leave, length):
            continue
        gap = float(start[2, 3] - size[2] / 2 - patch.surface_z_container)
        row = dict(source_id=patch.source_id, bounds_xy=bounds.tolist(),
                   surface_z_container=float(patch.surface_z_container),
                   box_bottom_z_container=float(start[2, 3] - size[2] / 2),
                   nominal_bottom_minus_top_m=gap,
                   overlap_start_m=max(0., enter), overlap_end_m=min(length, leave))
        rows.append(row)
        for kind, progress, axis in (('enter', enter, enter_axis), ('leave', leave, leave_axis)):
            if 0. <= progress <= length:
                at_event = footprint + progress * direction
                events.append(dict(kind=kind, progress_m=float(progress),
                    separating_axis_xy=axis, box_footprint_wkt=Polygon(at_event).wkt, **row))
    return dict(model_id='horizontal_fixed_orientation_translating_polygon_sat_v1',
                scope='source_geometry_edge_events_only_not_contact_or_robust_tube_certificate',
                length_m=length, direction_xy=direction.tolist(),
                patches=sorted(rows, key=lambda row: row['source_id']),
                events=sorted(events, key=lambda row: (row['progress_m'], row['source_id'], row['kind'])))
