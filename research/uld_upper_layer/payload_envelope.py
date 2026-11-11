"""Directional enclosure of a free rigid box with bounded pose error.

This proves separation only. Projection overlap means unproved, not collision.
An SO(3) angle ball is applied about the geometric box centre; translations
form either a Euclidean ball or its intersection with a plane perpendicular
to a supplied travel direction. Neither model describes contact mechanics.
"""
import numpy as np


def rotation_projection_range(vectors, axis, angle_rad):
    """Enclose n dot R v for every rotation with geodesic angle <= angle_rad."""
    vectors = np.asarray(vectors, dtype=float)
    axis = np.asarray(axis, dtype=float)
    if (vectors.ndim != 2 or vectors.shape[1] != 3 or axis.shape != (3,) or
            not np.isfinite(vectors).all() or not np.isfinite(axis).all() or
            not np.isfinite(angle_rad) or not 0. <= angle_rad <= np.pi or
            not np.isclose(np.linalg.norm(axis), 1., atol=1e-12, rtol=0)):
        raise ValueError('INVALID_ROTATION_PROJECTION_INPUT')
    radius = np.linalg.norm(vectors, axis=1)
    axial = vectors @ axis
    transverse = np.sqrt(np.maximum(0., radius * radius - axial * axial))
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    # A rotated vector traces a spherical cap. Once that cap reaches +/-n,
    # the extremum is +/-|v| rather than the cap boundary value.
    upper = np.where(axial >= radius * c, radius, axial * c + transverse * s)
    lower = np.where(-axial >= radius * c, -radius, axial * c - transverse * s)
    return lower, upper


def box_projection_range(box_pose, dimensions, axis, translation_m, angle_rad,
                         transverse_to=None):
    """Projection interval for the entire uncertain box on one unit world axis."""
    pose = np.asarray(box_pose, dtype=float).reshape(4, 4)
    size = np.asarray(dimensions, dtype=float)
    if (size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0) or
            not np.isfinite(pose).all() or not np.isfinite(translation_m) or translation_m < 0 or
            not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-12, rtol=0) or
            not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-10, rtol=0) or
            not np.isclose(np.linalg.det(pose[:3, :3]), 1., atol=1e-10, rtol=0)):
        raise ValueError('INVALID_BOX_ENVELOPE_INPUT')
    vertices = np.array([[i, j, k] for i in (-1., 1.) for j in (-1., 1.)
                         for k in (-1., 1.)]) * size / 2
    axis = np.asarray(axis, dtype=float)
    lower, upper = rotation_projection_range(vertices @ pose[:3, :3].T, axis, angle_rad)
    translation_projection = translation_m
    if transverse_to is not None:
        direction = np.asarray(transverse_to, dtype=float)
        if (direction.shape != (3,) or not np.isfinite(direction).all() or
                not np.isclose(np.linalg.norm(direction), 1., atol=1e-12, rtol=0)):
            raise ValueError('INVALID_TRANSVERSE_DIRECTION')
        translation_projection *= np.sqrt(max(0., 1. - float(axis @ direction) ** 2))
    centre = float(pose[:3, 3] @ axis)
    return (centre + float(lower.min()) - translation_projection,
            centre + float(upper.max()) + translation_projection)


def separation_lower_bound(box_pose, dimensions, obstacle_vertices, axes,
                           translation_m, angle_rad, transverse_to=None):
    """Lower distance bound from separating projections of the source geometry.

    Any finite nonempty unit-axis set is conservative. More axes can tighten
    the bound but cannot turn a negative result into proof of collision.
    The obstacle vertices must already be in the same frame as box_pose.
    """
    vertices = np.asarray(obstacle_vertices, dtype=float)
    axes = np.asarray(axes, dtype=float)
    if (vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) == 0 or
            not np.isfinite(vertices).all() or axes.ndim != 2 or axes.shape[1] != 3 or
            len(axes) == 0 or not np.isfinite(axes).all()):
        raise ValueError('INVALID_ENVELOPE_OBSTACLE_OR_AXES')
    best = dict(lower_m=0., axis=None, box_projection=None, obstacle_projection=None)
    for axis in axes:
        norm = float(np.linalg.norm(axis))
        if norm <= 0:
            raise ValueError('ZERO_ENVELOPE_AXIS')
        axis = axis / norm
        low, high = box_projection_range(box_pose, dimensions, axis, translation_m,
                                         angle_rad, transverse_to)
        projection = vertices @ axis
        other_low, other_high = float(projection.min()), float(projection.max())
        gap = max(other_low - high, low - other_high, 0.)
        if gap > best['lower_m']:
            best = dict(lower_m=float(gap), axis=axis.tolist(), box_projection=[low, high],
                        obstacle_projection=[other_low, other_high])
    return best
