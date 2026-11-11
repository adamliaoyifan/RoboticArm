"""Necessary whole-footprint gate for an actual suction_contact_frame pose.

This module projects the physical contact rectangle, including the existing
model margin, into the measurement-bound top domain. It never moves the pose,
fits a new normal, fills missing cells or issues a suction-quality permission.
Normal/contact-height quality and collisions remain separate required gates.
"""
import numpy as np

from luggage_perception.suction_footprint_containment import (
    ContainmentResult, check_convex_footprint_containment,
)


def project_contact_footprint(domain, contact_model, world_from_contact_frame, *,
                              include_boundary_margin=True):
    """Project physical contact corners; return None for a rank-zero outline.

    Use suction_contact_frame, not a TCP with release gap or a tool mounting
    frame. Both the pose and the domain refer to world. The model must be the
    same physical contact model that produced this observation.
    """
    if (contact_model.identity_hash != domain.contact_model_hash
            or contact_model.model_version != domain.contact_model_version
            or contact_model.contact_frame != 'suction_contact_frame'
            or contact_model.footprint_type != 'rectangle'):
        raise ValueError('matching rectangular suction contact model required')
    transform = np.asarray(world_from_contact_frame, dtype=float)
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0., 0., 0., 1.], atol=1e-9, rtol=0)
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3],
                               np.eye(3), atol=1e-7, rtol=0)
            or np.linalg.det(transform[:3, :3]) < 0):
        raise ValueError('finite proper world-from-contact rigid transform required')
    half = np.asarray(contact_model.footprint_size_xy_m, dtype=float)/2
    margin = float(contact_model.boundary_margin_m)
    if (half.shape != (2,) or not np.isfinite(half).all() or np.any(half <= 0)
            or not np.isfinite(margin) or margin < 0):
        raise ValueError('positive physical footprint and nonnegative margin required')
    corners = np.array([[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]]) * (
        half + (margin if include_boundary_margin else 0.))
    world = (corners @ transform[:3, :2].T) + transform[:3, 3]
    basis = np.asarray((domain.u_axis_world, domain.v_axis_world))
    # Numerical rank guard only. Physical normal alignment must be checked by
    # the separate quality gate; projection alone cannot authorize contact.
    if abs(np.linalg.det(basis @ transform[:3, :2])) <= 1e-12:
        return None
    return world @ basis.T


def check_observed_contact_footprint(domain, contact_model, world_from_contact_frame):
    """Necessary support gate; contact/surface quality remains separate."""
    vertices_uv = project_contact_footprint(domain, contact_model, world_from_contact_frame)
    if vertices_uv is None:
        return ContainmentResult(False, 'FOOTPRINT_PROJECTION_DEGENERATE', 0, 0)
    return check_convex_footprint_containment(domain.cells(),
        origin_uv=domain.origin_uv_top_m, cell_size_m=domain.cell_size_m,
        vertices_uv=vertices_uv)
