"""Describe source bottom-leading/top-approach edge contact without admitting it.

V1 domain: horizontal rigid cuboids and a push aligned with their side faces.
All poses, points, normals and direction share container-absolute coordinates.
The two edge normal cones are geometric evidence only: they do not distinguish
a harmless traversal from a stalled push, or establish force/stability safety.
The unchanged support_contact predicate remains the sole current admission.
"""
import numpy as np
from support_contact import support_contact_failure


def _pose(value):
    result = np.asarray(value, dtype=float).reshape(4, 4)
    if (not np.isfinite(result).all() or
            not np.allclose(result[3], (0, 0, 0, 1), atol=1e-10, rtol=0) or
            not np.allclose(result[:3, :3].T @ result[:3, :3], np.eye(3), atol=1e-10, rtol=0) or
            not np.isclose(np.linalg.det(result[:3, :3]), 1., atol=1e-10, rtol=0)):
        raise ValueError('INVALID_EDGE_DIAGNOSTIC_POSE')
    return result


def _unit(value):
    vector = np.asarray(value, dtype=float)
    if (vector.shape != (3,) or not np.isfinite(vector).all() or
            not np.isclose(np.linalg.norm(vector), 1., atol=1e-8, rtol=0)):
        raise ValueError('INVALID_EDGE_DIAGNOSTIC_UNIT_VECTOR')
    return vector


def _cone_residual(normal, first, second):
    # The two rigid cuboid face normals are orthogonal. Projection onto their
    # positive cone gives sin(angle_to_cone) when the projection is nonzero.
    projection = max(0., float(normal @ first)) * first + max(0., float(normal @ second)) * second
    return float(np.linalg.norm(normal - projection))


def diagnose_support_edge(box_pose, box_size, support_pose, support_bounds, point,
                          normal, push_direction, *, point_tolerance_m, penetration_m,
                          normal_alignment_deg, depth_m, normal_toward_box):
    box = _pose(box_pose); support = _pose(support_pose)
    size = np.asarray(box_size, dtype=float); bounds = np.asarray(support_bounds, dtype=float)
    point = np.asarray(point, dtype=float); normal = _unit(normal); direction = _unit(push_direction)
    if (size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0) or
            bounds.shape != (2, 3) or not np.isfinite(bounds).all() or np.any(bounds[1] <= bounds[0]) or
            point.shape != (3,) or not np.isfinite(point).all() or not np.isfinite(depth_m) or
            not np.isfinite(point_tolerance_m) or point_tolerance_m <= 0 or
            not np.isfinite(penetration_m) or penetration_m <= 0 or
            not np.isfinite(normal_alignment_deg) or not 0 < normal_alignment_deg < 90):
        raise ValueError('INVALID_EDGE_DIAGNOSTIC_GEOMETRY_OR_TOLERANCE')
    bottom_sign = -1 if box[2, 2] >= 0 else 1
    box_up = -bottom_sign * box[:3, 2]; support_up = support[:3, 2]
    box_local = box[:3, :3].T @ (point - box[:3, 3])
    support_local = support[:3, :3].T @ (point - support[:3, 3])
    box_push = box[:3, :3].T @ direction
    support_push = support[:3, :3].T @ direction
    box_axis = int(np.argmax(np.abs(box_push[:2])))
    support_axis = int(np.argmax(np.abs(support_push[:2])))
    box_sign = 1 if box_push[box_axis] >= 0 else -1
    approach_sign = -1 if support_push[support_axis] >= 0 else 1
    box_front = box_sign * box[:3, box_axis]
    support_approach = approach_sign * support[:3, support_axis]
    box_edge_distances = (abs(box_local[2] - bottom_sign * size[2] / 2),
                          abs(box_local[box_axis] - box_sign * size[box_axis] / 2))
    support_edge_coordinate = bounds[0 if approach_sign < 0 else 1, support_axis]
    support_edge_distances = (abs(support_local[2] - bounds[1, 2]),
                              abs(support_local[support_axis] - support_edge_coordinate))
    sine = float(np.sin(np.deg2rad(normal_alignment_deg)))
    cosine = float(np.cos(np.deg2rad(normal_alignment_deg)))
    candidates = [normal] if normal_toward_box else [normal, -normal]
    residuals = [(max(_cone_residual(n, box_up, -box_front),
                      _cone_residual(n, support_up, support_approach)), n) for n in candidates]
    residual, oriented_normal = min(residuals, key=lambda row: row[0])
    checks = dict(horizontal_material_faces=min(box_up[2], support_up[2]) >= cosine,
        face_aligned_push=min(abs(box_push[box_axis]), abs(support_push[support_axis])) >= cosine,
        point_on_box_bottom_leading_edge=max(box_edge_distances) <= point_tolerance_m,
        point_within_box_material=np.all(np.abs(box_local) <= size / 2 + point_tolerance_m),
        point_on_support_top_approach_edge=max(support_edge_distances) <= point_tolerance_m,
        point_within_support_material=np.all(support_local >= bounds[0] - point_tolerance_m) and
            np.all(support_local <= bounds[1] + point_tolerance_m),
        normal_in_both_edge_cones=residual <= sine,
        penetration_within_limit=abs(depth_m) <= penetration_m)
    checks = {name: bool(value) for name, value in checks.items()}
    strict = support_contact_failure(box, size, support, bounds, point, normal,
        point_tolerance_m, penetration_m, normal_alignment_deg, depth_m,
        normal_toward_box=normal_toward_box)
    candidate = all(checks.values())
    classification = ('TOP_BOTTOM_SUPPORT' if strict is None else
        'GEOMETRIC_EDGE_TRANSITION_CANDIDATE' if candidate else 'OTHER_OR_UNSUPPORTED_CONTACT')
    return dict(classification=classification, diagnostic_only=True, execution_admitted=False,
        current_point_predicate_passed=strict is None, current_contract_failure=strict,
        edge_candidate=candidate, checks=checks,
        failed_checks=[name for name, passed in checks.items() if not passed],
        point_box_local=box_local.tolist(), point_support_local=support_local.tolist(),
        box_edge_distances_m=list(map(float, box_edge_distances)),
        support_edge_distances_m=list(map(float, support_edge_distances)),
        normal_cone_residual=residual, oriented_normal=oriented_normal.tolist(),
        depth_m=float(depth_m), scope='geometry_only_no_traversal_jam_force_or_stability_admission')
