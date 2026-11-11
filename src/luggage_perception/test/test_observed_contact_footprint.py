"""Directed contact pose geometry: unknown, rotation, tilt and frame binding."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from luggage_perception.observed_suction_domain import ObservedSuctionDomain
from luggage_perception.observed_contact_footprint import check_observed_contact_footprint
from luggage_perception.suction_footprint_containment import (
    check_convex_footprint_containment, check_footprint_containment,
)


def domain_and_model():
    domain = ObservedSuctionDomain('box', 'instance', 3, 12_000_000_000, 'world', 1,
        'physical-model', (0., 0., .85), (0., 0., 0., 1.), (.7, .45, .3),
        (1., 0., 0.), (0., 1., 0.), (0., 0., 1.), -1., (-.35, -.225),
        .005, (140, 90), bytes([1])*12600)
    model = SimpleNamespace(identity_hash='physical-model', model_version=1,
        contact_frame='suction_contact_frame', footprint_type='rectangle',
        footprint_size_xy_m=(.28, .12), boundary_margin_m=.015)
    return domain, model


def contact_pose(x=0., y=0., yaw=0., tilt=0.):
    cy, sy, ct, st = np.cos(yaw), np.sin(yaw), np.cos(tilt), np.sin(tilt)
    yaw_rotation = np.array([[cy, -sy, 0.], [sy, cy, 0.], [0., 0., 1.]])
    tilt_rotation = np.array([[ct, 0., st], [0., 1., 0.], [-st, 0., ct]])
    result = np.eye(4)
    result[:3, :3] = yaw_rotation @ tilt_rotation @ np.diag([1., -1., -1.])
    result[:3, 3] = (x, y, 1.)
    return result


def test_direction_changes_admissible_eccentric_contact_for_non_square_tool():
    domain, model = domain_and_model()
    assert check_observed_contact_footprint(domain, model, contact_pose(y=.12)).contained
    assert not check_observed_contact_footprint(domain, model,
        contact_pose(y=.12, yaw=np.pi/2)).contained
    assert check_observed_contact_footprint(domain, model,
        contact_pose(y=.12, yaw=np.pi)).contained


def test_contact_can_touch_boundary_but_one_mm_overhang_rejects():
    domain, model = domain_and_model()
    assert check_observed_contact_footprint(domain, model, contact_pose(x=.195)).contained
    assert not check_observed_contact_footprint(domain, model, contact_pose(x=.196)).contained


def test_unknown_interior_and_edge_notch_reject_even_with_supported_corners():
    domain, model = domain_and_model()
    cells = domain.cells().copy()
    cells[70, 45] = False
    assert not check_observed_contact_footprint(
        replace(domain, support=cells.tobytes()), model, contact_pose()).contained
    cells[:] = True
    cells[99:102, 44:47] = False  # inward notch at the +u edge
    assert not check_observed_contact_footprint(
        replace(domain, support=cells.tobytes()), model, contact_pose()).contained


def test_projection_uses_actual_tilted_pose_without_minting_surface_quality():
    domain, model = domain_and_model()
    # Tilt reduces projected u extent. This gate checks support only; an
    # independent contact/quality gate still has to reject excessive tilt.
    assert not check_observed_contact_footprint(domain, model, contact_pose(x=.21)).contained
    assert check_observed_contact_footprint(domain, model,
        contact_pose(x=.21, tilt=np.pi/6)).contained
    assert not check_observed_contact_footprint(domain, model,
        contact_pose(tilt=np.pi/2)).contained


@pytest.mark.parametrize('angle', [0., np.pi/7, np.pi/4, np.pi/2, np.pi, -np.pi/2])
def test_projected_polygon_agrees_with_planar_rectangle_gate(angle):
    domain, model = domain_and_model()
    cells = domain.cells().copy()
    cells[95, 60] = False
    domain = replace(domain, support=cells.tobytes())
    for x, y in [(0., 0.), (.15, .05), (.20, .1), (-.16, -.12)]:
        actual = check_observed_contact_footprint(domain, model, contact_pose(x, y, angle))
        expected = check_footprint_containment(cells, origin_uv=domain.origin_uv_top_m,
            cell_size_m=domain.cell_size_m, center_uv=(x, y), yaw_rad=angle,
            footprint_size_xy_m=model.footprint_size_xy_m,
            boundary_margin_m=model.boundary_margin_m)
        assert actual == expected


def test_world_rotation_and_translation_preserve_observed_plane_containment():
    domain, model = domain_and_model()
    angle = .31
    rotation = np.array([[1., 0., 0.], [0., np.cos(angle), -np.sin(angle)],
                         [0., np.sin(angle), np.cos(angle)]])
    translation = np.array([1.2, -.6, .3])
    basis = np.asarray((domain.u_axis_world, domain.v_axis_world, domain.normal_world)) @ rotation.T
    moved = replace(domain, u_axis_world=tuple(basis[0]), v_axis_world=tuple(basis[1]),
        normal_world=tuple(basis[2]),
        plane_offset_world_m=domain.plane_offset_world_m - basis[2] @ translation,
        origin_uv_top_m=tuple(np.asarray(domain.origin_uv_top_m) + basis[:2] @ translation))
    pose = contact_pose(x=.18, yaw=.2)
    transformed = pose.copy()
    transformed[:3, :3] = rotation @ pose[:3, :3]
    transformed[:3, 3] = rotation @ pose[:3, 3] + translation
    assert check_observed_contact_footprint(moved, model, transformed) == \
        check_observed_contact_footprint(domain, model, pose)


@pytest.mark.parametrize('change', [dict(identity_hash='other'), dict(model_version=2),
    dict(contact_frame='tcp'), dict(footprint_type='circle'),
    dict(footprint_size_xy_m=(.1, 0.)), dict(boundary_margin_m=-.01)])
def test_mismatched_physical_model_refuses(change):
    domain, model = domain_and_model()
    model.__dict__.update(change)
    with pytest.raises(ValueError): check_observed_contact_footprint(domain, model, contact_pose())


@pytest.mark.parametrize('bad', [np.zeros((4, 4)), np.eye(3), np.eye(4)*np.nan,
    np.diag([-1., 1., 1., 1.])])
def test_malformed_or_improper_contact_pose_refuses(bad):
    domain, model = domain_and_model()
    with pytest.raises(ValueError): check_observed_contact_footprint(domain, model, bad)


@pytest.mark.parametrize('vertices', [
    [[0., 0.], [.1, .1], [0., .1], [.1, 0.]],  # crossed order
    [[0., 0.], [.1, 0.], [.05, .02], [0., .1]],  # concave
    [[0., 0.], [.1, 0.], [.1, 0.], [0., .1]],
])
def test_invalid_polygon_cannot_be_used_as_contact_outline(vertices):
    with pytest.raises(ValueError):
        check_convex_footprint_containment(np.ones((30, 30), dtype=bool),
            origin_uv=(-.1, -.1), cell_size_m=.01, vertices_uv=vertices)
