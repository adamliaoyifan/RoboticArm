"""Whole-footprint acceptance, independently exercising edges and holes."""
import math

import numpy as np
import pytest

from luggage_perception.suction_footprint_containment import check_footprint_containment


def check(domain, center=(0., 0.), yaw=0., size=(.18, .18), margin=.015):
    return check_footprint_containment(
        domain, origin_uv=(-.35, -.225), cell_size_m=.005,
        center_uv=center, yaw_rad=yaw, footprint_size_xy_m=size,
        boundary_margin_m=margin)


def face():
    return np.ones((140, 90), dtype=bool)  # measured 0.70 x 0.45 m face


def test_ideal_box_accepts_exact_geometric_limit_but_not_one_mm_overhang():
    assert check(face(), center=(.245, .120)).contained
    assert not check(face(), center=(.246, .120)).contained
    assert not check(face(), center=(.245, .121)).contained


def test_interior_and_too_large_footprint():
    assert check(face()).contained
    assert not check(face(), size=(.72, .18)).contained


def test_rotating_non_square_footprint_changes_admissible_offsets():
    assert check(face(), center=(0., .12), size=(.28, .12)).contained
    assert not check(face(), center=(0., .12), yaw=math.pi/2,
                     size=(.28, .12)).contained


def test_45_degree_projection_must_fit_entirely():
    assert check(face(), center=(0., .10)).contained
    assert not check(face(), center=(0., .10), yaw=math.pi/4).contained


def test_interior_unknown_hole_rejects_even_above_95_percent_coverage():
    domain = face()
    good = check(domain)
    domain[70, 45] = False
    bad = check(domain)
    assert bad.invalid_cell_count == 1
    assert 1 - bad.invalid_cell_count / good.covered_cell_count > .95
    assert not bad.contained
    assert bad.first_invalid_cell == (70, 45)


def test_concave_notch_rejects_despite_all_four_corners_inside():
    domain = face()
    # A notch crosses the midpoint of the footprint's +u edge. Its corners
    # all remain inside the face: a four-corner test would accept it.
    domain[87:, 44:47] = False
    assert not check(domain).contained


def test_neighbouring_box_does_not_fill_same_box_domain():
    domain = face()
    domain[70:, :] = False  # adjacent face remains false for THIS instance
    assert not check(domain).contained
    assert check(domain, center=(-.15, 0.)).contained


def test_rotated_polygon_not_its_axis_aligned_bounding_box():
    domain = face()
    # This cell is in the rotated square's AABB but outside the actual square.
    domain[97, 72] = False
    assert check(domain, yaw=math.pi/4).contained
    # The centre is inside the square at every angle.
    domain[70, 45] = False
    assert not check(domain, yaw=math.pi/4).contained


def test_cell_with_edge_overlap_counts_even_when_its_centre_is_outside():
    domain = face()
    # Exact upper edge u=.106 lies inside cell [0.105, 0.110]; its centre
    # is outside the footprint, but the cell must still be in the domain.
    domain[91, 45] = False
    assert check(domain).contained  # edge u=.105 just touches, no area overlap
    assert not check(domain, center=(.001, 0.)).contained


@pytest.mark.parametrize("yaw", [0., math.pi/2, math.pi, -math.pi/2])
def test_directed_quarter_turns_preserve_square_geometry(yaw):
    assert check(face(), center=(.245, .12), yaw=yaw).contained


def test_margin_expands_instead_of_shrinking_required_face():
    assert check(face(), center=(.25, 0.), margin=0.).contained
    assert not check(face(), center=(.25, 0.), margin=.015).contained


@pytest.mark.parametrize("changes", [
    {"cell_size_m": 0.}, {"yaw_rad": float("nan")},
    {"boundary_margin_m": -.01}, {"footprint_size_xy_m": (0., .1)},
    {"center_uv": (float("inf"), 0.)},
])
def test_invalid_inputs_reject(changes):
    args = dict(origin_uv=(-.35, -.225), cell_size_m=.005,
                center_uv=(0., 0.), yaw_rad=0., footprint_size_xy_m=(.18, .18),
                boundary_margin_m=.015)
    args.update(changes)
    with pytest.raises(ValueError):
        check_footprint_containment(face(), **args)


def test_input_domain_is_not_mutated_and_empty_domain_is_not_accepted():
    domain = face()
    original = domain.copy()
    check(domain, yaw=math.pi/4)
    np.testing.assert_array_equal(domain, original)
    assert not check(np.zeros((0, 0), dtype=bool)).contained
    with pytest.raises(ValueError):
        check(np.ones((140, 90), dtype=float))
