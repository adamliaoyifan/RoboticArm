#!/usr/bin/env python3
"""G2 tests for exact hull-weighted cargo-map volume and activation."""

import math

import pytest

from luggage_perception.cargo_volume_mapper import CargoVolumeMapper


EXPECTED_VOLUME = 4.22433625


def seven_face_descriptor():
    return {
        "schema_version": 1,
        "frame_id": "container_link",
        "length": 1.49,
        "width": 1.97,
        "floor_z": 0.53,
        "ceiling_z": 2.01,
        "chamfer": {
            "side": "positive_y",
            "floor_y": 0.55,
            "wall_y": 0.985,
            "wall_z": 0.90,
        },
    }


def mapper(resolution):
    return CargoVolumeMapper(
        inner_size=(1.49, 1.97, 1.48),
        center_base=(0.0, 0.0, 1.27),
        yaw=0.0,
        resolution=resolution,
        geometry_descriptor=seven_face_descriptor(),
    )


@pytest.mark.parametrize("resolution", [0.11, 0.19, 0.31])
def test_empty_non_divisor_grids_sum_to_exact_hull_volume(resolution):
    cargo_map = mapper(resolution)
    stats = cargo_map.stats()

    assert stats["usable_volume"] == pytest.approx(EXPECTED_VOLUME, abs=1e-10)
    assert stats["unknown_volume"] == pytest.approx(EXPECTED_VOLUME, abs=1e-10)
    assert stats["free_volume"] == 0.0
    assert stats["occupied_volume"] == 0.0
    assert stats["unknown_ratio"] == pytest.approx(1.0)
    assert (
        stats["unknown_volume"]
        + stats["free_volume"]
        + stats["occupied_volume"]
    ) == pytest.approx(EXPECTED_VOLUME, abs=1e-10)


@pytest.mark.parametrize("resolution", [0.13, 0.27])
def test_fill_as_free_reports_hull_not_allocation_volume(resolution):
    cargo_map = mapper(resolution)
    cargo_map.fill_unoccupied_as_free()
    stats = cargo_map.stats()

    allocation_volume = (
        cargo_map.nx * cargo_map.ny * cargo_map.nz * resolution ** 3)
    assert stats["free_volume"] == pytest.approx(EXPECTED_VOLUME, abs=1e-10)
    assert stats["unknown_volume"] == 0.0
    assert stats["occupied_volume"] == 0.0
    assert stats["unknown_ratio"] == 0.0
    assert not math.isclose(allocation_volume, stats["free_volume"], abs_tol=1e-3)


def test_boundary_cells_have_fractional_physical_volume():
    cargo_map = mapper(0.2)
    full_nominal_cell = cargo_map.resolution ** 3
    fractional = [
        value for value in cargo_map._cell_volumes
        if 1e-15 < value < full_nominal_cell - 1e-12
    ]
    assert fractional
    assert any(value < 0.25 * full_nominal_cell for value in fractional)
    assert any(1e-12 < value < 1.0 - 1e-12 for value in cargo_map._weights)


def test_zero_weight_wedge_cell_cannot_be_activated():
    cargo_map = mapper(0.1)
    before = cargo_map.stats()

    # This point lies inside the removed lower +Y triangular prism.
    cargo_map.mark_occupied_world(0.0, 0.90, 0.56)
    cargo_map.mark_placed_box(
        center=(0.0, 0.90, 0.56), size=(0.02, 0.02, 0.02))
    after = cargo_map.stats()

    assert after["occupied_count"] == 0
    assert after["occupied_volume"] == 0.0
    assert after["unknown_volume"] == pytest.approx(before["unknown_volume"])


def test_outside_point_in_fractional_boundary_cell_does_not_activate_fragment():
    cargo_map = mapper(0.2)
    # The cell intersects the slanted face, but this particular point is on
    # the removed side. Cell-level activation must still reject the point.
    cargo_map.mark_occupied_world(0.0, 0.80, 0.60)
    assert cargo_map.stats()["occupied_volume"] == 0.0


def test_cuboid_fallback_retains_exact_partial_edge_behavior():
    cargo_map = CargoVolumeMapper(
        inner_size=(1.0, 0.8, 0.6),
        center_base=(0.0, 0.0, 0.3),
        yaw=0.0,
        resolution=0.17,
    )
    assert cargo_map.stats()["usable_volume"] == pytest.approx(0.48, abs=1e-12)
    cargo_map.fill_unoccupied_as_free()
    assert cargo_map.stats()["free_volume"] == pytest.approx(0.48, abs=1e-12)


def test_non_box_predicate_without_descriptor_fails_closed():
    with pytest.raises(ValueError, match="geometry_descriptor is required"):
        CargoVolumeMapper(
            inner_size=(1.0, 1.0, 1.0),
            center_base=(0.0, 0.0, 0.5),
            yaw=0.0,
            resolution=0.2,
            hull_local_inside=lambda _x, _y, _z: True,
        )
