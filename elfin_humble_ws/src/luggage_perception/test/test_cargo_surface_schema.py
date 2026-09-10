#!/usr/bin/env python3
"""G2 tests for the pure geometry-bearing surface validator."""

import copy
import math

import pytest

from luggage_perception.cargo_surface_schema import (
    SurfaceMapValidationError,
    validate_surface_map,
)

from test_cargo_map_weighted_geometry import mapper


def test_surface_contract_carries_normalized_identity_and_column_geometry():
    cargo_map = mapper(0.17)
    surface = cargo_map.surface_map_2d()
    descriptor = validate_surface_map(
        surface, expected_geometry_hash=cargo_map.geometry_hash)

    assert descriptor == cargo_map.geometry_descriptor
    assert surface["frame_id"] == "container_link"
    assert surface["geometry_floor_z"] == pytest.approx(0.53)
    assert surface["height_semantics"] == "meters_above_geometry_floor"
    assert surface["floor_semantics"] == "horizontal_hull_floor_intersection"
    assert math.fsum(
        value for row in surface["column_usable_volume"] for value in row
    ) == pytest.approx(4.22433625, abs=1e-10)


def test_surface_columns_distinguish_floor_from_space_above_chamfer():
    cargo_map = mapper(0.1)
    surface = cargo_map.surface_map_2d()
    positive_y_column = int((0.90 + 0.5 * cargo_map.inner_w) / 0.1)
    center_x_column = int((0.0 + 0.5 * cargo_map.inner_l) / 0.1)

    assert surface["floor_support_area"][center_x_column][positive_y_column] == 0.0
    assert surface["floor_support"][center_x_column][positive_y_column] is False
    assert surface["floor_support_fraction"][center_x_column][positive_y_column] == 0.0
    assert surface["column_usable_volume"][center_x_column][positive_y_column] > 0.0


@pytest.mark.parametrize(
    ("mutation", "reason"),
    [
        (lambda data: data.pop("surface_schema_version"), "SURFACE_FIELD_MISSING"),
        (lambda data: data.update(surface_schema_version=99), "SURFACE_SCHEMA_UNSUPPORTED"),
        (lambda data: data.pop("geometry_descriptor"), "SURFACE_FIELD_MISSING"),
        (lambda data: data.update(geometry_hash="wrong"), "GEOMETRY_HASH_MISMATCH"),
        (lambda data: data["height"].pop(), "SURFACE_GRID_SHAPE_MISMATCH"),
    ],
)
def test_surface_validator_fails_closed(mutation, reason):
    surface = copy.deepcopy(mapper(0.25).surface_map_2d())
    mutation(surface)
    with pytest.raises(SurfaceMapValidationError) as raised:
        validate_surface_map(surface)
    assert raised.value.reason == reason


def test_expected_geometry_hash_mismatch_fails_closed():
    surface = mapper(0.25).surface_map_2d()
    with pytest.raises(SurfaceMapValidationError) as raised:
        validate_surface_map(surface, expected_geometry_hash="other")
    assert raised.value.reason == "GEOMETRY_HASH_MISMATCH"
