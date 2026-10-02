"""Shared fixtures for ST09-A tests: a production-shaped chamfered map.

The geometry descriptor (seven-face hull with the positive-Y chamfer and the
-X aperture door) is taken verbatim from the saved rehearsal-52 revision-2
surface, so the synthetic matrices run against the exact production hull the
24 saved slanted-wall candidates violated.
"""

import json
import math
import os

FIXTURE_DIR = os.path.join(os.path.dirname(__file__), "fixtures")

# Rehearsal-52 box-1 measured dimensions (frozen fixture value).
BOX_WDH = (0.646, 0.409, 0.277)


def load_rev2_surface():
    with open(os.path.join(
            FIXTURE_DIR, "rev2_starved_surface_2d.json")) as stream:
        return json.load(stream)


def load_saved24():
    with open(os.path.join(
            FIXTURE_DIR, "slot_hull_violation_saved24.json")) as stream:
        return json.load(stream)


def production_descriptor():
    """The production chamfered hull descriptor from the saved surface."""
    return load_rev2_surface()["geometry_descriptor"]


def chamfered_surface(states="free", heights=0.0, resolution=0.05,
                      descriptor=None, confidence="sensor"):
    """A fresh 30x40 container-local map on the production chamfered hull.

    ``states``/``heights`` accept either a uniform value or a callable
    ``(ix, iy) -> value`` for region-shaped maps. The floor elevation
    (0.53 m in container coordinates) and grid extents mirror the mapper's
    ``surface_map_2d`` contract for this container.
    """
    descriptor = descriptor or production_descriptor()
    length = float(descriptor["length"])
    width = float(descriptor["width"])
    floor_z = float(descriptor["floor_z"])
    ceiling_z = float(descriptor["ceiling_z"])
    nx = int(round(length / resolution))
    ny = int(round(width / resolution))
    inner_size = [length, width, ceiling_z - floor_z]

    def grid(make):
        return [
            [make(ix, iy) for iy in range(ny)]
            for ix in range(nx)
        ]

    state_make = states if callable(states) else (lambda ix, iy: states)
    height_make = heights if callable(heights) else (lambda ix, iy: heights)
    if callable(confidence):
        confidence_make = confidence
    else:
        # Mirror the mapper contract: observed cells carry "sensor"
        # confidence, unobserved columns "none" — an unknown column must
        # never look observed.
        def confidence_make(ix, iy):
            if confidence != "sensor":
                return confidence
            return "none" if state_make(ix, iy) == "unknown" else "sensor"
    return {
        "map_revision": 1,
        "resolution": resolution,
        "nx": nx,
        "ny": ny,
        "inner_size": inner_size,
        "floor_z": 0.0,
        "origin_local": [-length * 0.5, -width * 0.5],
        "center_base": [0.0, 0.0, floor_z + (ceiling_z - floor_z) * 0.5],
        "yaw": 0.0,
        "height": grid(height_make),
        "state": grid(state_make),
        "confidence": grid(confidence_make),
        "geometry_descriptor": descriptor,
        "geometry_hash": descriptor.get("geometry_hash", ""),
        "frame": "container_local",
    }


def surface_state_fn(regions, default="free"):
    """Build a ``(ix, iy) -> state`` callable from rectangular regions.

    ``regions`` is a list of ``(ix0, ix1, iy0, iy1, state)`` half-open cell
    ranges in grid indices.
    """
    def lookup(ix, iy):
        for ix0, ix1, iy0, iy1, state in regions:
            if ix0 <= ix < ix1 and iy0 <= iy < iy1:
                return state
        return default
    return lookup


def surface_height_fn(regions, default=0.0):
    def lookup(ix, iy):
        for ix0, ix1, iy0, iy1, height in regions:
            if ix0 <= ix < ix1 and iy0 <= iy < iy1:
                return height
        return default
    return lookup


def validator_with(placed_aabbs=None, inner_size=None, smallest_size=None,
                   hull=None, inner_h=None, aperture_y=None, floor_z=None):
    """The node's injected validator, built standalone for matrix cases.

    ``floor_z`` is the container-frame floor elevation (0.53 m on the
    production hull): candidate centres are container-absolute while
    ``surface_map_2d`` heights and the placed AABBs are floor-relative, and
    the overlap/corridor gates compare in the floor-relative frame the node
    uses. Defaults to the descriptor's floor.
    """
    from luggage_packing.placement_solver import placement_constraint_verdict
    if hull is None:
        hull = hull_from_descriptor()
    if floor_z is None:
        floor_z = float(hull.floor_z)
    if inner_h is None:
        if inner_size is None:
            inner_h = float(hull.ceiling_z) - float(hull.floor_z)
        else:
            inner_h = float(inner_size[2])
    if inner_size is None:
        inner_size = [2.0 * hull.half_x, 2.0 * hull.half_y, inner_h]

    def _validate(candidate):
        return placement_constraint_verdict(
            candidate, inner_h,
            placed_aabbs=placed_aabbs,
            aperture_y=aperture_y,
            inner_size=inner_size,
            smallest_size=smallest_size,
            hull=hull,
            floor_z=floor_z)
    return _validate


def hull_from_descriptor(descriptor=None):
    from luggage_description.container_geometry import normalize_descriptor
    return normalize_descriptor(descriptor or production_descriptor())


def depth_limits():
    """Grid x thresholds: deep third / shallow third on the 30x40 grid."""
    return {"deep_ix_min": 20, "shallow_ix_max": 10}


PARAMS_FULL = {"top_n": 200, "keep_rejected": 200}

YAWS = (0.0, math.pi / 2.0)
