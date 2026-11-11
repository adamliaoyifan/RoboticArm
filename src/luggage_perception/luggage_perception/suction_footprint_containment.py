"""Complete oriented suction-footprint containment in one measured top domain.

Coordinates are metres in the caller's observation-bound top plane (u, v).
``valid_cells[i, j]`` authorizes the WHOLE cell, not merely its centre or a
majority of observed points. The caller constructs that conservative same-box
domain and preserves observation identity. False means unknown or outside the
face; no convex hull, interpolation, or hole filling is done here.

This necessary geometric gate does not replace surface quality evaluation or
tool/arm/payload collision checks. A rectangular boundary margin enlarges the
footprint in its local axes; this conservatively contains a Euclidean margin.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class ContainmentResult:
    contained: bool
    reason: str
    covered_cell_count: int
    invalid_cell_count: int
    first_invalid_cell: tuple = ()


def _pair(values, name):
    if len(values) != 2:
        raise ValueError(name + " requires two finite coordinates")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(name + " requires two finite coordinates")
    return result


def check_footprint_containment(valid_cells, *, origin_uv, cell_size_m,
                                center_uv, yaw_rad, footprint_size_xy_m,
                                boundary_margin_m):
    """Test every raster cell with positive-area footprint intersection.

    Separating axes of both rectangles determine intersection, so this checks
    rotated edges and interior holes as well as corners. Exact tangency does
    not include a neighbouring cell; metre-scale roundoff is bounded by 1e-12.
    Out-of-grid area rejects before raster clipping. Inputs are never mutated.
    """
    domain = np.asarray(valid_cells)
    if domain.ndim != 2 or domain.dtype != np.bool_:
        raise ValueError("valid_cells must be a two-dimensional boolean domain")
    origin_u, origin_v = _pair(origin_uv, "origin_uv")
    center_u, center_v = _pair(center_uv, "center_uv")
    width, depth = _pair(footprint_size_xy_m, "footprint_size_xy_m")
    resolution = float(cell_size_m)
    margin = float(boundary_margin_m)
    yaw = float(yaw_rad)
    if (not all(math.isfinite(value) for value in (resolution, margin, yaw))
            or resolution <= 0 or margin < 0 or min(width, depth) <= 0):
        raise ValueError("positive cell/footprint sizes and nonnegative margin required")
    if 0 in domain.shape:
        return ContainmentResult(False, "EMPTY_TOP_DOMAIN", 0, 0)

    half_u, half_v = width / 2 + margin, depth / 2 + margin
    cosine, sine = math.cos(yaw), math.sin(yaw)
    extent_u = abs(cosine) * half_u + abs(sine) * half_v
    extent_v = abs(sine) * half_u + abs(cosine) * half_v
    bounds = (center_u - extent_u, center_v - extent_v,
              center_u + extent_u, center_v + extent_v)
    tolerance = 1e-12  # numerical boundary tolerance in metres, not a margin
    if (bounds[0] < origin_u - tolerance or bounds[1] < origin_v - tolerance
            or bounds[2] > origin_u + domain.shape[0] * resolution + tolerance
            or bounds[3] > origin_v + domain.shape[1] * resolution + tolerance):
        return ContainmentResult(False, "FOOTPRINT_OUTSIDE_TOP_GRID", 0, 0)

    # Include tangent rows in the broad window; the exact intersection below
    # removes them without losing thin positive-area boundary overlaps.
    first_u = max(0, math.floor((bounds[0] - origin_u - tolerance) / resolution))
    first_v = max(0, math.floor((bounds[1] - origin_v - tolerance) / resolution))
    last_u = min(domain.shape[0], math.ceil((bounds[2] - origin_u + tolerance) / resolution))
    last_v = min(domain.shape[1], math.ceil((bounds[3] - origin_v + tolerance) / resolution))
    iu, iv = np.meshgrid(np.arange(first_u, last_u),
                         np.arange(first_v, last_v), indexing="ij")
    delta_u = origin_u + (iu + .5) * resolution - center_u
    delta_v = origin_v + (iv + .5) * resolution - center_v
    half_cell = resolution / 2
    cell_projection = half_cell * (abs(cosine) + abs(sine))
    overlap = (
        (np.abs(delta_u) < extent_u + half_cell - tolerance)
        & (np.abs(delta_v) < extent_v + half_cell - tolerance)
        & (np.abs(delta_u * cosine + delta_v * sine)
           < half_u + cell_projection - tolerance)
        & (np.abs(-delta_u * sine + delta_v * cosine)
           < half_v + cell_projection - tolerance))
    invalid = overlap & ~domain[iu, iv]
    covered = int(np.count_nonzero(overlap))
    invalid_count = int(np.count_nonzero(invalid))
    if invalid_count:
        row = np.argwhere(invalid)[0]
        first = (int(iu[tuple(row)]), int(iv[tuple(row)]))
        return ContainmentResult(False, "FOOTPRINT_NOT_FULLY_CONTAINED",
                                 covered, invalid_count, first)
    if not covered:
        return ContainmentResult(False, "FOOTPRINT_NOT_RESOLVED", 0, 0)
    return ContainmentResult(True, "ok", covered, 0)
