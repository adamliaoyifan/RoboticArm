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


@dataclass(frozen=True)
class FootprintCellWindow:
    first_ij: tuple
    membership: np.ndarray
    reason: str


def convex_footprint_cell_window(grid_shape, *, origin_uv, cell_size_m, vertices_uv):
    """Rasterize positive-area cell intersections in a bounded local window.

    Vertices must be ordered around a strictly convex quadrilateral. This
    handles the parallelogram obtained when an actual tilted contact frame is
    projected onto the observed top plane. The caller expands the physical
    footprint by its configured margin BEFORE projecting it. Containment alone
    does not prove coplanar contact, surface quality, seal or collision freedom.
    """
    shape = tuple(grid_shape)
    origin = np.asarray(_pair(origin_uv, "origin_uv"))
    vertices = np.asarray(vertices_uv, dtype=float)
    resolution = float(cell_size_m)
    if len(shape) != 2 or any(type(v) is not int or v < 0 for v in shape):
        raise ValueError("nonnegative two-dimensional grid shape required")
    if (vertices.shape != (4, 2) or not np.isfinite(vertices).all()
            or not math.isfinite(resolution) or resolution <= 0):
        raise ValueError("finite quadrilateral and positive cell size required")
    tolerance = 1e-12  # metres of roundoff, not an observation/filling allowance
    edges = np.roll(vertices, -1, axis=0) - vertices
    lengths = np.linalg.norm(edges, axis=1)
    if np.any(lengths <= tolerance):
        raise ValueError("nondegenerate contact quadrilateral required")
    unit_edges = edges / lengths[:, None]
    turns = (unit_edges[:, 0] * np.roll(unit_edges, -1, axis=0)[:, 1]
             - unit_edges[:, 1] * np.roll(unit_edges, -1, axis=0)[:, 0])
    if not (np.all(turns > 0) or np.all(turns < 0)):
        raise ValueError("ordered strictly convex contact quadrilateral required")
    empty = np.frombuffer(b'', dtype=bool).reshape(0, 0)
    if 0 in shape:
        return FootprintCellWindow((0, 0), empty, "EMPTY_TOP_DOMAIN")
    lower, upper = vertices.min(axis=0), vertices.max(axis=0)
    if (np.any(lower < origin - tolerance)
            or np.any(upper > origin + np.asarray(shape)*resolution + tolerance)):
        return FootprintCellWindow((0, 0), empty, "FOOTPRINT_OUTSIDE_TOP_GRID")
    first = np.maximum(0, np.floor((lower-origin-tolerance)/resolution).astype(int))
    last = np.minimum(shape, np.ceil((upper-origin+tolerance)/resolution).astype(int))
    iu, iv = np.meshgrid(np.arange(first[0], last[0]),
                         np.arange(first[1], last[1]), indexing="ij")
    centers = origin + (np.stack((iu, iv), axis=-1) + .5)*resolution
    overlap = np.ones(iu.shape, dtype=bool)
    # Separating axes from both the grid cell and the projected polygon.
    axes = np.vstack((np.eye(2), np.column_stack((-unit_edges[:, 1], unit_edges[:, 0]))))
    for axis in axes:
        projection = vertices @ axis
        cell_center = centers @ axis
        radius = resolution/2 * np.abs(axis).sum()
        overlap &= ((cell_center + radius > projection.min() + tolerance)
                    & (cell_center - radius < projection.max() - tolerance))
    membership = np.frombuffer(overlap.tobytes(), dtype=bool).reshape(overlap.shape)
    return FootprintCellWindow(tuple(int(v) for v in first), membership, "ok")


def check_convex_footprint_containment(valid_cells, *, origin_uv, cell_size_m,
                                       vertices_uv):
    """Require all cells touched by the actual projected quadrilateral."""
    domain = np.asarray(valid_cells)
    if domain.ndim != 2 or domain.dtype != np.bool_:
        raise ValueError("two-dimensional boolean top domain required")
    window = convex_footprint_cell_window(domain.shape, origin_uv=origin_uv,
        cell_size_m=cell_size_m, vertices_uv=vertices_uv)
    if window.reason != 'ok':
        return ContainmentResult(False, window.reason, 0, 0)
    first_u, first_v = window.first_ij
    nu, nv = window.membership.shape
    covered = int(np.count_nonzero(window.membership))
    invalid = window.membership & ~domain[first_u:first_u+nu, first_v:first_v+nv]
    invalid_count = int(np.count_nonzero(invalid))
    if invalid_count:
        first_bad = tuple(np.argwhere(invalid)[0])
        return ContainmentResult(False, "FOOTPRINT_NOT_FULLY_CONTAINED", covered,
            invalid_count, (int(first_u+first_bad[0]), int(first_v+first_bad[1])))
    if not covered:
        return ContainmentResult(False, "FOOTPRINT_NOT_RESOLVED", 0, 0)
    return ContainmentResult(True, "ok", covered, 0)


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
