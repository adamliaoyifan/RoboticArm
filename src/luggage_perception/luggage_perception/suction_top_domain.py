"""Conservative whole-cell support on one observed connected box top.

Pixel centres have integer coordinates and support extends half a pixel on
each side. A plane cell is admitted only when the bounding rectangle of its
projected four corners is completely covered by admitted pixels. This may
reject extra cells for an oblique camera, but cannot fill holes or overhangs.
There is no interpolation, convex hull, majority vote or image-border clipping
that can turn unknown support into valid support. Sensor uncertainty still
requires the contact model's boundary margin in the footprint consumer.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TopDomain:
    valid_cells: np.ndarray  # (u, v), every admitted cell is wholly supported
    origin_uv: tuple
    cell_size_m: float
    stamp: float
    frame_id: str


def build_top_domain(hmap, top, depth_mm, instance_region, intrinsics,
                     optical_to_world, *, stamp, frame_id):
    """Build from one acquisition, rejecting missing/mismatched provenance.

    The caller must supply depth, mask, intrinsics and acquisition-time TF
    from its exact-stamp join. Top stamp/frame are checked here; plain arrays
    cannot independently establish a camera/instance identity. Invalid inputs
    raise ValueError and must never be replaced by an all-valid domain.
    """
    if (not np.isfinite(stamp) or float(top.stamp) != float(stamp)
            or not frame_id or str(top.frame) != str(frame_id)
            or top.reason != "ok"):
        raise ValueError("top domain acquisition identity mismatch")
    depth = np.asarray(depth_mm)
    region = np.asarray(instance_region)
    patch = np.asarray(getattr(top, "winning_patch_mask", None))
    labels = np.asarray(top.plane_labels)
    if (depth.ndim != 2 or not depth.size or region.dtype != np.bool_
            or patch.dtype != np.bool_ or region.shape != depth.shape
            or patch.shape != depth.shape or labels.shape != depth.shape
            or int(top.winner_plane_id) <= 0):
        raise ValueError("top domain requires same-image boolean masks and labels")
    admitted = (region & patch & (labels == top.winner_plane_id)
                & np.isfinite(depth) & (depth > 0))
    cells = project_supported_cells(
        admitted, origin_uv=(hmap.origin_u, hmap.origin_v),
        shape=hmap.shape, cell_size_m=hmap.cell_size_m,
        plane_basis=(hmap.u_axis, hmap.v_axis, hmap.normal),
        plane_offset=hmap.plane_d, intrinsics=intrinsics,
        optical_to_world=optical_to_world)
    cells.setflags(write=False)
    return TopDomain(cells, (hmap.origin_u, hmap.origin_v),
                     float(hmap.cell_size_m), float(stamp), str(frame_id))


def project_supported_cells(admitted_pixels, *, origin_uv, shape, cell_size_m,
                            plane_basis, plane_offset, intrinsics,
                            optical_to_world):
    """Project plane-grid cells into a rectified pinhole image (pure geometry)."""
    pixels = np.asarray(admitted_pixels)
    basis = np.asarray(plane_basis, dtype=float)
    transform = np.asarray(optical_to_world, dtype=float)
    origin = np.asarray(origin_uv, dtype=float)
    camera = np.asarray([intrinsics.fx, intrinsics.fy,
                         intrinsics.cx, intrinsics.cy], dtype=float)
    grid_shape = np.asarray(shape)
    if (pixels.ndim != 2 or pixels.dtype != np.bool_ or not pixels.size
            or origin.shape != (2,) or not np.isfinite(origin).all()
            or grid_shape.shape != (2,) or grid_shape.dtype.kind not in "iu"
            or np.any(grid_shape < 0)
            or not np.isfinite(cell_size_m) or cell_size_m <= 0
            or not np.isfinite(plane_offset)
            or basis.shape != (3, 3) or not np.isfinite(basis).all()
            or not np.allclose(basis @ basis.T, np.eye(3), atol=1e-7, rtol=0)
            or np.linalg.det(basis) < 0
            or transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3],
                               np.eye(3), atol=1e-7, rtol=0)
            or np.linalg.det(transform[:3, :3]) < 0
            or not np.isfinite(camera).all() or np.any(camera[:2] <= 0)):
        raise ValueError("invalid top domain projection geometry")
    result = np.zeros(tuple(grid_shape), dtype=bool)
    if not result.size:
        return result
    indices = np.indices(tuple(grid_shape)).reshape(2, -1).T
    corners_uv = (origin + (indices[:, None, :]
                  + np.asarray([[0, 0], [1, 0], [1, 1], [0, 1]]))
                  * float(cell_size_m))
    world = (corners_uv @ basis[:2] - float(plane_offset) * basis[2])
    # Inverse rigid transform for row-vector points.
    optical = (world - transform[:3, 3]) @ transform[:3, :3]
    front = np.all(optical[:, :, 2] > 1e-9, axis=1)
    safe_z = np.where(optical[:, :, 2] > 1e-9, optical[:, :, 2], 1.0)
    projected = optical[:, :, :2] / safe_z[:, :, None] * camera[:2] + camera[2:]
    lower = projected.min(axis=1)
    upper = projected.max(axis=1)
    height, width = pixels.shape
    in_image = (front & np.all(np.isfinite(projected), axis=(1, 2))
                & np.all(lower >= -.5, axis=1)
                & np.all(upper <= [width - .5, height - .5], axis=1))
    rows = np.flatnonzero(in_image)
    # Bounds only for in-image cells; clipping would admit partial overhangs.
    lo = np.floor(lower[rows] + .5).astype(int)
    hi = np.ceil(upper[rows] + .5).astype(int)
    summed = np.pad((~pixels).astype(np.int64), ((1, 0), (1, 0)))
    summed = summed.cumsum(axis=0).cumsum(axis=1)
    bad = (summed[hi[:, 1], hi[:, 0]] - summed[lo[:, 1], hi[:, 0]]
           - summed[hi[:, 1], lo[:, 0]] + summed[lo[:, 1], lo[:, 0]])
    nonempty = np.all(hi > lo, axis=1)
    result.flat[rows] = (bad == 0) & nonempty
    return result
