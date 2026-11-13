"""Unresolved geometric contacts are diagnostic hypotheses, never grasp permits."""
from dataclasses import dataclass
import math

import numpy as np

from luggage_perception.observed_suction_domain import MAX_DOMAIN_CELLS
from luggage_perception.suction_footprint_containment import convex_footprint_cell_window


def contact_matrix(domain, uv, yaw):
    normal = np.asarray(domain.normal_world)
    x = math.cos(yaw)*np.asarray(domain.u_axis_world)+math.sin(yaw)*np.asarray(domain.v_axis_world)
    matrix = np.eye(4)
    matrix[:3,:3] = np.column_stack((x,-np.cross(normal,x),-normal))
    matrix[:3,3] = domain.world_point(uv)
    return matrix


def footprint_window(vertices, origin, resolution, *, max_cells):
    """Rasterize the whole quad, including outside the source grid/image.

    The returned integer origin indexes the original grid. No outside cell is
    inferred valid. The allocation cap is a resource bound, not a sensor margin.
    """
    vertices = np.asarray(vertices, dtype=float)
    origin = np.asarray(origin, dtype=float)
    if (vertices.shape != (4, 2) or not np.isfinite(vertices).all()
            or origin.shape != (2,) or not np.isfinite(origin).all()
            or not math.isfinite(resolution) or resolution <= 0):
        raise ValueError('FINITE_PROJECTED_FOOTPRINT_REQUIRED')
    first = np.floor((vertices.min(axis=0)-origin)/resolution).astype(np.int64)
    last = np.ceil((vertices.max(axis=0)-origin)/resolution).astype(np.int64)
    shape = tuple(int(v) for v in last-first)
    if min(shape) <= 0 or math.prod(shape) > max_cells:
        raise ValueError('FOOTPRINT_DIAGNOSTIC_RESOURCE_LIMIT')
    local = convex_footprint_cell_window(shape, origin_uv=origin+first*resolution,
        cell_size_m=resolution, vertices_uv=vertices)
    if local.reason != 'ok':
        raise ValueError('FOOTPRINT_DIAGNOSTIC_UNRESOLVED:'+local.reason)
    return tuple(int(v) for v in first+local.first_ij), local.membership


def source_overlap(first, membership, source_shape):
    """Slices for a virtual footprint window and its intersection with a source."""
    low = np.maximum(first, 0)
    high = np.minimum(np.asarray(first)+membership.shape, source_shape)
    if np.any(high <= low):
        return (slice(0, 0), slice(0, 0)), (slice(0, 0), slice(0, 0))
    virtual = tuple(slice(int(a-f), int(b-f)) for a,b,f in zip(low,high,first))
    source = tuple(slice(int(a), int(b)) for a,b in zip(low,high))
    return virtual, source


@dataclass(frozen=True)
class UnresolvedContactProbe:
    query_id: str
    domain_sha256: str
    contact_model_hash: str
    center_uv_top_m: tuple
    yaw_rad: float
    footprint_uv_top_m: tuple
    unproven_cell_count: int

    def to_message(self, placement, snapshot, request_id, checked_centres):
        from luggage_msgs.msg import SuctionSupportProbeV2
        from luggage_planning.place.place_service import pose_message
        pose = pose_message(dict(contract_version=2, kind='pose', frame_id='container_link',
            length_unit='m', stamp_ns=0, time_basis='scene', scene_revision=snapshot.scene_revision,
            geometry_hash=snapshot.geometry_hash, map_revision=snapshot.map_revision,
            request_id=request_id, reference='box_center', orientation_semantics='box_axis_pi',
            position=placement.container_from_box.translation,
            quaternion_xyzw=placement.container_from_box.quaternion_xyzw))
        return SuctionSupportProbeV2(query_id=self.query_id, domain_sha256=self.domain_sha256,
            contact_model_hash=self.contact_model_hash, center_uv_top_m=list(self.center_uv_top_m),
            yaw_rad=self.yaw_rad, footprint_uv_top_m=list(self.footprint_uv_top_m),
            unproven_cell_count=self.unproven_cell_count, checked_centres=checked_centres,
            placement_slot_id=placement.slot_id, placement=pose)


def unresolved_probe(domain, model, query_id, center, yaw, vertices):
    """Count missing support without filling it or testing unbounded new centres."""
    try:
        first, membership = footprint_window(vertices, domain.origin_uv_top_m,
            domain.cell_size_m, max_cells=MAX_DOMAIN_CELLS)
    except ValueError:
        return None  # Optional diagnostics must not change ordinary search.
    virtual, source = source_overlap(first, membership, domain.shape)
    valid = np.count_nonzero(membership[virtual] & domain.cells()[source])
    unproven = int(np.count_nonzero(membership)-valid)
    if not unproven:
        return None
    return UnresolvedContactProbe(query_id, domain.digest(), model.identity_hash,
        tuple(float(v) for v in center), float(yaw),
        tuple(float(v) for v in np.asarray(vertices).ravel()), unproven)
