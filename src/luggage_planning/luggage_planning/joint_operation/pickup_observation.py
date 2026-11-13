"""Locate decision-relevant missing measurements; never fill observed support.

This pure boundary consumes one acquisition. A proposed view still needs full
robot motion checks, an actual new acquisition and conditional same-box matching.
"""
from dataclasses import dataclass

import numpy as np

from luggage_perception.observed_contact_footprint import project_contact_footprint
from luggage_perception.observed_suction_domain import MAX_DOMAIN_CELLS
from luggage_perception.pickup_view_evidence import PickupViewEvidence
from luggage_planning.joint_operation.contact_observation import (
    UnresolvedContactProbe, contact_matrix, footprint_window, source_overlap, unresolved_probe,
)


@dataclass(frozen=True)
class PickupObservationNeed:
    domain_sha256: str
    view_sha256: str
    query_id: str
    target_world: tuple
    footprint_world: tuple
    gap_targets_world: tuple
    region_source: str
    missing_depth_pixels: int
    outside_image_pixels: int
    excluded_region_pixels: int


@dataclass(frozen=True)
class PickupObservationAssessment:
    reason: str
    need: object = None
    required_pixels: int = 0
    missing_depth_pixels: int = 0
    outside_image_pixels: int = 0
    excluded_region_pixels: int = 0


def optical_projection(world_points, world_from_optical, intrinsics):
    """Rectified pinhole projection using the acquisition's actual geometry."""
    matrix = np.asarray(world_from_optical).reshape(4,4)
    points = (np.asarray(world_points)-matrix[:3,3]) @ matrix[:3,:3]
    if np.any(points[:,2] <= 0) or not np.isfinite(points).all():
        raise ValueError('FOOTPRINT_NOT_IN_FRONT_OF_CAMERA')
    fx,fy,cx,cy = intrinsics
    return points[:,:2]/points[:,2,None]*[fx,fy]+[cx,cy], points[:,2]


def assess_pickup_observation(domain, model, probe, view):
    """A mask exclusion or quality/IK failure alone is not an information gap.

    A bbox does not prove an instance contour. Unknown cause is retained: a
    missing depth sample is not called an interior hole or a true box face.
    """
    if type(probe) is not UnresolvedContactProbe or type(view) is not PickupViewEvidence:
        return PickupObservationAssessment('TYPED_PROBE_AND_ACQUISITION_REQUIRED')
    acquisition = view.acquisition
    if ((probe.domain_sha256, probe.contact_model_hash, view.domain_sha256, view.box_id) !=
            (domain.digest(), model.identity_hash, domain.digest(), domain.box_id)
            or (acquisition.stamp_ns,acquisition.instance_id,acquisition.generation) !=
               (domain.stamp_ns,domain.instance_id,domain.generation)):
        return PickupObservationAssessment('PROBE_VIEW_SOURCE_MISMATCH')
    try:
        vertices = project_contact_footprint(domain,model,
            contact_matrix(domain,probe.center_uv_top_m,probe.yaw_rad))
        if vertices is None or not np.allclose(vertices.ravel(),probe.footprint_uv_top_m,atol=1e-12,rtol=0):
            return PickupObservationAssessment('PROBE_PHYSICAL_FOOTPRINT_MISMATCH')
        checked = unresolved_probe(domain,model,probe.query_id,probe.center_uv_top_m,probe.yaw_rad,vertices)
        if checked is None or checked.unproven_cell_count != probe.unproven_cell_count:
            return PickupObservationAssessment('NO_BOUND_UNPROVEN_SUPPORT')
        world = np.array([domain.world_point(uv) for uv in vertices])
        pixels,_ = optical_projection(world,acquisition.world_from_optical,acquisition.intrinsics_fx_fy_cx_cy)
        # Pixel centres are integer coordinates; pixel square boundaries are i±.5.
        first,membership = footprint_window(pixels,(-.5,-.5),1.,max_cells=MAX_DOMAIN_CELLS)
        virtual,source = source_overlap(first,membership,acquisition.shape[::-1])
        inside = membership[virtual]
        depth = np.frombuffer(acquisition.depth_mm,dtype=acquisition.depth_dtype).reshape(acquisition.shape).T[source]
        region = np.frombuffer(acquisition.instance_region,dtype=bool).reshape(acquisition.shape).T[source]
        finite = np.isfinite(depth) & (depth > 0)
        missing = int(np.count_nonzero(inside & region & ~finite))
        excluded = int(np.count_nonzero(inside & ~region))
        required = int(np.count_nonzero(membership))
        outside = required-int(np.count_nonzero(inside))
        # Localize missing measurements in ray geometry, without declaring
        # these plane intersections to be real face/support measurements.
        gaps = []
        if missing:
            coordinates = np.argwhere(inside & region & ~finite)
            low = coordinates.min(axis=0)+[source[0].start,source[1].start]
            high = coordinates.max(axis=0)+[source[0].start,source[1].start]
            fx,fy,cx,cy = acquisition.intrinsics_fx_fy_cx_cy
            matrix = np.asarray(acquisition.world_from_optical).reshape(4,4)
            for px,py in (low,high,(low[0],high[1]),(high[0],low[1])):
                ray = matrix[:3,:3] @ [(px-cx)/fx,(py-cy)/fy,1.]
                divisor = float(np.dot(domain.normal_world,ray))
                if abs(divisor)>1e-12:
                    length = -(np.dot(domain.normal_world,matrix[:3,3])+domain.plane_offset_world_m)/divisor
                    if length>0:gaps.append(matrix[:3,3]+length*ray)
        if outside:
            # Quad corners represent a clipped extent; neither visibility nor
            # observed same-box face is claimed outside the original image.
            gaps.extend(world[np.any((pixels<[-.5,-.5]) | (pixels>np.array(acquisition.shape[::-1])-.5),axis=1)])
    except (ValueError,TypeError,OverflowError) as error:
        return PickupObservationAssessment('OBSERVATION_ASSESSMENT_UNRESOLVED:'+str(error))
    counts = dict(required_pixels=required,missing_depth_pixels=missing,
                  outside_image_pixels=outside,excluded_region_pixels=excluded)
    if not (missing or outside):
        return PickupObservationAssessment('NO_LOCALIZED_MISSING_MEASUREMENT',**counts)
    need = PickupObservationNeed(domain.digest(),view.digest(),probe.query_id,
        domain.world_point(probe.center_uv_top_m),tuple(float(v) for v in world.ravel()),
        tuple(float(v) for v in np.asarray(gaps).ravel()),
        acquisition.region_source,missing,outside,excluded)
    return PickupObservationAssessment('CONDITIONAL_OBSERVATION_NEED',need,**counts)
