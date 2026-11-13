"""Explicit-budget camera hypotheses; visibility is not measured information gain."""
from dataclasses import dataclass
import itertools
import math

import numpy as np

from luggage_perception.pickup_view_evidence import PickupViewEvidence
from luggage_planning.joint_operation.pickup_observation import PickupObservationNeed, optical_projection
from luggage_planning.placement_prior.contracts import Transform


@dataclass(frozen=True)
class PickupViewSampling:
    # All sampling/range/work settings are explicit. No production default views.
    tilts_rad: tuple
    azimuths_rad: tuple
    distance_scales: tuple
    min_optical_z_m: float
    max_optical_z_m: float
    max_view_proposals: int

    def __post_init__(self):
        for name in ('tilts_rad','azimuths_rad','distance_scales'):
            values = tuple(float(v) for v in getattr(self,name))
            if not values or not all(math.isfinite(v) for v in values):
                raise ValueError('EXPLICIT_FINITE_CAMERA_SAMPLING_REQUIRED')
            object.__setattr__(self,name,values)
        if (any(not 0 <= v < math.pi/2 for v in self.tilts_rad)
                or min(self.distance_scales) <= 0
                or not math.isfinite(self.min_optical_z_m) or not math.isfinite(self.max_optical_z_m)
                or not 0 < self.min_optical_z_m < self.max_optical_z_m
                or type(self.max_view_proposals) is not int or not 1 <= self.max_view_proposals <= 4096
                or len(self.tilts_rad)*len(self.azimuths_rad)*len(self.distance_scales)>4096):
            raise ValueError('BOUNDED_CAMERA_SAMPLING_AND_OPTICAL_RANGE_REQUIRED')


@dataclass(frozen=True)
class PickupCameraProposal:
    domain_sha256: str
    view_sha256: str
    query_id: str
    world_from_optical: Transform
    visible_gap_fraction: float
    camera_displacement_m: float
    projected_area_pixels: float
    # Even all corners in frame is not proof against occlusion or missing depth.
    reason: str = 'UNVALIDATED_CAMERA_HYPOTHESIS'


def propose_pickup_views(domain, view, need, sampling):
    """Generate/rank potential views, leaving full-scene robot validation to owner.

    Camera-to-target distance comes from this acquisition. The observed plane
    defines the sampling hemisphere; no box extent or simulator truth is used.
    """
    if (type(sampling) is not PickupViewSampling or type(need) is not PickupObservationNeed
            or type(view) is not PickupViewEvidence
            or (need.domain_sha256,need.view_sha256,view.domain_sha256,view.box_id) !=
               (domain.digest(),view.digest(),domain.digest(),domain.box_id)):
        raise ValueError('BOUND_OBSERVATION_NEED_AND_EXPLICIT_SAMPLING_REQUIRED')
    acquisition = view.acquisition
    domain_digest,view_digest = domain.digest(),view.digest()
    original = np.asarray(acquisition.world_from_optical).reshape(4,4)
    target = np.asarray(need.target_world)
    distance = float(np.linalg.norm(original[:3,3]-target))
    if distance <= 1e-12:
        return ()
    normal,u,v = map(np.asarray,(domain.normal_world,domain.u_axis_world,domain.v_axis_world))
    corners = np.asarray(need.footprint_world).reshape(4,3)
    gaps = np.asarray(need.gap_targets_world).reshape(-1,3)
    if not len(gaps):
        return ()
    # Ray-plane intersections and clipped corners are explicitly hypotheses.
    samples = np.vstack((corners,gaps))
    height,width = acquisition.shape
    proposals,seen = [],set()
    for tilt,azimuth,scale in itertools.product(sampling.tilts_rad,sampling.azimuths_rad,sampling.distance_scales):
        outward = math.cos(tilt)*normal+math.sin(tilt)*(math.cos(azimuth)*u+math.sin(azimuth)*v)
        eye = target+distance*scale*outward
        z = -outward
        x = original[:3,0]-np.dot(original[:3,0],z)*z
        if np.linalg.norm(x)<=1e-12:
            x = original[:3,1]-np.dot(original[:3,1],z)*z
        x = x/np.linalg.norm(x)
        matrix = np.eye(4);matrix[:3,:3]=np.column_stack((x,np.cross(z,x),z));matrix[:3,3]=eye
        if np.allclose(matrix,original,atol=1e-12,rtol=0):
            continue  # Repeating the same pose is not a proposed extra view.
        try:
            pixels,optical_z = optical_projection(samples,matrix,acquisition.intrinsics_fx_fy_cx_cy)
        except ValueError:
            continue
        in_range = (optical_z>=sampling.min_optical_z_m) & (optical_z<=sampling.max_optical_z_m)
        in_frame = np.all((pixels>=[-.5,-.5]) & (pixels<=[width-.5,height-.5]),axis=1)
        visible = in_range & in_frame
        if not np.any(visible[4:]):
            continue
        key = tuple(np.round(matrix.ravel(),12))  # Pose numerical equality only.
        if key in seen:
            continue
        seen.add(key)
        quad = pixels[:4]
        area = abs(float(np.sum(quad[:,0]*np.roll(quad[:,1],-1)-quad[:,1]*np.roll(quad[:,0],-1))))/2
        proposals.append(PickupCameraProposal(domain_digest,view_digest,need.query_id,
            Transform.from_matrix(matrix),float(np.mean(visible[4:])),
            float(np.linalg.norm(eye-original[:3,3])),area))
    proposals.sort(key=lambda p:(-p.visible_gap_fraction,-p.projected_area_pixels,p.camera_displacement_m))
    return tuple(proposals[:sampling.max_view_proposals])
