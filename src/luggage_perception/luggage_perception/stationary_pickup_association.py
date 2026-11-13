"""Conditional stationary-box correspondence from measured depth and RGB.

No default accuracy policy is selected here. Matching does not authorize suction,
fuse support, or prove physical scene detection completeness.
"""
from dataclasses import asdict, dataclass
import hashlib
import json
import math

import numpy as np

from luggage_perception.observed_suction_domain import ObservedSuctionDomain
from luggage_perception.pickup_view_evidence import PickupViewEvidence


MAX_ASSOCIATION_SAMPLES = 65536  # Computation bound, never a sensor tolerance.


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False,
                                    separators=(',', ':')).encode()).hexdigest()


@dataclass(frozen=True)
class StationaryPickupPolicy:
    depth_residual_m: float
    rgb_l2_difference: float
    bbox_margin_pixels: float
    min_paired_points: int
    min_fraction: float
    max_samples: int
    provenance: str

    def __post_init__(self):
        scalars = (self.depth_residual_m, self.rgb_l2_difference,
                   self.bbox_margin_pixels, self.min_fraction)
        if (any(type(v) not in (float, int) or not math.isfinite(v) for v in scalars)
                or self.depth_residual_m <= 0
                or not 0 <= self.rgb_l2_difference <= math.sqrt(3)*255
                or self.bbox_margin_pixels < 0 or not 0 < self.min_fraction <= 1
                or type(self.min_paired_points) is not int or self.min_paired_points < 3
                or type(self.max_samples) is not int
                or not self.min_paired_points <= self.max_samples <= MAX_ASSOCIATION_SAMPLES
                or not isinstance(self.provenance, str) or not self.provenance.strip()):
            raise ValueError('EXPLICIT_STATIONARY_ASSOCIATION_POLICY_REQUIRED')

    def digest(self):
        return _digest(asdict(self))


@dataclass(frozen=True)
class PickupCandidateCorrespondence:
    index: int
    verdict: str
    projected_points: int
    paired_points: int
    depth_matches: int
    rgb_matches: int
    joint_matches: int


@dataclass(frozen=True)
class StationaryPickupAssociation:
    accepted: bool
    reason: str
    before_domain_sha256: str
    after_domain_sha256: str
    before_view_sha256: str
    after_view_sha256: str
    policy_sha256: str
    settled_stamp_ns: int
    selected_index: int
    sampled_points: int
    candidates: tuple

    def __post_init__(self):
        candidates = tuple(self.candidates)
        object.__setattr__(self, 'candidates', candidates)
        # A retained positive proof must include its unique selected match.
        if self.accepted:
            compatible = [c for c in candidates if c.verdict == 'COMPATIBLE']
            if (self.reason != 'CONDITIONAL_UNIQUE_STATIONARY_MATCH'
                    or len(compatible) != 1 or compatible[0].index != self.selected_index
                    or any(c.verdict == 'UNRESOLVED_MEASUREMENTS' for c in candidates)):
                raise ValueError('UNIQUE_SELECTED_CORRESPONDENCE_REQUIRED')

    def digest(self):
        return _digest(asdict(self))


def measure_pickup_world_targets(view, world_targets):
    """Project identical physical targets, retaining raw measured depth residual.

    A region can be a bbox; membership is never called observed face support.
    Nearest pixel centres are a sampling convention, not an accuracy allowance.
    """
    acquisition = view.acquisition
    targets = np.asarray(world_targets, dtype=float)
    if targets.ndim != 2 or targets.shape[1] != 3 or not np.isfinite(targets).all():
        raise ValueError('FINITE_WORLD_TARGETS_REQUIRED')
    matrix = np.asarray(acquisition.world_from_optical).reshape(4, 4)
    optical = (targets-matrix[:3, 3]) @ matrix[:3, :3]
    positive = optical[:, 2] > 0
    fx, fy, cx, cy = acquisition.intrinsics_fx_fy_cx_cy
    pixels = np.full((len(targets), 2), np.nan)
    pixels[positive] = optical[positive, :2]/optical[positive, 2, None]*[fx, fy]+[cx, cy]
    safe = np.where(np.isfinite(pixels), pixels, 0)
    # Clip before conversion to avoid overflowing integers for near-zero optical Z.
    height, width = acquisition.shape
    inside = positive & np.all((safe >= [-.5, -.5]) &
                               (safe < [width-.5, height-.5]), axis=1)
    indices = np.floor(np.clip(safe, [0, 0], [width-1, height-1])+.5).astype(int)
    x, y = indices.T
    depth = np.frombuffer(acquisition.depth_mm, dtype=acquisition.depth_dtype).reshape(acquisition.shape)[y, x]/1000.
    region = np.frombuffer(acquisition.instance_region, dtype=bool).reshape(acquisition.shape)[y, x]
    valid = inside & np.isfinite(depth) & (depth > 0)
    inputs = acquisition.association_inputs
    return dict(pixel_xy=pixels, in_image=inside, in_region=inside & region,
                depth_valid=valid, depth_residual_m=np.abs(depth-optical[:, 2]),
                rgb=None if inputs is None else inputs.rgb.array()[y, x].astype(float))


def _observed_source_points(domain, view, policy):
    acquisition = view.acquisition
    depth = np.frombuffer(acquisition.depth_mm, dtype=acquisition.depth_dtype).reshape(acquisition.shape)/1000.
    region = np.frombuffer(acquisition.instance_region, dtype=bool).reshape(acquisition.shape)
    y, x = np.nonzero(region & np.isfinite(depth) & (depth > 0))
    z = depth[y, x]
    fx, fy, cx, cy = acquisition.intrinsics_fx_fy_cx_cy
    optical = np.column_stack(((x-cx)/fx*z, (y-cy)/fy*z, z))
    matrix = np.asarray(acquisition.world_from_optical).reshape(4, 4)
    world = optical @ matrix[:3, :3].T+matrix[:3, 3]
    uv = world @ np.asarray((domain.u_axis_world, domain.v_axis_world)).T
    indices = np.floor((uv-domain.origin_uv_top_m)/domain.cell_size_m).astype(int)
    bounded = np.all((indices >= 0) & (indices < domain.shape), axis=1)
    keep = np.zeros(len(world), dtype=bool)
    keep[bounded] = domain.cells()[tuple(indices[bounded].T)]
    keep &= np.abs(world @ domain.normal_world+domain.plane_offset_world_m) <= policy.depth_residual_m
    world, colours = world[keep], acquisition.association_inputs.rgb.array()[y[keep], x[keep]].astype(float)
    if len(world) > policy.max_samples:
        selected = np.linspace(0, len(world)-1, policy.max_samples, dtype=int)
        world, colours = world[selected], colours[selected]
    return world, colours


def associate_stationary_pickup(before_domain, before_view, after_domain, after_view, *,
                                settled_stamp_ns, stationary, policy):
    if (type(policy) is not StationaryPickupPolicy
            or any(type(d) is not ObservedSuctionDomain for d in (before_domain, after_domain))
            or any(type(v) is not PickupViewEvidence for v in (before_view, after_view))):
        raise ValueError('TYPED_BOUND_ASSOCIATION_INPUTS_REQUIRED')
    base = dict(before_domain_sha256=before_domain.digest(), after_domain_sha256=after_domain.digest(),
                before_view_sha256=before_view.digest(), after_view_sha256=after_view.digest(),
                policy_sha256=policy.digest(), settled_stamp_ns=settled_stamp_ns,
                selected_index=-1, sampled_points=0, candidates=())
    def result(reason, accepted=False, **values):
        return StationaryPickupAssociation(accepted, reason, **{**base, **values})
    if stationary is not True:
        return result('STATIONARY_PREMISE_REQUIRED')
    if (type(settled_stamp_ns) is not int
            or not before_view.acquisition.stamp_ns < settled_stamp_ns < after_view.acquisition.stamp_ns):
        return result('POST_SETTLE_NEW_ACQUISITION_REQUIRED')
    for domain, view in ((before_domain, before_view), (after_domain, after_view)):
        a = view.acquisition
        if ((view.domain_sha256, view.box_id, a.stamp_ns, a.instance_id, a.generation) !=
                (domain.digest(), domain.box_id, domain.stamp_ns, domain.instance_id, domain.generation)):
            return result('ASSOCIATION_DOMAIN_VIEW_MISMATCH')
    if (before_domain.frame_id, before_domain.contact_model_hash, before_domain.contact_model_version) != (
            after_domain.frame_id, after_domain.contact_model_hash, after_domain.contact_model_version):
        return result('ASSOCIATION_FRAME_OR_MODEL_CHANGED')
    if any(v.acquisition.association_inputs is None for v in (before_view, after_view)):
        return result('EXACT_ASSOCIATION_INPUTS_UNAVAILABLE')
    world, rgb = _observed_source_points(before_domain, before_view, policy)
    inputs = after_view.acquisition.association_inputs
    base.update(sampled_points=len(world), selected_index=inputs.selected_index)
    if len(world) < policy.min_paired_points:
        return result('INSUFFICIENT_ORIGINAL_OBSERVED_POINTS')
    measured = measure_pickup_world_targets(after_view, world)
    if np.count_nonzero(measured['in_image']) < max(policy.min_paired_points, policy.min_fraction*len(world)):
        return result('INSUFFICIENT_PROJECTED_WORLD_OVERLAP')
    rows = []
    for index, candidate in enumerate(inputs.candidates):
        low = np.asarray(candidate.bbox[:2])-policy.bbox_margin_pixels
        high = np.asarray(candidate.bbox[2:])+policy.bbox_margin_pixels
        in_box = measured['in_image'] & np.all((measured['pixel_xy'] >= low) &
                                              (measured['pixel_xy'] < high), axis=1)
        paired = in_box & measured['depth_valid']
        depth = paired & (measured['depth_residual_m'] <= policy.depth_residual_m)
        colour = paired & (np.linalg.norm(measured['rgb']-rgb, axis=1) <= policy.rgb_l2_difference)
        joint = depth & colour
        required = max(policy.min_paired_points, policy.min_fraction*len(world))
        counts = [int(np.count_nonzero(x)) for x in (in_box, paired, depth, colour, joint)]
        verdict = ('INCOMPATIBLE_WORLD_OVERLAP' if counts[0] < required else
                   'UNRESOLVED_MEASUREMENTS' if counts[1] < required else
                   'COMPATIBLE' if counts[4] >= required else 'INCOMPATIBLE_MEASUREMENTS')
        rows.append(PickupCandidateCorrespondence(index, verdict, *counts))
    base['candidates'] = tuple(rows)
    compatible = [r for r in rows if r.verdict == 'COMPATIBLE']
    if len(compatible) > 1:
        return result('AMBIGUOUS_REPORTED_CANDIDATES')
    if any(r.verdict == 'UNRESOLVED_MEASUREMENTS' for r in rows):
        return result('UNRESOLVED_REPORTED_CANDIDATE')
    if len(compatible) != 1:
        return result('NO_UNIQUE_MEASURED_CORRESPONDENCE')
    if compatible[0].index != inputs.selected_index:
        return result('SELECTED_CANDIDATE_NOT_MATCHED')
    return result('CONDITIONAL_UNIQUE_STATIONARY_MATCH', True)
