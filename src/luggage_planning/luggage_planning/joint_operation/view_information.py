"""Compare new measurements on the original physical contact footprint.

Sample gain is observation evidence only. It never fills a support grid or
replaces independent new-acquisition contact quality and trajectory admission.
"""
from dataclasses import asdict, dataclass
import hashlib
import json

import numpy as np

from luggage_perception.stationary_pickup_association import (
    StationaryPickupAssociation, StationaryPickupPolicy, measure_pickup_world_targets,
)
from luggage_planning.joint_operation.pickup_observation import PickupObservationNeed


@dataclass(frozen=True)
class PickupRegionInformation:
    before_view_sha256: str
    after_view_sha256: str
    association_sha256: str
    policy_sha256: str
    query_id: str
    footprint_world: tuple
    targets_world: tuple
    before_measured: tuple
    after_measured: tuple
    gained_samples: int
    lost_samples: int

    def digest(self):
        return hashlib.sha256(json.dumps(asdict(self), sort_keys=True,
            allow_nan=False, separators=(',', ':')).encode()).hexdigest()


def compare_pickup_region(need, before_view, after_view, association, policy, *, sample_shape):
    if (type(need) is not PickupObservationNeed
            or type(association) is not StationaryPickupAssociation
            or type(policy) is not StationaryPickupPolicy
            or association.accepted is not True
            or association.reason != 'CONDITIONAL_UNIQUE_STATIONARY_MATCH'):
        raise ValueError('ACCEPTED_MEASURED_ASSOCIATION_REQUIRED')
    before_digest, after_digest = before_view.digest(), after_view.digest()
    if ((need.domain_sha256, need.view_sha256, before_view.domain_sha256,
            after_view.domain_sha256, before_digest, after_digest, policy.digest()) !=
            (association.before_domain_sha256, association.before_view_sha256,
             association.before_domain_sha256, association.after_domain_sha256,
             association.before_view_sha256, association.after_view_sha256,
             association.policy_sha256)):
        raise ValueError('ORIGINAL_REGION_ASSOCIATION_BINDING_MISMATCH')
    shape = tuple(sample_shape)
    if (len(shape) != 2 or any(type(n) is not int or n < 1 for n in shape)
            or shape[0]*shape[1] > 4096):
        raise ValueError('EXPLICIT_BOUNDED_REGION_SAMPLE_GRID_REQUIRED')
    corners = np.asarray(need.footprint_world, dtype=float).reshape(4, 3)
    if not np.isfinite(corners).all():
        raise ValueError('FINITE_ORIGINAL_FOOTPRINT_REQUIRED')
    # Midpoint quadrature stays on the original quad, regardless of camera pose
    # or changes to either acquisition's support-grid size or world origin.
    targets = []
    for u in (np.arange(shape[0])+.5)/shape[0]:
        for v in (np.arange(shape[1])+.5)/shape[1]:
            targets.append((1-u)*(1-v)*corners[0]+u*(1-v)*corners[1]
                           +u*v*corners[2]+(1-u)*v*corners[3])
    def measured(view):
        values = measure_pickup_world_targets(view, targets)
        return (values['in_region'] & values['depth_valid'] &
                (values['depth_residual_m'] <= policy.depth_residual_m))
    before, after = measured(before_view), measured(after_view)
    return PickupRegionInformation(before_digest, after_digest, association.digest(), policy.digest(),
        need.query_id, tuple(need.footprint_world), tuple(float(v) for v in np.asarray(targets).ravel()),
        tuple(bool(v) for v in before), tuple(bool(v) for v in after),
        int(np.count_nonzero(after & ~before)), int(np.count_nonzero(before & ~after)))
