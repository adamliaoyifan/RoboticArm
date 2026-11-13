"""Observation-only safeguards for independent physical release experiments.

No force feedback or payload commands. Native contact checks run on every
physics step; geometry/support checks use the configured monitor period.
Passing this monitor cannot admit an arm plan or certify an error envelope.
"""
from dataclasses import dataclass, fields
from pathlib import Path
import math
import numpy as np
import yaml

from execution_monitor import pose_error
from observed_support import measure_observed_support
from support_contact import support_contact_failure


@dataclass(frozen=True)
class ReleaseMotionPolicy:
    tracking_translation_m: float
    tracking_rotation_deg: float
    endpoint_joint_error_m: float
    native_max_gap_s: float

    def __post_init__(self):
        for f in fields(self):
            v = getattr(self, f.name)
            if isinstance(v, bool) or not math.isfinite(v) or v <= 0:
                raise ValueError('INVALID_RELEASE_MOTION_POLICY:' + f.name)

    @classmethod
    def load(cls, path=None):
        return cls(**yaml.safe_load(Path(path or Path(__file__).with_name('release_motion_policy.yaml')).read_text()))


def motion_reference(motion, elapsed_s):
    """Interpolate the actual sampled commands; not an independent box model."""
    t = motion.time_from_start_s
    if not math.isfinite(elapsed_s):
        raise ValueError('INVALID_RELEASE_REFERENCE_TIME')
    elapsed = min(t[-1], max(0., elapsed_s))
    q = np.asarray(motion.joint_displacements_xz_m)
    positions = np.array([np.interp(elapsed, t, q[:, j]) for j in (0, 1)])
    tool = np.asarray(motion.container_T_tool_start).reshape(4, 4).copy()
    tool[[0, 2], 3] += positions - q[0]
    return tuple(positions), tuple(tool.ravel())


class ReleaseExecutionMonitor:
    def __init__(self, initial, dimensions, config, policy, motion_policy, scene):
        self._initial = initial; self._size = tuple(dimensions)
        self._config = config; self._policy = policy; self._motion_policy = motion_policy
        self._scene = scene; self._last = None; self._last_geometry = None
        self._native_support_points = 0; self._samples = 0; self._geometry_samples = 0
        self._max_gap = 0.; self._box_translation = self._box_rotation = 0.
        self._lower_translation = self._lower_rotation = 0.
        self._tracking_translation = self._tracking_rotation = 0.
        self._minimum_clearance = float('inf'); self._failure = None

    @property
    def failure(self):
        return self._failure

    def _fail(self, sample, reason, **details):
        if self._failure is None:
            self._failure = dict(stamp_s=sample.stamp_s, reason=reason, **details)
        return self._failure

    def observe(self, sample, *, detached, reference_tool, separating_interface):
        if self._failure:
            return self._failure
        if self._last is not None:
            gap = sample.stamp_s-self._last
            self._max_gap = max(self._max_gap, gap)
            if gap <= 0 or gap > self._motion_policy.native_max_gap_s:
                return self._fail(sample, 'RELEASE_NATIVE_GAP', gap_s=gap)
        self._last = sample.stamp_s; self._samples += 1
        if not sample.tool_contact_components_complete:
            return self._fail(sample, 'RELEASE_TOOL_CONTACTS_INCOMPLETE')
        if sample.tool_contacts:
            return self._fail(sample, 'RELEASE_UNEXPECTED_TOOL_CONTACT')
        if (sample.attachment_create_count != 1 or sample.attachment_detach_count > 1
                or (detached and (sample.attached or sample.constraint_present or sample.attachment_detach_count != 1))):
            return self._fail(sample, 'RELEASE_ATTACHMENT_STATE_INVALID')
        bodies = {b.name+'::body::collision': b for b in sample.lower}
        for c in sample.contacts:
            b = bodies.get(c.other_collision)
            if b is None:
                return self._fail(sample, 'RELEASE_UNCLASSIFIED_PAYLOAD_CONTACT', other=c.other_collision)
            half = np.array(b.dimensions)/2
            error = support_contact_failure(sample.container_T_box, self._size, b.container_T_body,
                [-half, half], c.point_container, c.normal_toward_payload_container,
                self._config.contact_tolerance_m, self._config.contact_penetration_m,
                min(self._config.contact_normal_alignment_deg, self._policy.contact_normal_alignment_deg),
                c.depth_m, normal_toward_box=True)
            if error:
                return self._fail(sample, 'RELEASE_INVALID_SUPPORT_CONTACT', diagnostic=error, body=b.name)
            self._native_support_points += 1
        # Check all native poses, not just a 50Hz subset: brief excursions matter.
        bt, br = pose_error(self._initial.container_T_box, sample.container_T_box)
        self._box_translation = max(self._box_translation, bt); self._box_rotation = max(self._box_rotation, br)
        if bt > self._policy.stability_translation_m or br > self._policy.stability_rotation_deg:
            return self._fail(sample, 'RELEASE_BOX_UNSTABLE', translation_m=bt, rotation_deg=br)
        old = {b.name: b for b in self._initial.lower}
        if set(old) != {b.name for b in sample.lower}:
            return self._fail(sample, 'RELEASE_LOWER_SET_CHANGED')
        for b in sample.lower:
            if b.dimensions != old[b.name].dimensions:
                return self._fail(sample, 'RELEASE_LOWER_DIMENSIONS_CHANGED')
            lt, lr = pose_error(old[b.name].container_T_body, b.container_T_body)
            self._lower_translation = max(self._lower_translation, lt); self._lower_rotation = max(self._lower_rotation, lr)
            if lt > self._policy.lower_translation_m or lr > self._policy.lower_rotation_deg:
                return self._fail(sample, 'RELEASE_LOWER_MOVED', body=b.name, translation_m=lt, rotation_deg=lr)
        tt, tr = pose_error(reference_tool, sample.container_T_tool)
        self._tracking_translation = max(self._tracking_translation, tt); self._tracking_rotation = max(self._tracking_rotation, tr)
        if tt > self._motion_policy.tracking_translation_m or tr > self._motion_policy.tracking_rotation_deg:
            return self._fail(sample, 'RELEASE_TRACKING_ERROR', translation_m=tt, rotation_deg=tr)
        if self._last_geometry is None or sample.stamp_s-self._last_geometry >= self._policy.monitor_period_s-1e-9:
            self._last_geometry = sample.stamp_s; self._geometry_samples += 1
            support = measure_observed_support(sample, self._size, (0., 0., 0.),
                self._config.contact_tolerance_m, self._config.contact_penetration_m)
            if not sample.contacts or support.near_contact_area_m2 <= 0:
                return self._fail(sample, 'RELEASE_SUPPORT_MISSING')
            geometry = self._scene.evaluate_observation(sample, separating_interface=separating_interface)
            if geometry.get('failure'):
                return self._fail(sample, 'RELEASE_OBSERVED_GEOMETRY_FAILED', diagnostic=geometry)
            self._minimum_clearance = min(self._minimum_clearance, geometry['minimum_m'])
        return None

    def metrics(self):
        return dict(native_samples=self._samples, geometry_samples=self._geometry_samples,
            native_support_points=self._native_support_points, max_native_gap_s=self._max_gap,
            max_box_translation_m=self._box_translation, max_box_rotation_deg=self._box_rotation,
            max_lower_translation_m=self._lower_translation, max_lower_rotation_deg=self._lower_rotation,
            max_tracking_translation_m=self._tracking_translation, max_tracking_rotation_deg=self._tracking_rotation,
            observed_minimum_clearance_m=self._minimum_clearance if math.isfinite(self._minimum_clearance) else None,
            failure=self._failure, execution_admitted=False,
            scope='independent_release_experiment_observed_coverage_not_continuous_error_envelope')
