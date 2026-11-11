"""Measurement-bound surface quality for task-proposed contact UV and angle.

Only directly observed cells are admitted. Pose fitting uses the existing
quality kernel over the cells actually intersected by the physical rectangle.
Refinement must reach an identical sampling mask; a capped/expired query gives
no permission. No grid-centre snapping, hole inference or quality relaxation.
"""
import copy
from dataclasses import dataclass, fields, replace
import hashlib
import json
import math
import time

import numpy as np

from luggage_perception.observed_contact_footprint import (
    check_observed_contact_footprint, project_contact_footprint,
)
from luggage_perception.suction_footprint_containment import convex_footprint_cell_window
from luggage_perception.suction_patch_evaluator import SuctionPatchEvaluator


QUALITY_METRICS = ('valid_coverage', 'mask_coverage', 'plane_coverage', 'rms_residual',
    'p95_residual', 'peak_to_valley', 'normal_deviation_p95', 'max_adjacent_step',
    'discontinuity_count')


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
        allow_nan=False).encode()).hexdigest()


@dataclass(frozen=True)
class DirectedSuctionResult:
    accepted: bool
    reason: str
    domain_sha256: str
    quality_policy_sha256: str
    samples_sha256: str
    query_uv: tuple
    yaw_rad: float
    # Row-major 4x4 world-from-suction_contact_frame; absent on refusal.
    world_from_contact_frame: tuple = ()
    metrics: tuple = ()
    refinement_count: int = 0
    evidence_sha256: str = ''

    def evidence_digest(self):
        """Canonical wire witness digest, with the digest field itself empty."""
        return _digest(replace(self, evidence_sha256='').__dict__)


def _contact_pose(domain, uv, yaw, center=None, normal=None):
    outward = np.asarray(domain.normal_world if normal is None else normal)
    direction = (math.cos(yaw)*np.asarray(domain.u_axis_world)
                 + math.sin(yaw)*np.asarray(domain.v_axis_world))
    x_axis = direction - (direction @ outward)*outward
    x_axis /= np.linalg.norm(x_axis)
    result = np.eye(4)
    result[:3, :3] = np.column_stack((x_axis, -np.cross(outward, x_axis), -outward))
    result[:3, 3] = domain.world_point(uv) if center is None else center
    return result


@dataclass(frozen=True, init=False)
class DirectedSuctionContext:
    """Owned immutable sampling snapshot plus the actual effective policy.

    Construct through the evaluator's directed_quality_context(detection) so
    the observed support and samples come from the same acquisition. The file
    model hash alone does not cover runtime quality overrides; policy_sha256
    deliberately covers every effective model field as well.
    """

    domain: object
    domain_sha256: str
    policy_sha256: str
    samples_sha256: str
    _model: object
    _height_map: object

    def __init__(self, domain, height_map, contact_model):
        if (height_map is None or height_map.shape != domain.shape
                or (height_map.origin_u, height_map.origin_v) != domain.origin_uv_top_m
                or height_map.cell_size_m != domain.cell_size_m
                or contact_model.cell_size_m != domain.cell_size_m
                or contact_model.identity_hash != domain.contact_model_hash
                or contact_model.model_version != domain.contact_model_version
                or height_map.plane_d != domain.plane_offset_world_m
                or not np.array_equal(height_map.u_axis, domain.u_axis_world)
                or not np.array_equal(height_map.v_axis, domain.v_axis_world)
                or not np.array_equal(height_map.normal, domain.normal_world)):
            raise ValueError('same-acquisition grid, basis and physical model required')
        object.__setattr__(self, 'domain', domain)
        object.__setattr__(self, 'domain_sha256', domain.digest())
        object.__setattr__(self, '_model', copy.deepcopy(contact_model))
        object.__setattr__(self, 'policy_sha256', _digest(self._model.as_dict()))
        object.__setattr__(self, '_height_map', copy.copy(height_map))
        samples_hash = hashlib.sha256(b'directed-suction-samples-v1\0')
        for field in fields(height_map):
            value = getattr(height_map, field.name)
            if isinstance(value, np.ndarray):
                # An immutable bytes buffer prevents callers re-enabling writes.
                value = np.frombuffer(value.tobytes(), dtype=value.dtype).reshape(value.shape)
                setattr(self._height_map, field.name, value)
                samples_hash.update(field.name.encode()+b'\0'+value.dtype.str.encode())
                samples_hash.update(str(value.shape).encode()+value.tobytes())
        samples_hash.update(self.domain_sha256.encode())
        object.__setattr__(self, 'samples_sha256', samples_hash.hexdigest())

    def evaluate(self, uv, yaw_rad, *, deadline, max_refinements, clock=time.monotonic):
        """Fit a directed contact without changing requested top-plane UV.

        max_refinements is a caller-owned compute bound, never a geometric or
        missing-depth allowance. yaw defines a direction in the observed basis,
        projected onto the fitted tangent plane to obtain the contact-frame X.
        """
        uv = tuple(float(v) for v in uv)
        yaw = float(yaw_rad)
        if (len(uv) != 2 or not all(math.isfinite(v) for v in uv)
                or not math.isfinite(yaw) or not math.isfinite(deadline)
                or type(max_refinements) is not int or max_refinements < 1):
            raise ValueError('finite query/deadline and positive refinement budget required')

        def result(accepted, reason, count, pose=(), metrics=()):
            value = DirectedSuctionResult(accepted, reason, self.domain_sha256,
                self.policy_sha256, self.samples_sha256, uv, yaw, pose, metrics, count)
            return replace(value, evidence_sha256=value.evidence_digest())

        pose = _contact_pose(self.domain, uv, yaw)
        evaluator = SuctionPatchEvaluator(self._model)
        for iteration in range(max_refinements):
            if clock() >= deadline:
                return result(False, 'QUALITY_DEADLINE', iteration)
            coverage = check_observed_contact_footprint(self.domain, self._model, pose)
            if not coverage.contained:
                return result(False, coverage.reason, iteration)
            window = self._window(pose)
            local, mask, half = self._sample_window(window)
            sec, ns = divmod(self.domain.stamp_ns, 10**9)
            records, rejects = evaluator._evaluate_batch(local,
                np.array([half[0]]), np.array([half[1]]), np.zeros(1),
                float(sec)+float(ns)*1e-9, 'world', self.domain.instance_id,
                self.domain.generation, window_half_shape=half,
                footprint_mask=mask[None], centers_uv=np.array([uv]))
            if clock() >= deadline:
                return result(False, 'QUALITY_DEADLINE', iteration+1)
            if not records:
                return result(False, rejects[0][2] if rejects else 'QUALITY_NO_SAMPLES', iteration+1)
            record = records[0]
            fitted = _contact_pose(self.domain, uv, yaw, record.center_world, record.normal_world)
            coverage = check_observed_contact_footprint(self.domain, self._model, fitted)
            if not coverage.contained:
                return result(False, coverage.reason, iteration+1)
            next_window = self._window(fitted)
            if (next_window.first_ij == window.first_ij
                    and np.array_equal(next_window.membership, window.membership)):
                if clock() >= deadline:
                    return result(False, 'QUALITY_DEADLINE', iteration+1)
                metrics = tuple((name, getattr(record, name)) for name in QUALITY_METRICS)
                return result(True, 'DIRECTED_CONTACT_QUALITY_OK', iteration+1,
                    tuple(float(v) for v in fitted.ravel()), metrics)
            pose = fitted
        return result(False, 'QUALITY_REFINEMENT_INCOMPLETE', max_refinements)

    def _window(self, pose):
        vertices = project_contact_footprint(self.domain, self._model, pose,
            include_boundary_margin=False)
        if vertices is None:
            raise ValueError('nondegenerate physical contact projection required')
        window = convex_footprint_cell_window(self.domain.shape,
            origin_uv=self.domain.origin_uv_top_m, cell_size_m=self.domain.cell_size_m,
            vertices_uv=vertices)
        if window.reason != 'ok' or not window.membership.any():
            raise ValueError('physical footprint has no complete sampling window')
        return window

    def _sample_window(self, window):
        """Crop/pad unknown outside cells; no clipping or support creation."""
        i, j = window.first_ij
        ni, nj = window.membership.shape
        pad = ((0, int(ni % 2 == 0)), (0, int(nj % 2 == 0)))
        updates = {}
        for name, fill in (('valid_count', 0), ('median_h', np.nan), ('smooth_h', np.nan),
                           ('plane_label', 0), ('mask_member', False), ('local_normal', np.nan)):
            source = getattr(self._height_map, name)[i:i+ni, j:j+nj]
            padding = pad if source.ndim == 2 else pad+((0, 0),)
            updates[name] = np.pad(source, padding, constant_values=fill)
        mask = np.pad(window.membership, pad, constant_values=False)
        updates.update(origin_u=self.domain.origin_uv_top_m[0]+i*self.domain.cell_size_m,
            origin_v=self.domain.origin_uv_top_m[1]+j*self.domain.cell_size_m,
            nu=mask.shape[0], nv=mask.shape[1])
        local = replace(self._height_map, **updates)
        return local, mask, ((mask.shape[0]-1)//2, (mask.shape[1]-1)//2)
