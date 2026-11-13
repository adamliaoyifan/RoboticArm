"""Explicit simulation-only depth loss workload; no box or planner inputs."""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class DepthDropoutConfig:
    model_hash: str
    scene_revision: str
    reference_observation: str
    reference_input_sha256: str
    world_frame: str
    optical_frame: str
    reference_axis_world: tuple
    pitch_pixels: int
    square_side_pixels: int
    active_axis_cone_rad: float

    @classmethod
    def from_record(cls, record):
        if record.get('schema') != 'simulation_depth_dropout_v1':
            raise ValueError('DEPTH_DROPOUT_SCHEMA')
        config = cls(**{key: record[key] for key in cls.__dataclass_fields__})
        for name in ('model_hash', 'scene_revision', 'reference_observation',
                     'world_frame', 'optical_frame'):
            if not isinstance(getattr(config, name), str) or not getattr(config, name).strip():
                raise ValueError('DEPTH_DROPOUT_IDENTITY:' + name)
        digest = config.reference_input_sha256
        if not isinstance(digest, str) or len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest):
            raise ValueError('DEPTH_DROPOUT_REFERENCE_HASH')
        for value in (config.pitch_pixels, config.square_side_pixels):
            if type(value) is not int or value <= 0:
                raise ValueError('DEPTH_DROPOUT_GRID')
        if config.square_side_pixels >= config.pitch_pixels:
            raise ValueError('DEPTH_DROPOUT_GRID')
        if (isinstance(config.active_axis_cone_rad, bool) or
                not math.isfinite(config.active_axis_cone_rad) or
                not 0 < config.active_axis_cone_rad < math.pi):
            raise ValueError('DEPTH_DROPOUT_CONE')
        axis = np.asarray(config.reference_axis_world, dtype=float)
        if axis.shape != (3,) or not np.isfinite(axis).all() or abs(np.linalg.norm(axis)-1.) > 1e-6:
            raise ValueError('DEPTH_DROPOUT_REFERENCE_AXIS')
        return config


def optical_axis_world(quaternion):
    """World direction of local optical +Z; camera roll does not change it."""
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1.) > 1e-6:
        raise ValueError('DEPTH_DROPOUT_TF_QUATERNION')
    x, y, z, w = q
    return np.array((2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)))


def apply_dropout(depth_mm, config, axis_world):
    """Return new bytes and measured effect, never add or synthesize depth."""
    depth = np.asarray(depth_mm)
    axis = np.asarray(axis_world, dtype=float)
    if depth.ndim != 2 or depth.dtype != np.dtype('<u2'):
        raise ValueError('DEPTH_DROPOUT_CANONICAL_DEPTH')
    if axis.shape != (3,) or not np.isfinite(axis).all() or abs(np.linalg.norm(axis)-1.) > 1e-6:
        raise ValueError('DEPTH_DROPOUT_AXIS')
    angle = math.acos(float(np.clip(np.dot(axis, config.reference_axis_world), -1., 1.)))
    active = angle <= config.active_axis_cone_rad
    output = depth.copy()
    changed = 0
    if active:
        rows = np.arange(depth.shape[0]) % config.pitch_pixels < config.square_side_pixels
        cols = np.arange(depth.shape[1]) % config.pitch_pixels < config.square_side_pixels
        mask = rows[:, None] & cols[None, :]
        changed = int(np.count_nonzero(depth[mask]))
        output[mask] = 0
    return output, dict(active=active, reason='ACTIVE_CONE' if active else 'OUTSIDE_CONE',
                        axis_world=axis.tolist(), angle_rad=angle, changed_pixels=changed)
