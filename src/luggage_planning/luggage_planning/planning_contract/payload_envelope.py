"""Continuous suction-pivot yaw envelope; no sensor, ROS or truth inputs.

For a corner (x,y,z), rotating about tool Z gives
x'=x*cos(a)-y*sin(a), y'=x*sin(a)+y*cos(a). Each component's
extrema occur at interval endpoints or derivative zeros. A linear functional
of a cuboid attains its extrema at a corner, so enumerating these critical
angles for every expanded corner encloses the entire continuous family.
Translation uncertainty is added afterwards in tool axes (Minkowski sum).
For nonparallel box/contact axes, height and away-position allowances extend
both ways: a fitted normal alone does not identify a one-sided contact plane.
This enclosure does not certify contact quality or suction support.
"""
from dataclasses import dataclass, asdict
from itertools import product
import math

from ._base import ContractViolation, digest_of, finite_tuple, positive_dimensions, unit_quaternion


@dataclass(frozen=True)
class EnvelopeLimits:
    yaw_half_range_deg: float
    size_growth_fraction_wdh: tuple
    position_half_range_tool_xy_m: tuple
    away_from_contact_position_error_m: float

    def __post_init__(self):
        yaw = self.yaw_half_range_deg
        if isinstance(yaw, bool) or not isinstance(yaw, (int, float)) or not math.isfinite(yaw) or not 0 <= yaw <= 180:
            raise ContractViolation('PAYLOAD_ENVELOPE_CONFIG_INVALID: yaw must be in [0,180] degrees')
        object.__setattr__(self, 'yaw_half_range_deg', float(yaw))
        for name, length in (('size_growth_fraction_wdh', 3),
                             ('position_half_range_tool_xy_m', 2)):
            raw = getattr(self, name)
            if not isinstance(raw, (list, tuple)) or len(raw) != length or any(
                    isinstance(v, bool) or not isinstance(v, (int, float)) for v in raw):
                raise ContractViolation('PAYLOAD_ENVELOPE_CONFIG_INVALID: %d numeric values required for %s' % (length, name))
            values = finite_tuple(raw, length, name)
            if any(v < 0 for v in values):
                raise ContractViolation('PAYLOAD_ENVELOPE_CONFIG_INVALID: negative margin')
            if name == 'size_growth_fraction_wdh' and any(v > 1 for v in values):
                raise ContractViolation('PAYLOAD_ENVELOPE_CONFIG_INVALID: size growth fraction exceeds 1')
            object.__setattr__(self, name, values)
        error = self.away_from_contact_position_error_m
        if isinstance(error, bool) or not isinstance(error, (int, float)) or not math.isfinite(error) or error < 0:
            raise ContractViolation('PAYLOAD_ENVELOPE_CONFIG_INVALID: invalid away-from-contact position error')
        object.__setattr__(self, 'away_from_contact_position_error_m', float(error))


@dataclass(frozen=True)
class PayloadEnvelope:
    dimensions_wdh: tuple
    t_tcp_box_translation: tuple
    t_tcp_box_quaternion_xyzw: tuple
    limits: EnvelopeLimits
    nominal_digest: str

    def to_dict(self):
        return asdict(self)

    def digest(self):
        return digest_of('tool-pivot-yaw-envelope-v1', self.to_dict())


def sinusoid_bounds(a, b, lower, upper):
    """Exact candidate extrema of a*cos(theta)+b*sin(theta)."""
    angles = [lower, upper]
    phase = math.atan2(b, a)
    first = math.ceil((lower - phase) / math.pi)
    last = math.floor((upper - phase) / math.pi)
    angles.extend(phase + k * math.pi for k in range(first, last + 1))
    values = [a * math.cos(t) + b * math.sin(t) for t in angles]
    return min(values), max(values)


def build_envelope(dimensions, translation, quaternion, limits):
    from .attachment import rotate_vector, transform_point

    dimensions = positive_dimensions(dimensions, 'dimensions_wdh')
    translation = finite_tuple(translation, 3, 't_tool_box.translation')
    quaternion = unit_quaternion(quaternion, 't_tool_box.quaternion')
    axis = rotate_vector(quaternion, (0., 0., 1.))
    # Retain the existing numerical parallelism tolerance, not a physical
    # acceptance angle. Surface-quality gates own admissible contact normals.
    parallel_axes = math.hypot(axis[0], axis[1]) <= 1e-9
    radius = math.radians(limits.yaw_half_range_deg)
    growth = limits.size_growth_fraction_wdh
    half_xy = [dimensions[i] * (1 + growth[i]) / 2 for i in range(2)]
    z_low, z_high = -dimensions[2] / 2, dimensions[2] / 2
    away_tool_sign = None
    if parallel_axes:
        z_faces = [transform_point(translation, quaternion,
                   (0., 0., sign * dimensions[2] / 2))[2] for sign in (-1, 1)]
        # Existing parallel contact construction and archive identity remain
        # unchanged, including its measured face-plane arithmetic tolerance.
        z_faces = [0. if abs(value) <= 1e-6 else value for value in z_faces]
        if (z_faces[0] * z_faces[1] < 0 or
                abs(abs(z_faces[0]) - abs(z_faces[1])) <= 1e-9):
            raise ContractViolation('PAYLOAD_ENVELOPE_CONTACT_FACE_AMBIGUOUS: cannot identify face away from suction origin')
        away_face_index = 0 if abs(z_faces[0]) > abs(z_faces[1]) else 1
        near_face_index = 1 - away_face_index
        away_local_sign = -1 if away_face_index == 0 else 1
        away_tool_sign = (-1 if z_faces[away_face_index] < z_faces[near_face_index] else 1)
    else:
        # A tilted top may cross tool Z=0. Face-centre distances in tool Z
        # cannot identify the contact side for an eccentric grasp. Enclose
        # either one-sided growth without inventing a face/angle prior.
        away_local_sign = None
    if away_local_sign is None or away_local_sign < 0:
        z_low -= dimensions[2] * growth[2]
    if away_local_sign is None or away_local_sign > 0:
        z_high += dimensions[2] * growth[2]
    low, high = [math.inf] * 3, [-math.inf] * 3
    for sx, sy, z in product((-1, 1), (-1, 1), (z_low, z_high)):
        corner = (sx * half_xy[0], sy * half_xy[1], z)
        x, y, z = transform_point(translation, quaternion, corner)
        bounds = (sinusoid_bounds(x, -y, -radius, radius),
                  sinusoid_bounds(y, x, -radius, radius), (z, z))
        for i, (lo, hi) in enumerate(bounds):
            low[i] = min(low[i], lo)
            high[i] = max(high[i], hi)
    # Outward arithmetic allowance, not a physical uncertainty margin.
    for i, margin in enumerate(limits.position_half_range_tool_xy_m):
        low[i] -= margin + 1e-12
        high[i] += margin + 1e-12
    if away_tool_sign is None or away_tool_sign < 0:
        low[2] -= limits.away_from_contact_position_error_m + 1e-12
    if away_tool_sign is None or away_tool_sign > 0:
        high[2] += limits.away_from_contact_position_error_m + 1e-12
    return PayloadEnvelope(
        tuple(hi - lo for lo, hi in zip(low, high)),
        tuple((hi + lo) / 2 for lo, hi in zip(low, high)),
        # Tool-aligned cuboid; +Z stays in the nominal box-Z hemisphere.
        # With tilt this is not the nominal box orientation. Consumers must
        # transform its full pose, including when converting solver targets.
        ((0., 0., 0., 1.) if axis[2] > 0 else (1., 0., 0., 0.)), limits,
        digest_of(dimensions, translation, quaternion))


def collision_payload(attachment):
    """Geometry view only. Never substitute this for nominal target synthesis."""
    return getattr(attachment, 'collision_envelope', None) or attachment
