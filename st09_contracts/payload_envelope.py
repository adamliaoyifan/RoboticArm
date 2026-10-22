"""Continuous suction-pivot yaw envelope; no sensor, ROS or truth inputs.

For a corner (x,y,z), rotating about tool Z gives
x'=x*cos(a)-y*sin(a), y'=x*sin(a)+y*cos(a). Each component's
extrema occur at interval endpoints or derivative zeros. A linear functional
of a cuboid attains its extrema at a corner, so enumerating these critical
angles for every expanded corner encloses the entire continuous family.
Translation uncertainty is added afterwards in tool axes (Minkowski sum).
"""
from dataclasses import dataclass, asdict
from itertools import product
import math

from ._base import ContractViolation, digest_of, finite_tuple, positive_dimensions, unit_quaternion


@dataclass(frozen=True)
class EnvelopeLimits:
    yaw_half_range_deg: float
    size_margin_per_face_m: tuple
    position_half_range_tool_m: tuple

    def __post_init__(self):
        yaw = self.yaw_half_range_deg
        if isinstance(yaw, bool) or not isinstance(yaw, (int, float)) or not math.isfinite(yaw) or not 0 <= yaw <= 180:
            raise ContractViolation('PAYLOAD_ENVELOPE_CONFIG_INVALID: yaw must be in [0,180] degrees')
        for name in ('size_margin_per_face_m', 'position_half_range_tool_m'):
            raw = getattr(self, name)
            if not isinstance(raw, (list, tuple)) or len(raw) != 3 or any(
                    isinstance(v, bool) or not isinstance(v, (int, float)) for v in raw):
                raise ContractViolation('PAYLOAD_ENVELOPE_CONFIG_INVALID: three numeric margins required')
            values = finite_tuple(raw, 3, name)
            if any(v < 0 for v in values):
                raise ContractViolation('PAYLOAD_ENVELOPE_CONFIG_INVALID: negative margin')
            object.__setattr__(self, name, values)


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
    if math.hypot(axis[0], axis[1]) > 1e-9:
        raise ContractViolation('PAYLOAD_ENVELOPE_UNSUPPORTED_TILT: box Z must be parallel to contact normal')
    radius = math.radians(limits.yaw_half_range_deg)
    half = [d / 2 + m for d, m in zip(dimensions, limits.size_margin_per_face_m)]
    low, high = [math.inf] * 3, [-math.inf] * 3
    for signs in product((-1, 1), repeat=3):
        corner = tuple(s * h for s, h in zip(signs, half))
        x, y, z = transform_point(translation, quaternion, corner)
        bounds = (sinusoid_bounds(x, -y, -radius, radius),
                  sinusoid_bounds(y, x, -radius, radius), (z, z))
        for i, (lo, hi) in enumerate(bounds):
            low[i] = min(low[i], lo)
            high[i] = max(high[i], hi)
    # Outward arithmetic allowance, not a physical uncertainty margin.
    for i, margin in enumerate(limits.position_half_range_tool_m):
        low[i] -= margin + 1e-12
        high[i] += margin + 1e-12
    return PayloadEnvelope(
        tuple(hi - lo for lo, hi in zip(low, high)),
        tuple((hi + lo) / 2 for lo, hi in zip(low, high)),
        (0., 0., 0., 1.), limits,
        digest_of(dimensions, translation, quaternion))


def collision_payload(attachment):
    """Geometry view only. Never substitute this for nominal target synthesis."""
    return getattr(attachment, 'collision_envelope', None) or attachment
