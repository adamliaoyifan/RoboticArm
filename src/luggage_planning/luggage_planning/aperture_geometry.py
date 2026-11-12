"""Opening-plane proposals; geometric fit is never motion permission.

Coordinates here are container-absolute, including Z. The conservative payload
already owns its uncertainty. This module does not add a process clearance.
"""
from dataclasses import dataclass
from itertools import product
import math

import numpy as np
from scipy.spatial.transform import Rotation

from luggage_planning.placement_prior_contracts import Transform


def box_vertices(dimensions):
    dimensions = np.asarray(dimensions, dtype=float)
    if dimensions.shape != (3,) or not np.all(np.isfinite(dimensions)) or np.any(dimensions <= 0):
        raise ValueError('three positive box dimensions required')
    return np.asarray(tuple(product((-1., 1.), repeat=3)))*dimensions/2


def transform_vertices(transform, vertices):
    return (Rotation.from_quat(transform.quaternion_xyzw).apply(vertices)
            + np.asarray(transform.translation))


@dataclass(frozen=True)
class ApertureRegion:
    lower_uv: tuple
    upper_uv: tuple
    outside_n: float
    inside_n: float
    proposal_kind: str

    @property
    def empty(self):
        return any(a >= b for a, b in zip(self.lower_uv, self.upper_uv))


class ApertureGeometry:
    def __init__(self, corners, inward_normal):
        points = np.asarray(corners, dtype=float)
        normal = np.asarray(inward_normal, dtype=float)
        if (points.shape != (4, 3) or normal.shape != (3,)
                or not np.all(np.isfinite(points)) or not np.all(np.isfinite(normal))
                or not np.isclose(np.linalg.norm(normal), 1., atol=1e-9)):
            raise ValueError('finite rectangular aperture and unit inward normal required')
        vertical = np.array((0., 0., 1.))
        if abs(normal @ vertical) > 1e-9:
            raise ValueError('vertical opening plane required')
        self.origin = points.mean(axis=0)
        # Columns are inward, lateral, up. The handedness is explicit.
        self.axes = np.column_stack((normal, np.cross(vertical, normal), vertical))
        coordinates = (points-self.origin) @ self.axes
        if np.max(np.abs(coordinates[:, 0])) > 1e-8:
            raise ValueError('coplanar aperture corners required')
        self.lower = coordinates[:, 1:].min(axis=0)
        self.upper = coordinates[:, 1:].max(axis=0)
        if np.any(self.upper-self.lower <= 0):
            raise ValueError('nonzero aperture area required')
        actual = sorted(map(tuple, np.round(coordinates[:, 1:], 8)))
        expected = sorted(product(*zip(np.round(self.lower, 8), np.round(self.upper, 8))))
        if actual != expected:
            raise ValueError('rectangular aperture required')

    @classmethod
    def from_scene(cls, config):
        from luggage_description.container_geometry import descriptor_from_scene_config, opening_inward_datum
        from luggage_description.scene_tf_config_utils import container_opening_aperture_corners_in_container
        corners = container_opening_aperture_corners_in_container(config, margin=0.)
        _, normal = opening_inward_datum(descriptor_from_scene_config(config), corners)
        return cls(corners, normal)

    def coordinates(self, point):
        return (np.asarray(point)-self.origin) @ self.axes

    def point(self, n, uv):
        return tuple(self.origin+self.axes @ np.asarray((n, *uv)))

    def region(self, geometry, container_from_tool, *, rigid_vertices=()):
        envelope = geometry.conservative
        tool_from_box = Transform(envelope.t_tcp_box_translation, envelope.t_tcp_box_quaternion_xyzw)
        vertices = transform_vertices(tool_from_box, box_vertices(envelope.dimensions_wdh))
        kind = 'CONSERVATIVE_PAYLOAD'
        if len(rigid_vertices):
            vertices = np.concatenate((vertices, np.asarray(rigid_vertices)))
            kind = 'PAYLOAD_AND_FIXED_TOOL_PROJECTION'
        rotation = Rotation.from_quat(container_from_tool.quaternion_xyzw)
        projected = rotation.apply(vertices) @ self.axes
        # Include the contact origin in the normal transit extent, even for an
        # eccentric grasp whose entire box lies to one side of that origin.
        outside = -max(0., float(projected[:, 0].max()))
        inside = -min(0., float(projected[:, 0].min()))
        return ApertureRegion(tuple(self.lower-projected[:, 1:].min(axis=0)),
            tuple(self.upper-projected[:, 1:].max(axis=0)), outside, inside, kind)

    def proposals(self, region, preferred_point, count):
        if type(count) is not int or not 1 <= count <= 32:
            raise ValueError('bounded aperture proposal count required')
        if region.empty:
            return ()
        lower, upper = np.asarray(region.lower_uv), np.asarray(region.upper_uv)
        preferred = np.clip(self.coordinates(preferred_point)[1:], lower, upper)
        centre = (lower+upper)/2
        # A centre and goal-aligned proposal first, then an expanding interior
        # grid. Fractions sample geometry; they do not define physical margins.
        side = max(2, math.ceil(math.sqrt(count)))
        fractions = np.linspace(0., 1., side+2)[1:-1]
        grid = [lower+(upper-lower)*uv for uv in product(fractions, repeat=2)]
        grid.sort(key=lambda uv: float(np.linalg.norm(uv-preferred)))
        result = []
        for uv in (centre, preferred, *grid):
            if not any(np.allclose(uv, old, atol=1e-10, rtol=0) for old in result):
                result.append(uv)
            if len(result) == count:
                break
        return tuple(self.point(0., uv) for uv in result)
