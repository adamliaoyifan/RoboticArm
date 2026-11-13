"""Conservative oriented-box geometry for an offline contact-action study.

Each mesh is enclosed in its own local AABB, then moved rigidly. A collision
is a conservative rejection, not proof that the source meshes intersect.
"""
from dataclasses import dataclass
from itertools import product
import numpy as np


@dataclass
class Box:
    name: str
    center: np.ndarray
    axes: np.ndarray
    half: np.ndarray

    def corners(self):
        signs = np.array(list(product((-1., 1.), repeat=3)))
        return self.center + (signs * self.half) @ self.axes.T

    def moved(self, transform):
        return Box(self.name, transform[:3, :3] @ self.center + transform[:3, 3],
                   transform[:3, :3] @ self.axes, self.half.copy())

    def record(self):
        return dict(name=self.name, center=self.center.tolist(),
                    axes=self.axes.tolist(), half=self.half.tolist())


def axis_box(name, low, high):
    low, high = np.asarray(low, float), np.asarray(high, float)
    if np.any(high <= low):
        raise ValueError(f"invalid bounds for {name}: {low}, {high}")
    return Box(name, (low + high) / 2, np.eye(3), (high - low) / 2)


def separated(a, b, clearance=0.0):
    """15-axis SAT; require a separating projection gap >= clearance.

    A positive clearance is sufficient, conservative Euclidean separation.
    At zero clearance exact face contact is allowed (not penetration).
    """
    directions = list(a.axes.T) + list(b.axes.T)
    directions += [np.cross(x, y) for x in a.axes.T for y in b.axes.T]
    delta = b.center - a.center
    for direction in directions:
        length = np.linalg.norm(direction)
        if length < 1e-10:
            continue
        direction = direction / length
        radius = np.abs(direction @ a.axes) @ a.half
        radius += np.abs(direction @ b.axes) @ b.half
        if abs(delta @ direction) - radius >= clearance - 1e-9:
            return True
    return False


def clipped_at_x(box, threshold):
    """Vertices of the box portion on/inside an aperture's x plane."""
    vertices = box.corners()
    result = [v for v in vertices if v[0] >= threshold - 1e-9]
    for i in range(8):
        for bit in (1, 2, 4):
            j = i ^ bit
            if j <= i:
                continue
            a, b = vertices[i], vertices[j]
            if (a[0] - threshold) * (b[0] - threshold) < 0:
                t = (threshold - a[0]) / (b[0] - a[0])
                result.append(a + t * (b - a))
    return np.asarray(result).reshape((-1, 3))


def pose(position, rotation=None):
    result = np.eye(4)
    result[:3, 3] = position
    if rotation is not None:
        result[:3, :3] = rotation
    return result


def handoff_interval(length, com_fraction, contact_from_front,
                     tool_forward, stability_margin, clearance):
    """Insertion d must put COM inside support while tool stays outside."""
    low = length * com_fraction + stability_margin
    high = min(length, contact_from_front - tool_forward - clearance)
    return dict(min_m=float(low), max_m=float(high), width_m=float(high-low),
                feasible=bool(high-low > 1e-6))
