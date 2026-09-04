"""Reusable deterministic occupancy and candidate fixtures."""

from __future__ import division

from luggage_planning.interior_view_scorer import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    CameraIntrinsics,
    SparseOccupancyGrid,
)


def narrow_intrinsics():
    """Return a three-ray horizontal pinhole camera."""
    return CameraIntrinsics(width=3, height=1, fx=3.0, fy=3.0)


def corridor_grid(barrier=False, normals=None):
    """Return a 1 m corridor with unknown cells beyond known free space."""
    cells = {}
    for x_index in range(10):
        state = FREE if x_index < 2 else UNKNOWN
        cells[(x_index, 1, 1)] = state
        cells[(x_index, 0, 1)] = state
        cells[(x_index, 2, 1)] = state
    if barrier:
        for y_index in range(3):
            cells[(4, y_index, 1)] = OCCUPIED
    return SparseOccupancyGrid(
        origin=(0.0, -0.15, -0.15),
        shape=(10, 3, 3),
        resolution=0.1,
        cells=cells,
        normals=normals,
        default_state=FREE,
    )


def candidate(name, **overrides):
    """Build a complete feasible candidate with stable defaults."""
    value = {
        "candidate_id": name,
        "camera_xyz": (0.05, 0.0, 0.0),
        "look_at": (1.0, 0.0, 0.0),
        "camera_up": (0.0, 0.0, 1.0),
        "hard_feasible": True,
        "geometry_feasible": True,
        "ik_feasible": True,
        "collision_free": True,
        "trajectory_feasible": True,
        "corridor_confidence": 1.0,
        "depth": 0.5,
        "manipulability": 0.5,
        "joint_margin": 0.5,
        "trajectory_quality": 0.5,
        "risk": 0.0,
    }
    value.update(overrides)
    return value
