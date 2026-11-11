"""Immutable, ROS-free boundary types for the independent upper action.

All poses map child coordinates into container_link, whose Z is absolute.
The box origin is its geometric centre. Tuples avoid mutable numpy aliases.
"""
from dataclasses import asdict, dataclass, fields
import hashlib
import json
import math
from pathlib import Path
import yaml


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


@dataclass(frozen=True)
class UpperConfig:
    support_area_ratio_min: float
    support_com_margin_m: float
    collision_margin_m: float
    contact_tolerance_m: float
    grid_resolution_m: float
    placement_step_m: float
    handoff_step_m: float
    grasp_step_m: float
    push_height_step_m: float
    withdraw_m: float
    release_drop_m: float
    placement_lift_m: float
    box_yaws_deg: tuple
    tool_yaws_deg: tuple
    solution_count_target: int
    minimum_executable_solutions: int
    max_action_candidates: int
    search_budget_s: float
    path_attempt_budget_s: float

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, tuple):
                if not value or not all(math.isfinite(v) for v in value):
                    raise ValueError(f'invalid {field.name}')
            elif not math.isfinite(value) or value < 0:
                raise ValueError(f'invalid {field.name}')
        if not 0 < self.support_area_ratio_min <= 1:
            raise ValueError('support_area_ratio_min outside (0, 1]')
        for name in ('grid_resolution_m', 'placement_step_m', 'handoff_step_m',
                     'grasp_step_m', 'push_height_step_m', 'search_budget_s',
                     'path_attempt_budget_s'):
            if getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive')
        for name in ('solution_count_target', 'minimum_executable_solutions', 'max_action_candidates'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f'{name} must be positive integer')
        if self.minimum_executable_solutions > self.solution_count_target:
            raise ValueError('minimum exceeds target')

    @property
    def revision(self):
        return digest(asdict(self))

    @classmethod
    def load(cls, path=None, **overrides):
        values = yaml.safe_load(Path(path or Path(__file__).with_name('upper_config.yaml')).read_text())
        values.update(overrides)
        for key in ('box_yaws_deg', 'tool_yaws_deg'):
            values[key] = tuple(values[key])
        return cls(**values)


@dataclass(frozen=True)
class OccupiedBox:
    name: str
    center: tuple
    size: tuple


@dataclass(frozen=True)
class SurfacePatch:
    source_id: str
    bounds_xy: tuple  # xmin, ymin, xmax, ymax
    surface_z_container: float


@dataclass(frozen=True)
class SceneSnapshot:
    snapshot_id: str
    geometry_hash: str
    frame_id: str
    source_kind: str
    grid_origin: tuple
    grid_shape: tuple
    resolution_m: float
    occupied: tuple
    surfaces: tuple
    observed_bounds: tuple
    floor_z_container: float
    ceiling_z_container: float


@dataclass(frozen=True)
class BoxState:
    box_id: str
    dimensions: tuple
    container_T_box: tuple
    com_local: tuple
    com_source: str = 'geometric_center_assumption'


@dataclass(frozen=True)
class SupportMetrics:
    contact_area_m2: float
    contact_area_ratio: float
    contact_wkt: str
    support_polygon_wkt: str
    com_margin_m: float  # negative outside
    source_ids: tuple
    accepted: bool
    reason: str


@dataclass(frozen=True)
class PlacementCandidate:
    placement_id: str
    snapshot_id: str
    config_revision: str
    final_box_pose: tuple
    support: SupportMetrics


@dataclass(frozen=True)
class ActionCandidate:
    action_id: str
    placement: PlacementCandidate
    box_T_tool: tuple
    pick_tool_pose: tuple
    handoff_box_pose: tuple
    handoff_tool_pose: tuple
    handoff_support: SupportMetrics
    push_start_tool_pose: tuple
    push_end_tool_pose: tuple
    push_direction: tuple
    push_distance_m: float
    tool_yaw_deg: float
    checks: tuple  # explicitly distinguish geometry from unperformed arm checks
