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
    path_simplification_budget_s: float
    contact_penetration_m: float
    support_contact_stroke_m: float
    push_contact_stroke_m: float
    contact_normal_alignment_deg: float
    sweep_step_m: float
    cartesian_step_m: float
    push_speed_mps: float
    push_acceleration_mps2: float
    contact_speed_mps: float
    joint_speed_scaling: float
    joint_acceleration_radps2: float
    ik_seed_count: int
    precontact_distance_m: float
    pick_lift_m: float
    outside_aperture_margin_m: float
    ompl_range_rad: float
    ompl_resolution_fraction: float
    distance_query_timeout_s: float
    push_face_spins_deg: tuple = (0.,90.,180.,270.)

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
                     'path_attempt_budget_s','sweep_step_m','cartesian_step_m',
                     'push_speed_mps','push_acceleration_mps2','contact_speed_mps',
                     'joint_speed_scaling','joint_acceleration_radps2','precontact_distance_m',
                     'pick_lift_m','outside_aperture_margin_m','ompl_resolution_fraction',
                     'distance_query_timeout_s','ompl_range_rad'):
            if getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive')
        for name in ('solution_count_target', 'minimum_executable_solutions', 'max_action_candidates','ik_seed_count'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f'{name} must be positive integer')
        for name in ('support_contact_stroke_m','push_contact_stroke_m'):
            if getattr(self,name) > self.contact_penetration_m:
                raise ValueError(name+' exceeds modeled contact penetration')
        if self.contact_normal_alignment_deg>=90:
            raise ValueError('contact_normal_alignment_deg must be below 90')
        if self.minimum_executable_solutions > self.solution_count_target:
            raise ValueError('minimum exceeds target')

    @property
    def revision(self):
        return digest(asdict(self))

    @classmethod
    def load(cls, path=None, **overrides):
        values = yaml.safe_load(Path(path or Path(__file__).with_name('upper_config.yaml')).read_text())
        values.update(overrides)
        for key in ('box_yaws_deg', 'tool_yaws_deg','push_face_spins_deg'):
            if key in values:values[key] = tuple(values[key])
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
    support_contact_tool_pose: tuple  # bounded gravity stroke, not the settled box reference
    handoff_support: SupportMetrics
    push_start_tool_pose: tuple
    push_end_tool_pose: tuple
    push_start_control_tool_pose: tuple  # nominal panel contact plus bounded acquisition stroke
    push_end_control_tool_pose: tuple
    push_direction: tuple
    push_distance_m: float
    tool_yaw_deg: float
    tool_T_push_surface: tuple  # seal TCP -> actual source-CAD rigid push face
    push_contact_geometry_id: str
    checks: tuple  # explicitly distinguish geometry from unperformed arm checks


def action_from_record(value):
    """Strict immutable reconstruction of a persisted, version-matched action."""
    value=dict(value);placement=dict(value.pop('placement'))
    def support(row):
        row=dict(row);row['source_ids']=tuple(row['source_ids'])
        return SupportMetrics(**row)
    placement['support']=support(placement['support'])
    placement['final_box_pose']=tuple(placement['final_box_pose'])
    value['placement']=PlacementCandidate(**placement)
    value['handoff_support']=support(value['handoff_support'])
    for key in ('box_T_tool','pick_tool_pose','handoff_box_pose','handoff_tool_pose','support_contact_tool_pose',
                'push_start_tool_pose','push_end_tool_pose','push_start_control_tool_pose',
                'push_end_control_tool_pose','push_direction',
                'tool_T_push_surface','checks'):
        value[key]=tuple(value[key])
    return ActionCandidate(**value)
