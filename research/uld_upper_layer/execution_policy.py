"""Externally loaded immutable physical experiment policy, never force feedback."""
from dataclasses import dataclass,asdict,fields
from pathlib import Path
import math
import yaml
from planning_types import digest


@dataclass(frozen=True)
class ExecutionPolicy:
    sim_budget_s: float
    wall_budget_s: float
    monitor_period_s: float
    monitor_max_gap_s: float
    observation_max_age_s: float
    contact_dwell_s: float
    condition_wait_budget_s: float
    stability_dwell_s: float
    stability_translation_m: float
    stability_rotation_deg: float
    lower_translation_m: float
    lower_rotation_deg: float
    push_lateral_error_m: float
    push_rotation_error_deg: float
    no_progress_window_s: float
    no_progress_tool_advance_m: float
    no_progress_box_advance_m: float
    final_translation_m: float
    final_rotation_deg: float
    initial_joint_error_rad: float
    initial_rotation_deg: float
    attachment_rotation_error_deg: float
    contact_normal_alignment_deg: float
    trigger_pre_s: float
    trigger_post_s: float
    max_case_bytes: int
    failure_capture_reserve_bytes: int
    controller_tracking_error_rad: float = .01
    failure_capture_lag_s: float = 1.

    push_edge_transition_enabled: bool = False
    push_edge_point_tolerance_m: float = .0001
    push_edge_penetration_m: float = .00005
    push_edge_event_band_m: float = .01
    push_edge_max_duration_s: float = 1.

    def __post_init__(self):
        for field in fields(self):
            value=getattr(self,field.name)
            if field.name=='push_edge_transition_enabled':
                if type(value) is not bool:raise ValueError('INVALID_EDGE_MODE_ENABLE')
                continue
            if isinstance(value,bool) or not math.isfinite(value) or value<=0:
                raise ValueError('INVALID_EXECUTION_POLICY:'+field.name)
        if self.failure_capture_reserve_bytes>=self.max_case_bytes:
            raise ValueError('CAPTURE_RESERVE_EXCEEDS_CASE_BUDGET')
        if self.monitor_period_s>self.monitor_max_gap_s or self.monitor_max_gap_s>self.observation_max_age_s:
            raise ValueError('INCONSISTENT_MONITOR_PERIOD')
        if self.failure_capture_lag_s<self.observation_max_age_s:
            raise ValueError('CAPTURE_RETENTION_BELOW_OBSERVATION_AGE')

    @classmethod
    def load(cls,path=None):
        return cls(**yaml.safe_load(Path(path or Path(__file__).with_name('execution_policy.yaml')).read_text()))

    @property
    def revision(self):return digest(asdict(self))
