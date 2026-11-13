"""Runtime selection parameters, separate from physical support constraints."""
from dataclasses import dataclass,asdict
from pathlib import Path
import math
import yaml
from planning_types import digest


@dataclass(frozen=True)
class SelectionPolicy:
    clearance_precision_m: float
    collision_tie_m: float
    distance_numeric_error_m: float
    placement_dedup_translation_m: float
    grasp_dedup_translation_m: float
    handoff_dedup_translation_m: float
    push_dedup_translation_m: float
    pose_dedup_rotation_deg: float

    def __post_init__(self):
        if (not all(math.isfinite(v) and v>0 for v in asdict(self).values()) or
            self.clearance_precision_m>self.collision_tie_m/4 or
            self.distance_numeric_error_m>self.clearance_precision_m/10):
            raise ValueError('INVALID_SELECTION_PRECISION_OR_TIE_BAND')

    @property
    def revision(self):return digest(asdict(self))

    @property
    def clearance_revision(self):
        # Ranking/dedup changes do not invalidate unchanged geometry evidence.
        return digest(dict(clearance_precision_m=self.clearance_precision_m,
                           distance_numeric_error_m=self.distance_numeric_error_m))

    @classmethod
    def load(cls,path=None):
        return cls(**yaml.safe_load(Path(path or Path(__file__).with_name('selection_policy.yaml')).read_text()))
