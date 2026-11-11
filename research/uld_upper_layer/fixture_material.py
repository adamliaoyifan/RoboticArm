"""Explicit simulation-only contact material; never a controller feedback input."""
from dataclasses import dataclass
from pathlib import Path
import math
import yaml

DEFAULT_PROFILE=Path(__file__).with_suffix('.yaml')


@dataclass(frozen=True)
class BoxContactMaterial:
    primary_friction: float
    secondary_friction: float
    first_friction_direction_local: tuple | None

    def __post_init__(self):
        for value in (self.primary_friction,self.secondary_friction):
            if not math.isfinite(value) or value<0:raise ValueError('INVALID_FIXTURE_FRICTION')
        direction=self.first_friction_direction_local
        if direction is not None and (len(direction)!=3 or not all(math.isfinite(x) for x in direction)
                or not math.isclose(sum(x*x for x in direction),1.,abs_tol=1e-9)):
            raise ValueError('FRICTION_DIRECTION_MUST_BE_MATERIAL_FRAME_UNIT_VECTOR')

    @classmethod
    def load(cls,path=None):
        values=yaml.safe_load(Path(path or DEFAULT_PROFILE).read_text())
        if values['first_friction_direction_local'] is not None:
            values['first_friction_direction_local']=tuple(values['first_friction_direction_local'])
        return cls(**values)
