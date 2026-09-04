"""ROS-free exploration policy contracts for production cargo loading."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Iterable, Mapping, Optional, Protocol, Tuple


SCHEMA_VERSION = 1
PlainValue = object
FrozenItems = Tuple[Tuple[str, PlainValue], ...]


class ViewOutcomeStatus(str, Enum):
    ACCEPTED = "accepted"
    REJECTED = "rejected"
    EXECUTED = "executed"
    INTEGRATED = "integrated"


def _freeze_mapping(values: Optional[Mapping[str, PlainValue]]) -> FrozenItems:
    if not values:
        return ()
    return tuple(sorted((str(key), value) for key, value in values.items()))


def _plain_mapping(values: FrozenItems) -> Dict[str, PlainValue]:
    return {key: value for key, value in values}


@dataclass(frozen=True)
class ExplorationContext:
    """Static per-session inputs owned by the exploration coordinator."""

    frames: Mapping[str, PlainValue]
    geometry_descriptor: Mapping[str, PlainValue]
    geometry_hash: str
    camera_model: Mapping[str, PlainValue]
    policy_config: Mapping[str, PlainValue] = field(default_factory=dict)
    budgets: Mapping[str, PlainValue] = field(default_factory=dict)
    schema_version: int = SCHEMA_VERSION

    def __post_init__(self) -> None:
        object.__setattr__(self, "frames", _freeze_mapping(self.frames))
        object.__setattr__(
            self, "geometry_descriptor", _freeze_mapping(self.geometry_descriptor)
        )
        object.__setattr__(self, "camera_model", _freeze_mapping(self.camera_model))
        object.__setattr__(self, "policy_config", _freeze_mapping(self.policy_config))
        object.__setattr__(self, "budgets", _freeze_mapping(self.budgets))

    def to_dict(self) -> Dict[str, PlainValue]:
        return {
            "schema_version": self.schema_version,
            "frames": _plain_mapping(self.frames),
            "geometry_descriptor": _plain_mapping(self.geometry_descriptor),
            "geometry_hash": self.geometry_hash,
            "camera_model": _plain_mapping(self.camera_model),
            "policy_config": _plain_mapping(self.policy_config),
            "budgets": _plain_mapping(self.budgets),
        }


@dataclass(frozen=True)
class ExplorationSnapshot:
    """Immutable online map/view snapshot supplied to a policy."""

    acquisition_stamp_start: float
    acquisition_stamp_end: float
    map_revision: int
    occupancy_summary: Mapping[str, PlainValue]
    visibility_summary: Mapping[str, PlainValue]
    robot_state: Mapping[str, PlainValue]
    visited_candidate_ids: Iterable[str] = ()
    diagnostics: Mapping[str, PlainValue] = field(default_factory=dict)
    schema_version: int = SCHEMA_VERSION

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "occupancy_summary", _freeze_mapping(self.occupancy_summary)
        )
        object.__setattr__(
            self, "visibility_summary", _freeze_mapping(self.visibility_summary)
        )
        object.__setattr__(self, "robot_state", _freeze_mapping(self.robot_state))
        object.__setattr__(
            self,
            "visited_candidate_ids",
            tuple(str(candidate_id) for candidate_id in self.visited_candidate_ids),
        )
        object.__setattr__(self, "diagnostics", _freeze_mapping(self.diagnostics))

    def to_dict(self) -> Dict[str, PlainValue]:
        return {
            "schema_version": self.schema_version,
            "acquisition_stamp_start": self.acquisition_stamp_start,
            "acquisition_stamp_end": self.acquisition_stamp_end,
            "map_revision": self.map_revision,
            "occupancy_summary": _plain_mapping(self.occupancy_summary),
            "visibility_summary": _plain_mapping(self.visibility_summary),
            "robot_state": _plain_mapping(self.robot_state),
            "visited_candidate_ids": list(self.visited_candidate_ids),
            "diagnostics": _plain_mapping(self.diagnostics),
        }


@dataclass(frozen=True)
class CandidateView:
    """Plain candidate sensor pose proposed by a policy."""

    candidate_id: str
    frame_id: str
    position_xyz: Tuple[float, float, float]
    orientation_xyzw: Tuple[float, float, float, float]
    score: float = 0.0
    diagnostics: Mapping[str, PlainValue] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "position_xyz", tuple(self.position_xyz))
        object.__setattr__(self, "orientation_xyzw", tuple(self.orientation_xyzw))
        object.__setattr__(self, "diagnostics", _freeze_mapping(self.diagnostics))

    def to_dict(self) -> Dict[str, PlainValue]:
        return {
            "candidate_id": self.candidate_id,
            "frame_id": self.frame_id,
            "position_xyz": list(self.position_xyz),
            "orientation_xyzw": list(self.orientation_xyzw),
            "score": self.score,
            "diagnostics": _plain_mapping(self.diagnostics),
        }


@dataclass(frozen=True)
class PolicyProposal:
    """Policy output before coordinator hard-gate validation."""

    policy_id: str
    proposal_schema_id: str
    candidates: Iterable[CandidateView] = ()
    done_recommended: bool = False
    done_reason: str = ""
    diagnostics: Mapping[str, PlainValue] = field(default_factory=dict)
    schema_version: int = SCHEMA_VERSION

    def __post_init__(self) -> None:
        object.__setattr__(self, "candidates", tuple(self.candidates))
        object.__setattr__(self, "diagnostics", _freeze_mapping(self.diagnostics))

    def to_dict(self) -> Dict[str, PlainValue]:
        return {
            "schema_version": self.schema_version,
            "policy_id": self.policy_id,
            "proposal_schema_id": self.proposal_schema_id,
            "candidates": [candidate.to_dict() for candidate in self.candidates],
            "done_recommended": self.done_recommended,
            "done_reason": self.done_reason,
            "diagnostics": _plain_mapping(self.diagnostics),
        }


@dataclass(frozen=True)
class ViewOutcome:
    """Coordinator-owned result for a proposed or executed view."""

    status: ViewOutcomeStatus
    reason_code: str
    candidate_id: str = ""
    acquisition_stamp: float = 0.0
    prior_map_revision: int = -1
    resulting_map_revision: int = -1
    diagnostics: Mapping[str, PlainValue] = field(default_factory=dict)
    schema_version: int = SCHEMA_VERSION

    def __post_init__(self) -> None:
        object.__setattr__(self, "diagnostics", _freeze_mapping(self.diagnostics))

    def to_dict(self) -> Dict[str, PlainValue]:
        return {
            "schema_version": self.schema_version,
            "status": self.status.value,
            "reason_code": self.reason_code,
            "candidate_id": self.candidate_id,
            "acquisition_stamp": self.acquisition_stamp,
            "prior_map_revision": self.prior_map_revision,
            "resulting_map_revision": self.resulting_map_revision,
            "diagnostics": _plain_mapping(self.diagnostics),
        }


class ExplorationPolicy(Protocol):
    """Replaceable policy boundary; implementations must not call ROS."""

    policy_id: str

    def reset(self, context: ExplorationContext) -> None:
        ...

    def propose(
        self, context: ExplorationContext, snapshot: ExplorationSnapshot
    ) -> PolicyProposal:
        ...

    def observe(self, outcome: ViewOutcome) -> None:
        ...
