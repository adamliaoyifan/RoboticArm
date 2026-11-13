"""ROS-free MoveIt validity adapter contract for ST09-C.

The integration task wires this policy to MoveIt.  The module here keeps the
certifier deterministic and testable: callers provide immutable validity
reports keyed by edge/sample, and missing reports fail closed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Tuple

from luggage_planning.planning_contract._base import ContractViolation, finite, nonempty_str
from luggage_planning.planning_contract.reasons import Reason


@dataclass(frozen=True)
class ValiditySample:
    """One whole-robot validity sample.

    ``support_contact`` is the only contact class that may be exempted, and
    only when ``support_id`` equals the intended target support.
    """

    edge_id: str
    sample_index: int
    valid: bool
    min_clearance_m: float
    contacts: Tuple[Mapping[str, object], ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        object.__setattr__(self, "edge_id", nonempty_str(self.edge_id, "edge_id"))
        if isinstance(self.sample_index, bool) or not isinstance(self.sample_index, int) or self.sample_index < 0:
            raise ContractViolation("sample_index: non-negative int required")
        if type(self.valid) is not bool:
            raise ContractViolation("valid: bool required")
        object.__setattr__(self, "min_clearance_m", finite(self.min_clearance_m, "min_clearance_m"))
        if not isinstance(self.contacts, tuple):
            raise ContractViolation("contacts: immutable tuple required")


class MoveItValidityAdapter:
    """Interface consumed by :class:`TrajectoryCertifier`."""

    def validate_sample(self, edge_id: str, sample_index: int) -> ValiditySample:
        raise NotImplementedError


class StaticMoveItValidityAdapter(MoveItValidityAdapter):
    """Fixture adapter with explicit reports for every requested sample."""

    def __init__(self, samples: Mapping[Tuple[str, int], ValiditySample]):
        self._samples = dict(samples)
        self.requests = []

    def validate_sample(self, edge_id: str, sample_index: int) -> ValiditySample:
        edge_id = nonempty_str(edge_id, "edge_id")
        self.requests.append((edge_id, sample_index))
        sample = self._samples.get((edge_id, sample_index))
        if sample is None:
            raise ContractViolation("%s: missing validity sample %s[%d]" % (
                Reason.CERT_UNKNOWN_INPUT.value, edge_id, sample_index))
        return sample


__all__ = ["MoveItValidityAdapter", "StaticMoveItValidityAdapter", "ValiditySample"]
