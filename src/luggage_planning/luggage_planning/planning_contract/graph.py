"""Directed-graph and route value objects (B owns the search, not these).

Node primary state is the full joint solution ``q``; pose / region /
load state / IK-family labels are attributes. Two nodes with the same
TCP XYZ but different ``q`` never merge.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping, Optional, Sequence, Tuple

from luggage_planning.planning_contract._base import (
    ContractViolation,
    digest_of,
    digest_str,
    finite,
    finite_tuple,
    nonempty_str,
    stamp_ns,
    unit_quaternion,
)


class LoadState(str, Enum):
    UNLOADED = "unloaded"
    CARRYING = "carrying"

    def __str__(self) -> str:  # pragma: no cover - trivial
        return self.value


@dataclass(frozen=True)
class GraphNode:
    """One graph node: actual joint solution + FK attributes."""

    node_id: str
    q: Tuple[float, ...]
    tcp_position: Tuple[float, float, float]
    tcp_quaternion_xyzw: Tuple[float, float, float, float]
    region: str
    load_state: LoadState
    ik_family: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "node_id", nonempty_str(self.node_id, "node_id"))
        q = finite_tuple(self.q, 6, "q")
        object.__setattr__(self, "q", q)
        object.__setattr__(self, "tcp_position", finite_tuple(self.tcp_position, 3, "tcp_position"))
        object.__setattr__(
            self, "tcp_quaternion_xyzw",
            unit_quaternion(self.tcp_quaternion_xyzw, "tcp_quaternion_xyzw"))
        object.__setattr__(self, "region", nonempty_str(self.region, "region"))
        if not isinstance(self.load_state, LoadState):
            raise ContractViolation("load_state: LoadState required (%r)" % (self.load_state,))
        object.__setattr__(self, "ik_family", str(self.ik_family or ""))

    def same_state(self, other: "GraphNode") -> bool:
        """Same primary state: equal ``q`` within joint tolerance.

        Same TCP XYZ with different ``q`` is NOT the same state. The
        IK-family label is not part of the state.
        """
        return all(abs(a - b) <= 1e-9 for a, b in zip(self.q, other.q))

    def digest(self) -> str:
        return digest_of(self.node_id, self.q, self.tcp_position,
                         self.tcp_quaternion_xyzw, self.region,
                         str(self.load_state), self.ik_family)


@dataclass(frozen=True)
class TimedTrajectory:
    """Complete timed joint trajectory with joint order preserved.

    Joint order, velocities, accelerations and nanosecond times are all
    payload — adapters must not re-sort or drop them.
    """

    joint_names: Tuple[str, ...]
    positions: Tuple[Tuple[float, ...], ...]
    velocities: Tuple[Tuple[float, ...], ...]
    accelerations: Tuple[Tuple[float, ...], ...]
    time_from_start_ns: Tuple[int, ...]

    def __post_init__(self) -> None:
        names = tuple(nonempty_str(n, "joint_names") for n in self.joint_names)
        object.__setattr__(self, "joint_names", names)
        n_joints = len(names)
        n_points = None
        for name, block in (("positions", self.positions),
                            ("velocities", self.velocities),
                            ("accelerations", self.accelerations)):
            rows = []
            for i, row in enumerate(block):
                rows.append(finite_tuple(row, n_joints, "%s[%d]" % (name, i)))
            if n_points is None:
                n_points = len(rows)
            elif len(rows) != n_points:
                raise ContractViolation(
                    "%s: %d rows, expected %d" % (name, len(rows), n_points))
            object.__setattr__(self, name, tuple(rows))
        times = tuple(stamp_ns(t, "time_from_start_ns") for t in self.time_from_start_ns)
        if len(times) != n_points:
            raise ContractViolation(
                "time_from_start_ns: %d entries, expected %d" % (len(times), n_points))
        object.__setattr__(self, "time_from_start_ns", times)
        for i in range(1, len(times)):
            if times[i] <= times[i - 1]:
                raise ContractViolation(
                    "time_from_start_ns: non-monotonic at %d (%d <= %d)" % (
                        i, times[i], times[i - 1]))

    def digest(self) -> str:
        return digest_of(
            self.joint_names, self.positions, self.velocities,
            self.accelerations, self.time_from_start_ns)


@dataclass(frozen=True)
class GraphEdge:
    """One certified-or-uncertified edge between joint states."""

    edge_id: str
    source_node_id: str
    destination_node_id: str
    load_state: LoadState
    static_identity: str
    trajectory: TimedTrajectory
    validation_revision: Optional[int] = None
    attributes: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "edge_id", nonempty_str(self.edge_id, "edge_id"))
        object.__setattr__(self, "source_node_id", nonempty_str(self.source_node_id, "source_node_id"))
        object.__setattr__(
            self, "destination_node_id", nonempty_str(self.destination_node_id, "destination_node_id"))
        if not isinstance(self.load_state, LoadState):
            raise ContractViolation("load_state: LoadState required (%r)" % (self.load_state,))
        object.__setattr__(self, "static_identity", digest_str(self.static_identity, "static_identity"))
        if not isinstance(self.trajectory, TimedTrajectory):
            raise ContractViolation("trajectory: TimedTrajectory required")
        if self.validation_revision is not None and (
                isinstance(self.validation_revision, bool)
                or not isinstance(self.validation_revision, int)
                or self.validation_revision < 0):
            raise ContractViolation("validation_revision: non-negative int or None")
        if not isinstance(self.attributes, Mapping):
            raise ContractViolation("attributes: mapping required")

    def digest(self) -> str:
        return digest_of(
            self.edge_id, self.source_node_id, self.destination_node_id,
            str(self.load_state), self.static_identity,
            self.trajectory.digest(), self.validation_revision)


@dataclass(frozen=True)
class PlannedRoute:
    """One unauthorized route awaiting admission (B output)."""

    route_id: str
    request_id: str
    snapshot_digest: str
    payload_digest: str
    attachment_digest: str
    candidate_id: str
    entry_edge_ids: Tuple[str, ...]
    retreat_edge_ids: Tuple[str, ...]
    edge_digests: Tuple[str, ...]
    trajectory_digest: str
    planning_revision: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "route_id", nonempty_str(self.route_id, "route_id"))
        object.__setattr__(self, "request_id", nonempty_str(self.request_id, "request_id"))
        for name in ("snapshot_digest", "payload_digest", "attachment_digest",
                     "trajectory_digest", "planning_revision"):
            object.__setattr__(self, name, digest_str(getattr(self, name), name))
        object.__setattr__(self, "candidate_id", nonempty_str(self.candidate_id, "candidate_id"))
        for name in ("entry_edge_ids", "retreat_edge_ids", "edge_digests"):
            if not isinstance(getattr(self, name), tuple):
                raise ContractViolation("%s: immutable tuple required" % name)
            for value in getattr(self, name):
                nonempty_str(value, name)

    def digest(self) -> str:
        return digest_of(
            self.route_id, self.request_id, self.snapshot_digest,
            self.payload_digest, self.attachment_digest, self.candidate_id,
            self.entry_edge_ids, self.retreat_edge_ids, self.edge_digests,
            self.trajectory_digest, self.planning_revision)
