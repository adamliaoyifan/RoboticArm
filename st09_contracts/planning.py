"""Planning-side frozen value objects.

``PlanningSnapshot`` / ``PayloadIdentity`` / ``PlacementCandidate`` /
``ObservationRequirement`` per parallel plan section 3 item 4.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping, Optional, Sequence, Tuple

from st09_contracts._base import (
    ContractViolation,
    digest_of,
    digest_str,
    finite,
    finite_tuple,
    nonempty_str,
    positive_dimensions,
    stamp_ns,
    unit_quaternion,
)


class OccupancyCellState(str, Enum):
    """Cargo-map cell states. ``UNKNOWN`` never promotes to ``FREE``."""

    FREE = "free"
    OCCUPIED = "occupied"
    UNKNOWN = "unknown"

    def __str__(self) -> str:  # pragma: no cover - trivial
        return self.value


@dataclass(frozen=True)
class PlanningSnapshot:
    """Immutable map + geometry state a planning decision was made against.

    Arrays are immutable tuples; a snapshot is never mutated in place —
    integration produces a new snapshot with a new revision.
    """

    geometry_id: str
    geometry_hash: str
    map_revision: int
    acquisition_stamp_ns: int
    commit_revision: int
    occupancy_digest: str
    resolution_m: float
    origin_xy: Tuple[float, float]
    cells_i: Tuple[int, ...]
    cells_j: Tuple[int, ...]
    cell_states: Tuple[str, ...]
    #: [ST09-I S5.7-6] Scene identity the snapshot was planned against
    #: (RuntimeIdentityV2.scene_revision). Empty on legacy constructions
    #: (the identity gates compare only what both sides declare — an
    #: empty snapshot scene never blocks a legacy producer). Excluded
    #: from digest(): the digest pins the MAP content, not the cell's
    #: identity binding.
    scene_revision: str = ""
    #: ``static_empty`` / ``measured`` / ``release_integrated``. Empty
    #: keeps the legacy acquisition-age check. Excluded from digest().
    map_source: str = ""
    #: Committed placed boxes. ``static_empty`` is legal only at 0.
    placed_count: int = 0
    #: True after a settled cargo view has been integrated. ``static_empty``
    #: is then no longer a legal identity.
    cargo_observed: bool = False
    #: True only when ``cell_states`` is the whole decision domain.
    #: A sparse snapshot that omitted unknown cells is not a full
    #: container, even if every listed cell is occupied. Excluded from
    #: digest(): it is an admission declaration, not map content.
    occupancy_domain_complete: bool = False
    #: Occupied-cell heights. ``geometry_hash`` is the container hull and
    #: ``occupancy_digest`` is cell state only, so two maps can share a
    #: revision and a hull hash while column tops differ. Empty means
    #: the source grid had no heights. Excluded from digest().
    column_height_digest: str = ""
    #: False after a cargo change that has not been checked by an
    #: observation. Age does not clear it. Excluded from digest().
    occupancy_confirmed: bool = True
    #: ``box_placed``, ``box_removed``, ``box_disturbed``,
    #: ``cargo_edited``, or ``container_pose_changed``. Empty when the
    #: snapshot is still the confirmed one. Excluded from digest().
    change_event: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "geometry_id", nonempty_str(self.geometry_id, "geometry_id"))
        object.__setattr__(self, "geometry_hash", digest_str(self.geometry_hash, "geometry_hash"))
        for name in ("map_revision", "commit_revision"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ContractViolation("%s: non-negative int required (%r)" % (name, value))
        object.__setattr__(self, "acquisition_stamp_ns", stamp_ns(self.acquisition_stamp_ns, "acquisition_stamp_ns"))
        source = str(self.map_source or "")
        if source not in ("", "static_empty", "measured", "release_integrated"):
            raise ContractViolation(
                "map_source: unknown identity %r" % source)
        object.__setattr__(self, "map_source", source)
        placed = self.placed_count
        if isinstance(placed, bool) or not isinstance(placed, int) or placed < 0:
            raise ContractViolation(
                "placed_count: non-negative int required (%r)" % placed)
        if not isinstance(self.cargo_observed, bool):
            raise ContractViolation(
                "cargo_observed: bool required (%r)" % self.cargo_observed)
        if not isinstance(self.occupancy_domain_complete, bool):
            raise ContractViolation(
                "occupancy_domain_complete: bool required (%r)"
                % self.occupancy_domain_complete)
        object.__setattr__(self, "resolution_m", finite(self.resolution_m, "resolution_m"))
        if self.resolution_m <= 0.0:
            raise ContractViolation("resolution_m: non-positive (%r)" % self.resolution_m)
        object.__setattr__(self, "origin_xy", finite_tuple(self.origin_xy, 2, "origin_xy"))
        for name in ("cells_i", "cells_j", "cell_states"):
            if not isinstance(getattr(self, name), tuple):
                raise ContractViolation("%s: immutable tuple required" % name)
        n = len(self.cells_i)
        if not (n == len(self.cells_j) == len(self.cell_states)):
            raise ContractViolation("cell arrays length mismatch")
        for i, state in enumerate(self.cell_states):
            try:
                OccupancyCellState(state)
            except ValueError as exc:
                raise ContractViolation("cell_states[%d]: unknown state %r" % (i, state)) from exc

    @property
    def cell_count(self) -> int:
        return len(self.cells_i)

    def occupancy_counts(self) -> Mapping[str, int]:
        counts = {state.value: 0 for state in OccupancyCellState}
        for state in self.cell_states:
            counts[state] += 1
        return counts

    def state_at(self, i: int, j: int) -> OccupancyCellState:
        for index, (ci, cj) in enumerate(zip(self.cells_i, self.cells_j)):
            if ci == i and cj == j:
                return OccupancyCellState(self.cell_states[index])
        return OccupancyCellState.UNKNOWN

    def digest(self) -> str:
        """Stable digest over geometry, revision and the cell arrays."""
        return digest_of(
            self.geometry_id, self.geometry_hash, self.map_revision,
            self.commit_revision, self.resolution_m, self.origin_xy,
            self.cells_i, self.cells_j, self.cell_states)

    def to_dict(self) -> Mapping[str, Any]:
        return {
            "geometry_id": self.geometry_id,
            "geometry_hash": self.geometry_hash,
            "map_revision": self.map_revision,
            "acquisition_stamp_ns": self.acquisition_stamp_ns,
            "map_source": self.map_source,
            "placed_count": self.placed_count,
            "cargo_observed": self.cargo_observed,
            "occupancy_domain_complete": self.occupancy_domain_complete,
            "column_height_digest": self.column_height_digest,
            "occupancy_confirmed": self.occupancy_confirmed,
            "change_event": self.change_event,
            "commit_revision": self.commit_revision,
            "occupancy_digest": self.occupancy_digest,
            "resolution_m": self.resolution_m,
            "origin_xy": list(self.origin_xy),
            "cells_i": list(self.cells_i),
            "cells_j": list(self.cells_j),
            "cell_states": list(self.cell_states),
        }


class PayloadSymmetry(str, Enum):
    """Declared payload symmetry (technical baseline 5.1).

    ``PI`` means the payload model declares a 180-degree footprint
    symmetry; theta and theta+pi are then two distinct physical headings
    with the same footprint, and each still gets its own TCP target via
    the actual attachment. ``NONE`` forbids the theta+pi expansion.
    """

    NONE = "none"
    PI = "pi"

    def __str__(self) -> str:  # pragma: no cover - trivial
        return self.value


@dataclass(frozen=True)
class PayloadIdentity:
    """The box being carried: measured identity + frozen attachment.

    ``dimensions_wdh`` are recorded along the fixed box axes; they are
    never re-sorted by length. ``t_tcp_box`` is the full rigid transform
    (translation + unit quaternion) that maps box coordinates into TCP
    coordinates; see :mod:`st09_contracts.attachment`.
    """

    cargo_id: str
    box_id: str
    request_id: str
    cycle_id: str
    dimensions_wdh: Tuple[float, float, float]
    symmetry: PayloadSymmetry
    t_tcp_box_translation: Tuple[float, float, float]
    t_tcp_box_quaternion_xyzw: Tuple[float, float, float, float]
    attachment_incarnation: int
    acquisition_stamp_ns: int

    def __post_init__(self) -> None:
        for name in ("cargo_id", "box_id", "request_id", "cycle_id"):
            object.__setattr__(self, name, nonempty_str(getattr(self, name), name))
        object.__setattr__(
            self, "dimensions_wdh", positive_dimensions(self.dimensions_wdh, "dimensions_wdh"))
        if not isinstance(self.symmetry, PayloadSymmetry):
            raise ContractViolation("symmetry: PayloadSymmetry required (%r)" % (self.symmetry,))
        object.__setattr__(
            self, "t_tcp_box_translation",
            finite_tuple(self.t_tcp_box_translation, 3, "t_tcp_box_translation"))
        object.__setattr__(
            self, "t_tcp_box_quaternion_xyzw",
            unit_quaternion(self.t_tcp_box_quaternion_xyzw, "t_tcp_box_quaternion_xyzw"))
        if isinstance(self.attachment_incarnation, bool) or not isinstance(self.attachment_incarnation, int) \
                or self.attachment_incarnation < 0:
            raise ContractViolation(
                "attachment_incarnation: non-negative int required (%r)" % (self.attachment_incarnation,))
        object.__setattr__(
            self, "acquisition_stamp_ns", stamp_ns(self.acquisition_stamp_ns, "acquisition_stamp_ns"))

    def footprint_classes(self) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """Container-frame footprint classes (theta=0 and theta=pi/2).

        Purely geometric enumeration used by placement; the heading
        expansion respects :attr:`symmetry` separately.
        """
        w, d, _ = self.dimensions_wdh
        return ((w, d), (d, w))

    def digest(self) -> str:
        return digest_of(
            self.cargo_id, self.box_id, self.request_id, self.cycle_id,
            self.dimensions_wdh, str(self.symmetry),
            self.t_tcp_box_translation, self.t_tcp_box_quaternion_xyzw,
            self.attachment_incarnation, self.acquisition_stamp_ns)


class GeometryVerdict(str, Enum):
    """Placement geometry verdicts (A owns the kernel, not the enum)."""

    LEGAL = "legal"
    HULL_VIOLATION = "hull_violation"
    APERTURE_VIOLATION = "aperture_violation"
    SUPPORT_MISSING = "support_missing"
    SUPPORT_TILT_UNSUPPORTED = "support_tilt_unsupported"
    OVERLAP = "overlap"
    SAME_LAYER_CLEARANCE = "same_layer_clearance"
    TOP_CLEARANCE = "top_clearance"

    def __str__(self) -> str:  # pragma: no cover - trivial
        return self.value


@dataclass(frozen=True)
class PlacementCandidate:
    """One legal-or-rejected box target in container coordinates."""

    candidate_id: str
    footprint_class: int
    heading_theta: float
    position_container: Tuple[float, float, float]
    quaternion_xyzw_container: Tuple[float, float, float, float]
    support_layer_z: float
    depth_m: float
    verdict: GeometryVerdict
    score: float
    detail: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "candidate_id", nonempty_str(self.candidate_id, "candidate_id"))
        object.__setattr__(self, "footprint_class", int(self.footprint_class))
        if self.footprint_class not in (0, 1):
            raise ContractViolation("footprint_class: 0 or 1 (%r)" % (self.footprint_class,))
        object.__setattr__(self, "heading_theta", finite(self.heading_theta, "heading_theta"))
        object.__setattr__(
            self, "position_container",
            finite_tuple(self.position_container, 3, "position_container"))
        object.__setattr__(
            self, "quaternion_xyzw_container",
            unit_quaternion(self.quaternion_xyzw_container, "quaternion_xyzw_container"))
        object.__setattr__(self, "support_layer_z", finite(self.support_layer_z, "support_layer_z"))
        object.__setattr__(self, "depth_m", finite(self.depth_m, "depth_m"))
        if not isinstance(self.verdict, GeometryVerdict):
            raise ContractViolation("verdict: GeometryVerdict required (%r)" % (self.verdict,))
        object.__setattr__(self, "score", finite(self.score, "score"))
        if not isinstance(self.detail, Mapping):
            raise ContractViolation("detail: mapping required")

    @property
    def legal(self) -> bool:
        return self.verdict is GeometryVerdict.LEGAL

    def sorting_key(self) -> tuple:
        """Deterministic deep-first key (support layer asc, depth desc).

        B applies the same key to route-bearing candidates (parallel plan
        section 4 ST09-A): layer up, depth bucket deep-first, then route
        clearance, frozen placement score, time/joint travel, stable id.
        """
        return (round(self.support_layer_z, 6), -round(self.depth_m, 6),
                -float(self.detail.get("route_clearance_m", 0.0)),
                -self.score, self.candidate_id)


@dataclass(frozen=True)
class ObservationRequirement:
    """What must still be observed before a candidate area is decidable.

    Contains only map-space requests — never ground truth. ``missing``
    explains why each requested cell cannot be decided from the current
    snapshot.
    """

    requirement_id: str
    candidate_ids: Tuple[str, ...]
    footprint_cells: Tuple[Tuple[int, int], ...]
    corridor_cells: Tuple[Tuple[int, int], ...]
    missing_cells: Tuple[Tuple[int, int], ...]
    missing_reasons: Tuple[str, ...]
    note: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "requirement_id", nonempty_str(self.requirement_id, "requirement_id"))
        for name in ("candidate_ids", "footprint_cells", "corridor_cells",
                     "missing_cells", "missing_reasons"):
            if not isinstance(getattr(self, name), tuple):
                raise ContractViolation("%s: immutable tuple required" % name)
        if len(self.missing_cells) != len(self.missing_reasons):
            raise ContractViolation("missing_cells/missing_reasons length mismatch")
        for reason in self.missing_reasons:
            nonempty_str(reason, "missing_reasons")
        for cells in ("footprint_cells", "corridor_cells", "missing_cells"):
            for cell in getattr(self, cells):
                finite_tuple(cell, 2, cells)

    @property
    def satisfiable(self) -> bool:
        return bool(self.missing_cells)

    def digest(self) -> str:
        return digest_of(
            self.requirement_id, self.candidate_ids, self.footprint_cells,
            self.corridor_cells, self.missing_cells, self.missing_reasons)
