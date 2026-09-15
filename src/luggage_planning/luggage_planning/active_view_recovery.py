#!/usr/bin/env python3
"""ROS-free detector-miss active viewpoint recovery policy (ACTIVE-VIEW-1).

Plain-Python decision and candidate-selection slice for pre-pick recovery
when measured evidence says cargo is present but the current view cannot
produce a planner-ready detection. The policy consumes immutable measured
window summaries and returns declarative recovery decisions plus a bounded
candidate view. It never executes motion, never changes the production
0.20 cargo acceptance floor, and the 0.05 diagnostic presence hint carries
no semantic mask, cargo cloud, or pick authority.

Reuse contract (docs/plans/active_view_detector_miss_recovery.md):
- ``exploration_contracts`` for immutable values and serialization;
- ``constrained_view_planner`` for admissibility and coverage ranking;
- ``cargo_nbv_planner`` for bounded visited-candidate behavior
  (:class:`RecoveryCandidateRanker` extends :class:`CargoNBVPlanner` with
  a task-space scoring adapter; it is not a parallel implementation).

No ROS, no numpy, no wall clock: the elapsed budget is computed from
measured acquisition stamps only.
"""

from __future__ import division

import math
from dataclasses import dataclass, field
from typing import List, Mapping, Optional, Tuple

from luggage_planning.cargo_nbv_planner import CargoNBVPlanner
from luggage_planning.constrained_view_planner import (
    candidate_is_admissible,
    coverage_score,
)
from luggage_planning.container_aim_utils import look_at_quaternion
from luggage_planning.exploration_contracts import (
    SCHEMA_VERSION,
    AcquisitionStamp,
    CandidateView,
    ExplorationContext,
    ExplorationSnapshot,
    PolicyProposal,
    ViewOutcome,
    ViewOutcomeStatus,
    _freeze_mapping,
    _plain_mapping,
)

POLICY_ID = "active-view-recovery-v1"
DECISION_SCHEMA_ID = "active-view-decision-v1"
WINDOW_SCHEMA_ID = "active-view-window-v1"

RECOVERY_SUCCEEDED = "RECOVERY_SUCCEEDED"
RECOVERY_NOT_REQUIRED = "RECOVERY_NOT_REQUIRED"
NO_CARGO_EVIDENCE = "NO_CARGO_EVIDENCE"
RECOVERY_EVIDENCE_INVALID = "RECOVERY_EVIDENCE_INVALID"
RECOVERY_STATE_INVALID = "RECOVERY_STATE_INVALID"
RECOVERY_BUDGET_EXHAUSTED = "RECOVERY_BUDGET_EXHAUSTED"
RECOVERY_NO_GAIN = "RECOVERY_NO_GAIN"
RECOVERY_CANDIDATES_EXHAUSTED = "RECOVERY_CANDIDATES_EXHAUSTED"

#: Every terminal reason; the only non-terminal decision is a fresh
#: candidate proposal, which carries an empty reason string.
TERMINAL_REASONS = frozenset({
    RECOVERY_SUCCEEDED,
    RECOVERY_NOT_REQUIRED,
    NO_CARGO_EVIDENCE,
    RECOVERY_EVIDENCE_INVALID,
    RECOVERY_STATE_INVALID,
    RECOVERY_BUDGET_EXHAUSTED,
    RECOVERY_NO_GAIN,
    RECOVERY_CANDIDATES_EXHAUSTED,
})

#: The only coordinator state in which recovery is legal.
PRE_PICK_DETECT_STATE = "pre_pick_detect"

#: Input keys that leak ground truth, eval-only scoring, or sim branches.
FORBIDDEN_KEY_TOKENS = (
    "gt_", "ground_truth", "iou", "sim_mode", "fixture", "eval_only",
    "gazebo", "entity_state",
)

#: Default candidate library relative to the calibrated ``pickup_observe``
#: camera pose: (candidate_id, camera offset xyz [m], look yaw [deg]).
DEFAULT_CANDIDATES: Tuple[Tuple[str, Tuple[float, float, float], float], ...] = (
    ("lateral_left", (0.0, 0.10, 0.0), 0.0),
    ("lateral_right", (0.0, -0.10, 0.0), 0.0),
    ("raised_center", (0.0, 0.0, 0.08), 0.0),
    ("yaw_left", (0.0, 0.0, 0.0), 12.0),
    ("yaw_right", (0.0, 0.0, 0.0), -12.0),
)

_CANCEL_REASON_CODES = ("cancelled", "canceled", "operator_abort", "e_stop")


def _require_finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("%s must be a finite number" % name)
    if not math.isfinite(float(value)):
        raise ValueError("%s must be a finite number" % name)
    return float(value)


def _require_bool(value, name):
    if not isinstance(value, bool):
        raise ValueError("%s must be a boolean" % name)
    return value


def _mapping_to_dict(value):
    if isinstance(value, dict):
        return value
    if value is None:
        return {}
    return _plain_mapping(value)


def scan_forbidden_keys(value, path="window"):
    """Find GT/eval/sim leakage anywhere in a plain policy input.

    Returns a list of human-readable key paths (empty when clean).
    """
    violations = []
    if isinstance(value, Mapping):
        for key, item in value.items():
            key_text = str(key).lower()
            for token in FORBIDDEN_KEY_TOKENS:
                if token in key_text:
                    violations.append("%s.%s" % (path, key))
                    break
            violations.extend(
                scan_forbidden_keys(item, "%s.%s" % (path, key)))
    elif isinstance(value, (list, tuple)):
        for idx, item in enumerate(value):
            violations.extend(
                scan_forbidden_keys(item, "%s[%d]" % (path, idx)))
    return violations


def bbox_iou(a, b):
    """IoU of two [x1, y1, x2, y2] pixel boxes (0.0 when degenerate)."""
    if not a or not b or len(a) < 4 or len(b) < 4:
        return 0.0
    ix1 = max(float(a[0]), float(b[0]))
    iy1 = max(float(a[1]), float(b[1]))
    ix2 = min(float(a[2]), float(b[2]))
    iy2 = min(float(a[3]), float(b[3]))
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    area_a = max(0.0, float(a[2]) - float(a[0])) * max(
        0.0, float(a[3]) - float(a[1]))
    area_b = max(0.0, float(b[2]) - float(b[0])) * max(
        0.0, float(b[3]) - float(b[1]))
    union = area_a + area_b - inter
    if union <= 0.0:
        return 0.0
    return inter / union


def _rotate_yaw(vector_xyz, yaw_deg):
    """Rotate the horizontal (x, y) components about the world Z axis."""
    yaw = math.radians(yaw_deg)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    return (
        cos_y * vector_xyz[0] - sin_y * vector_xyz[1],
        sin_y * vector_xyz[0] + cos_y * vector_xyz[1],
        vector_xyz[2],
    )


def _normalize_ray(eye, target):
    vec = [float(target[i]) - float(eye[i]) for i in range(3)]
    norm = math.sqrt(sum(v * v for v in vec))
    if norm < 1e-9:
        return (0.0, 0.0, 1.0)
    return tuple(v / norm for v in vec)


@dataclass(frozen=True)
class DiagnosticHint:
    """Low-confidence cargo hint (0.05 <= conf < 0.20); no mask authority."""

    confidence: float
    bbox: Tuple[int, int, int, int]
    label: int
    centre_world_xy: Optional[Tuple[float, float]] = None

    def __post_init__(self):
        object.__setattr__(
            self, "confidence",
            _require_finite(self.confidence, "hint.confidence"))
        object.__setattr__(
            self, "bbox", tuple(int(v) for v in self.bbox))
        if len(self.bbox) != 4:
            raise ValueError("hint.bbox must have 4 values")
        if self.centre_world_xy is not None:
            object.__setattr__(self, "centre_world_xy", (
                _require_finite(self.centre_world_xy[0], "hint.centre_world_xy[0]"),
                _require_finite(self.centre_world_xy[1], "hint.centre_world_xy[1]"),
            ))

    def to_dict(self):
        return {
            "schema_version": SCHEMA_VERSION,
            "confidence": self.confidence,
            "bbox": list(self.bbox),
            "label": self.label,
            "centre_world_xy": (list(self.centre_world_xy)
                                if self.centre_world_xy is not None else None),
        }


@dataclass(frozen=True)
class AcceptedDetection:
    """Accepted production cargo detection at the unchanged 0.20 floor."""

    confidence: float
    bbox: Tuple[int, int, int, int]
    label: int
    border_margin_px: Optional[int] = None
    valid_depth_ratio: Optional[float] = None
    centre_world_xy: Optional[Tuple[float, float]] = None

    def __post_init__(self):
        object.__setattr__(
            self, "confidence",
            _require_finite(self.confidence, "detection.confidence"))
        object.__setattr__(
            self, "bbox", tuple(int(v) for v in self.bbox))
        if len(self.bbox) != 4:
            raise ValueError("detection.bbox must have 4 values")
        if self.border_margin_px is not None:
            object.__setattr__(
                self, "border_margin_px", int(self.border_margin_px))
        if self.valid_depth_ratio is not None:
            object.__setattr__(
                self, "valid_depth_ratio",
                _require_finite(
                    self.valid_depth_ratio, "detection.valid_depth_ratio"))
        if self.centre_world_xy is not None:
            object.__setattr__(self, "centre_world_xy", (
                _require_finite(self.centre_world_xy[0], "detection.centre_world_xy[0]"),
                _require_finite(self.centre_world_xy[1], "detection.centre_world_xy[1]"),
            ))

    def to_dict(self):
        return {
            "schema_version": SCHEMA_VERSION,
            "confidence": self.confidence,
            "bbox": list(self.bbox),
            "label": self.label,
            "border_margin_px": self.border_margin_px,
            "valid_depth_ratio": self.valid_depth_ratio,
            "centre_world_xy": (list(self.centre_world_xy)
                                if self.centre_world_xy is not None else None),
        }


@dataclass(frozen=True)
class ForegroundComponent:
    """Depth-foreground component summary inside the pickup workspace."""

    pixel_count: int
    area_fraction: Optional[float] = None
    height_above_support_m: Optional[float] = None
    in_workspace: bool = False
    bbox: Optional[Tuple[int, int, int, int]] = None

    def __post_init__(self):
        object.__setattr__(self, "pixel_count", int(self.pixel_count))
        if self.area_fraction is not None:
            object.__setattr__(
                self, "area_fraction",
                _require_finite(
                    self.area_fraction, "component.area_fraction"))
        if self.height_above_support_m is not None:
            object.__setattr__(
                self, "height_above_support_m",
                _require_finite(
                    self.height_above_support_m,
                    "component.height_above_support_m"))
        if self.bbox is not None:
            object.__setattr__(
                self, "bbox", tuple(int(v) for v in self.bbox))

    def to_dict(self):
        return {
            "schema_version": SCHEMA_VERSION,
            "pixel_count": self.pixel_count,
            "area_fraction": self.area_fraction,
            "height_above_support_m": self.height_above_support_m,
            "in_workspace": self.in_workspace,
            "bbox": list(self.bbox) if self.bbox is not None else None,
        }


@dataclass(frozen=True)
class CargoGeometry:
    """Cargo cloud / support-geometry status for one frame."""

    point_count: int
    top_surface_valid: bool
    support_mode: str

    def __post_init__(self):
        object.__setattr__(self, "point_count", int(self.point_count))
        object.__setattr__(
            self, "top_surface_valid",
            _require_bool(self.top_surface_valid, "geometry.top_surface_valid"))
        if not isinstance(self.support_mode, str):
            raise ValueError("geometry.support_mode must be a string")

    @property
    def planner_ready(self):
        return self.top_surface_valid and self.support_mode == "FULL_3D"

    def to_dict(self):
        return {
            "schema_version": SCHEMA_VERSION,
            "point_count": self.point_count,
            "top_surface_valid": self.top_surface_valid,
            "support_mode": self.support_mode,
            "planner_ready": self.planner_ready,
        }


@dataclass(frozen=True)
class FrameObservation:
    """One settled D555 frame of the three-frame recovery window."""

    stamp: AcquisitionStamp
    frame_id: str
    camera_model_id: str
    geometry_hash: str
    map_revision: int
    rgb_ok: bool
    depth_ok: bool
    aligned_depth_ok: bool
    exact_tf_ok: bool
    robot_settled: bool
    accepted_detections: Tuple[AcceptedDetection, ...] = ()
    diagnostic_hints: Tuple[DiagnosticHint, ...] = ()
    foreground_components: Tuple[ForegroundComponent, ...] = ()
    cargo_geometry: Optional[CargoGeometry] = None

    def __post_init__(self):
        object.__setattr__(
            self, "stamp", AcquisitionStamp.coerce(self.stamp))
        for name in ("rgb_ok", "depth_ok", "aligned_depth_ok",
                     "exact_tf_ok", "robot_settled"):
            object.__setattr__(
                self, name, _require_bool(getattr(self, name), name))
        object.__setattr__(
            self, "accepted_detections",
            tuple(d if isinstance(d, AcceptedDetection) else AcceptedDetection(d)
                  for d in self.accepted_detections))
        object.__setattr__(
            self, "diagnostic_hints",
            tuple(h if isinstance(h, DiagnosticHint) else DiagnosticHint(h)
                  for h in self.diagnostic_hints))
        object.__setattr__(
            self, "foreground_components",
            tuple(c if isinstance(c, ForegroundComponent) else ForegroundComponent(c)
                  for c in self.foreground_components))
        if self.cargo_geometry is not None and not isinstance(
                self.cargo_geometry, CargoGeometry):
            raise ValueError("frame.cargo_geometry must be CargoGeometry")

    def to_dict(self):
        return {
            "schema_version": SCHEMA_VERSION,
            "stamp": self.stamp.to_dict(),
            "frame_id": self.frame_id,
            "camera_model_id": self.camera_model_id,
            "geometry_hash": self.geometry_hash,
            "map_revision": self.map_revision,
            "rgb_ok": self.rgb_ok,
            "depth_ok": self.depth_ok,
            "aligned_depth_ok": self.aligned_depth_ok,
            "exact_tf_ok": self.exact_tf_ok,
            "robot_settled": self.robot_settled,
            "accepted_detections": [d.to_dict() for d in self.accepted_detections],
            "diagnostic_hints": [h.to_dict() for h in self.diagnostic_hints],
            "foreground_components": [
                c.to_dict() for c in self.foreground_components],
            "cargo_geometry": (self.cargo_geometry.to_dict()
                               if self.cargo_geometry is not None else None),
        }


@dataclass(frozen=True)
class RecoveryWindow:
    """Immutable measured three-frame summary evaluated by the policy."""

    state: str
    payload_attached: bool
    vacuum_commanded: bool
    cancel_requested: bool
    image_width: int
    image_height: int
    frames: Tuple[FrameObservation, ...]
    diagnostics: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self):
        for name in ("payload_attached", "vacuum_commanded", "cancel_requested"):
            object.__setattr__(
                self, name, _require_bool(getattr(self, name), name))
        object.__setattr__(self, "image_width", int(self.image_width))
        object.__setattr__(self, "image_height", int(self.image_height))
        frames = tuple(self.frames)
        if len(frames) != 3:
            raise ValueError("recovery window must contain exactly three frames")
        if not all(isinstance(f, FrameObservation) for f in frames):
            raise ValueError("recovery window frames must be FrameObservation")
        object.__setattr__(self, "frames", frames)
        object.__setattr__(self, "diagnostics", _freeze_mapping(self.diagnostics))

    def stamp_span_sec(self):
        first = self.frames[0].stamp
        last = self.frames[-1].stamp
        return float(last.sec - first.sec) + 1e-9 * float(
            last.nanosec - first.nanosec)

    def to_dict(self):
        return {
            "schema_id": WINDOW_SCHEMA_ID,
            "schema_version": SCHEMA_VERSION,
            "state": self.state,
            "payload_attached": self.payload_attached,
            "vacuum_commanded": self.vacuum_commanded,
            "cancel_requested": self.cancel_requested,
            "image_width": self.image_width,
            "image_height": self.image_height,
            "frames": [f.to_dict() for f in self.frames],
            "diagnostics": _plain_mapping(self.diagnostics),
        }

    @classmethod
    def from_mapping(cls, data):
        """Build a window from plain measured data, failing closed.

        Raises ``ValueError`` on GT/eval leakage, malformed fields, or
        non-finite numbers; callers translate that into
        ``RECOVERY_EVIDENCE_INVALID``.
        """
        if not isinstance(data, Mapping):
            raise ValueError("recovery window must be a mapping")
        violations = scan_forbidden_keys(data)
        if violations:
            raise ValueError(
                "policy input contains forbidden keys: %s"
                % ", ".join(sorted(set(violations))))
        frames = []
        for raw in data.get("frames", ()):
            geometry = raw.get("cargo_geometry")
            frames.append(FrameObservation(
                stamp=AcquisitionStamp.coerce(raw["stamp"]),
                frame_id=str(raw["frame_id"]),
                camera_model_id=str(raw["camera_model_id"]),
                geometry_hash=str(raw["geometry_hash"]),
                map_revision=int(raw["map_revision"]),
                rgb_ok=raw["rgb_ok"],
                depth_ok=raw["depth_ok"],
                aligned_depth_ok=raw["aligned_depth_ok"],
                exact_tf_ok=raw["exact_tf_ok"],
                robot_settled=raw["robot_settled"],
                accepted_detections=[
                    AcceptedDetection(
                        confidence=d["confidence"],
                        bbox=d["bbox"],
                        label=int(d.get("label", 2)),
                        border_margin_px=d.get("border_margin_px"),
                        valid_depth_ratio=d.get("valid_depth_ratio"),
                        centre_world_xy=d.get("centre_world_xy"),
                    ) for d in raw.get("accepted_detections", ())],
                diagnostic_hints=[
                    DiagnosticHint(
                        confidence=h["confidence"],
                        bbox=h["bbox"],
                        label=int(h.get("label", 2)),
                        centre_world_xy=h.get("centre_world_xy"),
                    ) for h in raw.get("diagnostic_hints", ())],
                foreground_components=[
                    ForegroundComponent(
                        pixel_count=c["pixel_count"],
                        area_fraction=c.get("area_fraction"),
                        height_above_support_m=c.get("height_above_support_m"),
                        in_workspace=bool(c.get("in_workspace", False)),
                        bbox=c.get("bbox"),
                    ) for c in raw.get("foreground_components", ())],
                cargo_geometry=(CargoGeometry(
                    point_count=geometry["point_count"],
                    top_surface_valid=geometry["top_surface_valid"],
                    support_mode=str(geometry["support_mode"]),
                ) if geometry is not None else None),
            ))
        return cls(
            state=str(data["state"]),
            payload_attached=data["payload_attached"],
            vacuum_commanded=data["vacuum_commanded"],
            cancel_requested=bool(data.get("cancel_requested", False)),
            image_width=int(data["image_width"]),
            image_height=int(data["image_height"]),
            frames=tuple(frames),
            diagnostics=data.get("diagnostics", {}),
        )


@dataclass(frozen=True)
class ActiveViewConfig:
    """Plain configuration for the recovery policy (no YAML at runtime)."""

    hint_min_confidence: float = 0.05
    production_min_confidence: float = 0.20
    hint_min_side_px: int = 16
    area_fraction_min: float = 0.005
    area_fraction_max: float = 0.80
    foreground_min_pixels: int = 500
    foreground_height_min_m: float = 0.03
    foreground_height_max_m: float = 0.80
    association_iou_min: float = 0.30
    border_margin_min_px: int = 8
    valid_depth_ratio_min: float = 0.80
    cargo_cloud_min_points: int = 500
    max_recovery_views: int = 3
    max_elapsed_sec: float = 15.0
    no_gain_confidence_delta: float = 0.02
    no_gain_ratio_delta: float = 0.05
    no_gain_streak: int = 2
    window_max_stamp_span_sec: float = 0.50
    workspace_center_xy: Tuple[float, float] = (-1.0, 0.0)
    workspace_half_xy: Tuple[float, float] = (0.5, 0.5)
    workspace_margin_m: float = 0.10
    workspace_plane_z: float = 0.97
    nominal_camera_xyz: Tuple[float, float, float] = (-1.0, 0.0, 1.9)
    camera_z_max: float = 2.05
    camera_xy_radius_max: float = 1.20
    coverage_radius: float = 0.9
    alignment_min: float = 0.2
    coverage_weight: float = 1.0
    gain_weight: float = 1.0
    path_weight: float = 2.0
    smooth_weight: float = 0.0
    candidates: Tuple[Tuple[str, Tuple[float, float, float], float], ...] = (
        DEFAULT_CANDIDATES)

    def __post_init__(self):
        object.__setattr__(self, "workspace_center_xy", tuple(
            _require_finite(v, "workspace_center_xy")
            for v in self.workspace_center_xy))
        object.__setattr__(self, "workspace_half_xy", tuple(
            _require_finite(v, "workspace_half_xy")
            for v in self.workspace_half_xy))
        object.__setattr__(self, "nominal_camera_xyz", tuple(
            _require_finite(v, "nominal_camera_xyz")
            for v in self.nominal_camera_xyz))
        for name in ("max_elapsed_sec", "camera_z_max", "camera_xy_radius_max"):
            _require_finite(getattr(self, name), name)

    def accept_radius_m(self):
        return max(self.workspace_half_xy[0], self.workspace_half_xy[1]) \
            + self.workspace_margin_m

    def constraints(self):
        """Constraints dict shaped for ``constrained_view_planner``."""
        return {
            "camera_z_max": self.camera_z_max,
            "wrist_z_max": self.camera_z_max,
            "coverage_radius": self.coverage_radius,
            "alignment_min": self.alignment_min,
        }

    def to_dict(self):
        return {
            "schema_version": SCHEMA_VERSION,
            "hint_min_confidence": self.hint_min_confidence,
            "production_min_confidence": self.production_min_confidence,
            "hint_min_side_px": self.hint_min_side_px,
            "area_fraction_min": self.area_fraction_min,
            "area_fraction_max": self.area_fraction_max,
            "foreground_min_pixels": self.foreground_min_pixels,
            "foreground_height_min_m": self.foreground_height_min_m,
            "foreground_height_max_m": self.foreground_height_max_m,
            "association_iou_min": self.association_iou_min,
            "border_margin_min_px": self.border_margin_min_px,
            "valid_depth_ratio_min": self.valid_depth_ratio_min,
            "cargo_cloud_min_points": self.cargo_cloud_min_points,
            "max_recovery_views": self.max_recovery_views,
            "max_elapsed_sec": self.max_elapsed_sec,
            "no_gain_confidence_delta": self.no_gain_confidence_delta,
            "no_gain_ratio_delta": self.no_gain_ratio_delta,
            "no_gain_streak": self.no_gain_streak,
            "window_max_stamp_span_sec": self.window_max_stamp_span_sec,
            "workspace_center_xy": list(self.workspace_center_xy),
            "workspace_half_xy": list(self.workspace_half_xy),
            "workspace_margin_m": self.workspace_margin_m,
            "workspace_plane_z": self.workspace_plane_z,
            "nominal_camera_xyz": list(self.nominal_camera_xyz),
            "camera_z_max": self.camera_z_max,
            "camera_xy_radius_max": self.camera_xy_radius_max,
            "coverage_radius": self.coverage_radius,
            "alignment_min": self.alignment_min,
            "coverage_weight": self.coverage_weight,
            "gain_weight": self.gain_weight,
            "path_weight": self.path_weight,
            "smooth_weight": self.smooth_weight,
            "candidates": [
                {"candidate_id": cid,
                 "camera_offset_xyz": list(offset),
                 "look_yaw_deg": yaw}
                for cid, offset, yaw in self.candidates
            ],
        }

    @classmethod
    def from_mapping(cls, data):
        remap = {
            "candidates": lambda rows: tuple(
                (str(r["candidate_id"]),
                 tuple(float(v) for v in r["camera_offset_xyz"]),
                 float(r["look_yaw_deg"])) for r in rows),
        }
        kwargs = {}
        for name in cls.__dataclass_fields__:
            if name in data:
                kwargs[name] = (remap[name](data[name])
                                if name in remap else data[name])
        return cls(**kwargs)


@dataclass(frozen=True)
class RecoveryDecision:
    """One declarative recovery decision for an evaluated window."""

    reason: str
    candidates: Tuple[CandidateView, ...] = ()
    cargo_present: bool = False
    incomplete: bool = False
    defects: Tuple[str, ...] = ()
    diagnostics: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self):
        candidates = tuple(self.candidates)
        if not all(isinstance(c, CandidateView) for c in candidates):
            raise TypeError("recovery decision candidates must be CandidateView")
        object.__setattr__(self, "candidates", candidates)
        object.__setattr__(self, "defects", tuple(str(d) for d in self.defects))
        object.__setattr__(self, "diagnostics", _freeze_mapping(self.diagnostics))

    @property
    def motion_authority(self):
        """False for every terminal reason; a fresh proposal is the only
        decision that may eventually move the arm (still gated by the
        coordinator)."""
        return bool(self.candidates) and self.reason == ""

    def to_dict(self):
        return {
            "schema_id": DECISION_SCHEMA_ID,
            "schema_version": SCHEMA_VERSION,
            "policy_id": POLICY_ID,
            "reason": self.reason,
            "motion_authority": self.motion_authority,
            "cargo_present": self.cargo_present,
            "incomplete": self.incomplete,
            "defects": list(self.defects),
            "candidates": [c.to_dict() for c in self.candidates],
            "diagnostics": _plain_mapping(self.diagnostics),
        }


def full_candidate_admissible(candidate, config):
    """Full admissibility check for one task-space candidate.

    Combines ``constrained_view_planner.candidate_is_admissible`` with the
    pickup-rig camera XY envelope and the workspace bounds on the look
    point. Returns ``(ok, reason_code)``.
    """
    ok, reason = candidate_is_admissible(candidate, config.constraints())
    if not ok:
        return False, reason
    cam = candidate["camera_xyz"]
    look = candidate["look_at"]
    center = config.workspace_center_xy
    if math.hypot(cam[0] - center[0], cam[1] - center[1]) > \
            config.camera_xy_radius_max:
        return False, "camera_outside_workspace_envelope"
    if math.hypot(look[0] - center[0], look[1] - center[1]) > \
            config.accept_radius_m():
        return False, "look_at_outside_workspace"
    return True, "ok"


class RecoveryCandidateRanker(CargoNBVPlanner):
    """Task-space adapter over ``CargoNBVPlanner``.

    Reuses the parent's bounded visited-candidate behavior: the ``_visited``
    set, ``reset()``, termination-first structure, weight-shaped scoring
    ``coverage_w * quality - path_w * dist - smooth_w * dist - 0.01 * idx``
    with the library-order tiebreak, and mark-visited-on-selection. The
    per-candidate quality term is adapted from the joint-space proxy to
    ``constrained_view_planner.coverage_score`` plus a measured
    ray-diversity gain.
    """

    def __init__(self, candidate_dicts, config, visited_ids=()):
        self._config = config
        self.candidate_dicts = list(candidate_dicts)
        super().__init__(
            candidates=[
                {"name": cand["name"], "values": list(cand["camera_xyz"])}
                for cand in self.candidate_dicts
            ],
            joint_names=("x", "y", "z"),
            weights={
                "coverage_weight": config.coverage_weight,
                "gain_weight": config.gain_weight,
                "path_weight": config.path_weight,
                "smooth_weight": config.smooth_weight,
            },
            unknown_threshold=-1.0,
            max_views=config.max_recovery_views,
        )
        self._nominal_look = tuple(config.nominal_camera_xyz)
        for idx, cand in enumerate(self.candidate_dicts):
            if str(cand["name"]) in set(visited_ids):
                self._visited.add(idx)

    def set_nominal_look(self, look_xyz):
        self._nominal_look = tuple(float(v) for v in look_xyz)

    def set_frontier(self, frontier_points):
        self._frontier = [list(map(float, p)) for p in frontier_points]
        return self

    def quality(self, idx):
        """(coverage, gain, distance) terms for candidate ``idx``."""
        cand = self.candidate_dicts[idx]
        cov = coverage_score(
            cand, getattr(self, "_frontier", []), self._config.constraints())
        nominal = [float(v) for v in self.candidate_dicts[idx]["nominal_xyz"]]
        cam = cand["camera_xyz"]
        dist = math.sqrt(sum(
            (float(cam[i]) - nominal[i]) ** 2 for i in range(3)))
        ray_a = _normalize_ray(nominal, self._nominal_look)
        ray_b = _normalize_ray(cam, cand["look_at"])
        dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(ray_a, ray_b))))
        gain = min(1.0, math.degrees(math.acos(dot)) / 12.0)
        return cov, gain, dist

    def score_terms(self, idx):
        cov, gain, dist = self.quality(idx)
        return {
            "coverage": round(cov, 6),
            "gain": round(gain, 6),
            "distance_m": round(dist, 6),
            "library_index": idx,
        }

    def score(self, idx):
        cov, gain, dist = self.quality(idx)
        return (
            self.weights.get("coverage_weight", 1.0) * cov
            + self.weights.get("gain_weight", 1.0) * gain
            - self.weights.get("path_weight", 0.3) * dist
            - self.weights.get("smooth_weight", 0.2) * dist
            - 0.01 * idx
        )

    def rank_all(self, exclude_visited=True):
        """Deterministic ranking of admissible candidates.

        Returns ``[(library_idx, score, candidate_dict), ...]`` sorted by
        score descending, library order ascending.
        """
        scored = []
        for idx, cand in enumerate(self.candidate_dicts):
            if exclude_visited and idx in self._visited:
                continue
            ok, _reason = full_candidate_admissible(cand, self._config)
            if not ok:
                continue
            scored.append((idx, self.score(idx), cand))
        scored.sort(key=lambda item: (-item[1], item[0]))
        return scored

    def plan_next(self, stats=None, views_used=None):
        """Select and mark-visit the best admissible unvisited candidate."""
        if views_used is not None and int(views_used) >= self.max_views:
            return self._done("max recovery views reached")
        ranked = self.rank_all(exclude_visited=True)
        if not ranked:
            return self._done("no admissible recovery candidates left")
        best_idx = ranked[0][0]
        self._visited.add(best_idx)
        chosen = self.candidate_dicts[best_idx]
        return {
            "done": False,
            "view_index": best_idx,
            "name": str(chosen["name"]),
            "camera_xyz": list(chosen["camera_xyz"]),
            "look_at": list(chosen["look_at"]),
            "score": round(ranked[0][1], 6),
        }


class ActiveViewRecoveryPolicy:
    """ROS-free detector-miss recovery policy (``ExplorationPolicy`` shape).

    Terminal reasons and gates follow the ACTIVE-VIEW-1 contract exactly;
    every terminal reason forbids motion authority.
    """

    policy_id = POLICY_ID

    def __init__(self, config=None):
        self.config = config or ActiveViewConfig()
        self.reset(ExplorationContext(
            frames={}, geometry_descriptor={}, geometry_hash="",
            camera_model={}))

    # -- session state --------------------------------------------------

    def reset(self, context: ExplorationContext) -> None:
        self._context = context
        camera_model = _mapping_to_dict(context.camera_model)
        self._expected_camera_model_id = str(camera_model.get("model_id", ""))
        self._expected_geometry_hash = str(context.geometry_hash)
        self._visited_ids: List[str] = []
        self._counted_ids = set()
        self._views_executed = 0
        self._first_decision_stamp: Optional[AcquisitionStamp] = None
        self._best_confidence: Optional[float] = None
        self._best_ratio: Optional[float] = None
        self._no_gain_streak = 0
        self._external_cancel = False

    # -- candidate construction -------------------------------------------

    def _build_ranker(self, measured_look_xyz, frontier_points):
        """Build the ranker for this window from config + measured data."""
        config = self.config
        nominal = config.nominal_camera_xyz
        candidate_dicts = []
        seen_poses = set()
        for idx, (cid, offset, yaw_deg) in enumerate(config.candidates):
            camera = (
                nominal[0] + offset[0],
                nominal[1] + offset[1],
                nominal[2] + offset[2],
            )
            look = list(measured_look_xyz)
            if abs(yaw_deg) > 1e-9:
                ray = _rotate_yaw(
                    (look[0] - camera[0], look[1] - camera[1],
                     look[2] - camera[2]),
                    yaw_deg)
                look = [camera[0] + ray[0], camera[1] + ray[1],
                        camera[2] + ray[2]]
            if not all(math.isfinite(v) for v in tuple(camera) + tuple(look)):
                continue  # configuration-invalid (non-finite) candidate
            # A duplicate is the same *view* (camera pose AND look point);
            # yaw candidates share the nominal camera pose on purpose.
            view_key = (tuple(round(v, 4) for v in camera),
                        tuple(round(v, 4) for v in look))
            if view_key in seen_poses:
                continue  # duplicate candidate view
            seen_poses.add(view_key)
            candidate_dicts.append({
                "name": cid,
                "stage": "active_view_recovery",
                "camera_xyz": list(camera),
                "look_at": [float(look[0]), float(look[1]), float(look[2])],
                "orientation_quat": look_at_quaternion(camera, tuple(look)),
                "nominal_xyz": list(nominal),
                "library_index": idx,
            })
        ranker = RecoveryCandidateRanker(
            candidate_dicts, config, visited_ids=self._visited_ids)
        ranker.set_nominal_look(measured_look_xyz)
        ranker.set_frontier(frontier_points)
        return ranker

    def _measured_look_point(self, window):
        """Measured cargo centroid estimate (world frame) for look-at.

        Prefers the median measured landing point of *qualifying* evidence
        (hints that pass the presence floors, workspace-member accepted
        detections); falls back to the workspace centre on the pickup
        plane. Non-qualifying boxes must not drag the look point out of
        the workspace.
        """
        config = self.config
        points = []
        for frame in window.frames:
            for hint in frame.diagnostic_hints:
                if (hint.centre_world_xy is not None
                        and self._hint_qualifies(hint, window)):
                    points.append(hint.centre_world_xy)
            for det in frame.accepted_detections:
                if (det.centre_world_xy is not None
                        and det.confidence >= config.production_min_confidence
                        and self._workspace_member(det.centre_world_xy)):
                    points.append(det.centre_world_xy)
        if points:
            xs = sorted(p[0] for p in points)
            ys = sorted(p[1] for p in points)
            cx = xs[len(xs) // 2]
            cy = ys[len(ys) // 2]
        else:
            cx, cy = config.workspace_center_xy
        return (cx, cy, config.workspace_plane_z)

    # -- health / classification ------------------------------------------

    def _health_defect(self, window):
        frames = window.frames
        first = frames[0]
        for frame in frames[1:]:
            if frame.frame_id != first.frame_id:
                return "frame_id_mismatch"
            if frame.camera_model_id != first.camera_model_id:
                return "camera_model_mismatch"
            if frame.geometry_hash != first.geometry_hash:
                return "geometry_hash_mismatch"
            if frame.map_revision != first.map_revision:
                return "map_revision_mismatch"
        if self._expected_camera_model_id and \
                first.camera_model_id != self._expected_camera_model_id:
            return "camera_model_mismatch"
        if self._expected_geometry_hash and \
                first.geometry_hash != self._expected_geometry_hash:
            return "geometry_hash_mismatch"
        prev = None
        for frame in frames:
            stamp = frame.stamp
            if prev is not None and not (
                    stamp.sec > prev.sec or
                    (stamp.sec == prev.sec and stamp.nanosec > prev.nanosec)):
                return "stamps_not_strictly_increasing"
            prev = stamp
        if window.stamp_span_sec() > \
                self.config.window_max_stamp_span_sec + 1e-9:
            return "stamp_span_too_large"
        for frame in frames:
            if not (frame.rgb_ok and frame.depth_ok and frame.aligned_depth_ok
                    and frame.exact_tf_ok and frame.robot_settled):
                return "frame_unhealthy"
            if frame.cargo_geometry is None:
                return "cargo_geometry_missing"
        return None

    def _workspace_member(self, world_xy):
        if world_xy is None:
            return False
        config = self.config
        dx = world_xy[0] - config.workspace_center_xy[0]
        dy = world_xy[1] - config.workspace_center_xy[1]
        return math.hypot(dx, dy) <= config.accept_radius_m()

    def _hint_qualifies(self, hint, window):
        config = self.config
        if not (config.hint_min_confidence <= hint.confidence
                < config.production_min_confidence):
            return False
        width = hint.bbox[2] - hint.bbox[0]
        height = hint.bbox[3] - hint.bbox[1]
        if width < config.hint_min_side_px or height < config.hint_min_side_px:
            return False
        total = float(window.image_width * window.image_height)
        if total <= 0.0:
            return False
        frac = float(width * height) / total
        if not (config.area_fraction_min <= frac <= config.area_fraction_max):
            return False
        return self._workspace_member(hint.centre_world_xy)

    def _component_qualifies(self, component):
        config = self.config
        if not component.in_workspace:
            return False
        if component.pixel_count < config.foreground_min_pixels:
            return False
        if component.area_fraction is not None and not (
                config.area_fraction_min <= component.area_fraction
                <= config.area_fraction_max):
            return False
        if component.height_above_support_m is None:
            return False
        return (config.foreground_height_min_m
                <= component.height_above_support_m
                <= config.foreground_height_max_m)

    def _frame_evidence(self, window):
        """Per-frame qualifying cargo evidence.

        Qualifying evidence per frame: accepted production detections
        (confidence at the unchanged 0.20 floor — the strongest
        production-safe presence signal), diagnostic hints that pass the
        presence floors, and qualifying depth-foreground components.
        """
        config = self.config
        per_frame = []
        for frame in window.frames:
            dets = [d for d in frame.accepted_detections
                    if d.confidence >= config.production_min_confidence]
            hints = [h for h in frame.diagnostic_hints
                     if self._hint_qualifies(h, window)]
            comps = [c for c in frame.foreground_components
                     if self._component_qualifies(c)]
            per_frame.append((dets, hints, comps))
        return per_frame

    def _cargo_present(self, window):
        """Cargo-present determination with cross-frame IoU association."""
        config = self.config
        per_frame = self._frame_evidence(window)
        qualifying = [idx for idx, (dets, hints, comps) in enumerate(per_frame)
                      if dets or hints or comps]
        if len(qualifying) < 2:
            return False, per_frame
        for i in range(len(qualifying)):
            for j in range(i + 1, len(qualifying)):
                if self._evidence_associates(
                        per_frame[qualifying[i]], per_frame[qualifying[j]],
                        config):
                    return True, per_frame
        return False, per_frame

    @staticmethod
    def _evidence_associates(evidence_a, evidence_b, config):
        def boxes(evidence):
            out = [tuple(d.bbox) for d in evidence[0]]
            out.extend(tuple(h.bbox) for h in evidence[1])
            out.extend(tuple(c.bbox) for c in evidence[2]
                       if c.bbox is not None)
            return out

        for box_a in boxes(evidence_a):
            for box_b in boxes(evidence_b):
                if bbox_iou(box_a, box_b) >= config.association_iou_min:
                    return True
        return False

    def _all_frames_planner_ready(self, window):
        for frame in window.frames:
            geometry = frame.cargo_geometry
            if geometry is None or not geometry.planner_ready:
                return False
        return True

    def _incomplete_defects(self, window):
        """Measured defects that make a cargo-present view incomplete."""
        config = self.config
        accepted_total = sum(len(f.accepted_detections) for f in window.frames)
        if accepted_total == 0:
            return ["no_accepted_detections"]
        planner_ready_frames = 0
        defects = []
        for frame in window.frames:
            geometry = frame.cargo_geometry
            if geometry is not None and geometry.planner_ready:
                planner_ready_frames += 1
                continue
            for det in frame.accepted_detections:
                if det.border_margin_px is not None and \
                        det.border_margin_px < config.border_margin_min_px:
                    defects.append("bbox_border_margin")
                if det.valid_depth_ratio is not None and \
                        det.valid_depth_ratio < config.valid_depth_ratio_min:
                    defects.append("bbox_valid_depth_ratio")
            if geometry is None:
                defects.append("cargo_geometry_missing")
            else:
                if geometry.point_count < config.cargo_cloud_min_points:
                    defects.append("cargo_cloud_points")
                if not geometry.top_surface_valid:
                    defects.append("top_surface_invalid")
                if geometry.support_mode != "FULL_3D":
                    defects.append("support_mode_not_full_3d")
        if planner_ready_frames == len(window.frames):
            return []
        return defects

    def _elapsed_sec(self, window):
        anchor = self._first_decision_stamp
        if anchor is None:
            return 0.0
        end = window.frames[-1].stamp
        return float(end.sec - anchor.sec) + 1e-9 * float(
            end.nanosec - anchor.nanosec)

    # -- decision core ------------------------------------------------------

    def evaluate(self, window: RecoveryWindow) -> RecoveryDecision:
        """Evaluate one measured window and return the recovery decision."""
        config = self.config
        base_diag = {
            "source_stamps": [f.stamp.to_dict() for f in window.frames],
            "geometry_hash": window.frames[0].geometry_hash,
            "map_revision": window.frames[0].map_revision,
            "state": window.state,
        }

        # 1. State gate: recovery is legal only pre-pick, payload free,
        #    vacuum off, no cancellation or abort.
        if (window.state != PRE_PICK_DETECT_STATE
                or window.payload_attached
                or window.vacuum_commanded
                or window.cancel_requested
                or self._external_cancel):
            return RecoveryDecision(
                reason=RECOVERY_STATE_INVALID,
                diagnostics=dict(base_diag, gate="state"))

        # 2. Health / identity / stamp gate.
        defect = self._health_defect(window)
        if defect is not None:
            return RecoveryDecision(
                reason=RECOVERY_EVIDENCE_INVALID,
                diagnostics=dict(base_diag, health_defect=defect))

        # 3. Unchanged production detection plus planner-ready FULL_3D
        #    geometry ends recovery immediately.
        accepted_total = sum(
            len(f.accepted_detections) for f in window.frames)
        if accepted_total > 0 and self._all_frames_planner_ready(window):
            return RecoveryDecision(
                reason=RECOVERY_SUCCEEDED, cargo_present=True,
                diagnostics=dict(base_diag, gate="planner_ready"))

        # 4. Budget gates once recovery has started (measured stamps only).
        if self._first_decision_stamp is not None:
            elapsed = self._elapsed_sec(window)
            if self._views_executed >= config.max_recovery_views:
                return RecoveryDecision(
                    reason=RECOVERY_BUDGET_EXHAUSTED,
                    diagnostics=dict(
                        base_diag, gate="views_budget",
                        views_executed=self._views_executed,
                        max_recovery_views=config.max_recovery_views))
            if elapsed > config.max_elapsed_sec:
                return RecoveryDecision(
                    reason=RECOVERY_BUDGET_EXHAUSTED,
                    diagnostics=dict(
                        base_diag, gate="elapsed_budget",
                        elapsed_sec=round(elapsed, 6),
                        max_elapsed_sec=config.max_elapsed_sec))
            if self._no_gain_streak >= config.no_gain_streak:
                return RecoveryDecision(
                    reason=RECOVERY_NO_GAIN,
                    diagnostics=dict(
                        base_diag, gate="no_gain",
                        no_gain_streak=self._no_gain_streak))

        # 5. Cargo presence from measured, production-safe signals.
        cargo_present, per_frame = self._cargo_present(window)
        if not cargo_present:
            return RecoveryDecision(
                reason=NO_CARGO_EVIDENCE,
                diagnostics=dict(
                    base_diag, gate="cargo_absent",
                    evidence_frames=[
                        len(d) + len(h) + len(c)
                        for d, h, c in per_frame]))

        # 6. Incomplete-observation determination.
        defects = self._incomplete_defects(window)
        if not defects:
            return RecoveryDecision(
                reason=RECOVERY_NOT_REQUIRED, cargo_present=True,
                diagnostics=dict(base_diag, gate="complete_enough"))

        # 7. Bounded candidate proposal.
        look_point = self._measured_look_point(window)
        ranker = self._build_ranker(
            look_point, frontier_points=[list(look_point)])
        ranked_all = ranker.rank_all(exclude_visited=False)
        rejected = {}
        for idx, cand in enumerate(ranker.candidate_dicts):
            cid = str(cand["name"])
            if idx in ranker._visited:
                rejected[cid] = "visited"
                continue
            ok, reason = full_candidate_admissible(cand, config)
            if not ok:
                rejected[cid] = reason
        choice = ranker.plan_next(views_used=self._views_executed)
        if choice.get("done"):
            return RecoveryDecision(
                reason=RECOVERY_CANDIDATES_EXHAUSTED, cargo_present=True,
                incomplete=True, defects=defects,
                diagnostics=dict(
                    base_diag, gate="candidates_exhausted",
                    rejected=rejected))
        best_idx = choice["view_index"]
        chosen = ranker.candidate_dicts[best_idx]
        candidate_id = str(chosen["name"])
        self._visited_ids.append(candidate_id)
        if self._first_decision_stamp is None:
            self._first_decision_stamp = window.frames[-1].stamp
            self._seed_gain_baselines(window)
        candidate = CandidateView(
            candidate_id=candidate_id,
            frame_id=window.frames[-1].frame_id,
            position_xyz=tuple(float(v) for v in chosen["camera_xyz"]),
            orientation_xyzw=tuple(
                float(v) for v in chosen["orientation_quat"]),
            score=float(choice["score"]),
            diagnostics=dict(
                ranker.score_terms(best_idx),
                feasibility="unknown",
                source_stamps=base_diag["source_stamps"],
                geometry_hash=base_diag["geometry_hash"],
                map_revision=base_diag["map_revision"],
                look_at_xyz=[round(float(v), 6) for v in chosen["look_at"]],
            ),
        )
        return RecoveryDecision(
            reason="", candidates=(candidate,), cargo_present=True,
            incomplete=True, defects=defects,
            diagnostics=dict(
                base_diag, gate="propose",
                ranked_candidate_ids=[
                    str(c["name"]) for _i, _s, c in ranked_all],
                rejected=rejected,
                views_executed=self._views_executed,
                first_recovery_stamp=(
                    self._first_decision_stamp.to_dict()
                    if self._first_decision_stamp is not None else None),
            ))

    def evaluate_mapping(self, data):
        """Evaluate plain measured data, translating errors fail-closed."""
        try:
            window = RecoveryWindow.from_mapping(data)
        except (ValueError, TypeError, KeyError):
            return RecoveryDecision(
                reason=RECOVERY_EVIDENCE_INVALID,
                diagnostics={"gate": "input_invalid"})
        return self.evaluate(window)

    # -- ExplorationPolicy adapter -------------------------------------------

    def propose(self, context: ExplorationContext,
                snapshot: ExplorationSnapshot) -> PolicyProposal:
        raw = _mapping_to_dict(snapshot.diagnostics).get("active_view_window")
        if raw is None:
            decision = RecoveryDecision(
                reason=RECOVERY_EVIDENCE_INVALID,
                diagnostics={"gate": "window_missing"})
        else:
            decision = self.evaluate_mapping(raw)
        return PolicyProposal(
            policy_id=self.policy_id,
            proposal_schema_id=DECISION_SCHEMA_ID,
            candidates=decision.candidates,
            done_recommended=decision.reason != "",
            done_reason=decision.reason,
            diagnostics=dict(
                decision.diagnostics, decision_reason=decision.reason),
        )

    def observe(self, outcome: ViewOutcome) -> None:
        reason = str(outcome.reason_code or "")
        if reason in _CANCEL_REASON_CODES:
            self._external_cancel = True
        if outcome.candidate_id and outcome.candidate_id not in self._visited_ids:
            self._visited_ids.append(outcome.candidate_id)
        if outcome.status in (ViewOutcomeStatus.EXECUTED,
                              ViewOutcomeStatus.INTEGRATED):
            if outcome.candidate_id not in self._counted_ids:
                self._counted_ids.add(outcome.candidate_id)
                self._views_executed += 1
        if outcome.status == ViewOutcomeStatus.INTEGRATED:
            diagnostics = _mapping_to_dict(outcome.diagnostics)
            self._update_gain_trackers(
                diagnostics.get("best_cargo_confidence"),
                diagnostics.get("bbox_valid_depth_ratio"))

    def _seed_gain_baselines(self, window):
        """Seed no-gain baselines from the measured trigger window.

        Best cargo confidence starts at the strongest measured detection
        or hint confidence in the triggering window; the bbox valid-depth
        ratio starts at the strongest measured ratio, or 0.0 when the
        trigger window measured none (fail-open floor, never fabricated
        upward).
        """
        best_conf = None
        best_ratio = None
        for frame in window.frames:
            for det in frame.accepted_detections:
                if best_conf is None or det.confidence > best_conf:
                    best_conf = det.confidence
                if (det.valid_depth_ratio is not None
                        and (best_ratio is None
                             or det.valid_depth_ratio > best_ratio)):
                    best_ratio = det.valid_depth_ratio
            for hint in frame.diagnostic_hints:
                if best_conf is None or hint.confidence > best_conf:
                    best_conf = hint.confidence
        self._best_confidence = best_conf
        self._best_ratio = 0.0 if best_ratio is None else best_ratio

    def _update_gain_trackers(self, confidence, ratio):
        conf_gain = None
        ratio_gain = None
        if confidence is not None:
            confidence = float(confidence)
            prev = self._best_confidence
            conf_gain = None if prev is None else confidence - prev
            if prev is None or confidence > prev:
                self._best_confidence = confidence
        if ratio is not None:
            ratio = float(ratio)
            prev = self._best_ratio
            ratio_gain = None if prev is None else ratio - prev
            if prev is None or ratio > prev:
                self._best_ratio = ratio
        if conf_gain is not None and ratio_gain is not None:
            if (conf_gain < self.config.no_gain_confidence_delta
                    and ratio_gain < self.config.no_gain_ratio_delta):
                self._no_gain_streak += 1
            else:
                self._no_gain_streak = 0


def load_replay_fixture(path):
    """Load a bounded replay fixture.

    Returns ``(fixture_dict, sessions)``; each session carries ``window``
    (the plain policy-input mapping) and ``expected`` (harness-only labels
    that must never enter the policy input).
    """
    import json
    with open(path, "r") as handle:
        fixture = json.load(handle)
    return fixture, fixture.get("sessions", {})
