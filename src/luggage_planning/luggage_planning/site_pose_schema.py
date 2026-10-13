"""Deploy site-pose schema and observation policy (ST09-PD).

A deploy site file records the *measured* joint states the hardware
cell needs: pickup, return, portal observation and second-region
observation. Until the cell is tape-measured those entries stay
``valid: false`` and cannot execute -- ``current`` (stay at the live
``/joint_states``) is the only executable fallback, exactly as
``named_robot_poses`` defines it. A measured entry additionally pins
the scene revision and joint order it was captured against, so a
re-tuned container or a reordered model invalidates the measurement
instead of silently reusing it.

Validators reject simulation provenance outright: a pose file whose
provenance names simulation, Gazebo, or ``robot_poses.yaml.example``
is not a deploy site file no matter what its values look like.
"""

from __future__ import division

import math
import os

import yaml

from luggage_planning.named_robot_poses import JOINTS

SITE_POSE_SCHEMA = "luggage.site_poses/1"

#: [ST09-I S5.7-6] The simulation site-pose schema: the same measured
#: discipline (scene revision, joint order, per-entry capture binding)
#: with ``calibration_status: synthetic`` allowed and the
#: ``defaults.observe_pose``-must-stay-current rule lifted — a SIM file
#: is what the S5 gazebo runtime and the I-suite fixtures launch with.
#: The deploy validator's strictness is unchanged (``synthetic`` is
#: accepted ONLY under this schema).
SIM_SITE_POSE_SCHEMA = "luggage.site_poses.sim/1"

#: The two synthetic pose kinds the sim runtime freezes (the pick
#: observe anchor and the portal exit view; a sim cell never has a
#: second cargo region).
SIM_POSE_KINDS = (
    "pickup_observe_sim",
    "portal_observe_sim",
)

#: The four measured pose kinds the deploy cell freezes. ``return`` is
#: the post-place retreat/observe base; the two observation kinds cover
#: the portal view and the second cargo region view.
SITE_POSE_KINDS = (
    "pickup",
    "return",
    "portal_observation",
    "second_region_observation",
)

#: Provenance sources a deploy cell accepts for a measured pose.
MEASURED_SOURCES = frozenset((
    "tape_measure",
    "pendant_dump",
    "site_survey",
    "synthetic_fixture",
))

#: Provenance markers that mean simulation, never a deploy measurement.
SIM_PROVENANCE_MARKERS = ("sim", "gazebo", "example", "simulation")

#: Executable fallback when no measured entry is valid.
EXECUTABLE_FALLBACK = "current"

#: Validation outcome labels (a pose never reports both).
VALID = "valid"
NOT_EXECUTABLE = "not_executable"
NOT_EVALUATED = "not_evaluated"


def load_site_poses(path):
    """Load and shape-check a deploy site-pose yaml file."""
    if not path or not os.path.isfile(path):
        raise ValueError("deploy site pose file missing: %r" % (path,))
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError("deploy site pose file is not a mapping: %s" % path)
    return data


def validate_site_poses(data, *, expected_scene_revision,
                        expected_joint_names=None,
                        expected_srdf_sha256=None,
                        expected_urdf_sha256=None,
                        schema=SITE_POSE_SCHEMA,
                        pose_kinds=None):
    """Validate a deploy (or sim) site-pose record against the cell.

    Returns a report dict:
    ``{"ok", "schema", "scene_revision", "joint_order", "executable_fallback",
       "poses": {kind: {"valid", "outcome", "issues"[]}}, "issues"[]}``.
    ``ok`` is true only when the file itself is a well-formed site
    record for this exact cell (schema, scene, joint order, no
    simulation provenance). Individual unmeasured entries stay
    ``valid:false`` / ``not_executable`` -- that is the correct state
    before calibration, not a failure of the file.

    [ST09-I S5.7-6] ``schema=SIM_SITE_POSE_SCHEMA`` switches the pose
    kinds to ``SIM_POSE_KINDS``, accepts ``calibration_status:
    synthetic`` (SIM only — the deploy schema still demands
    ``measured``) and lifts the ``defaults.observe_pose``-must-stay-
    current rule (a sim file names its observe pose). Deploy callers
    pass nothing new: identical behavior.
    """
    schema = str(schema or SITE_POSE_SCHEMA)
    if schema not in (SITE_POSE_SCHEMA, SIM_SITE_POSE_SCHEMA):
        raise ValueError("unknown site-pose schema: %r" % schema)
    accept_synthetic = schema == SIM_SITE_POSE_SCHEMA
    pose_kinds = tuple(pose_kinds) if pose_kinds else (
        SIM_POSE_KINDS if accept_synthetic else SITE_POSE_KINDS)
    expected_joint_names = list(expected_joint_names or JOINTS)
    report = {
        "schema": data.get("schema"),
        "scene_revision": str((data.get("identity") or {}).get(
            "scene_revision") or ""),
        "joint_order": list((data.get("identity") or {}).get(
            "joint_names") or []),
        "executable_fallback": EXECUTABLE_FALLBACK,
        "poses": {},
        "issues": [],
        "ok": False,
    }

    def issue(text):
        report["issues"].append(text)

    if data.get("schema") != schema:
        issue("schema is %r, expected %s" % (
            data.get("schema"), schema))
    identity = data.get("identity") or {}
    if not isinstance(identity, dict):
        issue("identity block missing")
        identity = {}
    if not str(identity.get("scene_revision") or ""):
        issue("identity.scene_revision missing")
    elif identity["scene_revision"] != str(expected_scene_revision):
        issue("wrong scene: file pins %s, cell froze %s" % (
            identity["scene_revision"][:8], str(expected_scene_revision)[:8]))
    file_joints = list(identity.get("joint_names") or [])
    if file_joints != list(expected_joint_names):
        issue("joint order mismatch: %r != %r" % (
            file_joints, list(expected_joint_names)))
    for key, expected in (("srdf_sha256", expected_srdf_sha256),
                          ("urdf_sha256", expected_urdf_sha256)):
        declared = str(identity.get(key) or "")
        if declared and expected and declared != str(expected):
            issue("%s mismatch: %r != %r" % (key, declared, str(expected)))

    poses = data.get("poses")
    if not isinstance(poses, dict):
        issue("poses block missing")
        poses = {}
    for kind in pose_kinds:
        if kind not in poses:
            report["poses"][kind] = {
                "valid": False, "outcome": NOT_EVALUATED,
                "issues": ["entry missing"]}
            issue("pose %s missing" % kind)
            continue
        report["poses"][kind] = _validate_entry(
            kind, poses[kind], expected_scene_revision,
            accept_synthetic=accept_synthetic)
        if poses[kind].get("valid") is True and not report["poses"][kind]["valid"]:
            # A declared-measured entry that does not hold is a file-level
            # defect, not a per-entry footnote.
            issue("pose %s declares valid but fails: %s"
                  % (kind, "; ".join(report["poses"][kind]["issues"])))
    for key in sorted(set(poses) - set(pose_kinds)):
        issue("unknown pose kind %s" % key)

    if not accept_synthetic:
        if not str(data.get("defaults", {}).get("observe_pose")
                   or EXECUTABLE_FALLBACK) == EXECUTABLE_FALLBACK:
            issue("defaults.observe_pose must stay %r until every entry is "
                  "measured" % EXECUTABLE_FALLBACK)
    report["ok"] = not report["issues"]
    return report


def _validate_entry(kind, entry, expected_scene_revision,
                    accept_synthetic=False):
    result = {"valid": False, "outcome": NOT_EXECUTABLE, "issues": []}

    def problem(text):
        result["issues"].append(text)

    if not isinstance(entry, dict):
        result["outcome"] = NOT_EVALUATED
        problem("entry is not a mapping")
        return result
    provenance = entry.get("provenance") or {}
    source = str(provenance.get("source") or "").strip().lower()
    if any(marker in source for marker in SIM_PROVENANCE_MARKERS):
        problem("simulation provenance (%r) is never a deploy measurement"
                % provenance.get("source"))
        result["outcome"] = NOT_EVALUATED
        return result
    if entry.get("valid") is not True:
        if entry.get("valid") is not False:
            problem("valid flag missing")
            result["outcome"] = NOT_EVALUATED
        return result

    # valid:true from here on: every measured-entry requirement applies
    # (``synthetic`` calibration is the one SIM-schema allowance; the
    # provenance stays ``synthetic_fixture``, which the deploy cell
    # also accepts — it is the fixture discipline, not simulation
    # provenance).
    calibration = str(entry.get("calibration_status") or "").strip().lower()
    if calibration != "measured" and not (
            accept_synthetic and calibration == "synthetic"):
        problem("calibration_status is %r, not measured"
                % entry.get("calibration_status"))
    if source not in MEASURED_SOURCES:
        problem("provenance source %r is not an accepted measurement source"
                % provenance.get("source"))
    values = entry.get("values")
    if (not isinstance(values, list) or len(values) != len(JOINTS)
            or any(not isinstance(v, (int, float))
                   or not math.isfinite(float(v)) for v in values)):
        problem("values must be %d finite joint radians" % len(JOINTS))
    stamp = entry.get("acquisition_stamp_ns")
    if not isinstance(stamp, int) or isinstance(stamp, bool) or stamp <= 0:
        problem("acquisition_stamp_ns must be a positive integer")
    captured = str(entry.get("scene_revision_at_capture") or "")
    if not captured:
        problem("scene_revision_at_capture missing")
    elif captured != str(expected_scene_revision):
        problem("stale: captured against scene %s, cell froze %s"
                % (captured[:8], str(expected_scene_revision)[:8]))
    result["valid"] = not result["issues"]
    result["outcome"] = VALID if result["valid"] else NOT_EXECUTABLE
    if result["valid"]:
        result["values"] = [float(v) for v in values]
    return result


def executable_site_pose(report, kind):
    """Joint values when ``kind`` is a validated measured entry, else None.

    ``None`` means not executable: the caller stays on the
    ``current`` fallback and must not move the arm from this file.
    """
    if not report.get("ok"):
        return None
    entry = (report.get("poses") or {}).get(kind) or {}
    if not entry.get("valid"):
        return None
    return list(entry.get("values") or [])
