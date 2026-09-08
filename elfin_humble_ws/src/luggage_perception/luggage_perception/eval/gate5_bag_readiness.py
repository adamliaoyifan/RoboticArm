#!/usr/bin/env python3
"""Gate 5 rosbag readiness checker (PF-A2). No ROS, no real bag required.

Validates a backend-neutral dataset *manifest*. Passing this checker means
the recording/replay input can be validated. It never claims Gate 5
accuracy: that still needs a real bag and independently established
references (docs/plans/platform_free_height_gate5_bag_contract.md).
"""

from __future__ import division

import json
from copy import deepcopy

SCHEMA_ID = "gate5_bag_manifest/v1"

# Stable machine-readable reasons. Do not rename: tests and reports key on
# these strings.
BAG_MISSING_TOPIC = "BAG_MISSING_TOPIC"
BAG_WRONG_TYPE = "BAG_WRONG_TYPE"
BAG_EMPTY_STREAM = "BAG_EMPTY_STREAM"
BAG_DURATION_SHORT = "BAG_DURATION_SHORT"
BAG_STAMP_NON_MONOTONIC = "BAG_STAMP_NON_MONOTONIC"
BAG_WINDOWS_NO_OVERLAP = "BAG_WINDOWS_NO_OVERLAP"
BAG_MISSING_FRAME = "BAG_MISSING_FRAME"
BAG_TF_GAP = "BAG_TF_GAP"
BAG_SEGMENTATION_INPUT_MISSING = "BAG_SEGMENTATION_INPUT_MISSING"
BAG_REFERENCE_LEAK = "BAG_REFERENCE_LEAK"
BAG_COVERAGE_INSUFFICIENT = "BAG_COVERAGE_INSUFFICIENT"
BAG_GATE5_ACCURACY_NOT_CLAIMED = "BAG_GATE5_ACCURACY_NOT_CLAIMED"
BAG_SCHEMA_INVALID = "BAG_SCHEMA_INVALID"

REASON_CODES = (
    BAG_MISSING_TOPIC,
    BAG_WRONG_TYPE,
    BAG_EMPTY_STREAM,
    BAG_DURATION_SHORT,
    BAG_STAMP_NON_MONOTONIC,
    BAG_WINDOWS_NO_OVERLAP,
    BAG_MISSING_FRAME,
    BAG_TF_GAP,
    BAG_SEGMENTATION_INPUT_MISSING,
    BAG_REFERENCE_LEAK,
    BAG_COVERAGE_INSUFFICIENT,
    BAG_GATE5_ACCURACY_NOT_CLAIMED,
    BAG_SCHEMA_INVALID,
)

# Canonical preprocessed + TF + joints. Names match Humble node defaults.
REQUIRED_ONLINE_TOPICS = (
    ("/luggage/preprocessed/camera/color/image", "sensor_msgs/msg/Image"),
    ("/luggage/preprocessed/camera/color/camera_info",
     "sensor_msgs/msg/CameraInfo"),
    ("/luggage/preprocessed/camera/depth/image", "sensor_msgs/msg/Image"),
    ("/luggage/preprocessed/camera/depth/camera_info",
     "sensor_msgs/msg/CameraInfo"),
    ("/luggage/preprocessed/camera/depth/points",
     "sensor_msgs/msg/PointCloud2"),
    ("/luggage/preprocessed/status", "std_msgs/msg/String"),
    ("/joint_states", "sensor_msgs/msg/JointState"),
    ("/tf", "tf2_msgs/msg/TFMessage"),
    ("/tf_static", "tf2_msgs/msg/TFMessage"),
)

# Replay must have at least one deterministic segmentation input.
SEGMENTATION_TOPICS = (
    ("/luggage/semantic/mask", "sensor_msgs/msg/Image"),
    ("/luggage/semantic/yolo_detections", "luggage_msgs/msg/YoloDetections"),
)

DETECTOR_OUTPUT_TOPIC = (
    "/luggage/perception/detection_frame",
    "luggage_msgs/msg/DetectionFrame",
)

REQUIRED_FRAMES = (
    "world",
    "camera_depth_optical_frame",
)

REQUIRED_TF_PAIRS = (
    ("world", "camera_depth_optical_frame"),
)

# Topics that are eval/spawner geometry truth. They may be recorded under
# role=reference. They must never be listed as online algorithm inputs.
REFERENCE_TOPIC_NAMES = (
    "/pickup_box_spawner/get_current_box",
    "/luggage/perception/size_eval/spawned",
    "/luggage/eval/reference_box",
    "/luggage/eval/reconstructed_box",
)

ONLINE_ROLES = frozenset(("online", "online_algorithm"))
REFERENCE_ROLES = frozenset(("reference", "eval_only", "offline_label"))
TASK_STATE_ROLES = frozenset(("task_state",))
OUTPUT_ROLES = frozenset(("output", "detector_output"))

MIN_DURATION_SEC = 5.0
MIN_SIZES = 3
MIN_PLACEMENTS_PER_SIZE = 3
MIN_SETTLED = 30
LEGACY_PLATFORM_Z_M = 0.86


def normalize_type(ros_type):
    """Accept both sensor_msgs/Image and sensor_msgs/msg/Image."""
    text = str(ros_type or "").strip()
    if not text:
        return ""
    parts = text.split("/")
    if len(parts) == 2:
        return "%s/msg/%s" % (parts[0], parts[1])
    return text


def _issue(code, topic="", detail=""):
    return {"code": code, "topic": topic, "detail": detail}


def _topics_by_name(manifest):
    out = {}
    for entry in manifest.get("topics") or []:
        name = str(entry.get("name") or "")
        if name:
            out[name] = entry
    return out


def _role(entry):
    return str(entry.get("role") or "online").strip().lower()


def check_schema(manifest):
    issues = []
    if not isinstance(manifest, dict):
        return [_issue(BAG_SCHEMA_INVALID, detail="manifest is not an object")]
    if str(manifest.get("schema") or "") != SCHEMA_ID:
        issues.append(_issue(
            BAG_SCHEMA_INVALID,
            detail="expected schema %s" % SCHEMA_ID))
    if not isinstance(manifest.get("topics"), list):
        issues.append(_issue(BAG_SCHEMA_INVALID, detail="topics must be a list"))
    return issues


def check_required_topics(manifest):
    issues = []
    by_name = _topics_by_name(manifest)
    for name, expected_type in REQUIRED_ONLINE_TOPICS:
        entry = by_name.get(name)
        if entry is None:
            issues.append(_issue(BAG_MISSING_TOPIC, topic=name))
            continue
        got = normalize_type(entry.get("type"))
        if got != normalize_type(expected_type):
            issues.append(_issue(
                BAG_WRONG_TYPE, topic=name,
                detail="expected %s got %s" % (expected_type, got)))
        if int(entry.get("count") or 0) <= 0:
            issues.append(_issue(BAG_EMPTY_STREAM, topic=name))
        if _role(entry) in REFERENCE_ROLES:
            issues.append(_issue(
                BAG_REFERENCE_LEAK, topic=name,
                detail="required online topic marked as reference"))
    return issues


def check_segmentation_inputs(manifest):
    by_name = _topics_by_name(manifest)
    for name, expected_type in SEGMENTATION_TOPICS:
        entry = by_name.get(name)
        if entry is None:
            continue
        if int(entry.get("count") or 0) <= 0:
            continue
        got = normalize_type(entry.get("type"))
        if got != normalize_type(expected_type):
            return [_issue(
                BAG_WRONG_TYPE, topic=name,
                detail="expected %s got %s" % (expected_type, got))]
        return []
    return [_issue(
        BAG_SEGMENTATION_INPUT_MISSING,
        detail="need non-empty %s" % " or ".join(
            name for name, _type in SEGMENTATION_TOPICS))]


def check_time_windows(manifest, min_duration_sec=MIN_DURATION_SEC):
    issues = []
    windows = []
    for entry in manifest.get("topics") or []:
        if _role(entry) not in ONLINE_ROLES:
            continue
        name = str(entry.get("name") or "")
        stamps = list(entry.get("stamps_sec") or [])
        if stamps:
            prev = None
            for stamp in stamps:
                try:
                    value = float(stamp)
                except (TypeError, ValueError):
                    issues.append(_issue(
                        BAG_STAMP_NON_MONOTONIC, topic=name,
                        detail="non-numeric stamp"))
                    break
                if prev is not None and value < prev:
                    issues.append(_issue(BAG_STAMP_NON_MONOTONIC, topic=name))
                    break
                prev = value
            t0 = float(stamps[0])
            t1 = float(stamps[-1])
        else:
            try:
                t0 = float(entry.get("t_start_sec"))
                t1 = float(entry.get("t_end_sec"))
            except (TypeError, ValueError):
                continue
            if t1 < t0:
                issues.append(_issue(BAG_STAMP_NON_MONOTONIC, topic=name))
                continue
        windows.append((name, t0, t1))

    if not windows:
        return issues

    overlap_start = max(item[1] for item in windows)
    overlap_end = min(item[2] for item in windows)
    if overlap_end <= overlap_start:
        issues.append(_issue(
            BAG_WINDOWS_NO_OVERLAP,
            detail="online observation windows do not overlap"))
        return issues
    duration = overlap_end - overlap_start
    declared = manifest.get("duration_sec")
    if declared is not None:
        try:
            duration = min(duration, float(declared))
        except (TypeError, ValueError):
            pass
    if duration < float(min_duration_sec):
        issues.append(_issue(
            BAG_DURATION_SHORT,
            detail="overlap %.3fs < %.3fs" % (duration, min_duration_sec)))
    return issues


def check_frames_and_tf(manifest):
    issues = []
    frames = set(str(f) for f in (manifest.get("frame_ids") or []) if f)
    for entry in manifest.get("topics") or []:
        frame = entry.get("frame_id")
        if frame:
            frames.add(str(frame))
    for required in REQUIRED_FRAMES:
        if required not in frames:
            issues.append(_issue(BAG_MISSING_FRAME, detail=required))

    coverage = list(manifest.get("tf_coverage") or [])
    if not coverage:
        issues.append(_issue(
            BAG_TF_GAP, detail="tf_coverage is empty"))
        return issues
    by_pair = {}
    for row in coverage:
        parent = str(row.get("parent") or "")
        child = str(row.get("child") or "")
        present = bool(row.get("present", row.get("ok", False)))
        by_pair.setdefault((parent, child), []).append(present)
    for parent, child in REQUIRED_TF_PAIRS:
        flags = by_pair.get((parent, child))
        if not flags or not all(flags):
            issues.append(_issue(
                BAG_TF_GAP,
                detail="%s -> %s missing at an acquisition stamp"
                % (parent, child)))
    return issues


def check_reference_isolation(manifest):
    issues = []
    if bool(manifest.get("reference_published_to_online")):
        issues.append(_issue(
            BAG_REFERENCE_LEAK,
            detail="offline labels published onto online algorithm topics"))
    online_names = set(name for name, _type in REQUIRED_ONLINE_TOPICS)
    online_names.update(name for name, _type in SEGMENTATION_TOPICS)
    online_names.add(DETECTOR_OUTPUT_TOPIC[0])
    for entry in manifest.get("topics") or []:
        name = str(entry.get("name") or "")
        role = _role(entry)
        if name in REFERENCE_TOPIC_NAMES and role in ONLINE_ROLES:
            issues.append(_issue(
                BAG_REFERENCE_LEAK, topic=name,
                detail="eval/spawner geometry listed as online input"))
        if role in REFERENCE_ROLES and name in online_names:
            issues.append(_issue(
                BAG_REFERENCE_LEAK, topic=name,
                detail="reference occupying an online topic name"))
        if name == "/luggage/current_box" and role in ONLINE_ROLES:
            issues.append(_issue(
                BAG_REFERENCE_LEAK, topic=name,
                detail="current_box must be task_state, not online geometry"))
    refs = manifest.get("reference") or {}
    if bool(refs.get("publish_to_online")):
        issues.append(_issue(
            BAG_REFERENCE_LEAK,
            detail="reference.publish_to_online is true"))
    for name in refs.get("topics") or []:
        if str(name) in online_names:
            issues.append(_issue(
                BAG_REFERENCE_LEAK, topic=str(name),
                detail="reference topic collides with online input"))
    return issues


def check_coverage(manifest):
    issues = []
    coverage = manifest.get("coverage") or {}
    sizes = list(coverage.get("sizes") or [])
    if len(sizes) < MIN_SIZES:
        issues.append(_issue(
            BAG_COVERAGE_INSUFFICIENT,
            detail="need >= %d luggage sizes, got %d" % (MIN_SIZES, len(sizes))))
    placements = coverage.get("placements_per_size") or {}
    if isinstance(placements, dict):
        for size in sizes:
            n = int(placements.get(size) or 0)
            if n < MIN_PLACEMENTS_PER_SIZE:
                issues.append(_issue(
                    BAG_COVERAGE_INSUFFICIENT,
                    detail="%s placements %d < %d"
                    % (size, n, MIN_PLACEMENTS_PER_SIZE)))
    else:
        try:
            n = int(placements)
        except (TypeError, ValueError):
            n = 0
        if n < MIN_PLACEMENTS_PER_SIZE:
            issues.append(_issue(
                BAG_COVERAGE_INSUFFICIENT,
                detail="placements_per_size %d < %d"
                % (n, MIN_PLACEMENTS_PER_SIZE)))
    settled = int(coverage.get("settled_observations") or 0)
    if settled < MIN_SETTLED:
        issues.append(_issue(
            BAG_COVERAGE_INSUFFICIENT,
            detail="settled_observations %d < %d" % (settled, MIN_SETTLED)))
    heights = [
        float(v) for v in (coverage.get("platform_heights_m") or [])
        if v is not None]
    if not any(abs(h - LEGACY_PLATFORM_Z_M) > 0.02 for h in heights):
        issues.append(_issue(
            BAG_COVERAGE_INSUFFICIENT,
            detail="need a platform height other than %.2f m"
            % LEGACY_PLATFORM_Z_M))
    if not bool(coverage.get("support_visible")):
        issues.append(_issue(
            BAG_COVERAGE_INSUFFICIENT, detail="missing visible-support cases"))
    if not bool(coverage.get("support_occluded")):
        issues.append(_issue(
            BAG_COVERAGE_INSUFFICIENT,
            detail="missing support-occluded cases"))
    return issues


def check_accuracy_claim(manifest, claim_accuracy=False):
    """Accuracy is never granted by this checker."""
    claimed = bool(claim_accuracy) or bool(
        manifest.get("claim_gate5_accuracy"))
    if claimed:
        return [_issue(
            BAG_GATE5_ACCURACY_NOT_CLAIMED,
            detail="PF-A2 refuses Gate 5 accuracy without a real bag and "
                   "independent references")]
    return []


def check_manifest(manifest, min_duration_sec=MIN_DURATION_SEC,
                   claim_accuracy=False):
    """Return a dict: ready, reasons, gate5_accuracy, issues."""
    issues = []
    issues.extend(check_schema(manifest))
    schema_fatal = any(i["code"] == BAG_SCHEMA_INVALID for i in issues)
    if not schema_fatal:
        issues.extend(check_required_topics(manifest))
        issues.extend(check_segmentation_inputs(manifest))
        issues.extend(check_time_windows(manifest, min_duration_sec))
        issues.extend(check_frames_and_tf(manifest))
        issues.extend(check_reference_isolation(manifest))
        issues.extend(check_coverage(manifest))
    issues.extend(check_accuracy_claim(manifest, claim_accuracy=claim_accuracy))
    reasons = []
    for issue in issues:
        if issue["code"] not in reasons:
            reasons.append(issue["code"])
    ready = not issues
    return {
        "ready": ready,
        "gate5_accuracy": "not_claimed",
        "reasons": reasons,
        "issues": issues,
        "schema": SCHEMA_ID,
    }


def valid_fixture_manifest():
    """Deterministic ready manifest for synthetic tests. Not a real bag."""
    stamps = [10.0 + 0.2 * i for i in range(40)]
    topics = []
    for name, ros_type in REQUIRED_ONLINE_TOPICS:
        frame_id = "camera_depth_optical_frame"
        if name in ("/joint_states", "/tf", "/tf_static",
                    "/luggage/preprocessed/status"):
            frame_id = "world" if name != "/joint_states" else ""
        topics.append({
            "name": name,
            "type": ros_type,
            "role": "online",
            "count": len(stamps),
            "t_start_sec": stamps[0],
            "t_end_sec": stamps[-1],
            "stamps_sec": stamps,
            "frame_id": frame_id,
        })
    topics.append({
        "name": "/luggage/semantic/mask",
        "type": "sensor_msgs/msg/Image",
        "role": "online",
        "count": len(stamps),
        "t_start_sec": stamps[0],
        "t_end_sec": stamps[-1],
        "stamps_sec": stamps,
        "frame_id": "camera_depth_optical_frame",
    })
    topics.append({
        "name": DETECTOR_OUTPUT_TOPIC[0],
        "type": DETECTOR_OUTPUT_TOPIC[1],
        "role": "output",
        "count": len(stamps),
        "t_start_sec": stamps[0],
        "t_end_sec": stamps[-1],
        "stamps_sec": stamps,
        "frame_id": "world",
    })
    topics.append({
        "name": "/luggage/current_box",
        "type": "std_msgs/msg/String",
        "role": "task_state",
        "count": 3,
        "t_start_sec": stamps[0],
        "t_end_sec": stamps[-1],
        "frame_id": "",
    })
    topics.append({
        "name": "/luggage/eval/reconstructed_box",
        "type": "std_msgs/msg/String",
        "role": "reference",
        "count": 30,
        "t_start_sec": stamps[0],
        "t_end_sec": stamps[-1],
        "frame_id": "world",
    })
    tf_coverage = [
        {"stamp_sec": stamps[0], "parent": "world",
         "child": "camera_depth_optical_frame", "present": True},
        {"stamp_sec": stamps[-1], "parent": "world",
         "child": "camera_depth_optical_frame", "present": True},
    ]
    return {
        "schema": SCHEMA_ID,
        "bag_path": None,
        "duration_sec": stamps[-1] - stamps[0],
        "topics": topics,
        "frame_ids": ["world", "camera_depth_optical_frame", "elfin_base_link"],
        "tf_coverage": tf_coverage,
        "coverage": {
            "sizes": ["carryon", "standard", "large"],
            "placements_per_size": {
                "carryon": 3, "standard": 3, "large": 3,
            },
            "settled_observations": 30,
            "platform_heights_m": [0.86, 0.62],
            "support_visible": True,
            "support_occluded": True,
        },
        "reference": {
            "topics": ["/luggage/eval/reconstructed_box"],
            "publish_to_online": False,
            "established": False,
        },
        "claim_gate5_accuracy": False,
    }


def mutate_fixture(kind):
    """Return a copy of the valid fixture with one injected defect."""
    manifest = deepcopy(valid_fixture_manifest())
    if kind == "missing_topic":
        manifest["topics"] = [
            t for t in manifest["topics"]
            if t["name"] != "/luggage/preprocessed/status"]
    elif kind == "wrong_type":
        for topic in manifest["topics"]:
            if topic["name"] == "/joint_states":
                topic["type"] = "std_msgs/msg/String"
    elif kind == "empty_stream":
        for topic in manifest["topics"]:
            if topic["name"] == "/tf":
                topic["count"] = 0
                topic["stamps_sec"] = []
    elif kind == "non_overlap":
        for topic in manifest["topics"]:
            if topic["name"] == "/joint_states":
                topic["t_start_sec"] = 80.0
                topic["t_end_sec"] = 90.0
                topic["stamps_sec"] = [80.0, 85.0, 90.0]
    elif kind == "missing_tf":
        manifest["tf_coverage"] = [
            {"stamp_sec": 10.0, "parent": "world",
             "child": "camera_depth_optical_frame", "present": False},
        ]
    elif kind == "reference_leak":
        manifest["reference"]["publish_to_online"] = True
        manifest["reference_published_to_online"] = True
    elif kind == "non_monotonic":
        for topic in manifest["topics"]:
            if topic["name"].endswith("/color/image"):
                topic["stamps_sec"] = [10.0, 9.5, 11.0]
    elif kind == "claim_accuracy":
        manifest["claim_gate5_accuracy"] = True
    else:
        raise ValueError("unknown fixture kind: %s" % kind)
    return manifest


def load_manifest(path):
    with open(path, "r") as handle:
        return json.load(handle)


def dump_report(report, path):
    with open(path, "w") as handle:
        json.dump(report, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return path
