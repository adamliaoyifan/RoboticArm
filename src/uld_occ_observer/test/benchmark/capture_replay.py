"""Failure replay executes the product again. Reading a file back is not a replay.

Missing fields stay missing. This module does not invent a status, a drop
count, an identity, or a hash. A trace is complete only when it matches the
caller's expected operations. The trace file does not get to declare itself
complete.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import subprocess
import sys
from pathlib import Path

TRACE_FIELDS = (
    "schema_version", "run_id", "phase", "event_seq", "event",
    "mono_ns", "pid", "tid", "thread_cpu_ns", "request_id", "status",
)
BUNDLE_FIELDS = (
    "observation_id", "epoch", "status", "reason", "sensor_id",
    "input_file", "input_sha256", "input_epoch", "geometry_file", "geometry_sha256",
    "config_file", "config_sha256", "command", "tf", "map_prefix",
)
# CLI replay is implemented for this status only. Other faults stay
# not_evaluated instead of being reported as a general replay.
REPLAY_WHITELIST = ("EPOCH_MISMATCH",)


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_events(path):
    events = []
    errors = []
    text = Path(path).read_text()
    for index, line in enumerate(text.splitlines()):
        if not line.strip():
            continue
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            errors.append("invalid_json:%d" % index)
            continue
        if not isinstance(event, dict):
            errors.append("not_object:%d" % index)
            continue
        for field in TRACE_FIELDS:
            if field not in event:
                errors.append("missing_field:%d:%s" % (index, field))
        events.append(event)
    return events, errors


def _finite_nonnegative(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    if isinstance(value, float) and not math.isfinite(value):
        return False
    return value >= 0


def expected_profile_trace(observation_id, run_id=None):
    """Boundaries a profile CLI frame must emit, in order, for this observation."""
    operations = [
        {"event": "integrate_start", "request_id": observation_id, "status": ""},
        {"event": "integrate_end", "request_id": observation_id, "status": "INTEGRATED"},
        {"event": "roi_begin", "request_id": observation_id, "status": ""},
        {"event": "roi_end", "request_id": observation_id},
        {"event": "surface_begin", "request_id": observation_id, "status": "miss"},
        {"event": "surface_end", "request_id": observation_id, "status": "miss"},
        {"event": "surface_begin", "request_id": observation_id, "status": "hit"},
        {"event": "surface_end", "request_id": observation_id, "status": "hit"},
    ]
    expected = {"operations": operations}
    if run_id is not None:
        expected["run_id"] = run_id
    return expected


def certify_trace(path, expected=None):
    """Match a trace to the expected operation set. Replay stays false."""
    try:
        events, errors = _load_events(path)
    except OSError:
        return {"capture_complete": False, "replay_possible": False, "errors": ["unreadable"], "events": 0}
    if not isinstance(expected, dict) or not expected.get("operations"):
        errors.append("missing_expected_operations")
        operations = []
    else:
        operations = expected["operations"]
        if not isinstance(operations, list) or not operations:
            errors.append("missing_expected_operations")
            operations = []
    footers = [event for event in events if event.get("event") == "trace_footer"]
    if len(footers) != 1 or not events or events[-1].get("event") != "trace_footer":
        errors.append("footer")
    elif str(footers[0].get("status")) != "0":
        errors.append("dropped")
    run_ids = {event.get("run_id") for event in events}
    if len(run_ids) != 1:
        errors.append("run_mismatch")
    elif isinstance(expected, dict) and expected.get("run_id") not in (None, next(iter(run_ids))):
        errors.append("run_mismatch")
    sequences = []
    for index, event in enumerate(events):
        for key in ("mono_ns", "thread_cpu_ns"):
            if key in event and not _finite_nonnegative(event[key]):
                errors.append("bad_time:%d" % index)
        seq = event.get("event_seq")
        if isinstance(seq, int) and not isinstance(seq, bool) and seq >= 1:
            sequences.append(seq)
        elif "event_seq" in event:
            errors.append("bad_seq:%d" % index)
    if sequences != list(range(1, len(events) + 1)):
        errors.append("gap_or_reorder")
    body = [event for event in events if event.get("event") != "trace_footer"]
    if len(body) != len(operations):
        errors.append("operation_set")
    else:
        for index, (event, want) in enumerate(zip(body, operations)):
            if not isinstance(want, dict) or event.get("event") != want.get("event"):
                errors.append("operation_mismatch:%d" % index)
            elif event.get("request_id") != want.get("request_id") or not want.get("request_id"):
                errors.append("identity_mismatch:%d" % index)
            elif "status" in want and event.get("status") != want.get("status"):
                errors.append("status_mismatch:%d" % index)
    return {
        "capture_complete": not errors,
        "replay_possible": False,
        "errors": errors,
        "events": len(events),
    }


def _bundle(path):
    document = json.loads(Path(path).read_text())
    if not isinstance(document, dict):
        raise ValueError("bundle is not an object")
    return document


def validate_bundle(path):
    """Structural check. A complete document is still not a replay."""
    try:
        document = _bundle(path)
    except (OSError, json.JSONDecodeError, ValueError):
        return {"structurally_complete": False, "missing": ["unreadable"], "file_reread_is_replay": False}
    missing = [field for field in BUNDLE_FIELDS if field not in document or document[field] in (None, "")]
    tf = document.get("tf")
    if not isinstance(tf, dict) or "present" not in tf:
        missing.append("tf")
    elif tf.get("present") is True and "stamp_ns" not in tf:
        missing.append("tf.stamp_ns")
    prefix = document.get("map_prefix")
    if not isinstance(prefix, dict) or "tree_nodes" not in prefix:
        missing.append("map_prefix")
    command = document.get("command")
    if not isinstance(command, list) or len(command) < 2:
        missing.append("command")
    return {
        "structurally_complete": not missing,
        "missing": missing,
        "file_reread_is_replay": False,
    }


def _hash_problems(document):
    missing = []
    input_path = Path(document["input_file"])
    if not input_path.is_file() or sha256_file(input_path) != document["input_sha256"]:
        missing.append("input_sha256")
    try:
        import yaml
        input_epoch = yaml.safe_load(input_path.read_text())["epoch"]
    except (OSError, KeyError, TypeError, Exception):
        input_epoch = None
    if input_epoch != document["input_epoch"]:
        missing.append("input_epoch")
    for label, key in (("geometry_file", "geometry_sha256"), ("config_file", "config_sha256")):
        if not Path(document[label]).is_file() or sha256_file(document[label]) != document[key]:
            missing.append(key)
    return missing


def _replay_child(bundle_path, work_dir):
    """New process. Reads the bundle and input from disk and runs the CLI."""
    work = Path(work_dir)
    work.mkdir(parents=True, exist_ok=True)
    document = _bundle(bundle_path)
    problems = _hash_problems(document)
    observed_path = work / "observed.json"
    if problems:
        observed_path.write_text(json.dumps({
            "executed": False, "pid": os.getpid(), "missing": problems,
        }) + "\n")
        return 0
    import yaml
    input_path = Path(document["input_file"])
    raw = yaml.safe_load(input_path.read_text())
    transform = raw.get("transform") if isinstance(raw, dict) else None
    stamp = transform.get("stamp_ns") if isinstance(transform, dict) else None
    manifest = {
        "case_id": "replay-%s" % document["observation_id"],
        "mode": "off",
        "surface": False,
        "state_digest": False,
        "frames": [{
            "path": os.path.relpath(input_path, work),
            "sha256": document["input_sha256"],
            "warmup": False,
            "expect_status": document["status"],
        }],
    }
    manifest_path = work / "manifest.yaml"
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False))
    out = work / "samples.yaml"
    command = profile_command(
        document["command"][0], document["geometry_file"], document["config_file"],
        str(manifest_path), str(out))
    completed = subprocess.run(command, cwd=work, text=True, capture_output=True)
    payload = {"executed": True, "pid": os.getpid(), "returncode": completed.returncode}
    if completed.returncode != 0 or not out.is_file():
        payload["stderr"] = completed.stderr[-500:]
        observed_path.write_text(json.dumps(payload) + "\n")
        return 0
    row = next(yaml.safe_load_all(out.read_text()))
    receipt = row.get("receipt") if isinstance(row.get("receipt"), dict) else {}
    payload.update({
        "status": receipt.get("status"),
        "reason": receipt.get("status"),
        "observation_id": receipt.get("observation_id"),
        "epoch": receipt.get("epoch") if "epoch" in receipt else None,
        "sensor_id": receipt.get("sensor_id"),
        "tree_nodes": row.get("tree_nodes"),
        "content_sha256": row.get("probability_tree_sha256"),
        "tf": {"present": stamp is not None, "stamp_ns": stamp},
    })
    observed_path.write_text(json.dumps(payload) + "\n")
    return 0


def replay_failure(bundle_path, work_dir):
    """Re-execute in a child process and compare the persisted identity."""
    structure = validate_bundle(bundle_path)
    if not structure["structurally_complete"]:
        return {
            "replay_possible": False,
            "executed": False,
            "file_reread_is_replay": False,
            "missing": structure["missing"],
            "fault_support": "not_evaluated",
        }
    document = _bundle(bundle_path)
    if document.get("status") not in REPLAY_WHITELIST:
        return {
            "replay_possible": False,
            "executed": False,
            "file_reread_is_replay": False,
            "fault_support": "not_evaluated",
            "status": "not_evaluated",
        }
    problems = _hash_problems(document)
    if problems:
        return {
            "replay_possible": False, "executed": False, "file_reread_is_replay": False,
            "missing": problems, "fault_support": "whitelist",
        }
    expected_command = profile_command(
        document["command"][0], document["geometry_file"], document["config_file"], None, None)[:5]
    if list(document["command"]) != expected_command:
        return {
            "replay_possible": False, "executed": False, "file_reread_is_replay": False,
            "missing": ["command"], "fault_support": "whitelist",
        }
    work = Path(work_dir)
    if work.exists() and any(work.iterdir()):
        return {
            "replay_possible": False, "executed": False, "file_reread_is_replay": False,
            "missing": ["clean_workdir"], "fault_support": "whitelist",
        }
    work.mkdir(parents=True, exist_ok=True)
    completed = subprocess.run(
        [sys.executable, __file__, "--replay-child", str(bundle_path), str(work)],
        text=True, capture_output=True)
    observed_path = work / "observed.json"
    if completed.returncode != 0 or not observed_path.is_file():
        return {
            "replay_possible": False, "executed": False, "file_reread_is_replay": False,
            "fault_support": "whitelist", "stderr": completed.stderr[-500:],
        }
    observed = json.loads(observed_path.read_text())
    if not observed.get("executed"):
        return {
            "replay_possible": False, "executed": False, "file_reread_is_replay": False,
            "missing": observed.get("missing", []), "fault_support": "whitelist",
            "observed_pid": observed.get("pid"),
        }
    mismatches = []
    for field in ("status", "observation_id", "epoch", "sensor_id", "reason"):
        if observed.get(field) != document.get(field):
            mismatches.append(field)
    prefix = document["map_prefix"]
    if observed.get("tree_nodes") != prefix.get("tree_nodes"):
        mismatches.append("tree_nodes")
    if "content_sha256" in prefix and observed.get("content_sha256") != prefix.get("content_sha256"):
        mismatches.append("content_sha256")
    tf = document["tf"]
    observed_tf = observed.get("tf") if isinstance(observed.get("tf"), dict) else {}
    if observed_tf.get("present") != tf.get("present"):
        mismatches.append("tf_present")
    if tf.get("present") is True and observed_tf.get("stamp_ns") != tf.get("stamp_ns"):
        mismatches.append("tf_stamp")
    return {
        "replay_possible": not mismatches and observed.get("status") not in (None, ""),
        "executed": True,
        "file_reread_is_replay": False,
        "fault_support": "whitelist",
        "mismatches": mismatches,
        "observed": observed,
    }


def profile_command(cli, geometry, config, manifest=None, out=None):
    command = [cli, "--geometry", geometry, "--config", config]
    if manifest is not None:
        command.extend(["--manifest", str(manifest), "--out", str(out)])
    return command


if __name__ == "__main__":
    if len(sys.argv) == 4 and sys.argv[1] == "--replay-child":
        raise SystemExit(_replay_child(sys.argv[2], sys.argv[3]))
    raise SystemExit(2)
