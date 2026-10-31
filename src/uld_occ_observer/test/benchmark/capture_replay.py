"""Failure replay executes the product again. Reading a file back is not a replay.

Missing fields stay missing. This module does not invent a status, a drop
count, an identity, or a hash.
"""
from __future__ import annotations

import hashlib
import json
import os
import subprocess
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


def certify_trace(path):
    """Trace completeness. This does not re-execute the product and is not a replay."""
    try:
        events, errors = _load_events(path)
    except OSError:
        return {"capture_complete": False, "replay_possible": False, "errors": ["unreadable"]}
    footers = [event for event in events if event.get("event") == "trace_footer"]
    if len(footers) != 1:
        errors.append("footer")
    else:
        dropped = footers[0].get("status")
        if dropped != "0":
            errors.append("dropped")
    sequences = []
    for index, event in enumerate(events):
        if event.get("event") in ("integrate_start", "integrate_end") and not event.get("request_id"):
            errors.append("missing_identity:%d" % index)
        seq = event.get("event_seq")
        if isinstance(seq, int) and seq >= 1:
            sequences.append(seq)
        elif "event_seq" in event:
            errors.append("bad_seq:%d" % index)
    if sequences and sequences != list(range(sequences[0], sequences[0] + len(sequences))):
        errors.append("gap_or_reorder")
    if len(sequences) != len(events):
        errors.append("incomplete_sequence")
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


def replay_failure(bundle_path, work_dir):
    """Re-run the saved command in a clean directory and compare the new receipt.

    The comparison values are the fields stored from the first execution.
    This function does not fill a missing status.
    """
    structure = validate_bundle(bundle_path)
    if not structure["structurally_complete"]:
        return {
            "replay_possible": False,
            "executed": False,
            "file_reread_is_replay": False,
            "missing": structure["missing"],
        }
    document = _bundle(bundle_path)
    input_path = Path(document["input_file"])
    if not input_path.is_file() or sha256_file(input_path) != document["input_sha256"]:
        return {"replay_possible": False, "executed": False, "file_reread_is_replay": False,
                "missing": ["input_sha256"]}
    import yaml as yaml_input
    try:
        input_epoch = yaml_input.safe_load(input_path.read_text())["epoch"]
    except (OSError, yaml_input.YAMLError, KeyError, TypeError):
        return {"replay_possible": False, "executed": False, "file_reread_is_replay": False,
                "missing": ["input_epoch"]}
    if input_epoch != document["input_epoch"]:
        return {"replay_possible": False, "executed": False, "file_reread_is_replay": False,
                "missing": ["input_epoch"]}
    for label, key in (("geometry_file", "geometry_sha256"), ("config_file", "config_sha256")):
        if not Path(document[label]).is_file() or sha256_file(document[label]) != document[key]:
            return {"replay_possible": False, "executed": False, "file_reread_is_replay": False,
                    "missing": [key]}
    expected_command = profile_command(
        document["command"][0], document["geometry_file"], document["config_file"], None, None)[:5]
    if list(document["command"]) != expected_command:
        return {"replay_possible": False, "executed": False, "file_reread_is_replay": False,
                "missing": ["command"]}
    work = Path(work_dir)
    if any(work.iterdir()) if work.exists() else False:
        return {"replay_possible": False, "executed": False, "file_reread_is_replay": False,
                "missing": ["clean_workdir"]}
    work.mkdir(parents=True, exist_ok=True)
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
    import yaml
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False))
    out = work / "samples.yaml"
    command = profile_command(
        document["command"][0], document["geometry_file"], document["config_file"],
        str(manifest_path), str(out))
    completed = subprocess.run(command, cwd=work, text=True, capture_output=True)
    if completed.returncode != 0 or not out.exists():
        return {
            "replay_possible": False,
            "executed": True,
            "file_reread_is_replay": False,
            "returncode": completed.returncode,
            "stderr": completed.stderr[-500:],
        }
    import yaml as yaml_mod
    row = next(yaml_mod.safe_load_all(out.read_text()))
    receipt = row.get("receipt") if isinstance(row.get("receipt"), dict) else {}
    observed = {
        "status": receipt.get("status"),
        "observation_id": receipt.get("observation_id"),
        "epoch": (receipt.get("epoch") if "epoch" in receipt else None),
        "tree_nodes": row.get("tree_nodes"),
    }
    # Do not treat a missing observed field as equal to the saved field.
    mismatches = []
    if observed["status"] != document["status"]:
        mismatches.append("status")
    if observed["observation_id"] != document["observation_id"]:
        mismatches.append("observation_id")
    if observed["epoch"] != document["epoch"]:
        mismatches.append("epoch")
    if observed["tree_nodes"] != document["map_prefix"].get("tree_nodes"):
        mismatches.append("tree_nodes")
    return {
        "replay_possible": not mismatches and observed["status"] not in (None, ""),
        "executed": True,
        "file_reread_is_replay": False,
        "mismatches": mismatches,
        "observed": observed,
    }


def profile_command(cli, geometry, config, manifest=None, out=None):
    command = [cli, "--geometry", geometry, "--config", config]
    if manifest is not None:
        command.extend(["--manifest", str(manifest), "--out", str(out)])
    return command
