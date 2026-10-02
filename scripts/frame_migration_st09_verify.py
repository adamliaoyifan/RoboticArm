#!/usr/bin/env python3
"""ST09 evidence runner.

The runner owns evidence accounting, not planner decisions.  It creates output
directories exclusively, returns 0/1/2 for pass/failure/incomplete, and writes a
sample manifest computed from the acceptance manifest plus observed artifacts.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import time

CONFLICT_REASON_CODES = {
    "UNKNOWN_SCHEMA",
    "DATUM_CONFLICT",
    "IDENTITY_MISMATCH",
    "MIXED_SOURCE_CONFLICT",
    "OVERLAY_HASH_MISMATCH",
}
EXPECTED_SAMPLE_SCHEMA = "st09.sample/1"
EXPECTED_DATUM = "world"

REQUIRED_SAMPLE_KEYS = {
    "offline": (
        "geometry_frame_64",
        "deep_place_16",
        "yaw_ik_8_classes",
        "target_matrix_8",
        "attachment_negatives_9",
        "map_cases_6_x2",
        "common_negatives_per_profile",
        "mutations",
    ),
    "graph": (
        "performance",
        "startup_3_x2",
        "capture_replay",
    ),
}


def verification_outcome(commands, missing):
    """Execution/assertion failures take precedence over incomplete coverage.

    Only the migration checker's documented exit 2 means incomplete migration;
    pytest exit 2, timeout and every other nonzero command remain failures.
    """
    incomplete = [entry["name"] for entry in commands
                  if entry["name"] == "checker-migration" and entry.get("exit_code") == 2]
    failed = [entry["name"] for entry in commands
              if entry.get("exit_code") != 0 and entry["name"] not in incomplete]
    if failed:
        return "fail", failed
    return ("not_evaluated" if missing or incomplete else "pass"), failed


def load_acceptance_manifest(ws):
    path = Path(ws) / "st09_contracts" / "st09_acceptance_manifest.json"
    document = json.loads(path.read_text(encoding="utf-8"))
    return document


def required_samples_for_mode(acceptance, mode):
    inventory = acceptance.get("required_case_inventory") or {}
    keys = []
    if mode in ("offline", "all"):
        keys.extend(REQUIRED_SAMPLE_KEYS["offline"])
    if mode in ("graph", "all"):
        keys.extend(REQUIRED_SAMPLE_KEYS["graph"])
    if mode == "rehearsal":
        keys.extend(("capture_replay", "startup_3_x2"))
    required = []
    for key in keys:
        required.append({
            "sample_id": key,
            "requirement": inventory.get(key, ""),
            "artifact_globs": ["samples/%s.json" % key, "%s.json" % key],
            "expected_schema": EXPECTED_SAMPLE_SCHEMA,
            "expected_datum": EXPECTED_DATUM,
            "min_records": 1,
        })
    return required


def artifact_inventory(root):
    out = []
    root = Path(root)
    if not root.exists():
        return out
    for path in sorted(root.rglob("*")):
        if path.is_file():
            data = path.read_bytes()
            out.append({
                "path": str(path.relative_to(root)),
                "bytes": len(data),
                "sha256": hashlib.sha256(data).hexdigest(),
            })
    return out


def _canonical_sha256(value):
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def validate_sample_document(path, sample):
    """Validate an observed sample before it can satisfy coverage.

    The runner accepts only a versioned envelope with a matching sample id,
    stable identity, world datum, non-empty records, a records checksum, and
    complete joins. Bad content is reported as missing coverage rather than
    entering ``observed`` by filename alone.
    """
    try:
        document = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return False, "UNKNOWN_SCHEMA:%s" % error
    if not isinstance(document, dict) or document.get("schema") != sample["expected_schema"]:
        return False, "UNKNOWN_SCHEMA"
    if document.get("sample_id") != sample["sample_id"]:
        return False, "IDENTITY_MISMATCH"
    if document.get("datum") != sample["expected_datum"]:
        return False, "DATUM_CONFLICT"
    if document.get("source") == "mixed" or document.get("source_type") == "mixed":
        return False, "MIXED_SOURCE_CONFLICT"
    identity = document.get("identity")
    if not isinstance(identity, dict):
        return False, "IDENTITY_MISMATCH"
    for key in ("request_id", "cycle_id", "map_revision"):
        if key not in identity:
            return False, "IDENTITY_MISMATCH"
    records = document.get("records")
    if not isinstance(records, list) or len(records) < sample.get("min_records", 1):
        return False, "RECORD_COUNT_MISMATCH"
    expected_hash = document.get("records_sha256") or document.get("sha256")
    if expected_hash != _canonical_sha256(records):
        return False, "OVERLAY_HASH_MISMATCH"
    joins = document.get("joins")
    if not isinstance(joins, list) or not joins or any(not row.get("complete") for row in joins if isinstance(row, dict)):
        return False, "JOIN_INCOMPLETE"
    return True, "pass"


def compute_sample_manifest(acceptance, mode, out):
    required = required_samples_for_mode(acceptance, mode)
    artifacts = artifact_inventory(out)
    paths = {item["path"] for item in artifacts}
    artifact_by_path = {item["path"]: item for item in artifacts}
    observed, missing = [], []
    for sample in required:
        hits, rejected = [], []
        for candidate in sample["artifact_globs"]:
            if candidate in paths:
                ok, reason = validate_sample_document(Path(out) / candidate, sample)
                if ok:
                    artifact = artifact_by_path[candidate]
                    hits.append({
                        "path": candidate,
                        "bytes": artifact["bytes"],
                        "sha256": artifact["sha256"],
                    })
                else:
                    rejected.append({"path": candidate, "reason": reason})
        record = dict(sample)
        record["observed_artifacts"] = hits
        record["rejected_artifacts"] = rejected
        if hits:
            observed.append(record)
        else:
            missing.append(record)
    return {
        "schema": "st09.sample_manifest/1",
        "mode": mode,
        "required": required,
        "observed": observed,
        "missing": missing,
        "artifacts": artifacts,
    }


def write_sample_manifest(acceptance, mode, out):
    manifest = compute_sample_manifest(acceptance, mode, out)
    (Path(out) / "sample_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest


def removal_sensitivity_matrix(acceptance, mode):
    """Generate one removal case for every required stream/sample."""
    cases = []
    for sample in required_samples_for_mode(acceptance, mode):
        for artifact in sample["artifact_globs"]:
            cases.append({
                "case_id": "remove:%s:%s" % (sample["sample_id"], artifact),
                "sample_id": sample["sample_id"],
                "removed_artifact": artifact,
                "expected_status": "not_evaluated",
                "expected_exit_code": 2,
            })
    return {
        "schema": "st09.removal_sensitivity/1",
        "mode": mode,
        "case_count": len(cases),
        "cases": cases,
    }


def fault_replay_report(faults):
    """Replay frozen synthetic fault records by comparing reason/effect deltas."""
    results = []
    for fault in faults:
        expected = fault.get("expected") or {}
        observed = fault.get("observed") or {}
        expected_effects = expected.get("effects") or {}
        observed_effects = observed.get("effects") or {}
        effect_deltas = {
            key: observed_effects.get(key, 0) - expected_effects.get(key, 0)
            for key in sorted(set(expected_effects) | set(observed_effects))
        }
        pass_case = expected.get("reason") == observed.get("reason") and all(value == 0 for value in effect_deltas.values())
        results.append({
            "fault_id": fault.get("fault_id"),
            "expected_reason": expected.get("reason"),
            "observed_reason": observed.get("reason"),
            "effect_deltas": effect_deltas,
            "status": "pass" if pass_case else "fail",
        })
    return {
        "schema": "st09.fault_replay_report/1",
        "status": "pass" if all(row["status"] == "pass" for row in results) else "fail",
        "results": results,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("offline", "graph", "rehearsal", "all"))
    parser.add_argument("--ws", required=True, type=Path)
    parser.add_argument("--install", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--ros-domain-id", required=True, type=int)
    args = parser.parse_args()
    if not 0 <= args.ros_domain_id <= 232:
        parser.error("ROS domain must be between 0 and 232")
    ws, install, out = args.ws.resolve(), args.install.resolve(), args.out.resolve()
    if not (install / "setup.bash").is_file():
        parser.error("explicit install has no setup.bash")
    out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    git = lambda *argv: subprocess.check_output(["git", "-C", str(ws), *argv], text=True).strip()
    manifest = dict(schema="st09.verification/1", mode=args.mode,
                    started_at=datetime.now(timezone.utc).isoformat(), workspace=str(ws),
                    install=str(install), revision=git("rev-parse", "HEAD"),
                    dirty=git("status", "--porcelain"), ros_domain_id=args.ros_domain_id,
                    commands=[], missing=[], teardown="no processes launched", status="running")
    acceptance = load_acceptance_manifest(ws)
    clean_env = {key:value for key,value in os.environ.items()
                 if key not in ("AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH", "PYTHONPATH")}
    clean_env.update(ROS_DOMAIN_ID=str(args.ros_domain_id), ROS_LOG_DIR=str(out / "ros-logs"),
                     FRAME_MIGRATION_CASE_LOG=str(out / "cases.jsonl"))
    bootstrap = ('source /opt/ros/humble/setup.bash\nsource ' + shlex.quote(str(install / "setup.bash"))
                 + '\nexec "$@"')

    def save():
        (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")

    def run(name, argv, timeout=180):
        command = ["bash", "-c", bootstrap, "st09", *map(str, argv)]
        entry = dict(name=name, argv=command, deadline_seconds=timeout, started=time.monotonic())
        manifest["commands"].append(entry); save()
        with (out / (name + ".log")).open("x") as stream:
            try:
                result = subprocess.run(command, cwd=ws, env=clean_env, stdout=stream,
                                        stderr=subprocess.STDOUT, timeout=timeout)
                entry["exit_code"] = result.returncode
            except subprocess.TimeoutExpired:
                entry["exit_code"] = 124
        entry["elapsed_seconds"] = time.monotonic() - entry.pop("started")
        save()

    try:
        run("environment", ["python3", "-c",
            "import json,os,sys,luggage_description,luggage_gazebo; print(json.dumps(dict(python=sys.version,environment={k:os.environ.get(k) for k in ('ROS_DOMAIN_ID','ROS_DISTRO','RMW_IMPLEMENTATION','AMENT_PREFIX_PATH','PYTHONPATH')},description=luggage_description.__file__,gazebo=luggage_gazebo.__file__),indent=2))"])
        if args.mode in ("offline", "all"):
            run("tools", ["python3", "-m", "unittest", "discover", "-s", "src/luggage_gazebo/test", "-p", "test_frame_migration_tools.py", "-v"])
            run("rehearsal-evidence", ["python3", "-m", "unittest", "discover", "-s", "src/luggage_gazebo/test", "-p", "test_rehearsal_evidence.py", "-v"])
            run("execution", ["python3", "-m", "pytest", "src/luggage_planning/test/test_frame_migration_execution.py", "-q", "--tb=short"])
            run("contracts", ["python3", "-m", "unittest", "discover", "-s", "src/luggage_description/test", "-p", "test_frame_contracts.py", "-v"])
            for mode in ("baseline", "migration"):
                run("checker-" + mode, ["python3", "scripts/check_frame_contracts.py", "--mode", mode,
                                        "--output", out / ("checker-" + mode)])
            manifest["missing"].extend([
                "matched base/candidate affected-package regression and collected-node comparison",
                "all six driver adapters and scene/TF geometry boundaries",
                "behavioral mutation sensitivity and refreshed affected interface proofs"])
        if args.mode in ("graph", "all"):
            graph_out = out / "graph"
            run("graph-campaign", [
                "python3", "scripts/frame_migration_st09_graph.py",
                "--ws", ws, "--install", install, "--out", graph_out,
                "--ros-domain-id", args.ros_domain_id,
                "--runs", ",".join([
                    "healthy-1", "healthy-2", "healthy-3",
                    "fault:lease_expiry", "fault:restart", "fault:v1_bypass",
                    "fault:duplicate_provider", "fault:wrong_overlay",
                    "fault:stale_atlas"])],
                timeout=900)
            run("graph-isolation", [
                "python3", "scripts/frame_migration_st09_isolation.py",
                "--install", install,
                *[part for path in sorted(graph_out.glob("*.nodes.txt"))
                  for part in ("--graph", path)],
                "--output", graph_out / "f24-isolation.json"], timeout=300)
        if args.mode in ("rehearsal", "all"):
            manifest["missing"].append("immutable wired candidate and executable frozen capture configuration required before Gazebo rehearsal")
        samples = write_sample_manifest(acceptance, args.mode, out)
        manifest["sample_manifest"] = "sample_manifest.json"
        manifest["missing_samples"] = [item["sample_id"] for item in samples["missing"]]
        manifest["missing"].extend("missing sample: %s" % item["sample_id"]
                                   for item in samples["missing"])
        manifest["status"], manifest["failed_commands"] = verification_outcome(
            manifest["commands"], manifest["missing"])
    except Exception as error:
        import traceback
        manifest["status"] = "fail"
        manifest["execution_error"] = str(error)
        (out / "execution-error.log").write_text(traceback.format_exc())
    finally:
        manifest["elapsed_seconds"] = time.monotonic() - started
        manifest["completed_at"] = datetime.now(timezone.utc).isoformat()
        manifest["artifacts"] = artifact_inventory(out)
        save()
    print(json.dumps(dict(status=manifest["status"], missing=manifest["missing"], output=str(out))))
    return {"pass": 0, "fail": 1, "not_evaluated": 2}[manifest["status"]]


if __name__ == "__main__":
    raise SystemExit(main())
