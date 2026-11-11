#!/usr/bin/env python3
"""Offline v2 contract baseline and evidence-backed migration readiness gate.

Exit 0: requested gate passed. Exit 1: malformed input / failed check.
Exit 2: offline checks passed, but production migration is incomplete.
Never launches ROS, queries hardware, or modifies product inputs.
"""

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/luggage_description"))

from luggage_description.frame_contracts import (  # noqa: E402
    ContractError, require_horizontal, unit_quaternion, validate_atlas,
    validate_pose, validate_surface,
)
from luggage_description.frame_transforms import (  # noqa: E402
    box_to_tool_target, rotate_vector, transform_pose,
)

DEFAULT_SUITE = ROOT / "src/luggage_description/test/data/frame_contract_cases.json"
DEFAULT_INVENTORY = ROOT / "docs/architecture/frame_interfaces.json"
# Inventory v1 freezes these reviewed obligations. Deleting a row cannot turn
# an incomplete migration green; extra future boundaries are allowed.
REQUIRED_INTERFACE_IDS = {f"F{i:02d}" for i in range(1, 27)}
CONTRACT_SOURCES = (
    "src/luggage_description/luggage_description/frame_contracts.py",
    "src/luggage_description/luggage_description/frame_transforms.py",
)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def load_json(path):
    if path.stat().st_size > 1024 * 1024:
        raise ValueError(f"input exceeds 1 MiB: {path}")
    def invalid_constant(value):
        raise ValueError(f"nonfinite JSON constant: {value}")
    return json.loads(path.read_text(encoding="utf-8"),
                      object_pairs_hook=_unique_object, parse_constant=invalid_constant)


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def interface_digest(row):
    """Bind adoption evidence to the reviewed contract, not only source files."""
    fields = ("id", "producer", "consumer", "transport", "scope", "target", "sources", "checks")
    data = json.dumps({key: row[key] for key in fields}, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(data.encode("utf-8")).hexdigest()


def equivalent_rotation(a, b):
    a, b = unit_quaternion(a), unit_quaternion(b)
    return min(max(abs(x-y) for x, y in zip(a, b)),
               max(abs(x+y) for x, y in zip(a, b))) <= 1e-9


def case_input(case, suite):
    record = deepcopy(suite.get("records", {}).get(case.get("record"), {}))
    if "record" in case and case["record"] not in suite.get("records", {}):
        raise ValueError(f"unknown record: {case['record']}")
    record.update(case.get("overrides", {}))
    for key in case.get("remove", []):
        record.pop(key, None)
    transform = deepcopy(suite.get("transforms", {}).get(case.get("transform"), {}))
    if "transform" in case and case["transform"] not in suite.get("transforms", {}):
        raise ValueError(f"unknown transform: {case['transform']}")
    transform.update(case.get("transform_overrides", {}))
    return record, transform


def execute_case(case, suite, record, transform):
    op = case["op"]
    scene = suite["scene_revision"]
    if op == "pose":
        validate_pose(record, case["profile"], scene)
    elif op == "surface":
        validate_surface(record, scene, suite["atlas_identity"]["geometry_hash"],
                         suite["expected_map_revision"])
    elif op == "atlas":
        validate_atlas(record, suite["atlas_identity"])
    elif op == "horizontal":
        require_horizontal(case["quaternion_xyzw"])
    elif op == "transform":
        return transform_pose(record, transform)
    elif op == "tool_target":
        return box_to_tool_target(record, transform)
    elif op == "vector":
        return {"vector": list(rotate_vector(case["quaternion_xyzw"], case["vector"]))}
    elif op == "footprint":
        width, depth = case["size_wd"]
        corners = []
        for x, y in ((-width/2, -depth/2), (width/2, -depth/2),
                     (width/2, depth/2), (-width/2, depth/2)):
            corners.extend(rotate_vector(case["quaternion_xyzw"], (x, y, 0))[:2])
        return {"corners_xy": corners}
    elif op == "rotation_equivalence":
        return {"equivalent": equivalent_rotation(case["a"], case["b"])}
    elif op == "existing_base_helper":
        from luggage_description.scene_tf_config_utils import xyz_base_link_to_world
        config = {"robot": {"base_height": 0.86, "rotation_rpy": [0, 0, math.pi/2]},
                  "pedestal": {"enabled": False}}
        return {"position": xyz_base_link_to_world(config, case["point"])}
    else:
        raise ValueError(f"unsupported operation: {op}")
    return {"accepted": True}


def matches(expected, actual):
    for key, value in expected.items():
        if key not in actual:
            return False
        if key == "quaternion_xyzw":
            if not equivalent_rotation(value, actual[key]):
                return False
        elif isinstance(value, list):
            if (not isinstance(actual[key], (list, tuple)) or len(value) != len(actual[key])
                    or any(not math.isclose(a, b, abs_tol=1e-9, rel_tol=0.0)
                           for a, b in zip(value, actual[key]))):
                return False
        elif type(value) is not type(actual[key]) or value != actual[key]:
            return False
    return True


def run_cases(suite):
    if (type(suite.get("suite_version")) is not int or suite["suite_version"] != 1
            or not isinstance(suite.get("cases"), list) or not suite["cases"]):
        raise ValueError("suite_version=1 and nonempty cases required")
    ids = [case.get("id") for case in suite["cases"]]
    if any(not isinstance(value, str) or not value for value in ids) or len(set(ids)) != len(ids):
        raise ValueError("case IDs must be nonempty and unique")
    results = []
    for case in suite["cases"]:
        started = time.perf_counter()
        record, transform = {}, {}
        unexpected = False
        try:
            record, transform = case_input(case, suite)
            before = deepcopy((record, transform))
            actual = execute_case(case, suite, record, transform)
            if (record, transform) != before:
                raise AssertionError("operation mutated its input")
        except ContractError as exc:
            actual = exc.as_dict()
        except Exception as exc:  # preserve every unexpected case failure
            actual = {"exception": type(exc).__name__, "message": str(exc),
                      "traceback": traceback.format_exc()}
            unexpected = True
        expected = case.get("expect")
        try:
            passed = (not unexpected and isinstance(expected, dict) and bool(expected)
                      and matches(expected, actual))
        except (ValueError, TypeError, ContractError):
            passed = False
        results.append({"id": case["id"], "op": case.get("op"),
                        "input": {"record": record, "transform": transform, "case": case},
                        "expected": expected, "actual": actual,
                        "status": "pass" if passed else "fail",
                        "duration_sec": time.perf_counter() - started})
    return results


def inventory_report(inventory, case_ids, root=ROOT):
    if (type(inventory.get("inventory_version")) is not int or inventory["inventory_version"] != 1
            or type(inventory.get("contract_version")) is not int or inventory["contract_version"] != 2
            or not isinstance(inventory.get("interfaces"), list) or not inventory["interfaces"]):
        raise ValueError("nonempty interface inventory version 1 required")
    seen, pending, hashes = set(), [], {}
    for row in inventory["interfaces"]:
        identifier = row["id"]
        if not isinstance(identifier, str) or not identifier or identifier in seen:
            raise ValueError(f"duplicate/empty interface ID: {identifier}")
        seen.add(identifier)
        for key in ("producer", "consumer", "transport", "current", "target", "scope"):
            if not row.get(key):
                raise ValueError(f"{identifier}: missing {key}")
        checks = row.get("checks")
        if not checks or not set(checks) <= case_ids:
            raise ValueError(f"{identifier}: missing/unknown offline checks")
        sources = row.get("sources")
        if not sources:
            raise ValueError(f"{identifier}: no source pointers")
        proof_sources = list(dict.fromkeys([*sources, *CONTRACT_SOURCES]))
        for source in proof_sources:
            path = (root / source).resolve()
            if not path.is_relative_to(root.resolve()) or not path.is_file():
                raise ValueError(f"{identifier}: invalid source pointer {source}")
            hashes[source] = sha256(path)
        state = row.get("migration_state")
        if state not in ("pending", "verified", "isolated"):
            raise ValueError(f"{identifier}: invalid migration_state {state}")
        proof_path = row.get("runtime_evidence")
        reason = "runtime boundary adoption not verified"
        if state != "pending" and proof_path:
            path = (root / proof_path).resolve()
            if not path.is_relative_to((root / "docs/status/evidence").resolve()):
                raise ValueError(f"{identifier}: proof must be under docs/status/evidence")
            proof = load_json(path)
            required_kind = "isolation_check" if state == "isolated" else "runtime_boundary"
            valid = (proof.get("interface_id") == identifier and proof.get("status") == "pass"
                     and type(proof.get("contract_version")) is int and proof["contract_version"] == 2
                     and proof.get("interface_contract_sha256") == interface_digest(row)
                     and proof.get("kind") == required_kind
                     and bool(proof.get("command")) and bool(proof.get("observations"))
                     and type(proof.get("sample_count")) is int and proof["sample_count"] > 0
                     and all(proof.get("source_sha256", {}).get(p) == hashes[p] for p in proof_sources))
            if state == "isolated" and row["scope"] != "legacy":
                valid = False
            if valid:
                continue
            reason = "proof is incomplete, stale, or wrong kind (offline cases are not runtime proof)"
        pending.append({"id": identifier, "state": state, "reason": reason})
    missing = REQUIRED_INTERFACE_IDS - seen
    if missing:
        raise ValueError(f"inventory v1 is missing required boundaries: {sorted(missing)}")
    return {"interface_count": len(seen), "pending": pending, "source_sha256": hashes}


def git_output(*args):
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def write_json(path, value):
    data = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    if len(data.encode("utf-8")) > 1024 * 1024:
        raise ValueError(f"artifact exceeds 1 MiB: {path.name}")
    with path.open("x", encoding="utf-8") as handle:
        handle.write(data)
        handle.flush()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("baseline", "migration"), default="baseline")
    parser.add_argument("--suite", type=Path, default=DEFAULT_SUITE)
    parser.add_argument("--inventory", type=Path, default=DEFAULT_INVENTORY)
    parser.add_argument("--output", required=True, type=Path, help="new evidence directory; never overwritten")
    args = parser.parse_args(argv)
    try:
        args.output.mkdir(parents=True, exist_ok=False)
    except OSError as exc:
        print(f"refusing evidence output: {exc}", file=sys.stderr)
        return 1
    started = datetime.now(timezone.utc).isoformat()
    results, inventory, suite, report, errors = [], {}, {}, {}, []
    try:
        suite, inventory = load_json(args.suite), load_json(args.inventory)
        results = run_cases(suite)
        report = inventory_report(inventory, {case["id"] for case in results})
        if len(results) < 20:
            errors.append("baseline requires at least 20 nonempty deterministic cases")
    except Exception as exc:
        errors.append(f"{type(exc).__name__}: {exc}")
    failed = [case["id"] for case in results if case["status"] != "pass"]
    pending = report.get("pending", [])
    baseline_ok = not errors and not failed and bool(results) and bool(report)
    migration_ready = baseline_ok and not pending
    exit_code = 1 if not baseline_ok else (2 if args.mode == "migration" and not migration_ready else 0)
    summary = {"mode": args.mode, "baseline_pass": baseline_ok,
               "migration_ready": migration_ready, "case_count": len(results),
               "failed_cases": failed, "errors": errors,
               "interface_count": report.get("interface_count", 0), "pending_interfaces": pending,
               "runtime_execution": "not_evaluated", "exit_code": exit_code}
    tracked_sources = dict(report.get("source_sha256", {}))
    for path in (Path(__file__), args.suite, args.inventory,
                 ROOT / "src/luggage_description/luggage_description/frame_contracts.py",
                 ROOT / "src/luggage_description/luggage_description/frame_transforms.py",
                 ROOT / "src/luggage_description/luggage_description/scene_tf_config_utils.py"):
        if path.is_file():
            tracked_sources[str(path.resolve().relative_to(ROOT)) if path.resolve().is_relative_to(ROOT) else str(path)] = sha256(path)
    dirty = git_output("status", "--porcelain").splitlines()
    manifest = {"started_at": started, "completed_at": datetime.now(timezone.utc).isoformat(),
                "revision": git_output("rev-parse", "HEAD"), "dirty_count": len(dirty), "dirty_paths": dirty,
                "source_sha256": tracked_sources, "python": sys.version, "executable": sys.executable,
                "command": [sys.executable, str(Path(__file__).resolve()), *(argv if argv is not None else sys.argv[1:])],
                "profile": "offline_cpu_no_ros", "repetitions": 1, "seed": None,
                "capture_window": "one fully specified synchronous case",
                "first_failed_case": failed[0] if failed else None,
                "capture_complete": not errors, "replay_possible": not errors,
                "teardown": "not_applicable_no_sim_started",
                "artifacts": ["suite.json", "inventory.json", "cases.json", "summary.json", "manifest.json"]}
    for name, value in (("suite", suite), ("inventory", inventory), ("cases", results),
                        ("summary", summary), ("manifest", manifest)):
        write_json(args.output / f"{name}.json", value)
    print(json.dumps({"baseline_pass": baseline_ok, "migration_ready": migration_ready,
                      "cases": len(results), "failed_cases": failed, "errors": errors,
                      "pending_interfaces": len(pending), "evidence": str(args.output), "exit_code": exit_code}))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
