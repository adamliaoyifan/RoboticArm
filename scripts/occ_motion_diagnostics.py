#!/usr/bin/env python3
"""Audit OCC motion evidence, run a gated native replay, or compare paired runs."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import statistics
import subprocess

SELECT = "/trajectory_selector/select_place_path_v2"
CASE_BYTES = 100 * 1024**2
RUN_BYTES = 2 * 1024**3
ROOT = Path(__file__).resolve().parents[1]


def read(path, default):
    return json.loads(path.read_text()) if path.is_file() else default


def sha(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest() if hasattr(
            hashlib, "file_digest") else _stream_hash(stream)


def _stream_hash(stream):
    digest = hashlib.sha256()
    for chunk in iter(lambda: stream.read(1024**2), b""):
        digest.update(chunk)
    return digest.hexdigest()


def ledger_pairs(folder):
    """Join one synchronous Select call at a time; reject ambiguous ledgers."""
    ledger = folder / "boundaries.jsonl"
    if not ledger.is_file():
        return []
    pairs, pending = [], None
    for line in ledger.read_text().splitlines():
        row = json.loads(line)
        boundary = row.get("boundary")
        if boundary not in {SELECT + ":request", SELECT + ":response"}:
            continue
        name = row["file"]
        path = folder / name
        if Path(name).name != name or not path.is_file() or (
            path.stat().st_size != row["bytes"] or not 0 < row["bytes"] <= CASE_BYTES
        ):
            raise ValueError(f"invalid Select capture: {name}")
        row["sha256"] = sha(path)
        if boundary.endswith(":request"):
            if pending is not None or not row.get("request_id"):
                raise ValueError("ambiguous Select request")
            pending = row
        else:
            if (pending is None or row.get("request_id") not in ("", None, pending["request_id"])
                    or row["monotonic_ns"] < pending["monotonic_ns"]):
                raise ValueError("unmatched Select response")
            pairs.append({"request": pending, "response": row})
            pending = None
    if pending is not None:
        raise ValueError("Select request has no captured response")
    return pairs


def summarize_rows(rows):
    selected = {}
    predicates = (
        ("anchor_collision", lambda r: bool(r.get("anchor_gate", {}).get("rejected"))),
        ("move_group_failure", lambda r: any(
            e.get("planner") == "joint" and e.get("error_code") != 1
            for e in r.get("edges", []))),
        ("retime_observed", lambda r: r.get("stage_timings_s", {}).get(
            "retime_and_interpolation", {}).get("n", 0) > 0),
        ("budget_exhausted", lambda r: r.get("reason") == "PLANNING_BUDGET_EXHAUSTED"),
    )
    for label, predicate in predicates:
        hits = [i for i, row in enumerate(rows) if predicate(row)]
        if hits:
            selected.setdefault(hits[-1] if label == "budget_exhausted" else hits[0], []).append(label)
    cases = []
    for i, labels in selected.items():
        r = rows[i]
        cases.append({"row": i, "labels": labels, "slot_index": r.get("slot_index"),
                      "reason": r.get("reason"), "first_failure": "UNRESOLVED",
                      "anchor_gate": r.get("anchor_gate"),
                      "failed_joint_edges": [e for e in r.get("edges", [])
                                             if e.get("planner") == "joint" and e.get("error_code") != 1]})
    return {"cases": cases, "reasons": dict(Counter(r.get("reason") for r in rows)),
            "cumulative_stage_timings_s": next(
                (r["stage_timings_s"] for r in reversed(rows) if r.get("stage_timings_s")), {}),
            "attempt_summary": {k: v for r in rows if r.get("reason") == "ATTEMPT_SUMMARY"
                                for k, v in r.items() if k.startswith("n_")}}


def audit(run):
    dumps = run / "dumps"
    missing = []
    try:
        pairs = ledger_pairs(dumps / "typed_service_boundaries")
    except (ValueError, KeyError, OSError) as error:
        missing.append(str(error))
        pairs = []
    if not pairs:
        missing.append("matched Select request and response CDR")
    elif len(pairs) != 1:
        missing.append("exactly one Select call for this replay entry point")
    cases = []
    for index, pair in enumerate(pairs):
        paths = dumps / f"box_{index:02d}_paths"
        result = read(paths / "candidates.json", {})
        request = read(paths / "planning_request.json", {})
        if not result.get("rows") or request.get("request_id") != pair["request"]["request_id"]:
            missing.append(f"Select {index}: candidate dump request identity")
            continue
        cases.append(summarize_rows(result.get("rows", [])))
    manifests = [read(p, {}) for p in sorted(dumps.glob("trial_*/replay/manifest.json"))]
    # Native replay requires a lossless, explicitly joined Select-entry scene.
    # A post-failure summary or a later per-edge scene cannot substitute.
    scene = dumps / "select_replay" / "planning_scene.cdr"
    binding = read(scene.with_name("identity.json"), {})
    if (not pairs or not scene.is_file() or not 0 < scene.stat().st_size <= CASE_BYTES
            or binding.get("select_request_sha256") != pairs[0]["request"]["sha256"]
            or binding.get("request_id") != pairs[0]["request"]["request_id"]
            or binding.get("scene_sha256") != sha(scene)):
        missing.append("lossless Select-entry PlanningScene CDR with request binding")
    if not (dumps / "runtime_identity.json").is_file():
        missing.append("runtime_identity.json")
    surfaces = list(dumps.glob("trial_*/replay/surface_2d.json"))
    if len(surfaces) != 1:
        missing.append("matching surface snapshot")
    if cases and cases[0]["cases"]:
        first = read(dumps / "box_00_paths/candidates.json", {})["rows"][0]
        if first.get("atlas", {}).get("loaded"):
            missing.append("native replay support for captured loaded atlas")
    return {"status": "NOT_EVALUATED" if missing else "READY", "missing": missing,
            "historical_identity": read(run / "t0_manifest.json", {}),
            "select_pairs": pairs, "archive_manifests": manifests, "selections": cases,
            "scope": "archived observations; no new planner samples"}


def compare(path):
    """Only compare certified, identity-matched samples, in acquisition order."""
    groups = {}
    for row in read(path, []):
        groups.setdefault(row["case"], []).append(row)
    outcomes = {}
    for case, rows in groups.items():
        expected = ["baseline", "candidate"] * 10
        if ([r["arm"] for r in rows] != expected or not rows[0].get("input_identity")
                or any(r.get("input_identity") != rows[0]["input_identity"]
                       or r.get("capture_complete") is not True for r in rows)):
            outcomes[case] = {"status": "NOT_EVALUATED", "reason": "sample schedule or identity"}
            continue
        summary = {}
        violations = sum(r.get("success") is True and r.get("certified") is not True for r in rows)
        for arm in ("baseline", "candidate"):
            samples = [r["wall_s"] for r in rows if r["arm"] == arm and
                       r.get("success") is True and r.get("certified") is True]
            summary[arm] = {"successes": len(samples), "median_s": statistics.median(samples)
                            if len(samples) >= 5 else None, "max_s": max(samples, default=None)}
        a, b = summary["baseline"], summary["candidate"]
        status = "FAIL" if violations else "NOT_EVALUATED"
        if not violations and min(a["successes"], b["successes"]) >= 5:
            status = ("PASS" if b["successes"] >= a["successes"] and
                      b["median_s"] <= 0.8 * a["median_s"] else "FAIL")
        outcomes[case] = dict(summary, status=status, certification_violations=violations)
    statuses = {case["status"] for case in outcomes.values()}
    return {"status": "FAIL" if "FAIL" in statuses else
            "PASS" if statuses == {"PASS"} else "NOT_EVALUATED", "cases": outcomes}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["audit", "replay", "compare"])
    parser.add_argument("evidence", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--scene-tf", type=Path)
    parser.add_argument("--robot-poses", type=Path)
    parser.add_argument("--observe-pose")
    parser.add_argument("--asset", type=Path, action="append", default=[])
    args = parser.parse_args()
    if args.out.resolve().is_relative_to(args.evidence.resolve()):
        parser.error("output must be separate from original evidence")
    args.out.mkdir(parents=True, exist_ok=False)
    git = lambda *a: subprocess.check_output(["git", "-C", str(ROOT), *a])
    (args.out / "source.diff").write_bytes(git("diff", "HEAD", "--binary"))
    report = compare(args.evidence) if args.mode == "compare" else audit(args.evidence)
    report["environment"] = {"revision": git("rev-parse", "HEAD").decode().strip(),
                             "dirty": git("status", "--porcelain", "-uno").decode().splitlines(),
                             "assets_sha256": {str(p): sha(p) for p in args.asset}}
    if args.mode == "replay" and report["status"] == "READY":
        if not all((args.scene_tf, args.robot_poses, args.observe_pose)):
            report.update(status="NOT_EVALUATED", missing=["scene-tf, robot-poses, observe-pose"])
        else:
            command = ["python3", str(ROOT / "scripts/placement_prior/exact_route_replay.py"),
                       "--capture", str(args.evidence), "--scene",
                       str(args.evidence / "dumps/select_replay/planning_scene.cdr"),
                       "--scene-tf", str(args.scene_tf), "--robot-poses", str(args.robot_poses),
                       "--observe-pose", args.observe_pose]
            report["native_attempts"] = []
            for i in range(5):
                target = args.out / f"native_{i}"
                with (args.out / f"native_{i}.log").open("w") as log:
                    result = subprocess.run(command + ["--output", str(target)],
                                            stdout=log, stderr=subprocess.STDOUT)
                native = read(target / "result.json", {})
                size = sum(p.stat().st_size for p in args.out.rglob("*") if p.is_file())
                complete = (native.get("callback_shutdown_complete") is True
                            and "reason_code" in native and not native.get("error")
                            and not native.get("teardown_incomplete")
                            and (target / "response.cdr").is_file()
                            and size <= RUN_BYTES)
                report["native_attempts"].append({
                    "path": target.name, "exit": result.returncode,
                    "complete": complete, "success": native.get("success"),
                    "reason_code": native.get("reason_code"),
                    "wall_s": native.get("elapsed_s"),
                    "stage_timings": native.get("stage_timings"),
                })
                if not complete:
                    break
            report["status"] = ("REPLAYED" if len(report["native_attempts"]) == 5 and
                                complete else "NOT_EVALUATED")
            successful = [a["wall_s"] for a in report["native_attempts"]
                          if a["complete"] and a["success"] and a["wall_s"] is not None]
            report["successful_replays"] = len(successful)
            report["success_wall_s"] = (
                {"median": statistics.median(successful), "max": max(successful)}
                if successful else None)
    (args.out / "result.json").write_text(json.dumps(report, indent=2) + "\n")
    print(report["status"])
    return {"FAIL": 1, "NOT_EVALUATED": 2}.get(report["status"], 0)


if __name__ == "__main__":
    raise SystemExit(main())
