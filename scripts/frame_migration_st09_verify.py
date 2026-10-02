#!/usr/bin/env python3
"""ST09 evidence runner. Unimplemented live obligations explicitly block release."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import time


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
        manifest["artifacts"] = [{"path":str(path.relative_to(out)), "bytes":path.stat().st_size,
                                  "sha256":hashlib.sha256(path.read_bytes()).hexdigest()}
                                 for path in sorted(out.rglob("*")) if path.is_file() and path.name != "manifest.json"]
        save()
    print(json.dumps(dict(status=manifest["status"], missing=manifest["missing"], output=str(out))))
    return {"pass": 0, "fail": 1, "not_evaluated": 2}[manifest["status"]]


if __name__ == "__main__":
    raise SystemExit(main())
