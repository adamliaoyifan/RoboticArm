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
            manifest["missing"].append("production consumer identity enforcement and sim/deploy graph harness: three healthy startups and all fault cases")
        if args.mode in ("rehearsal", "all"):
            manifest["missing"].append("immutable wired candidate and executable frozen capture configuration required before Gazebo rehearsal")
        failed = [entry["name"] for entry in manifest["commands"] if entry.get("exit_code") != 0]
        manifest["failed_commands"] = failed
        manifest["status"] = "not_evaluated" if manifest["missing"] else "fail" if failed else "pass"
    finally:
        manifest["elapsed_seconds"] = time.monotonic() - started
        manifest["completed_at"] = datetime.now(timezone.utc).isoformat()
        manifest["artifacts"] = [{"path":str(path.relative_to(out)), "bytes":path.stat().st_size,
                                  "sha256":hashlib.sha256(path.read_bytes()).hexdigest()}
                                 for path in sorted(out.rglob("*")) if path.is_file() and path.name != "manifest.json"]
        save()
    print(json.dumps(dict(status=manifest["status"], missing=manifest["missing"], output=str(out))))
    return 0 if manifest["status"] == "pass" else 2


if __name__ == "__main__":
    raise SystemExit(main())
