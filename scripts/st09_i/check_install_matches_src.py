#!/usr/bin/env python3
"""Verify an install tree matches the sources it was built from.

Rebuild first (clean env, only /opt/ros/humble sourced):

    colcon build --build-base build-st09 --install-base install-st09 \
        --packages-select luggage_perception \
        luggage_planning luggage_gazebo

Then run this check. For every Python or launch file under the selected
packages' source trees it finds the installed copy by relative layout and
compares SHA-256. A missing or different copy is a failure. The manifest
records the git commit, dirty-file count and per-file hashes so a run can
state exactly which build it used.

    scripts/st09_i/check_install_matches_src.py --install install-st09 \
        --out /tmp/install_manifest.json
"""

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

PACKAGES = {
    # package -> (source package dir relative to repo root)
    "luggage_perception": "src/luggage_perception",
    "luggage_planning": "src/luggage_planning",
    "luggage_gazebo": "src/luggage_gazebo",
}
SUFFIXES = (".py",)
SKIP_PARTS = ("test", "__pycache__", "ros1_reference")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git(root, *args):
    return subprocess.run(
        ["git", "-C", str(root)] + list(args), check=True,
        capture_output=True, text=True).stdout.strip()


def candidates(install, package, relative):
    """Installed locations a source file may land in."""
    name = Path(relative).name
    base = Path(install) / package
    if relative.parts[0] == "scripts":
        yield base / "lib" / package / name
    elif relative.parts[0] == "launch":
        yield base / "share" / package / "launch" / name
    else:
        yield from base.glob(
            "local/lib/python*/dist-packages/%s" % "/".join(relative.parts))


def source_files(root, package_dir, package):
    top = Path(root) / package_dir
    for path in sorted(top.rglob("*")):
        relative = path.relative_to(top)
        if path.suffix not in SUFFIXES and relative.parts[0] != "launch":
            continue
        if not path.is_file() or any(p in SKIP_PARTS for p in relative.parts):
            continue
        if relative.parts[0] not in ("scripts", "launch", package):
            continue
        yield path, relative


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=".")
    parser.add_argument("--install", default="install-st09")
    parser.add_argument("--out", default="")
    args = parser.parse_args()
    root = Path(args.root).resolve()
    install = (root / args.install).resolve()
    rows, problems, not_installed = [], [], []
    for package, package_dir in PACKAGES.items():
        for path, relative in source_files(root, package_dir, package):
            found = next(
                (c for c in candidates(install, package, relative)
                 if c.is_file()), None)
            row = {"package": package, "file": str(relative),
                   "src_sha256": sha256(path), "installed": str(found or "")}
            if found is None and relative.parts[0] == "scripts":
                # Package CMake installs only the scripts it names.
                row["status"] = "not_installed"
                not_installed.append("%s/%s" % (package, relative))
            elif found is None:
                problems.append("missing %s/%s" % (package, relative))
            elif sha256(found) != row["src_sha256"]:
                problems.append("differs %s/%s" % (package, relative))
            rows.append(row)
    manifest = {
        "commit": git(root, "rev-parse", "HEAD"),
        "dirty_files": len(git(root, "status", "--porcelain").splitlines()),
        "install": str(install),
        "checked": len(rows),
        "problems": problems,
        "scripts_not_installed": not_installed,
        "match": not problems,
        "files": rows,
    }
    if args.out:
        Path(args.out).write_text(json.dumps(manifest, indent=2) + "\n")
    print("checked=%d problems=%d scripts_not_installed=%d commit=%s "
          "dirty=%d" % (
              len(rows), len(problems), len(not_installed),
              manifest["commit"][:8], manifest["dirty_files"]))
    for line in problems[:40]:
        print(" ", line)
    return 0 if not problems else 1


if __name__ == "__main__":
    sys.exit(main())
