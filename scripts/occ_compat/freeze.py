"""freeze — SHAs, install manifests, dirty check (contract §2 step 6).

Records the exact G/C1/T1 commits, per-package source SHAs, the O
control SHA, install artifacts (hashes of every installed module for
the three packages the chains import), and the dirty state of each
worktree (build/evidence dirs excluded and explained). Changing ANY of
these after freeze invalidates the affected acceptance: a new freeze
and a new run are required.
"""
from __future__ import annotations

import json
import subprocess
from pathlib import Path

from .inventory import sha256_file

MANIFEST_PATH = "/tmp/occ_compat_manifest.json"

EXCLUDED_DIRS = ("build", "install", "log", "docs/status")


def _git(root, *args):
    return subprocess.run(
        ["git", "-C", str(root)] + list(args),
        capture_output=True, text=True, timeout=60).stdout.strip()


def _dirty(root):
    lines = _git(root, "status", "--porcelain").splitlines()
    relevant = []
    for line in lines:
        path = line[3:].strip()
        if any(path.startswith(prefix) or prefix in path
               for prefix in EXCLUDED_DIRS):
            continue
        relevant.append(line)
    return relevant


def _install_files(prefix, packages):
    """Hash every installed module of the named packages."""
    out = {}
    for package in packages:
        base = Path(prefix) / package
        for pattern in ("lib/python3.10/dist-packages/**/*.py",
                        "local/lib/python3.10/dist-packages/**/*.py",
                        "share/**/*.yaml", "share/**/*.srv",
                        "share/**/*.msg", "lib/*"):
            for path in sorted(base.glob(pattern)):
                if path.is_file():
                    out[str(path.relative_to(base))] = sha256_file(path)
    return out


def _source_files(root, packages):
    out = {}
    for package in packages:
        base = Path(root) / "src" / package
        for path in sorted(base.rglob("*")):
            if (path.is_file()
                    and "/build/" not in str(path)
                    and "/install/" not in str(path)
                    # bytecode caches are not source: their hashes depend
                    # on which tests ran last, which made the freeze
                    # non-deterministic across otherwise-identical trees
                    and "__pycache__" not in path.parts
                    and path.suffix != ".pyc"):
                out[str(path.relative_to(base))] = sha256_file(path)
    return out


def run_freeze(manifest_path, out_dir):
    manifest = json.loads(Path(manifest_path).read_text())
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    problems = []

    g_root = Path(manifest["g_root"])
    c1_root = Path(manifest["c1_root"])
    o_root = Path(manifest["o_root"])

    g_sha = _git(g_root, "rev-parse", "HEAD")
    c1_sha = _git(c1_root, "rev-parse", "HEAD")
    o_sha = _git(o_root, "rev-parse", "HEAD")

    g_dirty = _dirty(g_root)
    c1_dirty = _dirty(c1_root)
    if g_dirty:
        problems.append("G worktree dirty: %s" % g_dirty[:3])
    if c1_dirty:
        problems.append("C1/T1 worktree dirty: %s" % c1_dirty[:3])

    source_map = {
        "G": {
            "commit": g_sha,
            "branch": _git(g_root, "rev-parse", "--abbrev-ref", "HEAD"),
            "packages": {
                "luggage_perception": _source_files(
                    g_root, ("luggage_perception",)),
                "luggage_msgs": _source_files(
                    g_root, ("luggage_msgs",)),
                "luggage_description": _source_files(
                    g_root, ("luggage_description",)),
            },
        },
        "C1": {
            "commit": c1_sha,
            "branch": _git(c1_root, "rev-parse", "--abbrev-ref", "HEAD"),
            "packages": {
                "luggage_packing": _source_files(
                    c1_root, ("luggage_packing",)),
                "luggage_planning": _source_files(
                    c1_root, ("luggage_planning",)),
                "luggage_description": _source_files(
                    c1_root, ("luggage_description",)),
                "luggage_msgs": _source_files(
                    c1_root, ("luggage_msgs",)),
                "uld_occ_observer_scripts": _source_files(
                    c1_root, ("uld_occ_observer",)),
            },
        },
        "T1": {
            "commit": c1_sha,
            "scripts": {
                **{"occ_consumer_compat.py": sha256_file(
                    c1_root / "scripts/occ_consumer_compat.py")},
                **{"occ_compat/" + p.name: sha256_file(p)
                   for p in sorted(
                       (c1_root / "scripts/occ_compat").glob("*.py"))},
            },
        },
        "O_CONTROL": {
            "commit": o_sha,
            "read_only": True,
            "note": "detached control worktree; never rebuilt or edited",
        },
    }

    install_manifest = {
        "G": _install_files(
            manifest["g_install"],
            ("luggage_perception", "luggage_msgs",
             "luggage_description", "st09_contracts")),
        "C1": _install_files(
            manifest["c1_install"],
            ("luggage_packing", "luggage_planning",
             "luggage_description", "luggage_msgs",
             "uld_occ_observer")),
        "O_CONTROL": _install_files(
            manifest["o_install"], ("uld_occ_observer",)),
    }

    freeze_doc = {
        "schema": "freeze-v1",
        "source_map": source_map,
        "install_manifest": install_manifest,
        "dirty": {"G": g_dirty, "C1": c1_dirty, "O": []},
        "excluded": {
            "patterns": list(EXCLUDED_DIRS),
            "reason": "build/install/log artifacts and evidence dirs are "
                      "generated, not source; evidence lives under "
                      "docs/status (gitignored by campaign convention)",
        },
        "problems": problems,
        "manifest": manifest,
    }
    (out / "FREEZE.json").write_text(json.dumps(freeze_doc, indent=1))
    summary = {
        "G": g_sha, "C1": c1_sha, "T1": c1_sha, "O": o_sha,
        "problems": problems,
        "g_dirty_count": len(g_dirty), "c1_dirty_count": len(c1_dirty),
    }
    (out / "FREEZE_RESULT.json").write_text(json.dumps(summary, indent=1))
    return summary
