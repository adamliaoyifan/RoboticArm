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


#: Install-tree closure per package: python modules, message/config
#: share payloads, launch scripts, top-level native libraries AND the
#: lib/<pkg>/ executables the single-level glob used to miss (G2 §3.1:
#: G23/C1 29/O8 node entry points were never hashed in freeze-v1).
INSTALL_PATTERNS = (
    ("lib/python3.10/dist-packages/**/*.py", "python-module"),
    ("local/lib/python3.10/dist-packages/**/*.py", "python-module"),
    ("lib/*/python3.10/dist-packages/**/*.py", "python-module"),
    ("share/**/*.yaml", "config"),
    ("share/**/*.srv", "message"),
    ("share/**/*.msg", "message"),
    ("share/**/*.xml", "manifest"),
    ("share/**/*.py", "launch-script"),
    ("lib/*", "native-lib"),
    ("lib/*/*", "executable"),
)


def _install_files(prefix, packages, problems, side="", store=None):
    """Hash the installed closure of the named packages.

    Keys are ``package/relative_path`` so two packages can never silently
    overwrite each other's entry (freeze-v1 keyed bare relative paths —
    structurally unsafe, though G1's package set never actually
    collided). A duplicate logical path is recorded as a problem instead
    of being dropped. With ``store`` set, every hashed byte is also
    copied into the content-addressed store for independent
    verification."""
    out = {}
    for package in packages:
        base = Path(prefix) / package
        if not base.is_dir():
            problems.append("%s install prefix missing: %s" % (side, base))
            continue
        for pattern, role in INSTALL_PATTERNS:
            for path in sorted(base.glob(pattern)):
                if not path.is_file():
                    continue
                key = "%s/%s" % (package, path.relative_to(base))
                if key in out:
                    problems.append(
                        "%s duplicate install path: %s" % (side, key))
                    continue
                digest = sha256_file(path)
                if store is not None:
                    _store_copy(store, path, problems, digest)
                out[key] = {
                    "sha256": digest,
                    "size": path.stat().st_size,
                    "role": role,
                }
    return out


def _source_files(root, packages, store=None, problems=None):
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
                digest = sha256_file(path)
                if store is not None and problems is not None:
                    _store_copy(store, path, problems, digest)
                out[str(path.relative_to(base))] = digest
    return out


def _store_copy(out, source_path, problems, digest=None):
    """Copy one payload into the freeze's content-addressed store.

    The store holds real bytes so an independent verifier can recompute
    hashes without reading the (volatile) worktrees — G2 §3.1: an
    archive must contain the bytes or reference a hash-verified
    read-only store, never a bare /tmp path."""
    digest = digest or sha256_file(source_path)
    shard = out / "store" / digest[:2] / digest
    if shard.exists():
        if sha256_file(shard) != digest:  # corrupted pre-existing shard
            problems.append("store shard corrupt: %s" % digest)
        return digest
    shard.parent.mkdir(parents=True, exist_ok=True)
    shard.write_bytes(source_path.read_bytes())
    return digest


def _t1_files(c1_root, out, problems):
    """Hash (and store) every T1 script the campaign CLI executes."""
    scripts = {}
    entry = c1_root / "scripts/occ_consumer_compat.py"
    _store_copy(out, entry, problems)
    scripts["occ_consumer_compat.py"] = sha256_file(entry)
    for path in sorted((c1_root / "scripts/occ_compat").glob("*.py")):
        _store_copy(out, path, problems)
        scripts["occ_compat/" + path.name] = sha256_file(path)
    return scripts


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
                    g_root, ("luggage_perception",), out, problems),
                "luggage_msgs": _source_files(
                    g_root, ("luggage_msgs",), out, problems),
                "luggage_description": _source_files(
                    g_root, ("luggage_description",), out, problems),
            },
        },
        "C1": {
            "commit": c1_sha,
            "branch": _git(c1_root, "rev-parse", "--abbrev-ref", "HEAD"),
            "packages": {
                "luggage_packing": _source_files(
                    c1_root, ("luggage_packing",), out, problems),
                "luggage_planning": _source_files(
                    c1_root, ("luggage_planning",), out, problems),
                "luggage_description": _source_files(
                    c1_root, ("luggage_description",), out, problems),
                "luggage_msgs": _source_files(
                    c1_root, ("luggage_msgs",), out, problems),
                "uld_occ_observer_scripts": _source_files(
                    c1_root, ("uld_occ_observer",), out, problems),
            },
        },
        "T1": {
            "commit": c1_sha,
            "scripts": _t1_files(c1_root, out, problems),
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
             "luggage_description", "st09_contracts"),
            problems, "G", out),
        "C1": _install_files(
            manifest["c1_install"],
            ("luggage_packing", "luggage_planning",
             "luggage_description", "luggage_msgs",
             "uld_occ_observer"),
            problems, "C1", out),
        "O_CONTROL": _install_files(
            manifest["o_install"], ("uld_occ_observer",),
            problems, "O_CONTROL", out),
    }

    store_shards = sorted(out.glob("store/*/*"))
    # Four runtime input classes (reviews 1210 #1): bytes into the
    # content-addressed store + declared digests, so A0 can close the
    # input identity of runs collected under this freeze.
    input_files = {}
    for role, key in (("scene_tf", "scene_tf"),
                      ("o_config", "o_config"),
                      ("o_geometry", "o_geometry"),
                      ("prep_inputs", "prep_inputs_json")):
        target = manifest.get(key)
        if not target:
            continue
        path = Path(target)
        candidates = [path] if path.is_file() else sorted(
            path.rglob("*")) if path.is_dir() else []
        for item in candidates:
            if item.is_file():
                digest = sha256_file(item)
                _store_copy(out, item, problems, digest)
                rel = (item.relative_to(path).as_posix()
                       if path.is_dir() else item.name)
                input_files["%s/%s" % (role, rel)] = {
                    "path": str(item), "sha256": digest}

    freeze_doc = {
        "schema": "freeze-v3",
        "input_files": input_files,
        "source_map": source_map,
        "install_manifest": install_manifest,
        "store": {
            "kind": "content-addressed-sha256",
            "layout": "store/<sha256[:2]>/<sha256>",
            "shards": len(store_shards),
            "bytes": sum(s.stat().st_size for s in store_shards),
            "note": "real payload bytes for independent a0 verification; "
                    "the freeze generator's declarations are not trusted "
                    "by the judge",
        },
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
        "schema": freeze_doc["schema"],
        "store_shards": len(store_shards),
        "problems": problems,
        "g_dirty_count": len(g_dirty), "c1_dirty_count": len(c1_dirty),
    }
    (out / "FREEZE_RESULT.json").write_text(json.dumps(summary, indent=1))
    return summary
