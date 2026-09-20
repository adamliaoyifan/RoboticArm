"""Eval-run provenance and install-staleness guard. Not for online nodes.

An eval report must be attributable to a code revision and an effective
config (AGENTS.md: claim only the coverage actually demonstrated). The
installed ``luggage_perception`` library under ``install/*/dist-packages``
is a real copy, not a symlink — library edits are invisible until
``colcon build --packages-select luggage_perception`` reruns, so a replay
can silently measure the stale library instead of the edit under test.

Both helpers degrade to ``"unknown"`` values instead of raising:
provenance must never kill an eval run, and a missing git tree or an
unlocatable source checkout only loses attribution, not the replay.
"""
from __future__ import division

import hashlib
import json
import os
import subprocess
import sys

# shutil.copy2 (colcon's installer) preserves mtimes, so a rebuilt file has
# the source mtime; the slack only absorbs filesystem timestamp jitter.
_MTIME_SLACK_SEC = 1.0

_GIT_TIMEOUT_SEC = 10


def _module_dir():
    import luggage_perception
    return os.path.dirname(os.path.abspath(luggage_perception.__file__))


def _is_install_path(path):
    return ("%sinstall%s" % (os.sep, os.sep)) in (path + os.sep)


def source_package_dir():
    """Directory of the source ``luggage_perception`` package, or None.

    When running from the source tree (PYTHONPATH=src) this is the module
    directory itself; when running from an install copy, the workspace
    root is recovered by walking out of the install prefix and checking
    for ``<ws>/src/luggage_perception/luggage_perception``.
    """
    module_dir = _module_dir()
    if not _is_install_path(module_dir):
        return module_dir
    probe = module_dir
    while _is_install_path(probe) and probe != os.path.dirname(probe):
        probe = os.path.dirname(probe)
    # probe is now the first ancestor outside the install prefix — the
    # workspace root (…/<ws>/install itself still matches, so the walk
    # ends at <ws>).
    candidate = os.path.join(
        probe, "src", "luggage_perception", "luggage_perception")
    return candidate if os.path.isdir(candidate) else None


def _git_state(work_dir):
    """(revision, dirty, dirty_files) via git, or ("unknown", None, None)."""
    def _run(args):
        try:
            proc = subprocess.run(
                ["git"] + args, cwd=work_dir, timeout=_GIT_TIMEOUT_SEC,
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                check=False)
        except (OSError, ValueError, subprocess.SubprocessError):
            return ""
        return proc.stdout.decode("utf-8", "replace").strip()

    revision = _run(["rev-parse", "HEAD"])
    if not revision or " " in revision:
        return "unknown", None, None
    status = _run(["status", "--porcelain", "--", "."])
    lines = [line for line in status.splitlines() if line.strip()]
    return revision, bool(lines), len(lines)


def collect_provenance(effective_config=None):
    """Attribution dict for a summary.json: code revision, dirty state,
    sha256 of the effective config, interpreter version."""
    git_dir = source_package_dir() or _module_dir()
    revision, dirty, dirty_files = _git_state(git_dir)
    config = dict(effective_config or {})
    config_hash = hashlib.sha256(
        json.dumps(config, sort_keys=True, default=str)
        .encode("utf-8")).hexdigest()
    return {
        "code_revision": revision,
        "code_revision_dirty": dirty,
        "dirty_files": dirty_files,
        "module_path": _module_dir(),
        "config_hash": config_hash,
        "python": sys.version.split()[0],
    }


def check_install_staleness():
    """Compare the running library copy against the source tree.

    Returns ``{"mode": "source"|"install"|"install-no-source",
    "stale": bool, "newer_files": [...], "checked": int,
    "module_path": str}``. A source-tree run is never stale; an install
    copy is stale when any source .py is newer than its installed copy
    (or missing from the install entirely — the running code cannot see
    that file at all).
    """
    module_dir = _module_dir()
    result = {
        "mode": "source",
        "module_path": module_dir,
        "stale": False,
        "newer_files": [],
        "checked": 0,
    }
    if not _is_install_path(module_dir):
        return result
    src_dir = source_package_dir()
    if src_dir is None:
        result["mode"] = "install-no-source"
        return result
    result["mode"] = "install"
    newer = []
    checked = 0
    for base, _dirs, files in os.walk(src_dir):
        if "__pycache__" in base:
            continue
        for name in sorted(files):
            if not name.endswith(".py"):
                continue
            src_file = os.path.join(base, name)
            rel = os.path.relpath(src_file, src_dir)
            inst_file = os.path.join(module_dir, rel)
            if not os.path.isfile(inst_file):
                newer.append("missing:%s" % rel)
                continue
            checked += 1
            if (os.path.getmtime(src_file)
                    > os.path.getmtime(inst_file) + _MTIME_SLACK_SEC):
                newer.append(rel)
    result["checked"] = checked
    result["newer_files"] = newer
    result["stale"] = bool(newer)
    return result


def staleness_warning(report=None):
    """One-line human warning for a stale install, or an empty string."""
    report = report if report is not None else check_install_staleness()
    if not report.get("stale"):
        return ""
    files = report.get("newer_files") or []
    sample = ", ".join(files[:3]) + (" ..." if len(files) > 3 else "")
    return ("WARNING: installed luggage_perception library is stale "
            "(%d file(s) newer or missing in the install copy: %s); "
            "rebuild with: colcon build --packages-select "
            "luggage_perception" % (len(files), sample))
