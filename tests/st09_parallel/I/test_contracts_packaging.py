"""ST09-I S1: `st09_contracts` is a colcon-installable package.

The frozen C0 layout keeps the python package at the repository root for
the parallel test trees; ST09-I additionally packages it so production
nodes resolve it from a plain install overlay. Two red lines: the
overlay-only import must work without the repository root anywhere on
sys.path, and the installed copy must produce byte-identical contract
digests to the repository-root copy (content identity, no drift).
"""

from __future__ import division

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
INSTALL_ROOT = REPO_ROOT / "install-st09"

_OVERLAY_IMPORT = (
    "import sys, os\n"
    "import st09_contracts\n"
    "loc = st09_contracts.__file__\n"
    "assert 'install-st09' in loc, loc\n"
    "assert not any(os.path.realpath(p) == %r for p in sys.path)\n"
    "from st09_contracts.fixtures import make_attachment, make_snapshot\n"
    "from st09_contracts.graph import PlannedRoute\n"
    "from st09_contracts.certification import CertificationResult\n"
    "from st09_contracts.protocols import Certifier\n"
    "attachment = make_attachment('centered', symmetry='none')\n"
    "snapshot = make_snapshot(1)\n"
    "print(attachment.digest())\n"
) % str(REPO_ROOT)


def _overlay_env():
    env = dict(os.environ)
    env.pop("PYTHONPATH", None)
    bootstrap = (
        "source /opt/ros/humble/setup.bash && "
        "source %s/setup.bash" % INSTALL_ROOT
    )
    return env, bootstrap


def _run_probe(bootstrap):
    """Run the overlay probe as a script file (bash quoting keeps \\n)."""
    import tempfile
    env, _ = _overlay_env()
    with tempfile.NamedTemporaryFile("w", suffix=".py",
                                     delete=False) as handle:
        handle.write(_OVERLAY_IMPORT)
        path = handle.name
    try:
        return subprocess.run(
            ["bash", "-c", bootstrap + " && cd /tmp && python3 %s" % path],
            env=env, capture_output=True, text=True, timeout=120)
    finally:
        os.unlink(path)


@pytest.mark.skipif(not (INSTALL_ROOT / "setup.bash").exists(),
                    reason="install-st09 overlay not built")
class TestContractsPackaging:

    def test_overlay_only_import_without_repo_root(self):
        _env, bootstrap = _overlay_env()
        result = _run_probe(bootstrap)
        assert result.returncode == 0, result.stderr
        assert len(result.stdout.strip().splitlines()[-1]) == 64

    def test_installed_digest_matches_repository_copy(self):
        """Same fixture -> same digest under both import roots."""
        _env, bootstrap = _overlay_env()
        overlay = _run_probe(bootstrap)
        assert overlay.returncode == 0, overlay.stderr
        overlay_digest = overlay.stdout.strip().splitlines()[-1]
        # In-process: the parallel conftest put the repo root first.
        from st09_contracts.fixtures import make_attachment
        import st09_contracts
        assert "install-st09" not in st09_contracts.__file__
        assert make_attachment("centered", symmetry="none").digest() \
            == overlay_digest

    def test_frozen_manifests_installed_to_share(self):
        share = INSTALL_ROOT / "st09_contracts" / "share" / "st09_contracts"
        for name in ("resolved_planning_contract.json",
                     "st09_acceptance_manifest.json"):
            installed = share / name
            assert installed.exists(), name
            assert json.loads(installed.read_text()) == \
                json.loads((REPO_ROOT / "st09_contracts" / name).read_text())

    def test_repo_root_import_still_wins_for_parallel_tests(self):
        """The C0 conftest contract: repo root on sys.path -> root copy."""
        sys.path.insert(0, str(REPO_ROOT))
        try:
            for cached in [m for m in list(sys.modules)
                           if m == "st09_contracts"
                           or m.startswith("st09_contracts.")]:
                del sys.modules[cached]
            import st09_contracts
            assert "install-st09" not in st09_contracts.__file__
        finally:
            sys.path.remove(str(REPO_ROOT))
