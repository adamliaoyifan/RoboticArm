"""Replay the frozen structure sequences. Evidence must already exist."""

import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = ROOT / "docs/status/evidence/occ_structure_optimize_20261102/t_freeze"
YAW = Path(os.environ.get(
    "OCC_YAW_EVIDENCE",
    "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/"
    "occ_structure_review_20261102/yaw_scalar_vectorized.json"))


def test_replay_matches_frozen_baseline():
    assert EVIDENCE.joinpath("MANIFEST.json").is_file(), EVIDENCE
    assert YAW.is_file(), YAW
    env = os.environ.copy()
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    env["PYTHONPATH"] = ".:src/luggage_perception/test:src/luggage_perception:src/luggage_description"
    completed = subprocess.run(
        [sys.executable, "scripts/occ_structure/compare_candidate.py",
         "--expected", str(EVIDENCE), "--yaw", str(YAW)],
        cwd=str(ROOT), env=env, text=True, capture_output=True)
    assert completed.returncode == 0, completed.stdout + completed.stderr
