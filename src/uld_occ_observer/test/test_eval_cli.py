#!/usr/bin/env python3
"""Behavioral tests for occ_observer_eval.py exit codes and arguments."""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
EVAL = ROOT / "scripts" / "g3" / "occ_observer_eval.py"


def _run(args):
    return subprocess.run(
        [sys.executable, str(EVAL), *args],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        check=False,
    )


def test_help_exits_zero():
    result = _run(["--help"])
    assert result.returncode == 0, result.stderr
    assert "fixtures" in result.stdout
    assert "correctness" in result.stdout
    assert "simulate" in result.stdout
    assert "replay" in result.stdout


def test_subcommand_help_exits_zero():
    for cmd in (
        "fixtures",
        "correctness",
        "benchmark",
        "concurrency",
        "stability",
        "capture-overhead",
        "simulate",
        "replay",
        "unsupported",
    ):
        result = _run([cmd, "--help"])
        assert result.returncode == 0, (cmd, result.stderr)


def test_invalid_command_exits_three():
    result = _run(["not-a-gate"])
    assert result.returncode == 3


def test_missing_required_exits_three():
    result = _run(["correctness"])
    assert result.returncode == 3
