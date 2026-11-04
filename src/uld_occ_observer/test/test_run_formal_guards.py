#!/usr/bin/env python3
"""Offline guard fixtures for the v2 formal driver (run_formal.sh).

No ROS, no node, no real stage: every stage command is stubbed through the
driver's STAGE_CMD_<name> override and the assertions read the driver's own
ACTUAL_COMMANDS.log / DRIVER_RESULT.json. Families (complete-evidence item
11): healthy full pass; O9 failure stops everything; V failure stops P/L/R;
P/H/L failure stops R; budget exhaustion skips with a record; signal death;
stale-process startup abort; final exit nonzero on any failure/skip.

The v1 run02 driver is NOT retested here -- it is preserved immutable in the
evidence tree together with its actual log.
"""
from __future__ import annotations

import json
import os
import stat
import subprocess
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
DRIVER = HERE / "benchmark" / "run_formal.sh"


def _run_driver(tmp_path, env_extra, expect):
    root = tmp_path / "formal_root"
    env = dict(os.environ)
    env.update({
        "ELFIN_W": str(tmp_path / "w"),
        "FORMAL_ROOT": str(root),
        "TOTAL_BUDGET_S": "36000",
        # Stub every stage; the sweep stub must materialize sweep_result.json
        # with a frozen rate or the driver records not_evaluated.
        "STAGE_CMD_o9": "true",
        "STAGE_CMD_v1v3": "true",
        "STAGE_CMD_v1v3_judge": "true",
        "STAGE_CMD_phl": "true",
        "STAGE_CMD_r_sweep":
            "python3 -c \"import json,pathlib;pathlib.Path('%s/pressure').mkdir(parents=True,exist_ok=True);pathlib.Path('%s/pressure/sweep_result.json').write_text(json.dumps({'frozen_rate_hz':20.0,'rates':[],'pressure_unreached':False}))\"" % (root, root),
        "STAGE_CMD_r_score": "true",
        "STAGE_CMD_r_judge": "true",
    })
    env.update(env_extra)
    (tmp_path / "w").mkdir(exist_ok=True)
    proc = subprocess.run(
        ["bash", str(DRIVER)], env=env, capture_output=True, text=True,
        timeout=120)
    result_path = root / "DRIVER_RESULT.json"
    result = json.loads(result_path.read_text()) \
        if result_path.is_file() else {}
    log = (root / "ACTUAL_COMMANDS.log").read_text() \
        if (root / "ACTUAL_COMMANDS.log").is_file() else ""
    return proc, result, log


def test_healthy_full_pass(tmp_path):
    proc, result, log = _run_driver(tmp_path, {}, None)
    assert proc.returncode == 0, (proc.returncode, result, log)
    assert all(e.get("exit") == 0 for e in result["stages"].values())
    assert result["stopped_at"] is None


def test_o9_failure_stops_everything(tmp_path):
    proc, result, log = _run_driver(
        tmp_path, {"STAGE_CMD_o9": "false"}, None)
    assert proc.returncode != 0
    assert result["stopped_at"] == "o9"
    assert result["stages"]["v1v3"]["ran"] is False
    assert result["stages"]["phl"]["ran"] is False
    assert result["stages"]["r_score"]["ran"] is False


def test_v_failure_stops_phr(tmp_path):
    proc, result, log = _run_driver(
        tmp_path, {"STAGE_CMD_v1v3": "false"}, None)
    assert proc.returncode != 0
    assert result["stopped_at"] == "v1v3"
    assert result["stages"]["phl"]["ran"] is False
    assert result["stages"]["r_sweep"]["ran"] is False


def test_phl_failure_stops_r(tmp_path):
    proc, result, log = _run_driver(
        tmp_path, {"STAGE_CMD_phl": "false"}, None)
    assert proc.returncode != 0
    assert result["stopped_at"] == "phl"
    assert result["stages"]["r_sweep"]["ran"] is False


def test_budget_exhaustion_skips_with_record(tmp_path):
    # With a 1 s total budget the FIRST budgeted stage (o9) cannot fit its
    # allocation: the driver must skip with an explicit record, mark the
    # stop, and exit nonzero -- dependents inherit the skip.
    proc, result, log = _run_driver(
        tmp_path, {"TOTAL_BUDGET_S": "1", "V13_BUDGET_S": "3600"}, None)
    assert proc.returncode != 0
    assert result["stop_reason"] == "budget_exhausted"
    assert "budget exhausted" in result["stages"]["o9"]["skipped"]
    assert "prerequisite failed: o9" in result["stages"]["v1v3"]["skipped"]
    assert result["stages"]["phl"]["ran"] is False


def test_signal_death_recorded(tmp_path):
    proc, result, log = _run_driver(
        tmp_path, {"STAGE_CMD_phl": "kill -9 $$"}, None)
    assert proc.returncode != 0
    assert "DIED BY SIGNAL" in log
    assert result["stopped_at"] == "phl"


def test_stale_process_startup_abort(tmp_path):
    proc, result, log = _run_driver(
        tmp_path, {"STALE_CHECK_CMD": "true"}, None)
    assert proc.returncode == 9
    assert result["stopped_at"] == "startup"


def test_pressure_unreached_is_not_evaluated_not_failure(tmp_path):
    root_stub = None
    proc, result, log = _run_driver(
        tmp_path,
        {"STAGE_CMD_r_sweep":
         "python3 -c \"import json,pathlib,os;root=pathlib.Path(os.environ['FORMAL_ROOT']);(root/'pressure').mkdir(parents=True,exist_ok=True);(root/'pressure/sweep_result.json').write_text(json.dumps({'frozen_rate_hz':None,'rates':[],'pressure_unreached':True}))\""},
        None)
    # R is not_evaluated; the run itself did not fail -> driver exit 0 with
    # the skip recorded as not_evaluated.
    assert proc.returncode == 0, (proc.returncode, result)
    assert "not_evaluated" in result["stages"]["r_score"]["skipped"]


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
