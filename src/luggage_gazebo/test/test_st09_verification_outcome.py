"""Failures cannot be hidden by still-missing ST09 coverage."""
import importlib.util
from pathlib import Path

script = Path(__file__).resolve().parents[3] / "scripts/frame_migration_st09_verify.py"
spec = importlib.util.spec_from_file_location("st09_verification", script)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def test_incomplete_does_not_mask_assertion_failure():
    assert runner.verification_outcome([dict(name="tools", exit_code=1)], ["graph"]) == ("fail", ["tools"])
    assert runner.verification_outcome([dict(name="tools", exit_code=2)], ["graph"]) == ("fail", ["tools"])
    assert runner.verification_outcome([dict(name="tools", exit_code=124)], ["graph"]) == ("fail", ["tools"])


def test_checker_incomplete_is_not_an_execution_failure():
    assert runner.verification_outcome([dict(name="checker-migration", exit_code=2)], []) == ("not_evaluated", [])
    assert runner.verification_outcome([dict(name="tools", exit_code=0)], ["graph"]) == ("not_evaluated", [])
    assert runner.verification_outcome([dict(name="tools", exit_code=0)], []) == ("pass", [])
