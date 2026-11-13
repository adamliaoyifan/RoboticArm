"""C0 acceptance-manifest and production-baseline tests.

Covers: manifest structure (requirement->task->test->artifact, file
owners, workload, commands), production QP non-activatability, and the
restore disposition completeness.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
MANIFEST = json.loads(
    (ROOT / "src" / "luggage_planning" / "config" / "planning_contract" / "st09_acceptance_manifest.json").read_text())
AUDIT = json.loads(
    (ROOT / "src" / "luggage_planning" / "config" / "planning_contract" / "v2_field_audit.json").read_text())
DISPOSITION = json.loads(
    (ROOT / "src" / "luggage_planning" / "config" / "planning_contract" / "restore_disposition.json").read_text())


def test_no_tbd_in_commands_or_config():
    text = json.dumps(MANIFEST)
    for banned in ("TBD", "tbd", "TODO", "FIXME"):
        assert banned not in text


def test_requirement_task_test_artifact_complete():
    rows = MANIFEST["requirement_task_test_artifact"]
    assert rows, "mapping must not be empty"
    tasks = {row["task"] for row in rows}
    for task in ("ST09-C0", "ST09-A", "ST09-B", "ST09-C", "ST09-D",
                 "ST09-E", "ST09-Q", "ST09-I", "ST09-R"):
        assert task in tasks
    for row in rows:
        assert row["requirement"] and row["test"] and row["artifact"]


def test_file_owners_exclusive():
    owners = MANIFEST["file_owners"]
    seen = {}
    for owner, paths in owners.items():
        for path in paths:
            if path.endswith("/**"):
                continue
            if path in seen:
                pytest.fail(
                    "%s claimed by both %s and %s" % (path, seen[path], owner))
            seen[path] = owner
    # shared entries stay with I; QP code exists only on the experiment branch
    assert "ST09-I (exclusive shared entries)" in owners
    assert "ST09-Q (experiment branch experiment/st09-qp-preserved only)" in owners


def test_m3_workload_frozen():
    assert MANIFEST["m3_workload"]["sequence_ids"] == [
        "carryon", "standard", "carryon"]


def test_unified_test_entry_and_c0_directory_exists():
    entry = MANIFEST["unified_test_entry"]["command"]
    assert entry == "python3 -m pytest tests/st09_parallel/<TASK> -q"
    assert (ROOT / "tests" / "st09_parallel" / "C0").is_dir()
    assert (ROOT / "tests" / "st09_parallel" / "conftest.py").is_file()


def test_case_inventory_names_plan_gates():
    inv = MANIFEST["required_case_inventory"]
    for key in ("geometry_frame_64", "deep_place_16", "yaw_ik_8_classes",
                "target_matrix_8", "attachment_negatives_9", "map_cases_6_x2",
                "startup_3_x2", "mutations", "performance", "capture_replay"):
        assert key in inv


class TestProductionQPAbsence:
    """C0 item 2: the production baseline cannot activate QP."""

    PRODUCTION_PATHS = [
        ROOT / "src",
        ROOT / "scripts",
    ]
    FORBIDDEN_PATTERNS = [
        "qp_carry",
        "carry_planner",
        "carry-planner",
        "osqp",
        "qp_carry_config_hash",
    ]
    # The timed-trajectory start gate keeps its historical wire name; the
    # generic mechanism is production behaviour, not QP activation.
    ALLOWED_LINES = ("QP_START_STATE_MISMATCH", "qp_start_state_mismatch")

    def test_no_qp_activation_surface_in_production(self):
        offenders = []
        for base in self.PRODUCTION_PATHS:
            for path in base.rglob("*.py"):
                text = path.read_text(errors="ignore")
                for lineno, line in enumerate(text.splitlines(), 1):
                    if any(p in line for p in self.ALLOWED_LINES):
                        continue
                    for pattern in self.FORBIDDEN_PATTERNS:
                        if pattern in line:
                            offenders.append("%s:%d: %s" % (path, lineno, line.strip()))
        assert offenders == [], "QP activation surface leaked into production:\n" + \
            "\n".join(offenders)

    def test_qp_files_absent_from_production_tree(self):
        assert not (ROOT / "src/luggage_planning/luggage_planning/qp_carry").exists()
        assert not (ROOT / "src/luggage_planning/config/qp_carry.yaml").exists()
        assert not (ROOT / "scripts/replay_qp_carry.py").exists()
        qp_tests = list((ROOT / "src/luggage_planning/test").glob("test_qp_*.py"))
        assert qp_tests == []

    def test_package_xml_has_no_osqp(self):
        text = (ROOT / "src/luggage_planning/package.xml").read_text()
        assert "osqp" not in text.lower()


class TestRestoreDisposition:
    """S0 item 2: per-file hash/source/disposition for the checkpoint."""

    def test_all_checkpoint_files_dispositioned(self):
        files = DISPOSITION["files"]
        assert len(files) == DISPOSITION["counts"]["checkpoint_files_hashed"]
        assert DISPOSITION["counts"]["checkpoint_files_hashed"] == 237
        allowed = {"restored_production", "qp_experiment_branch", "not_restored",
                   "restored_reference_only"}
        for name, entry in files.items():
            assert entry["disposition"] in allowed, name
            assert entry.get("reason"), name

    def test_qp_files_went_to_experiment_branch(self):
        qp = [n for n, e in DISPOSITION["files"].items()
              if e["disposition"] == "qp_experiment_branch"]
        assert len(qp) >= 16
        assert any("qp_carry/" in n for n in qp)

    def test_vendor_and_sdk_binaries_not_restored(self):
        not_restored = [n for n, e in DISPOSITION["files"].items()
                        if e["disposition"] == "not_restored"]
        assert any("vendor" in n for n in not_restored)
        assert any("CppSdkSample" in n for n in not_restored)

    def test_known_baseline_failures_recorded(self):
        known = DISPOSITION["known_baseline_failures"]
        assert any("test_j6_carry_unwind_is_not_refused" in f["node"]
                   for f in known)
