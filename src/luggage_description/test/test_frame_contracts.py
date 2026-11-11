"""Behavioral oracle, malformed-input and evidence/readiness checks; no ROS."""

from contextlib import redirect_stderr, redirect_stdout
from copy import deepcopy
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest

from luggage_description.frame_contracts import ContractError, validate_pose


ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("frame_checker", ROOT / "scripts/check_frame_contracts.py")
checker = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(checker)


class FrameContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.suite = checker.load_json(checker.DEFAULT_SUITE)
        cls.inventory = checker.load_json(checker.DEFAULT_INVENTORY)
        evidence = ROOT / "docs/status/evidence/frame_contracts"
        evidence.mkdir(parents=True, exist_ok=True)
        # Retained even on test failure: assertions must not erase replay inputs.
        cls.artifacts = Path(tempfile.mkdtemp(prefix="selftests-", dir=evidence))

    def write(self, name, value):
        path = self.artifacts / name
        path.write_text(json.dumps(value), encoding="utf-8")
        return path

    def run_cli(self, *args):
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            return checker.main(list(args))

    def test_independent_golden_cases(self):
        results = checker.run_cases(self.suite)
        self.write("golden-results.json", results)
        self.assertGreaterEqual(len(results), 20)
        for result in results:
            with self.subTest(case=result["id"]):
                self.assertEqual(result["status"], "pass", result)

    def test_each_interface_has_sources_and_known_cases(self):
        report = checker.inventory_report(
            self.inventory, {case["id"] for case in self.suite["cases"]})
        self.assertGreaterEqual(report["interface_count"], 26)
        # Labels cannot predict proof validity in a changing workspace. Every
        # explicitly pending boundary must remain pending; stale verified
        # proofs are tested below and may also add pending boundaries.
        pending = {
            row["id"] for row in self.inventory["interfaces"]
            if row.get("migration_state") == "pending"}
        self.assertTrue(pending <= {row["id"] for row in report["pending"]}, report["pending"])

    def test_verified_labels_without_runtime_proofs_leave_every_boundary_pending(self):
        inventory = deepcopy(self.inventory)
        for row in inventory["interfaces"]:
            row["migration_state"] = "verified"
            row.pop("runtime_evidence", None)
        report = checker.inventory_report(inventory, {c["id"] for c in self.suite["cases"]})
        self.assertEqual({r["id"] for r in report["pending"]},
                         {r["id"] for r in inventory["interfaces"]})

    def test_nonfinite_boolean_and_wrong_length_coordinates_rejected(self):
        for bad in (float("nan"), float("inf"), -float("inf"), True):
            with self.subTest(value=str(bad)):
                record = deepcopy(self.suite["records"]["slot"])
                record["position"][0] = bad
                with self.assertRaises(ContractError) as error:
                    validate_pose(record, "slot", self.suite["scene_revision"])
                self.assertEqual(error.exception.code, "INVALID_NUMERIC_PAYLOAD")
        record["position"] = [0, 1]
        with self.assertRaises(ContractError):
            validate_pose(record, "slot", self.suite["scene_revision"])

    def test_empty_or_duplicate_cases_never_pass(self):
        for cases in ([], [self.suite["cases"][0]] * 2):
            invalid = dict(self.suite, cases=cases)
            with self.assertRaises(ValueError):
                checker.run_cases(invalid)

    def test_wrong_numeric_oracle_is_a_failure_not_roundtrip_success(self):
        suite = deepcopy(self.suite)
        case = next(c for c in suite["cases"] if c["id"] == "translate_rotate90")
        case["expect"]["position"][2] -= 0.53
        results = checker.run_cases(suite)
        bad = [r for r in results if r["status"] == "fail"]
        self.assertEqual([r["id"] for r in bad], ["translate_rotate90"])

    def test_bad_case_exception_cannot_match_an_error_oracle(self):
        suite = deepcopy(self.suite)
        suite["cases"] = [{"id": "bad-operation", "op": "typo", "expect": {"exception": "ValueError"}}]
        result = checker.run_cases(suite)[0]
        self.assertEqual(result["status"], "fail")

    def test_inventory_typo_or_missing_pointer_fails(self):
        for key, value in (("checks", ["nonexistent-case"]),
                           ("sources", ["does/not/exist.py"]),
                           ("migration_state", "done")):
            inventory = deepcopy(self.inventory)
            inventory["interfaces"][0][key] = value
            with self.subTest(field=key), self.assertRaises(ValueError):
                checker.inventory_report(inventory, {c["id"] for c in self.suite["cases"]})

    def test_marking_verified_without_proof_does_not_clear_gate(self):
        inventory = deepcopy(self.inventory)
        inventory["interfaces"][0]["migration_state"] = "verified"
        report = checker.inventory_report(inventory, {c["id"] for c in self.suite["cases"]})
        self.assertIn("F01", {p["id"] for p in report["pending"]})

    def test_deleting_a_pending_boundary_cannot_make_inventory_pass(self):
        for boundary in ("F24", "F25", "F26"):
            inventory = deepcopy(self.inventory)
            inventory["interfaces"] = [r for r in inventory["interfaces"] if r["id"] != boundary]
            with self.subTest(boundary=boundary), self.assertRaisesRegex(ValueError, "missing required boundaries"):
                checker.inventory_report(inventory, {c["id"] for c in self.suite["cases"]})

    def test_offline_or_stale_proof_cannot_clear_runtime_gate(self):
        inventory = deepcopy(self.inventory)
        row = inventory["interfaces"][0]
        row["migration_state"] = "verified"
        proof = {"interface_id": row["id"], "kind": "offline_baseline", "status": "pass",
                 "contract_version": 2, "interface_contract_sha256": checker.interface_digest(row),
                 "command": "synthetic", "observations": ["synthetic"], "sample_count": 1,
                 "source_sha256": {p: checker.sha256(ROOT / p) for p in [*row["sources"], *checker.CONTRACT_SOURCES]}}
        path = self.write("invalid-runtime-proof.json", proof)
        row["runtime_evidence"] = str(path.relative_to(ROOT))
        report = checker.inventory_report(inventory, {c["id"] for c in self.suite["cases"]})
        self.assertIn("F01", {p["id"] for p in report["pending"]})
        proof["kind"] = "runtime_boundary"
        proof["source_sha256"][row["sources"][0]] = "stale"
        path.write_text(json.dumps(proof), encoding="utf-8")
        report = checker.inventory_report(inventory, {c["id"] for c in self.suite["cases"]})
        self.assertIn("F01", {p["id"] for p in report["pending"]})

    def test_proof_must_match_current_contract_as_well_as_sources(self):
        inventory = deepcopy(self.inventory)
        row = inventory["interfaces"][0]
        row["migration_state"] = "verified"
        proof = {"interface_id": row["id"], "kind": "runtime_boundary", "status": "pass",
                 "contract_version": 2, "interface_contract_sha256": checker.interface_digest(row),
                 "command": "synthetic gate unit fixture, not runtime evidence",
                 "observations": ["synthetic"], "sample_count": 1,
                 "source_sha256": {p: checker.sha256(ROOT / p) for p in [*row["sources"], *checker.CONTRACT_SOURCES]}}
        path = self.write("synthetic-valid-proof.json", proof)
        row["runtime_evidence"] = str(path.relative_to(ROOT))
        ids = {c["id"] for c in self.suite["cases"]}
        report = checker.inventory_report(inventory, ids)
        self.assertNotIn("F01", {p["id"] for p in report["pending"]})
        row["target"] = "different contract"
        report = checker.inventory_report(inventory, ids)
        self.assertIn("F01", {p["id"] for p in report["pending"]})

    def test_duplicate_keys_and_nonfinite_json_refused(self):
        path = self.artifacts / "invalid-json.json"
        for text in ('{"x": 1, "x": 2}', '{"x": NaN}', '{"x": Infinity}'):
            path.write_text(text, encoding="utf-8")
            with self.assertRaises(ValueError):
                checker.load_json(path)

    def test_cli_baseline_and_migration_have_different_verdicts(self):
        for mode, expected_code in (("baseline", 0), ("migration", 2)):
            output = self.artifacts / mode
            code = self.run_cli("--mode", mode, "--output", str(output))
            self.assertEqual(code, expected_code)
            summary = checker.load_json(output / "summary.json")
            manifest = checker.load_json(output / "manifest.json")
            self.assertTrue(summary["baseline_pass"])
            self.assertFalse(summary["migration_ready"])
            self.assertEqual(summary["runtime_execution"], "not_evaluated")
            self.assertTrue(manifest["capture_complete"])
            self.assertTrue(manifest["source_sha256"])

    def test_unexpected_failure_is_durable_and_replayable(self):
        suite = deepcopy(self.suite)
        suite["cases"][0]["expect"] = {"code": "SHOULD_NOT_PASS"}
        input_path = self.write("failing-suite.json", suite)
        output = self.artifacts / "failed-run"
        self.assertEqual(self.run_cli("--suite", str(input_path), "--output", str(output)), 1)
        result = checker.load_json(output / "cases.json")[0]
        self.assertEqual(result["status"], "fail")
        self.assertEqual(result["actual"], {"accepted": True})
        self.assertEqual(result["input"]["record"]["frame_id"], "container_link")
        manifest = checker.load_json(output / "manifest.json")
        self.assertTrue(manifest["capture_complete"])
        self.assertTrue(manifest["replay_possible"])
        self.assertEqual(manifest["first_failed_case"], "slot_valid")
        before = (output / "cases.json").read_bytes()
        self.assertEqual(self.run_cli("--output", str(output)), 1)
        self.assertEqual((output / "cases.json").read_bytes(), before)

    def test_malformed_suite_reports_incomplete_not_success(self):
        path = self.write("empty-suite.json", {"suite_version": 1, "cases": []})
        output = self.artifacts / "invalid-suite"
        self.assertEqual(self.run_cli("--suite", str(path), "--output", str(output)), 1)
        self.assertFalse(checker.load_json(output / "summary.json")["baseline_pass"])
        self.assertFalse(checker.load_json(output / "manifest.json")["capture_complete"])


if __name__ == "__main__":
    unittest.main()
