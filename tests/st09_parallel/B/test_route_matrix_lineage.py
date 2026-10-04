"""New route-matrix manifests name planner lineage; the old run does not."""
import importlib.util
import json
import os


_ROOT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
_SCRIPT = os.path.join(_ROOT, "scripts", "st09_i", "route_matrix_eval.py")
_OLD_MANIFEST = os.path.join(
    _ROOT, "docs", "status", "evidence", "frame_migration",
    "ST09-parallel-I-s5", "route_matrix_20261004", "manifest.json")


def _eval_module():
    spec = importlib.util.spec_from_file_location(
        "route_matrix_eval_under_test", _SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_lineage_names_the_three_planner_commits():
    lineage = _eval_module().MATRIX_LINEAGE
    assert set(lineage) == {
        "descend_pair_baseline",
        "container_drawing_rule",
        "candidate_scope",
    }
    assert lineage["descend_pair_baseline"].startswith("f9ba0e8")
    assert lineage["container_drawing_rule"].startswith("bdb9394")
    assert lineage["candidate_scope"].startswith("dd6d48e")


def test_historical_matrix_manifest_keeps_its_original_revision():
    manifest = json.loads(open(_OLD_MANIFEST, encoding="utf-8").read())
    assert manifest["commit"].startswith("e5c9a87")
    assert "lineage" not in manifest
