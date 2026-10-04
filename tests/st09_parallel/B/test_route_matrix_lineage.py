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


def test_retired_held_out_ids_stay_on_the_dev_split():
    module = _eval_module()
    assert module.assign_split("old-held-out", {"old-held-out"}) == "dev"
    again = module.assign_split("carryon|nominal|pick_tooldown|occ0|h0|door", set())
    assert again == module.assign_split(
        "carryon|nominal|pick_tooldown|occ0|h0|door", set())
    assert again in ("dev", "held_out")


def test_extra_slots_keep_the_original_four_names():
    module = _eval_module()
    slots = [(x * 0.15, y * 0.15, 0.4 + 0.05 * x)
             for x in range(6) for y in range(4)]
    picked = module._pick_slots(slots, limit=6)
    assert [name for name, _centre in picked[:4]] == [
        "door", "deep", "wall_y", "high"]
    assert len(picked) == 6
    assert picked[4][0].startswith("s")


def test_reference_classes_label_generated_cases_only():
    module = _eval_module()
    cases = [
        {"id": "a", "kind": "generated"},
        {"id": "b", "kind": "generated"},
        {"id": "bad", "kind": "invalid"},
        {"id": "slot", "kind": "regression"},
    ]
    module.apply_reference_classes(cases, {"a": "feasible", "b": "undetermined"})
    assert cases[0]["class"] == "feasible"
    assert cases[1]["class"] == "undetermined"
    assert cases[2]["class"] == "invalid"
    assert cases[3]["class"] == "regression"


def test_historical_matrix_manifest_keeps_its_original_revision():
    manifest = json.loads(open(_OLD_MANIFEST, encoding="utf-8").read())
    assert manifest["commit"].startswith("e5c9a87")
    assert "lineage" not in manifest
