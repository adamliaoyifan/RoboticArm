"""Freeze-tool contracts: salt, retired held-out, stable ids, reference-only."""
import hashlib
import importlib.util
import json
import os
import subprocess
import sys
from types import SimpleNamespace


_ROOT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
_SCRIPT = os.path.join(_ROOT, "scripts", "st09_i", "route_matrix_eval.py")
_OLD_SPLIT = os.path.join(
    _ROOT, "docs", "status", "evidence", "frame_migration",
    "ST09-parallel-I-s5", "route_matrix_20261004", "split.json")
_FROZEN_SPLIT = os.path.join(
    _ROOT, "docs", "status", "evidence", "frame_migration",
    "ST09-parallel-I-s5", "route_matrix_g2_20261005", "split.json")


def _module():
    name = "route_matrix_eval_freeze_under_test"
    spec = importlib.util.spec_from_file_location(name, _SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _unsalted(case_id, fraction=0.3):
    digest = int(hashlib.sha1(case_id.encode()).hexdigest()[:8], 16)
    if (digest % 1000) < fraction * 1000:
        return "held_out"
    return "dev"


def test_split_salt_is_stable_and_not_the_unsalted_bit():
    module = _module()
    assert module.SPLIT_SALT
    differed = False
    for index in range(40):
        case_id = "salt-probe-%d" % index
        first = module.assign_split(case_id, set())
        assert first == module.assign_split(case_id, set())
        assert first in ("dev", "held_out")
        if first != _unsalted(case_id):
            differed = True
    assert differed


def test_retired_held_out_ids_from_the_old_matrix_stay_dev():
    module = _module()
    retired = module.load_retired_held_out(_OLD_SPLIT)
    assert retired
    for case_id in list(retired)[:20]:
        assert module.assign_split(case_id, retired) == "dev"


def test_slot_names_and_order_repeat():
    module = _module()
    slots = [(x * 0.15, y * 0.15, 0.4 + 0.05 * x)
             for x in range(6) for y in range(4)]
    first = module._pick_slots(slots, limit=8)
    second = module._pick_slots(list(reversed(slots)), limit=8)
    assert first == second
    assert [name for name, _centre in first[:4]] == [
        "door", "deep", "wall_y", "high"]
    assert first[4][0] == module._grid_slot_name(first[4][1])


def test_case_ids_repeat_and_match_the_frozen_split():
    module = _module()
    scene = module.Scene(module.SCENE)
    urdf = module.load_urdf(scene_tf_config=module.SCENE)
    tcp = urdf.path_to("suction_contact_frame")
    starts = module._starts(urdf, tcp)
    retired = module.load_retired_held_out(_OLD_SPLIT)
    first, _occ = module.build_cases(scene, starts, retired)
    second, _occ2 = module.build_cases(scene, starts, retired)
    ids = [case["id"] for case in first]
    assert ids == [case["id"] for case in second]
    assert [case["split"] for case in first] == [
        case["split"] for case in second]
    frozen = json.loads(open(_FROZEN_SPLIT, encoding="utf-8").read())
    assert list(frozen) == ids
    assert [frozen[case_id] for case_id in ids] == [
        case["split"] for case in first]


def test_reference_freeze_writes_manifest_on_a_clean_contract(tmp_path):
    module = _module()
    cases = [
        {"id": "held-a", "kind": "generated", "split": "held_out",
         "class": "feasible"},
        {"id": "dev-b", "kind": "generated", "split": "dev",
         "class": "undetermined"},
    ]
    args = SimpleNamespace(
        out=str(tmp_path),
        retired_held_out="",
        reference_budget_s=60.0,
        reference_extra_seeds=8)
    freeze = module.write_reference_freeze(args, cases, started=0.0)
    assert freeze["ready_for_scored_run"] is False
    assert freeze["held_out_feasible"] == 1
    assert freeze["split_salt"] == module.SPLIT_SALT
    assert freeze["reference"] == {"budget_s": 60.0, "extra_seeds": 8}
    assert freeze["dirty_count"] == 0 or isinstance(freeze["dirty_count"], int)
    manifest = json.loads((tmp_path / "manifest.json").read_text())
    assert manifest["case_ids"] == ["held-a", "dev-b"]
    assert manifest["planner_revision"].startswith("dd6d48e")
    assert manifest["reference"]["extra_seeds"] == 8


def test_reference_only_does_not_score_baseline(tmp_path):
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([
        os.path.join(_ROOT, "src", "luggage_planning"),
        os.path.join(_ROOT, "src", "luggage_description"),
        _ROOT,
        env.get("PYTHONPATH", ""),
    ])
    proc = subprocess.run(
        [sys.executable, _SCRIPT, "--out", str(tmp_path),
         "--reference-only", "--limit", "1", "--reps", "1"],
        cwd=_ROOT, env=env, capture_output=True, text=True, timeout=180)
    assert proc.returncode in (0, 2), proc.stderr[-500:]
    rows = [
        json.loads(line) for line in (tmp_path / "trace.jsonl").read_text().splitlines()
        if line.strip()]
    assert rows and all(row.get("reference") for row in rows)
    assert all(row.get("mode") != "baseline" for row in rows)
    freeze = json.loads((tmp_path / "freeze.json").read_text())
    assert freeze["reference"]["budget_s"] == 60.0
    assert "manifest.json" == os.path.basename(
        str(tmp_path / "manifest.json")) and (tmp_path / "manifest.json").is_file()
