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
    assert module.case_is_scored(cases[0])
    assert not module.case_is_scored(cases[1])
    assert module.case_is_scored(cases[2])
    assert module.case_is_scored(cases[3])


def test_tool_down_joint1_branch_is_one_pi_shift():
    import math
    from luggage_planning.route_producer import (
        TOOL_DOWN_SEED_Q,
        tool_down_branch_seeds,
    )
    labels = [label for label, _q in tool_down_branch_seeds()]
    assert labels == ["tool_down", "tool_down_j1_minus_pi"]
    shifted = tool_down_branch_seeds()[1][1]
    assert abs((TOOL_DOWN_SEED_Q[0] - math.pi) - shifted[0]) < 1e-12
    assert shifted[1:] == tuple(TOOL_DOWN_SEED_Q[1:])


def test_dev_zero_goal_case_needs_the_joint1_branch():
    """Nominal tool-down misses this dev goal; the -pi joint-1 prior hits it."""
    module = _eval_module()
    from luggage_planning.route_producer import tool_down_branch_seeds
    scene = module.Scene(module.SCENE)
    urdf = module.load_urdf(scene_tf_config=module.SCENE)
    tcp = urdf.path_to("suction_contact_frame")
    starts = module._starts(urdf, tcp)
    retired = module.load_retired_held_out(module.os.path.join(
        module.ROOT, "docs", "status", "evidence", "frame_migration",
        "ST09-parallel-I-s5", "route_matrix_20261004", "split.json"))
    cases, _occ = module.build_cases(scene, starts, retired)
    case = next(c for c in cases
                if c["id"] == "carryon|nominal|pick_tooldown|occ0|h0|wall_y")
    assert case["split"] == "dev"
    size = module.PAYLOADS[case["payload"]]
    payload = module.Payload(size, module.OFFSETS[case["offset"]])
    pos, quat = payload.tcp_for_box(
        scene.to_world(case["centre_container"]),
        scene.yaw + module.HEADINGS[case["heading"]])
    nominal = tool_down_branch_seeds()[0][1]
    branch = tool_down_branch_seeds()[1][1]
    miss, _info = module.refine_to_pose(
        urdf, tcp, pos, quat, list(nominal), pos_tol=2e-3, rot_tol=2e-2)
    hit, _info = module.refine_to_pose(
        urdf, tcp, pos, quat, list(branch), pos_tol=2e-3, rot_tol=2e-2)
    assert miss is None
    assert hit is not None


def test_historical_matrix_manifest_keeps_its_original_revision():
    manifest = json.loads(open(_OLD_MANIFEST, encoding="utf-8").read())
    assert manifest["commit"].startswith("e5c9a87")
    assert "lineage" not in manifest
