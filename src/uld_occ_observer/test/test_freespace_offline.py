"""Freespace offline tests: oracle units, judge negatives, and an F-matrix
subset through the real CLI (the full 360-case matrix runs from
COMMANDS.sh's rehearsal stage; here one seed per class keeps the suite fast).
"""
from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

import pytest

BENCH = Path(__file__).resolve().parent / "benchmark"
sys.path.insert(0, str(BENCH))
import freespace_cases as cases  # noqa: E402
import freespace_contract as contract  # noqa: E402
import freespace_judge as judge  # noqa: E402
import freespace_oracle as oracle  # noqa: E402

FIXTURES = Path("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/"
                "occ_o6_o7_20261029/run01/fixtures")
GEOMETRY = str(FIXTURES / "geometry_chamfer.yaml")
CONFIG = str(FIXTURES / "config_chamfer_r0p02.yaml")
CLI = Path("/tmp/observer-fsi-g1-core/observer_cli")
DESCRIPTOR = cases.load_descriptor(GEOMETRY)
CONFIG_DICT = cases.load_config(CONFIG)


def _model(rays=None):
    return oracle.model_from_manifest({
        "descriptor_json": json.dumps(DESCRIPTOR, sort_keys=True),
        "resolution": 0.02,
        "rays": rays or [],
    })


def test_chamfer_active_matches_geometry():
    model = _model()
    sx, sy, sz = model.shape
    assert (sx, sy, sz) == (75, 99, 74)
    # Below wall_z the +y side is cut by the chamfer slope; above it is not.
    assert not model.active[10, sy - 2, 0]
    assert model.active[10, 10, 0]
    assert model.active[10, sy - 2, sz - 2]


def test_oracle_support_and_precedence():
    model = _model()
    model.add_row_block(0, 5, 7, 70)          # free prefix, occupied wall
    model.add_occupied_strip(3, 5, 7, 20, 25)  # occupied beats earlier free
    model.apply_precedence()
    assert model.free[0, 5, 0]
    assert model.occ[21, 5, 3]
    assert not model.free[21, 5, 3]
    assert model.tops()[21, 5] == 3


def test_oracle_empty_enumeration():
    model = _model()
    candidates, counters = model.enumerate((0.30, 0.20, 0.20), (0,))
    assert candidates == []
    assert counters["rejected_unknown"] > 0
    total = (counters["feasible_total"] + counters["rejected_out_of_domain"]
             + counters["rejected_collision"] + counters["rejected_clearance"]
             + counters["rejected_unknown"] + counters["rejected_support"])
    assert total == counters["grid_total"]


def _healthy_response(model, box=(0.30, 0.20, 0.20), yaw=0):
    candidates, counters = model.enumerate(box, (yaw,))
    fx = math.ceil(box[0] / 0.02)
    fy = math.ceil(box[1] / 0.02)
    fz = math.ceil(box[2] / 0.02)
    response = dict(counters)
    response.update({
        "op": "freespace", "semantic_status": "OK" if candidates else "NO_KNOWN_FREE",
        "done": True, "next_cursor": counters["feasible_total"],
        "identity": {"geometry_hash": DESCRIPTOR["geometry_hash"], "epoch": 1},
        "candidates": [
            {"cell": [c[1], c[2], c[3]], "yaw_deg": c[0], "support_kind": c[4],
             "support_ratio": 1.0,
             "origin": [0.0, 0.0, 0.0],
             "verify_begin": [c[1], c[2], c[3]],
             "verify_end": [c[1] + fx, c[2] + fy, c[3] + fz]}
            for c in candidates],
    })
    return response, candidates


def _case(rays, cls="observed_empty", meta=None):
    return {"case_id": "unit", "class": cls, "seed": 0, "box": [0.30, 0.20, 0.20],
            "yaw": 0, "rays": rays, "meta": meta or {},
            "descriptor_json": json.dumps(DESCRIPTOR, sort_keys=True),
            "resolution": 0.02, "cli_returncode": 0}


def _healthy_case_responses():
    rays = [{"kind": "row_block", "z": z, "y_from": 10, "y_to": 42, "x_end": 74}
            for z in range(0, 8)]
    case = _case(rays)
    responses = [{"op": "acquire", "status": "OK"}]
    response, _ = _healthy_response(_model(rays))
    responses.append(response)
    return case, responses, response


def test_judge_accepts_healthy_artifact():
    case, responses, _ = _healthy_case_responses()
    verdict = judge.judge_f_case(case, responses, DESCRIPTOR, 0.02)
    assert verdict["reasons"] == []


def _mutate(fn):
    case, responses, response = _healthy_case_responses()
    fn(response)
    verdict = judge.judge_f_case(case, responses, DESCRIPTOR, 0.02)
    assert verdict["reasons"], "falsified artifact accepted"
    return verdict["reasons"]


def test_judge_rejects_fake_free_candidate():
    def falsify(response):
        response["candidates"].append(
            {"cell": [1, 1, 1], "yaw_deg": 0, "support_kind": 1,
             "support_ratio": 1.0, "origin": [0.0, 0.0, 0.0],
             "verify_begin": [1, 1, 1], "verify_end": [16, 11, 11]})
    reasons = _mutate(falsify)
    assert any("candidate_set_mismatch" in r for r in reasons)


def test_judge_rejects_counter_lie():
    def falsify(response):
        response["feasible_total"] = response["feasible_total"] + 1
    reasons = _mutate(falsify)
    assert any("counter_identity" in r or "candidate_set_mismatch" in r
               or "next_cursor_mismatch" in r for r in reasons)


def test_judge_rejects_nan_and_negative_counters():
    def falsify(response):
        response["rejected_unknown"] = float("nan")
    assert any("rejected_unknown" in r for r in _mutate(falsify))

    def negative(response):
        response["rejected_support"] = -1
    assert any("rejected_support" in r for r in _mutate(negative))


def test_judge_rejects_status_lie():
    def falsify(response):
        response["semantic_status"] = "OK"
    assert any("semantic_status" in r for r in _mutate(falsify))


def test_judge_rejects_epoch_echo_drift():
    def falsify(response):
        response["identity"] = {"geometry_hash": DESCRIPTOR["geometry_hash"],
                                "epoch": 7}
    assert any("echo_epoch" in r for r in _mutate(falsify))


def test_judge_rejects_geometry_hash_drift():
    def falsify(response):
        response["identity"] = {"geometry_hash": "forged", "epoch": 1}
    assert any("geometry_hash" in r for r in _mutate(falsify))


def test_judge_rejects_missing_and_duplicate_pages():
    case, responses, _ = _healthy_case_responses()
    responses.append(json.loads(json.dumps(responses[1])))  # duplicate page
    verdict = judge.judge_f_case(case, responses, DESCRIPTOR, 0.02)
    # A repeated page is judged as a paging probe with a wrong slice.
    assert verdict["reasons"]


def test_judge_rejects_missing_artifact_and_bad_cli():
    case = _case([], cls="observed_empty")
    verdict = judge.judge_f_case(case, None, DESCRIPTOR, 0.02)
    assert verdict["reasons"] == ["responses_missing"]
    rays = [{"kind": "row_block", "z": 0, "y_from": 10, "y_to": 40, "x_end": 74}]
    case = _case(rays)
    case["cli_returncode"] = 3
    verdict = judge.judge_f_case(case, [{"op": "acquire", "status": "OK"}],
                                 DESCRIPTOR, 0.02)
    assert any("cli_returncode" in r for r in verdict["reasons"])


def test_empty_tree_never_runs_freespace():
    case = _case([], cls="empty_tree")
    bad = judge.judge_f_case(case, [{"op": "acquire", "status": "OK"},
                                    {"op": "freespace", "semantic_status": "OK"}],
                             DESCRIPTOR, 0.02)
    assert bad["reasons"]
    good = judge.judge_f_case(case, [{"op": "acquire", "status": "NO_DATA"}],
                              DESCRIPTOR, 0.02)
    assert good["reasons"] == []


def test_two_version_requires_two_views():
    rays = [{"kind": "row_block", "z": 0, "y_from": 10, "y_to": 40, "x_end": 74}]
    case = _case(rays, cls="two_version", meta={"old_ray_count": 1})
    verdict = judge.judge_f_case(case, [{"op": "acquire", "status": "OK"}],
                                 DESCRIPTOR, 0.02)
    assert any("two_views" in r or "no_freespace" in r for r in verdict["reasons"])


@pytest.mark.skipif(not CLI.exists(), reason="observer_cli not built")
@pytest.mark.parametrize("cls", contract.F_CLASSES)
def test_f_matrix_one_seed_per_class(tmp_path, cls):
    out = tmp_path / cls
    completed = subprocess.run(
        [sys.executable, str(BENCH / "freespace_cases.py"),
         "--geometry", GEOMETRY, "--config", CONFIG, "--cli", str(CLI),
         "--out", str(out), "--classes", cls, "--seeds", "1"],
        text=True, capture_output=True)
    assert completed.returncode == 0, completed.stdout[-2000:] + completed.stderr[-2000:]
    summary = json.loads((out / "summary.json").read_text())
    assert summary["passed"] == summary["total"] == 4, summary["failures"]


def test_f_summary_matrix_size_gate():
    assert judge.judge_f_summary({"total": 360, "passed": 360}) == []
    assert judge.judge_f_summary({"total": 359, "passed": 359})
    assert judge.judge_f_summary({"total": 360, "passed": 358})
    assert judge.judge_f_summary({})

