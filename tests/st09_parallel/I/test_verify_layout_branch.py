"""verify_layout_branch tells the deployment run from the simulation-layout run.

Fixtures are two recorded runs: ``g3_twobox_rehearsal_20261007b`` (stack and
driver on scene_tf_deploy.yaml; box 0 won at slot 656 through the lattice
planner) and ``two_box_confirm/2026-10-07_rehearsal_a`` (driver on the
simulation scene file: container at (1.5, 0, 0), atlas identity of another
scene).
"""
import importlib.util
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
EVIDENCE = REPO / "docs" / "status" / "evidence"
DEPLOY_RUN = (EVIDENCE / "frame_migration" / "ST09-parallel-I-s5" / "m3"
              / "g3_twobox_rehearsal_20261007b")
SIM_LAYOUT_RUN = EVIDENCE / "two_box_confirm" / "2026-10-07_rehearsal_a"
SCENE = REPO / "src" / "luggage_description" / "config" / "scene_tf_deploy.yaml"


def _tool():
    spec = importlib.util.spec_from_file_location(
        "verify_layout_branch",
        REPO / "scripts" / "st09_i" / "verify_layout_branch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _report(tool, run, capsys):
    code = tool.main(["--run", str(run), "--scene", str(SCENE)])
    import json
    return code, json.loads(capsys.readouterr().out)


@pytest.mark.skipif(not DEPLOY_RUN.is_dir(), reason="evidence not present")
def test_deployment_run_passes_layout_atlas_and_lattice(capsys):
    code, report = _report(_tool(), DEPLOY_RUN, capsys)
    assert code == 0 and report["ok"]
    first = report["boxes"][0]
    assert first["layout"]["ok"] and first["atlas"]["ok"]
    assert first["lattice"]["winner_slot"] == 656
    assert first["lattice"]["winner_is_lattice"]


@pytest.mark.skipif(not SIM_LAYOUT_RUN.is_dir(), reason="evidence not present")
def test_simulation_layout_run_is_rejected(capsys):
    code, report = _report(_tool(), SIM_LAYOUT_RUN, capsys)
    assert code == 1 and not report["ok"]
    first = report["boxes"][0]
    assert not first["layout"]["ok"]
    assert first["layout"]["translation"][0] == pytest.approx(1.5)
    assert not first["atlas"]["ok"]
