"""Evidence joins and performance admission must fail closed."""
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest

PATH = Path(__file__).resolve().parents[3] / "scripts/occ_motion_diagnostics.py"
SPEC = importlib.util.spec_from_file_location("occ_motion_diagnostics", PATH)
diagnostics = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(diagnostics)


def test_exact_select_scene_is_bound_to_request(tmp_path):
    from luggage_msgs.srv import SelectPlacePathV2
    from moveit_msgs.msg import PlanningScene
    from rclpy.serialization import deserialize_message, serialize_message
    from luggage_planning.select_replay_capture import capture_select_scene

    request = SelectPlacePathV2.Request(request_id="first")
    scene = PlanningScene(name="frozen")
    request_bytes = serialize_message(request)
    capture_select_scene(tmp_path, request.request_id, request_bytes, lambda: scene)
    captured = (tmp_path / "planning_scene.cdr").read_bytes()
    assert deserialize_message(captured, PlanningScene) == scene
    binding = json.loads((tmp_path / "identity.json").read_text())
    assert binding["select_request_sha256"] == hashlib.sha256(
        request_bytes).hexdigest()
    assert binding["scene_sha256"] == hashlib.sha256(captured).hexdigest()
    capture_select_scene(tmp_path, "second", b"other",
                         lambda: pytest.fail("capture is capped at one scene"))
    assert (tmp_path / "planning_scene.cdr").read_bytes() == captured


def test_missing_scene_is_not_a_replayable_capture(tmp_path):
    from luggage_planning.select_replay_capture import capture_select_scene

    with pytest.raises(RuntimeError, match="SELECT_REPLAY_SCENE_UNAVAILABLE"):
        capture_select_scene(tmp_path, "missing", b"request", lambda: None)
    assert not (tmp_path / "identity.json").exists()


def test_exact_wire_pair_does_not_replace_a_missing_scene(tmp_path):
    folder = tmp_path / "dumps/typed_service_boundaries"
    folder.mkdir(parents=True)
    ledger = []
    for index, side in enumerate(("request", "response")):
        name = f"{side}.cdr"
        (folder / name).write_bytes(b"test")
        ledger.append(dict(file=name, bytes=4, monotonic_ns=index,
                           request_id="select-1" if index == 0 else "",
                           boundary=diagnostics.SELECT + ":" + side))
    (folder / "boundaries.jsonl").write_text("\n".join(map(json.dumps, ledger)))
    report = diagnostics.audit(tmp_path)
    assert len(report["select_pairs"]) == 1
    assert report["status"] == "NOT_EVALUATED"
    assert any("PlanningScene" in reason for reason in report["missing"])


def test_compare_requires_certification_identity_and_minimum_samples(tmp_path):
    path = tmp_path / "samples.json"
    rows = [dict(case="portal", arm=arm, input_identity="same",
                 capture_complete=True, success=True, certified=True,
                 wall_s=10 if arm == "baseline" else 8)
            for _ in range(10) for arm in ("baseline", "candidate")]

    def status():
        path.write_text(json.dumps(rows))
        return diagnostics.compare(path)["status"]

    assert status() == "PASS"
    rows[1]["input_identity"] = "changed"
    assert status() == "NOT_EVALUATED"
    rows[1]["input_identity"] = "same"
    rows[1]["certified"] = False
    assert status() == "FAIL"
    for row in rows[1:13:2]:
        row["success"] = False
    assert status() == "NOT_EVALUATED"
