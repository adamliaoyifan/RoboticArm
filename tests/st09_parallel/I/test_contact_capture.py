"""Contact-capture parsers, the pre-motion gate, and the absence rule."""
import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts/st09_i"))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src/luggage_planning"))
import contact_capture as cap  # noqa: E402


CONTACT = """header {
  stamp { sec: 12 nsec: 500000000 }
}
contact {
  collision1 { id: 1 name: "pickup_box_0001::suitcase_link::body_collision" }
  collision2 { id: 2 name: "S20::suction_panel::p01_collision" }
  depth: 0.004
  wrench { body_1_wrench { force { x: 0 y: 3 z: 4 } torque { x: 0 y: 0 z: 0 } } }
}
contact {
  collision1 { id: 1 name: "pickup_box_0001::suitcase_link::body_collision" }
  collision2 { id: 3 name: "S20::elfin_link2::collision" }
  depth: 0.001
}
"""

POSE = """header { stamp { sec: 3 nsec: 0 } }
pose {
  name: "pickup_box_0001_carryon"
  position { x: 1 y: 2 z: 3 }
  orientation { x: 0 y: 0 z: 0 w: 1 }
}
pose {
  name: "ground_plane"
  position { x: 0 y: 0 z: 0 }
  orientation { x: 0 y: 0 z: 0 w: 1 }
}
"""


def test_stamp_does_not_read_sec_out_of_nsec():
    text = "header {\n  stamp {\n    nsec: 990000000\n  }\n}\n"
    assert cap._header_stamp(text) == 0.99


def test_contact_parser_keeps_pairs_depth_and_force():
    rows = cap.parse_contacts(CONTACT)
    assert len(rows) == 2
    assert rows[0]["stamp"] == 12.5
    assert rows[0]["b"].endswith("suction_panel::p01_collision")
    assert abs(rows[0]["depth_m"] - 0.004) < 1e-9
    assert abs(rows[0]["force_n"] - 5.0) < 1e-9
    assert cap.classify_pair(rows[0]["a"], rows[0]["b"]) == "suction_interface"
    assert cap.classify_pair(rows[1]["a"], rows[1]["b"]) == "robot_body"
    assert cap.classify_pair("ground", "pedestal") == "not_box"


def test_pose_parser_keeps_the_stamp_and_every_name():
    stamp, rows = cap.parse_pose_v(POSE)
    assert stamp == 3.0
    assert rows[0][0] == "pickup_box_0001_carryon"
    assert rows[0][1] == (1.0, 2.0, 3.0)
    assert len(rows) == 2


def test_gate_passes_only_when_every_stream_is_live(tmp_path):
    trace = tmp_path / "follow.jsonl"
    trace.write_text(json.dumps({
        "ok": True, "box_xyz": [0, 0, 0], "panel_xyz": [0, 0, 1]}) + "\n")
    sdf = tmp_path / "box.sdf"
    sdf.write_text(
        '<model><static>true</static>'
        '<collision name="body_collision"></collision>'
        '<sensor type="contact"><contact>'
        '<collision>body_collision</collision></contact></sensor></model>')
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        '<robot><link name="suction_panel"><collision/></link>'
        '<link name="elfin_link2"><collision/></link></robot>')
    status = tmp_path / "status.json"
    status.write_text(json.dumps({
        "alive": True, "pose_messages": 4, "contact_messages": 0}))
    now = time.time()
    ok, detail = cap.capture_ready(
        str(trace), "Publishers:\n  tcp://1", str(sdf), str(urdf),
        str(status), now=now)
    assert ok, detail
    none, _ = cap.capture_ready(
        str(trace), "No publishers", str(sdf), str(urdf), str(status), now=now)
    assert not none
    os.utime(trace, (now - 10, now - 10))
    stale, _ = cap.capture_ready(
        str(trace), "Publishers:\n  tcp://1", str(sdf), str(urdf),
        str(status), now=now)
    assert not stale


def test_no_contact_messages_are_not_evidence_when_the_cup_is_inside_the_box():
    assert cap.absence_is_evidence(
        saw_contact=True, publisher_ok=True, recorder_covered=True,
        panel_inside_box=True)
    assert not cap.absence_is_evidence(
        saw_contact=False, publisher_ok=True, recorder_covered=True,
        panel_inside_box=True)
    assert cap.absence_is_evidence(
        saw_contact=False, publisher_ok=True, recorder_covered=True,
        panel_inside_box=False)


def test_inside_box_and_coincidence_use_the_stated_thresholds():
    import importlib.util
    path = Path(__file__).resolve().parents[3] / "scripts/st09_i/contact_tracking_analysis.py"
    spec = importlib.util.spec_from_file_location("contact_tracking_analysis", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert mod.inside_box((0, 0, 0), (0, 0, 0), (0, 0, 0, 1), (0.2, 0.2, 0.2))
    assert not mod.inside_box((1, 0, 0), (0, 0, 0), (0, 0, 0, 1), (0.2, 0.2, 0.2))
    # 180 deg about z: inverse sends +x to -x, still inside a wide box.
    assert mod.inside_box((0.1, 0, 0), (0, 0, 0), (0, 0, 1, 0), (0.2, 0.2, 0.2))
    import numpy as np
    t = np.array([0.0, 0.1, 0.2, 0.3])
    err = np.array([0.01, 0.05, 0.08, 0.01])
    contacts = [
        {"stamp": 0.1, "klass": "robot_body", "depth_m": 0.002, "force_n": 4.0},
        {"stamp": 0.2, "klass": "robot_body", "depth_m": 0.003, "force_n": 5.0},
    ]
    rows = mod.summarize_windows(
        [("transit", 0.0, 0.3)], t, err, contacts, [])
    frac = rows[0]["contacts"]["robot_body"]["fraction_of_undertrack_samples"]
    assert frac == 1.0
    assert rows[0]["contacts"]["suction_interface"]["fraction_of_undertrack_samples"] == 0.0
    assert rows[0]["joint2_undertrack_samples"] == 2


def test_follow_trace_is_off_unless_the_env_path_is_set(tmp_path, monkeypatch):
    from luggage_planning.vacuum_backend import SimVacuumBackend
    backend = SimVacuumBackend(object(), object())
    backend._model_name = "pickup_box"
    monkeypatch.delenv("ELFIN_FOLLOW_TRACE", raising=False)
    backend._write_follow_trace((0, 0, 0), (0, 0, 0, 1), (1, 0, 0),
                                (0, 0, 0, 1), True, "")
    path = tmp_path / "trace.jsonl"
    assert not path.exists()
    monkeypatch.setenv("ELFIN_FOLLOW_TRACE", str(path))
    backend._write_follow_trace((0, 0, 1), (0, 0, 0, 1), (1, 0, 0),
                                (0, 0, 0, 1), True, "ok")
    row = json.loads(path.read_text().strip())
    assert row["ok"] is True and row["box_xyz"] == [1.0, 0.0, 0.0]
