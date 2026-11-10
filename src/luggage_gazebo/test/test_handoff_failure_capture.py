"""The diagnostic tail is bounded, inert by default, and never clears a box."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace

from luggage_gazebo.handoff_failure_capture import retain_failure


def test_capture_inert_without_explicit_root(monkeypatch):
    monkeypatch.delenv("ELFIN_HANDOFF_FAILURE_CAPTURE_DIR", raising=False)
    assert retain_failure(None, {}) is None


def test_sim_window_keeps_loaded_state_before_return(monkeypatch):
    monkeypatch.delenv("ELFIN_BOX_CONTACT_TOPIC", raising=False)
    clock = [10.0]
    driver = SimpleNamespace(ros_now_sec=lambda: clock[0],
                             _last_joint_state={"position": [0.12]*6},
                             _vacuum_state={"attached": True})

    def sleep(delta):
        clock[0] += delta

    with tempfile.TemporaryDirectory() as root:
        out = retain_failure(driver, {"code": "HANDOFF_SETTLE_TIMEOUT",
                                      "deadline_mono": 9.0},
                             root=root, now=lambda: clock[0], sleep=sleep)
        result = json.loads((Path(out)/"manifest.json").read_text())
        assert result["end_reason"] == "sim_window_complete"
        assert result["ended_sim"]-result["started_sim"] < 5.1
        assert result["changes_to_motion_or_verdict"] is False
        assert driver._vacuum_state["attached"] is True


def test_frozen_sim_stops_at_wall_limit(monkeypatch):
    monkeypatch.delenv("ELFIN_BOX_CONTACT_TOPIC", raising=False)
    wall = [100.0]
    driver = SimpleNamespace(ros_now_sec=lambda: 12.0)

    def sleep(delta):
        wall[0] += delta

    with tempfile.TemporaryDirectory() as root:
        out = retain_failure(driver, {"code": "HANDOFF_SETTLE_TIMEOUT",
                                      "deadline_mono": 99.0},
                             root=root, now=lambda: wall[0], sleep=sleep)
        result = json.loads((Path(out)/"manifest.json").read_text())
        assert result["end_reason"] == "wall_limit"
        assert result["ended_mono"]-result["started_mono"] < 30.1
