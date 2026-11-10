"""Backend registry and four-interface gates (offline, no ROS import)."""
import json

import pytest

from joint_command_capture.backends import (
    allocate_event_id,
    available_backends,
    get_backend,
    write_trigger,
)


def test_unimplemented_backend_is_explicitly_refused():
    with pytest.raises(NotImplementedError) as exc:
        get_backend("elfin_vendor")
    assert "elfin_vendor" in str(exc.value)
    assert "sim_gz" in str(exc.value)          # lists what exists
    assert "no fallback" in str(exc.value)      # refusal is explicit


def test_registry_has_only_sim_gz():
    assert available_backends() == ("sim_gz",)


def test_sim_gz_describe_backend_declares_semantics(tmp_path):
    be = get_backend("sim_gz", str(tmp_path))
    desc = be.describe_backend()
    # "command received" must never be declared as "motor executed"
    assert desc.command_means_motor_executed is False
    assert desc.execution_path == "sim_gz"
    assert "positions" in desc.fields_command


def test_four_interfaces_write_jsonl(tmp_path):
    be = get_backend("sim_gz", str(tmp_path))
    be.record_command({"cycle": 1, "positions": [0.0] * 6})
    be.record_feedback({"cycle": 1, "positions": [0.0] * 6})
    be.record_event({"kind": "test"})
    for name in ("context_commands.jsonl", "context_feedback.jsonl",
                 "context_events.jsonl"):
        rows = [json.loads(x) for x in
                (tmp_path / name).read_text().splitlines()]
        assert rows and "wall" in rows[0] and "mono" in rows[0]


def test_trigger_id_allocation_monotonic_and_idempotent(tmp_path):
    run = tmp_path / "run"
    assert allocate_event_id(run) == 1
    write_trigger(run, 1, "manual")
    assert allocate_event_id(run) == 2
    write_trigger(run, 2, "settle_fail")
    # Same id twice: no new file, idempotent delivery.
    p = write_trigger(run, 2, "settle_fail again")
    ctl = run / "ctl"
    triggers = sorted(x.name for x in ctl.iterdir() if x.name.startswith("trigger-"))
    assert triggers == ["trigger-1", "trigger-2"]
    assert p.exists()
