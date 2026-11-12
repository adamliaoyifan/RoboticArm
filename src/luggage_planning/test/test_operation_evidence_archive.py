"""Replay-critical CDR remains exact within a stored-byte evidence budget."""
import gzip
import hashlib
import json
from types import SimpleNamespace

import pytest
from luggage_planning.joint_operation.prepare_node import JointOperationPrepareNode


def test_repeated_private_scene_capture_is_lossless_and_accounts_stored_bytes(tmp_path):
    owner = SimpleNamespace(_evidence=str(tmp_path), _evidence_seq=0, _evidence_bytes=0)
    raw = b"repeated-scene-mesh" * 50000
    JointOperationPrepareNode._record_owned(owner, "aperture_ik_request", raw)
    row = json.loads((tmp_path / "boundaries.jsonl").read_text())
    stored = (tmp_path / row["file"]).read_bytes()
    assert row["encoding"] == "gzip"
    assert gzip.decompress(stored) == raw
    assert row["sha256"] == hashlib.sha256(raw).hexdigest()
    assert row["stored_sha256"] == hashlib.sha256(stored).hexdigest()
    assert row["bytes"] == len(raw)
    assert owner._evidence_bytes == row["stored_bytes"] == len(stored) < len(raw) / 10


def test_full_archive_refuses_without_partial_boundary(tmp_path):
    owner = SimpleNamespace(_evidence=str(tmp_path), _evidence_seq=0,
                            _evidence_bytes=256*1024*1024)
    with pytest.raises(RuntimeError, match="OPERATION_EVIDENCE_BUDGET_EXHAUSTED"):
        JointOperationPrepareNode._record_owned(owner, "request", b"scene")
    assert not list(tmp_path.iterdir())
    assert owner._evidence_seq == 0
