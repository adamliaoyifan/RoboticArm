"""Evidence must survive beyond the ring and refuse corruption or dropped joins."""

import json

import pytest

from occ_query_wire_journal import WireJournal, WireJournalError, read_journal, read_payload


def test_complete_window_and_pending_are_lossless(tmp_path):
    journal = WireJournal(tmp_path / "wire")
    for sequence in range(100):
        journal.request("summary", sequence, b"request" + bytes([sequence]), phase="scoring")
        if sequence != 99:
            journal.response("summary", sequence, b"response" + bytes([sequence]), phase="scoring")
    rows = read_journal(tmp_path / "wire")
    assert len(rows) == 100
    assert read_payload(tmp_path / "wire", rows["summary", 0]["response"]) == b"response\0"
    assert rows["summary", 98]["request"]["metadata"] == dict(phase="scoring")
    assert "response" not in rows["summary", 99]
    assert journal.summary()["pending"] == [("summary", 99)]


@pytest.mark.parametrize("fault", ["duplicate", "orphan", "budget", "path", "hash", "truncated"])
def test_capture_fault_cannot_read_as_complete(tmp_path, fault):
    journal = WireJournal(tmp_path / "wire", max_bytes=1 if fault == "budget" else 4096)
    if fault == "budget":
        with pytest.raises(WireJournalError, match="WIRE_CAPTURE_BUDGET"):
            journal.request("roi", 0, b"request")
        assert not list((tmp_path / "wire").iterdir())
        return
    if fault == "orphan":
        with pytest.raises(WireJournalError, match="UNJOINABLE_RESPONSE"):
            journal.response("roi", 0, b"response")
        return
    journal.request("roi", 0, b"request")
    if fault == "duplicate":
        with pytest.raises(WireJournalError, match="DUPLICATE_REQUEST"):
            journal.request("roi", 0, b"request")
        return
    path = tmp_path / "wire/index.jsonl"
    row = json.loads(path.read_text())
    if fault == "path":
        row["filename"] = "../escape"
    elif fault == "hash":
        row["sha256"] = "0" * 64
    else:
        payload = tmp_path / "wire" / row["filename"]
        payload.write_bytes(payload.read_bytes()[:-1])
    path.write_text(json.dumps(row) + "\n")
    with pytest.raises(WireJournalError):
        read_journal(tmp_path / "wire")


def test_unindexed_message_or_crc_damage_is_not_complete(tmp_path):
    journal = WireJournal(tmp_path / "wire")
    journal.request("roi", 0, b"request")
    journal.response("roi", 0, b"response")
    index = tmp_path / "wire/index.jsonl"
    original = index.read_text()
    index.write_text(original.splitlines()[0] + "\n")
    with pytest.raises(WireJournalError, match="UNINDEXED_WIRE"):
        read_journal(tmp_path / "wire")
    index.write_text(original)
    payload = tmp_path / "wire/roi-0.response.cdr.gz"
    damaged = bytearray(payload.read_bytes())
    damaged[-8] ^= 1
    payload.write_bytes(damaged)
    with pytest.raises(WireJournalError, match="WIRE_COMPRESSION_CORRUPTED"):
        read_journal(tmp_path / "wire")
