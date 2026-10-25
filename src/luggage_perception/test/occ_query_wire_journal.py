"""Lossless, bounded evidence for every sent request and completed response.

The ten-second failure ring remains separate. This journal preserves older
revisions needed to replay a complete pressure window without retaining a
whole campaign's raw messages in RAM. It does not alter product callbacks.
"""

import gzip
import hashlib
from io import BytesIO
import json
import zlib
from pathlib import Path

MAX_WIRE_BYTES = 4 * 1024**3
MAX_MESSAGE_BYTES = 64 * 1024**2


class WireJournalError(RuntimeError):
    pass


class WireJournal:
    def __init__(self, root, *, max_bytes=MAX_WIRE_BYTES):
        if type(max_bytes) is not int or max_bytes <= 0:
            raise WireJournalError("INVALID_WIRE_BUDGET")
        self.root = Path(root)
        self.root.mkdir(exist_ok=False)
        self.max_bytes = max_bytes
        self.bytes_written = 0
        self.requests = set()
        self.responses = set()

    def _append(self, direction, case, sequence, payload, metadata):
        key = (case, sequence)
        if not case.replace("_", "").isalnum() or type(sequence) is not int:
            raise WireJournalError("INVALID_WIRE_ID")
        if len(payload) > MAX_MESSAGE_BYTES:
            raise WireJournalError("WIRE_MESSAGE_LIMIT")
        if direction == "request":
            if key in self.requests:
                raise WireJournalError("DUPLICATE_REQUEST")
        elif key not in self.requests or key in self.responses:
            raise WireJournalError("UNJOINABLE_RESPONSE")
        filename = f"{case}-{sequence}.{direction}.cdr.gz"
        compressed = gzip.compress(payload, compresslevel=1, mtime=0)
        record = dict(
            direction=direction,
            case=case,
            sequence=sequence,
            filename=filename,
            raw_bytes=len(payload),
            compressed_bytes=len(compressed),
            sha256=hashlib.sha256(payload).hexdigest(),
            metadata=metadata,
        )
        line = (json.dumps(record, sort_keys=True) + "\n").encode()
        if self.bytes_written + len(compressed) + len(line) > self.max_bytes:
            raise WireJournalError("WIRE_CAPTURE_BUDGET")
        (self.root / filename).write_bytes(compressed)
        with (self.root / "index.jsonl").open("ab") as stream:
            stream.write(line)
        self.bytes_written += len(compressed) + len(line)
        (self.requests if direction == "request" else self.responses).add(key)

    def request(self, case, sequence, payload, **metadata):
        self._append("request", case, sequence, payload, metadata)

    def response(self, case, sequence, payload, **metadata):
        self._append("response", case, sequence, payload, metadata)

    def summary(self):
        return dict(
            requests=len(self.requests),
            responses=len(self.responses),
            pending=sorted(self.requests - self.responses),
            compressed_bytes=self.bytes_written,
            maximum_bytes=self.max_bytes,
        )


def read_journal(root):
    """Validate every byte/join; missing responses remain explicit pending work."""
    root = Path(root)
    entries = {}
    indexed_files = set()
    for line in (root / "index.jsonl").read_text().splitlines():
        record = json.loads(line)
        direction = record["direction"]
        if direction not in ("request", "response"):
            raise WireJournalError("INVALID_WIRE_DIRECTION")
        case, sequence = record["case"], record["sequence"]
        if (
            not isinstance(case, str)
            or not case.replace("_", "").isalnum()
            or type(sequence) is not int
        ):
            raise WireJournalError("INVALID_WIRE_ID")
        expected = f"{case}-{sequence}.{direction}.cdr.gz"
        if record["filename"] != expected:
            raise WireJournalError("INVALID_WIRE_PATH")
        key = (case, sequence)
        item = entries.setdefault(key, {})
        if direction in item or (direction == "response" and "request" not in item):
            raise WireJournalError("UNJOINABLE_WIRE")
        raw_size = record["raw_bytes"]
        if type(raw_size) is not int or not 0 <= raw_size <= MAX_MESSAGE_BYTES:
            raise WireJournalError("WIRE_MESSAGE_LIMIT")
        read_payload(root, record)
        indexed_files.add(expected)
        # Keep metadata only. A baseline full-volume campaign can contain
        # gigabytes of raw messages even when its compressed journal is small.
        item[direction] = record
    if indexed_files != {path.name for path in root.glob("*.cdr.gz")}:
        raise WireJournalError("UNINDEXED_WIRE")
    return entries


def read_payload(root, record):
    path = Path(root) / record["filename"]
    if path.stat().st_size != record["compressed_bytes"]:
        raise WireJournalError("WIRE_LENGTH_MISMATCH")
    raw_size = record["raw_bytes"]
    try:
        with gzip.GzipFile(fileobj=BytesIO(path.read_bytes())) as stream:
            payload = stream.read(raw_size + 1)
    except (OSError, EOFError, zlib.error) as error:
        raise WireJournalError("WIRE_COMPRESSION_CORRUPTED") from error
    if len(payload) != raw_size or hashlib.sha256(payload).hexdigest() != record["sha256"]:
        raise WireJournalError("WIRE_HASH_MISMATCH")
    return payload
