"""Binary record schema shared with the C++ recorder (records.hpp).

All structs are #pragma pack(1) little-endian on this platform; dtypes below
must match sizeof() exactly (validated by tests against the manifest).
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import struct
import json

import numpy as np

K_MAX_JOINTS = 16

HEADER_MAGIC = b"JCC1"
FOOTER_MAGIC = b"JCCF"

HEADER_DTYPE = np.dtype(
    [
        ("magic", "S4"),
        ("header_size", "<u4"),
        ("record_size", "<u4"),
        ("record_kind", "<u4"),
        ("reserved", "<u4"),
        ("wall_mono_start_ns", "<u8"),
        ("wall_system_start_ns", "<u8"),
        ("watermark", "S64"),
        ("epoch", "S40"),
    ]
)
FOOTER_DTYPE = np.dtype(
    [
        ("magic", "S4"),
        ("footer_size", "<u4"),
        ("records_written", "<u8"),
        ("close_reason_code", "<u4"),
    ]
)

STREAM_DTYPES = {
    1: ("plugin_write", np.dtype(
        [
            ("seq", "<u8"),
            ("sim_time_ns", "<u8"),
            ("wall_mono_ns", "<u8"),
            ("n_joints", "<u4"),
            ("update_rate", "<i4"),
            ("cmd_received", "<f8", (K_MAX_JOINTS,)),
            ("position_used", "<f8", (K_MAX_JOINTS,)),
            ("write_applied", "<f8", (K_MAX_JOINTS,)),
            ("branch", "u1", (K_MAX_JOINTS,)),
            ("outcome", "u1", (K_MAX_JOINTS,)),
            ("dropped_before", "<u8"),
        ])),
    2: ("plugin_read", np.dtype(
        [
            ("seq", "<u8"),
            ("sim_time_ns", "<u8"),
            ("wall_mono_ns", "<u8"),
            ("n_joints", "<u4"),
            ("flags", "u1", (K_MAX_JOINTS,)),
            ("position", "<f8", (K_MAX_JOINTS,)),
            ("velocity", "<f8", (K_MAX_JOINTS,)),
            ("effort", "<f8", (K_MAX_JOINTS,)),
            ("dropped_before", "<u8"),
        ])),
    3: ("iteration", np.dtype(
        [
            ("seq", "<u8"),
            ("iterations", "<u8"),
            ("sim_time_ns", "<u8"),
            ("real_time_ns", "<u8"),
            ("dt_ns", "<u8"),
            ("paused", "u1"),
        ])),
    4: ("events", np.dtype(
        [
            ("event_id", "<u8"),
            ("wall_mono_ns", "<u8"),
            ("sim_time_ns", "<u8"),
            ("kind", "u1"),
            ("reserved", "u1", (7,)),
            ("text", "S96"),
        ])),
}

WRITE_BRANCH_NAMES = {
    0: "NONE",
    1: "SKIP_NULL_ENTITY",
    2: "VELOCITY",
    3: "POSITION_PV",
    4: "EFFORT",
    5: "FALLBACK_ZERO_VEL",
    6: "NO_METHOD",
}
WRITE_OUTCOME_NAMES = {
    0: "NONE",
    1: "CREATED",
    2: "WRITTEN",
    3: "RETAINED_EMPTY",
    4: "SKIPPED",
}
READ_FLAG_NAMES = {0: "OK", 1: "SKIP_NULL", 2: "MISSING_COMPONENT"}
EVENT_KIND_NAMES = {
    0: "INTERNAL",
    1: "CONFIGURED",
    2: "TRIGGER",
    3: "CLOSED",
    4: "FAULT",
    5: "TRUNCATED",
    6: "PAUSE_CHANGE",
}


@dataclass
class StreamFile:
    name: str
    path: Path
    header: dict
    records: np.ndarray
    footer: dict


def read_stream(path: str | Path) -> StreamFile:
    """Read one recorder stream file: header + records + footer."""
    path = Path(path)
    raw = path.read_bytes()
    if len(raw) < HEADER_DTYPE.itemsize + FOOTER_DTYPE.itemsize:
        raise ValueError(f"{path}: truncated file ({len(raw)} bytes)")
    header = np.frombuffer(raw[: HEADER_DTYPE.itemsize], dtype=HEADER_DTYPE)[0]
    if header["magic"] != HEADER_MAGIC:
        raise ValueError(f"{path}: bad header magic {header['magic']!r}")
    kind = int(header["record_kind"])
    if kind not in STREAM_DTYPES:
        raise ValueError(f"{path}: unknown record kind {kind}")
    name, dtype = STREAM_DTYPES[kind]
    if int(header["record_size"]) != dtype.itemsize:
        raise ValueError(
            f"{path}: record size {int(header['record_size'])} != schema "
            f"{dtype.itemsize} (vendored plugin/schema mismatch)")
    foot = np.frombuffer(raw[-FOOTER_DTYPE.itemsize:], dtype=FOOTER_DTYPE)[0]
    if foot["magic"] != FOOTER_MAGIC:
        raise ValueError(f"{path}: bad footer magic {foot['magic']!r}")
    body = raw[HEADER_DTYPE.itemsize: -FOOTER_DTYPE.itemsize]
    n_expected = int(foot["records_written"])
    if len(body) % dtype.itemsize != 0:
        raise ValueError(f"{path}: body not a whole number of records")
    records = np.frombuffer(body, dtype=dtype)
    header_dict = {
        "watermark": header["watermark"].decode(errors="replace").rstrip("\x00"),
        "epoch": header["epoch"].decode(errors="replace").rstrip("\x00"),
        "wall_mono_start_ns": int(header["wall_mono_start_ns"]),
        "wall_system_start_ns": int(header["wall_system_start_ns"]),
        "record_kind": kind,
        "record_size": dtype.itemsize,
    }
    footer_dict = {
        "records_written": n_expected,
        "close_reason_code": int(foot["close_reason_code"]),
        "records_present": len(records),
    }
    if len(records) != n_expected:
        raise ValueError(
            f"{path}: footer says {n_expected} records, found {len(records)}")
    return StreamFile(name=name, path=path, header=header_dict,
                      records=records, footer=footer_dict)


def load_run(run_dir: str | Path) -> dict:
    """Load all streams + manifest of one capture run directory."""
    run_dir = Path(run_dir)
    out = {"run_dir": run_dir}
    manifest_path = run_dir / "manifest_recorder.json"
    if manifest_path.exists():
        out["manifest"] = json.loads(manifest_path.read_text())
    for kind, (name, _dtype) in STREAM_DTYPES.items():
        p = run_dir / f"{name}.bin"
        if p.exists():
            out[name] = read_stream(p)
    return out
