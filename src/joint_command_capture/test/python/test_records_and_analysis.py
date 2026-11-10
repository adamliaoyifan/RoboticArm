"""Schema reader + boundary localization on synthetic runs (offline).

Synthetic files are built with the exact POD layout the C++ recorder writes;
the analysis must reproduce the injected deviation layer and must report
measured (not nominal) L2xL3 coverage.
"""
import struct

import numpy as np
import pytest

from joint_command_capture import analysis
from joint_command_capture.records_schema import (
    FOOTER_DTYPE,
    HEADER_DTYPE,
    STREAM_DTYPES,
    read_stream,
)

GAIN = 0.8
RATE = 100


def _clean_pair(n=100, dt_ns=1_000_000):
    """Consistent write/read streams: P-conversion honored by physics.

    Row dicts use {index: value} for per-joint array fields; _emit expands
    them. Write row i: sim time (i+1)*dt carrying cmd(i) (the one-cycle
    PreUpdate/PostUpdate ordering of the real chain).
    """
    writes, reads = [], []
    pos = 0.0
    for i in range(n):
        t = (i + 1) * dt_ns
        cmd = 0.001 * i
        target = -GAIN * (pos - cmd) * RATE
        writes.append({
            "seq": i, "sim_time_ns": t, "wall_mono_ns": 10 + t,
            "n_joints": 6, "update_rate": RATE,
            "branch": {0: 3},          # POSITION_PV on joint 0
            "outcome": {0: 2},         # WRITTEN
            "cmd_received": {0: cmd},
            "position_used": {0: pos},
            "write_applied": {0: target},
        })
        pos += target * 1e-3           # physics integrates commanded velocity
        reads.append({
            "seq": i, "sim_time_ns": t, "wall_mono_ns": 20 + t,
            "n_joints": 6, "flags": {0: 0},
            "position": {0: pos},
            "velocity": {0: target},
            "effort": {0: 1.0},
        })
    return writes, reads


def _emit(run_dir, writes, reads):
    for kind, rows in ((1, writes), (2, reads)):
        name, dtype = STREAM_DTYPES[kind]
        rec = np.zeros(len(rows), dtype=dtype)
        # Mirror the C++ recorder: per-joint float arrays start as NaN so
        # "unavailable" stays NaN end-to-end.
        for field in dtype.names:
            if dtype[field].base == np.float64 and dtype[field].shape:
                rec[field][:] = np.nan
        for i, row in enumerate(rows):
            for key, value in row.items():
                if isinstance(value, dict):
                    for idx, v in value.items():
                        rec[key][i][idx] = v
                else:
                    rec[key][i] = value
        hdr = np.zeros(1, dtype=HEADER_DTYPE)
        hdr["magic"] = b"JCC1"
        hdr["header_size"] = HEADER_DTYPE.itemsize
        hdr["record_size"] = dtype.itemsize
        hdr["record_kind"] = kind
        hdr["watermark"] = "0.1.0 git:test"
        hdr["epoch"] = "testepoch"
        foot = np.zeros(1, dtype=FOOTER_DTYPE)
        foot["magic"] = b"JCCF"
        foot["records_written"] = len(rows)
        (run_dir / f"{name}.bin").write_bytes(
            hdr.tobytes() + rec.tobytes() + foot.tobytes())


def test_read_stream_roundtrip(tmp_path):
    writes, reads = _clean_pair(10)
    _emit(tmp_path, writes, reads)
    ws = read_stream(tmp_path / "plugin_write.bin")
    assert ws.header["watermark"] == "0.1.0 git:test"
    assert len(ws.records) == 10
    assert ws.records["branch"][0][0] == 3


def test_bad_magic_rejected(tmp_path):
    writes, reads = _clean_pair(2)
    _emit(tmp_path, writes, reads)
    p = tmp_path / "plugin_write.bin"
    p.write_bytes(b"XXXX" + p.read_bytes()[4:])
    with pytest.raises(ValueError):
        read_stream(p)


def test_nan_and_unavailable_preserved_verbatim(tmp_path):
    writes, reads = _clean_pair(2)
    # Joints 1-5 were never filled: they must read back as NaN, not zeros.
    _emit(tmp_path, writes, reads)
    rs = read_stream(tmp_path / "plugin_read.bin")
    for j in range(1, 6):
        assert np.isnan(rs.records["position"][0][j])
        assert np.isnan(rs.records["velocity"][0][j])
        assert np.isnan(rs.records["effort"][0][j])
    ws = read_stream(tmp_path / "plugin_write.bin")
    assert np.isnan(ws.records["cmd_received"][0][1])


def test_verdict_insufficient_evidence_on_clean_run(tmp_path):
    writes, reads = _clean_pair(100)
    _emit(tmp_path, writes, reads)
    v = analysis.analyze_run(tmp_path)
    assert v.verdict == analysis.VERDICT_INSUFFICIENT
    assert v.details["position_gain_estimate_median"] == pytest.approx(
        GAIN, abs=1e-9)
    assert v.details["position_gain_estimate_spread"] < 1e-3


def test_verdict_physics_when_velocity_deviates(tmp_path):
    writes, reads = _clean_pair(100)
    # Inject: physics stops honoring applied velocity from row 60.
    for row in reads[60:]:
        row["velocity"] = {0: row["velocity"][0] + 0.5}
    _emit(tmp_path, writes, reads)
    v = analysis.analyze_run(tmp_path, vel_tol_rad_s=0.05)
    assert v.verdict == analysis.VERDICT_PHYSICS
    assert v.boundary == "plugin_applied_vs_physical_response"
    assert v.first_deviation_sim_ns == 61_000_000


def test_verdict_plugin_when_conversion_inconsistent(tmp_path):
    writes, reads = _clean_pair(50)
    for row in writes[30:]:
        row["write_applied"] = {0: row["write_applied"][0] + 1.0}
    for row in reads:
        row["velocity"] = {0: row["velocity"][0]}   # keep physics consistent
    _emit(tmp_path, writes, reads)
    v = analysis.analyze_run(tmp_path)
    assert v.verdict == analysis.VERDICT_PLUGIN
    assert v.details["position_gain_estimate_spread"] > 1e-3


def test_l2_l3_join_measures_coverage_and_finds_deviation(tmp_path):
    writes, reads = _clean_pair(100)
    _emit(tmp_path, writes, reads)
    dt = 1_000_000
    # JTC publishes at every other control cycle (nominal 50 Hz) with the
    # one-cycle-later write alignment; joint deviates from row 40 on.
    cs_rows = []
    for i in range(0, 100, 2):
        des = writes[i]["cmd_received"][0]
        if i >= 40:
            des += 0.5
        cs_rows.append({"header": {"stamp": {
            "sec": 0, "nanosec": i * dt}},
            "desired": {"positions": [des] + [0.0] * 5}})
    cs = tmp_path / "controller_state.jsonl"
    cs.write_text("\n".join(
        __import__("json").dumps(r) for r in cs_rows) + "\n")
    v = analysis.analyze_run(str(tmp_path), str(cs), pos_tol_rad=0.01)
    assert v.verdict == analysis.VERDICT_UNSEPARATED
    assert v.boundary == "jtc_output_vs_plugin_receive"
    # Measured coverage must equal the true overlap (50/50 rows matched).
    assert v.details["l2_l3_measured_coverage"] == pytest.approx(1.0)
    assert v.details["l2_l3_first_excess_sim_ns"] == 40 * dt
