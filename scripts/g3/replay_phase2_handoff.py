#!/usr/bin/env python3
"""Replay attempt A using its measured receive clock and exact joint rows."""
import argparse
import json
from pathlib import Path

from luggage_gazebo.handoff_settle import HandoffPolicy, await_handoff


def replay(root):
    dump = root / "attempt_A/driver_out/dumps/box_00_HANDOFF_SETTLE_TIMEOUT"
    original = json.loads((dump / "handoff.json").read_text())
    rows = [json.loads(line) for line in
            (dump / "replay/joint_ring.jsonl").read_text().splitlines()]
    start = original["action_result"]["result_mono"]
    rows = [row for row in rows if row["rx_mono"] >= start]
    clock = [start]
    cursor = [0]

    def read():
        fresh = []
        while cursor[0] < len(rows) and rows[cursor[0]]["rx_mono"] <= clock[0]:
            row = rows[cursor[0]]
            fresh.append((row, row["rx_mono"]))
            cursor[0] += 1
        return fresh

    def sleep(seconds):
        clock[0] += seconds

    names = rows[0]["name"]
    endpoints = original["continuity"]
    result = await_handoff(
        HandoffPolicy(**original["policy"]),
        (names, endpoints["previous_last_joints"]),
        (names, endpoints["next_first_joints"]),
        read_samples=read, now_mono=lambda: clock[0], sleep=sleep,
        started_mono=start)
    return dict(
        status=result["status"], code=result["code"],
        accepted_samples=len(result["samples"]),
        last_sample_age_at_original_deadline_s=original["deadline_mono"]-rows[-1]["rx_mono"],
        receive_span_s=rows[-1]["rx_mono"]-rows[0]["rx_mono"],
        sim_span_s=rows[-1]["stamp"]-original["action_result"]["result_sim_stamp"],
        final_start_check=result["final_start_check"],
        last_verdict=result["last_verdict"], rejected=result["rejected"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("evidence_root", type=Path)
    print(json.dumps(replay(parser.parse_args().evidence_root), indent=2))
