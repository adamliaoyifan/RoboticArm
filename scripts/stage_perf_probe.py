#!/usr/bin/env python3
"""Compare live bag rate with three local replay reads.

Replay never joins ROS domain 7. Throughput degradation is
(live_rate - replay_rate) / live_rate. The median of the repeats must stay
at or below 10 percent for a pass. Topic counts from the storage are the
join check: a short read fails closed.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sqlite3
import sys
import time

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "luggage_gazebo"))

from luggage_gazebo.rehearsal_evidence import (  # noqa: E402
    median_degradation, throughput_degradation, topic_counts_match,
)


def load_metadata(bag):
    path = Path(bag)
    meta = path / "metadata.yaml" if path.is_dir() else path.parent / "metadata.yaml"
    document = yaml.safe_load(meta.read_text(encoding="utf-8"))
    info = document["rosbag2_bagfile_information"]
    duration_s = int(info["duration"]["nanoseconds"]) / 1_000_000_000
    counts = {}
    for row in info.get("topics_with_message_count") or []:
        counts[row["topic_metadata"]["name"]] = int(row["message_count"])
    if duration_s <= 0 or not counts:
        raise ValueError("not_evaluated: bag metadata has no positive duration or topics")
    return duration_s, counts, meta.parent


def storage_files(bag_dir):
    files = sorted(Path(bag_dir).rglob("*.db3"))
    if not files:
        raise ValueError("not_evaluated: no db3 storage in %s" % bag_dir)
    return files


def consume_once(files):
    """Read every stored payload once and return (messages/sec, per-topic counts)."""
    started = time.perf_counter()
    total = 0
    per_topic = {}
    topic_names = {}
    for storage in files:
        connection = sqlite3.connect("file:%s?mode=ro" % storage, uri=True)
        try:
            for topic_id, name in connection.execute("select id, name from topics"):
                topic_names[topic_id] = name
                per_topic.setdefault(name, 0)
            for topic_id, payload in connection.execute(
                    "select topic_id, data from messages"):
                if not payload:
                    raise ValueError("empty payload on topic id %s" % topic_id)
                total += 1
                name = topic_names.get(topic_id, str(topic_id))
                per_topic[name] = per_topic.get(name, 0) + 1
        finally:
            connection.close()
    elapsed = time.perf_counter() - started
    if elapsed <= 0 or total <= 0:
        raise ValueError("not_evaluated: replay read produced no messages")
    return total / elapsed, per_topic


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--domain", type=int, default=42)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if args.domain == 7:
        raise SystemExit("refusing to evaluate a replay on domain 7")
    if args.repeats < 3:
        raise SystemExit("three paired replays are required")
    duration_s, live_counts, bag_dir = load_metadata(args.bag)
    live_total = sum(live_counts.values())
    live_rate = live_total / duration_s
    files = storage_files(bag_dir)
    samples = []
    mismatches = []
    for _ in range(args.repeats):
        rate, counts = consume_once(files)
        samples.append(throughput_degradation(live_rate, rate))
        mismatches.append(topic_counts_match(live_counts, counts))
    median = median_degradation(samples)
    joined = all(not row for row in mismatches)
    result = {
        "domain": args.domain,
        "live_rate_hz": live_rate,
        "degradations": samples,
        "median_degradation": median,
        "pass": joined and median <= 0.10,
        "topic_mismatches": mismatches,
    }
    text = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
