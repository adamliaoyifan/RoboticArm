#!/usr/bin/env python3
"""Summarize owner-closed subtask timing from discuss threads."""

from __future__ import annotations

import argparse
import json
import re
from datetime import datetime
from pathlib import Path
from typing import Any


POST_RE = re.compile(
    r"^## Post -- .+ -- (?P<at>\d{4}-\d{2}-\d{2} \d{2}:\d{2}) -- .+$"
)
FIELD_RE = re.compile(r"^- (?P<key>[a-z_]+):\s*(?P<value>.+)$")
RUNNABLE_KINDS = {"subtask", "integration", "regression"}


def _parse_time(value: str) -> datetime:
    parsed = datetime.fromisoformat(value)
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=datetime.now().astimezone().tzinfo)
    return parsed


def _minutes(start: datetime | None, end: datetime | None) -> float | None:
    if start is None or end is None:
        return None
    return round((end - start).total_seconds() / 60.0, 2)


def parse_thread(path: Path) -> dict[str, Any] | None:
    metadata: dict[str, str] = {}
    runtime: dict[str, str] = {}
    dispatch_at: datetime | None = None
    in_header = True

    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("## "):
            in_header = False
        match = FIELD_RE.match(line)
        if match:
            target = metadata if in_header else runtime
            target[match.group("key")] = match.group("value").strip()
        if dispatch_at is None:
            post_match = POST_RE.match(line)
            if post_match:
                dispatch_at = _parse_time(post_match.group("at"))

    if metadata.get("kind") not in RUNNABLE_KINDS:
        return None

    started_at = (
        _parse_time(runtime["started_at"]) if "started_at" in runtime else None
    )
    completed_at = (
        _parse_time(runtime["completed_at"])
        if "completed_at" in runtime
        else None
    )
    return {
        "thread": path.name,
        "kind": metadata.get("kind", "unknown"),
        "parent": metadata.get("parent", "n/a"),
        "subtask": metadata.get("subtask", "n/a"),
        "depends_on": metadata.get("depends_on", "none"),
        "status": metadata.get("status", "unknown"),
        "outcome": runtime.get("outcome", "pending"),
        "dispatch_at": dispatch_at,
        "started_at": started_at,
        "completed_at": completed_at,
        "queue_wait_min": _minutes(dispatch_at, started_at),
        "owner_cycle_min": _minutes(started_at, completed_at),
    }


def summarize(rows: list[dict[str, Any]], parent: str) -> dict[str, Any]:
    dispatches = [row["dispatch_at"] for row in rows if row["dispatch_at"]]
    completions = [row["completed_at"] for row in rows if row["completed_at"]]
    cycles = [
        row["owner_cycle_min"]
        for row in rows
        if row["owner_cycle_min"] is not None
    ]
    all_passed = bool(rows) and all(
        row["status"] == "done" and row["outcome"] == "pass" for row in rows
    )
    lead_time = None
    if all_passed and dispatches and completions:
        lead_time = _minutes(min(dispatches), max(completions))
    parallel_efficiency = None
    if lead_time and cycles:
        parallel_efficiency = round(sum(cycles) / lead_time, 3)
    return {
        "parent": parent,
        "subtasks": len(rows),
        "completed": sum(
            row["status"] == "done" and row["outcome"] == "pass"
            for row in rows
        ),
        "all_passed": all_passed,
        "unified_lead_time_min": lead_time,
        "parallel_efficiency": parallel_efficiency,
    }


def _json_ready(value: Any) -> Any:
    if isinstance(value, datetime):
        return value.isoformat()
    if isinstance(value, dict):
        return {key: _json_ready(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_ready(item) for item in value]
    return value


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--parent", required=True, help="Unified parent task id")
    parser.add_argument(
        "--threads-dir",
        type=Path,
        default=Path("docs/agents/discuss"),
        help="Directory containing discuss threads",
    )
    parser.add_argument("--json", action="store_true", help="Emit JSON")
    args = parser.parse_args()

    rows = []
    for path in sorted(args.threads_dir.glob("*.md")):
        row = parse_thread(path)
        if row and row["parent"] == args.parent:
            rows.append(row)
    summary = summarize(rows, args.parent)

    if args.json:
        print(json.dumps(_json_ready({"summary": summary, "rows": rows}), indent=2))
        return 0

    print(
        "subtask\tstatus\toutcome\tqueue_wait_min\towner_cycle_min\tdepends_on"
    )
    for row in rows:
        queue_wait = row["queue_wait_min"]
        owner_cycle = row["owner_cycle_min"]
        print(
            f"{row['subtask']}\t{row['status']}\t{row['outcome']}\t"
            f"{queue_wait if queue_wait is not None else 'pending'}\t"
            f"{owner_cycle if owner_cycle is not None else 'pending'}\t"
            f"{row['depends_on']}"
        )
    print(
        f"summary\t{summary['completed']}/{summary['subtasks']} complete\t"
        f"lead={summary['unified_lead_time_min']}\t"
        f"parallel={summary['parallel_efficiency']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
