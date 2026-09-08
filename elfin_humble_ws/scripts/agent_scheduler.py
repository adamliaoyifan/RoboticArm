#!/usr/bin/env python3
"""Plan and optionally dispatch ready owner-assigned agent mailbox work."""

from __future__ import annotations

import argparse
import json
import subprocess
import time
from dataclasses import dataclass
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any


RUNNABLE_KINDS = {"subtask", "integration", "regression"}
TERMINAL_STATES = {"superseded", "cancelled"}
OPEN_COLUMNS = [
    "id",
    "kind",
    "parent",
    "subtask",
    "depends_on",
    "revision",
    "to_role",
    "to_agent",
    "to_model",
    "from_role",
    "from_agent",
    "from_model",
    "cli",
    "thread",
    "request",
    "generation",
    "plan_revision",
]
REGISTRY_COLUMNS = [
    "id",
    "role",
    "agent",
    "model",
    "cli",
    "session",
    "worktree",
    "state",
    "capabilities",
    "heartbeat",
]


@dataclass(frozen=True)
class Dispatch:
    mailbox_id: str
    thread: str
    parent: str
    subtask: str
    kind: str
    owner: str
    cli: str
    session: str
    worktree: str
    action: str
    reason: str


def split_md_row(line: str) -> list[str]:
    cells: list[str] = []
    cell: list[str] = []
    escaped = False
    body = line.strip()
    if body.startswith("|"):
        body = body[1:]
    if body.endswith("|"):
        body = body[:-1]
    for char in body:
        if escaped:
            cell.append(char)
            escaped = False
        elif char == "\\":
            escaped = True
        elif char == "|":
            cells.append("".join(cell).strip())
            cell = []
        else:
            cell.append(char)
    cells.append("".join(cell).strip())
    return cells


def read_table(path: Path, columns: list[str]) -> list[dict[str, str]]:
    if not path.exists():
        return []
    rows: list[dict[str, str]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.startswith("|"):
            continue
        cells = split_md_row(line)
        if not cells or cells[0] == columns[0] or set(cells[0]) == {"-"}:
            continue
        if len(cells) < len(columns):
            cells.extend([""] * (len(columns) - len(cells)))
        elif len(cells) > len(columns):
            cells = cells[: len(columns) - 1] + [" | ".join(cells[len(columns) - 1 :])]
        rows.append(dict(zip(columns, cells, strict=True)))
    return rows


def read_metadata(path: Path) -> dict[str, str]:
    metadata: dict[str, str] = {}
    if not path.exists():
        return metadata
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("## "):
            break
        if not line.startswith("- ") or ": " not in line:
            continue
        key, value = line[2:].split(": ", 1)
        metadata[key.strip()] = value.strip()
    return metadata


def thread_claimed(path: Path) -> bool:
    text = path.read_text(encoding="utf-8")
    return "## Claim -- " in text and "\n- started_at: " in text


def parse_generation(value: str | None) -> int:
    if not value:
        return 1
    if not value.isdigit() or int(value) < 1:
        return 0
    return int(value)


def latest_event_fields(path: Path, prefixes: tuple[str, ...]) -> dict[str, str]:
    fields: dict[str, str] = {}
    active = False
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("## "):
            active = any(line.startswith(prefix) for prefix in prefixes)
            if active:
                fields = {}
            continue
        if active and line.startswith("- ") and ": " in line:
            key, value = line[2:].split(": ", 1)
            fields[key.strip()] = value.strip()
    return fields


def replacement_for_stop(path: Path) -> str:
    fields = latest_event_fields(path, ("## Superseded --", "## Cancelled --"))
    return fields.get("replacement", "n/a")


def result_passed(path: Path) -> bool:
    text = path.read_text(encoding="utf-8")
    return "## Result -- " in text and "\n- outcome: pass\n" in text


def dependency_passed(threads_dir: Path, parent: str, subtask: str) -> bool:
    current: tuple[int, Path, dict[str, str]] | None = None
    for path in threads_dir.glob("*.md"):
        if path.name in {"OPEN.md", "README.md"}:
            continue
        metadata = read_metadata(path)
        if metadata.get("parent") != parent or metadata.get("subtask") != subtask:
            continue
        if metadata.get("kind") not in RUNNABLE_KINDS:
            continue
        generation = parse_generation(metadata.get("generation"))
        if generation < 1:
            return False
        if current is None or generation > current[0]:
            current = (generation, path, metadata)
    if current is None:
        return False
    _generation, path, metadata = current
    return metadata.get("status") == "done" and result_passed(path)


def dependencies_ready(row: dict[str, str], threads_dir: Path) -> tuple[bool, str]:
    depends_on = row["depends_on"]
    if depends_on == "none":
        return True, "ready"
    missing = [
        dependency.strip()
        for dependency in depends_on.split(",")
        if dependency.strip()
        and not dependency_passed(threads_dir, row["parent"], dependency.strip())
    ]
    if missing:
        return False, "waiting for " + ",".join(missing)
    return True, "ready"


def matches_owner(row: dict[str, str], session: dict[str, str]) -> bool:
    role_match = row["to_role"] in {"any", session["role"]}
    agent_match = row["to_agent"] in {"any", session["agent"]}
    model_match = row["to_model"] in {"any", session["model"]}
    return role_match and agent_match and model_match


def owner_key(row: dict[str, str]) -> tuple[str, str, str]:
    return row["to_role"], row["to_agent"], row["to_model"]


def row_stopped(row: dict[str, str], threads_dir: Path) -> bool:
    if row["kind"] not in RUNNABLE_KINDS:
        return False
    thread_path = threads_dir / row["thread"]
    if not thread_path.exists():
        return False
    metadata = read_metadata(thread_path)
    return metadata.get("status") in TERMINAL_STATES and thread_claimed(thread_path)


def reviewers_dispatch_ready(metadata: dict[str, str]) -> tuple[bool, str]:
    value = metadata.get("dispatch_ready", "")
    if not value:
        return True, "legacy dispatch gate absent"
    if value == "yes":
        return True, "dispatch_ready"
    return False, "waiting for reviewers dispatch_ready"


def capability_set(session: dict[str, str]) -> set[str]:
    return {
        item.strip()
        for item in session.get("capabilities", "").split(",")
        if item.strip()
    }


def parse_time(value: str) -> datetime | None:
    if not value or value == "n/a":
        return None
    try:
        parsed = datetime.fromisoformat(value)
    except ValueError:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=datetime.now().astimezone().tzinfo)
    return parsed


def fresh_session(session: dict[str, str], max_age: timedelta) -> tuple[bool, str]:
    if session.get("state") != "idle":
        return False, f"session state is {session.get('state', 'unknown')}"
    heartbeat = parse_time(session.get("heartbeat", ""))
    if heartbeat is None:
        return False, "missing or invalid heartbeat"
    age = datetime.now(heartbeat.tzinfo) - heartbeat
    if age > max_age:
        return False, f"stale heartbeat {round(age.total_seconds())}s"
    return True, "ready"


def lease_path(leases_dir: Path, thread: str) -> Path:
    return leases_dir / f"{thread}.lease.md"


def active_lease(path: Path, now: datetime) -> tuple[bool, str]:
    metadata = read_metadata(path)
    if metadata.get("status") != "leased":
        return False, "no active lease"
    expires = parse_time(metadata.get("lease_until", ""))
    if expires is None:
        return True, "lease has no valid expiry"
    if expires > now:
        return True, f"leased until {expires.isoformat()}"
    return False, "expired"


def write_lease(path: Path, dispatch: Dispatch, ttl: timedelta) -> None:
    now = datetime.now().astimezone()
    until = now + ttl
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join(
            [
                f"# Scheduler lease -- {dispatch.thread}",
                "",
                "- status: leased",
                f"- thread: {dispatch.thread}",
                f"- parent: {dispatch.parent}",
                f"- subtask: {dispatch.subtask}",
                f"- owner: {dispatch.owner}",
                f"- cli: {dispatch.cli}",
                f"- session: {dispatch.session}",
                f"- worktree: {dispatch.worktree}",
                f"- leased_at: {now.isoformat()}",
                f"- lease_until: {until.isoformat()}",
                "",
                "## Dispatch",
                "",
                dispatch.reason,
                "",
            ]
        ),
        encoding="utf-8",
    )


def owner_message(row: dict[str, str], session: dict[str, str]) -> str:
    return "\n".join(
        [
            "You have an owner-assigned runnable mailbox task.",
            "",
            f"- parent: {row['parent']}",
            f"- subtask: {row['subtask']}",
            f"- kind: {row['kind']}",
            f"- thread: docs/agents/discuss/{row['thread']}",
            f"- request: {row['request']}",
            "",
            "Read AGENTS.md, docs/agents/discuss/OPEN.md, and the thread. "
            "Run scripts/agent_start.sh before editing, then implement, test, "
            "repair failures, write evidence, and close with scripts/agent_complete.sh.",
            "",
            f"Workspace: {session['worktree']}",
        ]
    )


def stop_message(row: dict[str, str], session: dict[str, str], replacement: str) -> str:
    return "\n".join(
        [
            "A claimed mailbox task assigned to you has been stopped.",
            "",
            f"- parent: {row['parent']}",
            f"- subtask: {row['subtask']}",
            f"- kind: {row['kind']}",
            f"- thread: docs/agents/discuss/{row['thread']}",
            f"- replacement: {replacement}",
            f"- request: {row['request']}",
            "",
            "Stop work on the old thread. Read the thread, append a StopAck "
            "with scripts/agent_mailbox.py stop-ack, then poll again for any "
            "replacement work. Do not close the stopped generation as pass.",
            "",
            f"Workspace: {session['worktree']}",
        ]
    )


def queue_codex(row: dict[str, str], session: dict[str, str]) -> None:
    subprocess.run(
        [
            "codex",
            "queue",
            "--thread",
            session["session"],
            "--message",
            owner_message(row, session),
        ],
        check=True,
    )


def queue_codex_stop(row: dict[str, str], session: dict[str, str], replacement: str) -> None:
    subprocess.run(
        [
            "codex",
            "queue",
            "--thread",
            session["session"],
            "--message",
            stop_message(row, session, replacement),
        ],
        check=True,
    )


def plan_dispatches(
    open_rows: list[dict[str, str]],
    sessions: list[dict[str, str]],
    threads_dir: Path,
    leases_dir: Path,
    heartbeat_max_age: timedelta,
    lease_ttl: timedelta,
) -> list[Dispatch]:
    now = datetime.now().astimezone()
    dispatches: list[Dispatch] = []
    stopped_owners = {
        owner_key(row)
        for row in open_rows
        if row_stopped(row, threads_dir)
    }
    for row in open_rows:
        if row["kind"] not in RUNNABLE_KINDS:
            continue
        thread_path = threads_dir / row["thread"]
        if not thread_path.exists():
            dispatches.append(
                Dispatch(
                    row["id"],
                    row["thread"],
                    row["parent"],
                    row["subtask"],
                    row["kind"],
                    f"{row['to_agent']}/{row['to_model']}",
                    "n/a",
                    "n/a",
                    "n/a",
                    "skip",
                    "thread missing",
                )
            )
            continue
        metadata = read_metadata(thread_path)
        if metadata.get("status") in TERMINAL_STATES:
            replacement = replacement_for_stop(thread_path)
            lease = lease_path(leases_dir, f"stop-{row['thread']}")
            is_leased, lease_reason = active_lease(lease, now)
            if is_leased:
                dispatches.append(
                    Dispatch(
                        row["id"],
                        row["thread"],
                        row["parent"],
                        row["subtask"],
                        row["kind"],
                        f"{row['to_agent']}/{row['to_model']}",
                        "n/a",
                        "n/a",
                        "n/a",
                        "skip",
                        f"stop notice {lease_reason}; replacement {replacement}",
                    )
                )
                continue
            candidates = [session for session in sessions if matches_owner(row, session)]
            if not candidates:
                dispatches.append(
                    Dispatch(
                        row["id"],
                        row["thread"],
                        row["parent"],
                        row["subtask"],
                        row["kind"],
                        f"{row['to_agent']}/{row['to_model']}",
                        "n/a",
                        "n/a",
                        "n/a",
                        "stop-file-only",
                        f"stopped; replacement {replacement}; no registered matching session",
                    )
                )
                continue
            for session in candidates:
                fresh, fresh_reason = fresh_session(session, heartbeat_max_age)
                if not fresh:
                    dispatches.append(
                        Dispatch(
                            row["id"],
                            row["thread"],
                            row["parent"],
                            row["subtask"],
                            row["kind"],
                            f"{session['agent']}/{session['model']}",
                            session["cli"],
                            session["session"],
                            session["worktree"],
                            "wait",
                            fresh_reason,
                        )
                    )
                    continue
                capabilities = capability_set(session)
                if session["cli"] == "codex" and "queue" in capabilities:
                    dispatches.append(
                        Dispatch(
                            row["id"],
                            row["thread"],
                            row["parent"],
                            row["subtask"],
                            row["kind"],
                            f"{session['agent']}/{session['model']}",
                            session["cli"],
                            session["session"],
                            session["worktree"],
                            "stop",
                            f"ready for codex stop queue; replacement {replacement}",
                        )
                    )
                    break
                dispatches.append(
                    Dispatch(
                        row["id"],
                        row["thread"],
                        row["parent"],
                        row["subtask"],
                        row["kind"],
                        f"{session['agent']}/{session['model']}",
                        session["cli"],
                        session["session"],
                        session["worktree"],
                        "stop-file-only",
                        f"stopped; replacement {replacement}; registered session has no supported live queue adapter",
                    )
                )
            continue
        if owner_key(row) in stopped_owners:
            dispatches.append(
                Dispatch(
                    row["id"],
                    row["thread"],
                    row["parent"],
                    row["subtask"],
                    row["kind"],
                    f"{row['to_agent']}/{row['to_model']}",
                    "n/a",
                    "n/a",
                    "n/a",
                    "wait",
                    "waiting for owner stop acknowledgement",
                )
            )
            continue
        ready_for_dispatch, dispatch_reason = reviewers_dispatch_ready(metadata)
        if not ready_for_dispatch:
            dispatches.append(
                Dispatch(
                    row["id"],
                    row["thread"],
                    row["parent"],
                    row["subtask"],
                    row["kind"],
                    f"{row['to_agent']}/{row['to_model']}",
                    "n/a",
                    "n/a",
                    "n/a",
                    "wait",
                    dispatch_reason,
                )
            )
            continue
        if thread_claimed(thread_path):
            dispatches.append(
                Dispatch(
                    row["id"],
                    row["thread"],
                    row["parent"],
                    row["subtask"],
                    row["kind"],
                    f"{row['to_agent']}/{row['to_model']}",
                    "n/a",
                    "n/a",
                    "n/a",
                    "skip",
                    "already claimed",
                )
            )
            continue
        lease = lease_path(leases_dir, row["thread"])
        is_leased, lease_reason = active_lease(lease, now)
        if is_leased:
            dispatches.append(
                Dispatch(
                    row["id"],
                    row["thread"],
                    row["parent"],
                    row["subtask"],
                    row["kind"],
                    f"{row['to_agent']}/{row['to_model']}",
                    "n/a",
                    "n/a",
                    "n/a",
                    "skip",
                    lease_reason,
                )
            )
            continue
        ready, reason = dependencies_ready(row, threads_dir)
        if not ready:
            dispatches.append(
                Dispatch(
                    row["id"],
                    row["thread"],
                    row["parent"],
                    row["subtask"],
                    row["kind"],
                    f"{row['to_agent']}/{row['to_model']}",
                    "n/a",
                    "n/a",
                    "n/a",
                    "wait",
                    reason,
                )
            )
            continue
        candidates = [session for session in sessions if matches_owner(row, session)]
        if not candidates:
            dispatches.append(
                Dispatch(
                    row["id"],
                    row["thread"],
                    row["parent"],
                    row["subtask"],
                    row["kind"],
                    f"{row['to_agent']}/{row['to_model']}",
                    "n/a",
                    "n/a",
                    "n/a",
                    "file-only",
                    "no registered matching session",
                )
            )
            continue
        for session in candidates:
            fresh, fresh_reason = fresh_session(session, heartbeat_max_age)
            if not fresh:
                dispatches.append(
                    Dispatch(
                        row["id"],
                        row["thread"],
                        row["parent"],
                        row["subtask"],
                        row["kind"],
                        f"{session['agent']}/{session['model']}",
                        session["cli"],
                        session["session"],
                        session["worktree"],
                        "wait",
                        fresh_reason,
                    )
                )
                continue
            capabilities = capability_set(session)
            if session["cli"] == "codex" and "queue" in capabilities:
                dispatches.append(
                    Dispatch(
                        row["id"],
                        row["thread"],
                        row["parent"],
                        row["subtask"],
                        row["kind"],
                        f"{session['agent']}/{session['model']}",
                        session["cli"],
                        session["session"],
                        session["worktree"],
                        "queue",
                        "ready for codex queue",
                    )
                )
                break
            dispatches.append(
                Dispatch(
                    row["id"],
                    row["thread"],
                    row["parent"],
                    row["subtask"],
                    row["kind"],
                    f"{session['agent']}/{session['model']}",
                    session["cli"],
                    session["session"],
                    session["worktree"],
                    "file-only",
                    "registered session has no supported live queue adapter",
                )
            )
    return dispatches


def as_dict(dispatch: Dispatch) -> dict[str, str]:
    return {
        "mailbox_id": dispatch.mailbox_id,
        "thread": dispatch.thread,
        "parent": dispatch.parent,
        "subtask": dispatch.subtask,
        "kind": dispatch.kind,
        "owner": dispatch.owner,
        "cli": dispatch.cli,
        "session": dispatch.session,
        "worktree": dispatch.worktree,
        "action": dispatch.action,
        "reason": dispatch.reason,
    }


def print_dispatches(dispatches: list[Dispatch], json_output: bool) -> None:
    if json_output:
        print(json.dumps([as_dict(item) for item in dispatches], indent=2))
        return
    if not dispatches:
        print("no runnable mailbox rows")
        return
    print("id\taction\towner\tthread\treason")
    for item in dispatches:
        print(
            f"{item.mailbox_id}\t{item.action}\t{item.owner}\t"
            f"{item.thread}\t{item.reason}"
        )


def dispatch_ready(items: list[Dispatch], rows_by_thread: dict[str, dict[str, str]], lease_ttl: timedelta, leases_dir: Path) -> int:
    failures = 0
    for item in items:
        if item.action not in {"queue", "stop"}:
            continue
        row = rows_by_thread[item.thread]
        try:
            session = {
                "cli": item.cli,
                "session": item.session,
                "worktree": item.worktree,
                "agent": item.owner.split("/", 1)[0],
                "model": item.owner.split("/", 1)[1],
            }
            if item.action == "stop":
                queue_codex_stop(row, session, item.reason.rsplit("replacement ", 1)[-1])
                write_lease(lease_path(leases_dir, f"stop-{item.thread}"), item, lease_ttl)
            else:
                queue_codex(row, session)
                write_lease(lease_path(leases_dir, item.thread), item, lease_ttl)
        except (OSError, subprocess.CalledProcessError) as exc:
            failures += 1
            print(f"dispatch failed for {item.thread}: {exc}", flush=True)
    return failures


def run_once(args: argparse.Namespace) -> int:
    open_rows = read_table(args.open, OPEN_COLUMNS)
    sessions = read_table(args.registry, REGISTRY_COLUMNS)
    dispatches = plan_dispatches(
        open_rows,
        sessions,
        args.open.parent,
        args.leases_dir,
        timedelta(seconds=args.heartbeat_max_age),
        timedelta(seconds=args.lease_ttl),
    )
    print_dispatches(dispatches, args.json)
    if not args.dispatch:
        return 0
    rows_by_thread = {row["thread"]: row for row in open_rows}
    return dispatch_ready(
        dispatches,
        rows_by_thread,
        timedelta(seconds=args.lease_ttl),
        args.leases_dir,
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--open",
        type=Path,
        default=Path("docs/agents/discuss/OPEN.md"),
        help="Mailbox table to inspect.",
    )
    parser.add_argument(
        "--registry",
        type=Path,
        default=Path("docs/agents/RUNTIME.md"),
        help="Runtime session registry table.",
    )
    parser.add_argument(
        "--leases-dir",
        type=Path,
        default=Path("docs/agents/discuss/leases"),
        help="Directory for scheduler lease files.",
    )
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--dispatch", action="store_true", help="Queue ready supported sessions.")
    parser.add_argument("--watch", action="store_true", help="Poll continuously.")
    parser.add_argument("--interval", type=float, default=5.0)
    parser.add_argument("--heartbeat-max-age", type=int, default=300)
    parser.add_argument("--lease-ttl", type=int, default=900)
    args = parser.parse_args()

    if not args.watch:
        return run_once(args)
    while True:
        result = run_once(args)
        if result:
            return result
        time.sleep(args.interval)


if __name__ == "__main__":
    raise SystemExit(main())
