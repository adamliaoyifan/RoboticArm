#!/usr/bin/env python3
"""Fetch mailbox rows assigned to this agent and keep a local queue.

Cursor and Claude sessions are file-only: they cannot receive `codex queue`.
This poller lists matching OPEN.md rows, classifies them as active / ready /
waiting, and writes a queue file. `--watch` prints a wake sentinel only when
the ready set changes so a local agent loop can claim work.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import agent_scheduler as sched  # noqa: E402

QUESTION_KINDS = {"question", "consensus"}
ACTION_ORDER = {"stop": 0, "active": 1, "ready": 2, "wait": 3, "skip": 4}


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def matches_identity(
    row: dict[str, str],
    agent: str,
    model: str,
    include_broadcast: bool,
) -> bool:
    agent_ok = row["to_agent"] in {"any", agent}
    model_ok = row["to_model"] in {"any", model}
    if row["to_agent"] == "any" and row["to_model"] == "any":
        return include_broadcast
    return agent_ok and model_ok


def matches_roles(row: dict[str, str], roles: list[str]) -> bool:
    if not roles:
        return True
    return row["to_role"] in {"any", *roles}


def classify_row(row: dict[str, str], threads_dir: Path) -> tuple[str, str]:
    thread_path = threads_dir / row["thread"]
    if not thread_path.exists():
        return "skip", "thread missing"
    metadata = sched.read_metadata(thread_path)
    if metadata.get("status") in sched.TERMINAL_STATES:
        if sched.thread_claimed(thread_path):
            replacement = sched.replacement_for_stop(thread_path)
            return "stop", f"{metadata.get('status')}; replacement {replacement}"
        return "skip", f"thread {metadata.get('status')}"
    if metadata.get("status") == "done":
        return "skip", "thread done"
    if row["kind"] in QUESTION_KINDS:
        return "ready", "awaiting reply"
    if row["kind"] not in sched.RUNNABLE_KINDS:
        return "skip", f"unsupported kind {row['kind']}"
    ready_for_dispatch, dispatch_reason = sched.reviewers_dispatch_ready(metadata)
    if not ready_for_dispatch:
        return "wait", dispatch_reason
    if sched.thread_claimed(thread_path):
        return "active", "already claimed"
    ready, reason = sched.dependencies_ready(row, threads_dir)
    if not ready:
        return "wait", reason
    return "ready", "dependencies passed; claim with agent_start.sh"


def item_from_row(
    row: dict[str, str], action: str, reason: str
) -> dict[str, str]:
    return {
        "id": row["id"],
        "kind": row["kind"],
        "parent": row["parent"],
        "subtask": row["subtask"],
        "depends_on": row["depends_on"],
        "revision": row["revision"],
        "to_role": row["to_role"],
        "to_agent": row["to_agent"],
        "to_model": row["to_model"],
        "thread": row["thread"],
        "request": row["request"],
        "action": action,
        "reason": reason,
    }


def pick_next(items: list[dict[str, str]]) -> dict[str, str] | None:
    stopped = [item for item in items if item["action"] == "stop"]
    if stopped:
        return stopped[0]
    active = [item for item in items if item["action"] == "active"]
    if active:
        return active[0]
    ready_runnable = [
        item
        for item in items
        if item["action"] == "ready" and item["kind"] in sched.RUNNABLE_KINDS
    ]
    if ready_runnable:
        return ready_runnable[0]
    ready_other = [item for item in items if item["action"] == "ready"]
    if ready_other:
        return ready_other[0]
    return None


def snapshot(
    open_path: Path,
    agent: str,
    model: str,
    roles: list[str],
    include_broadcast: bool,
) -> dict[str, Any]:
    threads_dir = open_path.parent
    rows = sched.read_table(open_path, sched.OPEN_COLUMNS)
    mine: list[dict[str, str]] = []
    for row in rows:
        if not matches_identity(row, agent, model, include_broadcast):
            continue
        if not matches_roles(row, roles):
            continue
        action, reason = classify_row(row, threads_dir)
        if action == "skip":
            continue
        mine.append(item_from_row(row, action, reason))
    mine.sort(
        key=lambda item: (
            ACTION_ORDER.get(item["action"], 9),
            0 if item["kind"] in sched.RUNNABLE_KINDS else 1,
            item["id"],
        )
    )
    next_item = pick_next(mine)
    return {
        "polled_at": now_iso(),
        "agent": agent,
        "model": model,
        "roles": roles,
        "mine": mine,
        "stop": [item for item in mine if item["action"] == "stop"],
        "active": [item for item in mine if item["action"] == "active"],
        "ready": [item for item in mine if item["action"] == "ready"],
        "waiting": [item for item in mine if item["action"] == "wait"],
        "queue": [item["id"] for item in mine],
        "next": next_item,
    }


def ready_key(data: dict[str, Any]) -> tuple[str, ...]:
    return tuple(item["id"] for item in data.get("ready", []))


def write_queue(path: Path | None, data: dict[str, Any]) -> None:
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def print_text(data: dict[str, Any]) -> None:
    mine = data["mine"]
    if not mine:
        print(f"no mailbox rows for {data['agent']}/{data['model']}")
        return
    print("id\taction\tkind\tsubtask\treason")
    for item in mine:
        print(
            f"{item['id']}\t{item['action']}\t{item['kind']}\t"
            f"{item['subtask']}\t{item['reason']}"
        )
    nxt = data["next"]
    if nxt is None:
        print("next\tnone")
        return
    print(f"next\t{nxt['id']}\t{nxt['action']}\t{nxt['kind']}")


def refresh_heartbeat(
    registry: Path,
    register_id: str,
    register_script: Path,
    quiet: bool = False,
) -> None:
    sessions = sched.read_table(registry, sched.REGISTRY_COLUMNS)
    session = next((row for row in sessions if row["id"] == register_id), None)
    if session is None:
        raise SystemExit(f"unknown registry id: {register_id}")
    cmd = [
        str(register_script),
        "--id",
        session["id"],
        "--role",
        session["role"],
        "--agent",
        session["agent"],
        "--model",
        session["model"],
        "--cli",
        session["cli"],
        "--session",
        session["session"],
        "--worktree",
        session["worktree"],
        "--state",
        session["state"],
        "--capabilities",
        session["capabilities"],
        "--registry",
        str(registry),
    ]
    subprocess.run(
        cmd,
        check=True,
        stdout=subprocess.DEVNULL if quiet else None,
    )


def wake_line(prompt: str) -> str:
    payload = json.dumps({"prompt": prompt}, ensure_ascii=True)
    return f"AGENT_LOOP_WAKE_mailbox {payload}"


def execution_prompt(item: dict[str, str], agent: str, model: str, cli: str) -> str:
    return "\n".join(
        [
            "You claimed an owner-assigned runnable mailbox task.",
            "",
            f"- id: {item['id']}",
            f"- parent: {item['parent']}",
            f"- subtask: {item['subtask']}",
            f"- kind: {item['kind']}",
            f"- revision: {item['revision']}",
            f"- owner: {item['to_role']}/{agent}/{model}",
            f"- cli: {cli}",
            f"- thread: docs/agents/discuss/{item['thread']}",
            f"- request: {item['request']}",
            "",
            "Execute this task end to end in the current session: read the "
            "thread and pointers, implement or audit within scope, run the "
            "required tests, repair failures, write evidence/role notes, and "
            "close the same thread with scripts/agent_complete.sh.",
        ]
    )


def default_wake_prompt(agent: str, model: str) -> str:
    return (
        f"Poll mailbox with scripts/agent_poll_self.py --agent {agent} "
        f"--model {model}. If next is a ready question or consensus, reply on "
        "that thread. If next is a ready subtask, integration, or regression, "
        "claim with scripts/agent_start.sh and execute it end to end. At most "
        "one runnable at a time. Do not take other agents' rows. If next is "
        "waiting or missing, refresh heartbeat and wait."
    )


def parse_roles(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def default_queue_path(agent: str, model: str) -> Path:
    safe = f"{agent}_{model}".replace("/", "_")
    return Path(f"/tmp/elfin_agent_queue_{safe}.json")


def maybe_claim_next(data: dict[str, Any], args: argparse.Namespace) -> None:
    next_item = data.get("next")
    if not (
        args.claim_next
        and next_item
        and next_item["action"] == "ready"
        and next_item["kind"] in sched.RUNNABLE_KINDS
    ):
        return
    subprocess.run(
        [
            str(args.start_script),
            "--thread",
            next_item["thread"],
            "--role",
            next_item["to_role"],
            "--agent",
            args.agent,
            "--model",
            args.model,
            "--cli",
            args.cli,
        ],
        check=True,
    )
    data["claimed"] = next_item
    data["execute_prompt"] = execution_prompt(
        next_item, args.agent, args.model, args.cli
    )


def run_once(args: argparse.Namespace, previous_ready: tuple[str, ...] | None) -> tuple[dict[str, Any], int]:
    data = snapshot(
        args.open,
        args.agent,
        args.model,
        args.roles,
        args.include_broadcast,
    )
    previous: dict[str, Any] | None = None
    if args.queue_file.exists():
        try:
            previous = json.loads(args.queue_file.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            previous = None
    if previous_ready is None:
        previous_ready = ready_key(previous) if previous else ()
    current_ready = ready_key(data)
    changed = current_ready != previous_ready
    data["changed"] = changed
    maybe_claim_next(data, args)
    write_queue(args.queue_file, data)
    if args.refresh_register_id:
        refresh_heartbeat(
            args.registry,
            args.refresh_register_id,
            args.register_script,
            quiet=False,
        )
    if args.json:
        print(json.dumps(data, indent=2))
    else:
        print_text(data)
        if data.get("execute_prompt"):
            print("")
            print(data["execute_prompt"])
    if args.changed_exit and changed and current_ready:
        return data, 2
    return data, 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--agent", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--cli", default="codex")
    parser.add_argument(
        "--roles",
        default="",
        help="Optional comma-separated to_role filter. Empty matches any role.",
    )
    parser.add_argument(
        "--open",
        type=Path,
        default=Path("docs/agents/discuss/OPEN.md"),
    )
    parser.add_argument(
        "--registry",
        type=Path,
        default=Path("docs/agents/RUNTIME.md"),
    )
    parser.add_argument(
        "--queue-file",
        type=Path,
        default=None,
        help="JSON queue path. Defaults to /tmp/elfin_agent_queue_<agent>_<model>.json",
    )
    parser.add_argument("--json", action="store_true")
    parser.add_argument(
        "--include-broadcast",
        action="store_true",
        help="Also queue to_agent=any and to_model=any rows.",
    )
    parser.add_argument(
        "--changed-exit",
        action="store_true",
        help="Exit 2 when the ready set changed and is non-empty.",
    )
    parser.add_argument("--watch", action="store_true")
    parser.add_argument("--interval", type=float, default=45.0)
    parser.add_argument("--refresh-register-id", default="")
    parser.add_argument(
        "--register-script",
        type=Path,
        default=SCRIPTS_DIR / "agent_register.sh",
    )
    parser.add_argument(
        "--start-script",
        type=Path,
        default=SCRIPTS_DIR / "agent_start.sh",
    )
    parser.add_argument(
        "--claim-next",
        action="store_true",
        help="Claim the next ready runnable row for this agent/model.",
    )
    parser.add_argument(
        "--wake-prompt",
        default="",
        help="Prompt payload printed on ready-set changes in --watch mode.",
    )
    args = parser.parse_args()
    args.roles = parse_roles(args.roles)
    if args.queue_file is None:
        args.queue_file = default_queue_path(args.agent, args.model)
    if not args.wake_prompt:
        args.wake_prompt = default_wake_prompt(args.agent, args.model)

    if not args.watch:
        _data, code = run_once(args, None)
        return code

    previous_ready: tuple[str, ...] | None = None
    first = True
    while True:
        data = snapshot(
            args.open,
            args.agent,
            args.model,
            args.roles,
            args.include_broadcast,
        )
        current_ready = ready_key(data)
        data["changed"] = current_ready != (previous_ready or ())
        if current_ready and (first or data["changed"]):
            maybe_claim_next(data, args)
        write_queue(args.queue_file, data)
        if args.refresh_register_id:
            refresh_heartbeat(
                args.registry,
                args.refresh_register_id,
                args.register_script,
                quiet=args.watch,
            )
        if not first and current_ready and current_ready != previous_ready:
            print(
                wake_line(data.get("execute_prompt") or args.wake_prompt),
                flush=True,
            )
        elif first and data.get("execute_prompt"):
            print(wake_line(data["execute_prompt"]), flush=True)
        previous_ready = current_ready
        first = False
        time.sleep(args.interval)


if __name__ == "__main__":
    raise SystemExit(main())
