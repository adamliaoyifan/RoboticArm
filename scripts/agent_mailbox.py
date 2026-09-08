#!/usr/bin/env python3
"""Generation-aware mailbox lifecycle helpers.

This module is intentionally file based. It keeps the existing Markdown
mailbox, but gives owner lifecycle commands one shared parser and one place for
freshness checks.
"""

from __future__ import annotations

import argparse
import fcntl
import os
import re
import shutil
import subprocess
import sys
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterator


RUNNABLE_KINDS = {"subtask", "integration", "regression"}
LEGACY_OPEN_COLUMNS = [
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
]
OPEN_COLUMNS = LEGACY_OPEN_COLUMNS + ["generation", "plan_revision"]
TERMINAL_STATES = {"done", "superseded", "cancelled"}


@dataclass(frozen=True)
class Row:
    cells: dict[str, str]
    line_index: int


def trim(value: str) -> str:
    return value.strip()


def trim_cell(value: str) -> str:
    return value.replace("\n", " ").replace("|", r"\|")


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


def format_row(values: dict[str, str], columns: list[str]) -> str:
    return "| " + " | ".join(trim_cell(values.get(column, "")) for column in columns) + " |"


def read_open(path: Path) -> tuple[list[str], list[Row], list[str]]:
    lines = path.read_text(encoding="utf-8").splitlines()
    columns: list[str] | None = None
    rows: list[Row] = []
    for index, line in enumerate(lines):
        if not line.startswith("|"):
            continue
        cells = split_md_row(line)
        if cells and cells[0] == "id":
            columns = cells
            continue
        if not cells or set(cells[0]) == {"-"}:
            continue
        if columns is None:
            continue
        if len(cells) < len(columns):
            cells.extend([""] * (len(columns) - len(cells)))
        elif len(cells) > len(columns):
            cells = cells[: len(columns) - 1] + [" | ".join(cells[len(columns) - 1 :])]
        rows.append(Row(dict(zip(columns, cells, strict=True)), index))
    if columns is None:
        columns = LEGACY_OPEN_COLUMNS
    return columns, rows, lines


def write_open(path: Path, columns: list[str], rows: list[dict[str, str]], prefix_lines: list[str] | None = None) -> None:
    prefix = prefix_lines
    if prefix is None:
        prefix = [
            "# Open cross-agent work",
            "",
            "Read this at session start. Claim a row only when `to_agent` and `to_model`",
            "match your concrete identity and every `depends_on` item is complete. The",
            "assigned owner reads the requirement, implements, tests, repairs failures, and",
            "closes runnable work end to end. Add a row when another agent must act.",
            "",
        ]
    output = list(prefix)
    output.append(format_row(dict(zip(columns, columns, strict=True)), columns))
    output.append("|" + "|".join("---" for _ in columns) + "|")
    output.extend(format_row(row, columns) for row in rows)
    path.write_text("\n".join(output) + "\n", encoding="utf-8")


def open_prefix(lines: list[str]) -> list[str]:
    for index, line in enumerate(lines):
        if line.startswith("| id "):
            return lines[:index]
    return []


def read_metadata(path: Path) -> dict[str, str]:
    metadata: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("## "):
            break
        if line.startswith("- ") and ": " in line:
            key, value = line[2:].split(": ", 1)
            metadata[key.strip()] = value.strip()
    return metadata


def update_metadata(path: Path, updates: dict[str, str]) -> None:
    lines = path.read_text(encoding="utf-8").splitlines()
    out: list[str] = []
    seen: set[str] = set()
    in_meta = False
    inserted = False
    for line in lines:
        if line.startswith("## ") and not inserted:
            for key, value in updates.items():
                if key not in seen:
                    out.append(f"- {key}: {value}")
            inserted = True
            out.append(line)
            continue
        if not inserted and (line.startswith("- ") or in_meta):
            in_meta = True
            if line.startswith("- ") and ": " in line:
                key = line[2:].split(": ", 1)[0].strip()
                if key in updates:
                    out.append(f"- {key}: {updates[key]}")
                    seen.add(key)
                    continue
        out.append(line)
    if not inserted:
        for key, value in updates.items():
            if key not in seen:
                out.append(f"- {key}: {value}")
    path.write_text("\n".join(out) + "\n", encoding="utf-8")


def append_block(path: Path, heading: str, fields: dict[str, str], body: str = "") -> None:
    with path.open("a", encoding="utf-8") as handle:
        handle.write(f"{heading}\n\n")
        for key, value in fields.items():
            handle.write(f"- {key}: {value}\n")
        if body:
            handle.write(f"\n{body}\n")
        handle.write("\n")


def now_human() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M")


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def resolve_root() -> Path:
    root = os.environ.get("AGENT_COORD_ROOT", "")
    if root:
        root_path = Path(root)
        print(f"AGENT_COORD_ROOT is set: mailbox root -> {root_path}", file=sys.stderr)
        if (
            not (root_path / "docs/agents/README.md").is_file()
            or not (root_path / "src/luggage_gazebo").is_dir()
        ) and os.environ.get("AGENT_MAILBOX_ALLOW_ANY_ROOT") != "1":
            raise SystemExit("refusing: coordination root does not look like the primary workspace")
        return root_path
    result = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if result.returncode == 0:
        return Path(result.stdout.strip())
    return Path.cwd()


@contextmanager
def mailbox_lock(root: Path) -> Iterator[None]:
    lock_path = Path(os.environ.get("AGENT_MAILBOX_LOCK", "/tmp/elfin_humble_agents_open.lock"))
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open("w", encoding="utf-8") as handle:
        fcntl.flock(handle, fcntl.LOCK_EX)
        yield


def parse_generation(value: str | None, *, legacy_ok: bool) -> int:
    if not value:
        if legacy_ok:
            return 1
        raise SystemExit("missing generation")
    if not re.fullmatch(r"[1-9][0-9]*", value):
        raise SystemExit(f"invalid generation: {value}")
    return int(value)


def plan_revision(meta: dict[str, str], row: dict[str, str] | None, *, legacy_ok: bool) -> str:
    value = meta.get("plan_revision") or (row or {}).get("plan_revision", "")
    if value:
        return value
    if legacy_ok:
        return meta.get("revision") or (row or {}).get("revision", "")
    raise SystemExit("missing plan_revision")


def find_row(rows: list[Row], thread: str) -> dict[str, str] | None:
    for row in rows:
        if row.cells.get("thread") == thread:
            return row.cells
    return None


def thread_claimed(path: Path) -> bool:
    text = path.read_text(encoding="utf-8")
    return "## Claim -- " in text and "\n- started_at: " in text


def latest_event_fields(path: Path, prefix: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    active = False
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("## "):
            active = line.startswith(prefix)
            if active:
                fields = {}
            continue
        if active and line.startswith("- ") and ": " in line:
            key, value = line[2:].split(": ", 1)
            fields[key.strip()] = value.strip()
    return fields


def threads_for_lineage(threads_dir: Path, parent: str, subtask: str, *, legacy_ok: bool) -> list[tuple[Path, dict[str, str], int]]:
    matches: list[tuple[Path, dict[str, str], int]] = []
    for path in threads_dir.glob("*.md"):
        if path.name in {"OPEN.md", "README.md"}:
            continue
        meta = read_metadata(path)
        if meta.get("parent") != parent or meta.get("subtask") != subtask:
            continue
        if meta.get("kind") not in RUNNABLE_KINDS:
            continue
        generation = parse_generation(meta.get("generation"), legacy_ok=legacy_ok)
        matches.append((path, meta, generation))
    return matches


def highest_generation(threads_dir: Path, parent: str, subtask: str, *, legacy_ok: bool) -> tuple[Path, dict[str, str], int] | None:
    matches = threads_for_lineage(threads_dir, parent, subtask, legacy_ok=legacy_ok)
    if not matches:
        return None
    return max(matches, key=lambda item: item[2])


def result_passed(path: Path) -> bool:
    text = path.read_text(encoding="utf-8")
    return "## Result -- " in text and "\n- outcome: pass\n" in text


def dependency_passed(threads_dir: Path, parent: str, subtask: str, *, legacy_ok: bool) -> tuple[bool, int | None]:
    current = highest_generation(threads_dir, parent, subtask, legacy_ok=legacy_ok)
    if current is None:
        return False, None
    path, meta, generation = current
    return meta.get("status") == "done" and result_passed(path), generation


def dependency_snapshot(row: dict[str, str], threads_dir: Path, *, legacy_ok: bool) -> str:
    depends_on = row.get("depends_on", "none")
    if depends_on == "none":
        return "none"
    parts: list[str] = []
    for dependency in [item.strip() for item in depends_on.split(",") if item.strip()]:
        passed, generation = dependency_passed(
            threads_dir, row["parent"], dependency, legacy_ok=legacy_ok
        )
        if not passed or generation is None:
            raise SystemExit(f"dependency is not complete: {row['parent']}/{dependency}")
        parts.append(f"{dependency}={generation}")
    return ",".join(parts) if parts else "none"


def parse_dependency_snapshot(value: str) -> dict[str, int]:
    if not value or value == "none":
        return {}
    result: dict[str, int] = {}
    for item in value.split(","):
        if "=" not in item:
            raise SystemExit(f"invalid claimed_dependencies entry: {item}")
        subtask, generation = item.split("=", 1)
        result[subtask.strip()] = parse_generation(generation.strip(), legacy_ok=False)
    return result


def validate_thread_and_row(thread_path: Path, open_path: Path, role: str, agent: str, model: str) -> tuple[dict[str, str], dict[str, str], list[Row], list[str], list[str]]:
    columns, rows, lines = read_open(open_path)
    meta = read_metadata(thread_path)
    row = find_row(rows, thread_path.name)
    if row is None:
        raise SystemExit(f"thread has no open mailbox row: {thread_path.name}")
    if meta.get("status") not in {"open", "superseded", "cancelled"}:
        raise SystemExit(f"thread is not open: {meta.get('status', '')}")
    if meta.get("kind") not in RUNNABLE_KINDS:
        raise SystemExit(f"thread is not runnable owner work: {meta.get('kind', '')}")
    if meta.get("to_role") not in {role, "any"}:
        raise SystemExit(f"role mismatch: assigned to {meta.get('to_role', '')}")
    if meta.get("to_agent") != agent or meta.get("to_model") != model:
        raise SystemExit(f"owner mismatch: assigned to {meta.get('to_agent', '')}/{meta.get('to_model', '')}")
    return meta, row, rows, columns, lines


def ensure_open_and_current(thread_path: Path, row: dict[str, str], meta: dict[str, str], threads_dir: Path) -> tuple[int, str]:
    if meta.get("status") != "open":
        raise SystemExit(f"thread is terminal: {meta.get('status', '')}")
    dispatch_ready = meta.get("dispatch_ready", "")
    if dispatch_ready and dispatch_ready != "yes":
        raise SystemExit("thread is not dispatch_ready")
    has_new_meta = "generation" in meta or "plan_revision" in meta
    has_new_row = bool(row.get("generation") or row.get("plan_revision"))
    legacy_ok = not (has_new_meta or has_new_row)
    if not legacy_ok:
        if not meta.get("generation") or not meta.get("plan_revision"):
            raise SystemExit("thread missing generation or plan_revision")
        if not row.get("generation") or not row.get("plan_revision"):
            raise SystemExit("row missing generation or plan_revision")
    generation = parse_generation(meta.get("generation") or row.get("generation"), legacy_ok=legacy_ok)
    plan_rev = plan_revision(meta, row, legacy_ok=legacy_ok)
    if has_new_meta and has_new_row:
        if row.get("generation") != str(generation) or row.get("plan_revision") != plan_rev:
            raise SystemExit("row/thread generation or plan_revision mismatch")
    if not legacy_ok:
        current = highest_generation(
            threads_dir, meta["parent"], meta["subtask"], legacy_ok=False
        )
        if current is None or current[0].name != thread_path.name:
            raise SystemExit("thread is not highest generation for its lineage")
    return generation, plan_rev


def cmd_start(args: argparse.Namespace) -> int:
    root = resolve_root()
    os.chdir(root)
    thread_path = root / "docs/agents/discuss" / args.thread
    open_path = root / "docs/agents/discuss/OPEN.md"
    if "/" in args.thread or not args.thread.endswith(".md"):
        raise SystemExit("--thread must be a discuss filename")
    if not thread_path.is_file() or not open_path.is_file():
        raise SystemExit("missing thread or mailbox")
    with mailbox_lock(root):
        meta, row, _rows, _columns, _lines = validate_thread_and_row(
            thread_path, open_path, args.role, args.agent, args.model
        )
        generation, plan_rev = ensure_open_and_current(
            thread_path, row, meta, open_path.parent
        )
        if thread_claimed(thread_path):
            print(f"already started {args.thread}")
            return 0
        dependencies = dependency_snapshot(row, open_path.parent, legacy_ok=not bool(row.get("generation") or meta.get("generation")))
        append_block(
            thread_path,
            f"## Claim -- {args.role}/{args.agent} -- {now_human()} -- {args.cli}/{args.model}",
            {
                "started_at": now_iso(),
                "claimed_generation": str(generation),
                "claimed_plan_revision": plan_rev,
                "claimed_dependencies": dependencies,
            },
        )
    print(f"started {args.thread} at {now_iso()}")
    return 0


def resolve_commit(revision: str) -> str:
    result = subprocess.run(
        ["git", "rev-parse", "--verify", f"{revision}^{{commit}}"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if result.returncode != 0:
        raise SystemExit(f"revision does not resolve to a Git commit: {revision}")
    return result.stdout.strip()


def validate_completion_freshness(thread_path: Path, row: dict[str, str], meta: dict[str, str], threads_dir: Path) -> None:
    generation, plan_rev = ensure_open_and_current(thread_path, row, meta, threads_dir)
    claim = latest_event_fields(thread_path, "## Claim -- ")
    if not claim and latest_event_fields(thread_path, "## Rework Claim -- "):
        claim = latest_event_fields(thread_path, "## Rework Claim -- ")
    if claim.get("claimed_generation") and claim["claimed_generation"] != str(generation):
        raise SystemExit("claim generation is stale")
    if claim.get("claimed_plan_revision") and claim["claimed_plan_revision"] != plan_rev:
        raise SystemExit("claim plan_revision is stale")
    for subtask, claimed_generation in parse_dependency_snapshot(
        claim.get("claimed_dependencies", "none")
    ).items():
        current = highest_generation(threads_dir, meta["parent"], subtask, legacy_ok=True)
        if current is None:
            raise SystemExit(f"claimed dependency missing: {subtask}")
        dep_path, dep_meta, current_generation = current
        if current_generation != claimed_generation:
            raise SystemExit(f"claimed dependency generation is stale: {subtask}")
        if dep_meta.get("status") != "done" or not result_passed(dep_path):
            raise SystemExit(f"claimed dependency is not currently passing: {subtask}")


def remove_open_row(open_path: Path, thread: str) -> None:
    lines = open_path.read_text(encoding="utf-8").splitlines()
    out = [line for line in lines if not (line.startswith("|") and f"| {thread} |" in line)]
    open_path.write_text("\n".join(out) + "\n", encoding="utf-8")


def cmd_complete(args: argparse.Namespace) -> int:
    root = resolve_root()
    os.chdir(root)
    thread_path = root / "docs/agents/discuss" / args.thread
    open_path = root / "docs/agents/discuss/OPEN.md"
    if "/" in args.thread or not args.thread.endswith(".md"):
        raise SystemExit("--thread must be a discuss filename")
    if args.outcome == "pass" and (not args.revision or not args.tests):
        raise SystemExit("--outcome pass requires --revision and --tests")
    if not thread_path.is_file() or not open_path.is_file():
        raise SystemExit("missing thread or mailbox")
    with mailbox_lock(root):
        meta, row, _rows, _columns, _lines = validate_thread_and_row(
            thread_path, open_path, args.role, args.agent, args.model
        )
        if not thread_claimed(thread_path):
            if not args.started_at:
                raise SystemExit("thread has no Claim event; run agent_start.sh before work")
            append_block(
                thread_path,
                f"## Claim -- {args.role}/{args.agent} -- {now_human()} -- {args.cli}/{args.model}",
                {
                    "started_at": args.started_at,
                    "claimed_generation": str(parse_generation(meta.get("generation") or row.get("generation"), legacy_ok=True)),
                    "claimed_plan_revision": plan_revision(meta, row, legacy_ok=True),
                    "claimed_dependencies": dependency_snapshot(row, open_path.parent, legacy_ok=True),
                },
            )
        resolved_revision = args.revision
        if args.outcome == "pass":
            resolved_revision = resolve_commit(args.revision)
            validate_completion_freshness(thread_path, row, meta, open_path.parent)
        with thread_path.open("a", encoding="utf-8") as handle:
            handle.write(
                f"## Result -- {args.role}/{args.agent} -- {now_human()} -- {args.cli}/{args.model}\n\n"
            )
            handle.write(f"- outcome: {args.outcome}\n")
            handle.write(f"- completed_at: {now_iso()}\n")
            if resolved_revision:
                handle.write(f"- revision: {resolved_revision}\n")
            if args.tests:
                handle.write(f"- tests: {trim_cell(args.tests)}\n")
            handle.write(f"- summary: {trim_cell(args.summary)}\n")
            for pointer in args.evidence:
                handle.write(f"- evidence: {pointer}\n")
            handle.write("\n")
        if args.outcome == "pass":
            update_metadata(thread_path, {"status": "done"})
            remove_open_row(open_path, args.thread)
    if args.outcome == "pass":
        print(f"completed {args.thread} at {resolved_revision}")
    else:
        print(f"left {args.thread} open as blocked")
    return 0


def cmd_transition(args: argparse.Namespace) -> int:
    root = resolve_root()
    os.chdir(root)
    thread_path = root / "docs/agents/discuss" / args.thread
    open_path = root / "docs/agents/discuss/OPEN.md"
    with mailbox_lock(root):
        meta = read_metadata(thread_path)
        status = meta.get("status")
        if args.state == "superseded":
            if status not in {"open", "done"}:
                raise SystemExit(f"thread cannot be superseded from status: {status}")
        elif status != "open":
            raise SystemExit(f"thread is not open: {status}")
        if meta.get("kind") not in RUNNABLE_KINDS:
            raise SystemExit("thread is not runnable owner work")
        generation = parse_generation(meta.get("generation"), legacy_ok=False)
        replacement = args.replacement or "n/a"
        if args.state == "superseded":
            if replacement == "n/a":
                raise SystemExit("supersede requires --replacement")
            replacement_meta = read_metadata(open_path.parent / replacement)
            if replacement_meta.get("parent") != meta.get("parent") or replacement_meta.get("subtask") != meta.get("subtask"):
                raise SystemExit("replacement is not in the same lineage")
            replacement_generation = parse_generation(
                replacement_meta.get("generation"), legacy_ok=False
            )
            if replacement_generation <= generation:
                raise SystemExit("replacement generation must be higher")
        update_metadata(thread_path, {"status": args.state})
        append_block(
            thread_path,
            f"## {args.state.capitalize()} -- {args.role}/{args.agent} -- {now_human()} -- {args.cli}/{args.model}",
            {
                "transitioned_at": now_iso(),
                "old_generation": str(generation),
                "replacement": replacement,
                "reason": trim_cell(args.reason),
            },
        )
        if not thread_claimed(thread_path):
            remove_open_row(open_path, args.thread)
    print(f"{args.state} {args.thread}")
    return 0


def cmd_stop_ack(args: argparse.Namespace) -> int:
    root = resolve_root()
    os.chdir(root)
    thread_path = root / "docs/agents/discuss" / args.thread
    open_path = root / "docs/agents/discuss/OPEN.md"
    with mailbox_lock(root):
        meta = read_metadata(thread_path)
        if meta.get("status") not in {"superseded", "cancelled"}:
            raise SystemExit(f"thread is not stopped: {meta.get('status', '')}")
        append_block(
            thread_path,
            f"## StopAck -- {args.role}/{args.agent} -- {now_human()} -- {args.cli}/{args.model}",
            {"acknowledged_at": now_iso(), "reason": trim_cell(args.reason or "acknowledged")},
        )
        remove_open_row(open_path, args.thread)
    print(f"acknowledged stop for {args.thread}")
    return 0


def migrate_thread(path: Path, generation: str, plan_rev: str, dry_run: bool) -> bool:
    meta = read_metadata(path)
    if meta.get("kind") not in RUNNABLE_KINDS:
        return False
    updates: dict[str, str] = {}
    if not meta.get("generation"):
        updates["generation"] = generation
    if not meta.get("plan_revision"):
        updates["plan_revision"] = plan_rev
    if updates and not dry_run:
        update_metadata(path, updates)
    return bool(updates)


def cmd_migrate(args: argparse.Namespace) -> int:
    root = resolve_root()
    os.chdir(root)
    plan_rev = resolve_commit(args.plan_revision)
    open_path = root / "docs/agents/discuss/OPEN.md"
    threads_dir = open_path.parent
    with mailbox_lock(root):
        columns, open_rows, lines = read_open(open_path)
        rows = [dict(row.cells) for row in open_rows]
        lineages: dict[tuple[str, str, int], str] = {}
        for path in threads_dir.glob("*.md"):
            if path.name in {"OPEN.md", "README.md"}:
                continue
            meta = read_metadata(path)
            if meta.get("kind") not in RUNNABLE_KINDS:
                continue
            generation = parse_generation(meta.get("generation"), legacy_ok=True)
            key = (meta.get("parent", ""), meta.get("subtask", ""), generation)
            if key in lineages:
                raise SystemExit(f"ambiguous duplicate lineage: {key[0]}/{key[1]}/{key[2]}")
            lineages[key] = path.name
        changed = 0
        for path in threads_dir.glob("*.md"):
            if path.name in {"OPEN.md", "README.md"}:
                continue
            if migrate_thread(path, "1", plan_rev, args.dry_run):
                changed += 1
        new_columns = columns
        if "generation" not in new_columns:
            new_columns = columns + ["generation", "plan_revision"]
        for row in rows:
            if row.get("kind") in RUNNABLE_KINDS:
                row.setdefault("generation", "1")
                row.setdefault("plan_revision", plan_rev)
                if not row.get("generation"):
                    row["generation"] = "1"
                if not row.get("plan_revision"):
                    row["plan_revision"] = plan_rev
        if not args.dry_run:
            write_open(open_path, new_columns, rows, open_prefix(lines))
    action = "would migrate" if args.dry_run else "migrated"
    print(f"{action} {changed} runnable thread(s) with plan_revision {plan_rev}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)

    start = subparsers.add_parser("start")
    start.add_argument("--thread", required=True)
    start.add_argument("--role", required=True)
    start.add_argument("--agent", required=True)
    start.add_argument("--model", required=True)
    start.add_argument("--cli", required=True)
    start.set_defaults(func=cmd_start)

    complete = subparsers.add_parser("complete")
    complete.add_argument("--thread", required=True)
    complete.add_argument("--role", required=True)
    complete.add_argument("--agent", required=True)
    complete.add_argument("--model", required=True)
    complete.add_argument("--cli", required=True)
    complete.add_argument("--outcome", choices=["pass", "blocked"], required=True)
    complete.add_argument("--summary", required=True)
    complete.add_argument("--revision", default="")
    complete.add_argument("--tests", default="")
    complete.add_argument("--started-at", default="")
    complete.add_argument("--evidence", action="append", default=[])
    complete.set_defaults(func=cmd_complete)

    transition = subparsers.add_parser("transition")
    transition.add_argument("--thread", required=True)
    transition.add_argument("--state", choices=["superseded", "cancelled"], required=True)
    transition.add_argument("--replacement", default="")
    transition.add_argument("--reason", required=True)
    transition.add_argument("--role", required=True)
    transition.add_argument("--agent", required=True)
    transition.add_argument("--model", required=True)
    transition.add_argument("--cli", required=True)
    transition.set_defaults(func=cmd_transition)

    ack = subparsers.add_parser("stop-ack")
    ack.add_argument("--thread", required=True)
    ack.add_argument("--reason", default="")
    ack.add_argument("--role", required=True)
    ack.add_argument("--agent", required=True)
    ack.add_argument("--model", required=True)
    ack.add_argument("--cli", required=True)
    ack.set_defaults(func=cmd_stop_ack)

    migrate = subparsers.add_parser("migrate")
    migrate.add_argument("--plan-revision", required=True)
    migrate.add_argument("--dry-run", action="store_true")
    migrate.set_defaults(func=cmd_migrate)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
