#!/usr/bin/env python3
"""Generate idempotent daily and weekly progress summaries from repo facts."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from datetime import date, datetime, time as day_time, timedelta
from pathlib import Path
from typing import Any, Iterable


SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import agent_scheduler as sched  # noqa: E402


NOTE_NAME_RE = re.compile(
    r"^(?P<date>\d{4}-\d{2}-\d{2})_(?P<hhmm>\d{4})_.+\.md$"
)
RESULT_BLOCK_RE = re.compile(
    r"^## Result(?: -- (?P<header>[^\n]+))?\n\n(?P<body>.*?)(?=^## |\Z)",
    re.MULTILINE | re.DOTALL,
)
BLOCKER_RE = re.compile(
    r"\b(block(?:ed|er|ing)?|fail(?:ed|ure)?|waiting|missing|"
    r"inconclusive|not ready|remains open|cannot)\b|卡点|阻塞|缺失|失败",
    re.IGNORECASE,
)


@dataclass(frozen=True)
class Commit:
    revision: str
    authored_at: str
    subject: str


@dataclass(frozen=True)
class ResultEvent:
    thread: str
    parent: str
    subtask: str
    outcome: str
    completed_at: str
    revision: str
    summary: str
    tests: str


@dataclass(frozen=True)
class ReviewDecision:
    note: str
    recorded_at: str
    summary: str


@dataclass(frozen=True)
class WorkItem:
    mailbox_id: str
    parent: str
    subtask: str
    owner: str
    action: str
    reason: str
    request: str
    thread: str
    open_detail: str


@dataclass(frozen=True)
class PlanItem:
    parent: str
    subtask: str
    owner: str
    depends_on: str
    plan: str


@dataclass(frozen=True)
class ReadinessBlocker:
    parent: str
    subtask: str
    outcome: str
    summary: str
    thread: str


@dataclass(frozen=True)
class WorkspaceState:
    revision: str
    tracked_modified: int
    untracked: int


@dataclass(frozen=True)
class PollResult:
    report_date: str
    daily_path: str
    daily_changed: bool
    weekly_path: str | None
    weekly_changed: bool
    index_changed: bool
    source_digest: str


def local_timezone():
    return datetime.now().astimezone().tzinfo


def period_bounds(report_date: date, weekly: bool) -> tuple[datetime, datetime]:
    tz = local_timezone()
    if weekly:
        first = report_date - timedelta(days=report_date.weekday())
        last_exclusive = first + timedelta(days=7)
    else:
        first = report_date
        last_exclusive = report_date + timedelta(days=1)
    return (
        datetime.combine(first, day_time.min, tzinfo=tz),
        datetime.combine(last_exclusive, day_time.min, tzinfo=tz),
    )


def run_git(root: Path, *args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(root), *args],
        check=True,
        capture_output=True,
        text=True,
    ).stdout


def collect_commits(root: Path, start: datetime, end: datetime) -> list[Commit]:
    output = run_git(
        root,
        "log",
        "--format=%H%x1f%aI%x1f%s",
        f"--since={start.isoformat()}",
        f"--before={end.isoformat()}",
    )
    commits: list[Commit] = []
    for line in output.splitlines():
        parts = line.split("\x1f", 2)
        if len(parts) == 3:
            commits.append(Commit(*parts))
    return commits


def parse_metadata(text: str) -> dict[str, str]:
    metadata: dict[str, str] = {}
    for line in text.splitlines():
        if line.startswith("## "):
            break
        if line.startswith("- ") and ": " in line:
            key, value = line[2:].split(": ", 1)
            metadata[key.strip()] = value.strip()
    return metadata


def parse_key_values(block: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in block.splitlines():
        if not line.startswith("- ") or ": " not in line:
            continue
        key, value = line[2:].split(": ", 1)
        if key not in values:
            values[key.strip()] = value.strip()
    return values


def parse_timestamp(value: str) -> datetime | None:
    cleaned = value.strip().replace(" local", "")
    if not cleaned:
        return None
    try:
        parsed = datetime.fromisoformat(cleaned)
    except ValueError:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=local_timezone())
    return parsed


def timestamp_from_result_header(header: str) -> datetime | None:
    matches = re.findall(r"\d{4}-\d{2}-\d{2}[ T]\d{2}:\d{2}(?::\d{2})?", header)
    return parse_timestamp(matches[-1]) if matches else None


def normalize_text(value: str, limit: int = 700) -> str:
    normalized = " ".join(value.split())
    if len(normalized) <= limit:
        return normalized
    return f"{normalized[:limit].rstrip()} [truncated from {len(normalized)} chars]"


def collect_results(
    root: Path, start: datetime, end: datetime
) -> list[ResultEvent]:
    discuss = root / "docs" / "agents" / "discuss"
    events: list[ResultEvent] = []
    for path in sorted(discuss.glob("*.md")):
        if path.name in {"OPEN.md", "README.md"}:
            continue
        text = path.read_text(encoding="utf-8")
        metadata = parse_metadata(text)
        for match in RESULT_BLOCK_RE.finditer(text):
            fields = parse_key_values(match.group("body"))
            completed = parse_timestamp(fields.get("completed_at", ""))
            if completed is None:
                completed = timestamp_from_result_header(match.group("header") or "")
            if completed is None or not (start <= completed < end):
                continue
            events.append(
                ResultEvent(
                    thread=path.name,
                    parent=metadata.get("parent", "n/a"),
                    subtask=metadata.get("subtask", "n/a"),
                    outcome=fields.get("outcome", "unknown"),
                    completed_at=completed.isoformat(timespec="seconds"),
                    revision=fields.get("revision", "n/a"),
                    summary=normalize_text(fields.get("summary", "No summary recorded.")),
                    tests=normalize_text(fields.get("tests", "Not recorded.")),
                )
            )
    events.sort(key=lambda event: event.completed_at, reverse=True)
    return events


def result_blocks(path: Path) -> list[tuple[dict[str, str], dict[str, str]]]:
    text = path.read_text(encoding="utf-8")
    metadata = parse_metadata(text)
    return [
        (metadata, parse_key_values(match.group("body")))
        for match in RESULT_BLOCK_RE.finditer(text)
    ]


def completed_subtasks(root: Path) -> set[tuple[str, str]]:
    completed: set[tuple[str, str]] = set()
    discuss = root / "docs" / "agents" / "discuss"
    for path in discuss.glob("*.md"):
        for metadata, fields in result_blocks(path):
            if fields.get("outcome") == "pass":
                completed.add(
                    (metadata.get("parent", "n/a"), metadata.get("subtask", "n/a"))
                )
    return completed


def collect_readiness_blockers(root: Path) -> list[ReadinessBlocker]:
    blockers: list[ReadinessBlocker] = []
    discuss = root / "docs" / "agents" / "discuss"
    for path in sorted(discuss.glob("*.md")):
        blocks = result_blocks(path)
        if not blocks:
            continue
        metadata, fields = blocks[-1]
        outcome = fields.get("outcome", "unknown")
        summary = normalize_text(fields.get("summary", ""))
        readiness_blocked = bool(
            re.search(r"\b[a-z_]*readiness\s*=\s*blocked\b", summary, re.IGNORECASE)
        )
        if outcome != "blocked" and not readiness_blocked:
            continue
        blockers.append(
            ReadinessBlocker(
                parent=metadata.get("parent", "n/a"),
                subtask=metadata.get("subtask", "n/a"),
                outcome=outcome,
                summary=summary or "Latest Result remains blocked.",
                thread=path.name,
            )
        )
    blockers.sort(key=lambda item: (item.parent, item.subtask))
    return blockers


def section_text(text: str, heading: str) -> str:
    pattern = re.compile(
        rf"^{re.escape(heading)}\s*$\n(?P<body>.*?)(?=^## |\Z)",
        re.MULTILINE | re.DOTALL,
    )
    match = pattern.search(text)
    if match is None:
        return ""
    lines = []
    for line in match.group("body").strip().splitlines():
        stripped = line.strip()
        if not stripped:
            if lines:
                break
            continue
        lines.append(stripped.removeprefix("- "))
    return normalize_text(" ".join(lines))


def collect_review_decisions(
    root: Path, start: datetime, end: datetime
) -> list[ReviewDecision]:
    decisions: list[ReviewDecision] = []
    for path in sorted((root / "docs" / "agents" / "reviews").glob("*.md")):
        match = NOTE_NAME_RE.match(path.name)
        if match is None:
            continue
        recorded = parse_timestamp(
            f"{match.group('date')}T{match.group('hhmm')[:2]}:"
            f"{match.group('hhmm')[2:]}:00"
        )
        if recorded is None or not (start <= recorded < end):
            continue
        text = path.read_text(encoding="utf-8")
        metadata = parse_metadata(text)
        if metadata.get("status") != "done":
            continue
        summary = section_text(text, "## Summary")
        if not summary:
            continue
        decisions.append(
            ReviewDecision(path.name, recorded.isoformat(timespec="seconds"), summary)
        )
    decisions.sort(key=lambda item: item.recorded_at, reverse=True)
    return decisions


def latest_open_detail(text: str) -> str:
    matches = list(
        re.finditer(r"^## Open\s*$\n(?P<body>.*?)(?=^## |\Z)", text,
                    re.MULTILINE | re.DOTALL)
    )
    if not matches:
        return ""
    detail = normalize_text(matches[-1].group("body").strip().removeprefix("- "))
    return "" if detail.lower() in {"none", "none."} else detail


def collect_work_items(root: Path) -> list[WorkItem]:
    open_path = root / "docs" / "agents" / "discuss" / "OPEN.md"
    threads_dir = open_path.parent
    items: list[WorkItem] = []
    for row in sched.read_table(open_path, sched.OPEN_COLUMNS):
        thread_path = threads_dir / row["thread"]
        if not thread_path.exists():
            action, reason, detail = "blocked", "thread missing", ""
        else:
            thread_text = thread_path.read_text(encoding="utf-8")
            detail = latest_open_detail(thread_text)
            if sched.thread_claimed(thread_path):
                action, reason = "active", "claimed by assigned owner"
            else:
                ready, dependency_reason = sched.dependencies_ready(row, threads_dir)
                if ready:
                    action, reason = "ready", "dependencies passed"
                else:
                    action, reason = "waiting", dependency_reason
        items.append(
            WorkItem(
                mailbox_id=row["id"],
                parent=row["parent"],
                subtask=row["subtask"],
                owner=f"{row['to_role']}/{row['to_agent']}/{row['to_model']}",
                action=action,
                reason=reason,
                request=normalize_text(row["request"]),
                thread=row["thread"],
                open_detail=detail,
            )
        )
    order = {"active": 0, "ready": 1, "waiting": 2, "blocked": 3}
    items.sort(key=lambda item: (order.get(item.action, 9), item.mailbox_id))
    return items


def collect_future_plan_items(
    root: Path,
    relevant_parents: set[str],
    work: list[WorkItem],
) -> list[PlanItem]:
    completed = completed_subtasks(root)
    open_keys = {(item.parent, item.subtask) for item in work}
    items: dict[tuple[str, str], PlanItem] = {}
    parent_re = re.compile(r"^Parent task:\s*`([^`]+)`\s*$", re.MULTILINE)
    for path in sorted((root / "docs" / "plans").glob("*.md")):
        text = path.read_text(encoding="utf-8")
        parent_match = parent_re.search(text)
        if parent_match is None:
            continue
        parent = parent_match.group(1)
        if parent not in relevant_parents:
            continue
        for line in text.splitlines():
            if not line.startswith("|"):
                continue
            cells = sched.split_md_row(line)
            if len(cells) < 4:
                continue
            subtask = cells[0].strip()
            if subtask in {"ID", "---"} or set(subtask) == {"-"}:
                continue
            if not re.match(r"^[A-Z][A-Z0-9-]+$", subtask):
                continue
            key = (parent, subtask)
            if key in completed or key in open_keys:
                continue
            items[key] = PlanItem(
                parent=parent,
                subtask=subtask,
                owner=normalize_text(cells[1].strip("`")),
                depends_on=normalize_text(cells[2].strip("`")),
                plan=path.name,
            )
    return sorted(items.values(), key=lambda item: (item.parent, item.subtask))


def collect_workspace_state(root: Path, output_root: Path) -> WorkspaceState:
    revision = run_git(root, "rev-parse", "HEAD").strip()
    status = run_git(root, "status", "--porcelain", "--untracked-files=all")
    output_rel = output_root.resolve().relative_to(root.resolve()).as_posix() + "/"
    tracked = 0
    untracked = 0
    for line in status.splitlines():
        if not line:
            continue
        rel = line[3:].split(" -> ")[-1]
        if rel.startswith(output_rel):
            continue
        if line.startswith("??"):
            untracked += 1
        else:
            tracked += 1
    return WorkspaceState(revision, tracked, untracked)


def source_digest(payload: dict[str, Any]) -> str:
    canonical = json.dumps(payload, sort_keys=True, ensure_ascii=True)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]


def md_escape(value: str) -> str:
    return value.replace("|", "\\|")


def bullet_or_none(items: Iterable[str]) -> list[str]:
    values = list(items)
    return values if values else ["- None recorded."]


def render_report(
    *,
    report_date: date,
    weekly: bool,
    start: datetime,
    end: datetime,
    commits: list[Commit],
    results: list[ResultEvent],
    decisions: list[ReviewDecision],
    work: list[WorkItem],
    planned: list[PlanItem],
    readiness_blockers: list[ReadinessBlocker],
    workspace: WorkspaceState,
    digest: str,
) -> str:
    if weekly:
        iso = report_date.isocalendar()
        title = f"Week {iso.year}-W{iso.week:02d} Progress"
    else:
        title = f"{report_date.isoformat()} Daily Progress"

    active = [item for item in work if item.action == "active"]
    ready = [item for item in work if item.action == "ready"]
    waiting = [item for item in work if item.action in {"waiting", "blocked"}]
    blockers = list(waiting)
    for item in active + ready:
        if item.open_detail and BLOCKER_RE.search(item.open_detail):
            blockers.append(item)

    lines = [
        f"# {title}",
        "",
        f"- Window: `{start.isoformat()}` to `{end.isoformat()}`",
        f"- Snapshot revision: `{workspace.revision}`",
        f"- Source digest: `{digest}`",
        "- Generated by: `scripts/progress_poll.py`",
        "",
        "## Summary",
        "",
        f"- Commits: {len(commits)}",
        f"- Passing result events: {sum(item.outcome == 'pass' for item in results)}",
        f"- Failed/blocked result events: "
        f"{sum(item.outcome != 'pass' for item in results)}",
        f"- Review decisions: {len(decisions)}",
        f"- Current work: {len(active)} active, {len(ready)} ready, "
        f"{len(waiting)} waiting/blocked",
        "",
        "## Completed Work",
        "",
    ]
    result_lines = []
    for event in (item for item in results if item.outcome == "pass"):
        result_lines.append(
            f"- `{event.subtask}` ({event.outcome}, `{event.revision}`): "
            f"{md_escape(event.summary)} "
            f"([thread](../../../agents/discuss/{event.thread}))"
        )
        result_lines.append(f"  Tests: {md_escape(event.tests)}")
    lines.extend(bullet_or_none(result_lines))

    lines.extend(["", "## Failed Or Blocked Results", ""])
    failed_lines = []
    for event in (item for item in results if item.outcome != "pass"):
        failed_lines.append(
            f"- `{event.subtask}` ({event.outcome}, `{event.revision}`): "
            f"{md_escape(event.summary)} "
            f"([thread](../../../agents/discuss/{event.thread}))"
        )
    lines.extend(bullet_or_none(failed_lines))

    lines.extend(["", "## Commits", ""])
    commit_lines = [
        f"- `{item.revision[:8]}` {md_escape(item.subject)} "
        f"({item.authored_at})"
        for item in commits
    ]
    lines.extend(bullet_or_none(commit_lines))

    lines.extend(["", "## Review Decisions", ""])
    decision_lines = [
        f"- {md_escape(item.summary)} "
        f"([note](../../../agents/reviews/{item.note}))"
        for item in decisions
    ]
    lines.extend(bullet_or_none(decision_lines))

    lines.extend(["", "## Current Execution", ""])
    active_lines = [
        f"- `{item.mailbox_id}` `{item.subtask}` -> `{item.owner}`: "
        f"{md_escape(item.request)} "
        f"([thread](../../../agents/discuss/{item.thread}))"
        for item in active
    ]
    lines.extend(bullet_or_none(active_lines))

    lines.extend(["", "## Future Plan", ""])
    future_lines = []
    for item in ready:
        future_lines.append(
            f"- Start `{item.subtask}` (`{item.owner}`): {md_escape(item.request)}"
        )
    for item in waiting:
        future_lines.append(
            f"- After {md_escape(item.reason)}, continue `{item.subtask}` "
            f"(`{item.owner}`)."
        )
    for item in planned:
        future_lines.append(
            f"- Planned, not dispatched: `{item.subtask}` (`{item.owner}`), "
            f"depends on `{md_escape(item.depends_on)}` "
            f"([plan](../../../plans/{item.plan}))."
        )
    lines.extend(bullet_or_none(future_lines))

    lines.extend(["", "## Current Blockers", ""])
    blocker_lines = []
    seen: set[str] = set()
    for item in blockers:
        if item.subtask in seen:
            continue
        seen.add(item.subtask)
        detail = item.open_detail or item.reason
        blocker_lines.append(
            f"- `{item.subtask}`: {md_escape(detail)} "
            f"([thread](../../../agents/discuss/{item.thread}))"
        )
    for item in readiness_blockers:
        if item.subtask in seen:
            continue
        seen.add(item.subtask)
        blocker_lines.append(
            f"- `{item.subtask}` readiness: {md_escape(item.summary)} "
            f"([thread](../../../agents/discuss/{item.thread}))"
        )
    lines.extend(bullet_or_none(blocker_lines))

    lines.extend(
        [
            "",
            "## Workspace",
            "",
            f"- Tracked modified files outside progress output: "
            f"{workspace.tracked_modified}",
            f"- Untracked files outside progress output: {workspace.untracked}",
            "- Dirty files are reported as context only; they are not counted "
            "as completed work without a Result or commit.",
            "",
        ]
    )
    return "\n".join(lines)


def write_if_changed(path: Path, content: str, dry_run: bool) -> bool:
    previous = path.read_text(encoding="utf-8") if path.exists() else None
    if previous == content:
        return False
    if dry_run:
        return True
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(content, encoding="utf-8")
    temporary.replace(path)
    return True


def render_index(output_root: Path, pending: Iterable[Path] = ()) -> str:
    daily = set((output_root / "daily").glob("*.md"))
    weekly = set((output_root / "weekly").glob("*.md"))
    for path in pending:
        if path.parent.name == "daily":
            daily.add(path)
        elif path.parent.name == "weekly":
            weekly.add(path)
    lines = [
        "# Project Progress",
        "",
        "Generated by `scripts/progress_poll.py`. Files are idempotent snapshots",
        "of repository commits, agent Results, review decisions, and the open mailbox.",
        "",
        "## Daily",
        "",
    ]
    lines.extend(
        [f"- [{path.stem}](daily/{path.name})" for path in sorted(daily, reverse=True)]
        or ["- None yet."]
    )
    lines.extend(["", "## Weekly", ""])
    lines.extend(
        [f"- [{path.stem}](weekly/{path.name})" for path in sorted(weekly, reverse=True)]
        or ["- None yet."]
    )
    lines.append("")
    return "\n".join(lines)


def collect_payload(root: Path, output_root: Path, start: datetime, end: datetime):
    commits = collect_commits(root, start, end)
    results = collect_results(root, start, end)
    decisions = collect_review_decisions(root, start, end)
    work = collect_work_items(root)
    relevant_parents = {item.parent for item in work} | {item.parent for item in results}
    planned = collect_future_plan_items(root, relevant_parents, work)
    readiness_blockers = collect_readiness_blockers(root)
    workspace = collect_workspace_state(root, output_root)
    payload = {
        "commits": [asdict(item) for item in commits],
        "results": [asdict(item) for item in results],
        "decisions": [asdict(item) for item in decisions],
        "work": [asdict(item) for item in work],
        "planned": [asdict(item) for item in planned],
        "readiness_blockers": [asdict(item) for item in readiness_blockers],
        "workspace": asdict(workspace),
    }
    return (
        commits,
        results,
        decisions,
        work,
        planned,
        readiness_blockers,
        workspace,
        source_digest(payload),
    )


def poll_once(
    root: Path,
    output_root: Path,
    report_date: date,
    force_weekly: bool = False,
    dry_run: bool = False,
) -> PollResult:
    daily_start, daily_end = period_bounds(report_date, weekly=False)
    daily_data = collect_payload(root, output_root, daily_start, daily_end)
    daily_digest = daily_data[-1]
    daily_path = output_root / "daily" / f"{report_date.isoformat()}.md"
    daily_text = render_report(
        report_date=report_date,
        weekly=False,
        start=daily_start,
        end=daily_end,
        commits=daily_data[0],
        results=daily_data[1],
        decisions=daily_data[2],
        work=daily_data[3],
        planned=daily_data[4],
        readiness_blockers=daily_data[5],
        workspace=daily_data[6],
        digest=daily_digest,
    )
    daily_changed = write_if_changed(daily_path, daily_text, dry_run)

    weekly_path: Path | None = None
    weekly_changed = False
    if report_date.weekday() == 6 or force_weekly:
        weekly_start, weekly_end = period_bounds(report_date, weekly=True)
        weekly_data = collect_payload(root, output_root, weekly_start, weekly_end)
        iso = report_date.isocalendar()
        weekly_path = output_root / "weekly" / f"{iso.year}-W{iso.week:02d}.md"
        weekly_text = render_report(
            report_date=report_date,
            weekly=True,
            start=weekly_start,
            end=weekly_end,
            commits=weekly_data[0],
            results=weekly_data[1],
            decisions=weekly_data[2],
            work=weekly_data[3],
            planned=weekly_data[4],
            readiness_blockers=weekly_data[5],
            workspace=weekly_data[6],
            digest=weekly_data[-1],
        )
        weekly_changed = write_if_changed(weekly_path, weekly_text, dry_run)

    pending = [daily_path] + ([weekly_path] if weekly_path is not None else [])
    index_text = render_index(output_root, pending)
    index_changed = write_if_changed(output_root / "README.md", index_text, dry_run)
    return PollResult(
        report_date=report_date.isoformat(),
        daily_path=str(daily_path.relative_to(root)),
        daily_changed=daily_changed,
        weekly_path=(str(weekly_path.relative_to(root)) if weekly_path else None),
        weekly_changed=weekly_changed,
        index_changed=index_changed,
        source_digest=daily_digest,
    )


def parse_args() -> argparse.Namespace:
    default_root = SCRIPTS_DIR.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=default_root)
    parser.add_argument(
        "--output-root",
        type=Path,
        help="Defaults to <root>/docs/status/progress.",
    )
    parser.add_argument("--date", help="Local report date in YYYY-MM-DD.")
    parser.add_argument("--force-weekly", action="store_true")
    parser.add_argument("--watch", action="store_true")
    parser.add_argument("--interval", type=float, default=900.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--json", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = args.root.resolve()
    output_root = (
        args.output_root.resolve()
        if args.output_root
        else root / "docs" / "status" / "progress"
    )
    if args.watch and args.date:
        raise SystemExit("--date cannot be combined with --watch")
    if args.interval < 1.0:
        raise SystemExit("--interval must be at least 1 second")

    last_result: PollResult | None = None
    while True:
        report_date = date.fromisoformat(args.date) if args.date else date.today()
        try:
            result = poll_once(
                root,
                output_root,
                report_date,
                force_weekly=args.force_weekly,
                dry_run=args.dry_run,
            )
            if last_result != result or not args.watch:
                if args.json:
                    print(json.dumps(asdict(result), indent=2, sort_keys=True))
                else:
                    changed = []
                    if result.daily_changed:
                        changed.append(result.daily_path)
                    if result.weekly_changed and result.weekly_path:
                        changed.append(result.weekly_path)
                    if result.index_changed:
                        changed.append(str((output_root / "README.md").relative_to(root)))
                    print("progress updated: " + (", ".join(changed) or "no changes"))
            last_result = result
        except (OSError, subprocess.SubprocessError, ValueError) as exc:
            if not args.watch:
                raise
            print(f"progress poll failed: {exc}", file=sys.stderr)
        if not args.watch:
            return 0
        try:
            time.sleep(args.interval)
        except KeyboardInterrupt:
            return 0


if __name__ == "__main__":
    raise SystemExit(main())
