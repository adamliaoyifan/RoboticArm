#!/usr/bin/env python3
"""Poll completed eng notes (and known source deliverables), run focused
gates, and persist one test note plus evidence directory per item.

Usage:
  scripts/poll_eng_completed.py              # one pass
  scripts/poll_eng_completed.py --interval 60
  scripts/poll_eng_completed.py --dry-run
  scripts/poll_eng_completed.py --force      # ignore skip cache

Does not launch Gazebo. Gate 4/5/6 stay on explicit checkpoint rows.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

ROOT = Path(__file__).resolve().parents[1]
GATES_FILE = ROOT / "scripts" / "eng_checkpoint_gates.json"
ENG_DIR = ROOT / "docs" / "agents" / "eng"
TEST_DIR = ROOT / "docs" / "agents" / "test"
OPEN_FILE = ROOT / "docs" / "agents" / "discuss" / "OPEN.md"
EVIDENCE_ROOT = ROOT / "docs" / "status" / "evidence" / "eng_poll"
STATE_FILE = EVIDENCE_ROOT / "state.json"
META_RE = re.compile(r"^- ([A-Za-z0-9_]+): (.+)$")
HEADING_RE = re.compile(r"^## ")
CHANGED_RE = re.compile(r"^- `([^`]+)`")
RANGE_CKPT_RE = re.compile(r"^[A-Za-z]*\d+-[A-Za-z]*\d+$")

AGENT = os.environ.get("ELFIN_TEST_AGENT", "cursor")
MODEL = os.environ.get("ELFIN_TEST_MODEL", "grok-4.6")
CLI = os.environ.get("ELFIN_TEST_CLI", "cursor")


@dataclass
class WorkItem:
    source: str
    source_kind: str
    checkpoint: str
    revision: str
    changed: List[str]
    slug: str
    mailbox_thread: Optional[str] = None
    mailbox_id: Optional[str] = None
    fingerprint: str = ""


def load_gates() -> dict:
    return json.loads(GATES_FILE.read_text())


def git_revision() -> str:
    try:
        out = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=ROOT, text=True, stderr=subprocess.DEVNULL)
        return out.strip() or "unknown"
    except subprocess.CalledProcessError:
        return "unknown"


def now_stamp() -> Tuple[str, str]:
    dt = datetime.now()
    return dt.strftime("%Y-%m-%d"), dt.strftime("%H%M")


def human_time() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M")


def parse_front_matter(text: str) -> Dict[str, str]:
    meta: Dict[str, str] = {}
    for line in text.splitlines():
        if HEADING_RE.match(line):
            break
        match = META_RE.match(line.strip())
        if match:
            meta[match.group(1)] = match.group(2).strip()
    return meta


def section_items(text: str, heading: str) -> List[str]:
    lines = text.splitlines()
    collecting = False
    items: List[str] = []
    for line in lines:
        if HEADING_RE.match(line):
            collecting = line.strip() == heading
            continue
        if collecting:
            match = CHANGED_RE.match(line.strip())
            if match:
                items.append(match.group(1))
    return items


def file_digest(paths: Sequence[str]) -> str:
    hasher = hashlib.sha256()
    for rel in sorted(paths):
        path = ROOT / rel
        hasher.update(rel.encode())
        if path.is_file():
            hasher.update(path.read_bytes())
        else:
            hasher.update(b"missing")
    return hasher.hexdigest()[:16]


def infer_checkpoints(changed: Sequence[str], prefixes: Sequence[dict]) -> List[str]:
    hits: List[str] = []
    for rel in changed:
        for row in prefixes:
            prefix = row["prefix"]
            if rel == prefix or rel.startswith(prefix):
                ckpt = row["checkpoint"]
                if ckpt not in hits:
                    hits.append(ckpt)
    return hits


def parse_eng_notes(cfg: dict, revision: str) -> List[WorkItem]:
    items: List[WorkItem] = []
    prefixes = cfg["path_prefixes"]
    for path in sorted(ENG_DIR.glob("*.md")):
        if path.name == "README.md":
            continue
        text = path.read_text()
        meta = parse_front_matter(text)
        if meta.get("role") != "eng":
            continue
        if meta.get("status") != "done":
            continue
        changed = section_items(text, "## Changed")
        if not changed:
            changed = section_items(text, "## Pointers")
        declared = meta.get("checkpoint", "n/a")
        note_rev = meta.get("revision", revision)
        if note_rev in ("", "n/a"):
            note_rev = revision
        checkpoints: List[str] = []
        if declared not in ("", "n/a") and not RANGE_CKPT_RE.match(declared):
            checkpoints = [declared]
        else:
            checkpoints = infer_checkpoints(changed, prefixes)
            if not checkpoints:
                checkpoints = ["GIT"]
        slug_base = path.stem.split("_", 2)[-1]
        for ckpt in checkpoints:
            items.append(WorkItem(
                source=str(path.relative_to(ROOT)),
                source_kind="eng-note",
                checkpoint=ckpt,
                revision=note_rev,
                changed=changed,
                slug=slug_base,
            ))
    return items


def parse_mailbox(revision: str) -> List[WorkItem]:
    items: List[WorkItem] = []
    if not OPEN_FILE.is_file():
        return items
    for line in OPEN_FILE.read_text().splitlines():
        if not line.startswith("| Q-"):
            continue
        cells = [c.strip() for c in line.strip("|").split("|")]
        if len(cells) < 13:
            continue
        (qid, kind, checkpoint, rev, to_role, _to_agent, _to_model,
         _from_role, _from_agent, _from_model, _cli, thread, _question) = cells[:13]
        if kind != "checkpoint":
            continue
        if to_role not in ("test", "any"):
            continue
        if checkpoint in ("", "n/a") or RANGE_CKPT_RE.match(checkpoint):
            continue
        items.append(WorkItem(
            source=qid,
            source_kind="mailbox",
            checkpoint=checkpoint,
            revision=rev if rev not in ("", "n/a") else revision,
            changed=[],
            slug=Path(thread).stem.split("_", 2)[-1] if thread else checkpoint.lower(),
            mailbox_thread=thread,
            mailbox_id=qid,
        ))
    return items


def parse_code_scan(cfg: dict, revision: str) -> List[WorkItem]:
    items: List[WorkItem] = []
    for row in cfg.get("code_scan", []):
        files = list(row["files"])
        if not all((ROOT / rel).is_file() for rel in files):
            continue
        ckpt = row["checkpoint"]
        items.append(WorkItem(
            source="code-scan:" + ",".join(files),
            source_kind="code",
            checkpoint=ckpt,
            revision=revision,
            changed=files,
            slug="code-" + ckpt.lower(),
        ))
    return items


def fingerprint(item: WorkItem) -> str:
    payload = "|".join([
        item.source, item.checkpoint, item.revision,
        file_digest(item.changed) if item.changed else "no-files",
    ])
    return hashlib.sha256(payload.encode()).hexdigest()[:20]


def load_state() -> dict:
    if STATE_FILE.is_file():
        return json.loads(STATE_FILE.read_text())
    return {"items": {}}


def save_state(state: dict) -> None:
    EVIDENCE_ROOT.mkdir(parents=True, exist_ok=True)
    STATE_FILE.write_text(json.dumps(state, indent=2) + "\n")


def unique_note_path(date: str, hhmm: str, slug: str) -> Path:
    base = TEST_DIR / f"{date}_{hhmm}_{slug}.md"
    if not base.exists():
        return base
    for idx in range(2, 50):
        candidate = TEST_DIR / f"{date}_{hhmm}_{slug}-{idx}.md"
        if not candidate.exists():
            return candidate
    raise RuntimeError("could not allocate test note filename")


def run_argv(argv: Sequence[str], env_extra: Optional[dict], log_dir: Path,
             timeout: int) -> dict:
    env = os.environ.copy()
    env.pop("AGENT_COORD_ROOT", None)
    if env_extra:
        for key, value in env_extra.items():
            if key == "PYTHONPATH":
                env[key] = str(ROOT / value) if not os.path.isabs(value) else value
            else:
                env[key] = value
    stdout_path = log_dir / "stdout.log"
    stderr_path = log_dir / "stderr.log"
    (log_dir / "command.txt").write_text(" ".join(argv) + "\n")
    try:
        proc = subprocess.run(
            list(argv), cwd=ROOT, env=env, text=True,
            capture_output=True, timeout=timeout)
        stdout_path.write_text(proc.stdout)
        stderr_path.write_text(proc.stderr)
        return {
            "argv": list(argv),
            "returncode": proc.returncode,
            "passed": proc.returncode == 0,
        }
    except subprocess.TimeoutExpired as exc:
        stdout_path.write_text(exc.stdout or "")
        stderr_path.write_text((exc.stderr or "") + "\nTIMEOUT\n")
        return {"argv": list(argv), "returncode": 124, "passed": False}


def builtin_wf1() -> Tuple[bool, str]:
    proc = subprocess.run(
        [str(ROOT / "scripts" / "check_agent_contract.sh")],
        cwd=ROOT, text=True, capture_output=True)
    ok = proc.returncode == 0
    return ok, (proc.stdout + proc.stderr).strip()


def builtin_git() -> Tuple[bool, str]:
    inside = subprocess.run(
        ["git", "rev-parse", "--is-inside-work-tree"],
        cwd=ROOT, text=True, capture_output=True)
    gitignore = (ROOT / ".gitignore").is_file()
    ok = inside.returncode == 0 and gitignore
    return ok, f"inside={inside.stdout.strip()} gitignore={gitignore}"


def builtin_e0() -> Tuple[bool, str]:
    luggage = (ROOT / "src/luggage_msgs/msg/DetectedLuggage.msg").read_text()
    frame = (ROOT / "src/luggage_msgs/msg/DetectionFrame.msg").read_text()
    missing = []
    for token in (
        "top_surface_pose", "top_surface_valid", "height_valid",
        "HEIGHT_SOURCE_UNAVAILABLE", "HEIGHT_SOURCE_MEASURED_SUPPORT",
        "HEIGHT_SOURCE_CONFIGURED_SUPPORT", "HEIGHT_SOURCE_CATALOG_PRIOR",
        "std_msgs/Header header",
    ):
        if token not in luggage:
            missing.append("DetectedLuggage:" + token)
    for token in (
        "support_valid", "support_reason", "support_z",
        "GEOMETRY_TOP_ONLY", "GEOMETRY_FULL_3D", "geometry_level",
    ):
        if token not in frame:
            missing.append("DetectionFrame:" + token)
    if "float64 height" in luggage and "height_valid" not in luggage:
        missing.append("numeric height without height_valid")
    ok = not missing
    return ok, "ok" if ok else "missing " + ", ".join(missing)


def builtin_e2() -> Tuple[bool, str]:
    env = os.environ.copy()
    env["PYTHONPATH"] = str(ROOT / "src/luggage_perception")
    proc = subprocess.run(
        [sys.executable, "-c",
         "from luggage_perception.platform_free_pipeline import ("
         "PlatformFreeDetector, SUPPORT_MODES); "
         "assert 'auto' in SUPPORT_MODES; "
         "PlatformFreeDetector()"],
        cwd=ROOT, env=env, text=True, capture_output=True)
    ok = proc.returncode == 0
    return ok, (proc.stdout + proc.stderr).strip() or "import ok"


BUILTINS = {
    "wf1_contract": builtin_wf1,
    "git_workspace": builtin_git,
    "e0_interface": builtin_e0,
    "e2_pipeline_smoke": builtin_e2,
}


def run_gate(checkpoint: str, cfg: dict, log_dir: Path, timeout: int) -> dict:
    spec = cfg["gates"].get(checkpoint)
    if spec is None:
        return {
            "checkpoint": checkpoint,
            "passed": False,
            "result": "inconclusive",
            "reason": "no gate mapping for " + checkpoint,
            "commands": [],
        }
    log_dir.mkdir(parents=True, exist_ok=True)
    if "builtin" in spec:
        ok, detail = BUILTINS[spec["builtin"]]()
        (log_dir / "stdout.log").write_text(detail + "\n")
        (log_dir / "command.txt").write_text("builtin:" + spec["builtin"] + "\n")
        return {
            "checkpoint": checkpoint,
            "passed": ok,
            "result": "pass" if ok else "fail",
            "reason": detail.splitlines()[-1] if detail else ("pass" if ok else "fail"),
            "commands": ["builtin:" + spec["builtin"]],
        }
    ran = run_argv(spec["argv"], spec.get("env"), log_dir, timeout)
    return {
        "checkpoint": checkpoint,
        "passed": ran["passed"],
        "result": "pass" if ran["passed"] else "fail",
        "reason": "exit %s" % ran["returncode"],
        "commands": [" ".join(spec["argv"])],
        "returncode": ran["returncode"],
    }


def write_test_note(item: WorkItem, outcome: dict, evidence_rel: str,
                    note_path: Path, date: str) -> None:
    status = "done" if outcome["result"] != "inconclusive" else "open"
    commands = outcome.get("commands") or ["(none)"]
    cmd_lines = "\n".join("- `%s`" % c for c in commands)
    body = f"""# {date} — {item.checkpoint} {item.slug}

- role: test
- agent: {AGENT}
- model: {MODEL}
- cli: {CLI}
- status: {status}
- checkpoint: {item.checkpoint}
- revision: {item.revision}

## Summary

Polled completed eng work `{item.source}` ({item.source_kind}) at revision
`{item.revision}`. Focused gate `{item.checkpoint}` result: {outcome['result']}
({outcome['reason']}).

## Commands

{cmd_lines}

## Evidence

- `{evidence_rel}`

## Result

- {outcome['result']}: {outcome['reason']}

## Pointers

- `{item.source}`
- `scripts/poll_eng_completed.py`
- `scripts/eng_checkpoint_gates.json`
"""
    note_path.write_text(body)


def notify_failure(item: WorkItem, outcome: dict, note_rel: str,
                   evidence_rel: str) -> None:
    notify = ROOT / "scripts" / "agent_notify.sh"
    question = (
        "Checkpoint %s failed at revision %s: %s"
        % (item.checkpoint, item.revision, outcome["reason"]))
    argv = [
        str(notify),
        "--kind", "regression",
        "--checkpoint", item.checkpoint,
        "--revision", item.revision,
        "--to", "eng",
        "--to-agent", "any",
        "--to-model", "any",
        "--from", "test",
        "--from-agent", AGENT,
        "--from-model", MODEL,
        "--cli", CLI,
        "--slug", "%s-regression" % item.checkpoint.lower(),
        "--question", question,
        "--pointer", note_rel,
        "--pointer", evidence_rel,
    ]
    if item.mailbox_thread:
        argv.extend(["--thread", item.mailbox_thread])
    env = os.environ.copy()
    env.pop("AGENT_COORD_ROOT", None)
    subprocess.run(argv, cwd=ROOT, env=env, check=False)


def process_item(item: WorkItem, cfg: dict, state: dict, args: argparse.Namespace,
                 date: str, hhmm: str, run_id: str,
                 gate_cache: Dict[str, dict]) -> Optional[dict]:
    item.fingerprint = fingerprint(item)
    previous = state["items"].get(item.fingerprint)
    if previous and not args.force and previous.get("result") in ("pass", "fail"):
        return None
    slug = "poll-%s-%s" % (item.checkpoint.lower(), item.slug)
    slug = re.sub(r"[^a-z0-9._-]+", "-", slug.lower()).strip("-")
    note_path = unique_note_path(date, hhmm, slug)
    evidence_dir = EVIDENCE_ROOT / run_id / note_path.stem
    evidence_dir.mkdir(parents=True, exist_ok=True)
    cache_key = item.checkpoint + "@" + item.revision
    if cache_key in gate_cache:
        outcome = dict(gate_cache[cache_key])
        (evidence_dir / "stdout.log").write_text(
            "reused gate result for %s\n" % cache_key)
        (evidence_dir / "command.txt").write_text(
            "reused:" + "; ".join(outcome.get("commands") or []) + "\n")
    else:
        outcome = run_gate(item.checkpoint, cfg, evidence_dir, args.timeout)
        gate_cache[cache_key] = dict(outcome)
    summary = {
        "source": item.source,
        "source_kind": item.source_kind,
        "checkpoint": item.checkpoint,
        "revision": item.revision,
        "fingerprint": item.fingerprint,
        "mailbox_id": item.mailbox_id,
        **outcome,
        "test_note": str(note_path.relative_to(ROOT)),
        "evidence": str(evidence_dir.relative_to(ROOT)),
        "time": human_time(),
    }
    (evidence_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    write_test_note(item, outcome, summary["evidence"], note_path, date)
    state["items"][item.fingerprint] = {
        "result": outcome["result"],
        "test_note": summary["test_note"],
        "evidence": summary["evidence"],
        "time": summary["time"],
        "checkpoint": item.checkpoint,
        "revision": item.revision,
        "source": item.source,
    }
    if outcome["result"] == "fail" and args.notify_on_fail:
        notify_failure(item, outcome, summary["test_note"], summary["evidence"])
    return summary


def collect_items(cfg: dict, revision: str, args: argparse.Namespace) -> List[WorkItem]:
    items: List[WorkItem] = []
    items.extend(parse_eng_notes(cfg, revision))
    items.extend(parse_mailbox(revision))
    if args.scan_code:
        items.extend(parse_code_scan(cfg, revision))
    # One row per (source, checkpoint, revision).
    unique: Dict[Tuple[str, str, str], WorkItem] = {}
    for item in items:
        unique[(item.source, item.checkpoint, item.revision)] = item
    return list(unique.values())


def one_pass(args: argparse.Namespace) -> int:
    cfg = load_gates()
    revision = git_revision()
    date, hhmm = now_stamp()
    run_id = "%s_%s" % (date, hhmm)
    items = collect_items(cfg, revision, args)
    if args.dry_run:
        for item in items:
            print("%s\t%s\t%s\t%s" % (
                item.source_kind, item.checkpoint, item.revision, item.source))
        return 0
    state = load_state()
    ran: List[dict] = []
    skipped = 0
    gate_cache: Dict[str, dict] = {}
    for item in items:
        summary = process_item(
            item, cfg, state, args, date, hhmm, run_id, gate_cache)
        if summary is None:
            skipped += 1
            continue
        ran.append(summary)
        print("%s %s %s -> %s (%s)" % (
            item.checkpoint, item.source_kind, item.source,
            summary["result"], summary["test_note"]))
    save_state(state)
    tick_path = EVIDENCE_ROOT / run_id / "poll.json"
    tick_path.parent.mkdir(parents=True, exist_ok=True)
    tick_path.write_text(json.dumps({
        "time": human_time(),
        "revision": revision,
        "discovered": len(items),
        "ran": len(ran),
        "skipped": skipped,
        "results": ran,
    }, indent=2) + "\n")
    failures = [row for row in ran if row["result"] == "fail"]
    print("poll done: ran=%d skipped=%d fail=%d evidence=%s" % (
        len(ran), skipped, len(failures), tick_path.relative_to(ROOT)))
    return 1 if failures else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interval", type=int, default=0,
                        help="seconds between passes; 0 = once")
    parser.add_argument("--force", action="store_true",
                        help="re-test items already in the skip cache")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--scan-code", action="store_true", default=True)
    parser.add_argument("--no-scan-code", action="store_false", dest="scan_code")
    parser.add_argument("--notify-on-fail", action="store_true", default=True)
    parser.add_argument("--no-notify-on-fail", action="store_false",
                        dest="notify_on_fail")
    parser.add_argument("--timeout", type=int, default=180)
    args = parser.parse_args()
    if args.interval <= 0:
        return one_pass(args)
    rc = 0
    while True:
        print("POLL_TICK eng-completed %s" % human_time())
        rc = one_pass(args) or rc
        time.sleep(args.interval)


if __name__ == "__main__":
    sys.exit(main())
