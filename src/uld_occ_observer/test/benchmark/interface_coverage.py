"""Phase A. Read historical coverage. Do not start a new load."""
from __future__ import annotations

import hashlib
import json
import subprocess
from pathlib import Path

EVIDENCE = Path("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029")
PLAN = "d7af0b70fc92c30dcde50e5386f935d36447e1f8:docs/plans/OCC_OCTOMAP_OBSERVER_G1_20261026.md"
WORKTREE = Path("/tmp/elfin-occ-octomap-perf-cursor-g1")


def sha256(path: Path):
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def plan_rows():
    text = subprocess.check_output(["git", "-C", str(WORKTREE), "show", PLAN], text=True)
    rows = []
    for line in text.splitlines():
        if line.startswith("| O5") or line.startswith("| O6") or line.startswith("| O7") or line.startswith("| O9"):
            rows.append(line)
    return rows


def main(out_dir):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    groups = (EVIDENCE / "closeout_g2" / "GROUP_LEDGER.jsonl").read_text().splitlines()
    windows = json.loads((EVIDENCE / "closeout_g2" / "WINDOW_LEDGER.json").read_text())
    pending = json.loads((EVIDENCE / "closeout_g2" / "PENDING_IDS.json").read_text())
    normal = json.loads((EVIDENCE / "run03" / "o7" / "normal_b00_r0" / "ledger.json").read_text())
    overload = json.loads((EVIDENCE / "run03" / "o7" / "overload" / "ledger.json").read_text())
    query = normal["queries"][0]
    health_keys = sorted(normal.keys())
    window_keys = sorted(next(iter(windows["normal"].values())).keys())
    lines = [
        "# O5–O7 historical coverage",
        "",
        "This file only re-reads frozen evidence. It does not replace a missing sample and it does not change O6 or O7 failures.",
        "",
        "## Contract rows",
        "",
        "Source `%s`." % PLAN,
        "",
    ]
    lines.extend(plan_rows())
    lines.extend([
        "",
        "O5's physical ROI text is `4096cell ROI±.15×.3`. The O7 harness calls ReadCells with begin `[0,0,0]`, end `[16,16,16]`, limit 4096. That is 4096 cells at 1 cm, a 0.16 m cube from the grid origin. It is not the same statement as a ±0.15 m by 0.3 m physical window. This collection does not redefine either one.",
        "",
        "## What the frozen runs actually contain",
        "",
        "- Group ledger lines: %d. O6 failure `r01_b00_cuboid_c30000_o50_x50` stays 517.297808 ms in the closeout judge. This phase does not recompute that percentile from a new process." % len(groups),
        "- Normal windows: %d. Each closeout window record has keys `%s`." % (len(windows["normal"]), ", ".join(window_keys)),
        "- `snapshot_age` in the closeout normal record is the string `not_in_normal_ledger`.",
        "- Raw normal ledger keys: `%s`." % ", ".join(health_keys),
        "- A stored query has keys `%s`. There is no ROI length, content hash, release status, or planned timestamp." % ", ".join(sorted(query)),
        "- Health samples in the raw normal ledger: %d timestamps. The payload text is not stored. score_start and score_end are not fields of that ledger." % len(normal["health_times"]),
        "- Overload ledger keys: `%s`." % ", ".join(sorted(overload)),
        "- Pending ids left unknown: %d. A new experiment must not backfill their historical terminal state. Completion field: `%s`." % (pending["count"], pending["completion"]),
        "",
        "## Hashes",
        "",
    ])
    for path in (
        EVIDENCE / "closeout_g2" / "GROUP_LEDGER.jsonl",
        EVIDENCE / "closeout_g2" / "WINDOW_LEDGER.json",
        EVIDENCE / "closeout_g2" / "PENDING_IDS.json",
        EVIDENCE / "run03" / "o7" / "overload" / "ledger.json",
        WORKTREE / "src" / "uld_occ_observer" / "src" / "observer_node.cpp",
    ):
        lines.append("- `%s` `%s`" % (path, sha256(path)))
    lines.append("")
    lines.append("Formal O5 remains not_evaluated. ApplyBatch representative runs are a different fixture set and are not a substitute for the O5 row above.")
    text = "\n".join(lines) + "\n"
    (out / "O5_O7_COVERAGE.md").write_text(text)
    print(out / "O5_O7_COVERAGE.md")
    return 0


if __name__ == "__main__":
    import sys
    raise SystemExit(main(sys.argv[1]))
