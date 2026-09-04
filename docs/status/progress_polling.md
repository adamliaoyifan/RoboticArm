# Progress Polling

`scripts/progress_poll.py` builds deterministic project progress documents from
repository facts. It reads Git commits, agent thread Result events, completed
review notes, the open mailbox, and approved plan tables. It does not inspect
chat transcripts or start ROS and simulation processes.

## Output

- Daily: `docs/status/progress/daily/YYYY-MM-DD.md`
- Weekly: `docs/status/progress/weekly/YYYY-Www.md`
- Index: `docs/status/progress/README.md`

Each report records completed and failed Results, commits, review decisions,
current execution, future plan rows, current blockers, and workspace status.
Reports overwrite the same date/week atomically and only when source facts
change.

## Commands

Refresh today's daily report. On Sunday this also refreshes the ISO weekly
report:

```bash
python3 scripts/progress_poll.py
```

Continuously poll in the foreground every 15 minutes:

```bash
python3 scripts/progress_poll.py --watch --interval 900
```

Backfill a date or explicitly generate that date's weekly report:

```bash
python3 scripts/progress_poll.py --date 2026-09-06
python3 scripts/progress_poll.py --date 2026-09-06 --force-weekly
```

Install a persistent user-level systemd timer. The timer checks every 15
minutes, so Sunday progress is summarized throughout the day:

```bash
scripts/install_progress_poll_timer.sh --dry-run
scripts/install_progress_poll_timer.sh --interval 15min
systemctl --user list-timers elfin-progress-poll.timer
```

Remove it with:

```bash
scripts/install_progress_poll_timer.sh --uninstall
```

## Interpretation

- A passing audit Result is listed as completed work even when its independent
  readiness field is blocked.
- `gt_readiness=blocked`, `nbv_readiness=blocked`, and latest blocked Results
  remain in Current Blockers until a later Result clears them.
- Future Plan includes open mailbox work and approved plan rows that are not
  yet completed or dispatched.
- Dirty files are context only. They are not counted as progress without a Git
  commit or agent Result.
