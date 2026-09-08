#!/usr/bin/env bash
# Install or remove the repository progress poller as a user systemd timer.
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
UNIT_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
SERVICE="$UNIT_DIR/elfin-progress-poll.service"
TIMER="$UNIT_DIR/elfin-progress-poll.timer"
INTERVAL="15min"
MODE="install"

usage() {
  cat <<'EOF'
Usage:
  scripts/install_progress_poll_timer.sh [--interval 15min] [--dry-run]
  scripts/install_progress_poll_timer.sh --uninstall

The user timer runs progress_poll.py periodically. Each run refreshes today's
daily report, and Sunday runs also refresh the current ISO weekly report.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --interval)
      INTERVAL="${2:-}"
      shift 2
      ;;
    --dry-run)
      MODE="dry-run"
      shift
      ;;
    --uninstall)
      MODE="uninstall"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! "$INTERVAL" =~ ^[1-9][0-9]*(s|min|h)$ ]]; then
  echo "invalid --interval '$INTERVAL' (examples: 30s, 15min, 2h)" >&2
  exit 2
fi

service_text() {
  cat <<EOF
[Unit]
Description=Refresh elfin workspace daily and weekly progress summaries

[Service]
Type=oneshot
WorkingDirectory=$ROOT
ExecStart=/usr/bin/python3 $ROOT/scripts/progress_poll.py --root $ROOT
EOF
}

timer_text() {
  cat <<EOF
[Unit]
Description=Poll elfin workspace progress every $INTERVAL

[Timer]
OnBootSec=2min
OnUnitActiveSec=$INTERVAL
Persistent=true
Unit=elfin-progress-poll.service

[Install]
WantedBy=timers.target
EOF
}

if [[ "$MODE" == "dry-run" ]]; then
  printf '%s\n' "--- elfin-progress-poll.service"
  service_text
  printf '%s\n' "--- elfin-progress-poll.timer"
  timer_text
  exit 0
fi

if [[ "$MODE" == "uninstall" ]]; then
  systemctl --user disable --now elfin-progress-poll.timer 2>/dev/null || true
  rm -f "$SERVICE" "$TIMER"
  systemctl --user daemon-reload
  echo "removed elfin-progress-poll.timer"
  exit 0
fi

mkdir -p "$UNIT_DIR"
service_text > "$SERVICE"
timer_text > "$TIMER"
systemctl --user daemon-reload
systemctl --user enable --now elfin-progress-poll.timer
systemctl --user --no-pager status elfin-progress-poll.timer
