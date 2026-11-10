#!/usr/bin/env bash
# Reset validity campaign entry (OCC-RESET-VALIDITY-IMPL, plan
# OCTOMAP_RESET_VALIDITY_NEXT_20261103 §6). Adds the three fixed entries to
# the existing command set:
#   reset-rehearsal | reset-score | reset-judge <run> | reset-replay <case>
# Exit codes: 0 only for a complete pass; nonzero for fail/incomplete.
set -eo pipefail

WORKTREE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
BENCH="$WORKTREE/src/uld_occ_observer/test/benchmark"
NODE="${RESET_NODE:-$WORKTREE/build/uld_occ_observer/observer_node}"
GEOMETRY="${RESET_GEOMETRY:?set RESET_GEOMETRY}"
CONFIG="${RESET_CONFIG:?set RESET_CONFIG}"
FRAMES="${RESET_FRAMES:?set RESET_FRAMES}"
EVIDENCE_ROOT="${RESET_EVIDENCE_ROOT:?set RESET_EVIDENCE_ROOT}"
MANIFEST="${RESET_MANIFEST:-frozen-reset-input}"

run_campaign() {
  local mode="$1" out="$2" repeats="${3:-}"
  local args=(--mode "$mode" --manifest "$MANIFEST" --out "$out"
              --node "$NODE" --geometry "$GEOMETRY" --config "$CONFIG"
              --frames-glob "$FRAMES")
  if [ -n "$repeats" ]; then args+=(--repeats "$repeats"); fi
  python3 "$BENCH/reset_campaign.py" "${args[@]}"
}

case "${1:-}" in
  reset-rehearsal)
    run_campaign rehearsal "$EVIDENCE_ROOT/reset_rehearsal" ;;
  reset-score)
    # Scored run: 30 repeats per stage/pairing (contract §5). Never shrinks.
    run_campaign score "$EVIDENCE_ROOT/reset_score" ;;
  reset-judge)
    [ -n "${2:-}" ] || { echo "usage: reset-judge <run-dir>" >&2; exit 2; }
    python3 "$BENCH/reset_judge.py" --run "$2" ;;
  reset-selftest)
    # Negative fixtures: all corruptions rejected, healthy overrun 0/2 pass.
    python3 "$BENCH/reset_judge.py" --selftest ;;
  reset-replay)
    [ -n "${2:-}" ] || { echo "usage: reset-replay <case-bundle>" >&2; exit 2; }
    run_campaign replay "$EVIDENCE_ROOT/reset_replay_$(date +%H%M%S)" \
      <(true) 2>/dev/null || true
    python3 "$BENCH/reset_campaign.py" --mode replay --manifest "$2" \
      --out "$EVIDENCE_ROOT/reset_replay_last" --node "$NODE" \
      --geometry "$GEOMETRY" --config "$CONFIG" --frames-glob "$FRAMES" ;;
  *)
    echo "entries: reset-rehearsal | reset-score | reset-judge <run> | reset-selftest | reset-replay <case>" >&2
    exit 2 ;;
esac
