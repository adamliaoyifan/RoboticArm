#!/usr/bin/env bash
# OCC-RESET-VALIDITY formal scoring driver, v2 (gen3 repair of the run02
# driver). Order per contract §8: O9 -> V1-V3 -> P/H -> L -> R.
#
# v2 changes over the run02 driver (item 11 of the 2026-11-04_1013 review):
#   * hard stop rules enforced: O9 failure stops everything; V failure stops
#     P/L/R; P/H/L failure stops R (a failed prerequisite is never scored
#     past -- but the failure itself is recorded, not laundered);
#   * the declared budgets are actually enforced between stages: a stage
#     whose remaining budget cannot fit its declared allocation is skipped
#     with an explicit record and a nonzero final exit;
#   * signal death (rc >= 128) is recorded as a product/process failure;
#   * DRIVER_RESULT.json + stage_digest.json are always written; the final
#     exit code is nonzero if any expected stage failed or was skipped;
#   * every stage command is overridable via STAGE_CMD_<name> for OFFLINE
#     guard fixtures (never for scored runs).
#
# This script does not authorize re-running run02's score; it is the
# corrected driver for any future frozen candidate.
set -uo pipefail

W=${ELFIN_W:-/tmp/elfin-occ-reset-g1}
ROOT=${FORMAL_ROOT:-$W/docs/status/evidence/occ_reset_validity_20261103/formal_run}
BENCH="$W/src/uld_occ_observer/test/benchmark"
S2="$W/src/uld_occ_observer/test/s2_o9.py"
NODE="$W/build/uld_occ_observer/observer_node"
CLI="$W/build/uld_occ_observer/observer_profile_cli"
EVAL="$W/build/uld_occ_observer/observer_eval_native"
GEOM=${GEOM:-/tmp/observer-cursor-rehearsal/geometry_chamfer.yaml}
CONF=${CONF:-/tmp/occ-bitmap-perf-smoke/cfg_0.02.yaml}
FRAMES=${FRAMES:-'/tmp/elfin-occ-freespace-impl-g4/docs/status/evidence/occ_freespace_final_20261102/implementation_g4/rehearsal/frames/b*.yaml'}
LOG="$ROOT/ACTUAL_COMMANDS.log"
START_MONO=$(date +%s)
TOTAL_BUDGET_S=${TOTAL_BUDGET_S:-28800}     # 8 h formal budget
V13_BUDGET_S=${V13_BUDGET_S:-3600}
PHL_BUDGET_S=${PHL_BUDGET_S:-25200}         # remainder-oriented allocation
R_BUDGET_S=${R_BUDGET_S:-1800}
O9_BUDGET_S=${O9_BUDGET_S:-600}

mkdir -p "$ROOT"
: > "$LOG"
declare -A STAGE_RC
declare -A STAGE_SKIPPED
STOPPED_AT=""
STOP_REASON=""

logline() { printf '%s\n' "$*" >> "$LOG"; }

remaining_s() {
  echo $(( TOTAL_BUDGET_S - ( $(date +%s) - START_MONO ) ))
}

budget_fits() {
  local need="$1"
  [ "$(remaining_s)" -ge "$need" ]
}

skip_stage() {
  local name="$1" why="$2"
  STAGE_SKIPPED[$name]="$why"
  logline "SKIP $(date -Is) :: $name :: $why"
}

run_stage() {
  local name="$1" budget="$2"; shift 2
  if [ -n "${STOPPED_AT}" ]; then
    skip_stage "$name" "prerequisite failed: $STOPPED_AT"
    return 1
  fi
  if ! budget_fits "$budget"; then
    skip_stage "$name" "budget exhausted: remaining=$(remaining_s)s need=${budget}s"
    STOPPED_AT="$name"; STOP_REASON="budget_exhausted"
    return 1
  fi
  local override="STAGE_CMD_${name}"
  if [ -n "${!override:-}" ]; then
    set -- bash -c "${!override}"
  fi
  local t0=$(date +%s)
  logline "START $(date -Is) :: $*"
  "$@" >> "$ROOT/stage_${name}.out" 2>&1
  local rc=$?
  local t1=$(date +%s)
  STAGE_RC[$name]=$rc
  if [ "$rc" -ge 128 ]; then
    logline "END $(date -Is) exit=$rc dur=$((t1 - t0))s :: $name :: DIED BY SIGNAL $((rc - 128))"
  else
    logline "END $(date -Is) exit=$rc dur=$((t1 - t0))s :: $name"
  fi
  return $rc
}

product_failed() {
  # A stage's own exit code is the product/truth signal for the stop rules.
  local name="$1"
  [ "${STAGE_RC[$name]:-1}" -ne 0 ]
}

stop_here() {
  STOPPED_AT="$1"; STOP_REASON="${2:-product_failure}"
  logline "STOP $(date -Is) :: candidate scoring stops at $1 ($STOP_REASON); dependent stages are skipped, not scored"
}

# --- startup guards ---------------------------------------------------------
if bash -c "${STALE_CHECK_CMD:-pgrep -x observer_node >/dev/null 2>&1}"; then
  logline "ABORT $(date -Is) :: stale observer_node present"
  printf '{"stopped_at":"startup","reason":"stale_process","exit":9}\n' \
    > "$ROOT/DRIVER_RESULT.json"
  exit 9
fi

# --- O9 (qualification; takes the perf lock itself -- run it OUTSIDE ours) --
if ! run_stage o9 "$O9_BUDGET_S" python3 "$S2" --cli "$CLI" --eval-native "$EVAL" \
  --geometry "$GEOM" --config "$CONF" --out "$ROOT/o9"; then
  [ -n "$STOPPED_AT" ] || stop_here o9 "o9_failed"
fi

# --- perf lock for the scored stages (they do not self-lock) ----------------
if [ -z "$STOPPED_AT" ]; then
  exec 9>/tmp/elfin-occ-backend-perf.lock
  if ! flock -n 9; then
    logline "ABORT $(date -Is) :: perf lock busy"
    printf '{"stopped_at":"startup","reason":"perf_lock_busy","exit":9}\n' \
      > "$ROOT/DRIVER_RESULT.json"
    exit 9
  fi
  logline "LOCK acquired $(date -Is) :: /tmp/elfin-occ-backend-perf.lock"
fi

# --- V1-V3 reset safety ------------------------------------------------------
if ! run_stage v1v3 "$V13_BUDGET_S" python3 "$BENCH/reset_campaign.py" --mode score \
  --manifest integ-frozen --out "$ROOT/reset_v1v3" \
  --node "$NODE" --geometry "$GEOM" --config "$CONF" --frames-glob "$FRAMES"; then
  [ -n "$STOPPED_AT" ] || stop_here v1v3 "reset_safety_gate_failed"
elif ! run_stage v1v3_judge 60 python3 "$BENCH/reset_judge.py" \
    --run "$ROOT/reset_v1v3"; then
  [ -n "$STOPPED_AT" ] || stop_here v1v3_judge "reset_safety_gate_failed"
fi

# --- P/H + L (P failure stops L inside the campaign; L verdict from judges) --
if [ -z "$STOPPED_AT" ]; then
  if ! run_stage phl "$PHL_BUDGET_S" env OCC_FS_SCORE_OPEN=1 \
    python3 "$BENCH/freespace_campaign.py" \
    --node "$NODE" --eval-native "$EVAL" --frames-glob "$FRAMES" \
    --out "$ROOT/campaign" --stages p,l --formal; then
    [ -n "$STOPPED_AT" ] || stop_here phl "phl_failed"
  fi
fi

# --- R pressure (sweep -> frozen rate -> 3 scored windows + recovery) --------
if [ -z "$STOPPED_AT" ]; then
  if ! run_stage r_sweep "$((R_BUDGET_S / 3))" python3 "$BENCH/reset_pressure.py" --mode sweep \
    --node "$NODE" --geometry "$GEOM" --config "$CONF" --frames-glob "$FRAMES" \
    --out "$ROOT/pressure"; then
    [ -n "$STOPPED_AT" ] || stop_here r_sweep "r_sweep_failed"
  else
    FROZEN=$(python3 -c "
import json,sys
try: print(json.load(open('$ROOT/pressure/sweep_result.json'))['frozen_rate_hz'] or '')
except Exception: print('')")
    if [ -n "$FROZEN" ]; then
      if ! run_stage r_score "$((R_BUDGET_S / 3))" python3 "$BENCH/reset_pressure.py" --mode score \
        --frozen-rate "$FROZEN" --windows 3 --window-seconds 60 --recovery-seconds 30 \
        --node "$NODE" --geometry "$GEOM" --config "$CONF" --frames-glob "$FRAMES" \
        --out "$ROOT/pressure"; then
        [ -n "$STOPPED_AT" ] || stop_here r_score "r_score_failed"
      elif ! run_stage r_judge 60 python3 "$BENCH/reset_pressure.py" --mode judge \
          --out "$ROOT/pressure"; then
        [ -n "$STOPPED_AT" ] || stop_here r_judge "r_judge_failed"
      fi
    else
      skip_stage r_score "not_evaluated: pressure_unreached (cap 80 Hz)"
      skip_stage r_judge "not_evaluated: pressure_unreached"
      logline "R not_evaluated $(date -Is) :: pressure_unreached"
    fi
  fi
fi

# --- closeout ----------------------------------------------------------------
if [ -e /proc/$$/fd/9 ]; then exec 9>&-; fi
TOTAL=$(( $(date +%s) - START_MONO ))
logline "DONE $(date -Is) total=${TOTAL}s stopped_at=${STOPPED_AT:-none}"

python3 - "$ROOT" "$STOPPED_AT" "$STOP_REASON" <<'EOF'
import json, sys
from pathlib import Path
root, stopped_at, stop_reason = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
result = {"stages": {}, "stopped_at": stopped_at or None,
          "stop_reason": stop_reason or None}
log = (root / "ACTUAL_COMMANDS.log").read_text().splitlines()
for name in ("o9", "v1v3", "v1v3_judge", "phl", "r_sweep", "r_score", "r_judge"):
    entry = {"ran": (root / ("stage_%s.out" % name)).exists()}
    for line in log:
        if line.startswith("END ") and line.rstrip().endswith(":: " + name):
            entry["exit"] = int(line.split("exit=")[1].split()[0])
        if line.startswith("SKIP ") and (":: %s ::" % name) in line:
            entry["skipped"] = line.split(":: %s ::" % name)[1].strip()
    result["stages"][name] = entry
failed = [n for n, e in result["stages"].items() if e.get("exit", 0) != 0]
skipped_bad = [n for n, e in result["stages"].items()
               if e.get("skipped") and "not_evaluated" not in e["skipped"]]
result["final_exit"] = 1 if (failed or skipped_bad or stopped_at) else 0
(root / "DRIVER_RESULT.json").write_text(json.dumps(result, indent=1))
sys.exit(result["final_exit"])
EOF
exit $?
