#!/usr/bin/env bash
# Bounded static precheck boot (user authorization 2026-10-21):
#   gz + controllers, NO motion goal; 30-minute wall cap; failure = evidence
#   + stop (no mid-precheck fixing of gates).
# Probe discipline (attempt-4): controllers-active is detected by the
# SIDECAR seeing /joint_states rows (DDS-service probes false-negative
# under full-stack load); the recorder is closed via ctl/stop so footers
# and the manifest are written by design, never by process kill.
set -o pipefail
export AMENT_TRACE_SETUP_FILES=0   # setup.bash is not -u clean

WS=/tmp/elfin-master-pack-baseline-g3
MAIN=/home/adamliao/work/elfin_humble_ws
ROOT=/tmp/jcc_runs/static_$(date +%Y%m%d_%H%M%S)
ON=$ROOT/on
OFF=$ROOT/off
DEADLINE=$(( $(date +%s) + 1800 ))   # user cap: 30 min wall

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export PYTHONPATH="$WS/src/joint_command_capture/python:$PYTHONPATH"

mkdir -p "$ON" "$OFF"

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$ROOT/boot.log"; }

stop_all() {
  bash "$MAIN/scripts/stop_sim.sh" --all >/dev/null 2>&1 || true
  sleep 2
  pkill -TERM -f "gz sim" >/dev/null 2>&1 || true
  pkill -TERM -f "ign gazebo" >/dev/null 2>&1 || true
  sleep 3
  pkill -9 -f "gz sim" >/dev/null 2>&1 || true
  pkill -9 -f "ign gazebo" >/dev/null 2>&1 || true
  rm -f /dev/shm/fastrtps_* >/dev/null 2>&1 || true
}

exclusivity() {
  stop_all
  if pgrep -f "gazebo" >/dev/null 2>&1; then
    log "EXCLUSIVITY_UNMET: foreign gazebo still running; aborting"
    return 1
  fi
  return 0
}

deadline_check() {
  if [ "$(date +%s)" -gt "$DEADLINE" ]; then
    log "WALL_DEADLINE_EXCEEDED; stopping precheck"
    exit 3
  fi
}

# run_phase <phase> <run_dir> <sim_seconds> [extra launch args...]
run_phase() {
  local phase=$1 dir=$2 sim_s=$3
  shift 3
  log "phase=$phase start dir=$dir extra=$*"
  ros2 launch luggage_gazebo sim_world.launch.py \
    gui:=false use_rviz:=false use_semantic:=false use_motion:=false \
    use_vacuum:=false use_packing:=false "$@" \
    > "$ROOT/${phase}_launch.log" 2>&1 &
  local lpid=$!
  echo "$lpid" > "$ROOT/${phase}_launch_pid"
  # Sidecar starts immediately; it only writes rows after the first /clock
  # (i.e. after gz is up) and gates /joint_states on the live controller.
  timeout 400 python3 "$WS/scripts/g3/jcc_static_precheck.py" record "$dir" \
    --sim-seconds "$sim_s" > "$dir/sidecar_stdout.json" 2>&1 &
  local spid=$!
  # Controllers-active probe: sidecar joint_states rows appear.
  local ok="" n=0
  while [ $n -lt 80 ]; do
    sleep 5
    if [ -s "$dir/sidecar_joint_states.jsonl" ]; then ok=1; break; fi
    kill -0 "$lpid" 2>/dev/null || break
    kill -0 "$spid" 2>/dev/null || break
    deadline_check
    n=$((n+1))
  done
  if [ -z "$ok" ]; then
    log "PHASE_FAIL phase=$phase: no joint_states rows within probe window"
    tail -30 "$ROOT/${phase}_launch.log" >> "$ROOT/boot.log" || true
    kill -INT "$lpid" 2>/dev/null || true; kill -TERM "$spid" 2>/dev/null || true
    sleep 5; pkill -TERM -f "ign gazebo" >/dev/null 2>&1 || true; sleep 5
    pkill -9 -f "ign gazebo" >/dev/null 2>&1 || true
    echo "${phase}_PHASE_NO_JOINT_STATES" >> "$dir/PRECHECK_GAPS.txt"
    return 1
  fi
  log "phase=$phase controllers alive (joint_states flowing)"
  # Wait for the sidecar to finish its sim window.
  for _ in $(seq 1 240); do
    kill -0 "$spid" 2>/dev/null || break
    sleep 2
    deadline_check
  done
  kill -TERM "$spid" 2>/dev/null || true
  return 0
}

shutdown_phase() {
  local phase=$1 dir=$2
  # Clean recorder close by design: ctl/stop, then wait for the manifest.
  if [ -d "$dir/capture/ctl" ]; then
    : > "$dir/capture/ctl/stop"
    for _ in $(seq 1 30); do
      [ -f "$dir/capture/manifest_recorder.json" ] && break
      sleep 2
    done
  fi
  local lpid
  lpid=$(cat "$ROOT/${phase}_launch_pid" 2>/dev/null || echo "")
  [ -n "$lpid" ] && kill -INT "$lpid" 2>/dev/null || true
  for _ in $(seq 1 20); do
    [ -n "$lpid" ] && kill -0 "$lpid" 2>/dev/null || break
    sleep 1
  done
  [ -n "$lpid" ] && kill -TERM "$lpid" 2>/dev/null || true
  sleep 3
  pkill -TERM -f "ign gazebo" >/dev/null 2>&1 || true
  sleep 5
  pkill -9 -f "ign gazebo" >/dev/null 2>&1 || true
  sleep 2
}

log "=== static precheck start ROOT=$ROOT deadline=${DEADLINE} ==="
exclusivity || exit 2

# ---- Phase ON (capture window mode) ----
ON_CAP=$ON/capture
if run_phase on "$ON" 30 joint_capture_mode:=window joint_capture_run_dir:="$ON_CAP"; then
  python3 "$WS/scripts/g3/jcc_static_precheck.py" maps "$ON/gz_loaded_libraries.json" \
    > "$ON/maps_stdout.json" 2>&1 || echo "MAPS_CAPTURE_FAILED" >> "$ON/PRECHECK_GAPS.txt"
  shutdown_phase on "$ON"
  [ -f "$ON_CAP/manifest_recorder.json" ] || \
    echo "RECORDER_MANIFEST_MISSING" >> "$ON/PRECHECK_GAPS.txt"
fi
deadline_check

# ---- Phase OFF (no capture wiring) ----
if run_phase off "$OFF" 20; then
  shutdown_phase off "$OFF"
  if ls "$OFF"/*.bin >/dev/null 2>&1 || [ -e "$OFF/health.json" ] || [ -d "$OFF/ctl" ]; then
    echo "OFF_RUN_HAS_CAPTURE_ARTIFACTS" >> "$OFF/PRECHECK_GAPS.txt"
  fi
fi

# ---- Verdict ----
python3 "$WS/scripts/g3/jcc_static_precheck.py" verdict "$ON/capture" --off-dir "$OFF" \
  > "$ROOT/PRECHECK_REPORT_stdout.json" 2>&1
rc=$?
log "verdict rc=$rc; report at $ROOT/PRECHECK_REPORT_stdout.json"
log "=== static precheck end ==="
exit "$rc"
