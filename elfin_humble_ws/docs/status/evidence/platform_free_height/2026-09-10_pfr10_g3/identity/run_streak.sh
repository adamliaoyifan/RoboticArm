#!/usr/bin/env bash
# PF-R10 g3 identity: three consecutive gate4_short6 + PF-G6S on df9c7a2.
set +u
set -o pipefail

ROOT=/home/adamliao/work/elfin_humble_ws
WT=/tmp/pfr10_g3
EVID="$ROOT/docs/status/evidence/platform_free_height/2026-09-10_pfr10_g3/identity"
COMMIT=df9c7a27f8ad81102b6f5462b08b724910b042cd
PIDFILE=/tmp/elfin_humble_sim.pid
export ELFIN_SIM_PIDFILE="$PIDFILE"
export ROS_DOMAIN_ID=7
export PYTHONUNBUFFERED=1

LAUNCH_PARAMS='gui:=false use_rviz:=false use_semantic:=true use_motion:=true use_vacuum:=true visual_kind:=mesh size_mode:=catalog sequence_ids:=carryon,standard,large xy_jitter_range:=0.12,0.12 yaw_range:=-0.6,0.6 observe_pose_name:=pickup_observe'
LAUNCH_RECORD="$LAUNCH_PARAMS (worktree $WT @ ${COMMIT:0:7}, dirty=0)"

source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
set -u

mkdir -p "$EVID"

if [[ "$(git -C "$WT" rev-parse HEAD)" != "$COMMIT" ]]; then
  echo "STREAK_FAIL worktree HEAD is not $COMMIT"
  git -C "$WT" rev-parse HEAD
  exit 1
fi
if [[ -n "$(git -C "$WT" status --porcelain)" ]]; then
  echo "STREAK_FAIL worktree dirty"
  git -C "$WT" status --porcelain
  exit 1
fi

count_stack() {
  local self=$$
  {
    pgrep -a -f 'airport_loading\.sdf' 2>/dev/null || true
    pgrep -a -f '/opt/ros/humble/bin/ros2 launch luggage_gazebo' 2>/dev/null || true
    pgrep -a -f 'ign gazeb[o]' 2>/dev/null || true
  } | awk -v self="$self" '
    NF && $1 ~ /^[0-9]+$/ && $1 != self && !seen[$1]++ { c++ }
    END { print c+0 }
  '
}

wait_held_pose() {
  local log="$1"
  local deadline=$((SECONDS + 180))
  while (( SECONDS < deadline )); do
    if grep -q "Held pose 'pickup_observe' error_code=0" "$log" 2>/dev/null; then
      return 0
    fi
    if grep -q "Held pose 'pickup_observe' error_code=" "$log" 2>/dev/null && \
       ! grep -q "Held pose 'pickup_observe' error_code=0" "$log" 2>/dev/null; then
      return 2
    fi
    sleep 1
  done
  return 1
}

start_sim() {
  local log="$1"
  : > "$log"
  cd "$ROOT"
  ros2 launch luggage_gazebo sim_world.launch.py \
    gui:=false use_rviz:=false use_semantic:=true use_motion:=true \
    use_vacuum:=true visual_kind:=mesh size_mode:=catalog \
    sequence_ids:=carryon,standard,large xy_jitter_range:=0.12,0.12 \
    yaw_range:=-0.6,0.6 observe_pose_name:=pickup_observe \
    >"$log" 2>&1 &
  echo $! > "$PIDFILE"
  wait_held_pose "$log"
}

stop_stack() {
  local log="$1"
  "$ROOT/scripts/stop_sim.sh" >"$log" 2>&1 || true
  sleep 20
}

c2_and_c1_ok() {
  python3 - "$1" "$2" "$COMMIT" <<'PY'
import json, sys
gate = json.loads(open(sys.argv[1]).read())
g6s = json.loads(open(sys.argv[2]).read())
want = sys.argv[3]
fails = []
rev = gate.get("revision") or {}
commit = str(rev.get("git_commit") or "")
if not commit.startswith(want[:7]):
    fails.append("commit %s" % commit)
dirty = rev.get("git_dirty_files")
if dirty is None or int(dirty) != 0:
    fails.append("dirty=%s" % dirty)
if not gate.get("gate4_pass"):
    fails.append("gate4_pass false: %s" % gate.get("gate4_failures"))
hz = gate.get("active_output_hz")
if hz is None or float(hz) < 4.0:
    fails.append("active_output_hz %s" % hz)
if float(gate.get("top_surface_rate") or 0) < 0.95:
    fails.append("top_surface_rate %s" % gate.get("top_surface_rate"))
if float(gate.get("full3d_rate") or 0) < 0.95:
    fails.append("full3d_rate %s" % gate.get("full3d_rate"))
if int(gate.get("false_measured_height") or 0) != 0:
    fails.append("false_measured_height")
if int((gate.get("categories") or {}).get("failed") or 0) != 0:
    fails.append("failed %s" % (gate.get("categories") or {}).get("failed"))
if int(gate.get("spawn_failures") or 0) != 0:
    fails.append("spawn_failures %s" % gate.get("spawn_failures"))
lag = g6s.get("executor_lag_sec") or {}
if not lag.get("q4_mean_ok") or not lag.get("ratio_ok"):
    fails.append("executor_lag %s" % lag)
for name, entry in (g6s.get("rss") or {}).items():
    if not entry.get("pass"):
        fails.append("rss %s slope=%s" % (name, entry.get("slope_mib_per_min")))
for name, entry in (g6s.get("occupancy") or {}).items():
    if entry.get("peak_within_maxlen") is False:
        fails.append("occ peak %s" % name)
    if name.startswith("filter.") and "q4_mean_within_half" in entry \
            and not entry.get("q4_mean_within_half"):
        fails.append("occ q4 %s" % name)
if fails:
    print("C1C2_FAIL: " + " | ".join(fails))
    sys.exit(1)
print("C1C2_PASS")
PY
}

run_one() {
  local name="$1"
  local out="$EVID/$name"
  mkdir -p "$out/gate4"
  rm -f "$out/STOP"
  echo "=== $name start $(date -Iseconds) ==="

  local tries=0
  local ready=1
  while (( tries < 2 )); do
    tries=$((tries + 1))
    stop_stack "$out/stop_sim_pre.log"
    if ! start_sim "$out/launch.log"; then
      echo "$name sim ready failed try=$tries"
      stop_stack "$out/stop_sim_bootfail.log"
      ready=1
      continue
    fi
    ready=0
    break
  done
  if (( ready != 0 )); then
    echo "STREAK_FAIL $name sim never ready" | tee "$out/RESULT.txt"
    return 1
  fi
  sleep 8

  cd "$WT"
  python3 "$WT/src/luggage_perception/test/pf_r10_g6s_probe.py" \
    --out "$out" --duration 240 --stop-file "$out/STOP" \
    >"$out/g6s_probe.log" 2>&1 &
  local probe_pid=$!
  date +%s > "$out/eval_start_epoch"
  python3 "$WT/scripts/platform_free_height_gate4_eval.py" \
    --out "$out/gate4" --trials 6 --settle-sec 8.0 --warmup-frames 30 \
    --min-trials-per-size 2 \
    --launch-params "$LAUNCH_RECORD" \
    >"$out/gate4_stdout.log" 2>&1
  local gate_rc=$?
  echo "$gate_rc" > "$out/gate4_rc"
  date +%s > "$out/eval_end_epoch"
  touch "$out/STOP"
  wait "$probe_pid" || true
  echo "$?" > "$out/probe_rc"

  stop_stack "$out/stop_sim.log"
  local residual
  residual="$(count_stack)"
  echo "$residual" > "$out/residual_count"
  echo "$name residual=$residual gate_rc=$gate_rc"

  if [[ "$residual" != "0" ]]; then
    echo "STREAK_FAIL $name residual=$residual" | tee "$out/RESULT.txt"
    return 1
  fi
  if ! c2_and_c1_ok "$out/gate4/summary.json" "$out/g6s_summary.json" "$COMMIT"; then
    echo "STREAK_FAIL $name c1/c2" | tee "$out/RESULT.txt"
    return 1
  fi
  echo "RUN_PASS $name" | tee "$out/RESULT.txt"
  return 0
}

if pgrep -f '/opt/ros/humble/bin/ros2 launch luggage_gazebo' >/dev/null; then
  echo "STREAK_FAIL sim already running"
  exit 1
fi

if ! run_one run1; then echo STREAK_FAIL; exit 1; fi
if ! run_one run2; then echo STREAK_FAIL; exit 1; fi
if ! run_one run3; then echo STREAK_FAIL; exit 1; fi
echo STREAK_DONE
exit 0
