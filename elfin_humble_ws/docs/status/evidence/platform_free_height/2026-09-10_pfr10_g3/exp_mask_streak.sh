#!/usr/bin/env bash
# Two more C1 scored runs after exp_mask_accepted2.
set +u
set -o pipefail

ROOT=/home/adamliao/work/elfin_humble_ws
EVID_ROOT="$ROOT/docs/status/evidence/platform_free_height/2026-09-10_pfr10_g3"
PIDFILE=/tmp/elfin_humble_sim.pid
export ELFIN_SIM_PIDFILE="$PIDFILE"
export ROS_DOMAIN_ID=7
export PYTHONUNBUFFERED=1

LAUNCH_PARAMS='gui:=false use_rviz:=false use_semantic:=true use_motion:=true use_vacuum:=true visual_kind:=mesh size_mode:=catalog sequence_ids:=carryon,standard,large xy_jitter_range:=0.12,0.12 yaw_range:=-0.6,0.6 observe_pose_name:=pickup_observe'

source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
set -u

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

if pgrep -f '/opt/ros/humble/bin/ros2 launch luggage_gazebo' >/dev/null; then
  echo "STREAK_FAIL sim already running"
  exit 1
fi

c1_ok() {
  python3 - "$1" <<'PY'
import json, sys
gate = json.loads(open(sys.argv[1]).read())
fails = []
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
for rec in gate.get("trial_recoveries") or []:
    n = rec.get("n_settled")
    t = rec.get("t_first_full3d_sec")
    if n is None or int(n) < 30:
        fails.append("trial %s n_settled=%s" % (rec.get("trial"), n))
    if t is None or float(t) > 1.4:
        fails.append("trial %s t_first_full3d=%s" % (rec.get("trial"), t))
if fails:
    print("C1_FAIL: " + " | ".join(fails))
    sys.exit(1)
print("C1_PASS")
PY
}

run_one() {
  local name="$1"
  local out="$EVID_ROOT/$name"
  mkdir -p "$out/gate4"
  rm -f "$out/STOP"
  local record="$LAUNCH_PARAMS ($name; accepted-only cargo mask; vectorized refine; no dump-dir)"
  echo "=== $name start $(date -Iseconds) ==="

  "$ROOT/scripts/stop_sim.sh" >"$out/stop_sim_pre.log" 2>&1 || true
  sleep 8
  : > "$out/launch.log"
  cd "$ROOT"
  ros2 launch luggage_gazebo sim_world.launch.py \
    gui:=false use_rviz:=false use_semantic:=true use_motion:=true \
    use_vacuum:=true visual_kind:=mesh size_mode:=catalog \
    sequence_ids:=carryon,standard,large xy_jitter_range:=0.12,0.12 \
    yaw_range:=-0.6,0.6 observe_pose_name:=pickup_observe \
    >"$out/launch.log" 2>&1 &
  echo $! > "$PIDFILE"
  if ! wait_held_pose "$out/launch.log"; then
    echo "STREAK_FAIL $name sim never ready" | tee "$out/RESULT.txt"
    "$ROOT/scripts/stop_sim.sh" >"$out/stop_sim_bootfail.log" 2>&1 || true
    return 1
  fi
  sleep 8

  python3 "$ROOT/src/luggage_perception/test/pf_r10_g6s_probe.py" \
    --out "$out" --duration 240 --stop-file "$out/STOP" \
    >"$out/g6s_probe.log" 2>&1 &
  local probe_pid=$!
  date +%s > "$out/eval_start_epoch"
  python3 "$ROOT/scripts/platform_free_height_gate4_eval.py" \
    --out "$out/gate4" --trials 6 --settle-sec 8.0 --warmup-frames 30 \
    --min-trials-per-size 2 \
    --launch-params "$record" \
    >"$out/gate4_stdout.log" 2>&1
  local gate_rc=$?
  echo "$gate_rc" > "$out/gate4_rc"
  date +%s > "$out/eval_end_epoch"
  touch "$out/STOP"
  wait "$probe_pid" || true
  echo "$?" > "$out/probe_rc"

  "$ROOT/scripts/stop_sim.sh" >"$out/stop_sim.log" 2>&1 || true
  sleep 2
  local residual
  residual="$(count_stack)"
  echo "$residual" > "$out/residual_count"

  if [[ "$residual" != "0" ]]; then
    echo "STREAK_FAIL $name residual=$residual" | tee "$out/RESULT.txt"
    return 1
  fi
  if [[ "$gate_rc" != "0" ]]; then
    echo "STREAK_FAIL $name gate_rc=$gate_rc" | tee "$out/RESULT.txt"
    return 1
  fi
  if ! c1_ok "$out/gate4/summary.json"; then
    echo "STREAK_FAIL $name c1" | tee "$out/RESULT.txt"
    return 1
  fi
  echo "RUN_PASS $name residual=0" | tee "$out/RESULT.txt"
  return 0
}

if ! run_one exp_mask_r3; then echo STREAK_FAIL; exit 1; fi
if ! run_one exp_mask_r4; then echo STREAK_FAIL; exit 1; fi
echo STREAK_DONE
exit 0
