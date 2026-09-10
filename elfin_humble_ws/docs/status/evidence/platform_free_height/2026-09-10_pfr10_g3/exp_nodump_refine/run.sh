#!/usr/bin/env bash
# PF-R10 g3 experiment: vectorized refine, scored gate4_short6, no --dump-dir.
set +u
set -o pipefail

ROOT=/home/adamliao/work/elfin_humble_ws
EVID="$ROOT/docs/status/evidence/platform_free_height/2026-09-10_pfr10_g3/exp_nodump_refine"
PIDFILE=/tmp/elfin_humble_sim.pid
export ELFIN_SIM_PIDFILE="$PIDFILE"
export ROS_DOMAIN_ID=7
export PYTHONUNBUFFERED=1

LAUNCH_PARAMS='gui:=false use_rviz:=false use_semantic:=true use_motion:=true use_vacuum:=true visual_kind:=mesh size_mode:=catalog sequence_ids:=carryon,standard,large xy_jitter_range:=0.12,0.12 yaw_range:=-0.6,0.6 observe_pose_name:=pickup_observe'
LAUNCH_RECORD="$LAUNCH_PARAMS (exp_nodump_refine; vectorized _refine_rectangle; no dump-dir)"

source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
set -u

mkdir -p "$EVID/gate4"
: > "$EVID/launch.log"

count_stack() {
  local self=$$
  {
    pgrep -a -f 'airport_loading\.sdf' 2>/dev/null || true
    pgrep -a -f 'ros2 launch luggage_gazebo' 2>/dev/null || true
    pgrep -a -f 'ign gazeb[o]' 2>/dev/null || true
    pgrep -a -f '/install/luggage_(gazebo|perception|planning|description|packing)/' 2>/dev/null || true
  } | awk -v self="$self" '
    NF && $1 ~ /^[0-9]+$/ && $1 != self && !seen[$1]++ {
      cmd=$0
      if (cmd ~ /exp_nodump_refine/ || cmd ~ /stop_sim\.sh/ || cmd ~ /pgrep /) next
      c++
    }
    END { print c+0 }
  '
}

wait_held_pose() {
  local log="$1"
  local deadline=$((SECONDS + 120))
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

if pgrep -af 'ros2 launch luggage_gazebo' | grep -v -E 'pgrep|exp_nodump_refine' >/dev/null; then
  echo "EXP_FAIL sim already running" | tee "$EVID/RESULT.txt"
  pgrep -af 'ros2 launch luggage_gazebo' | tee -a "$EVID/RESULT.txt"
  exit 1
fi

"$ROOT/scripts/stop_sim.sh" >"$EVID/stop_sim_pre.log" 2>&1 || true
sleep 2

cd "$ROOT"
ros2 launch luggage_gazebo sim_world.launch.py \
  gui:=false use_rviz:=false use_semantic:=true use_motion:=true \
  use_vacuum:=true visual_kind:=mesh size_mode:=catalog \
  sequence_ids:=carryon,standard,large xy_jitter_range:=0.12,0.12 \
  yaw_range:=-0.6,0.6 observe_pose_name:=pickup_observe \
  >"$EVID/launch.log" 2>&1 &
echo $! > "$PIDFILE"

if ! wait_held_pose "$EVID/launch.log"; then
  echo "EXP_FAIL sim never ready" | tee "$EVID/RESULT.txt"
  "$ROOT/scripts/stop_sim.sh" >"$EVID/stop_sim_bootfail.log" 2>&1 || true
  exit 1
fi
sleep 8

python3 "$ROOT/src/luggage_perception/test/pf_r10_g6s_probe.py" \
  --out "$EVID" --duration 240 --stop-file "$EVID/STOP" \
  >"$EVID/g6s_probe.log" 2>&1 &
probe_pid=$!

date +%s > "$EVID/eval_start_epoch"
python3 "$ROOT/scripts/platform_free_height_gate4_eval.py" \
  --out "$EVID/gate4" --trials 6 --settle-sec 8.0 --warmup-frames 30 \
  --min-trials-per-size 2 \
  --launch-params "$LAUNCH_RECORD" \
  >"$EVID/gate4_stdout.log" 2>&1
gate_rc=$?
echo "$gate_rc" > "$EVID/gate4_rc"
date +%s > "$EVID/eval_end_epoch"
touch "$EVID/STOP"
wait "$probe_pid" || true
echo "$?" > "$EVID/probe_rc"

"$ROOT/scripts/stop_sim.sh" >"$EVID/stop_sim.log" 2>&1 || true
sleep 2
count_stack > "$EVID/residual_count"

echo "EXP_DONE gate_rc=$gate_rc residual=$(cat "$EVID/residual_count")" | tee "$EVID/RESULT.txt"
exit 0
