#!/usr/bin/env bash
# PF-R7 generation-3: G0-G3 once, then one bounded live G4/G6 campaign.
# Production nodes: /tmp/pfr10_g6 at 60dafb7. Harness: primary workspace.
set -euo pipefail

PRIMARY="/home/adamliao/work/elfin_humble_ws"
OVERLAY="/tmp/pfr10_g6"
OFFLINE_SRC="/tmp/pfr7_e2e"
EVAL_SHA="$(git -C "$PRIMARY" rev-parse HEAD)"
EV="$PRIMARY/docs/status/evidence/platform_free_height/2026-09-14_pfr7_g3/h1_${EVAL_SHA:0:12}"
OLD_EV="$PRIMARY/docs/status/evidence/platform_free_height/2026-09-14_pfr7_g3/sha_60dafb7"
export ROS_DOMAIN_ID=7
export ELFIN_SIM_PIDFILE=/tmp/elfin_humble_sim.pid
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1

mkdir -p "$EV/offline/pytest" "$EV/live"

cp -f "$PRIMARY/src/luggage_gazebo/scripts/pickup_box_spawner_node.py" \
  "$OVERLAY/src/luggage_gazebo/scripts/pickup_box_spawner_node.py"

source_overlay() {
  unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH || true
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "$OVERLAY/install/setup.bash"
  set -u
  export PYTHONPATH="$PRIMARY/src/luggage_perception:${PYTHONPATH:-}"
  export ROS_DOMAIN_ID=7
  export ELFIN_SIM_PIDFILE=/tmp/elfin_humble_sim.pid
}

{
  echo "evaluator_commit=$EVAL_SHA"
  echo "production_commit=60dafb7deee50a6f3a76d48076b743bf3e3e1bc8"
  echo "overlay=$(git -C "$OVERLAY" rev-parse HEAD)"
  echo "overlay_dirty=$(git -C "$OVERLAY" status --porcelain | wc -l)"
  echo "offline_src=$(git -C "$OFFLINE_SRC" rev-parse HEAD 2>/dev/null || echo missing)"
} | tee "$EV/revision.txt"

slot_busy() {
  pgrep -f '/usr/bin/python3 /opt/ros/humble/bin/ros2 launch luggage_gazebo' >/dev/null && return 0
  pgrep -f 'scripts/place_only_run.sh' >/dev/null && return 0
  pgrep -f 'place_only_eval_driver.py' >/dev/null && return 0
  return 1
}

echo "==== wait for exclusive sim slot ====" | tee "$EV/wait_slot.log"
echo "waiter started $(date -Iseconds)" | tee -a "$EV/wait_slot.log"
for i in $(seq 1 720); do
  if ! slot_busy; then
    echo "slot free at iter $i $(date -Iseconds)" | tee -a "$EV/wait_slot.log"
    break
  fi
  echo "busy iter=$i $(date +%H:%M:%S)" | tee -a "$EV/wait_slot.log"
  sleep 5
done
if slot_busy; then
  echo "sim still busy after wait; not taking the slot" | tee -a "$EV/wait_slot.log"
  exit 2
fi

# Recopy after wait so the overlay node matches the latest harness params.
cp -f "$PRIMARY/src/luggage_gazebo/scripts/pickup_box_spawner_node.py" \
  "$OVERLAY/src/luggage_gazebo/scripts/pickup_box_spawner_node.py"

echo "==== cleanup leftovers ====" | tee "$EV/cleanup.log"
"$PRIMARY/scripts/stop_sim.sh" | tee -a "$EV/cleanup.log" || true
sleep 5
pgrep -af '/usr/bin/python3 /opt/ros/humble/bin/ros2 launch luggage_gazebo' \
  | grep -v pgrep | tee "$EV/preflight_launch.txt" || true
pgrep -af '/opt/ros/humble/lib/ros_gz_bridge' | grep -v pgrep \
  | tee "$EV/preflight_bridge.txt" || true
if slot_busy; then
  echo "slot reoccupied during cleanup; abort" | tee -a "$EV/wait_slot.log"
  exit 2
fi

if grep -q 'offline_pass True' "$OLD_EV/offline/VERDICT.txt" 2>/dev/null; then
  echo "==== skip G0-G3 (already passed) ====" | tee "$EV/offline/commands.txt"
else
echo "==== G0-G3 / PF-A1 ====" | tee "$EV/offline/commands.txt"
source_overlay
export PYTHONPATH="$OFFLINE_SRC/src/luggage_perception:$OFFLINE_SRC/src/luggage_planning:$OFFLINE_SRC/src/luggage_packing:${PYTHONPATH:-}"
run_pytest() {
  local name="$1"
  shift
  set +e
  python3 -m pytest "$@" -q --tb=line \
    >"$EV/offline/pytest/${name}.stdout" 2>"$EV/offline/pytest/${name}.stderr"
  local rc=$?
  set -e
  echo "$rc" >"$EV/offline/pytest/${name}.rc"
  echo "pytest $name rc=$rc" | tee -a "$EV/offline/commands.txt"
}

run_pytest g0a "$OFFLINE_SRC/src/luggage_planning/test/test_pf_g0a_adapter_contract.py"
run_pytest g1 "$OFFLINE_SRC/src/luggage_perception/test/test_top_support_estimator.py"
run_pytest g2a "$OFFLINE_SRC/src/luggage_perception/test/test_pf_g2a_stamped_status_tf.py"
run_pytest g3a "$OFFLINE_SRC/src/luggage_perception/test/test_pf_g3a_raw_fail_closed.py"
run_pytest g4h "$OFFLINE_SRC/src/luggage_perception/test/test_pf_g4h_evaluator.py"
run_pytest a1 "$OFFLINE_SRC/src/luggage_perception/test/eval/test_pf_a1_static_audit.py"

python3 "$OFFLINE_SRC/scripts/platform_free_height_gate1_metrics.py" \
  --out "$EV/offline/gate1" | tee "$EV/offline/gate1_stdout.txt"

python3 - <<PY | tee "$EV/offline/VERDICT.txt"
import pathlib
ev = pathlib.Path("$EV/offline/pytest")
rcs = {p.stem: int(p.read_text()) for p in ev.glob("*.rc")}
print("pytest_rcs", rcs)
print("offline_pass", all(v == 0 for v in rcs.values()))
raise SystemExit(0 if all(v == 0 for v in rcs.values()) else 1)
PY
fi

echo "==== live campaign ====" | tee "$EV/live/start.txt"
source_overlay
export PYTHONPATH="$PRIMARY/src/luggage_perception:${PYTHONPATH:-}"
set +e
python3 "$PRIMARY/scripts/pf_r7_bounded_acceptance.py" \
  --live \
  --out "$EV/live" \
  --slots 3 --eligible-per-size 2 \
  --max-attempts-per-slot 12 --max-exclusions-per-slot 6 \
  --max-consecutive-size-exclusions 3 --max-stack-resets 2 \
  --max-attempts-campaign 36 --campaign-timeout-sec 2700 \
  --ros-domain-id 7 \
  --overlay "$OVERLAY" \
  --production-commit 60dafb7deee50a6f3a76d48076b743bf3e3e1bc8 \
  --git-commit "$EVAL_SHA" \
  --pidfile "$ELFIN_SIM_PIDFILE"
echo "live_exit=$?" | tee "$EV/live/exit.txt"
set -e
"$PRIMARY/scripts/stop_sim.sh" | tee "$EV/live/stop_sim.log" || true
pgrep -af '/usr/bin/python3 /opt/ros/humble/bin/ros2 launch luggage_gazebo' \
  | grep -v pgrep | tee "$EV/live/residual_launch.txt" || true
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv \
  | tee "$EV/live/gpu_after.csv" || true
