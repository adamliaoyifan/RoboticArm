#!/usr/bin/env bash
# ST09-I S5 M3 three-box runner: deployment scene, exclusive domain 7.
#
# Usage:
#   scripts/st09_i/s5_m3_run.sh --out DIR [--n 3] [--deadline-sec 1200]
#                              [--sequence carryon,standard,carryon]
#                              [--no-record-bag] [--keep-stack]
#
# Hard session deadline (default 20 min): the driver gets SIGINT, the bag
# stops, the already-atomic evidence stands, the stack tears down within
# 90 s, and the verdict records SESSION_DEADLINE — a stuck OMPL run or
# sensor flow never holds the Gazebo slot hostage. The output directory
# must NOT exist (no overwrite of a previous run's evidence).
set -uo pipefail

OUT=""
N=3
DEADLINE_SEC=1200
SEQUENCE="carryon,standard,carryon"
RECORD_BAG=1
KEEP_STACK=0
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --out) OUT="$2"; shift 2 ;;
    --n) N="$2"; shift 2 ;;
    --deadline-sec) DEADLINE_SEC="$2"; shift 2 ;;
    --sequence) SEQUENCE="$2"; shift 2 ;;
    --record-bag) RECORD_BAG=1; shift ;;
    --no-record-bag) RECORD_BAG=0; shift ;;
    --keep-stack) KEEP_STACK=1; shift ;;
    --ws) WS="$2"; shift 2 ;;
    *) echo "unknown arg $1" >&2; exit 2 ;;
  esac
done
[[ -n "$OUT" ]] || { echo "--out required" >&2; exit 2; }
# Never overwrite a previous run's evidence.
if [[ -e "$OUT" ]]; then
  echo "refusing existing output directory: $OUT" >&2
  exit 2
fi

log() { echo "{\"stage\": \"$1\", \"ts\": \"$(date -Is)\"$2}" >> "$OUT/lifecycle.jsonl"; }

unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH || true
set +u
source /opt/ros/humble/setup.bash
source "$WS/install-st09/setup.bash"
set -u
export PYTHONPATH="$WS/src/luggage_perception:$WS/src/luggage_planning:$WS/src/luggage_packing:$WS/src/luggage_gazebo:$WS/src/luggage_description:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=7 ROS_LOCALHOST_ONLY=1
export M3_SCENE_TF="$WS/install-st09/luggage_description/share/luggage_description/config/scene_tf_deploy.yaml"
export M3_ATLAS_DIR="$WS/src/luggage_planning/data/reachability_atlas_deploy_sim"
export ATLAS_RUNTIME_CONFIG="$WS/install-st09/luggage_planning/share/luggage_planning/config/atlas_runtime_deploy_sim.yaml"
POSES="$WS/install-st09/luggage_description/share/luggage_description/config/robot_poses.sim_deploy.yaml"
mkdir -p "$OUT"

# -- exclusive slot ------------------------------------------------------
bash "$WS/scripts/st09_i/kill_stack.sh" || true
residuals=$(pgrep -af 'ros2 launch luggage_gazebo|ign gazebo|gz sim|ros_gz_bridge/parameter_bridge|clock_bridge|camera_bridge' 2>/dev/null | grep -v -e 'bash -c' -e 's5_m3_run.sh' | wc -l | tr -d ' ')
log clean_room ", \"residual_processes\": $residuals"
if [[ "$residuals" -ne 0 ]]; then
  echo "residual sim/bridge processes ($residuals); refusing to launch" >&2
  exit 1
fi

# -- stack (deployment scene, headless, typed chain on) ------------------
LAUNCH_LOG="$OUT/launch.log"
ros2 launch luggage_gazebo sim_world.launch.py \
  scene_tf_config:="$M3_SCENE_TF" \
  robot_poses_config:="$POSES" \
  observe_pose_name:=pickup_observe_sim \
  gui:=false use_rviz:=false \
  use_vacuum:=true use_motion:=true use_semantic:=true \
  frame_runtime:=true \
  atlas_runtime_config:="$ATLAS_RUNTIME_CONFIG" \
  sequence_ids:="$SEQUENCE" \
  > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
log stack_start ", \"launch_pid\": $LAUNCH_PID"

# -- bag (small topic set; volumes are NOT a cap — verified post-run) -----
BAG_PID=""
if [[ "$RECORD_BAG" -eq 1 ]]; then
  ros2 bag record -o "$OUT/bag" --max-bag-size 200000000 \
    /clock /tf /joint_states \
    /vacuum/state /vacuum/events_json \
    /luggage/current_box /luggage/finalized_models \
    /luggage/perception/size_eval/spawned \
    /luggage/cargo_map/surface_2d /luggage/cargo_map/surface_v2 \
    /luggage/semantic/cargo_points_untracked \
    /motion_planner/last_boundary \
    > "$OUT/bag_record.log" 2>&1 &
  BAG_PID=$!
  log bag_start ", \"bag_pid\": $BAG_PID"
fi

stop_bag() {
  if [[ -n "$BAG_PID" ]]; then
    kill -INT "$BAG_PID" 2>/dev/null || true
    sleep 2
    kill -TERM "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
    BAG_PID=""
    log bag_stop ""
  fi
}

# -- driver under the hard deadline ---------------------------------------
DRIVER_RC=255
DEADLINE_HIT=0
start_s=$SECONDS
python3 "$WS/src/luggage_gazebo/scripts/m3_three_box_driver.py" \
  --out "$OUT" --n "$N" --m3-sequence "$SEQUENCE" \
  --rehearsal "$OUT/rehearsal" \
  --frame-runtime --frame-peer motion_occ_driver \
  --observe-pose pickup_observe_sim \
  --payload vacuum --place-probe on --goto-timeout 60 \
  > "$OUT/driver.log" 2>&1 &
DRIVER_PID=$!
log driver_start ", \"driver_pid\": $DRIVER_PID, \"deadline_sec\": $DEADLINE_SEC"

while kill -0 "$DRIVER_PID" 2>/dev/null; do
  if (( SECONDS - start_s >= DEADLINE_SEC )); then
    DEADLINE_HIT=1
    log deadline_hit ", \"elapsed_s\": $((SECONDS - start_s))"
    kill -INT "$DRIVER_PID" 2>/dev/null || true
    grace=0
    while kill -0 "$DRIVER_PID" 2>/dev/null && (( grace < 30 )); do
      sleep 2; grace=$((grace + 2))
    done
    kill -TERM "$DRIVER_PID" 2>/dev/null || true
    break
  fi
  sleep 5
done
wait "$DRIVER_PID" 2>/dev/null || true
DRIVER_RC=$?
log driver_end ", \"rc\": $DRIVER_RC, \"deadline\": $DEADLINE_HIT, \"elapsed_s\": $((SECONDS - start_s))"

stop_bag

# -- teardown within 90 s --------------------------------------------------
if [[ "$KEEP_STACK" -eq 0 ]]; then
  teardown_start=$SECONDS
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  grace=0
  while kill -0 "$LAUNCH_PID" 2>/dev/null && (( grace < 60 )); do
    sleep 3; grace=$((grace + 3))
  done
  if kill -0 "$LAUNCH_PID" 2>/dev/null; then
    bash "$WS/scripts/st09_i/kill_stack.sh" || true
  fi
  wait "$LAUNCH_PID" 2>/dev/null || true
  log stack_stop ", \"teardown_s\": $((SECONDS - teardown_start))"
fi

# -- post-run size verification (bag splitting is not a cap) ----------------
bag_bytes=$(du -sb "$OUT/bag" 2>/dev/null | cut -f1 || echo 0)
if [[ "$RECORD_BAG" -eq 1 ]]; then
  if (( bag_bytes > 1073741824 )); then
    log bag_size_exceeded ", \"bytes\": $bag_bytes"
    echo "bag exceeds 1 GiB: $bag_bytes bytes" >&2
  else
    log bag_size_ok ", \"bytes\": $bag_bytes"
  fi
fi

log end ", \"driver_rc\": $DRIVER_RC"
exit "$DRIVER_RC"
