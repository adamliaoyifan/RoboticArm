#!/usr/bin/env bash
# Release-window observation run (non-scoring, user-sanctioned 2026-10-17).
#
# One rehearsal box with SYNCHRONIZED SIM-CLOCK capture around the
# vacuum-off -> static->dynamic swap -> fall -> settle window:
#   pose_stream.txt      gz /world/<w>/pose/info + dynamic_pose/info
#                        (protobuf text, sim-stamped, per-step for
#                        dynamic bodies) — box pose/attitude per physics
#                        step; static-body last pose and dynamic-body
#                        spawn pose are both in this one stream.
#   contact_stream.txt   gz contact sensor on the RELEASED box
#                        (ELFIN_BOX_CONTACT_TOPIC) — contact parties,
#                        time, depth/force where the sensor provides it.
#   tool_stream.jsonl    /joint_states + /tf (sim-stamped headers) for
#                        the tool side.
# Driver-side event timings (vacuum off, delete/spawn, retreat start,
# verify poses) land in the trial dump as usual — all stamps are
# simulation time.
#
# usage: observe_release.sh <run-name>
set -euo pipefail
NAME="${1:?run name}"
WS="/tmp/elfin-master-pack-baseline-g3"
MAIN="/home/adamliao/work/elfin_humble_ws"
EVID="$MAIN/docs/status/evidence/master_pack_recovery/2026-10-14_g3"
OUT="$EVID/$NAME"
WORLD="airport_loading"
BOX_MODEL_PREFIX="pickup_box"
[[ -e "$OUT" ]] && { echo "run dir exists: $OUT" >&2; exit 2; }
mkdir -p "$OUT"
export ROS_DOMAIN_ID=7

set +u
source /opt/ros/humble/setup.bash
source "$MAIN/install/setup.bash"
source "$WS/install/setup.bash"
set -u

# Queue behind any live stack, then clean-room.
if pgrep -f 'ros2 launch luggage_gazebo sim_world' >/dev/null 2>&1; then
  echo "sim slot busy" >&2; exit 2
fi
bash "$WS/scripts/stop_sim.sh" >/dev/null 2>&1 || true

IGN_BIN="$(command -v ign || command -v gz)"
capture() {  # capture <gz-topic> <outfile>
  ( exec setsid "$IGN_BIN" topic -e -t "$1" > "$2" 2>/dev/null & )
}
cleanup() {
  local status=$?
  for pid in "${CAPTURE_PIDS[@]:-}"; do
    [[ -n "$pid" ]] && kill -TERM -"$pid" 2>/dev/null || true
  done
  pkill -TERM -f 'ros2 launch luggage_gazebo sim_world' 2>/dev/null || true
  sleep 8
  pkill -KILL -f 'ign gazebo|gz sim' 2>/dev/null || true
  bash "$WS/scripts/stop_sim.sh" >/dev/null 2>&1 || true
  exit $status
}
trap cleanup EXIT INT TERM

# Launch the proven stack (rehearsal wiring).
(
  ros2 launch luggage_gazebo sim_world.launch.py \
    gui:=false use_rviz:=false \
    use_semantic:=true use_motion:=true use_vacuum:=true \
    use_cargo_map:=true use_packing:=true \
    visual_kind:=mesh size_mode:=catalog \
    sequence_ids:=inch20 yaw_mode:=discrete random_seed:=0 \
    observe_pose_name:=pickup_observe_sim \
    frame_runtime:=true frame_driver_peer:=pack_eval_driver \
    semantic_require_backend:=bbox_fill \
    semantic_model_name:="$MAIN/yolov8s-world.pt" \
    semantic_clip_vendor_dir:="$MAIN/src/luggage_perception/vendor" \
    semantic_pydeps_dir:="$MAIN/pydeps-frozen" \
    scene_tf_config:="$WS/src/luggage_description/config/scene_tf_deploy_update.yaml" \
    atlas_runtime_config:="$WS/src/luggage_planning/config/atlas_runtime_deploy_update.yaml" \
    robot_poses_config:="$WS/src/luggage_description/config/robot_poses.baseline_update.yaml" \
    > "$OUT/launch.log" 2>&1 &
)
sleep 2
LAUNCH_PID="$(pgrep -f 'ros2 launch luggage_gazebo sim_world' | head -1)"
[[ -n "$LAUNCH_PID" ]] || { echo "launch failed" >&2; exit 1; }

# Released box publishes its contacts (sdf contact sensor hook).
export ELFIN_BOX_CONTACT_TOPIC="/obs/box_contact"
export ELFIN_BOX_SDF_DUMP="$OUT/sdf/spawn.sdf"
export ELFIN_BOX_RELEASE_SDF="$OUT/sdf/release.sdf"
export ST09_SELECT_REQUEST_CAPTURE_DIR="$OUT/select_capture"
export MOTION_BOUNDARY_DUMP="$OUT/dumps/boundaries"
mkdir -p "$OUT/sdf" "$ST09_SELECT_REQUEST_CAPTURE_DIR" "$OUT/dumps/boundaries"

# Wait for the sim to be publishing, then start the synchronized captures.
# OBS_NO_CAPTURE=1 skips them (bisect control: capture load vs code path).
if [[ -z "${OBS_NO_CAPTURE:-}" ]]; then
  for _ in $(seq 1 60); do
    "$IGN_BIN" topic -l 2>/dev/null | grep -q "pose/info" && break
    sleep 2
  done
  CAPTURE_PIDS=()
  capture "/world/$WORLD/pose/info" "$OUT/pose_stream.txt"
  capture "/world/$WORLD/dynamic_pose/info" "$OUT/dyn_pose_stream.txt"
  capture "/obs/box_contact" "$OUT/contact_stream.txt"
  sleep 2
  python3 "$WS/scripts/g3/record_tool_stream.py" "$OUT/tool_stream.jsonl" &
  CAPTURE_PIDS+=("$!")
else
  CAPTURE_PIDS=()
  echo "captures disabled (OBS_NO_CAPTURE)"
fi

# Readiness: the stack must hold the observe pose first (the proven
# gate; cold-boot controller races retry by rerunning this script).
ready=0
for _ in $(seq 1 40); do
  if grep -q "startup aborted\|startup_failed\|plugin_urdf_not_received\|Caught exception in launch" "$OUT/launch.log"; then
    echo "startup failed" >&2; exit 1
  fi
  if grep -q "Held pose 'pickup_observe_sim'" "$OUT/launch.log"; then
    ready=1; break
  fi
  sleep 3
done
if [[ "$ready" -ne 1 ]]; then
  echo "observe pose was not held within 120 s" >&2; exit 1
fi

# One rehearsal box through the full release window (motion; the
# observation run was user-sanctioned 2026-10-17).
set +e
timeout --signal=INT 2400 ros2 run luggage_gazebo pack_eval_driver.py \
  --frame-runtime --frame-peer pack_eval_driver \
  --observe-pose pickup_observe_sim \
  --sequence-ids inch20 \
  --max-boxes 1 \
  --goto-timeout 60 \
  --plan-timeout 180 \
  --seed 0 \
  --auth-max-attempts 8 \
  --auth-max-elapsed-sec 8 \
  --stop-on-first-failure \
  --scene-tf-config "$WS/src/luggage_description/config/scene_tf_deploy_update.yaml" \
  --out "$OUT" > "$OUT/driver.log" 2>&1
driver_rc=$?
set -e
echo "{\"stage\":\"driver_exit\",\"rc\":$driver_rc,\"ts\":\"$(date -Is)\"}" \
  >> "$OUT/lifecycle.jsonl"

wc -c "$OUT/pose_stream.txt" "$OUT/dyn_pose_stream.txt" \
     "$OUT/contact_stream.txt" "$OUT/tool_stream.jsonl" || true
