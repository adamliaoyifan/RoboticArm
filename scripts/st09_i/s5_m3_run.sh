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
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
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
# NO src-tree PYTHONPATH: the whole session (stack AND driver) must
# resolve from the install overlay — a src-resolved module reports
# src-tree provenance and every peer's readiness gate rejects it
# (WRONG_OVERLAY, live in smoke03/smoke04).
export ROS_DOMAIN_ID=7 ROS_LOCALHOST_ONLY=1
# Opt-in: ST09_I_CERT_CAPTURE=1 writes each ValidateRouteV2 request and
# response under the run directory. The selector inherits the directory.
if [[ "${ST09_I_CERT_CAPTURE:-0}" == "1" ]]; then
  export ST09_I_CERT_CAPTURE_DIR="$OUT/cert_capture"
  mkdir -p "$ST09_I_CERT_CAPTURE_DIR"
fi
# [ST09-I S5.7-6] semantic assets resolve from the INSTALL overlay
# (colcon-installed package share) — formal runs must not depend on the
# main workspace, the source tree, or the user cache. The launch's own
# empty-flag default resolves the same share; passing it explicitly
# keeps the runner's fail-closed pre-checks meaningful.
M3_PERCEPTION_SHARE="$WS/install-st09/luggage_perception/share/luggage_perception"
M3_CLIP_VENDOR="${M3_CLIP_VENDOR:-$M3_PERCEPTION_SHARE/vendor}"
if [[ ! -d "$M3_CLIP_VENDOR/clip_pkg" ]]; then
  echo "CLIP vendor incomplete at $M3_CLIP_VENDOR (need clip_pkg/ in the install overlay; rebuild luggage_perception)" >&2
  exit 1
fi
M3_YOLO_WEIGHTS="${M3_YOLO_WEIGHTS:-$M3_PERCEPTION_SHARE/models/yolov8s-world.pt}"
if [[ ! -f "$M3_YOLO_WEIGHTS" ]]; then
  echo "YOLO-World weights missing at $M3_YOLO_WEIGHTS (install overlay models/; rebuild luggage_perception)" >&2
  exit 1
fi
# [ST09-I S5.7-6] the backend's interpreter deps come from the
# candidate-owned frozen prefix — NEVER the executing user's site.
M3_PYDEPS="${M3_PYDEPS:-$WS/pydeps-frozen}"
if [[ ! -f "$M3_PYDEPS/DEPLOYED.json" ]]; then
  echo "frozen python deps prefix absent at $M3_PYDEPS (scripts/st09_i/deploy_python_deps.sh; the user site is not an allowed source)" >&2
  exit 1
fi
# Hash-record the frozen semantic assets into the lifecycle log (the
# acceptance manifest carries the same values).
weights_sha=$(sha256sum "$M3_YOLO_WEIGHTS" | cut -d' ' -f1)
vendor_sha=$(cd "$M3_CLIP_VENDOR" && LC_ALL=C find clip_pkg deps clip_models -type f \
  | LC_ALL=C sort | xargs sha256sum | sha256sum | cut -d' ' -f1)
mkdir -p "$OUT"
pydeps_dists=$(python3 -c "import json;print(len(json.load(open('$M3_PYDEPS/DEPLOYED.json'))['distributions']))")
log semantic_assets ", \"vendor\": \"$M3_CLIP_VENDOR\", \"vendor_tree_sha256\": \"$vendor_sha\", \"weights\": \"$M3_YOLO_WEIGHTS\", \"weights_sha256\": \"$weights_sha\", \"pydeps\": \"$M3_PYDEPS\", \"pydeps_dists\": $pydeps_dists"
export M3_REPO="$WS"
export M3_SCENE_TF="$WS/install-st09/luggage_description/share/luggage_description/config/scene_tf_deploy.yaml"
export M3_ATLAS_DIR="$WS/src/luggage_planning/data/reachability_atlas_deploy_sim"
export ATLAS_RUNTIME_CONFIG="$WS/install-st09/luggage_planning/share/luggage_planning/config/atlas_runtime_deploy_sim.yaml"
POSES="$WS/install-st09/luggage_description/share/luggage_description/config/robot_poses.sim_deploy.yaml"
mkdir -p "$OUT"

# -- exclusive slot ------------------------------------------------------
bash "$WS/scripts/st09_i/kill_stack.sh" || true
residuals=$(pgrep -af 'ros2 launch luggage_gazebo|ign gazebo|gz sim|ros_gz_bridge/parameter_bridge|clock_bridge|camera_bridge' 2>/dev/null | grep -v -e 'bash -c' -e 's5_m3_run.sh' | wc -l | tr -d ' ')
log clean_room ", \"residual_processes\": $residuals"
export ELFIN_AMBIGUITY_CAPTURE="$OUT/ambiguity"
if [[ "${M3_CONTACT_CAPTURE:-0}" == "1" ]]; then
  # Observation paths only. Set before launch so the spawner writes the
  # contact sensor into the carry SDF and the vacuum node logs set_pose.
  mkdir -p "$OUT/collision_models" "$OUT/contact_capture"
  : > "$OUT/follow_trace.jsonl"
  export ELFIN_BOX_CONTACT_TOPIC=/pickup_box_contact
  export ELFIN_FOLLOW_TRACE="$OUT/follow_trace.jsonl"
  export ELFIN_BOX_SDF_DUMP="$OUT/collision_models/pickup_box.sdf"
  export ELFIN_BOX_RELEASE_SDF="$OUT/collision_models/pickup_box_release.sdf"
  export ELFIN_ROBOT_URDF=/tmp/elfin_s20_gazebo.urdf
  export ELFIN_CONTACT_STATUS="$OUT/contact_capture/recorder_status.json"
  export ELFIN_ST09_SCRIPTS="$WS/scripts/st09_i"
fi
# One collision, Gazebo spawn file only. Unset means the full panel stays.
if [[ -n "${M3_STRIP_PANEL_COLLISION:-}" ]]; then
  export ELFIN_GZ_STRIP_PANEL_COLLISION="$M3_STRIP_PANEL_COLLISION"
  echo "{\"strip_mesh\": \"$M3_STRIP_PANEL_COLLISION\"}" > "$OUT/panel_strip.json"
fi
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
  use_cargo_map:=true use_packing:=true \
  frame_runtime:=true \
  semantic_require_backend:=bbox_fill \
  semantic_clip_vendor_dir:="${M3_CLIP_VENDOR}" \
  semantic_model_name:="${M3_YOLO_WEIGHTS}" \
  atlas_runtime_config:="$ATLAS_RUNTIME_CONFIG" \
  corridor_planner:=lattice \
  box_count:="$N" \
  sequence_ids:="$SEQUENCE" \
  require_confirmed_map:="${M3_REQUIRE_CONFIRMED_MAP:-false}" \
  > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
log stack_start ", \"launch_pid\": $LAUNCH_PID"
CONTACT_PID=""
if [[ "${M3_CONTACT_CAPTURE:-0}" == "1" ]]; then
  python3 "$WS/scripts/st09_i/contact_pose_record.py" \
    --out-dir "$OUT/contact_capture" \
    > "$OUT/contact_capture/recorder.log" 2>&1 &
  CONTACT_PID=$!
  log contact_record_start ", \"pid\": $CONTACT_PID"
fi

# -- bag: light topics plus 3 s copies of the fat sensor streams ----------
# Full-rate color/depth/mask exceeded 1 GiB in 51 s. The throttle keeps a
# 1200 s session under that cap. Do not delete an earlier oversized bag.
BAG_PID=""
THROTTLE_PID=""
if [[ "$RECORD_BAG" -eq 1 ]]; then
  if ! BUDGET_JSON="$(PYTHONPATH="$WS/src/luggage_gazebo${PYTHONPATH:+:$PYTHONPATH}" \
    python3 -c 'import json; from luggage_gazebo.capture_budget import THROTTLE_PERIOD_S, bag_topics, worst_case_bytes, RUN_BUDGET_BYTES; total=worst_case_bytes(); assert total < RUN_BUDGET_BYTES, total; print(json.dumps({"topics": bag_topics(), "period_s": THROTTLE_PERIOD_S, "worst_case_bytes": total}))')"; then
    echo "capture budget refuses this topic set" >&2
    exit 2
  fi
  WORST_BYTES="$(python3 -c 'import json,sys; print(json.loads(sys.argv[1])["worst_case_bytes"])' "$BUDGET_JSON")"
  PERIOD_S="$(python3 -c 'import json,sys; print(json.loads(sys.argv[1])["period_s"])' "$BUDGET_JSON")"
  mapfile -t BAG_TOPICS < <(python3 -c 'import json,sys; print("\n".join(json.loads(sys.argv[1])["topics"]))' "$BUDGET_JSON")
  python3 "$WS/scripts/st09_i/throttle_capture_topics.py" \
    > "$OUT/capture_throttle.log" 2>&1 &
  THROTTLE_PID=$!
  ros2 bag record -o "$OUT/bag" --max-bag-size 200000000 \
    "${BAG_TOPICS[@]}" \
    > "$OUT/bag_record.log" 2>&1 &
  BAG_PID=$!
  log bag_start ", \"bag_pid\": $BAG_PID, \"throttle_pid\": $THROTTLE_PID, \"throttle_period_s\": $PERIOD_S, \"worst_case_bytes\": $WORST_BYTES"
fi

stop_bag() {
  if [[ -n "${CONTACT_PID:-}" ]]; then
    kill -INT "$CONTACT_PID" 2>/dev/null || true
    wait "$CONTACT_PID" 2>/dev/null || true
    CONTACT_PID=""
    log contact_record_stop ""
  fi
  if [[ -n "$BAG_PID" ]]; then
    kill -INT "$BAG_PID" 2>/dev/null || true
    sleep 2
    kill -TERM "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
    BAG_PID=""
    log bag_stop ""
  fi
  if [[ -n "$THROTTLE_PID" ]]; then
    kill -INT "$THROTTLE_PID" 2>/dev/null || true
    wait "$THROTTLE_PID" 2>/dev/null || true
    THROTTLE_PID=""
  fi
}

# -- pre-warm: the semantic chain must be producing decisions before the
#    driver starts (first start AutoUpdates CLIP and can take minutes;
#    YOLO_NOT_READY at spawn time was the smoke07 failure).
seg_ready=0
for attempt in $(seq 1 100); do
  stats=$(timeout 5 ros2 topic echo /semantic_segmenter/stats_json --once --no-arr 2>/dev/null | head -40)
  if echo "$stats" | grep -q "data"; then
    seg_ready=1
    break
  fi
  sleep 3
done
log semantic_prewarm ", \"ready\": $seg_ready, \"waited_attempt\": $attempt"
# [ST09-I S5.7-6] the runtime-reported load paths must sit inside the
# install overlay (frozen-asset isolation; a src/ or main-workspace
# path here means the launch flag wiring drifted).
if [[ "$seg_ready" -eq 1 ]]; then
  if echo "$stats" | grep -q "clip_vendor\|model_path" \
     && ! echo "$stats" | grep -q "install-st09"; then
    echo "semantic assets resolved OUTSIDE the install overlay: $stats" >&2
    stop_bag
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    sleep 5
    bash "$WS/scripts/st09_i/kill_stack.sh" || true
    exit 4
  fi
  prov=$(echo "$stats" | grep -o '[^ "]*install-st09[^ "]*' | sort -u \
    | head -4 | tr '\n' ' ')
  log semantic_provenance ", \"overlay_paths\": \"$prov\""
fi
if [[ "$seg_ready" -ne 1 ]]; then
  echo "semantic segmenter never published stats; refusing to start driver" >&2
  stop_bag
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  sleep 5
  bash "$WS/scripts/st09_i/kill_stack.sh" || true
  exit 3
fi

# -- layout gate: the live container TF must be the deployment scene's ----
# (the driver gets the same file via --scene-tf-config). A mismatch costs
# no case: stop before the driver starts.
layout_json=$(python3 "$WS/scripts/st09_i/check_live_layout.py" --scene "$M3_SCENE_TF" --timeout 30)
layout_rc=$?
echo "$layout_json" > "$OUT/live_layout.json"
log layout_check ", \"rc\": $layout_rc, \"require_confirmed_map\": \"${M3_REQUIRE_CONFIRMED_MAP:-false}\""
if [[ "$layout_rc" -ne 0 ]]; then
  echo "live container layout differs from $M3_SCENE_TF: $layout_json" >&2
  stop_bag
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  sleep 5
  bash "$WS/scripts/st09_i/kill_stack.sh" || true
  exit 5
fi

# -- capture preflight (opt-in: M3_CAPTURE_PREFLIGHT=1) ---------------------
# Refuse to enter motion unless controller_state (reference, feedback, six
# joint names) and /joint_states are both really arriving, and the bag is
# subscribed to both. Content, not topic existence. Failing here costs no case.
if [[ "${M3_CAPTURE_PREFLIGHT:-0}" == "1" ]]; then
  pre_json=$(python3 "$WS/scripts/st09_i/capture_preflight.py" --window-sec 3 --timeout-sec 30)
  pre_rc=$?
  echo "$pre_json" > "$OUT/capture_preflight.json"
  rec_ok=1
  if [[ "$RECORD_BAG" -eq 1 ]]; then
    for topic in /elfin_arm_controller/controller_state /joint_states; do
      # Capture first: `ros2 ... | grep -q` under pipefail reports failure
      # (SIGPIPE) even when the recorder IS subscribed.
      topic_info=$(timeout 15 ros2 topic info -v "$topic" 2>/dev/null || true)
      grep -q rosbag2_recorder <<<"$topic_info" || rec_ok=0
    done
  fi
  log capture_preflight ", \"rc\": $pre_rc, \"bag_subscribed\": $rec_ok"
  if [[ "$pre_rc" -ne 0 || "$rec_ok" -ne 1 ]]; then
    echo "capture preflight failed (rc=$pre_rc bag_subscribed=$rec_ok): $pre_json" >&2
    stop_bag
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    sleep 5
    bash "$WS/scripts/st09_i/kill_stack.sh" || true
    exit 6
  fi
fi

# -- driver under the hard deadline ---------------------------------------
# The mapper gate and the driver's commit go together: with the gate on, each
# released box is committed to the mapper ledger and confirmed by evidence.
COMMIT_ARGS=()
if [[ "${M3_REQUIRE_CONFIRMED_MAP:-false}" == "true" ]]; then
  COMMIT_ARGS=(--commit-placed-box)
fi
DRIVER_RC=255
DEADLINE_HIT=0
start_s=$SECONDS
# Diagnostic only: on SIGABRT at exit (rc 134 seen once) the Python stack of
# the aborting thread is printed to driver.log.
export PYTHONFAULTHANDLER=1
python3 "$WS/install-st09/luggage_gazebo/lib/luggage_gazebo/m3_three_box_driver.py" \
  --out "$OUT" --n "$N" --m3-sequence "$SEQUENCE" \
  --scene-tf-config "$M3_SCENE_TF" \
  --rehearsal "$OUT/rehearsal" \
  --frame-runtime --frame-peer motion_occ_driver \
  --observe-pose pickup_observe_sim \
  --payload vacuum --place-probe on --goto-timeout 60 --geometry-timeout 90 \
  "${COMMIT_ARGS[@]}" \
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
wait "$DRIVER_PID" 2>/dev/null
DRIVER_RC=$?
log driver_end ", \"rc\": $DRIVER_RC, \"deadline\": $DEADLINE_HIT, \"elapsed_s\": $((SECONDS - start_s))"
# A driver that finished its work but died on a signal is NOT a normal exit.
# Keep what is needed to diagnose it if it recurs.
if (( DRIVER_RC >= 128 )); then
  log driver_abnormal_exit ", \"rc\": $DRIVER_RC, \"signal\": $((DRIVER_RC - 128))"
  {
    echo "{\"rc\": $DRIVER_RC, \"signal\": $((DRIVER_RC - 128)),"
    echo " \"core_pattern\": \"$(cat /proc/sys/kernel/core_pattern 2>/dev/null)\","
    echo " \"ulimit_core\": \"$(ulimit -c)\","
    echo " \"driver_log_tail\": $(tail -n 60 "$OUT/driver.log" | python3 -c 'import json,sys; print(json.dumps(sys.stdin.read()))')}"
  } > "$OUT/driver_exit_diagnostics.json"
fi

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

if [[ "${M3_CONTACT_CAPTURE:-0}" == "1" && -f /tmp/elfin_s20_gazebo.urdf ]]; then
  cp /tmp/elfin_s20_gazebo.urdf "$OUT/collision_models/robot.urdf"
fi
if [[ "${M3_CONTACT_CAPTURE:-0}" == "1" && -f /tmp/elfin_s20_gazebo.sdf ]]; then
  cp /tmp/elfin_s20_gazebo.sdf "$OUT/collision_models/robot.sdf"
fi
log end ", \"driver_rc\": $DRIVER_RC"
exit "$DRIVER_RC"
