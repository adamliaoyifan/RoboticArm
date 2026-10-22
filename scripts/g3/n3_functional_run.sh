#!/usr/bin/env bash
# [Phase-2 functional n=3, 2026-10-22] ONE n=3 run under the SAME
# functional-diagnostic config as phase 1 (180 s per-Select budget
# reaching the searcher). Frozen version = post-fix tree
# (ac2a92d3: driver --scene-tf-config, plugin param-delivery 0.8).
# Stack: production (diagnostic_capture:=false — no capture routing
# hops), joint_capture window kept (plugin-side only, +1.3% measured).
# No skipped boxes, no manual placement, no lowered gates; prior OFF
# (reported separately as NOT adopted). Pass = "relaxed-budget n=3
# functional closed-loop pass" ONLY.
set -o pipefail
export AMENT_TRACE_SETUP_FILES=0

WS="/tmp/elfin-master-pack-baseline-g3"
MAIN="/home/adamliao/work/elfin_humble_ws"
OUT="/tmp/jcc_runs/n3_functional_$(date +%Y%m%d_%H%M%S)"
RUN_ID="n3func_$(date +%H%M%S)"
DEADLINE=$(( $(date +%s) + 5400 ))
DIAG_ROUTE_BUDGET_S=180
mkdir -p "$OUT"
export ROS_DOMAIN_ID=7

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$OUT/runner.log"; }

# ---- Boot discipline (verbatim from transit_diag_run.sh) ----
bash "$WS/scripts/stop_sim.sh" --all >> "$OUT/bootsweep.log" 2>&1 || true
if ! pgrep -f 'ros2 launc[h]' >/dev/null 2>&1; then
  pkill -TERM -f 'opt/ros/humble/li[b]' 2>/dev/null || true
  pkill -TERM -f 'install/luggage_.*/li[b]' 2>/dev/null || true
  sleep 4
  pkill -KILL -f 'opt/ros/humble/li[b]' 2>/dev/null || true
  pkill -KILL -f 'install/luggage_.*/li[b]' 2>/dev/null || true
  N_SHM=$(ls /dev/shm 2>/dev/null | grep -c '^fastrtps' || true)
  if [[ "${N_SHM:-0}" -gt 0 ]]; then
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
  fi
fi
if pgrep -f 'ros2 launch luggage_gazebo sim_world.launch.py' >/dev/null 2>&1; then
  log "sim slot busy"; exit 2
fi

# ---- Same A-prime scene rebind + driver scene arg (phase-1 fix) ----
export G3_CONTAINER_MODEL="airport_container_real_aprime"
SCENE="$OUT/scene_tf_used.yaml"
POSES_OUT="$OUT/robot_poses_used.yaml"
python3 - "$SCENE" "$POSES_OUT" << 'PYSCENE'
import hashlib, os, sys, yaml
cfg = yaml.safe_load(open(
    "/tmp/elfin-master-pack-baseline-g3/src/luggage_description/config/"
    "scene_tf_deploy_update.yaml"))
cfg.setdefault("gazebo", {})["container_model"] = \
    os.environ["G3_CONTAINER_MODEL"]
yaml.safe_dump(cfg, open(sys.argv[1], "w"), sort_keys=False)
scene_md5 = hashlib.md5(open(sys.argv[1], "rb").read()).hexdigest()
poses = yaml.safe_load(open(
    "/tmp/elfin-master-pack-baseline-g3/src/luggage_description/config/"
    "robot_poses.baseline_update.yaml"))
poses.setdefault("identity", {})["scene_revision"] = scene_md5
poses["identity"]["_rebind_note"] = (
    "scene_revision re-bound for n3_functional boot (md5 %s); joint values "
    "unchanged" % scene_md5)
for _name, pose in (poses.get("poses") or {}).items():
    if isinstance(pose, dict) and pose.get("scene_revision_at_capture"):
        pose["scene_revision_at_capture"] = scene_md5
yaml.safe_dump(poses, open(sys.argv[2], "w"), sort_keys=False)
print("scene_md5", scene_md5)
PYSCENE

WEIGHTS="$MAIN/yolov8s-world.pt"
VENDOR="$MAIN/src/luggage_perception/vendor"
PYDEPS="$MAIN/pydeps-frozen"

set +u
source /opt/ros/humble/setup.bash
source "$MAIN/install/setup.bash"
source "$WS/install/setup.bash"
set -u
# No PYTHONPATH src prefixes — driver identity must report INSTALL overlay
# paths (src prefixes triggered WRONG_OVERLAY in the 10-22 attempts).

(ros2 launch luggage_gazebo sim_world.launch.py \
    gui:=false use_rviz:=false \
    use_semantic:=true use_motion:=true use_vacuum:=true \
    use_cargo_map:=true use_packing:=true \
    visual_kind:=mesh size_mode:=catalog \
    sequence_ids:=inch20 \
    yaw_mode:=discrete \
    random_seed:=0 \
    observe_pose_name:=pickup_observe_sim \
    frame_runtime:=true frame_driver_peer:=pack_eval_driver \
    semantic_require_backend:=bbox_fill \
    semantic_model_name:="$WEIGHTS" \
    semantic_clip_vendor_dir:="$VENDOR" \
    semantic_pydeps_dir:="$PYDEPS" \
    scene_tf_config:="$SCENE" \
    start_placement_prior:=false \
    robot_poses_config:="$POSES_OUT" \
    diagnostic_route_budget_s:=$DIAG_ROUTE_BUDGET_S \
    joint_capture_mode:=window joint_capture_run_dir:="$OUT/joint_capture" \
    > "$OUT/launch.log" 2>&1 &)
sleep 2
LPID="$(pgrep -f 'ros2 launch luggage_gazebo sim_world.launch.py' | head -1 || true)"
if [[ -z "$LPID" ]]; then log "launch process missing"; exit 1; fi
log "launch pid=$LPID out=$OUT run_id=$RUN_ID"

# ---- Fix-verification gate: the plugin MUST report gain 0.8 now ----
GAIN_OK=""
for _ in $(seq 1 60); do
  sleep 5
  G=$(grep -m1 "position_proportional_gain has been set to:" "$OUT/launch.log" 2>/dev/null | sed 's/.*set to: //')
  if [[ -n "$G" ]]; then
    if [[ "$G" == "0.8" ]]; then GAIN_OK=1; else log "GAIN_VERIFY_FAIL runtime=$G (expected 0.8)"; fi
    break
  fi
  [ "$(date +%s)" -gt "$DEADLINE" ] && break
done
if [[ -z "$GAIN_OK" ]]; then
  log "aborting before any motion: param-delivery fix not active"
  kill -INT "$LPID" 2>/dev/null || true; sleep 5
  pkill -TERM -f "ign gazebo" 2>/dev/null || true; sleep 3
  pkill -9 -f "ign gazebo" 2>/dev/null || true
  exit 5
fi
log "gain verify OK (0.8)"

# ---- Driver: production n=3 + functional-diagnostic budget ----
export JOINT_CAPTURE_DIR="$OUT/joint_capture"
export JOINT_CAPTURE_MODE="window"
export JOINT_CAPTURE_POST_S="5"
log "driver start (n=3)"
timeout --signal=INT 4200 ros2 run luggage_gazebo pack_eval_driver.py \
  --frame-runtime --frame-peer pack_eval_driver \
  --observe-pose pickup_observe_sim \
  --sequence-ids inch20 \
  --max-boxes 3 \
  --goto-timeout 60 \
  --plan-timeout 180 \
  --select-budget-s "$DIAG_ROUTE_BUDGET_S" \
  --scene-tf-config "$SCENE" \
  --seed 0 \
  --auth-max-attempts 8 \
  --auth-max-elapsed-sec 8 \
  --stop-on-first-failure \
  --out "$OUT/driver_out" > "$OUT/driver.log" 2>&1
DRC=$?
log "driver rc=$DRC"

# ---- Teardown: recorder ctl stop, then stack ----
if [ -d "$OUT/joint_capture/ctl" ] && [ ! -f "$OUT/joint_capture/manifest_recorder.json" ]; then
  : > "$OUT/joint_capture/ctl/stop"
  for _ in $(seq 1 30); do
    [ -f "$OUT/joint_capture/manifest_recorder.json" ] && break; sleep 2
  done
fi
kill -INT "$LPID" 2>/dev/null || true
sleep 5
kill -TERM "$LPID" 2>/dev/null || true
sleep 3
pkill -TERM -f "ign gazebo" >/dev/null 2>&1 || true
sleep 5
pkill -9 -f "ign gazebo" >/dev/null 2>&1 || true
log "end out=$OUT"
exit 0
