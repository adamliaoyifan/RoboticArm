#!/usr/bin/env bash
# Single-leg hold diagnosis boot (bounded; user-authorized 2026-10-20).
# Boots the SAME G3 scene stack as run.sh (A-prime container, seed 0)
# WITHOUT the eval driver / prior sidecars / boxes, then hands over to
# hold_diag.py. No scored run: this stack performs zero product motion
# of its own.
# usage: hold_diag_boot.sh <diag-out-dir>
set -euo pipefail
WS="/tmp/elfin-master-pack-baseline-g3"
MAIN="/home/adamliao/work/elfin_humble_ws"
OUT="${1:?diag out dir}"
mkdir -p "$OUT"

export ROS_DOMAIN_ID=7

# ---- Boot discipline v2 (same block as scripts/g3/run.sh) ----
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
    echo "{\"stage\":\"shm_sweep\",\"stale_files\":$N_SHM,\"ts\":\"$(date -Is)\"}" >> "$OUT/lifecycle.jsonl"
  fi
fi
if pgrep -f 'ros2 launch luggage_gazebo sim_world.launch.py' >/dev/null 2>&1; then
  echo "sim slot busy" >&2; exit 2
fi

# ---- Same scene rebind as run.sh (A-prime container; joint values
# unchanged, only the identity stamp follows the derived yaml) ----
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
    "scene_revision re-bound for hold_diag boot (md5 %s); joint values "
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

echo "{\"stage\":\"launch_begin\",\"ts\":\"$(date -Is)}\"" >> "$OUT/lifecycle.jsonl"
(ros2 launch luggage_gazebo sim_world.launch.py \
    gui:=false use_rviz:=false \
    use_semantic:=true use_motion:=true use_vacuum:=true \
    use_cargo_map:=true use_packing:=true \
    visual_kind:=mesh size_mode:=catalog \
    sequence_ids:=inch20 \
    yaw_mode:=discrete \
    random_seed:=0 \
    observe_pose_name:=pickup_observe_sim \
    frame_runtime:=true frame_driver_peer:=hold_diag \
    semantic_require_backend:=bbox_fill \
    semantic_model_name:="$WEIGHTS" \
    semantic_clip_vendor_dir:="$VENDOR" \
    semantic_pydeps_dir:="$PYDEPS" \
    scene_tf_config:="$SCENE" \
    start_placement_prior:=false \
    robot_poses_config:="$POSES_OUT" \
    > "$OUT/launch.log" 2>&1 &)
sleep 2
pid="$(pgrep -f 'ros2 launch luggage_gazebo sim_world.launch.py' | head -1 || true)"
if [[ -z "$pid" ]]; then echo "launch process missing" >&2; exit 1; fi
echo "$pid" > "$OUT/launch.pid"

# Wait for the arm action server (controllers active) — bounded 6 min.
for _ in $(seq 1 180); do
  if ros2 action list 2>/dev/null | grep -q "elfin_arm_controller/follow_joint_trajectory"; then
    echo "{\"stage\":\"arm_action_ready\",\"ts\":\"$(date -Is)}\"}" >> "$OUT/lifecycle.jsonl"
    echo "arm action ready"
    exit 0
  fi
  sleep 2
done
echo "arm action never became ready; see $OUT/launch.log" >&2
exit 3
