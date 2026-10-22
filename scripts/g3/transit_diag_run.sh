#!/usr/bin/env bash
# First-transit diagnostic (user step 2, 2026-10-21 ruling; precheck PASSED).
# Frozen production boot + frozen trajectory/safety gates; NO new variables.
# Only additions over hold_diag_boot.sh: diagnostic_capture:=true routing and
# joint_capture window wiring. Bounded: driver wall timeouts + 30-min cap.
# Post-failure retention: JOINT_DIAG_FAIL_RETAIN_NS=5e9 (not the 60 s wait).
set -o pipefail
export AMENT_TRACE_SETUP_FILES=0

# [Phase-1 functional diagnostic, 2026-10-22] One explicit knob feeding
# BOTH sides: the searcher (selector param diagnostic_route_budget_s)
# and the client wait (driver --select-budget-s = budget + 30 s comm
# overhead). 180 s per Select; per-edge budgets, retries, and all safety
# gates unchanged. Non-scoring — no performance claim.
DIAG_ROUTE_BUDGET_S=180

WS="/tmp/elfin-master-pack-baseline-g3"
MAIN="/home/adamliao/work/elfin_humble_ws"
OUT="/tmp/jcc_runs/transit_diag_$(date +%Y%m%d_%H%M%S)"
RUN_ID="transitdiag_$(date +%H%M%S)"
DEADLINE=$(( $(date +%s) + 1800 ))
mkdir -p "$OUT"
export ROS_DOMAIN_ID=7

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$OUT/runner.log"; }

# ---- Boot discipline v2 (verbatim block from hold_diag_boot.sh) ----
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

# ---- Same A-prime scene rebind as hold_diag_boot.sh ----
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
    "scene_revision re-bound for transit_diag boot (md5 %s); joint values "
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
# NOTE: no PYTHONPATH src prefixes — the driver's frame identity must report
# INSTALL overlay paths (src paths triggered WRONG_OVERLAY rejections in
# attempt 1). diagnostic_transit_driver.py imports its sibling
# pack_eval_driver module by script directory; everything else resolves from
# the worktree install.

# ---- Launch the frozen stack (+ the two authorized additions) ----
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
    diagnostic_capture:=true \
    diagnostic_route_budget_s:=$DIAG_ROUTE_BUDGET_S \
    joint_capture_mode:=window joint_capture_run_dir:="$OUT/joint_capture" \
    > "$OUT/launch.log" 2>&1 &)
sleep 2
LPID="$(pgrep -f 'ros2 launch luggage_gazebo sim_world.launch.py' | head -1 || true)"
if [[ -z "$LPID" ]]; then log "launch process missing"; exit 1; fi
log "launch pid=$LPID out=$OUT run_id=$RUN_ID"

# Wait for the capture-routing stack readiness (controllers up).
ok=""
for _ in $(seq 1 60); do
  sleep 5
  grep -q "controller elfin_arm_controller.*active" "$OUT/launch.log" 2>/dev/null \
    && grep -q "\[jcc\] capture mode='window' active=1" "$OUT/launch.log" 2>/dev/null \
    && ok=1 && break
  [ "$(date +%s)" -gt "$DEADLINE" ] && break
done
if [[ -z "$ok" ]]; then
  log "stack readiness gate not observed in launch.log; continuing to driver anyway (bounded)"
fi

# ---- Capture node (fresh dir; bridges routed actions + service) ----
python3 "$WS/src/luggage_planning/scripts/diagnostic_capture_node.py" \
  --directory "$OUT/capture_node" --run-id "$RUN_ID" \
  > "$OUT/capture_node.log" 2>&1 &
CPID=$!
# Wait until the health topic is actually visible on this loaded domain
# (fresh-participant discovery under the full stack can exceed the driver's
# own 5 s readiness gate — the failed first attempt died exactly there).
log "waiting for /diagnostic_capture/health visibility"
HOK=""
for _ in $(seq 1 60); do
  if timeout 5 python3 - "$RUN_ID" <<'PYPOLL' >> "$OUT/health_poll.log" 2>&1
import sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
rclpy.init()
n = Node("jcc_health_poll")
got = {"v": False}
def cb(m):
    got["v"] = "transitdiag" in m.data
n.create_subscription(String, "/diagnostic_capture/health", cb, 10)
t0 = time.time()
while rclpy.ok() and not got["v"] and time.time() - t0 < 4.0:
    rclpy.spin_once(n, timeout_sec=0.1)
n.destroy_node(); rclpy.shutdown()
sys.exit(0 if got["v"] else 1)
PYPOLL
  then HOK=1; break; fi
  kill -0 "$CPID" 2>/dev/null || break
  [ "$(date +%s)" -gt "$DEADLINE" ] && break
done
if [[ -z "$HOK" ]]; then
  log "health topic never became visible; aborting before any motion"
  kill -INT "$CPID" 2>/dev/null || true; sleep 3
  kill -TERM "$CPID" 2>/dev/null || true
  kill -INT "$LPID" 2>/dev/null || true; sleep 5
  pkill -TERM -f "ign gazebo" 2>/dev/null || true; sleep 3
  pkill -9 -f "ign gazebo" 2>/dev/null || true
  exit 4
fi
log "health visible; starting driver"

# ---- Driver: production args (prodcap-frozen) + bounded new envs ----
export JOINT_CAPTURE_DIR="$OUT/joint_capture"
export JOINT_CAPTURE_MODE="window"
export JOINT_CAPTURE_POST_S="5"
export JOINT_DIAG_HOLD_NS="10e9"
export JOINT_DIAG_FAIL_RETAIN_NS="5e9"
log "driver start"
timeout 1200 python3 "$WS/src/luggage_gazebo/scripts/diagnostic_transit_driver.py" \
  --capture-directory "$OUT/capture_driver" --capture-run-id "$RUN_ID" \
  --out "$OUT/driver_out" --dump-dir "$OUT/dumps" \
  --scene-tf-config "$SCENE" \
  --sequence-ids inch20 --observe-pose pickup_observe_sim \
  --plan-timeout 180 --goto-timeout 60 --auth-max-elapsed-sec 8 \
  --select-budget-s "$DIAG_ROUTE_BUDGET_S" \
  --frame-runtime --frame-peer pack_eval_driver \
  --stop-on-first-failure \
  > "$OUT/diag_driver.log" 2>&1
DRC=$?
log "driver rc=$DRC"

# ---- Teardown: capture node (graceful), recorder (ctl stop), stack ----
kill -INT "$CPID" 2>/dev/null || true
for _ in $(seq 1 20); do kill -0 "$CPID" 2>/dev/null || break; sleep 1; done
kill -TERM "$CPID" 2>/dev/null || true
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

python3 - "$OUT" <<'PYSUM'
import json, sys
from pathlib import Path
out = Path(sys.argv[1])
summary = {"out": str(out)}
for name in ("driver_out/diagnostic_capture_result.json",
             "joint_capture/manifest_recorder.json"):
    p = out / name
    if p.exists():
        summary[name.split("/")[0]] = json.loads(p.read_text())
print(json.dumps(summary, indent=2, default=str))
Path(out, "RUN_SUMMARY.json").write_text(json.dumps(summary, indent=2, default=str))
PYSUM
log "end out=$OUT"
exit 0
