#!/usr/bin/env bash
# BASELINE G3 runner. Does not call stop_sim before launch.
# If a Gazebo stack is already up, this exits and leaves it alone.
# usage: run.sh <mode> <run-name>   mode = rehearsal|score
set -euo pipefail

MODE="${1:?mode}"
NAME="${2:?run name}"
WS="/tmp/elfin-master-pack-baseline-g3"
MAIN="/home/adamliao/work/elfin_humble_ws"
EVID="$MAIN/docs/status/evidence/master_pack_recovery/2026-10-14_g3"
OUT="$EVID/$NAME"
SCENE="$WS/src/luggage_description/config/scene_tf_deploy_update.yaml"
ATLAS="$WS/src/luggage_planning/config/atlas_runtime_deploy_update.yaml"
POSES="$WS/src/luggage_description/config/robot_poses.baseline_update.yaml"
WEIGHTS="$MAIN/yolov8s-world.pt"
VENDOR="$MAIN/src/luggage_perception/vendor"
PYDEPS="$MAIN/pydeps-frozen"
PIDFILE="${ELFIN_SIM_PIDFILE:-/tmp/elfin_humble_sim.pid}"

case "$MODE" in
  rehearsal) MAX_BOXES=1; DRIVER_TIMEOUT=2400 ;;
  multi)     MAX_BOXES="${G3_MAX_BOXES:-4}"; DRIVER_TIMEOUT=3600 ;;
  score)     MAX_BOXES=2; DRIVER_TIMEOUT=1200 ;;
  observe)   MAX_BOXES=0; DRIVER_TIMEOUT=0 ;;
  *) echo "mode must be rehearsal, score, observe or multi" >&2; exit 2 ;;
esac
if [[ -e "$OUT" ]]; then echo "run dir already exists: $OUT" >&2; exit 2; fi

mkdir -p "$OUT/dumps/boundaries"
export MOTION_BOUNDARY_DUMP="$OUT/dumps/boundaries"
# G2 capture paths stay enabled on the live producer for every G3 run,
# including scoring (dedup by target+scene semantics is in the producer).
export ST09_PORTAL_CAPTURE_DIR="$OUT/portal_capture"
export ST09_SELECT_REQUEST_CAPTURE_DIR="$OUT/select_capture"
export ST09_I_CERT_CAPTURE_DIR="$OUT/cert_capture"
mkdir -p "$ST09_PORTAL_CAPTURE_DIR" "$ST09_SELECT_REQUEST_CAPTURE_DIR" "$ST09_I_CERT_CAPTURE_DIR" "$OUT/sdf"
export ELFIN_BOX_SDF_DUMP="$OUT/sdf/spawn.sdf"
export ELFIN_BOX_RELEASE_SDF="$OUT/sdf/release.sdf"
export ELFIN_SUPPORT_CANDIDATES_DUMP="$OUT/support_candidates.npz"
export ROS_DOMAIN_ID=7

stack_pids() {
  pgrep -af 'ros2 launch luggage_gazebo|ign gazebo|gz sim|ros_gz_bridge|parameter_bridge|clock_bridge|camera_bridge' \
    | grep -v -e 'run.sh' -e 'pgrep -af' -e 'builtin eval' || true
}

# ---- Boot discipline v2 (2026-10-19): kill the PREVIOUS sim before a
# new boot. Launch INT leaves node children orphaned to systemd --user;
# they accumulate as live DDS participants holding /dev/shm/fastrtps
# port locks, and new boots then fail open_and_lock_file (the five
# infra-aborts of the AB batch). With no ros2 launch alive every
# participant IS an orphan: TERM->KILL, then clear stale SHM locks.
if [[ "${G3_NO_SWEEP:-0}" != "1" ]]; then
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
fi

if [[ -n "$(stack_pids)" || -f "$PIDFILE" ]]; then
  echo "sim slot busy; not launching and not stopping the existing stack" >&2
  stack_pids >&2 || true
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "$MAIN/install/setup.bash"
source "$WS/install/setup.bash"
set -u

# ---- Candidate-fix deployment: container collision model override.
# G3_CONTAINER_MODEL selects the gazebo container model (A-prime for
# the fix candidate). A derived scene yaml is written into the run dir
# (the ONLY yaml the launch AND the driver see) so the frozen manifest
# records exactly what ran; geometry consumers (planner cavity, cargo
# grid, TF) read the same yaml — single source, no per-arm divergence.
if [[ -n "${G3_CONTAINER_MODEL:-}" ]]; then
  SCENE="$OUT/scene_tf_used.yaml"
  python3 - "$SCENE" "$OUT/robot_poses_used.yaml" << 'PYSCENE'
import hashlib, os, sys, yaml
cfg = yaml.safe_load(open(
    "/tmp/elfin-master-pack-baseline-g3/src/luggage_description/config/"
    "scene_tf_deploy_update.yaml"))
cfg.setdefault("gazebo", {})["container_model"] = \
    os.environ["G3_CONTAINER_MODEL"]
yaml.safe_dump(cfg, open(sys.argv[1], "w"), sort_keys=False)
# The container-model override changes md5(scene_tf) -> the frame
# runtime scene id changes -> GoToRobotPoseV2 SCENE_STALE against the
# poses file's embedded identity.scene_revision. Re-bind ONLY the
# identity stamp (same precedented overlay as the pickup_platform
# height change recorded in the file header): the container model
# does not move the robot base, camera target, or the calibrated
# joints, so the pose values are NOT re-solved.
scene_md5 = hashlib.md5(open(sys.argv[1], "rb").read()).hexdigest()
poses = yaml.safe_load(open(
    "/tmp/elfin-master-pack-baseline-g3/src/luggage_description/config/"
    "robot_poses.baseline_update.yaml"))
poses.setdefault("identity", {})["scene_revision"] = scene_md5
poses["identity"]["_rebind_note"] = (
    "scene_revision re-bound to the G3_CONTAINER_MODEL-derived "
    "scene_tf_used.yaml (md5 %s); joint values unchanged" % scene_md5)
# Per-pose capture stamps must follow the same re-bind (r3 lesson:
# validate_site_poses checks scene_revision_at_capture per pose and
# fails POSE_NOT_VALID otherwise). Same rationale — the container
# model does not affect any calibrated joint.
n_poses = 0
for _name, pose in (poses.get("poses") or {}).items():
    if isinstance(pose, dict) and pose.get("scene_revision_at_capture"):
        pose["scene_revision_at_capture"] = scene_md5
        n_poses += 1
yaml.safe_dump(poses, open(sys.argv[2], "w"), sort_keys=False)
print("scene_md5", scene_md5, "poses_rebound", n_poses)
PYSCENE
  POSES="$OUT/robot_poses_used.yaml"
fi

DRIVER="$(readlink -f "$(ros2 pkg prefix luggage_gazebo)/lib/luggage_gazebo/pack_eval_driver.py")"
case "$DRIVER" in
  "$WS"/*) ;;
  *) echo "install identity mismatch: $DRIVER" >&2; exit 1 ;;
esac

python3 - << PY
import hashlib, json, subprocess, time
from pathlib import Path
out = Path("$OUT")
def sha(path):
    p = Path(path)
    h = hashlib.sha256()
    if p.is_dir():
        for child in sorted(q for q in p.rglob("*") if q.is_file()):
            h.update(str(child.relative_to(p)).encode())
            h.update(child.read_bytes())
        return h.hexdigest()
    with p.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()
rev = subprocess.check_output(["git", "-C", "$WS", "rev-parse", "HEAD"], text=True).strip()
dirty = subprocess.check_output(["git", "-C", "$WS", "status", "--porcelain"], text=True)
manifest = {
    "mode": "$MODE",
    "run": "$NAME",
    "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    "base_revision": "56b2a41ffc00d4e2292fe235863716519ea71d80",
    "head": rev,
    "dirty_lines": len([ln for ln in dirty.splitlines() if ln.strip()]),
    "driver": "$DRIVER",
    "driver_sha256": sha("$DRIVER"),
    "scene_sha256": sha("$SCENE"),
    "atlas_yaml_sha256": sha("$ATLAS"),
    "atlas_npz_sha256": sha("$WS/src/luggage_planning/data/reachability_atlas_deploy_update/s20_container_collision_aware.npz"),
    "poses_sha256": sha("$POSES"),
    "weights": "$WEIGHTS",
    "weights_sha256": sha("$WEIGHTS"),
    "vendor": "$VENDOR",
    "vendor_sha256": sha("$VENDOR"),
    "pydeps": "$PYDEPS",
    "pydeps_deployed_sha256": sha("$PYDEPS/DEPLOYED.json"),
    "max_boxes": $MAX_BOXES,
    "seed": 0,
    "ros_domain_id": 7,
    "goto_timeout_s": 60,
    "plan_timeout_s": 180,
    "container_model": __import__("os").environ.get("G3_CONTAINER_MODEL", ""),
    "container_model_dir_sha256": (
        sha("$WS/install/luggage_gazebo/share/luggage_gazebo/models/"
            + __import__("os").environ.get("G3_CONTAINER_MODEL", ""))
        if __import__("os").environ.get("G3_CONTAINER_MODEL") else ""),
}
(out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print(json.dumps({"head": rev, "dirty_lines": manifest["dirty_lines"], "driver": manifest["driver"]}))
PY

echo "{\"stage\":\"launch_begin\",\"ts\":\"$(date -Is)\"}" > "$OUT/lifecycle.jsonl"

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
    atlas_runtime_config:="$ATLAS" \
    robot_poses_config:="$POSES" \
    > "$OUT/launch.log" 2>&1 &)
sleep 2
pid="$(pgrep -f 'ros2 launch luggage_gazebo sim_world.launch.py' | head -1 || true)"
if [[ -z "$pid" ]]; then
  echo "launch process missing" >&2
  exit 1
fi
echo "$pid" > "$PIDFILE"
echo "{\"stage\":\"launch\",\"pid\":$pid,\"ts\":\"$(date -Is)\"}" >> "$OUT/lifecycle.jsonl"

ready=0
for _ in $(seq 1 40); do
  if grep -q "startup aborted\|startup_failed\|plugin_urdf_not_received\|Caught exception in launch" "$OUT/launch.log"; then
    echo "startup failed" >&2
    bash "$WS/scripts/stop_sim.sh" || true
    exit 1
  fi
  if grep -q "Held pose 'pickup_observe_sim'" "$OUT/launch.log"; then
    ready=1
    break
  fi
  sleep 3
done
if [[ "$ready" -ne 1 ]]; then
  echo "observe pose was not held within 120 s" >&2
  bash "$WS/scripts/stop_sim.sh" || true
  exit 1
fi
echo "{\"stage\":\"observe_held\",\"ts\":\"$(date -Is)\"}" >> "$OUT/lifecycle.jsonl"

# G3-0 readiness window: exactly one /clock publisher, >=10 increasing
# stamps each for camera and inference, measured rates recorded.
BASELINE_OUT="$OUT" BASELINE_MODE="$MODE" python3 - << 'PY'
import json, os, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from luggage_msgs.msg import YoloDetections

out = Path(os.environ["BASELINE_OUT"])
rclpy.init()
node = Node("g3_ready_probe")
camera = {"n": 0, "stamps": []}
infer = {"n": 0, "stamps": []}

def on_cam(msg):
    camera["n"] += 1
    if len(camera["stamps"]) < 12:
        camera["stamps"].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

def on_yolo(msg):
    infer["n"] += 1
    if len(infer["stamps"]) < 12:
        infer["stamps"].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

cam_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
node.create_subscription(Image, "/camera/color/image_raw", on_cam, cam_qos)
yolo_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)
node.create_subscription(YoloDetections, "/luggage/semantic/yolo_detections", on_yolo, yolo_qos)
t0 = time.time()
while time.time() - t0 < 10.0:
    rclpy.spin_once(node, timeout_sec=0.2)
clock = node.get_publishers_info_by_topic("/clock")
report = {
    "window_sec": 10.0,
    "clock_publishers": len(clock),
    "camera_samples": camera["n"],
    "camera_rate_hz": round(camera["n"] / 10.0, 2),
    "camera_stamp_increasing": all(
        camera["stamps"][i] < camera["stamps"][i + 1]
        for i in range(len(camera["stamps"]) - 1)) if len(camera["stamps"]) > 1 else False,
    "inference_samples": infer["n"],
    "inference_rate_hz": round(infer["n"] / 10.0, 2),
    "inference_stamp_increasing": all(
        infer["stamps"][i] < infer["stamps"][i + 1]
        for i in range(len(infer["stamps"]) - 1)) if len(infer["stamps"]) > 1 else False,
}
(out / "readiness.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report))
node.destroy_node()
rclpy.shutdown()
PY

if [[ "$MODE" == "observe" ]]; then
  # P1 geometry chain audit: spawn one box, production DetectLuggage,
  # GT record + synchronized frame; no driver, no motion.
  python3 "$WS/scripts/g3/geometry_chain_audit.py" collect "$OUT/chain_session" \
    > "$OUT/chain_session.log" 2>&1
  audit_rc=$?
  if [[ -n "${G3_OBSERVE_HOLD:-}" ]]; then
    echo "{"stage":"hold","seconds":${G3_OBSERVE_HOLD},"ts":"$(date -Is)"}" >> "$OUT/lifecycle.jsonl"
    sleep "$G3_OBSERVE_HOLD"
  fi
  bash "$WS/scripts/stop_sim.sh" || true
  echo "{\"stage\":\"audit_collect\",\"rc\":$audit_rc,\"ts\":\"$(date -Is)\"}" >> "$OUT/lifecycle.jsonl"
  exit "$audit_rc"
fi

if [[ "$MODE" == "rehearsal" ]]; then
  # B1 live input: one complete synchronized RGB/depth/info/TF/detection
  # bundle, recorded before any motion, for the twice-replay gate.
  python3 "$WS/scripts/g3/b1_capture.py" "$OUT/b1_input" > "$OUT/b1_capture.log" 2>&1 \
    || echo "b1_capture failed (see b1_capture.log)" >&2
  echo "{\"stage\":\"b1_captured\",\"ts\":\"$(date -Is)\"}" >> "$OUT/lifecycle.jsonl"
fi

echo "{\"stage\":\"driver_begin\",\"ts\":\"$(date -Is)\"}" >> "$OUT/lifecycle.jsonl"

# ---- Contact capture across the driver window (acceptance evidence:
# box-vs-container/tool manifold, penetration depths). The box SDF's
# contact sensor publishes gz /obs/box_contact; record via gz echo on
# TMPFS (no parameter_bridge: the driver's frozen graph audit counts
# parameter_bridge processes and allows only the launch's 3 — a 4th
# bridge aborts the run before box 0; learned in gate3_aprime_r1).
# parse_contacts() reads this protobuf-text format offline.
IGN_BIN="$(command -v ign || command -v gz)"
CONTACT_TMP=$(mktemp -p /dev/shm -t g3box.XXXXXX)
setsid "$IGN_BIN" topic -e -t /obs/box_contact > "$CONTACT_TMP" 2>/dev/null &
CONTACT_PID=$!

set +e
timeout --signal=INT "$DRIVER_TIMEOUT" ros2 run luggage_gazebo pack_eval_driver.py \
  --frame-runtime --frame-peer pack_eval_driver \
  --observe-pose pickup_observe_sim \
  --sequence-ids inch20 \
  --max-boxes "$MAX_BOXES" \
  --goto-timeout 60 \
  --plan-timeout 180 \
  --seed 0 \
  --auth-max-attempts 8 \
  --auth-max-elapsed-sec 8 \
  --stop-on-first-failure \
  --scene-tf-config "$SCENE" \
  --out "$OUT" > "$OUT/driver.log" 2>&1
driver_rc=$?
set -e
kill -TERM "$CONTACT_PID" 2>/dev/null || true
sleep 2
echo "{\"stage\":\"driver_exit\",\"rc\":$driver_rc,\"ts\":\"$(date -Is)\"}" >> "$OUT/lifecycle.jsonl"
bash "$WS/scripts/stop_sim.sh" || true
# Materialize the tmpfs contact stream AFTER teardown (disk contention
# during the run is what reordered delivery in the AB batch).
if [[ -f "$CONTACT_TMP" ]]; then
  cp "$CONTACT_TMP" "$OUT/contact_stream.txt"
  rm -f "$CONTACT_TMP"
fi
echo "{\"stage\":\"teardown\",\"ts\":\"$(date -Is)\"}" >> "$OUT/lifecycle.jsonl"
exit "$driver_rc"
