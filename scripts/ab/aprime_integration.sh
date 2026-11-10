#!/usr/bin/env bash
# A-prime INTEGRATION CHECK (post-descope, 2026-10-19): one repeat of
# the frozen release-boundary replay on the A-prime arm with the FIXED
# contact recorder. Purpose: (a) reproducibility of the 0.013 mm
# residual (n was 1/1); (b) FIRST contact-manifold evidence for A'
# (no-penetration / partner set). Not an AB-batch run; occupies no
# batch slot; scored nowhere. Same frozen input bundle as batch 1.
set -euo pipefail
WS="/tmp/elfin-master-pack-baseline-g3"
MAIN="/home/adamliao/work/elfin_humble_ws"
EVID="$MAIN/docs/status/evidence/master_pack_recovery/2026-10-14_g3"
OBS="$EVID/obs_rel_07"
OUT="$WS/docs/status/evidence/occ_owner_b1/aprime_integration_01"
WORLD="airport_loading"
ARM="APRIME"
MODEL="airport_container_real_aprime"
[[ -e "$OUT" ]] && { echo "integration dir exists (no re-runs): $OUT" >&2; exit 2; }

SCENE=/tmp/ab_scene_aprime_int.yaml
python3 - "$SCENE" << 'PY'
import sys, yaml
cfg = yaml.safe_load(open(
    "/tmp/elfin-master-pack-baseline-g3/src/luggage_description/config/"
    "scene_tf_deploy_update.yaml"))
cfg.setdefault("gazebo", {})["container_model"] = \
    "airport_container_real_aprime"
yaml.safe_dump(cfg, open(sys.argv[1], "w"), sort_keys=False)
PY
mkdir -p "$OUT/dumps" "$OUT/sdf"
echo "{\"stage\":\"start\",\"arm\":\"$ARM\",\"ts\":\"$(date -Is)\"}" > "$OUT/lifecycle.jsonl"

# ---- Boot discipline v2 (shared with run_one.sh). ----
bash "$WS/scripts/stop_sim.sh" --all >> "$OUT/bootsweep.log" 2>&1 || true
for pid in $(pgrep -x parameter_bridge); do kill -KILL "$pid" 2>/dev/null || true; done
sleep 1
if pgrep -x ign >/dev/null 2>&1 || pgrep -x gz >/dev/null 2>&1 \
   || pgrep -f '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch luggage_gazebo' >/dev/null 2>&1 \
   || pgrep -x parameter_bridge >/dev/null 2>&1; then
  echo "sim slot still busy AFTER sweep; aborting" >&2
  exit 3
fi
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

set +u
source /opt/ros/humble/setup.bash
source "$MAIN/install/setup.bash"
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID=7

OWN_PIDS=()
cleanup() {
  local status=$?
  for pid in "${OWN_PIDS[@]:-}"; do kill -TERM "$pid" 2>/dev/null || true; done
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    sleep 6
  fi
  bash "$WS/scripts/stop_sim.sh" --all >> "$OUT/bootsweep.log" 2>&1 || true
  for pid in $(pgrep -x parameter_bridge); do kill -KILL "$pid" 2>/dev/null || true; done
  exit $status
}
trap cleanup EXIT INT TERM
IGN_BIN="$(command -v ign || command -v gz)"

# ---- Launch with A-prime container (proven arg set). ----
ros2 launch luggage_gazebo sim_world.launch.py \
  gui:=false use_rviz:=false \
  use_semantic:=true use_motion:=true use_vacuum:=true \
  use_cargo_map:=true use_packing:=true \
  visual_kind:=mesh size_mode:=catalog \
  sequence_ids:=inch20 yaw_mode:=discrete random_seed:=0 \
  observe_pose_name:=pickup_observe_sim \
  semantic_require_backend:=bbox_fill \
  semantic_model_name:="$MAIN/yolov8s-world.pt" \
  semantic_clip_vendor_dir:="$MAIN/src/luggage_perception/vendor" \
  semantic_pydeps_dir:="$MAIN/pydeps-frozen" \
  scene_tf_config:="$SCENE" \
  atlas_runtime_config:="$WS/src/luggage_planning/config/atlas_runtime_deploy_update.yaml" \
  robot_poses_config:="$WS/src/luggage_description/config/robot_poses.baseline_update.yaml" \
  frame_runtime:=true frame_driver_peer:=pack_eval_driver \
  > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
OWN_PIDS+=("$LAUNCH_PID")

# ---- Asset record (runtime-loaded model). ----
MODELS_SHARE="$WS/install/luggage_gazebo/share/luggage_gazebo/models"
python3 - "$MODEL" "$MODELS_SHARE" "$OUT/asset_record.json" << 'PY'
import hashlib, json, os, sys
model, share, out = sys.argv[1:4]
d = os.path.join(share, model)
rec = {"model": model, "dir": d, "files": {}}
for name in sorted(os.listdir(d)) + (
    ["meshes/" + f for f in sorted(os.listdir(os.path.join(d, "meshes")))]
    if os.path.isdir(os.path.join(d, "meshes")) else []):
    p = os.path.join(d, name)
    if os.path.isfile(p):
        rec["files"][name] = hashlib.sha256(open(p, "rb").read()).hexdigest()
json.dump(rec, open(out, "w"), indent=2)
PY

# ---- Captures (v2 discipline + depths). ----
export ELFIN_BOX_SDF_DUMP="$OUT/sdf/spawn.sdf"
export ELFIN_BOX_RELEASE_SDF="$OUT/sdf/release.sdf"
CAP_PIDS=()
POSE_TMP=$(mktemp -p /dev/shm -t abpose.XXXXXX)
setsid "$IGN_BIN" topic -e -t "/world/$WORLD/dynamic_pose/info" \
  > "$POSE_TMP" 2>/dev/null &
CAP_PIDS+=($!)
OBS_TOOL_JOINTS_ONLY=1 python3 "$WS/scripts/g3/record_tool_stream.py" \
  "$OUT/tool_stream.jsonl" &
CAP_PIDS+=($!)
IGN_BIN="$IGN_BIN" python3 "$WS/scripts/ab/spawn_contact_bridge.py" &
CAP_PIDS+=($!)
sleep 1
python3 "$WS/scripts/ab/record_box_stream.py" \
  "$OUT/box_contacts.jsonl" pickup_box_0001_inch20 > "$OUT/recorder.log" 2>&1 &
CAP_PIDS+=($!)
OWN_PIDS+=("${CAP_PIDS[@]}")

for _ in $(seq 1 120); do
  grep -q "Controller manager: loaded" "$OUT/launch.log" 2>/dev/null && break
  sleep 2
done
sleep 5

python3 - "$OUT" << 'PY' > "$OUT/replay_spec.json"
import json, sys
out = sys.argv[1]
fi = json.load(open(
    "/tmp/elfin-master-pack-baseline-g3/docs/status/evidence/occ_owner_b1/"
    "AB_RUN_ORDER.json"))["frozen_input"]
print(json.dumps({
    "n": "I1", "world": "airport_loading",
    "model": "pickup_box_0001_inch20",
    "pose6": list(fi["spawn_pose_world"]) + [
        fi["spawn_quat_xyzw"][3], fi["spawn_quat_xyzw"][0],
        fi["spawn_quat_xyzw"][1], fi["spawn_quat_xyzw"][2]],
    "release_sdf": out + "/sdf/release.sdf",
    "tool_stream": "/home/adamliao/work/elfin_humble_ws/docs/status/"
    "evidence/master_pack_recovery/2026-10-14_g3/obs_rel_07/"
    "tool_stream.jsonl",
    "t_release": fi["t_release_sim_s"],
    "t_spawn": fi["t_spawn_sim_s"],
    "window_s": 45.0,
    "out_json": out + "/replay_report.json",
}))
PY
cp "$OBS/sdf/release.sdf" "$OUT/sdf/release.sdf"

timeout 520 python3 "$WS/scripts/ab/replay_node.py" \
  "$(cat "$OUT/replay_spec.json")" > "$OUT/replay.log" 2>&1 || true
echo "{\"stage\":\"replay_done\",\"ts\":\"$(date -Is)\"}" >> "$OUT/lifecycle.jsonl"

kill -TERM "${CAP_PIDS[@]}" 2>/dev/null || true
sleep 3
[[ -f "$POSE_TMP" ]] && { cp "$POSE_TMP" "$OUT/dyn_pose_stream.txt"; rm -f "$POSE_TMP"; }

T_SPAWN=$(python3 -c "import json;print(json.load(open('$OUT/replay_spec.json'))['t_spawn'])")
python3 "$WS/scripts/ab/metrics.py" --run-dir "$OUT" \
  --model pickup_box_0001_inch20 --t-spawn-sim "$T_SPAWN" \
  --out run_metrics.json > "$OUT/metrics.log" 2>&1 || true

kill -INT "$LAUNCH_PID" 2>/dev/null || true
sleep 8
bash "$WS/scripts/stop_sim.sh" --all >> "$OUT/bootsweep.log" 2>&1 || true
echo "{\"stage\":\"teardown\",\"ts\":\"$(date -Is)\"}" >> "$OUT/lifecycle.jsonl"
wc -c "$OUT/dyn_pose_stream.txt" "$OUT/box_contacts.jsonl" \
  "$OUT/tool_stream.jsonl" || true
echo "APRIME INTEGRATION 01 COMPLETE -> $OUT"
