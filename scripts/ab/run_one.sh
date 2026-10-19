#!/usr/bin/env bash
# One A0/A'/B control run (frozen 2026-10-19 contract).
# usage: run_one.sh <n>  (n = 1..9 per AB_RUN_ORDER.json)
set -euo pipefail
N="${1:?run number}"
WS="/tmp/elfin-master-pack-baseline-g3"
MAIN="/home/adamliao/work/elfin_humble_ws"
EVID="$MAIN/docs/status/evidence/master_pack_recovery/2026-10-14_g3"
ORDER="$WS/docs/status/evidence/occ_owner_b1/AB_RUN_ORDER.json"
OBS="$EVID/obs_rel_07"
OUT="$EVID/ab_run_$(printf '%02d' "$N")"
WORLD="airport_loading"
[[ -e "$OUT" ]] && { echo "run dir exists (no re-runs): $OUT" >&2; exit 2; }

read -r ARM MODEL SCENE < <(python3 - "$N" << 'PY'
import json, sys, yaml, copy, os
order = json.load(open("docs/status/evidence/occ_owner_b1/AB_RUN_ORDER.json"))
run = next(r for r in order["runs"] if r["n"] == int(sys.argv[1]))
arm = run["arm"]
model = {"A0": "airport_container_real",
         "APRIME": "airport_container_real_aprime",
         "B": "airport_container_real_bprim"}[arm]
out = "/tmp/ab_scene_%d.yaml" % int(sys.argv[1])
cfg = yaml.safe_load(open("src/luggage_description/config/"
                          "scene_tf_deploy_update.yaml"))
cfg.setdefault("gazebo", {})["container_model"] = model
# Keep the same robot-poses source the observation used.
yaml.safe_dump(cfg, open(out, "w"), sort_keys=False)
print(arm, model, out)
PY
)
mkdir -p "$OUT/dumps"
echo "{\"stage\":\"start\",\"n\":$N,\"arm\":\"$ARM\",\"model\":\"$MODEL\",\"ts\":\"$(date -Is)\"}" > "$OUT/lifecycle.jsonl"

# ---- Clean room, then sim exclusivity. ----
# Clean room FIRST: reap THIS experiment's crashed-attempt orphans
# (ghost parameter bridges publish ghost /clock and answer create
# requests — they caused 5 straight broadcaster-configure failures).
# Sim binaries (ign/gz) are NOT reaped here: those belong to whoever
# runs them; the gate below still refuses on any live sim.
for pid in $(pgrep -x parameter_bridge); do kill -KILL "$pid" 2>/dev/null || true; done
sleep 1
if pgrep -x ign >/dev/null 2>&1 || pgrep -x gz >/dev/null 2>&1 \
    || pgrep -f '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch luggage_gazebo' >/dev/null 2>&1 \
    || pgrep -x parameter_bridge >/dev/null 2>&1; then
  echo "sim slot busy; aborting (no re-run without exclusivity)" >&2
  exit 3
fi
# Match real binaries only: -x on binary names (ign/gz) plus the
# launch process; a plain -f pattern matches monitoring shells whose
# command line merely CONTAINS the string (aborted run 01 that way).


set +u
source /opt/ros/humble/setup.bash
source "$MAIN/install/setup.bash"
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID=7

OWN_PIDS=()
cleanup() {
  local status=$?
  for pid in "${OWN_PIDS[@]:-}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  for pid in "${OWN_PIDS[@]:-}"; do
    kill -KILL "$pid" 2>/dev/null || true
  done
  # Experiment-owned cleanup: the launch tree we started, then the
  # workspace's proven sim reaper (launch INT leaves bridges/ign alive —
  # the orphans then trip the NEXT run's gate; verified exclusive before
  # launch, so every sim process at cleanup is ours).
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    sleep 6
  fi
  bash "$WS/scripts/stop_sim.sh" >/dev/null 2>&1 || true
  for pid in $(pgrep -x parameter_bridge); do kill -KILL "$pid" 2>/dev/null || true; done
  exit $status
}
trap cleanup EXIT INT TERM

IGN_BIN="$(command -v ign || command -v gz)"
capture() { ( exec setsid "$IGN_BIN" topic -e -t "$1" > "$2" 2>/dev/null & echo $! > /dev/stderr ) ; }

# ---- Launch with the arm's container (light: no perception/packing). ----
(
  # PROVEN bring-up path: the observation launch arg set (obs_rel_07
  # succeeded on it 5x). The light set raced the broadcaster spawner
  # past the robot's gz_ros2_control bring-up ("Failed to configure
  # controller", S20 never created) twice — not a run variable, so the
  # harness uses the proven args and varies ONLY the container model
  # via scene_tf_config.
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
)
sleep 2
LAUNCH_PID="$(pgrep -f 'ros2 launch luggage_gazebo' | head -1 || true)"
[[ -n "$LAUNCH_PID" ]] || { echo "launch failed" >&2; exit 1; }
OWN_PIDS+=("$LAUNCH_PID")

# ---- Record the runtime-loaded collision asset (hash + params). ----
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

# ---- Captures (light discipline: dynamic stream + contacts + joints). ----
export ELFIN_BOX_SDF_DUMP="$OUT/sdf/spawn.sdf"
export ELFIN_BOX_RELEASE_SDF="$OUT/sdf/release.sdf"
mkdir -p "$OUT/sdf"
CAP_PIDS=()
setsid "$IGN_BIN" topic -e -t "/world/$WORLD/dynamic_pose/info" \
  > "$OUT/dyn_pose_stream.txt" 2>/dev/null &
CAP_PIDS+=($!)
setsid "$IGN_BIN" topic -e -t "/obs/box_contact" \
  > "$OUT/contact_stream.txt" 2>/dev/null &
CAP_PIDS+=($!)
OBS_TOOL_JOINTS_ONLY=1 python3 "$WS/scripts/g3/record_tool_stream.py" \
  "$OUT/tool_stream.jsonl" &
CAP_PIDS+=($!)
OWN_PIDS+=("${CAP_PIDS[@]}")

# ---- Wait for controllers, then run the frozen replay. ----
for _ in $(seq 1 120); do
  grep -q "Controller manager: loaded" "$OUT/launch.log" 2>/dev/null && break
  sleep 2
done
sleep 5

python3 - "$N" "$OUT" << 'PY' > "$OUT/replay_spec.json"
import json, sys
n, out = sys.argv[1:3]
order = json.load(open("docs/status/evidence/occ_owner_b1/AB_RUN_ORDER.json"))
fi = order["frozen_input"]
spec = {
    "n": int(n), "world": "airport_loading",
    "model": "pickup_box_0001_inch20",
    "pose6": fi["spawn_pose_world"]
    + [-fi["spawn_quat_xyzw"][3], *fi["spawn_quat_xyzw"][:3]]
    if False else
    list(fi["spawn_pose_world"]) + [
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
}
# The shell redirect writes stdout to replay_spec.json; writing the
    # file from inside too interleaves two JSON documents (run-01 crash).
print(json.dumps(spec))
PY

# Seed the recorded release.sdf (the exact production SDF).
cp "$OBS/sdf/release.sdf" "$OUT/sdf/release.sdf"

timeout 300 python3 "$WS/scripts/ab/replay_node.py" \
  "$(cat "$OUT/replay_spec.json")" > "$OUT/replay.log" 2>&1 || true

echo "{\"stage\":\"replay_done\",\"ts\":\"$(date -Is)\"}" >> "$OUT/lifecycle.jsonl"

# ---- Metric + validity gate (sim-clock). ----
T_SPAWN=$(python3 -c "import json;print(json.load(open('$OUT/replay_spec.json'))['t_spawn'])")
python3 "$WS/scripts/ab/metrics.py" --run-dir "$OUT" \
  --model pickup_box_0001_inch20 --t-spawn-sim "$T_SPAWN" \
  > "$OUT/metrics.log" 2>&1 || true

# ---- Teardown (own processes only) + record residual check. ----
kill -INT "$LAUNCH_PID" 2>/dev/null || true
sleep 8
kill -TERM "${CAP_PIDS[@]}" 2>/dev/null || true
sleep 2
RESIDUAL=$(pgrep -f "gz sim.*$WORLD" | wc -l)
echo "{\"stage\":\"teardown\",\"residual_gz\":$RESIDUAL,\"ts\":\"$(date -Is)\"}" >> "$OUT/lifecycle.jsonl"
wc -c "$OUT/dyn_pose_stream.txt" "$OUT/contact_stream.txt" \
  "$OUT/tool_stream.jsonl" || true
echo "RUN $N ($ARM) COMPLETE -> $OUT"
