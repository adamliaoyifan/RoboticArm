#!/usr/bin/env bash
# Non-scored contact-chain preflight (batch-2 precondition).
# Proves: (1) boot discipline v2 — previous sim swept BEFORE boot;
# (2) the FIXED record_box_stream.py runs and subscribes;
# (3) a dynamically spawned box produces non-empty box_contacts.jsonl.
# No arm replay, no metrics, occupies no batch slot.
set -euo pipefail
WS="/tmp/elfin-master-pack-baseline-g3"
MAIN="/home/adamliao/work/elfin_humble_ws"
OBS="$MAIN/docs/status/evidence/master_pack_recovery/2026-10-14_g3/obs_rel_07"
OUT="$WS/docs/status/evidence/occ_owner_b1/preflight_contact_$(date +%H%M%S)"
mkdir -p "$OUT"
LOG() { echo "[$(date +%T)] $*" | tee -a "$OUT/preflight.log"; }

# 1) Boot discipline v2 — same sweep as run_one.sh.
LOG "sweep previous sim"
bash "$WS/scripts/stop_sim.sh" --all >> "$OUT/bootsweep.log" 2>&1 || true
for pid in $(pgrep -x parameter_bridge); do kill -KILL "$pid" 2>/dev/null || true; done
sleep 1
if pgrep -x ign >/dev/null 2>&1 || pgrep -x gz >/dev/null 2>&1 \
   || pgrep -f '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch luggage_gazebo' >/dev/null 2>&1 \
   || pgrep -x parameter_bridge >/dev/null 2>&1; then
  LOG "FAIL: sim slot busy after sweep"
  echo '{"pass": false, "stage": "sweep"}' > "$OUT/verdict.json"
  exit 3
fi
LOG "slot clean"

set +u
source /opt/ros/humble/setup.bash
source "$MAIN/install/setup.bash"
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID=7

IGN_BIN="$(command -v ign || command -v gz)"

# 2) Boot the PROVEN launch set (A0 arm's scene).
LOG "launch sim"
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
  scene_tf_config:="$MAIN/src/luggage_description/config/scene_tf_deploy_update.yaml" \
  atlas_runtime_config:="$WS/src/luggage_planning/config/atlas_runtime_deploy_update.yaml" \
  robot_poses_config:="$WS/src/luggage_description/config/robot_poses.baseline_update.yaml" \
  frame_runtime:=true frame_driver_peer:=pack_eval_driver \
  > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
LOG "launch pid $LAUNCH_PID"

for _ in $(seq 1 120); do
  grep -q "Controller manager: loaded" "$OUT/launch.log" 2>/dev/null && break
  sleep 2
done
sleep 5

# 3) Bridge + FIXED recorder, then verify subscription exists.
IGN_BIN="$IGN_BIN" python3 "$WS/scripts/ab/spawn_contact_bridge.py" &
BRIDGE_PID=$!
sleep 2
python3 "$WS/scripts/ab/record_box_stream.py" \
  "$OUT/box_contacts.jsonl" pickup_box_0001_inch20 > "$OUT/recorder.log" 2>&1 &
REC_PID=$!
sleep 3
SUB_OK=$(ros2 node info /ab_box_recorder 2>/dev/null | grep -c "/obs/box_contact" || true)
LOG "recorder subscription lines: $SUB_OK"

# 4) Spawn the frozen dynamic box; let it fall and settle.
python3 "$WS/scripts/ab/preflight_spawn_box.py" "$OBS/sdf/release.sdf" \
  "$OUT/spawn.json" >> "$OUT/preflight.log" 2>&1 || true
LOG "spawn result: $(cat "$OUT/spawn.json" 2>/dev/null || echo none)"
sleep 25

# 5) Teardown FIRST (stop the flow), then inspect content.
kill -TERM "$REC_PID" "$BRIDGE_PID" 2>/dev/null || true
sleep 2
kill -INT "$LAUNCH_PID" 2>/dev/null || true
sleep 8
bash "$WS/scripts/stop_sim.sh" --all >> "$OUT/bootsweep.log" 2>&1 || true
for pid in $(pgrep -x parameter_bridge); do kill -KILL "$pid" 2>/dev/null || true; done
RESID=$(pgrep -x ign | wc -l)

python3 - "$OUT" "$SUB_OK" "$RESID" << 'PY' | tee -a "$OUT/preflight.log"
import json, sys
out, sub_ok, resid = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
path = out + "/box_contacts.jsonl"
lines, contact_rows = 0, 0
try:
    for raw in open(path):
        lines += 1
        if '"t_sim_ns"' in raw:
            contact_rows += 1
except FileNotFoundError:
    pass
verdict = {
    "pass": bool(lines >= 1 and contact_rows >= 1 and sub_ok >= 1
                 and json.load(open(out + "/spawn.json"))["spawned"]
                 and resid == 0),
    "recorder_file_lines": lines,
    "contact_rows": contact_rows,
    "recorder_subscribed": sub_ok >= 1,
    "spawned": json.load(open(out + "/spawn.json"))["spawned"],
    "residual_ign_after_teardown": resid,
}
json.dump(verdict, open(out + "/verdict.json", "w"), indent=2)
print(json.dumps(verdict))
PY
LOG "verdict at $OUT/verdict.json"
