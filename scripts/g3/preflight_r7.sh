#!/usr/bin/env bash
# r7 NON-SCORED preflight (user quota: <=30 min cumulative, no packing).
# Verifies: boot discipline ownership, service readiness, the pose/info
# stream freshness against /clock, the FIVE-collision contact-recess SDF
# accepted by gz with a LIVE multi-body sensor, and teardown cleanliness.
# Any failure -> stop and report the blocker; r7 is NOT consumed.
set -euo pipefail
WS="/tmp/elfin-master-pack-baseline-g3"
MAIN="/home/adamliao/work/elfin_humble_ws"
OBS="$MAIN/docs/status/evidence/master_pack_recovery/2026-10-14_g3/obs_rel_07"
OUT="$WS/docs/status/evidence/occ_owner_b1/gate3_aprime/preflight_r7_$(date +%H%M%S)"
mkdir -p "$OUT"
LOG() { echo "[$(date +%T)] $*" | tee -a "$OUT/preflight.log"; }

# 1) Boot discipline: sweep previous sim, then verify the slot.
LOG "sweep previous sim"
bash "$WS/scripts/stop_sim.sh" --all >> "$OUT/bootsweep.log" 2>&1 || true
if ! pgrep -f 'ros2 launc[h]' >/dev/null 2>&1; then
  pkill -TERM -f 'opt/ros/humble/li[b]' 2>/dev/null || true
  pkill -TERM -f 'install/luggage_.*/li[b]' 2>/dev/null || true
  sleep 4
  pkill -KILL -f 'opt/ros/humble/li[b]' 2>/dev/null || true
  pkill -KILL -f 'install/luggage_.*/li[b]' 2>/dev/null || true
  N_SHM=$(ls /dev/shm 2>/dev/null | grep -c '^fastrtps' || true)
  [[ "${N_SHM:-0}" -gt 0 ]] && rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
fi
if pgrep -x ign >/dev/null 2>&1 || pgrep -x gz >/dev/null 2>&1 \
   || pgrep -x parameter_bridge >/dev/null 2>&1; then
  LOG "FAIL: sim slot busy after sweep"; exit 3
fi
LOG "slot clean"

set +u
source /opt/ros/humble/setup.bash
source "$MAIN/install/setup.bash"
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID=7
export ELFIN_BOX_CONTACT_TOPIC=/obs/box_contact
IGN_BIN="$(command -v ign || command -v gz)"
SCENE="$OUT/scene_tf_used.yaml"
python3 - "$SCENE" "$OUT/robot_poses_used.yaml" << 'PYSCENE'
import hashlib, os, sys, yaml
cfg = yaml.safe_load(open(
    "/tmp/elfin-master-pack-baseline-g3/src/luggage_description/config/"
    "scene_tf_deploy_update.yaml"))
cfg.setdefault("gazebo", {})["container_model"] = \
    "airport_container_real_aprime"
yaml.safe_dump(cfg, open(sys.argv[1], "w"), sort_keys=False)
scene_md5 = hashlib.md5(open(sys.argv[1], "rb").read()).hexdigest()
poses = yaml.safe_load(open(
    "/tmp/elfin-master-pack-baseline-g3/src/luggage_description/config/"
    "robot_poses.baseline_update.yaml"))
poses.setdefault("identity", {})["scene_revision"] = scene_md5
poses["identity"]["_rebind_note"] = "preflight r7 scene rebind"
for _n, pose in (poses.get("poses") or {}).items():
    if isinstance(pose, dict) and pose.get("scene_revision_at_capture"):
        pose["scene_revision_at_capture"] = scene_md5
yaml.safe_dump(poses, open(sys.argv[2], "w"), sort_keys=False)
PYSCENE

LOG "launch sim (A-prime, r7 deployment)"
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
  robot_poses_config:="$OUT/robot_poses_used.yaml" \
  frame_runtime:=true frame_driver_peer:=pack_eval_driver \
  > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
for _ in $(seq 1 120); do
  grep -q "Held pose 'pickup_observe_sim'" "$OUT/launch.log" 2>/dev/null && break
  sleep 2
done
grep -q "Held pose" "$OUT/launch.log" || { LOG "FAIL: observe pose not held"; kill -INT $LAUNCH_PID; exit 3; }
LOG "observe held"

# 2) Service readiness (create/remove/follow action).
ros2 service list > "$OUT/services.txt" 2>/dev/null || true
for svc in "/world/airport_loading/create" "/world/airport_loading/remove"; do
  grep -q "$svc" "$OUT/services.txt" || { LOG "FAIL: missing $svc"; kill -INT $LAUNCH_PID; exit 3; }
done
LOG "services ready"

# 3) Pose/info stream: spawn the FIVE-collision recess box (production
# SDF builder, frozen pose), then sample pose/info + /clock freshness
# and the multi-body contact sensor.
python3 - "$OUT" << 'PYSPAWN' | tee -a "$OUT/preflight.log"
import json, os, sys
out = sys.argv[1]
sys.path.insert(0, "/tmp/elfin-master-pack-baseline-g3/install/"
                 "luggage_description/local/lib/python3.10/dist-packages")
from luggage_description.suitcase_visual import pickup_visual_sdf
sdf = pickup_visual_sdf(
    "preflight_box_inch20", (0.35, 0.53, 0.22), 5.67, "suitcase_loafbrr",
    static=False, contact_topic="/obs/box_contact", collide_bitmask=255,
    collision_kind="box_recess", lid_offset=0.011033)
open(out + "/recess_box.sdf", "w").write(sdf)
n = sdf.count("<collision name=")
print("recess SDF collisions:", n)
assert n == 5, "SDF builder must emit five collisions"
PYSPAWN
sed -i 's/preflight_box_inch20/pickup_box_0001_inch20/' "$OUT/recess_box.sdf"
python3 - "$OUT" << 'PYSPAWN2' >> "$OUT/preflight.log" 2>&1
import json, sys, time
out = sys.argv[1]
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity
import math
rclpy.init()
node = Node("r7_preflight_spawn")
cli = node.create_client(SpawnEntity, "/world/airport_loading/create")
cli.wait_for_service(timeout_sec=30.0)
req = SpawnEntity.Request()
req.entity_factory.name = "pickup_box_0001_inch20"
req.entity_factory.sdf = open(out + "/recess_box.sdf").read()
req.entity_factory.pose.position.x = -1.98001
req.entity_factory.pose.position.y = 0.460003
req.entity_factory.pose.position.z = 0.691353
req.entity_factory.pose.orientation.w = 0.706982
req.entity_factory.pose.orientation.x = -0.0136845
req.entity_factory.pose.orientation.y = 0.0119861
req.entity_factory.pose.orientation.z = 0.707117
fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=20.0)
res = fut.result()
print(json.dumps({"spawned": bool(res and res.success),
                  "msg": str(getattr(res, "status_message", ""))}))
node.destroy_node(); rclpy.shutdown()
PYSPAWN2
sleep 8

# pose/info freshness: sample stream, compare stamps to /clock.
timeout 6 "$IGN_BIN" topic -e -t /world/airport_loading/pose/info --num 3 \
  > "$OUT/pose_sample.txt" 2>/dev/null || true
timeout 4 "$IGN_BIN" topic -e -t /clock --num 2 > "$OUT/clock_sample.txt" 2>/dev/null || true
python3 - "$OUT" << 'PYPOSE' | tee -a "$OUT/preflight.log"
import re, sys
out = sys.argv[1]
pose = open(out + "/pose_sample.txt").read()
clock = open(out + "/clock_sample.txt").read()
def latest_stamp(text):
    sec = re.findall(r"(?m)^\s*sec: (\d+)\s*$", text)
    nsec = re.findall(r"(?m)^\s*nsec: (\d+)\s*$", text)
    if not sec:
        return None
    return int(sec[-1]) * 10**9 + int(nsec[-1] if nsec else 0)
ps, cs = latest_stamp(pose), latest_stamp(clock)
print("pose stream latest stamp ns:", ps, " clock ns:", cs)
assert ps is not None, "pose/info stream is silent"
assert cs is not None, "/clock is silent"
age_ms = (cs - ps) / 1e6
print("pose stream vs clock age ms:", round(age_ms, 1))
assert abs(age_ms) <= 500.0, "pose stream stale vs clock"
print("POSE_STREAM_FRESH")
PYPOSE

# multi-body contact sensor: the resting box must produce contacts that
# name the recess bodies.
timeout 8 "$IGN_BIN" topic -e -t /obs/box_contact --num 5 \
  > "$OUT/contact_sample.txt" 2>/dev/null || true
python3 - "$OUT" << 'PYCONTACT' | tee -a "$OUT/preflight.log"
import re, sys
out = sys.argv[1]
text = open(out + "/contact_sample.txt").read()
bodies = set(re.findall(
    r"suitcase_link::(\w+)", text))
print("contact-bearing bodies:", sorted(bodies))
assert bodies, "no contacts on /obs/box_contact"
assert "body_collision" in bodies or any(b.startswith("rim_") for b in bodies), \
    "sensor not reporting recess bodies"
print("CONTACT_SENSOR_LIVE")
PYCONTACT

# 4) Teardown ownership + cleanliness.
kill -INT "$LAUNCH_PID" 2>/dev/null || true
sleep 8
bash "$WS/scripts/stop_sim.sh" --all >> "$OUT/bootsweep.log" 2>&1 || true
for pid in $(pgrep -x parameter_bridge); do kill -KILL "$pid" 2>/dev/null || true; done
RESID=$(( $(pgrep -x ign | wc -l || true) ))
N_SHM=$(ls /dev/shm 2>/dev/null | grep -c '^fastrtps' || true)
[[ "${N_SHM:-0}" -gt 0 ]] && rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
LOG "teardown: residual_ign=$RESID shm_cleared=$N_SHM"
[[ "$RESID" -eq 0 ]] || { LOG "FAIL: residual ign after teardown"; exit 3; }
LOG "PREFLIGHT PASS -> $OUT/verdict.json"
python3 -c "import json; json.dump({'pass': True, 'out': '$OUT'}, open('$OUT/verdict.json','w'), indent=2)"
