#!/usr/bin/env bash
# ST06 atlas build campaign: four profiles, exclusive stack, gate + FK cert.
# Usage: bash run_campaign.sh <worktree-root> <evidence-dir>
set -o pipefail
WS="$(cd "$1" && pwd)"
EV="$(cd "$2" && pwd)"
cd "$WS"
set +u  # ROS setup scripts reference unbound tracing variables.
source /opt/ros/humble/setup.bash
# Default-base install: sim_world.launch resolves its GPU-gate script through
# the install prefix's grandparent, which must be the worktree root.
source "$WS/install/local_setup.bash"
export ROS_DOMAIN_ID=7

PROFILES=(
  "s20_container_collision_aware|"
  "s20_container_collision_aware_payload_0.55x0.40x0.25|0.55,0.40,0.25"
  "s20_container_collision_aware_payload_0.70x0.45x0.28|0.70,0.45,0.28"
  "s20_container_collision_aware_payload_0.80x0.50x0.32|0.80,0.50,0.32"
)

mkdir -p "$EV/old" "$EV/logs" "$EV/fk"
echo "{\"stage\": \"campaign-start\", \"ts\": \"$(date -Is)\", \"ws\": \"$WS\"}" \
  > "$EV/lifecycle.jsonl"

# 1. Preserve the v1 artifacts for the gate comparison (evidence too).
#    Restored from git so a rerun cannot compare against an already-rebuilt
#    file; the shape guard refuses anything that is not the 2-bin v1.
for row in "${PROFILES[@]}"; do
  name="${row%%|*}"
  git show "HEAD:src/luggage_planning/data/reachability_atlas/$name.npz" > "$EV/old/$name.npz" || exit 1
  git show "HEAD:src/luggage_planning/data/reachability_atlas/$name.yaml" > "$EV/old/$name.yaml" || exit 1
done
python3 - "$EV" <<'PYGUARD'
import sys, numpy as np
ev = sys.argv[1]
names = [
    "s20_container_collision_aware",
    "s20_container_collision_aware_payload_0.55x0.40x0.25",
    "s20_container_collision_aware_payload_0.70x0.45x0.28",
    "s20_container_collision_aware_payload_0.80x0.50x0.32",
]
for name in names:
    shape = np.load(ev + "/old/" + name + ".npz")["status"].shape
    if shape[3] != 2:
        raise SystemExit("old artifact %s has %d yaw bins (want v1=2)" % (name, shape[3]))
    print("v1 ok", name, shape)
PYGUARD
echo "{\"stage\": \"v1-preserved\", \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"

# 2. Exclusive stack: gazebo + RSP + move_group + scene_manager, no GUI.
(ros2 launch luggage_gazebo sim_world.launch.py \
    gui:=false use_rviz:=false use_motion:=true \
    > "$EV/logs/launch.log" 2>&1 &)
sleep 5
LAUNCH_PID="$(pgrep -f 'ros2 launch luggage_gazebo sim_world.launch.py' | head -1)"
if [[ -z "$LAUNCH_PID" ]]; then
  echo "{\"stage\": \"launch-failed\", \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
  exit 1
fi
echo "{\"stage\": \"launch\", \"pid\": $LAUNCH_PID, \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"

# 3. Wait for the two services the builder needs.
for i in $(seq 1 60); do
  SV1="$(ros2 service list 2>/dev/null | grep -c '^/check_state_validity' || true)"
  SV2="$(ros2 service list 2>/dev/null | grep -c '^/scene_manager/sync_static_scene' || true)"
  if [[ "$SV1" -ge 1 && "$SV2" -ge 1 ]]; then break; fi
  sleep 5
done
echo "{\"stage\": \"services-ready\", \"check_state_validity\": $SV1,
       \"sync_static_scene\": $SV2, \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
if [[ "$SV1" -lt 1 || "$SV2" -lt 1 ]]; then
  echo "services did not come up" >&2
  bash scripts/stop_sim.sh || true
  exit 1
fi

# 4. Build each profile with the regression gate against the v1 copy.
STATUS=0
for row in "${PROFILES[@]}"; do
  name="${row%%|*}"; payload="${row#*|}"
  args=(--output-dir src/luggage_planning/data/reachability_atlas
        --gate --old-atlas "$EV/old/$name")
  if [[ -n "$payload" ]]; then args+=(--payload "$payload"); fi
  echo "{\"stage\": \"build\", \"profile\": \"$name\", \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
  if python3 scripts/build_reachability_atlas_humble.py "${args[@]}" \
      > "$EV/logs/$name.log" 2>&1; then
    echo "{\"stage\": \"build-ok\", \"profile\": \"$name\", \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
  else
    rc=$?
    echo "{\"stage\": \"build-fail\", \"profile\": \"$name\", \"rc\": $rc, \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
    STATUS=1
    break
  fi
done

# 5. FK-certify every rebuilt artifact (offline; stack may stay up).
if [[ "$STATUS" -eq 0 ]]; then
  for row in "${PROFILES[@]}"; do
    name="${row%%|*}"
    echo "{\"stage\": \"certify\", \"profile\": \"$name\", \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
    if python3 scripts/certify_atlas_seeds.py \
        --atlas "src/luggage_planning/data/reachability_atlas/$name" \
        --urdf-file src/luggage_planning/data/reachability_atlas/robot_description.xml \
        --output "$EV/fk/$name" > "$EV/logs/$name.certify.log" 2>&1; then
      echo "{\"stage\": \"certify-ok\", \"profile\": \"$name\", \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
    else
      echo "{\"stage\": \"certify-fail\", \"profile\": \"$name\", \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
      STATUS=1
    fi
  done
fi

# 6. Teardown the stack this campaign started.
bash scripts/stop_sim.sh > "$EV/logs/stop_sim.log" 2>&1 || true
pkill -f "ros2 launch luggage_gazebo sim_world.launch.py" 2>/dev/null || true
echo "{\"stage\": \"campaign-done\", \"status\": $STATUS, \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"

# 7. Scene identity continuity vs the v1 artifacts (strict runtime gate).
python3 - "$EV" <<'PYEOF'
import sys, yaml, json
ev = sys.argv[1]
def dep(path):
    meta = yaml.safe_load(open(path))
    c = meta.get("contract") or {}
    return meta.get("dependencies", {}).get("scene_tf_hash"), c.get("scene_revision")
old = dep(ev + "/old/s20_container_collision_aware.yaml")
new = dep("src/luggage_planning/data/reachability_atlas/s20_container_collision_aware.yaml")
print(json.dumps({"old": old, "new": new,
                  "scene_identity_continuous": old == new and bool(new[0])}, indent=1))
PYEOF
exit "$STATUS"
