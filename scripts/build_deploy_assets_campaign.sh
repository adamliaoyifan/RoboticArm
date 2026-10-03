#!/usr/bin/env bash
# ST09-PD deploy asset campaign: four atlases + four route graphs + the
# identity manifest, sampled against the deploy scene through the
# no-Gazebo planning-scene stack. No hardware motion.
# Usage: bash scripts/build_deploy_assets_campaign.sh <ws-root> <evidence-dir>
set -o pipefail -e
WS="$(cd "$1" && pwd)"
mkdir -p "$2"
EV="$(cd "$2" && pwd)"
cd "$WS"
set +u  # ROS setup scripts reference unbound tracing variables.
source /opt/ros/humble/setup.bash
source "$WS/install/local_setup.bash"
export ROS_DOMAIN_ID=17

SCENE="$WS/src/luggage_description/config/scene_tf_deploy.yaml"
DATA="$WS/src/luggage_planning/data/reachability_atlas_deploy"

PROFILES=(
  "s20_container_collision_aware|"
  "s20_container_collision_aware_payload_0.55x0.40x0.25|0.55,0.40,0.25"
  "s20_container_collision_aware_payload_0.70x0.45x0.28|0.70,0.45,0.28"
  "s20_container_collision_aware_payload_0.80x0.50x0.32|0.80,0.50,0.32"
)

mkdir -p "$EV/logs" "$EV/fk"
echo "{\"stage\": \"campaign-start\", \"ts\": \"$(date -Is)\", \"ws\": \"$WS\", \"scene_md5\": \"$(md5sum "$SCENE" | cut -d' ' -f1)\"}" \
  > "$EV/lifecycle.jsonl"

# 1. Refuse to launch over same-domain orphans (a surviving move_group/RSP
#    answers service discovery and races the builder with a stale scene).
#    Processes on other ROS domains (other agents' cells) are not ours.
RESIDUALS=""
for pid in $(pgrep -f 'moveit_ros_move_group|robot_state_publisher|deploy_asset_stack|ros2 launch luggage_planning' || true); do
  if tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -q "^ROS_DOMAIN_ID=$ROS_DOMAIN_ID$"; then
    RESIDUALS="$RESIDUALS $pid"
  fi
done
if [[ -n "${RESIDUALS// }" ]]; then
  echo "{\"stage\": \"residual-stack-refused\", \"pids\": \"$RESIDUALS\", \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
  echo "residual planning-stack processes on domain $ROS_DOMAIN_ID ($RESIDUALS); refusing to launch" >&2
  exit 1
fi

# 2. Offline stack: RSP + static joint_states + move_group + scene_manager.
#    No Gazebo anywhere in this chain.
rm -rf "$DATA"
(ros2 launch luggage_planning deploy_asset_stack.launch.py \
    > "$EV/logs/launch.log" 2>&1 &)
sleep 5
LAUNCH_PID="$(pgrep -f 'ros2 launch luggage_planning deploy_asset_stack' | head -1)"
if [[ -z "$LAUNCH_PID" ]]; then
  echo "{\"stage\": \"launch-failed\", \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
  cat "$EV/logs/launch.log" >&2 || true
  exit 1
fi
echo "{\"stage\": \"launch\", \"pid\": $LAUNCH_PID, \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"

cleanup() {
  # Kill only this campaign's own domain; other agents' cells are not ours.
  pkill -f 'ros2 launch luggage_planning deploy_asset_stack' 2>/dev/null || true
  sleep 2
  for pattern in 'moveit_ros_move_group' 'static_joint_state_publisher' \
                 'scene_manager' 'robot_state_publisher'; do
    for pid in $(pgrep -f "$pattern" || true); do
      if tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -q "^ROS_DOMAIN_ID=$ROS_DOMAIN_ID$"; then
        kill "$pid" 2>/dev/null || true
      fi
    done
  done
}
trap cleanup EXIT

# 3. Wait for the two services the builders need.
for i in $(seq 1 60); do
  SV1="$(ros2 service list 2>/dev/null | grep -c '^/check_state_validity' || true)"
  SV2="$(ros2 service list 2>/dev/null | grep -c '^/scene_manager/sync_static_scene' || true)"
  if [[ "$SV1" -ge 1 && "$SV2" -ge 1 ]]; then break; fi
  sleep 5
done
sleep 10
SV1="$(ros2 service list 2>/dev/null | grep -c '^/check_state_validity' || true)"
SV2="$(ros2 service list 2>/dev/null | grep -c '^/scene_manager/sync_static_scene' || true)"
echo "{\"stage\": \"services-ready\", \"check_state_validity\": $SV1, \"sync_static_scene\": $SV2, \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
if [[ "$SV1" -lt 1 || "$SV2" -lt 1 ]]; then
  echo "services did not come up" >&2
  tail -50 "$EV/logs/launch.log" >&2 || true
  exit 1
fi

# 4. Build the four atlas profiles against the deploy scene/URDF.
STATUS=0
for row in "${PROFILES[@]}"; do
  name="${row%%|*}"; payload="${row#*|}"
  args=(--output-dir "$DATA" --scene-tf "$SCENE")
  if [[ -n "$payload" ]]; then args+=(--payload "$payload"); fi
  echo "{\"stage\": \"atlas-build\", \"profile\": \"$name\", \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
  if python3 scripts/build_reachability_atlas_humble.py "${args[@]}" \
      > "$EV/logs/$name.log" 2>&1; then
    echo "{\"stage\": \"atlas-build-ok\", \"profile\": \"$name\", \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
  else
    rc=$?
    echo "{\"stage\": \"atlas-build-fail\", \"profile\": \"$name\", \"rc\": $rc, \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
    STATUS=1
    break
  fi
done

# 5. FK-certify every artifact against the captured robot_description.
if [[ "$STATUS" -eq 0 ]]; then
  for row in "${PROFILES[@]}"; do
    name="${row%%|*}"
    echo "{\"stage\": \"fk-certify\", \"profile\": \"$name\", \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
    if python3 scripts/certify_atlas_seeds.py \
        --atlas "$DATA/$name" \
        --urdf-file "$DATA/robot_description.xml" \
        --output "$EV/fk/$name" > "$EV/logs/$name.certify.log" 2>&1; then
      echo "{\"stage\": \"fk-certify-ok\", \"profile\": \"$name\", \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
    else
      echo "{\"stage\": \"fk-certify-fail\", \"profile\": \"$name\", \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
      STATUS=1
    fi
  done
fi

# 6. Build the four directed route graphs (still against the stack).
if [[ "$STATUS" -eq 0 ]]; then
  echo "{\"stage\": \"graph-build\", \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
  if python3 scripts/build_deploy_route_graphs.py \
      --data-dir "$DATA" --scene-tf "$SCENE" \
      --output "$DATA/graphs/summary.json" \
      > "$EV/logs/graphs.log" 2>&1; then
    echo "{\"stage\": \"graph-build-ok\", \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
  else
    rc=$?
    echo "{\"stage\": \"graph-build-fail\", \"rc\": $rc, \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
    STATUS=1
  fi
fi

# 7. Generate the identity manifest from the finished artifacts.
if [[ "$STATUS" -eq 0 ]]; then
  echo "{\"stage\": \"manifest\", \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
  if python3 scripts/generate_deploy_asset_manifest.py \
      > "$EV/logs/manifest.log" 2>&1; then
    echo "{\"stage\": \"manifest-ok\", \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
  else
    rc=$?
    echo "{\"stage\": \"manifest-fail\", \"rc\": $rc, \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
    STATUS=1
  fi
fi

echo "{\"stage\": \"campaign-done\", \"status\": $STATUS, \"ts\": \"$(date -Is)\"}" >> "$EV/lifecycle.jsonl"
exit $STATUS
