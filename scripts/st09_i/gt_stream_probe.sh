#!/usr/bin/env bash
# Non-scored GT collector check. One spawned carry-on, no place motion.
set -uo pipefail
OUT=""
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --out) OUT="$2"; shift 2 ;;
    --ws) WS="$2"; shift 2 ;;
    *) echo "unknown arg $1" >&2; exit 2 ;;
  esac
done
[[ -n "$OUT" ]] || { echo "--out required" >&2; exit 2; }
if [[ -e "$OUT" ]]; then
  echo "refusing existing output directory: $OUT" >&2
  exit 2
fi
mkdir -p "$OUT"
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH || true
set +u
source /opt/ros/humble/setup.bash
source "$WS/install-st09/setup.bash"
set -u
export ROS_DOMAIN_ID=7 ROS_LOCALHOST_ONLY=1
DRIVER="$WS/install-st09/luggage_gazebo/lib/luggage_gazebo/m3_three_box_driver.py"
SCENE="$WS/install-st09/luggage_description/share/luggage_description/config/scene_tf_deploy.yaml"
bash "$WS/scripts/st09_i/kill_stack.sh" || true
residuals=$(pgrep -af 'ros2 launch luggage_gazebo|ign gazebo|gz sim|ros_gz_bridge/parameter_bridge|clock_bridge|camera_bridge' 2>/dev/null | grep -v -e 'bash -c' -e 'gt_stream_probe.sh' | wc -l | tr -d ' ')
if [[ "$residuals" -ne 0 ]]; then
  echo "residual sim/bridge processes ($residuals); refusing to launch" >&2
  exit 1
fi
ros2 launch luggage_gazebo sim_world.launch.py \
  scene_tf_config:="$SCENE" \
  gui:=false use_rviz:=false \
  use_vacuum:=false use_motion:=false use_semantic:=false \
  use_cargo_map:=false use_packing:=false \
  frame_runtime:=false \
  sequence_ids:=carryon \
  > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
echo "$LAUNCH_PID" > /tmp/elfin_humble_sim.pid
rc=0
python3 "$WS/scripts/st09_i/gt_stream_probe.py" \
  --out "$OUT" \
  --driver "$DRIVER" \
  --expect pickup_box_0001_carryon \
  > "$OUT/probe.log" 2>&1 || rc=$?
kill -INT "$LAUNCH_PID" 2>/dev/null || true
grace=0
while kill -0 "$LAUNCH_PID" 2>/dev/null && (( grace < 60 )); do
  sleep 3
  grace=$((grace + 3))
done
bash "$WS/scripts/st09_i/kill_stack.sh" || true
wait "$LAUNCH_PID" 2>/dev/null || true
rm -f /tmp/elfin_humble_sim.pid
exit "$rc"
