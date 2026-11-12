#!/usr/bin/env bash
# Interactive scene only: no automatic pick/place commands. Ctrl+C tears down.
set -eo pipefail
UPPER_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
UPPER_COORD=/home/adamliao/work/elfin_humble_ws
UPPER_INSTALL="${ULD_UPPER_INSTALL:-$UPPER_REPO/install}"
UPPER_SCENE="${ULD_UPPER_SCENE:-/tmp/uld_upper_layer_scene_v1_aligned}"
UPPER_GUI=true
if [[ "${1:-}" == --headless ]]; then UPPER_GUI=false; shift; fi
if (( $# )); then echo 'Usage: launch_scene.sh [--headless]' >&2; exit 2; fi
if [[ ! -f "$UPPER_INSTALL/setup.bash" ]]; then
  echo "Build the isolated overlay first; see $UPPER_REPO/research/uld_upper_layer/README.md" >&2
  exit 2
fi
python3 - <<'PY'
import subprocess
for line in subprocess.check_output(['ps','-eo','pid,comm,args'],text=True).splitlines()[1:]:
    fields=line.split(None,2)
    if len(fields)<3 or fields[1] in ('bash','sh','python3','timeout','codex-linux-san'): continue
    if any(token in fields[2] for token in ('ign gazebo','ign-gazebo-server','gz sim',
            'ros2 launch luggage_gazebo','sim_world.launch.py','parameter_bridge')):
        raise SystemExit('Another sim or bridge is active; wait for its owner: '+line[:240])
PY
source /opt/ros/humble/setup.bash
source "$UPPER_COORD/install/setup.bash"
source "$UPPER_INSTALL/setup.bash"
if [[ ! -f "$UPPER_SCENE/fixture.json" ]]; then
  python3 "$UPPER_REPO/research/uld_upper_layer/scene.py" --output "$UPPER_SCENE"
fi
# Fail closed if an unrelated/old overlay was accidentally sourced.
UPPER_SHARE="$(ros2 pkg prefix --share luggage_gazebo)"
if [[ "$UPPER_SHARE" != "$UPPER_INSTALL"/* ]]; then
  echo "Wrong luggage_gazebo overlay: $UPPER_SHARE" >&2; exit 2
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/uld-upper-scene-ros-logs}"
export IGN_PARTITION="${IGN_PARTITION:-uld_upper_layer_scene}"
UPPER_PIDFILE=/tmp/elfin_uld_upper_scene.pid
setsid ros2 launch "$UPPER_REPO/src/luggage_gazebo/launch/sim_world.launch.py" \
  world_sdf:="$UPPER_SCENE/overlay.sdf" \
  scene_tf_config:="${ULD_UPPER_SCENE_CONFIG:-$UPPER_REPO/src/luggage_description/config/scene_tf.yaml}" \
  gui:="$UPPER_GUI" use_rviz:=false use_moveit:=true \
  use_perception:=false use_semantic:=false use_motion:=false \
  use_packing:=false use_cargo_map:=false use_vacuum:=false &
UPPER_LAUNCH_PID=$!
printf '%s\n' "$UPPER_LAUNCH_PID" > "$UPPER_PIDFILE"
trap '"$UPPER_COORD/scripts/stop_sim.sh" --owned-pgid "$UPPER_LAUNCH_PID" --pidfile "$UPPER_PIDFILE"' EXIT
trap 'exit 130' INT TERM
wait "$UPPER_LAUNCH_PID"
