#!/usr/bin/env bash
# 实机代码 (site / real-cell). Not part of the Gazebo simulation stack.
#
# Start the current live perception graph and record one static case.
# Does not start hardware_pick_driver. The arm stays where it is.
# Person on e-stop. Ctrl+C stops the bag and the graph (no BlackOut).
#
# Perception path:
#   D555 raw image_raw (compress off, 640x360@15). Driver stays raw.
#   Preprocessor subscribes that raw pair (use_compressed:=false).
#   YOLO uncapped (max_rate_hz:=0), overlay off.
#   Livox Mid-360, CPS /joint_states, scene TF.
#
#   cd deployment_ws
#   ./scripts/record_perception_case.sh -n a5_carryon_C_0deg_01
#   ./scripts/record_perception_case.sh -n a5_carryon_C_0deg_01 -t 15
#   ./scripts/record_perception_case.sh -n observe_hold -t 0
#
# Do not run this while Orin already owns D555, Livox, and YOLO.
# Do not ros2 bag play onto ROS_DOMAIN_ID=7. Replay reads the raw images.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: record_perception_case.sh -n NAME [-o DIR] [-t SEC] [--note TEXT]

  Start D555 (raw) + Livox + the current hardware_pick perception graph,
  record one bag, then stop the graph. Does not move the arm.

  -n, --name NAME   bag folder name (required), e.g. a5_carryon_C_0deg_01
  -o, --out DIR     parent directory (default: ~/robotarm_bags/pickup_cases)
  -t, --duration S  record S seconds then stop (default: 15). 0 = until Ctrl+C
      --note TEXT   one-line comment written into notes.yaml
  -h, --help

Place the box, keep hands out of frame, arm still, then start.
One layout = one -n. Do not ros2 bag play on ROS_DOMAIN_ID=7.
EOF
}

OUT_DIR="${HOME}/robotarm_bags/pickup_cases"
BAG_NAME=""
DURATION="15"
NOTE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    -n|--name) BAG_NAME="$2"; shift 2 ;;
    -o|--out) OUT_DIR="$2"; shift 2 ;;
    -t|--duration) DURATION="$2"; shift 2 ;;
    --note) NOTE="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "$BAG_NAME" ]]; then
  echo "need -n NAME (one bag per box layout)" >&2
  usage >&2
  exit 2
fi
if [[ "$BAG_NAME" == *"/"* || "$BAG_NAME" == "."* ]]; then
  echo "NAME must be a single folder component, got: $BAG_NAME" >&2
  exit 2
fi
if [[ ! "$DURATION" =~ ^[0-9]+$ ]]; then
  echo "duration must be a non-negative integer, got: $DURATION" >&2
  exit 2
fi

DEPLOY="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="$(cd "$DEPLOY/.." && pwd)"
ENV_SH="${DEPLOY}/scripts/env_jazzy_real.sh"
PICK="${DEPLOY}/scripts/hardware_pick.sh"
QOS="${DEPLOY}/src/elfin_trajectory_executor/config/bag_qos_overrides.yaml"
LIVOX_CFG="${DEPLOY}/config/MID360s_config.json"
if [[ ! -f "$QOS" ]]; then
  QOS="${DEPLOY}/install/elfin_trajectory_executor/share/elfin_trajectory_executor/config/bag_qos_overrides.yaml"
fi

if [[ ! -f "$ENV_SH" || ! -f "$PICK" ]]; then
  echo "missing env or hardware_pick.sh under $DEPLOY" >&2
  exit 1
fi
if [[ ! -f "$LIVOX_CFG" ]]; then
  echo "missing Livox config: $LIVOX_CFG" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1090
source "$ENV_SH"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
unset ROS_LOCALHOST_ONLY

if pgrep -f 'd555_rgbd.launch.py|mid360.launch.py|hardware_pick.launch.py|livox_ros_driver2_node|semantic_segmenter_node.py|ros2 bag record' >/dev/null 2>&1; then
  echo "a camera, Livox, pick graph, or bag record is already running. Stop it first." >&2
  pgrep -af 'd555_rgbd.launch.py|mid360.launch.py|hardware_pick.launch.py|livox_ros_driver2_node|semantic_segmenter_node.py|ros2 bag record' >&2 || true
  exit 1
fi

OUT_DIR="$(mkdir -p "$OUT_DIR" && cd "$OUT_DIR" && pwd)"
BAG_PATH="${OUT_DIR}/${BAG_NAME}"
LOG_DIR="${OUT_DIR}/${BAG_NAME}_logs"
if [[ -e "$BAG_PATH" || -e "$LOG_DIR" ]]; then
  echo "already exists: $BAG_PATH or $LOG_DIR" >&2
  echo "pick another -n NAME" >&2
  exit 1
fi
mkdir -p "$LOG_DIR"

D555_PID=""
LIVOX_PID=""
PICK_PID=""
REC_PID=""

stop_pid() {
  local pid="$1"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
  fi
}

cleanup() {
  local pid
  stop_pid "$REC_PID"
  if [[ -n "$REC_PID" ]]; then
    wait "$REC_PID" 2>/dev/null || true
  fi
  stop_pid "$PICK_PID"
  stop_pid "$D555_PID"
  stop_pid "$LIVOX_PID"
  for pid in "$PICK_PID" "$D555_PID" "$LIVOX_PID"; do
    if [[ -n "$pid" ]]; then
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap 'exit 130' INT TERM
trap cleanup EXIT

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "case=${BAG_NAME}"
echo "logs -> ${LOG_DIR}"
echo "arm stays still. e-stop person required. Ctrl+C stops the graph (no BlackOut)."

ros2 launch elfin_trajectory_executor d555_rgbd.launch.py \
  compress:=false \
  publish_raw:=true \
  color_profile:=640,360,15 \
  depth_profile:=640,360,15 \
  >"${LOG_DIR}/d555.log" 2>&1 &
D555_PID=$!

ros2 launch luggage_description mid360.launch.py \
  user_config_path:="${LIVOX_CFG}" \
  frame_id:=livox_frame \
  xfer_format:=0 \
  >"${LOG_DIR}/livox.log" 2>&1 &
LIVOX_PID=$!

"$PICK" \
  start_d555:=false \
  start_perception:=true \
  start_executor:=true \
  start_planning:=true \
  start_scene:=true \
  use_rviz:=false \
  use_compressed:=false \
  publish_overlay:=false \
  max_rate_hz:=0.0 \
  >"${LOG_DIR}/hardware_pick.log" 2>&1 &
PICK_PID=$!

wait_publisher() {
  local topic="$1"
  local tries="$2"
  local i
  for ((i = 1; i <= tries; i++)); do
    if ros2 topic info "$topic" 2>/dev/null | grep -q 'Publisher count: [1-9]'; then
      return 0
    fi
    if ! kill -0 "$D555_PID" 2>/dev/null || ! kill -0 "$PICK_PID" 2>/dev/null; then
      echo "graph exited while waiting for $topic. See ${LOG_DIR}" >&2
      return 1
    fi
    sleep 1
  done
  echo "timed out waiting for publisher on $topic. See ${LOG_DIR}" >&2
  return 1
}

wait_publisher /camera/d555/color/image_raw 40
wait_publisher /joint_states 30
wait_publisher /livox/lidar 20
wait_publisher /luggage/semantic/yolo_detections 45

echo "graph ready. recording raw images + YOLO + joints + Livox."
echo "bag -> ${BAG_PATH}"
if [[ "$DURATION" == "0" ]]; then
  echo "recording until Ctrl+C"
else
  echo "recording ${DURATION}s then stop"
fi

REGEX='(/joint_states$|/tf$|/tf_static$|/camera/d555/color/image_raw$|/camera/d555/aligned_depth_to_color/image_raw$|/camera/d555/color/camera_info$|/camera/d555/aligned_depth_to_color/camera_info$|/luggage/semantic/yolo_detections$|/luggage/semantic/mask$|/luggage/semantic/instance_mask$|/luggage/preprocessed/status$|/livox/lidar$|/clock_sync/master$)'
EXCLUDE='(/parameter_events|/diagnostics|/rosout|/luggage/semantic/overlay|compressed$|_hw$)'

cmd=(
  ros2 bag record
  -o "$BAG_PATH"
  --regex "$REGEX"
  --exclude-regex "$EXCLUDE"
  --disable-keyboard-controls
  --max-cache-size "$((64 * 1024 * 1024))"
)
if [[ -f "$QOS" ]]; then
  cmd+=(--qos-profile-overrides-path "$QOS")
fi

set +e
if [[ "$DURATION" == "0" ]]; then
  "${cmd[@]}" &
  REC_PID=$!
  wait "$REC_PID"
  rec_rc=$?
else
  timeout --signal=INT --kill-after=8 "${DURATION}" "${cmd[@]}" &
  REC_PID=$!
  wait "$REC_PID"
  rec_rc=$?
fi
set -e
REC_PID=""
# timeout SIGINT -> 124; a clean Ctrl+C is 130.
if [[ "$rec_rc" -ne 0 && "$rec_rc" -ne 124 && "$rec_rc" -ne 130 ]]; then
  echo "ros2 bag record exited ${rec_rc}. See ${LOG_DIR}" >&2
  exit "$rec_rc"
fi

if [[ ! -d "$BAG_PATH" ]]; then
  echo "bag directory was not created: $BAG_PATH" >&2
  exit 1
fi

NOTE_ESC="${NOTE//\"/\\\"}"
STAMP="$(date +%Y-%m-%dT%H:%M:%S%z)"
cat > "${BAG_PATH}/notes.yaml" <<EOF
name: ${BAG_NAME}
recorded_at: ${STAMP}
ros_domain_id: ${ROS_DOMAIN_ID}
duration_s: ${DURATION}
perception: raw image_raw, use_compressed false, publish_overlay false, max_rate_hz 0
arm: hold_no_driver
comment: "${NOTE_ESC}"
replay: |
  # Never: ros2 bag play on ROS_DOMAIN_ID=7
  # bag_mcap_source reads /camera/d555/*/image_raw first.
EOF

printf '%s\t%s\t%s\t%s\t%s\n' \
  "$STAMP" "$BAG_NAME" "$BAG_PATH" "$DURATION" "${NOTE}" \
  >> "${OUT_DIR}/index.tsv"

echo "wrote ${BAG_PATH}/notes.yaml"
echo "stopping graph."
