#!/usr/bin/env bash
# =============================================================================
# run.sh — elfin_s_robot (ROS Noetic) Docker helper
# =============================================================================
# Usage:
#   ./run.sh build              Build the elfin-noetic image
#   ./run.sh sim                Start interactive shell (simulation profile)
#   ./run.sh hw                 Start interactive shell (hardware profile)
#   ./run.sh exec               Open a second shell in the running container
#   ./run.sh gazebo             Launch Gazebo (S20) in running container
#   ./run.sh moveit             Launch MoveIt + RViz (S20)
#   ./run.sh api                Launch elfin_basic_api control panel
#
# Environment variables:
#   ELFIN_MODEL     Robot model prefix: s05|s10|s20|s30  (default: s20)
#   ELFIN_WS        Catkin workspace on host (default: ~/elfin_noetic_ws)
#   CONTAINER_NAME  Docker container name (default: elfin_noetic)
#   USE_GPU         Set to 1 to pass --gpus all (default: 0)
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOTARM_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ELFIN_WS="${ELFIN_WS:-${ROBOTARM_DIR}/elfin_noetic_ws}"
DOCKERFILE="${SCRIPT_DIR}/Dockerfile"
IMAGE_NAME="${IMAGE_NAME:-elfin-noetic}"
CONTAINER_NAME="${CONTAINER_NAME:-elfin_noetic}"
ELFIN_MODEL="${ELFIN_MODEL:-s20}"
USE_GPU="${USE_GPU:-0}"
LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}"

MODEL_PREFIX="elfin_${ELFIN_MODEL}"

common_run_args() {
    local detached="${1:-0}"
    local -a args=()
    if [[ "${detached}" == "1" ]]; then
        args+=(-d)
    else
        args+=(-it)
    fi
    args+=(
        --name "${CONTAINER_NAME}"
        --net=host
        --env "DISPLAY=${DISPLAY:-:0}"
        --env QT_X11_NO_MITSHM=1
        --env XDG_RUNTIME_DIR=/tmp/runtime-root
        --env "LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE}"
        --volume /tmp/.X11-unix:/tmp/.X11-unix:rw
        --volume "${HOME}/.gazebo:/root/.gazebo"
        --volume "${ELFIN_WS}/src:/catkin_ws/src"
    )
    if [[ "${USE_GPU}" == "1" ]]; then
        args+=(--gpus all)
        args+=(--env NVIDIA_VISIBLE_DEVICES=all)
        args+=(--env NVIDIA_DRIVER_CAPABILITIES=all)
    fi
    echo "${args[@]}"
}

ensure_x11() {
    if command -v xhost >/dev/null 2>&1; then
        xhost +local:docker >/dev/null 2>&1 || xhost +local: >/dev/null 2>&1 || true
    fi
}

ensure_workspace() {
    if [[ ! -d "${ELFIN_WS}/src/elfin_s_robot" ]]; then
        echo "Missing workspace. Clone first:"
        echo "  mkdir -p ${ELFIN_WS}/src"
        echo "  git clone -b noetic https://github.com/huayan-robotics/elfin_s_robot.git ${ELFIN_WS}/src/elfin_s_robot"
        exit 1
    fi
}

cmd_build() {
    ensure_workspace
    echo "Building ${IMAGE_NAME} from ${ELFIN_WS} ..."
    docker build -f "${DOCKERFILE}" -t "${IMAGE_NAME}" "${ELFIN_WS}"
    echo "Done. Run: ${SCRIPT_DIR}/run.sh sim"
}

cmd_sim() {
    ensure_x11
    ensure_workspace
    if docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
        echo "Container ${CONTAINER_NAME} already exists. Use: ${SCRIPT_DIR}/run.sh exec"
        exit 1
    fi
    # shellcheck disable=SC2046
    docker run --rm $(common_run_args) "${IMAGE_NAME}" bash
}

cmd_hw() {
    ensure_x11
    ensure_workspace
    CONTAINER_NAME="${CONTAINER_NAME}_hw"
    if docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
        echo "Container ${CONTAINER_NAME} already exists. Use:"
        echo "  CONTAINER_NAME=${CONTAINER_NAME} ${SCRIPT_DIR}/run.sh exec"
        exit 1
    fi
    # shellcheck disable=SC2046
    docker run --rm $(common_run_args) \
        --privileged \
        --cap-add=SYS_NICE \
        --cap-add=IPC_LOCK \
        "${IMAGE_NAME}" bash
}

cmd_exec() {
    docker exec -it "${CONTAINER_NAME}" bash -lc 'source /catkin_ws/devel/setup.bash && exec bash'
}

docker_exec_shell() {
    local -a flags=(-i)
    if [[ -t 1 ]]; then
        flags+=(-t)
    fi
    docker exec "${flags[@]}" "${CONTAINER_NAME}" bash -lc "$1"
}

cmd_roslaunch() {
    local pkg="$1"
    local launch="$2"
    shift 2
    docker_exec_shell "source /catkin_ws/devel/setup.bash && roslaunch ${pkg} ${launch} $*"
}

has_robot_description() {
    docker exec "${CONTAINER_NAME}" bash -lc \
        'source /catkin_ws/devel/setup.bash && rosparam get /robot_description >/dev/null 2>&1'
}

cmd_moveit() {
    local -a extra_args=("$@")
    if has_robot_description; then
        echo "Using /robot_description from Gazebo or bringup."
    else
        echo "WARNING: Gazebo is not running — loading URDF via MoveIt."
        echo "         For trajectory execution in sim, start Gazebo first:"
        echo "           ${SCRIPT_DIR}/run.sh gazebo"
        extra_args+=(load_robot_description:=true)
    fi
    cmd_roslaunch "${MODEL_PREFIX}_moveit_config" moveit_planning_execution.launch "${extra_args[@]}"
}

cmd_sim_all() {
    if [[ "${ELFIN_MODEL}" != "s20" ]]; then
        echo "sim-all is implemented for S20 only. Use gazebo + moveit for ${ELFIN_MODEL}."
        exit 1
    fi
    cmd_roslaunch elfin_gazebo elfin_s20_gazebo_moveit.launch "${@}"
}

case "${1:-}" in
    build)
        cmd_build
        ;;
    sim)
        cmd_sim
        ;;
    start)
        ensure_x11
        ensure_workspace
        if docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
            echo "Container ${CONTAINER_NAME} is already running."
            exit 0
        fi
        # shellcheck disable=SC2046
        docker run $(common_run_args 1) "${IMAGE_NAME}" sleep infinity
        echo "Started ${CONTAINER_NAME}. Use: ${SCRIPT_DIR}/run.sh exec|gazebo|moveit|api"
        ;;
    stop)
        docker rm -f "${CONTAINER_NAME}" 2>/dev/null && echo "Stopped ${CONTAINER_NAME}." || echo "Container not running."
        ;;
    hw)
        cmd_hw
        ;;
    exec)
        cmd_exec
        ;;
    gazebo)
        cmd_roslaunch elfin_gazebo "${MODEL_PREFIX}_empty_world.launch"
        ;;
    moveit)
        cmd_moveit "${@:2}"
        ;;
    sim-all)
        cmd_sim_all "${@:2}"
        ;;
    api)
        cmd_roslaunch elfin_basic_api elfin_basic_api.launch
        ;;
    *)
        cat <<EOF
Usage: $(basename "$0") <command>

Commands:
  build     Build the elfin-noetic Docker image
  sim       Start container (simulation profile, interactive shell)
  start     Start container in background (for multi-terminal workflow)
  stop      Stop and remove the running container
  hw        Start container (hardware: privileged + realtime caps)
  exec      Open another shell in the running container
  gazebo    roslaunch elfin_gazebo ${MODEL_PREFIX}_empty_world.launch
  moveit    MoveIt + RViz (auto-loads URDF if Gazebo not running)
  sim-all   Gazebo + MoveIt + RViz in one launch (S20)
  api       roslaunch elfin_basic_api elfin_basic_api.launch

Environment:
  ELFIN_MODEL=${ELFIN_MODEL}   ELFIN_WS=${ELFIN_WS}
  CONTAINER_NAME=${CONTAINER_NAME}   USE_GPU=${USE_GPU}
  LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE}  (set 1 if RViz/Gazebo black window)

Simulation workflow (recommended — Gazebo must be first for execution):
  1. $(basename "$0") start
  2. $(basename "$0") gazebo          # wait until Gazebo is up
  3. $(basename "$0") moveit
  4. $(basename "$0") api

Or single launch (S20):
  $(basename "$0") sim-all
EOF
        exit 1
        ;;
esac
