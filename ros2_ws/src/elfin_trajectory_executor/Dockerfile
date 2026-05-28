# =============================================================================
# Dockerfile — Elfin S20 Trajectory Executor
# =============================================================================
# Runs ROS2 Humble on Ubuntu 20.04 host via Docker so the deployment machine
# can participate in the same DDS domain as the planning machine (Humble /
# Ubuntu 22.04) while also reaching the HuayanRobot controller over TCP/IP.
#
# Build (run from the workspace root — the directory containing ros2_ws/):
#   # Copy the CPS SDK .so to the workspace root first:
#   cp /path/to/CPS_python3_Linux.so /path/to/RobotArm/
#
#   cd /path/to/RobotArm
#   docker build \
#     -f ros2_ws/src/elfin_trajectory_executor/Dockerfile \
#     -t elfin_executor .
#
# Run (simulation — no robot required):
#   docker run --rm --network host \
#     -e ROS_DOMAIN_ID=0 \
#     elfin_executor \
#     ros2 launch elfin_trajectory_executor sim.launch.py
#
# Run (real hardware):
#   docker run --rm --network host \
#     -e ROS_DOMAIN_ID=0 \
#     elfin_executor \
#     ros2 launch elfin_trajectory_executor real.launch.py \
#       robot_ip:=192.168.0.10
#
# Notes:
#   --network host   Required for ROS2 DDS multicast to reach other machines.
#   ROS_DOMAIN_ID    Must match the planning node's domain ID (default 0).
#   Build context    Must be the workspace root so COPY paths resolve correctly.
# =============================================================================

FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

# ---------------------------------------------------------------------------
# System dependencies
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-numpy \
        ros-humble-control-msgs \
        ros-humble-trajectory-msgs \
        ros-humble-sensor-msgs \
        ros-humble-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# HuayanRobot CPSClient SDK
# The .so file must be present in the workspace root (build context root).
# It is installed as CPS.so so Python can import it as:
#     from CPS import CPSClient
# ---------------------------------------------------------------------------
COPY CPS_python3_Linux.so /opt/huayan/CPS.so

# Make the SDK importable from any Python process in the container.
ENV PYTHONPATH=/opt/huayan:${PYTHONPATH}

# ---------------------------------------------------------------------------
# ROS2 workspace
# ---------------------------------------------------------------------------
WORKDIR /ros2_ws

# Copy the source tree.
COPY ros2_ws/src ./src

# Build with colcon (source Humble first).
RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install \
        --packages-select elfin_trajectory_executor \
    && echo "Build complete."

# Source the workspace on every new shell.
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# ---------------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------------
# The base image (osrf/ros:humble-desktop-full) already ships
# /ros2_entrypoint.sh which sources /opt/ros/humble/setup.bash.
# We patch it in-place to also source the workspace overlay so every
# exec'd command sees both the Humble and the package install spaces.
RUN sed -i \
    's|exec "\$@"|source /ros2_ws/install/setup.bash \&\& exec "$@"|' \
    /ros2_entrypoint.sh \
    && echo "Entrypoint patched to source workspace overlay."

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["ros2", "launch", "elfin_trajectory_executor", "real.launch.py"]
