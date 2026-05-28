# =============================================================================
# Dockerfile — elfin_s_robot (ROS Noetic) on Ubuntu 20.04
# =============================================================================
# Runs the Huayan Elfin-S ROS Noetic stack inside Docker so an Ubuntu 22.04
# host can use Gazebo/MoveIt without downgrading the OS.
#
# Build (from the catkin workspace root):
#   cd ~/elfin_noetic_ws
#   docker build -f /path/to/RobotArm/docker/noetic/Dockerfile -t elfin-noetic .
#
# Or use:  ./docker/noetic/run.sh build
# =============================================================================

FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

ENV XDG_RUNTIME_DIR=/tmp/runtime-root
RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root

# Upstream dependencies from elfin_s_robot README (noetic branch)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-soem \
    ros-noetic-gazebo-ros-control \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-moveit-* \
    ros-noetic-trac-ik \
    build-essential \
    libgtk-3-dev \
    python3-pip \
    python3-wxgtk4.0 \
    git \
    && rm -rf /var/lib/apt/lists/*

# wxPython for elfin_basic_api control panel (apt preferred; pip as fallback)
RUN pip3 install --no-cache-dir wxpython || true

WORKDIR /catkin_ws
COPY src ./src

RUN source /opt/ros/noetic/setup.bash \
    && catkin_make \
    && echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
