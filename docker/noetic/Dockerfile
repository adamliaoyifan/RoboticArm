# =============================================================================
# Dockerfile — elfin_s_robot (ROS Noetic) on Ubuntu 20.04
# =============================================================================
# Runs the Huayan Elfin-S ROS Noetic stack inside Docker so an Ubuntu 22.04
# host can use Gazebo/MoveIt without downgrading the OS.
#
# Build (from the RobotArm repo root):
#   docker build -f docker/noetic/Dockerfile -t elfin-noetic .
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
    ros-noetic-octomap-msgs \
    ros-noetic-octomap-rviz-plugins \
    build-essential \
    libgtk-3-dev \
    python3-pip \
    python3-wxgtk4.0 \
    git \
    && rm -rf /var/lib/apt/lists/*

# wxPython for elfin_basic_api control panel (apt preferred; pip as fallback)
RUN pip3 install --no-cache-dir wxpython || true

# ---------------------------------------------------------------------------
# Huayan TCP SDKs
# ---------------------------------------------------------------------------
# Python SDK: importable as `from CPS import CPSClient`.
COPY third_party/huayan_python_sdk/CPS.py /opt/huayan/CPS.py
ENV PYTHONPATH=/opt/huayan

# C++ SDK: headers and shared library used by elfin_cps_executor.
COPY SDK_sample/CppLinux_SDK/sdk /opt/huayan/cpp_sdk
COPY SDK_sample/CppLinux_SDK /opt/huayan/cpp_sample
ENV HUAYAN_CPP_SDK=/opt/huayan/cpp_sdk
ENV LD_LIBRARY_PATH=/opt/huayan/cpp_sdk/lib:/opt/huayan/cpp_sdk/HRCPS

WORKDIR /catkin_ws
COPY elfin_noetic_ws/src ./src
# Real-robot TCP executor lives with the other runtime nodes in deployment_ws.
COPY deployment_ws/noetic/elfin_cps_executor ./src/elfin_cps_executor

RUN source /opt/ros/noetic/setup.bash \
    && catkin_make \
    && echo "export PYTHONPATH=/opt/huayan:\${PYTHONPATH}" >> /root/.bashrc \
    && echo "export HUAYAN_CPP_SDK=/opt/huayan/cpp_sdk" >> /root/.bashrc \
    && echo "export LD_LIBRARY_PATH=/opt/huayan/cpp_sdk/lib:/opt/huayan/cpp_sdk/HRCPS:\${LD_LIBRARY_PATH}" >> /root/.bashrc \
    && echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
