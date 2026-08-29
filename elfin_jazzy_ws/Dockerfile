FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV XDG_RUNTIME_DIR=/tmp/runtime-root
RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root

# Upstream dependencies from README (use moveit metapackage, not moveit-* wildcard)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-soem \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-moveit \
    ros-noetic-trac-ik \
    ros-noetic-octomap-msgs \
    ros-noetic-octomap-rviz-plugins \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-message-filters \
    python3-octomap \
    python3-yaml \
    python3-numpy \
    libgtk-3-dev \
    python3-wxgtk4.0 \
    && rm -rf /var/lib/apt/lists/*

# Semantic perception pipeline (luggage_perception). Heavy ML deps —
# installed lazily and optional; the nodes fall back to a stub backend when
# these are missing. CPU-only torch to keep the image smaller; swap the
# index URL for CUDA 12.1 if a GPU is available.
#   --index-url https://download.pytorch.org/whl/cu121
RUN pip3 install --no-cache-dir \
    torch torchvision \
    --index-url https://download.pytorch.org/whl/cpu \
    && pip3 install --no-cache-dir \
    ultralytics>=8.1.0 \
    opencv-python-headless>=4.8.0

WORKDIR /catkin_ws
COPY src ./src

RUN source /opt/ros/noetic/setup.bash \
    && catkin_make -j"$(nproc)" \
    && echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
