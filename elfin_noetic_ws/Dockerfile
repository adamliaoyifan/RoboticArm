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
    libgtk-3-dev \
    python3-wxgtk4.0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws
COPY src ./src

RUN source /opt/ros/noetic/setup.bash \
    && catkin_make -j"$(nproc)" \
    && echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
