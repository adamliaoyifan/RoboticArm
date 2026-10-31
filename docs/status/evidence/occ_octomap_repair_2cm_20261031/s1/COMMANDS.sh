#!/bin/sh
set -eu
cmake -S /tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer -B /tmp/observer-s1 -DOBSERVER_WITH_ROS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/observer-s1 --target observer_repair_s1_behavior -j"$(nproc)"
/tmp/observer-s1/observer_repair_s1_behavior
. /opt/ros/humble/setup.bash
cmake -S /tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer -B /tmp/observer-s1-ros -DOBSERVER_WITH_ROS=ON -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/observer-s1-ros --target observer_node observer_repair_s1_behavior -j"$(nproc)"
