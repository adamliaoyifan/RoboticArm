#!/bin/sh
# S1 gen3 replay. No Gazebo and no O5-O9 scoring.
set -eu
BASE=32a3267e633cc79d0e912531fabd4684212b53e0
ROOT=/tmp/elfin-occ-octomap-repair-2cm-g1
GEOM=/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures/geometry_cuboid.yaml
CFG=/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures/config_cuboid_r0p05.yaml

cmake -S "$ROOT/src/uld_occ_observer" -B /tmp/observer-s1-g3-core -DOBSERVER_WITH_ROS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/observer-s1-g3-core --target observer_repair_s1_behavior -j"$(nproc)"
/tmp/observer-s1-g3-core/observer_repair_s1_behavior
c++ -std=c++17 -O2 -pthread \
  -I"$ROOT/src/uld_occ_observer/include" -I/opt/ros/humble/include \
  /home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_octomap_repair_2cm_20261031/s1_review/probe.cpp \
  /tmp/observer-s1-g3-core/libuld_occ_observer_io.a /tmp/observer-s1-g3-core/libuld_occ_observer_core.a \
  /opt/ros/humble/lib/x86_64-linux-gnu/liboctomap.so /opt/ros/humble/lib/x86_64-linux-gnu/liboctomath.so \
  -lcrypto -lyaml-cpp -Wl,-rpath,/opt/ros/humble/lib/x86_64-linux-gnu \
  -o /tmp/observer-s1-g3-core/resource_probe
/tmp/observer-s1-g3-core/resource_probe
python3 -m pytest -q --tb=line "$ROOT/src/uld_occ_observer/test/test_repair_s0.py"

. /opt/ros/humble/setup.bash
cd "$ROOT"
colcon build --packages-select uld_occ_observer \
  --build-base /tmp/s1g3-build --install-base /tmp/s1g3-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 4
. /tmp/s1g3-install/setup.bash
python3 "$ROOT/src/uld_occ_observer/test/s1_g3_node_behavior.py" \
  --node /tmp/s1g3-install/uld_occ_observer/lib/uld_occ_observer/observer_node \
  --geometry "$GEOM" --config "$CFG" \
  --result "$ROOT/docs/status/evidence/occ_octomap_repair_2cm_20261031/s1_g3/live_checks.json" \
  --log-dir /tmp/s1g3-live
echo "base $BASE"
