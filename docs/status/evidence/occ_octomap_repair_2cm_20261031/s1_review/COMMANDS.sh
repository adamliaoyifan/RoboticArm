#!/usr/bin/env bash
set -euo pipefail
cd /home/adamliao/work/elfin_humble_ws
cmake -S /tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer -B /tmp/observer-s1-review -DOBSERVER_WITH_ROS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/observer-s1-review --target observer_repair_s1_behavior -j2
timeout 30 /tmp/observer-s1-review/observer_repair_s1_behavior
c++ -std=c++17 -O2 -pthread -I/tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer/include -I/opt/ros/humble/include docs/status/evidence/occ_octomap_repair_2cm_20261031/s1_review/probe.cpp /tmp/observer-s1-review/libuld_occ_observer_io.a /tmp/observer-s1-review/libuld_occ_observer_core.a /opt/ros/humble/lib/x86_64-linux-gnu/liboctomap.so /opt/ros/humble/lib/x86_64-linux-gnu/liboctomath.so -lcrypto -lyaml-cpp -Wl,-rpath,/opt/ros/humble/lib/x86_64-linux-gnu -o /tmp/observer-s1-review/resource_probe
# Expected exit 1 on 32a3267e: three attribution assertions fail.
/tmp/observer-s1-review/resource_probe
