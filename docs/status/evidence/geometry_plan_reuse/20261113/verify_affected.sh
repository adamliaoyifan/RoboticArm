#!/usr/bin/env bash
# Deterministic affected-source regression; no simulator and no launch tests.
set -eo pipefail
source /opt/ros/humble/setup.bash
source install-production/setup.bash
set -u
export PYTHONPATH="$PWD/src/luggage_packing:$PWD/src/luggage_perception:$PWD/src/luggage_planning:$PWD/src/luggage_gazebo:$PYTHONPATH"
# The ROS launch-testing plugin imports unrelated tests while collecting explicit
# unit-test paths. These behavioural unit tests require no auto-loaded plugins.
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
python3 -m pytest -q \
  src/luggage_packing/test/test_selected_pose_admission.py \
  src/luggage_packing/test/test_placement_query.py \
  src/luggage_packing/test/test_placement_query_consumers.py \
  src/luggage_perception/test/test_geometry_window.py \
  src/luggage_perception/test/test_lidar_structure.py \
  src/luggage_perception/test/test_observer_lidar.py \
  src/luggage_planning/test/test_successful_path_cache.py \
  src/luggage_planning/test/test_joint_operation_planner.py \
  src/luggage_planning/test/test_joint_operation_runtime.py \
  src/luggage_planning/test/test_placement_prior_requests.py \
  src/luggage_gazebo/test/test_operation_launch.py
