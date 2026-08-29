cmake_minimum_required(VERSION 3.0.2)
project(luggage_bringup)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/orchestrator_node.py
  scripts/interior_explore_loop.py
  scripts/interior_exploration_bag_harness.py
  scripts/active_loading_bag_harness.py
  scripts/multi_box_gazebo_matrix.py
  scripts/phase_evidence_check.py
  scripts/size_uncertainty.py
  scripts/step_gate.py
  scripts/run_events_recorder.py
  scripts/reset_observe_client.py
  scripts/pose_tune_gui.py
  scripts/manual_loading_gui.py
  scripts/pose_tune_draft_utils.py
  scripts/pose_tune_scene_viz_node.py
  scripts/pose_tune_workspace_analyzer.py
  scripts/export_observe_joints.py
  scripts/camera_mount_tune_gui.py
  scripts/camera_mount_tune_init.py
  scripts/cam_mount_joint_state_merger.py
  scripts/verify_camera_mount_config.py
  scripts/mount_config_utils.py
  scripts/sync_camera_mount_config.py
  scripts/container_tf_publisher.py
  scripts/probe_execution_utils.py
  scripts/explore_execution_utils.py
  scripts/ompl_lvsf_benchmark.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
