cmake_minimum_required(VERSION 3.0.2)
project(luggage_bringup)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/orchestrator_node.py
  scripts/reset_observe_client.py
  scripts/pose_tune_gui.py
  scripts/export_observe_joints.py
  scripts/camera_mount_tune_gui.py
  scripts/camera_mount_tune_init.py
  scripts/cam_mount_joint_state_merger.py
  scripts/verify_camera_mount_config.py
  scripts/mount_config_utils.py
  scripts/sync_camera_mount_config.py
  scripts/container_tf_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
