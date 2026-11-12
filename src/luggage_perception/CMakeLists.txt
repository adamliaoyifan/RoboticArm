cmake_minimum_required(VERSION 3.8)
project(luggage_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The O ray adapters share one acquisition-time, actual-URDF self mask.
find_package(rclcpp REQUIRED)
find_package(luggage_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(GLUT REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(srdfdom REQUIRED)
find_package(OpenSSL REQUIRED)
add_executable(observer_self_mask_node src/observer_self_mask_node.cpp)
ament_target_dependencies(observer_self_mask_node rclcpp moveit_core
  moveit_ros_perception sensor_msgs std_msgs tf2_ros urdf srdfdom luggage_msgs)
target_link_libraries(observer_self_mask_node OpenSSL::Crypto)
target_compile_options(observer_self_mask_node PRIVATE -Wall -Wextra)
install(TARGETS observer_self_mask_node DESTINATION lib/${PROJECT_NAME})

# NOTE: ament_python_install_package COPIES the package under
# install/*/dist-packages (the scripts below stay symlinks only under
# --symlink-install). Library edits are invisible to installed CLIs until
# `colcon build --packages-select luggage_perception` reruns; the eval
# CLIs guard this with --check-install (eval/run_provenance.py).
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# YOLO-World checkpoint. Optional until scripts/download_yolo_world.sh runs.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/yolov8s-world.pt")
  install(FILES yolov8s-world.pt
    DESTINATION share/${PROJECT_NAME}/models)
else()
  message(WARNING "yolov8s-world.pt missing; run scripts/download_yolo_world.sh before semantic bringup")
endif()

# Offline CLIP vendor (package + deps + ViT-B/32 weights) so YOLO-World
# backends never trigger ultralytics' network auto-install. Search order in
# semantic_segmenter._setup_clip_vendor: $LUGGAGE_CLIP_VENDOR_DIR, source
# tree, this share location.
install(DIRECTORY vendor
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(luggage_perception_pytest test)
endif()

install(PROGRAMS
  scripts/observer_depth_node.py
  scripts/observer_lidar_node.py
  scripts/cargo_observation_node.py
  scripts/luggage_detector_node.py
  scripts/sensor_preprocessor_node.py
  scripts/d555_transport_adapter_node.py
  scripts/semantic_segmenter_node.py
  scripts/semantic_point_filter_node.py
  scripts/cargo_volume_mapper_node.py
  scripts/dump_camera_frames.py
  scripts/handeye_capture.py
  scripts/handeye_solve.py
  scripts/pendant_bag_replay_eval.py
  scripts/backfill_pickup_labels.py
  scripts/export_lidar_camera_calib.py
  scripts/table_patch_icp.py
  scripts/viz_eef_tf_fit.py
  scripts/perception_site.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
