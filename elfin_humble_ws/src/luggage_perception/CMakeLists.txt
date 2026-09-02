cmake_minimum_required(VERSION 3.8)
project(luggage_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# YOLO-World checkpoint for the semantic backends. Installed so the node can
# resolve it from the package share instead of downloading on first run.
install(FILES yolov8s-world.pt
  DESTINATION share/${PROJECT_NAME}/models
)

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
  scripts/luggage_detector_node.py
  scripts/sensor_preprocessor_node.py
  scripts/semantic_segmenter_node.py
  scripts/semantic_point_filter_node.py
  scripts/cargo_volume_mapper_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
