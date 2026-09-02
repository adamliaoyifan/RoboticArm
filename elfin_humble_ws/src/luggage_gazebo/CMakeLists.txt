cmake_minimum_required(VERSION 3.8)
project(luggage_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch worlds models config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pickup_box_spawner_node.py
  scripts/depth_image_republisher.py
  scripts/observe_pose_hold.py
  scripts/scene_viz_node.py
  scripts/pick_retreat_eval_driver.py
  scripts/place_smoke_driver.py
  scripts/pack_eval_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(eval_metrics_pytest test/test_eval_metrics.py)
  ament_add_pytest_test(place_metrics_pytest test/test_place_metrics.py)
  ament_add_pytest_test(place_gt_dump_pytest test/test_place_gt_dump.py)
endif()

# Export the GZ_SIM_RESOURCE_PATH entry so model:// URIs in the world and
# package:// URIs inside the robot URDF resolve inside Gazebo Fortress.
ament_environment_hooks("env/${PROJECT_NAME}.dsv.in")

ament_package()
