cmake_minimum_required(VERSION 3.8)
project(luggage_planning)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ST09-I: the batched route validation service (ValidateRouteV2) —
# MoveIt-native state/collision/distance checks against a frozen private
# scene copy. Pure C++ so certification needs no per-sample ROS calls.
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(srdfdom REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(luggage_msgs REQUIRED)
add_executable(validate_route_node src/validate_route_node.cpp)
ament_target_dependencies(validate_route_node
  rclcpp moveit_core moveit_msgs geometry_msgs urdf srdfdom luggage_msgs)
target_link_libraries(validate_route_node OpenSSL::Crypto)
target_compile_options(validate_route_node PRIVATE -Wall -Wextra)
install(TARGETS validate_route_node DESTINATION lib/${PROJECT_NAME})

add_executable(placement_tool_check_node src/placement_tool_check_node.cpp)
target_include_directories(placement_tool_check_node PRIVATE include)
ament_target_dependencies(placement_tool_check_node rclcpp moveit_core moveit_msgs geometry_msgs urdf srdfdom luggage_msgs)
target_link_libraries(placement_tool_check_node OpenSSL::Crypto)
target_compile_options(placement_tool_check_node PRIVATE -Wall -Wextra)
install(TARGETS placement_tool_check_node DESTINATION lib/${PROJECT_NAME})

find_package(moveit_ros_planning REQUIRED)
add_executable(grasp_local_check_node src/grasp_local_check_node.cpp)
ament_target_dependencies(grasp_local_check_node rclcpp moveit_core moveit_ros_planning moveit_msgs geometry_msgs luggage_msgs)
target_link_libraries(grasp_local_check_node OpenSSL::Crypto)
target_compile_options(grasp_local_check_node PRIVATE -Wall -Wextra)
install(TARGETS grasp_local_check_node DESTINATION lib/${PROJECT_NAME})

add_executable(snapshot_joint_path_node src/snapshot_joint_path_node.cpp)
ament_target_dependencies(snapshot_joint_path_node rclcpp moveit_core moveit_ros_planning moveit_msgs geometry_msgs luggage_msgs)
target_link_libraries(snapshot_joint_path_node OpenSSL::Crypto)
target_compile_options(snapshot_joint_path_node PRIVATE -Wall -Wextra)
install(TARGETS snapshot_joint_path_node DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/waypoint_generator_node.py
  scripts/diagnostic_capture_node.py
  scripts/motion_planner_node.py
  scripts/trajectory_selector_node.py
  scripts/vacuum_controller_node.py
  scripts/scene_manager_node.py
  scripts/hardware_pick_driver.py
  scripts/place_prior_node.py
  scripts/joint_operation_prepare_node.py
  scripts/static_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(luggage_planning_pytest test
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test
  )
endif()

ament_package()
