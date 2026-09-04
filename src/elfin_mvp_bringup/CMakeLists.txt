cmake_minimum_required(VERSION 3.8)
project(elfin_mvp_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(move_to_joint_goal src/move_to_joint_goal.cpp)
ament_target_dependencies(move_to_joint_goal rclcpp moveit_ros_planning_interface)

install(TARGETS move_to_joint_goal
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/send_joint_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
