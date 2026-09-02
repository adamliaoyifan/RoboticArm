cmake_minimum_required(VERSION 3.8)
project(luggage_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "backups" EXCLUDE
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.launch" EXCLUDE
)

# EEF / suction STLs live in luggage_gazebo/models. Install a copy so RViz can
# resolve them from luggage_description without building the Gazebo stack.
set(_gz_models "${CMAKE_CURRENT_SOURCE_DIR}/../luggage_gazebo/models")
if(EXISTS "${_gz_models}/arm_realsense/arm_realsense_v1.3.stl")
  install(DIRECTORY "${_gz_models}/arm_realsense"
    DESTINATION share/${PROJECT_NAME}/meshes)
endif()
if(EXISTS "${_gz_models}/suction_panel/collision")
  install(DIRECTORY "${_gz_models}/suction_panel/collision/"
    DESTINATION share/${PROJECT_NAME}/meshes/suction_panel)
endif()

install(PROGRAMS
  scripts/container_tf_publisher
  scripts/xacro_robot_with_scene_base
  scripts/zero_joint_state_publisher
  scripts/generate_sized_suitcases.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(luggage_description_pytest test
    TIMEOUT 180
  )
endif()

ament_package()
