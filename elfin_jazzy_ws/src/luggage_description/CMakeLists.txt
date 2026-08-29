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
