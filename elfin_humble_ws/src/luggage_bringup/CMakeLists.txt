cmake_minimum_required(VERSION 3.8)
project(luggage_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/orchestrator_node.py
  scripts/operator_control.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES launch/production_orchestrator.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(luggage_bringup_pytest test
    TIMEOUT 180
  )
endif()

ament_package()
