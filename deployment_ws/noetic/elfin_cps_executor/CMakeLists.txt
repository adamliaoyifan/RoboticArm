cmake_minimum_required(VERSION 3.0.2)
project(elfin_cps_executor)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/cps_trajectory_executor.py
  scripts/trajectory_command_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES scripts/huayan_backend.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

set(HUAYAN_CPP_SDK "$ENV{HUAYAN_CPP_SDK}")
if(NOT HUAYAN_CPP_SDK)
  set(HUAYAN_CPP_SDK "/opt/huayan/cpp_sdk")
endif()

set(HUAYAN_CPP_INCLUDE_DIR "${HUAYAN_CPP_SDK}/include/HRSDK")
set(HUAYAN_CPP_LIB_DIR "${HUAYAN_CPP_SDK}/lib")
set(HUAYAN_CPP_HRCPS_DIR "${HUAYAN_CPP_SDK}/HRCPS")

if(EXISTS "${HUAYAN_CPP_INCLUDE_DIR}/HR_Pro.h" AND EXISTS "${HUAYAN_CPP_LIB_DIR}/libHR_Pro.so")
  add_executable(cpp_cps_trajectory_executor
    src/cpp_cps_trajectory_executor.cpp
  )

  target_include_directories(cpp_cps_trajectory_executor PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${HUAYAN_CPP_INCLUDE_DIR}
  )

  target_link_libraries(cpp_cps_trajectory_executor
    ${catkin_LIBRARIES}
    ${HUAYAN_CPP_LIB_DIR}/libHR_Pro.so
  )

  set_target_properties(cpp_cps_trajectory_executor PROPERTIES
    BUILD_RPATH "${HUAYAN_CPP_LIB_DIR};${HUAYAN_CPP_HRCPS_DIR}"
    INSTALL_RPATH "${HUAYAN_CPP_LIB_DIR};${HUAYAN_CPP_HRCPS_DIR}"
  )

  install(TARGETS cpp_cps_trajectory_executor
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  message(WARNING "Huayan C++ SDK not found at ${HUAYAN_CPP_SDK}; skipping cpp_cps_trajectory_executor")
endif()

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
