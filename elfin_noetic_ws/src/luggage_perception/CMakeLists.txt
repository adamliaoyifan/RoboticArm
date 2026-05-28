cmake_minimum_required(VERSION 3.0.2)
project(luggage_perception)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/luggage_detector_node.py
  scripts/container_inspector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
