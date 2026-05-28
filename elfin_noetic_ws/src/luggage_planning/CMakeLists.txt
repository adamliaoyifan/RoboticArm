cmake_minimum_required(VERSION 3.0.2)
project(luggage_planning)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/scene_manager_node.py
  scripts/waypoint_generator_node.py
  scripts/motion_planner_node.py
  scripts/vacuum_simulator_node.py
  scripts/container_aim_utils.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/waypoint_generator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
