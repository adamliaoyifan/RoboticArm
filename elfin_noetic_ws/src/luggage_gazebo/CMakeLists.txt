cmake_minimum_required(VERSION 3.0.2)
project(luggage_gazebo)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/clear_gazebo_models.py
  scripts/pickup_box_spawner_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
