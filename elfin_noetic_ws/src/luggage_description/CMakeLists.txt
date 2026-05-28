cmake_minimum_required(VERSION 3.0.2)
project(luggage_description)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/description_params_node.py
  scripts/container_config_utils.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config urdf launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
