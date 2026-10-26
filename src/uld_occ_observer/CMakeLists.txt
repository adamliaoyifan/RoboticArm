cmake_minimum_required(VERSION 3.16)
project(uld_occ_observer VERSION 0.1.0 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Both headers and libraries must come from this exact ROS prefix.
find_package(octomap 1.9.8 EXACT REQUIRED CONFIG
  PATHS /opt/ros/humble/share/octomap NO_DEFAULT_PATH)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
add_library(uld_occ_observer_core
  src/preprocessing.cpp src/geometry.cpp src/snapshot.cpp src/observer.cpp src/snapshot_repository.cpp src/observation_worker.cpp)
target_include_directories(uld_occ_observer_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(uld_occ_observer_core PUBLIC ${OCTOMAP_LIBRARIES} OpenSSL::Crypto Threads::Threads)
target_compile_options(uld_occ_observer_core PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(uld_occ_observer_core PROPERTIES
  BUILD_RPATH "${OCTOMAP_LIBRARY_DIRS}" INSTALL_RPATH "${OCTOMAP_LIBRARY_DIRS}")
add_library(uld_occ_observer_io src/configuration.cpp src/checkpoint_io.cpp src/serialization.cpp)
target_link_libraries(uld_occ_observer_io PUBLIC uld_occ_observer_core yaml-cpp)
add_executable(observer_cli src/observer_cli.cpp)
target_link_libraries(observer_cli PRIVATE uld_occ_observer_io)
set_target_properties(observer_cli PROPERTIES BUILD_RPATH "${OCTOMAP_LIBRARY_DIRS}" INSTALL_RPATH "${OCTOMAP_LIBRARY_DIRS}")
install(TARGETS observer_cli DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/export_geometry.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS uld_occ_observer_io DESTINATION lib)
install(TARGETS uld_occ_observer_core EXPORT observerTargets DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
# Plain CMake builds the ROS-free core. Colcon enables the ROS wrapper later.
option(OBSERVER_WITH_ROS "Build standalone ROS interfaces" ON)
if(OBSERVER_WITH_ROS)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/SnapshotIdentity.msg" "msg/UpdateReceipt.msg"
    "srv/IntegrateObservation.srv" "srv/GetSnapshot.srv" "srv/ReadCells.srv"
    "srv/ReadSurface.srv" "srv/ReleaseSnapshot.srv" "srv/ResetObservation.srv"
    "srv/CheckpointObservation.srv"
    DEPENDENCIES sensor_msgs geometry_msgs)
  add_executable(observer_node src/observer_node.cpp src/ros_adapters.cpp)
  target_link_libraries(observer_node uld_occ_observer_io)
  ament_target_dependencies(observer_node rclcpp sensor_msgs geometry_msgs std_msgs tf2_ros)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(observer_node "${cpp_typesupport_target}")
  set_target_properties(observer_node PROPERTIES BUILD_RPATH "${OCTOMAP_LIBRARY_DIRS}" INSTALL_RPATH "${OCTOMAP_LIBRARY_DIRS}")
  install(TARGETS observer_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
  ament_export_include_directories(include)
  ament_export_libraries(uld_occ_observer_core)
  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
endif()
