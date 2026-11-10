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
add_library(uld_occ_observer_core src/validity.cpp
  src/preprocessing.cpp src/geometry.cpp src/block_storage.cpp src/snapshot.cpp
  src/occupancy_grid.cpp src/observer.cpp
  src/snapshot_repository.cpp src/observation_worker.cpp
  src/observation_ledger.cpp src/observation_ingress.cpp src/snapshot_wait_board.cpp
  src/observation_terminal.cpp src/interface_trace.cpp
  src/freespace_query.cpp)
target_include_directories(uld_occ_observer_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(uld_occ_observer_core PUBLIC ${OCTOMAP_LIBRARIES} OpenSSL::Crypto Threads::Threads)
target_compile_options(uld_occ_observer_core PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(uld_occ_observer_core PROPERTIES
  BUILD_RPATH "${OCTOMAP_LIBRARY_DIRS}" INSTALL_RPATH "${OCTOMAP_LIBRARY_DIRS}")
add_library(uld_occ_observer_io src/configuration.cpp src/checkpoint_io.cpp src/serialization.cpp)
target_link_libraries(uld_occ_observer_io PUBLIC uld_occ_observer_core yaml-cpp)
include(CTest)
if(BUILD_TESTING)
  add_executable(observer_profile_checks test/test_profiling.cpp test/fusion_profile_checks.cpp
                 test/block_storage_checks.cpp test/apply_batch_checks.cpp test/query_content_digest.cpp)
  target_link_libraries(observer_profile_checks PRIVATE uld_occ_observer_io)
  add_test(NAME observer_profile_checks COMMAND observer_profile_checks)
  add_executable(observer_keyset_checks test/test_occupancy_keyset.cpp)
  target_link_libraries(observer_keyset_checks PRIVATE uld_occ_observer_io)
  target_compile_definitions(observer_keyset_checks PRIVATE OBSERVER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  add_test(NAME observer_keyset_checks COMMAND observer_keyset_checks)
  add_executable(observer_validity_checks test/test_validity.cpp)
  target_link_libraries(observer_validity_checks PRIVATE uld_occ_observer_core)
  add_test(NAME observer_validity_checks COMMAND observer_validity_checks)
  add_test(NAME observer_hot_path_disassembly
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/check_hot_path.py
            ${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/uld_occ_observer_core.dir/src/occupancy_grid.cpp.o)
  add_test(NAME observer_config_entry
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_config_entry.py
            $<TARGET_FILE:observer_cli> $<TARGET_FILE:observer_profile_cli>
            $<TARGET_FILE:observer_eval_native>
            ${CMAKE_CURRENT_SOURCE_DIR}/config/observer.yaml)
  add_executable(observer_repair_s1_behavior test/test_repair_s1_behavior.cpp)
  add_executable(observer_freespace_behavior test/test_freespace_behavior.cpp)
  target_link_libraries(observer_repair_s1_behavior PRIVATE uld_occ_observer_io)
  target_link_libraries(observer_freespace_behavior PRIVATE uld_occ_observer_io)
  add_test(NAME observer_repair_s1_behavior COMMAND observer_repair_s1_behavior)
  add_test(NAME observer_freespace_behavior COMMAND observer_freespace_behavior)
  add_test(NAME repair_s0_judge
    COMMAND python3 -m pytest -q --tb=line ${CMAKE_CURRENT_SOURCE_DIR}/test/test_repair_s0.py)
endif()
add_executable(observer_profile_cli src/profile_cli.cpp test/query_content_digest.cpp)
target_include_directories(observer_profile_cli PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
target_link_libraries(observer_profile_cli PRIVATE uld_occ_observer_io)
set_target_properties(observer_profile_cli PROPERTIES BUILD_RPATH "${OCTOMAP_LIBRARY_DIRS}" INSTALL_RPATH "${OCTOMAP_LIBRARY_DIRS}")
install(TARGETS observer_profile_cli DESTINATION lib/${PROJECT_NAME})
add_executable(observer_cli src/observer_cli.cpp)
target_link_libraries(observer_cli PRIVATE uld_occ_observer_io)
set_target_properties(observer_cli PROPERTIES BUILD_RPATH "${OCTOMAP_LIBRARY_DIRS}" INSTALL_RPATH "${OCTOMAP_LIBRARY_DIRS}")
install(TARGETS observer_cli DESTINATION lib/${PROJECT_NAME})
add_executable(observer_eval_native test/eval_native.cpp test/independent_oracle.cpp test/load_model.cpp)
target_include_directories(observer_eval_native PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
target_link_libraries(observer_eval_native PRIVATE uld_occ_observer_io)
set_target_properties(observer_eval_native PROPERTIES
  BUILD_RPATH "${OCTOMAP_LIBRARY_DIRS}" INSTALL_RPATH "${OCTOMAP_LIBRARY_DIRS}")
add_executable(observer_ray_probe test/native_ray_probe.cpp test/independent_oracle.cpp)
target_include_directories(observer_ray_probe PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(observer_ray_probe PRIVATE ${OCTOMAP_LIBRARIES})
set_target_properties(observer_ray_probe PROPERTIES
  BUILD_RPATH "${OCTOMAP_LIBRARY_DIRS}" INSTALL_RPATH "${OCTOMAP_LIBRARY_DIRS}")
install(TARGETS observer_eval_native observer_ray_probe DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS test/run_package_tests.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/export_geometry.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
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
    "msg/SnapshotIdentity.msg" "msg/UpdateReceipt.msg" "msg/ValidityStamp.msg"
    "srv/IntegrateObservation.srv" "srv/GetSnapshot.srv" "srv/ReadCells.srv"
    "srv/ReadSurface.srv" "srv/ReleaseSnapshot.srv" "srv/ResetObservation.srv"
    "srv/CheckpointObservation.srv"
    "srv/GetObservationStatus.srv" "srv/WaitSnapshot.srv" "srv/GetResourceUsage.srv"
    "srv/SetBehaviorHook.srv" "srv/QueryFreespace.srv" "srv/GetValidity.srv"
    DEPENDENCIES sensor_msgs geometry_msgs)
  add_executable(observer_node src/observer_node.cpp src/ros_adapters.cpp)
  target_link_libraries(observer_node uld_occ_observer_io)
  ament_target_dependencies(observer_node rclcpp sensor_msgs geometry_msgs std_msgs tf2_ros)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(observer_node "${cpp_typesupport_target}")
  set_target_properties(observer_node PROPERTIES BUILD_RPATH "${OCTOMAP_LIBRARY_DIRS}" INSTALL_RPATH "${OCTOMAP_LIBRARY_DIRS}")
  add_executable(ros_discretize_client test/ros_discretize_client.cpp)
  target_link_libraries(ros_discretize_client "${cpp_typesupport_target}")
  ament_target_dependencies(ros_discretize_client rclcpp sensor_msgs)
  install(TARGETS observer_node ros_discretize_client DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
  if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(observer_eval_cli
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_eval_cli.py")
  endif()
  ament_export_include_directories(include)
  ament_export_libraries(uld_occ_observer_core)
  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
endif()
