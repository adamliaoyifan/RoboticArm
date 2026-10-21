cmake_minimum_required(VERSION 3.16)
project(joint_command_capture)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build-stamped watermark: package version + repo git sha at build time.
find_package(Git QUIET)
set(JCC_GIT_SHA "unknown")
if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short=12 HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE JCC_GIT_SHA
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
endif()
message(STATUS "joint_command_capture watermark: ${PROJECT_VERSION} git:${JCC_GIT_SHA}")

add_library(joint_command_capture_core SHARED src/recorder.cpp)
target_include_directories(joint_command_capture_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(joint_command_capture_core PRIVATE
  JCC_VERSION="${PROJECT_VERSION}"
  JCC_GIT_SHA="${JCC_GIT_SHA}")

install(TARGETS joint_command_capture_core
  EXPORT export_joint_command_capture
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT export_joint_command_capture
  DESTINATION share/${PROJECT_NAME}/cmake)

ament_export_targets(export_joint_command_capture)
ament_export_include_directories(include)

ament_python_install_package(joint_command_capture
  PACKAGE_DIR python/joint_command_capture)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_recorder_core test/test_recorder_core.cpp)
  target_link_libraries(test_recorder_core joint_command_capture_core)
  ament_add_gtest(test_conversion_model test/test_conversion_model.cpp)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_jcc_backends test/python/test_backends.py
    ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python=$ENV{PYTHONPATH}")
  ament_add_pytest_test(test_jcc_records_analysis test/python/test_records_and_analysis.py
    ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python=$ENV{PYTHONPATH}")
endif()

ament_package()
