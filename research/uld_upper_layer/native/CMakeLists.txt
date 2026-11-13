cmake_minimum_required(VERSION 3.16)
project(uld_upper_vacuum LANGUAGES CXX)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
add_library(uld_upper_vacuum SHARED ideal_vacuum.cc)
target_compile_features(uld_upper_vacuum PRIVATE cxx_std_17)
target_link_libraries(uld_upper_vacuum PRIVATE
 ignition-gazebo6::ignition-gazebo6 ignition-plugin1::ignition-plugin1)
add_executable(uld_carriage_joint_command joint_command_bridge.cc)
target_compile_features(uld_carriage_joint_command PRIVATE cxx_std_17)
target_link_libraries(uld_carriage_joint_command PRIVATE
 ignition-transport11::ignition-transport11 ignition-msgs8::ignition-msgs8)
