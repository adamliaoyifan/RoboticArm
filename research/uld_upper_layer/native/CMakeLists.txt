cmake_minimum_required(VERSION 3.16)
project(uld_upper_vacuum LANGUAGES CXX)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
add_library(uld_upper_vacuum SHARED ideal_vacuum.cc)
target_compile_features(uld_upper_vacuum PRIVATE cxx_std_17)
target_link_libraries(uld_upper_vacuum PRIVATE
 ignition-gazebo6::ignition-gazebo6 ignition-plugin1::ignition-plugin1)
