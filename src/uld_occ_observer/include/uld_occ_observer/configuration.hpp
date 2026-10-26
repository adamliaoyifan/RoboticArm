#pragma once
#include "uld_occ_observer/observer.hpp"

namespace uld_occ_observer {
// File IO is separate from the ROS-free algorithm and state owner.
struct ObserverConfiguration {
  GeometryDescriptor geometry;
  DepthZConfig depth;
  double resolution{.01};
  std::string clock_domain;
  std::string digest;
};
ObserverConfiguration load_configuration(const std::string& geometry_file,
                                         const std::string& config_file);
}  // namespace uld_occ_observer
