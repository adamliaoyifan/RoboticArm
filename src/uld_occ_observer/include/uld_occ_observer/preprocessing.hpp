#pragma once
#include "uld_occ_observer/types.hpp"

namespace uld_occ_observer {

// Sensor-specific validity belongs here, before ULD transforms or ray traversal.
// This class is stateless after construction; config changes require a new owner.
class OpticalDepthPreprocessor {
 public:
  explicit OpticalDepthPreprocessor(DepthZConfig config);
  FilterResult prepare(const std::vector<Vec3>& optical_points_m) const;
  DepthZConfig config() const { return config_; }
 private:
  const DepthZConfig config_;
};

class UldFrameAdapter {
 public:
  static UldObservation transform(const ObservationIdentity& identity,
                                  const FilterResult& filtered,
                                  const RigidTransform& uld_from_optical);
};

}  // namespace uld_occ_observer
