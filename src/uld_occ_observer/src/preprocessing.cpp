#include "uld_occ_observer/preprocessing.hpp"
#include <cmath>

namespace uld_occ_observer {
bool finite(const Vec3& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

void DepthZConfig::validate() const {
  if (!std::isfinite(min_m) || !std::isfinite(max_m) || min_m <= 0 || min_m >= max_m)
    throw Error("INVALID_DEPTH_Z_CONFIG");
}

void RigidTransform::validate(std::int64_t acquisition_stamp_ns) const {
  if (acquisition_stamp_ns <= 0 || stamp_ns != acquisition_stamp_ns)
    throw Error("TF_STAMP_MISMATCH");
  if (!finite(translation)) throw Error("INVALID_TRANSFORM");
  for (double value : rotation)
    if (!std::isfinite(value)) throw Error("INVALID_TRANSFORM");
  for (int row = 0; row < 3; ++row) {
    for (int other = 0; other < 3; ++other) {
      double dot = 0;
      for (int col = 0; col < 3; ++col)
        dot += rotation[3*row+col] * rotation[3*other+col];
      if (std::abs(dot - (row == other ? 1.0 : 0.0)) > 1e-8)
        throw Error("INVALID_TRANSFORM");
    }
  }
  const auto& r = rotation;
  const double determinant = r[0]*(r[4]*r[8]-r[5]*r[7]) -
      r[1]*(r[3]*r[8]-r[5]*r[6]) + r[2]*(r[3]*r[7]-r[4]*r[6]);
  if (std::abs(determinant-1.0) > 1e-8) throw Error("INVALID_TRANSFORM");
}

Vec3 RigidTransform::apply(const Vec3& p) const {
  return {rotation[0]*p.x + rotation[1]*p.y + rotation[2]*p.z + translation.x,
          rotation[3]*p.x + rotation[4]*p.y + rotation[5]*p.z + translation.y,
          rotation[6]*p.x + rotation[7]*p.y + rotation[8]*p.z + translation.z};
}

OpticalDepthPreprocessor::OpticalDepthPreprocessor(DepthZConfig config) : config_(config) {
  config_.validate();
}

FilterResult OpticalDepthPreprocessor::prepare(const std::vector<Vec3>& points) const {
  FilterResult result;
  result.raw_count = points.size();
  result.accepted_mask.resize(points.size(), 0);
  result.optical_points.reserve(points.size());
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto& point = points[index];
    if (!finite(point)) { ++result.nonfinite_count; continue; }
    if (point.z < config_.min_m) { ++result.below_count; continue; }
    if (point.z > config_.max_m) { ++result.above_count; continue; }
    result.accepted_mask[index] = 1;
    result.optical_points.push_back(point);
  }
  return result;
}

UldObservation UldFrameAdapter::transform(const ObservationIdentity& identity,
                                         const FilterResult& filtered,
                                         const RigidTransform& transform) {
  if (identity.sensor_id.empty() || identity.observation_id.empty() ||
      identity.optical_frame.empty() || identity.clock_domain.empty() ||
      identity.geometry_hash.empty()) throw Error("INVALID_OBSERVATION_IDENTITY");
  transform.validate(identity.stamp_ns);
  UldObservation result;
  result.identity = identity;
  result.origin = transform.apply({0, 0, 0});
  result.raw_count = filtered.raw_count;
  result.rejected_count = filtered.raw_count - filtered.optical_points.size();
  result.points.reserve(filtered.optical_points.size());
  for (const auto& point : filtered.optical_points) {
    const auto transformed = transform.apply(point);
    if (!finite(transformed)) throw Error("INVALID_TRANSFORM_OUTPUT");
    result.points.push_back(transformed);
  }
  return result;
}
}  // namespace uld_occ_observer
