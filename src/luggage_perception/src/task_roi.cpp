// Implementation of TaskRoi.
#include "luggage_perception/task_roi.h"

#include <cmath>

namespace luggage_perception {

namespace {

constexpr double kEps = 1e-9;

double axisCoord(const Eigen::Vector3d& p, const Eigen::Vector3d& axis) {
  return p.dot(axis);
}

}  // namespace

bool TaskRoi::insideObb(const RoiGeometry& geo, const Eigen::Vector3d& p) {
  // Transform point to OBB local frame (inverse rotation, subtract center).
  const Eigen::Vector3d local = geo.obb_rot.inverse() * (p - geo.obb_center);
  return std::abs(local.x()) <= geo.obb_half.x() + kEps &&
         std::abs(local.y()) <= geo.obb_half.y() + kEps &&
         std::abs(local.z()) <= geo.obb_half.z() + kEps;
}

bool TaskRoi::insideCorridor(const RoiGeometry& geo, const Eigen::Vector3d& p) {
  // Project point onto corridor axes (relative to opening center).
  const Eigen::Vector3d d = p - geo.opening_center;
  const double lat = axisCoord(d, geo.lateral_axis);
  const double vert = axisCoord(d, geo.vertical_axis);
  const double norm = axisCoord(d, geo.normal_axis);
  // normal_axis points outward; inward is negative normal direction.
  return std::abs(lat) <= geo.lateral_extent + kEps &&
         std::abs(vert) <= geo.vertical_extent + kEps &&
         norm >= -geo.inward_extent - kEps &&
         norm <= geo.outward_extent + kEps;
}

bool TaskRoi::contains(const Eigen::Vector3d& base_point) const {
  if (!ready_) return false;
  return insideObb(geo_, base_point) || insideCorridor(geo_, base_point);
}

bool TaskRoi::validate(const RoiGeometry& geo, std::string& reason) {
  // OBB half-extents must be positive.
  if (geo.obb_half.x() <= kEps || geo.obb_half.y() <= kEps ||
      geo.obb_half.z() <= kEps) {
    reason = "obb_non_positive_size";
    return false;
  }
  // Corridor axes must be non-degenerate (unit-length-ish).
  if (geo.lateral_axis.squaredNorm() < 0.5 ||
      geo.vertical_axis.squaredNorm() < 0.5 ||
      geo.normal_axis.squaredNorm() < 0.5) {
    reason = "corridor_axis_degenerate";
    return false;
  }
  // Corridor extents must be positive.
  if (geo.lateral_extent <= kEps || geo.vertical_extent <= kEps ||
      geo.outward_extent <= kEps) {
    reason = "corridor_non_positive_extent";
    return false;
  }
  reason = "ok";
  return true;
}

bool TaskRoi::commit(const RoiGeometry& geo, std::string& reason) {
  if (!validate(geo, reason)) {
    return false;  // Keep old ROI; caller may fall back to scene_tf.
  }
  int rev = geo_.roi_revision;
  geo_ = geo;
  geo_.roi_revision = rev + 1;
  ready_ = true;
  reason = "ok";
  return true;
}

}  // namespace luggage_perception
