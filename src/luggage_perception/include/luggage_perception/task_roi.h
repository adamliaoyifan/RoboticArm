// Task ROI: container inner OBB ∪ opening corridor prism.
//
// Limits the MoveIt explore Octomap to points inside the container interior
// or the opening entry corridor. ROI is NOT collision clearance (that's the
// probe corridor); it only decides which observations enter the map.
//
// See docs/urdf_self_filter_task_roi_execution_plan.md section 4.3-4.4 (PR3).
#ifndef LUGGAGE_PERCEPTION_TASK_ROI_H
#define LUGGAGE_PERCEPTION_TASK_ROI_H

#include <string>

#include <Eigen/Geometry>
#include <ros/time.h>

namespace luggage_perception {

/// Geometric description of the task ROI at one instant.
struct RoiGeometry {
  // --- Container inner OBB (oriented bounding box) ---
  Eigen::Vector3d obb_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d obb_half = Eigen::Vector3d::Zero();  ///< half-extents + margin
  Eigen::Quaterniond obb_rot = Eigen::Quaterniond::Identity();

  // --- Opening corridor prism (along the opening normal) ---
  Eigen::Vector3d opening_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d lateral_axis = Eigen::Vector3d::UnitX();
  Eigen::Vector3d vertical_axis = Eigen::Vector3d::UnitY();
  Eigen::Vector3d normal_axis = Eigen::Vector3d::UnitZ();
  double lateral_extent = 0.0;   ///< aperture_width/2 + margin
  double vertical_extent = 0.0;  ///< aperture_height/2 + margin
  double outward_extent = 0.0;   ///< opening_outward_depth
  double inward_extent = 0.0;    ///< min(inner_depth, opening_inward_depth)

  // --- Versioning ---
  int geometry_version = 0;
  std::string geometry_source;  ///< "scene_tf" | "tag_depth" | "depth_only" ...
  ros::Time geometry_stamp;
  int roi_revision = 0;
};

/// Manages the active task ROI with atomic version-gated commits.
class TaskRoi {
 public:
  TaskRoi() = default;

  /// Whether a valid ROI has been committed.
  bool ready() const { return ready_; }

  /// Check if @p base_point (in base_frame) is inside the ROI (OBB ∪ corridor).
  bool contains(const Eigen::Vector3d& base_point) const;

  /// Validate and atomically commit a new ROI geometry.
  ///
  /// On success, replaces the active ROI and increments roi_revision.
  /// On failure (degenerate axes, non-positive sizes), keeps the old ROI
  /// and returns false with @p reason set.
  bool commit(const RoiGeometry& geo, std::string& reason);

  const RoiGeometry& geometry() const { return geo_; }
  int roiRevision() const { return geo_.roi_revision; }
  const std::string& geometrySource() const { return geo_.geometry_source; }
  int geometryVersion() const { return geo_.geometry_version; }

 private:
  static bool validate(const RoiGeometry& geo, std::string& reason);
  static bool insideObb(const RoiGeometry& geo, const Eigen::Vector3d& p);
  static bool insideCorridor(const RoiGeometry& geo, const Eigen::Vector3d& p);

  RoiGeometry geo_;
  bool ready_ = false;
};

}  // namespace luggage_perception

#endif  // LUGGAGE_PERCEPTION_TASK_ROI_H
