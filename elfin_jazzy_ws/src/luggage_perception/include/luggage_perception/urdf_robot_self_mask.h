// URDF-driven robot self point-cloud mask.
//
// Builds a set of inflated collision volumes from a URDF string (link
// <collision> geometry), then removes points that fall inside any volume.
// Containment is computed directly (box/sphere/cylinder math) so no
// geometric_shapes::bodies dependency is needed. Auto-adapts to S20/S30
// link lengths and tool/camera mounts because it reads the actual collision
// geometry from the URDF.
//
// See docs/urdf_self_filter_task_roi_execution_plan.md section 4.1 (PR2).
#ifndef LUGGAGE_PERCEPTION_URDF_ROBOT_SELF_MASK_H
#define LUGGAGE_PERCEPTION_URDF_ROBOT_SELF_MASK_H

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>

namespace luggage_perception {

enum class ShapeType { kBox, kSphere, kCylinder, kMesh };

/// One inflated collision volume, in a link's local frame.
struct CollisionBody {
  std::string link_name;
  std::string shape_name;  ///< "box" | "sphere" | "cylinder" | "mesh".
  ShapeType type = ShapeType::kMesh;
  /// box: (size_x, size_y, size_z); sphere: (radius, 0, 0);
  /// cylinder: (radius, length, 0).
  Eigen::Vector3d dims = Eigen::Vector3d::Zero();
  Eigen::Isometry3d collision_origin_in_link = Eigen::Isometry3d::Identity();
  double padding = 0.0;
  double scale = 1.0;
};

/// Result of masking one cloud frame.
struct MaskResult {
  bool published = false;          ///< False => frame dropped (fail-closed).
  std::string reason;              ///< "ok" | "exact_stamp_tf_missing" | ...
  int input_count = 0;
  int dropped_robot = 0;
  int kept_count = 0;
  double cloud_age_sec = 0.0;
  std::vector<std::string> tf_missing_links;
};

/// Loads URDF collision geometry and masks robot points from a point cloud.
class UrdfRobotSelfMask {
 public:
  UrdfRobotSelfMask(const std::string& urdf_xml,
                    const std::string& planning_group,
                    const std::vector<std::string>& extra_links,
                    double body_padding = 0.03,
                    double body_scale = 1.05);

  ~UrdfRobotSelfMask() = default;

  bool ready() const { return ready_; }
  const std::string& robotModelName() const { return robot_model_name_; }
  const std::string& robotDescriptionHash() const { return robot_hash_; }
  int collisionBodyCount() const { return static_cast<int>(bodies_.size()); }
  std::vector<std::string> activeLinks() const;

  std::string bodyLinkName(std::size_t i) const {
    return i < bodies_.size() ? bodies_[i].link_name : std::string();
  }
  std::string bodyShapeName(std::size_t i) const {
    return i < bodies_.size() ? bodies_[i].shape_name : std::string();
  }

  /// Check whether a point (in the body's LINK frame) is inside body @p i.
  bool containsInLink(std::size_t i, const Eigen::Vector3d& p) const;

  /// Mask robot points from @p cloud. Fail-closed on missing TF.
  bool mask(const sensor_msgs::PointCloud2& cloud,
            const tf2_ros::Buffer& tf_buffer,
            const std::string& base_frame,
            sensor_msgs::PointCloud2& filtered_cloud,
            MaskResult& result) const;

  /// Check whether a base-frame point overlaps any body at @p stamp.
  bool pointOverlapsRobot(const Eigen::Vector3d& base_point,
                          const tf2_ros::Buffer& tf_buffer,
                          const std::string& base_frame,
                          const ros::Time& stamp,
                          std::string& missing_link) const;

 private:
  bool buildFromXml();
  /// Containment in the body's local (collision-origin) frame.
  static bool containsLocal(const CollisionBody& cb,
                            const Eigen::Vector3d& local_point);

  std::string urdf_xml_;
  std::string planning_group_;
  std::vector<std::string> extra_links_;
  double body_padding_;
  double body_scale_;

  bool ready_ = false;
  std::string robot_model_name_;
  std::string robot_hash_;
  std::vector<std::string> active_link_names_;
  std::vector<CollisionBody> bodies_;
};

}  // namespace luggage_perception

#endif  // LUGGAGE_PERCEPTION_URDF_ROBOT_SELF_MASK_H
