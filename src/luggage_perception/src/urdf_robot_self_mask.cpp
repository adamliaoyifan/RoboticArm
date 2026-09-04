// Implementation of UrdfRobotSelfMask.
//
// Loads link <collision> geometry from a URDF string, builds inflated
// collision volumes, and removes cloud points inside any volume. Containment
// is computed directly (box/sphere/cylinder math) -- no geometric_shapes
// dependency. Evaluated in base_frame at the cloud's exact stamp TF; output
// preserves the original cloud frame and all PointCloud2 fields.
#include "luggage_perception/urdf_robot_self_mask.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>

#include <ros/ros.h>
#include <urdf/model.h>
#include <geometry_msgs/Transform.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>

namespace luggage_perception {

namespace {

Eigen::Isometry3d toEigen(const urdf::Pose& pose) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(
      pose.position.x, pose.position.y, pose.position.z);
  t.rotate(Eigen::Quaterniond(
      pose.rotation.w, pose.rotation.x, pose.rotation.y,
      pose.rotation.z));
  return t;
}

// Manual conversion from geometry_msgs::Transform to Eigen::Isometry3d.
// (tf2_eigen only provides fromMsg for Pose, not Transform.)
Eigen::Isometry3d transformToEigen(const geometry_msgs::Transform& t) {
  Eigen::Isometry3d e = Eigen::Isometry3d::Identity();
  e.translation() = Eigen::Vector3d(
      t.translation.x, t.translation.y, t.translation.z);
  e.rotate(Eigen::Quaterniond(
      t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z));
  return e;
}

std::string hashString(const std::string& s) {
  uint64_t h = 1469598103934665603ULL;
  for (char c : s) {
    h ^= static_cast<uint64_t>(static_cast<unsigned char>(c));
    h *= 1099511628211ULL;
  }
  std::ostringstream ss;
  ss << std::hex << h;
  return ss.str();
}

}  // namespace

UrdfRobotSelfMask::UrdfRobotSelfMask(const std::string& urdf_xml,
                                     const std::string& planning_group,
                                     const std::vector<std::string>& extra_links,
                                     double body_padding, double body_scale)
    : urdf_xml_(urdf_xml),
      planning_group_(planning_group),
      extra_links_(extra_links),
      body_padding_(body_padding),
      body_scale_(body_scale) {
  ready_ = buildFromXml();
}

bool UrdfRobotSelfMask::buildFromXml() {
  if (urdf_xml_.empty()) {
    ROS_ERROR("[UrdfRobotSelfMask] empty URDF XML");
    return false;
  }
  robot_hash_ = hashString(urdf_xml_);

  urdf::Model model;
  if (!model.initString(urdf_xml_)) {
    ROS_ERROR("[UrdfRobotSelfMask] failed to parse URDF");
    return false;
  }
  robot_model_name_ = model.getName();

  // Collect link names: arm chain (elfin_link*/elfin_end_link) + extras.
  std::vector<urdf::LinkSharedPtr> all_links;
  model.getLinks(all_links);
  for (const auto& link : all_links) {
    const std::string& n = link->name;
    if (n.rfind("elfin_link", 0) == 0 || n == "elfin_end_link") {
      active_link_names_.push_back(n);
    }
  }
  for (const auto& n : extra_links_) {
    active_link_names_.push_back(n);
  }
  std::sort(active_link_names_.begin(), active_link_names_.end());
  active_link_names_.erase(
      std::unique(active_link_names_.begin(), active_link_names_.end()),
      active_link_names_.end());

  for (const auto& link_name : active_link_names_) {
    urdf::LinkSharedPtr link;
    model.getLink(link_name, link);  // returns void; fills link.
    if (!link) continue;

    std::vector<urdf::CollisionSharedPtr> cols;
    if (!link->collision_array.empty()) {
      cols = link->collision_array;
    } else if (link->collision) {
      cols.push_back(link->collision);
    }
    for (const auto& col : cols) {
      CollisionBody cb;
      cb.link_name = link_name;
      cb.padding = body_padding_;
      cb.scale = body_scale_;
      cb.collision_origin_in_link = toEigen(col->origin);

      const urdf::GeometrySharedPtr& geom = col->geometry;
      if (!geom) continue;
      if (auto box = std::dynamic_pointer_cast<urdf::Box>(geom)) {
        cb.type = ShapeType::kBox;
        cb.shape_name = "box";
        cb.dims = Eigen::Vector3d(box->dim.x, box->dim.y, box->dim.z);
      } else if (auto sph = std::dynamic_pointer_cast<urdf::Sphere>(geom)) {
        cb.type = ShapeType::kSphere;
        cb.shape_name = "sphere";
        cb.dims = Eigen::Vector3d(sph->radius, 0.0, 0.0);
      } else if (auto cyl = std::dynamic_pointer_cast<urdf::Cylinder>(geom)) {
        cb.type = ShapeType::kCylinder;
        cb.shape_name = "cylinder";
        cb.dims = Eigen::Vector3d(cyl->radius, cyl->length, 0.0);
      } else if (auto msh = std::dynamic_pointer_cast<urdf::Mesh>(geom)) {
        // Load the mesh and compute its AABB as a conservative box body.
        shapes::Mesh* sm = shapes::createMeshFromResource(
            msh->filename,
            Eigen::Vector3d(msh->scale.x, msh->scale.y, msh->scale.z));
        if (!sm || sm->vertex_count == 0) {
          ROS_WARN_ONCE(
              "[UrdfRobotSelfMask] skipping empty mesh on link %s",
              link_name.c_str());
          continue;
        }
        Eigen::Vector3d min_v, max_v;
        for (unsigned int vi = 0; vi < sm->vertex_count; ++vi) {
          Eigen::Vector3d v(
              sm->vertices[3 * vi], sm->vertices[3 * vi + 1],
              sm->vertices[3 * vi + 2]);
          if (vi == 0) {
            min_v = max_v = v;
          } else {
            min_v = min_v.cwiseMin(v);
            max_v = max_v.cwiseMax(v);
          }
        }
        delete sm;
        cb.type = ShapeType::kBox;  // treat mesh AABB as a box
        cb.shape_name = "mesh_aabb";
        cb.dims = max_v - min_v;
        // Shift the collision origin to the AABB center (in the mesh local
        // frame), composed with the URDF collision origin (link -> mesh).
        Eigen::Vector3d aabb_center = (min_v + max_v) * 0.5;
        cb.collision_origin_in_link =
            toEigen(col->origin) * Eigen::Translation3d(aabb_center);
      } else {
        // Mesh: skip (would need resource retrieval).
        ROS_WARN_ONCE(
            "[UrdfRobotSelfMask] skipping mesh collision on link %s",
            link_name.c_str());
        continue;
      }
      bodies_.push_back(std::move(cb));
    }
  }

  if (bodies_.empty()) {
    ROS_ERROR("[UrdfRobotSelfMask] no collision bodies loaded from URDF");
    return false;
  }
  ROS_INFO(
      "[UrdfRobotSelfMask] ready: model=%s hash=%s bodies=%zu links=%zu",
      robot_model_name_.c_str(), robot_hash_.substr(0, 8).c_str(),
      bodies_.size(), active_link_names_.size());
  return true;
}

std::vector<std::string> UrdfRobotSelfMask::activeLinks() const {
  return active_link_names_;
}

bool UrdfRobotSelfMask::containsLocal(const CollisionBody& cb,
                                      const Eigen::Vector3d& p) {
  const double pad = cb.padding;
  switch (cb.type) {
    case ShapeType::kBox: {
      const double hx = cb.dims.x() * cb.scale * 0.5 + pad;
      const double hy = cb.dims.y() * cb.scale * 0.5 + pad;
      const double hz = cb.dims.z() * cb.scale * 0.5 + pad;
      return std::abs(p.x()) <= hx && std::abs(p.y()) <= hy &&
             std::abs(p.z()) <= hz;
    }
    case ShapeType::kSphere: {
      const double r = cb.dims.x() * cb.scale + pad;
      return p.squaredNorm() <= r * r;
    }
    case ShapeType::kCylinder: {
      const double r = cb.dims.x() * cb.scale + pad;
      const double h = cb.dims.y() * cb.scale * 0.5 + pad;
      return (p.x() * p.x() + p.y() * p.y() <= r * r) &&
             (std::abs(p.z()) <= h);
    }
    default:
      return false;  // mesh: not masked
  }
}

bool UrdfRobotSelfMask::containsInLink(std::size_t i,
                                       const Eigen::Vector3d& p) const {
  if (i >= bodies_.size()) return false;
  const CollisionBody& cb = bodies_[i];
  Eigen::Vector3d local = cb.collision_origin_in_link.inverse() * p;
  return containsLocal(cb, local);
}

bool UrdfRobotSelfMask::mask(const sensor_msgs::PointCloud2& cloud,
                             const tf2_ros::Buffer& tf_buffer,
                             const std::string& base_frame,
                             sensor_msgs::PointCloud2& filtered_cloud,
                             MaskResult& result) const {
  result = MaskResult();
  result.input_count =
      static_cast<int>(cloud.width) * static_cast<int>(cloud.height);

  if (!ready_) {
    result.reason = "mask_not_ready";
    return false;
  }

  const ros::Time stamp = cloud.header.stamp;
  const std::string sensor_frame = cloud.header.frame_id;

  int x_off = -1, y_off = -1, z_off = -1;
  for (const auto& f : cloud.fields) {
    if (f.name == "x") x_off = f.offset;
    else if (f.name == "y") y_off = f.offset;
    else if (f.name == "z") z_off = f.offset;
  }
  if (x_off < 0 || y_off < 0 || z_off < 0) {
    result.reason = "no_xyz_fields";
    return false;
  }

  // Exact-stamp TF: sensor -> base.
  Eigen::Isometry3d sensor_to_base;
  try {
    geometry_msgs::TransformStamped tfm = tf_buffer.lookupTransform(
        base_frame, sensor_frame, stamp, ros::Duration(0.0));
    sensor_to_base = transformToEigen(tfm.transform);
  } catch (const tf2::TransformException&) {
    result.reason = "exact_stamp_tf_missing";
    result.tf_missing_links.push_back(sensor_frame);
    return false;
  }

  // Per-body world (base-frame) pose inverses at the cloud stamp.
  std::vector<Eigen::Isometry3d> world_inv(bodies_.size());
  for (std::size_t bi = 0; bi < bodies_.size(); ++bi) {
    const CollisionBody& cb = bodies_[bi];
    geometry_msgs::TransformStamped ltf;
    try {
      ltf = tf_buffer.lookupTransform(
          base_frame, cb.link_name, stamp, ros::Duration(0.0));
    } catch (const tf2::TransformException&) {
      result.reason = "exact_stamp_tf_missing";
      result.tf_missing_links.push_back(cb.link_name);
      return false;
    }
    Eigen::Isometry3d link_to_base = transformToEigen(ltf.transform);
    world_inv[bi] = (link_to_base * cb.collision_origin_in_link).inverse();
  }

  const int point_step = cloud.point_step;
  const int n_points = result.input_count;
  std::vector<uint8_t> kept;
  kept.reserve(cloud.data.size());

  const uint8_t* src = cloud.data.data();
  for (int i = 0; i < n_points; ++i) {
    const uint8_t* p = src + i * point_step;
    float sx, sy, sz;
    std::memcpy(&sx, p + x_off, sizeof(float));
    std::memcpy(&sy, p + y_off, sizeof(float));
    std::memcpy(&sz, p + z_off, sizeof(float));
    if (!std::isfinite(sx) || !std::isfinite(sy) || !std::isfinite(sz)) {
      kept.insert(kept.end(), p, p + point_step);
      continue;
    }
    Eigen::Vector3d bp = sensor_to_base * Eigen::Vector3d(sx, sy, sz);
    bool inside = false;
    for (std::size_t bi = 0; bi < bodies_.size(); ++bi) {
      if (containsLocal(bodies_[bi], world_inv[bi] * bp)) {
        inside = true;
        break;
      }
    }
    if (inside) {
      result.dropped_robot++;
    } else {
      kept.insert(kept.end(), p, p + point_step);
    }
  }
  result.kept_count =
      static_cast<int>(kept.size()) / std::max(1, point_step);

  filtered_cloud = cloud;
  filtered_cloud.width = result.kept_count;
  filtered_cloud.height = 1;
  filtered_cloud.is_dense = cloud.is_dense;
  filtered_cloud.data = std::move(kept);
  result.reason = "ok";
  result.published = true;
  return true;
}

bool UrdfRobotSelfMask::pointOverlapsRobot(
    const Eigen::Vector3d& base_point, const tf2_ros::Buffer& tf_buffer,
    const std::string& base_frame, const ros::Time& stamp,
    std::string& missing_link) const {
  if (!ready_) {
    missing_link = "mask_not_ready";
    return false;
  }
  for (const auto& cb : bodies_) {
    geometry_msgs::TransformStamped ltf;
    try {
      ltf = tf_buffer.lookupTransform(
          base_frame, cb.link_name, stamp, ros::Duration(0.0));
    } catch (const tf2::TransformException&) {
      missing_link = cb.link_name;
      return false;
    }
    Eigen::Isometry3d link_to_base = transformToEigen(ltf.transform);
    Eigen::Vector3d local =
        (link_to_base * cb.collision_origin_in_link).inverse() * base_point;
    if (containsLocal(cb, local)) {
      return true;
    }
  }
  return false;
}

}  // namespace luggage_perception
