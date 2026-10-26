#include "uld_occ_observer/ros_adapters.hpp"
#include <cstring>
#include <cmath>

namespace uld_occ_observer {
std::vector<Vec3> optical_points(const sensor_msgs::msg::PointCloud2& cloud) {
  if (cloud.is_bigendian || !cloud.point_step ||
      cloud.row_step < static_cast<std::uint64_t>(cloud.width)*cloud.point_step ||
      cloud.data.size() != static_cast<std::uint64_t>(cloud.row_step)*cloud.height)
    throw Error("UNSUPPORTED_OR_INVALID_CLOUD_LAYOUT");
  std::array<sensor_msgs::msg::PointField,3> fields;
  const std::array<std::string,3> names{"x","y","z"};
  for (std::size_t i = 0; i < names.size(); ++i) {
    bool found = false;
    for (const auto& field : cloud.fields) {
      if (field.name != names[i]) continue;
      if (found) throw Error("DUPLICATE_XYZ_FIELD");
      const auto bytes = field.datatype == sensor_msgs::msg::PointField::FLOAT32 ? 4u :
                         field.datatype == sensor_msgs::msg::PointField::FLOAT64 ? 8u : 0u;
      if (!bytes || field.count != 1 || static_cast<std::uint64_t>(field.offset)+bytes > cloud.point_step)
        throw Error("INVALID_XYZ_FIELD");
      fields[i] = field; found = true;
    }
    if (!found) throw Error("MISSING_XYZ_FIELD");
  }
  std::vector<Vec3> points;
  points.reserve(static_cast<std::size_t>(cloud.width)*cloud.height);
  for (std::size_t row = 0; row < cloud.height; ++row) {
    for (std::size_t col = 0; col < cloud.width; ++col) {
      double values[3];
      for (std::size_t i = 0; i < 3; ++i) {
        const auto* input = cloud.data.data() + row*cloud.row_step + col*cloud.point_step + fields[i].offset;
        if (fields[i].datatype == sensor_msgs::msg::PointField::FLOAT32) {
          float value; std::memcpy(&value,input,sizeof(value)); values[i] = value;
        } else std::memcpy(&values[i],input,sizeof(double));
      }
      points.push_back({values[0],values[1],values[2]});
    }
  }
  return points;
}
RigidTransform rigid_transform(const geometry_msgs::msg::TransformStamped& input,
                               const std::string& optical_frame) {
  if (input.header.frame_id != "container_link" || input.child_frame_id != optical_frame)
    throw Error("TF_FRAME_MISMATCH");
  const auto& q = input.transform.rotation;
  const double norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  if (!std::isfinite(norm) || std::abs(norm-1) > 1e-8) throw Error("INVALID_QUATERNION");
  RigidTransform result;
  result.stamp_ns = static_cast<std::int64_t>(input.header.stamp.sec)*1000000000 + input.header.stamp.nanosec;
  const auto& t = input.transform.translation; result.translation = {t.x,t.y,t.z};
  result.rotation = {1-2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y-q.z*q.w), 2*(q.x*q.z+q.y*q.w),
      2*(q.x*q.y+q.z*q.w), 1-2*(q.x*q.x+q.z*q.z), 2*(q.y*q.z-q.x*q.w),
      2*(q.x*q.z-q.y*q.w), 2*(q.y*q.z+q.x*q.w), 1-2*(q.x*q.x+q.y*q.y)};
  return result;
}
msg::SnapshotIdentity ros_identity(const SnapshotIdentity& id) {
  msg::SnapshotIdentity result;
  result.epoch = id.epoch; result.map_revision = id.map_revision;
  result.observed_revision = id.observed_revision;
  result.last_integrated_stamp_ns = id.last_integrated_stamp_ns;
  result.geometry_hash = id.geometry_hash; result.config_digest = id.config_digest;
  result.clock_domain = id.clock_domain; return result;
}
msg::UpdateReceipt ros_receipt(const CompletedObservation& completed) {
  const auto& r = completed.receipt;
  msg::UpdateReceipt out; out.status = r.status;
  out.sensor_id = r.sensor_id; out.observation_id = r.observation_id; out.input_stamp_ns = r.input_stamp_ns; out.identity = ros_identity(r.identity);
  out.raw_points = r.raw_points; out.accepted_points = r.accepted_points;
  out.free_keys = r.free_keys; out.hit_keys = r.hit_keys; out.changed_keys = r.changed_keys;
  out.occupancy_changed_keys = r.occupancy_changed_keys; out.rebuilt_blocks = r.rebuilt_blocks;
  out.preprocessing_ms = r.preprocessing_ms; out.preparation_ms = r.preparation_ms;
  out.fusion_ms = r.fusion_ms; out.total_ms = r.total_ms;
  out.queue_ms = completed.queue_ms; return out;
}
}  // namespace uld_occ_observer
