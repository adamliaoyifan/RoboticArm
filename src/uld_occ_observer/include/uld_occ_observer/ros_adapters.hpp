#pragma once
#include "uld_occ_observer/observation_worker.hpp"
#include "uld_occ_observer/msg/update_receipt.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
namespace uld_occ_observer {
std::vector<Vec3> optical_points(const sensor_msgs::msg::PointCloud2& cloud);
RigidTransform rigid_transform(const geometry_msgs::msg::TransformStamped& transform,
                               const std::string& optical_frame);
msg::SnapshotIdentity ros_identity(const SnapshotIdentity& identity);
msg::UpdateReceipt ros_receipt(const CompletedObservation& receipt);
}  // namespace uld_occ_observer
