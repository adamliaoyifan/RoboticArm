// Task cloud filter node (PR3).
//
// Subscribes to the input depth cloud, applies the URDF robot self-mask (PR2)
// and the Task ROI (container OBB ∪ opening corridor, PR3), then publishes:
//   /camera/depth/points_robot_filtered  -- robot-filtered global cloud
//   /camera/depth/points_task_roi        -- robot-filtered ∩ ROI cloud
//   /camera/depth/points_planning        -- mode-dependent planning cloud
//   /task_cloud_filter/stats_json        -- status (§7.2 schema)
//
// Fail-closed: if the mask or ROI is not ready, or TF is missing at the
// cloud stamp, NO task/robot-filtered cloud is published (unsafe passthrough
// count stays zero).
//
// See docs/urdf_self_filter_task_roi_execution_plan.md sections 4.1-4.5.
#include <cmath>
#include <cstring>
#include <memory>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Transform.h>

#include "luggage_perception/urdf_robot_self_mask.h"
#include "luggage_perception/task_roi.h"

using luggage_perception::UrdfRobotSelfMask;
using luggage_perception::TaskRoi;
using luggage_perception::RoiGeometry;
using luggage_perception::MaskResult;

namespace {

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

// Build a JSON stats string (avoids jsoncpp dependency).
std::string buildStatsJson(
    bool mask_ready, const std::string& mask_reason,
    const std::string& robot_model, const std::string& robot_hash,
    int mask_revision, bool roi_ready, int roi_revision,
    int geometry_version, const std::string& geometry_source,
    const std::string& planning_mode, int planning_mode_revision,
    double cloud_stamp, double cloud_age, int input_count,
    int dropped_robot, int roi_kept, int roi_outside_count,
    int unsafe_passthrough, const std::string& tf_missing) {
  std::ostringstream ss;
  ss << "{";
  ss << "\"ready\":" << (mask_ready && roi_ready ? "true" : "false");
  ss << ",\"reason\":\"" << mask_reason << "\"";
  ss << ",\"robot_model\":\"" << robot_model << "\"";
  ss << ",\"robot_description_hash\":\"" << robot_hash << "\"";
  ss << ",\"mask_revision\":" << mask_revision;
  ss << ",\"geometry_version\":" << geometry_version;
  ss << ",\"geometry_source\":\"" << geometry_source << "\"";
  ss << ",\"roi_revision\":" << roi_revision;
  ss << ",\"task_roi_ready\":" << (roi_ready ? "true" : "false");
  ss << ",\"planning_mode\":\"" << planning_mode << "\"";
  ss << ",\"planning_mode_revision\":" << planning_mode_revision;
  ss << ",\"cloud_stamp\":" << cloud_stamp;
  ss << ",\"cloud_age\":" << cloud_age;
  ss << ",\"input_count\":" << input_count;
  ss << ",\"dropped_robot\":" << dropped_robot;
  ss << ",\"roi_kept\":" << roi_kept;
  ss << ",\"roi_outside_count\":" << roi_outside_count;
  ss << ",\"post_filter_robot_overlap_count\":0";
  ss << ",\"unsafe_passthrough_count\":" << unsafe_passthrough;
  ss << ",\"tf_missing_links\":\"" << tf_missing << "\"";
  ss << "}";
  return ss.str();
}

// Crop a PointCloud2 to points inside the ROI (in base_frame). Preserves all
// fields. Returns the cropped cloud and roi_outside_count.
int cropToRoi(const sensor_msgs::PointCloud2& cloud,
              const TaskRoi& roi,
              const Eigen::Isometry3d& sensor_to_base,
              const std::string& base_frame,
              sensor_msgs::PointCloud2& out) {
  int x_off = -1, y_off = -1, z_off = -1;
  for (const auto& f : cloud.fields) {
    if (f.name == "x") x_off = f.offset;
    else if (f.name == "y") y_off = f.offset;
    else if (f.name == "z") z_off = f.offset;
  }
  if (x_off < 0) return -1;

  const int step = cloud.point_step;
  const int n = cloud.width * cloud.height;
  std::vector<uint8_t> kept;
  kept.reserve(cloud.data.size());
  int outside = 0;
  const uint8_t* src = cloud.data.data();
  for (int i = 0; i < n; ++i) {
    const uint8_t* p = src + i * step;
    float sx, sy, sz;
    std::memcpy(&sx, p + x_off, sizeof(float));
    std::memcpy(&sy, p + y_off, sizeof(float));
    std::memcpy(&sz, p + z_off, sizeof(float));
    if (!std::isfinite(sx) || !std::isfinite(sy) || !std::isfinite(sz)) {
      ++outside;
      continue;
    }
    Eigen::Vector3d bp = sensor_to_base * Eigen::Vector3d(sx, sy, sz);
    if (roi.contains(bp)) {
      kept.insert(kept.end(), p, p + step);
    } else {
      ++outside;
    }
  }
  out = cloud;
  out.width = static_cast<uint32_t>(kept.size() / std::max(1, step));
  out.height = 1;
  out.data = std::move(kept);
  return outside;
}

}  // namespace

class TaskCloudFilterNode {
 public:
  TaskCloudFilterNode()
      : nh_("~"),
        tf_buffer_(),
        tf_listener_(tf_buffer_) {
    // --- Load URDF self-mask ---
    std::string urdf_xml;
    std::string robot_desc_param = nh_.param<std::string>(
        "robot_description", "/robot_description");
    ros::NodeHandle().getParam(robot_desc_param, urdf_xml);

    std::string planning_group = nh_.param<std::string>(
        "planning_group", "elfin_arm");
    double padding = nh_.param<double>("body_padding", 0.03);
    double scale = nh_.param<double>("body_scale", 1.05);
    // Extra links (camera, suction, adapter) from param list.
    std::vector<std::string> extra_links;
    nh_.param("extra_links", extra_links, std::vector<std::string>());

    mask_ = std::make_unique<UrdfRobotSelfMask>(
        urdf_xml, planning_group, extra_links, padding, scale);
    if (!mask_->ready()) {
      ROS_ERROR("[task_cloud_filter] URDF self-mask not ready");
    }

    // --- Build Task ROI from params (scene_tf fallback) ---
    buildRoiFromParams();

    // --- Planning mode ---
    planning_mode_ = nh_.param<std::string>(
        "planning_mode", "EXPLORE_CONTAINER");

    // --- Publishers / subscriber ---
    std::string input_topic = nh_.param<std::string>(
        "input_cloud", "/camera/depth/points_filtered");
    pub_robot_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/camera/depth/points_robot_filtered", 1);
    pub_roi_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/camera/depth/points_task_roi", 1);
    pub_planning_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/camera/depth/points_planning", 1);
    pub_stats_ = nh_.advertise<std_msgs::String>(
        "/task_cloud_filter/stats_json", 1, /*latch=*/true);
    sub_cloud_ = nh_.subscribe(
        input_topic, 1, &TaskCloudFilterNode::cloudCallback, this);

    ROS_INFO("[task_cloud_filter] ready=%s mode=%s roi=%s",
             mask_->ready() ? "true" : "false",
             planning_mode_.c_str(),
             roi_.ready() ? "true" : "false");
  }

  void buildRoiFromParams() {
    RoiGeometry g;
    g.geometry_source = "scene_tf";
    g.geometry_version = 0;

    // Container inner OBB.
    std::vector<double> center, dims;
    nh_.param("container_center", center, std::vector<double>{0, 0, 0});
    nh_.param("container_dims", dims, std::vector<double>{1.0, 1.0, 1.0});
    double yaw = nh_.param<double>("container_yaw", 0.0);
    double margin = nh_.param<double>("container_margin", 0.03);
    if (center.size() >= 3 && dims.size() >= 3) {
      g.obb_center = Eigen::Vector3d(center[0], center[1], center[2]);
      g.obb_half = Eigen::Vector3d(
          dims[0] * 0.5 + margin, dims[1] * 0.5 + margin,
          dims[2] * 0.5 + margin);
      g.obb_rot = Eigen::Quaterniond(
          Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    }

    // Opening corridor.
    std::vector<double> open_center, open_normal;
    nh_.param("opening_center", open_center, std::vector<double>{0, 0, 0});
    nh_.param("opening_normal", open_normal, std::vector<double>{0, 0, 1});
    double aw = nh_.param<double>("aperture_width", 1.0);
    double ah = nh_.param<double>("aperture_height", 1.0);
    double lmargin = nh_.param<double>("opening_lateral_margin", 0.05);
    double vmargin = nh_.param<double>("opening_vertical_margin", 0.05);
    double outward = nh_.param<double>("opening_outward_depth", 0.35);
    double inward = nh_.param<double>("opening_inward_depth", 0.30);
    if (open_center.size() >= 3 && open_normal.size() >= 3) {
      g.opening_center = Eigen::Vector3d(
          open_center[0], open_center[1], open_center[2]);
      Eigen::Vector3d n(open_normal[0], open_normal[1], open_normal[2]);
      n.normalize();
      g.normal_axis = n;
      // Lateral = normal × Z; vertical = normal × lateral.
      Eigen::Vector3d lat = n.cross(Eigen::Vector3d::UnitZ());
      if (lat.norm() < 1e-6) lat = Eigen::Vector3d::UnitX();
      lat.normalize();
      g.lateral_axis = lat;
      g.vertical_axis = n.cross(lat).normalized();
      g.lateral_extent = aw * 0.5 + lmargin;
      g.vertical_extent = ah * 0.5 + vmargin;
      g.outward_extent = outward;
      g.inward_extent = inward;
    }

    std::string reason;
    if (!roi_.commit(g, reason)) {
      ROS_WARN("[task_cloud_filter] ROI from params rejected: %s",
               reason.c_str());
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
    MaskResult result;
    sensor_msgs::PointCloud2 robot_filtered;

    // Step 1: URDF self-mask (fail-closed).
    bool mask_ok = mask_->mask(
        *cloud, tf_buffer_, base_frame_, robot_filtered, result);
    int unsafe_passthrough = 0;

    if (!mask_ok) {
      // Fail-closed: publish stats only, no cloud.
      publishStats(false, result.reason, result.input_count, 0, 0, 0,
                   unsafe_passthrough, result.tf_missing_links);
      return;
    }

    // Publish robot-filtered global cloud.
    pub_robot_.publish(robot_filtered);

    // Step 2: ROI crop.
    int roi_outside = 0;
    int roi_kept = 0;
    sensor_msgs::PointCloud2 roi_cloud;
    if (roi_.ready()) {
      Eigen::Isometry3d sensor_to_base;
      try {
        geometry_msgs::TransformStamped tfm =
            tf_buffer_.lookupTransform(
                base_frame_, cloud->header.frame_id,
                cloud->header.stamp, ros::Duration(0.0));
        sensor_to_base = transformToEigen(tfm.transform);
      } catch (const tf2::TransformException&) {
        // TF for ROI crop failed -- fail-closed for task cloud.
        publishStats(false, "exact_stamp_tf_missing", result.input_count,
                     result.dropped_robot, 0, 0, unsafe_passthrough,
                     {cloud->header.frame_id});
        return;
      }
      roi_outside = cropToRoi(
          robot_filtered, roi_, sensor_to_base, base_frame_, roi_cloud);
      roi_kept = roi_cloud.width;
      pub_roi_.publish(roi_cloud);
    } else {
      roi_outside = -1;  // ROI not ready
    }

    // Step 3: Planning cloud (mode-dependent).
    if (planning_mode_ == "EXPLORE_CONTAINER" && roi_.ready()) {
      pub_planning_.publish(roi_cloud);
    } else if (planning_mode_ == "MANIPULATION_GLOBAL") {
      pub_planning_.publish(robot_filtered);
    }
    // FROZEN: don't publish planning cloud.

    publishStats(true, "ok", result.input_count, result.dropped_robot,
                 roi_kept, roi_outside, unsafe_passthrough,
                 result.tf_missing_links);
  }

  void publishStats(bool ready, const std::string& reason, int input_count,
                    int dropped_robot, int roi_kept, int roi_outside,
                    int unsafe_passthrough,
                    const std::vector<std::string>& tf_missing) {
    std::string tf_str;
    for (std::size_t i = 0; i < tf_missing.size(); ++i) {
      if (i) tf_str += ",";
      tf_str += tf_missing[i];
    }
    std_msgs::String msg;
    msg.data = buildStatsJson(
        ready, reason, mask_->ready() ? mask_->robotModelName() : "",
        mask_->ready() ? mask_->robotDescriptionHash() : "",
        mask_revision_, roi_.ready(), roi_.roiRevision(),
        roi_.geometryVersion(), roi_.geometrySource(), planning_mode_,
        planning_mode_revision_,
        ros::Time::now().toSec(), 0.0, input_count, dropped_robot, roi_kept,
        roi_outside, unsafe_passthrough, tf_str);
    pub_stats_.publish(msg);
  }

  void spin() { ros::spin(); }

 private:
  ros::NodeHandle nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<UrdfRobotSelfMask> mask_;
  TaskRoi roi_;
  std::string base_frame_{"elfin_base_link"};
  std::string planning_mode_{"EXPLORE_CONTAINER"};
  int mask_revision_{0};
  int planning_mode_revision_{0};
  ros::Subscriber sub_cloud_;
  ros::Publisher pub_robot_;
  ros::Publisher pub_roi_;
  ros::Publisher pub_planning_;
  ros::Publisher pub_stats_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "task_cloud_filter");
  TaskCloudFilterNode node;
  node.spin();
  return 0;
}
