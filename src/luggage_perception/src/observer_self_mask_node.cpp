// Remove robot returns before O integrates rays. Each discarded endpoint
// discards its entire ray: it must never clear space behind the robot.
// Geometry is the actual robot_description, at the cloud acquisition TF.
#include <moveit/robot_model/robot_model.h>
#include <moveit/point_containment_filter/shape_mask.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <openssl/sha.h>
#include <cstring>
#include <map>
#include <sstream>

using Cloud = sensor_msgs::msg::PointCloud2;
using Handle = point_containment_filter::ShapeHandle;
namespace {
std::string sha(const std::string& text) {
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(text.data()), text.size(), digest);
  std::string result;
  for (auto b : digest) {
    result += "0123456789abcdef"[b >> 4];
    result += "0123456789abcdef"[b & 15];
  }
  return result;
}
Eigen::Isometry3d transform(const geometry_msgs::msg::Transform& msg) {
  Eigen::Quaterniond q(msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z);
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(msg.translation.x, msg.translation.y, msg.translation.z);
  if (!q.coeffs().allFinite() || std::abs(q.squaredNorm() - 1.) > 1e-5 ||
      !result.translation().allFinite())
    throw std::runtime_error("invalid acquisition transform");
  result.linear() = q.toRotationMatrix();
  return result;
}
struct Body { std::string link; Eigen::Isometry3d origin; };
class SelfMask : public rclcpp::Node {
public:
  SelfMask() : Node("observer_self_mask"), buffer_(get_clock()), listener_(buffer_) {
    const auto xml = declare_parameter<std::string>("robot_description", "");
    const auto semantic_xml = declare_parameter<std::string>("robot_description_semantic", "");
    const auto urdf = urdf::parseURDF(xml);
    if (!urdf) throw std::runtime_error("actual robot_description required");
    auto semantic = std::make_shared<srdf::Model>();
    if (!semantic->initString(*urdf, semantic_xml))
      throw std::runtime_error("actual semantic description required");
    model_ = std::make_shared<moveit::core::RobotModel>(urdf, semantic);
    for (const auto* link : model_->getLinkModelsWithCollisionGeometry()) {
      const auto& shapes = link->getShapes();
      const auto& origins = link->getCollisionOriginTransforms();
      for (size_t i = 0; i < shapes.size(); ++i) {
        // No hand-fitted volumes, guessed scale or unmeasured padding.
        auto handle = mask_.addShape(shapes[i], 1., 0.);
        bodies_.emplace(handle, Body{link->getName(), origins.at(i)});
      }
    }
    if (bodies_.empty()) throw std::runtime_error("robot has no collision bodies");
    model_sha_ = sha(xml + semantic_xml);
    mask_.setTransformCallback([this](Handle handle, Eigen::Isometry3d& out) {
      out = poses_.at(handle); return true;
    });
    status_ = create_publisher<std_msgs::msg::String>("/luggage/observation/self_mask_status", 10);
    for (const auto& name : {"optical", "lidar"}) {
      const auto output = std::string("/luggage/observation/") + name + "_points";
      auto publisher = create_publisher<Cloud>(output, 10);
      subscriptions_.push_back(create_subscription<Cloud>(output + "_unmasked", rclcpp::SensorDataQoS(),
          [this, publisher, name](Cloud::ConstSharedPtr input) { update(*input, publisher, name); }));
    }
  }
private:
  void update(const Cloud& input, const rclcpp::Publisher<Cloud>::SharedPtr& publisher,
              const std::string& sensor) {
    size_t removed = 0, clipped = 0;
    std::string reason = "FILTERED";
    try {
      if (input.header.frame_id.empty() ||
          (input.header.stamp.sec == 0 && input.header.stamp.nanosec == 0))
        throw std::runtime_error("nonzero acquisition stamp and physical frame required");
      if (input.is_bigendian || input.point_step == 0 ||
          input.row_step < input.width * input.point_step ||
          input.data.size() < size_t(input.row_step) * input.height)
        throw std::runtime_error("unsupported point layout");
      // ShapeMask itself can continue after a failed transform callback.
      // Resolve EVERY body first; missing exact-stamp TF drops the whole cloud.
      std::map<std::string, Eigen::Isometry3d> links;
      const rclcpp::Time stamp(input.header.stamp);
      for (const auto& [handle, body] : bodies_) {
        if (!links.count(body.link))
          links.emplace(body.link, transform(buffer_.lookupTransform(
              input.header.frame_id, body.link, stamp).transform));
        poses_[handle] = links.at(body.link) * body.origin;
      }
      std::vector<int> labels;
      mask_.maskContainment(input, Eigen::Vector3d::Zero(), 0.,
                            std::numeric_limits<double>::max(), labels);
      if (labels.size() != size_t(input.width) * input.height)
        throw std::runtime_error("mask size mismatch");
      Cloud output = input;
      output.height = 1; output.width = 0; output.data.clear();
      for (size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] == point_containment_filter::ShapeMask::INSIDE) { ++removed; continue; }
        if (labels[i] != point_containment_filter::ShapeMask::OUTSIDE) { ++clipped; continue; }
        const auto offset = (i / input.width) * input.row_step + (i % input.width) * input.point_step;
        output.data.insert(output.data.end(), input.data.begin() + offset,
                           input.data.begin() + offset + input.point_step);
        ++output.width;
      }
      output.row_step = output.width * output.point_step;
      // Empty clouds remain empty; never replace removed hits with infinity.
      publisher->publish(output);
    } catch (const std::exception& error) {
      reason = std::string("REFUSED: ") + error.what();
    }
    std::ostringstream report;
    report << "sensor=" << sensor << " stamp=" << input.header.stamp.sec << '.'
           << input.header.stamp.nanosec << " input=" << input.width * input.height
           << " robot_removed=" << removed << " clipped=" << clipped << " bodies=" << bodies_.size()
           << " model_sha256=" << model_sha_ << " reason=" << reason;
    std_msgs::msg::String status; status.data = report.str(); status_->publish(status);
  }
  moveit::core::RobotModelPtr model_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  point_containment_filter::ShapeMask mask_;
  std::map<Handle, Body> bodies_;
  std::map<Handle, Eigen::Isometry3d> poses_;
  std::string model_sha_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  std::vector<rclcpp::Subscription<Cloud>::SharedPtr> subscriptions_;
};
}
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<SelfMask>()); }
  catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("observer_self_mask"), "%s", error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
