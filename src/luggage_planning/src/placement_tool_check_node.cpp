// Read-only snapshot collision service. No scene update clients/subscriptions.
#include <chrono>
#include <openssl/sha.h>
#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include "luggage_planning/prior_tool_collision.hpp"
#include "luggage_msgs/srv/check_placement_tool_v2.hpp"

using Service = luggage_msgs::srv::CheckPlacementToolV2;
namespace {
std::string sha(const std::string& text) {
  unsigned char bytes[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(text.data()), text.size(), bytes);
  std::string out; const char* hex = "0123456789abcdef";
  for (auto b : bytes) { out += hex[b >> 4]; out += hex[b & 15]; }
  return out;
}
Eigen::Isometry3d pose(const geometry_msgs::msg::Pose& p) {
  Eigen::Quaterniond q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
  if (!q.coeffs().allFinite() || std::abs(q.squaredNorm()-1) > 1e-6)
    throw std::invalid_argument("invalid quaternion");
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = q.toRotationMatrix();
  out.translation() = Eigen::Vector3d(p.position.x,p.position.y,p.position.z);
  if (!out.matrix().allFinite()) throw std::invalid_argument("invalid position");
  return out;
}
class ToolCheckNode : public rclcpp::Node {
public:
  ToolCheckNode() : Node("placement_tool_check") {
    auto xml = declare_parameter<std::string>("robot_description", "");
    auto semantic = declare_parameter<std::string>("robot_description_semantic", "");
    auto urdf = std::make_shared<urdf::Model>();
    auto srdf = std::make_shared<srdf::Model>();
    if (!urdf->initString(xml) || !srdf->initString(*urdf,semantic))
      throw std::runtime_error("valid robot model required");
    model_ = std::make_shared<moveit::core::RobotModel>(urdf,srdf);
    hash_ = sha(xml+semantic);
    service_ = create_service<Service>("/placement_prior/check_tool",
        [this](Service::Request::SharedPtr req, Service::Response::SharedPtr res) { handle(*req,*res); });
  }
private:
  void handle(const Service::Request& req, Service::Response& res) {
    auto started = std::chrono::steady_clock::now();
    auto elapsed = [&](){return std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();};
    res.request_id=req.request_id;res.snapshot_id=req.snapshot_id;res.model_sha256=hash_;
    res.complete=false;
    try {
      if(req.request_id.empty() || req.snapshot_id.empty() || req.model_sha256!=hash_ ||
         req.frame_id!=model_->getModelFrame() || req.tcp_poses.empty() ||
         req.tcp_poses.size()!=req.box_poses.size() || req.tcp_poses.size()>32 ||
         !std::isfinite(req.budget_s) || req.budget_s<=0 || req.budget_s>10)
        throw std::invalid_argument("invalid identity, shape or budget");
      luggage_planning::PriorToolCollision checker(model_,req.scene,req.tcp_link);
      for(size_t i=0;i<req.tcp_poses.size();++i) {
        if(elapsed()>=req.budget_s) { res.reason="DEADLINE";res.valid.clear();res.service_time_s=elapsed();return; }
        auto result=checker.check(pose(req.tcp_poses[i]),pose(req.box_poses[i]),
            Eigen::Vector3d(req.box_wdh[0],req.box_wdh[1],req.box_wdh[2]),req.payload_id);
        res.valid.push_back(!result.collision);
        for(const auto& pair:result.contacts) for(const auto& c:pair.second) {
          luggage_msgs::msg::RouteContact contact;
          contact.sample_index=i;contact.body_1=c.body_name_1;contact.body_2=c.body_name_2;
          contact.body_type_1=static_cast<uint8_t>(c.body_type_1);contact.body_type_2=static_cast<uint8_t>(c.body_type_2);
          contact.depth=c.depth;contact.position.x=c.pos.x();contact.position.y=c.pos.y();contact.position.z=c.pos.z();
          contact.normal.x=c.normal.x();contact.normal.y=c.normal.y();contact.normal.z=c.normal.z();res.contacts.push_back(contact);
        }
      }
      if(elapsed()>=req.budget_s) {res.reason="DEADLINE";res.valid.clear();}
      else {res.complete=true;res.reason="CHECKED_RIGID_TOOL_ONLY";}
    } catch(const std::exception& e) {res.reason=std::string("INVALID_SNAPSHOT_OR_QUERY: ")+e.what();res.valid.clear();}
    res.service_time_s=elapsed();
  }
  moveit::core::RobotModelPtr model_;
  std::string hash_;
  rclcpp::Service<Service>::SharedPtr service_;
};
}
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<ToolCheckNode>());rclcpp::shutdown();}
