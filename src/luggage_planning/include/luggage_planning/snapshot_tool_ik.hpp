// Exact-target IK on one caller-owned full scene. No live-scene mutation.
#pragma once
#include "luggage_msgs/srv/refine_snapshot_tool_target_v2.hpp"
#include <moveit/planning_scene/planning_scene.h>
#include <openssl/sha.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>

namespace luggage_planning {
using SnapshotIKService = luggage_msgs::srv::RefineSnapshotToolTargetV2;

class SnapshotToolIK {
 public:
  SnapshotToolIK(moveit::core::RobotModelConstPtr model, std::string hash)
      : model_(std::move(model)), model_hash_(std::move(hash)) {}

  void handle(const SnapshotIKService::Request& req, SnapshotIKService::Response& res) const {
    const auto began = std::chrono::steady_clock::now();
    auto elapsed = [&]() {return std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();};
    res.request_id=req.request_id;res.snapshot_id=req.snapshot_id;res.model_sha256=model_hash_;
    unsigned char bytes[SHA256_DIGEST_LENGTH];
    SHA256(req.scene_cdr.data(),req.scene_cdr.size(),bytes);
    for (auto byte:bytes) {
      res.scene_sha256 += "0123456789abcdef"[byte>>4];
      res.scene_sha256 += "0123456789abcdef"[byte&15];
    }
    std::string contact;
    try {
      const auto* group=model_->getJointModelGroup("elfin_arm");
      const auto* tcp=model_->getLinkModel("suction_contact_frame");
      const std::vector<std::string> joints={"elfin_joint1","elfin_joint2","elfin_joint3",
          "elfin_joint4","elfin_joint5","elfin_joint6"};
      if (!group || !tcp || !group->getSolverInstance() || model_->getModelFrame()!="world"
          || group->getVariableNames()!=joints)
        throw std::invalid_argument("world-rooted canonical six-joint IK model required");
      if (req.request_id.empty() || req.snapshot_id.empty() || req.model_sha256!=model_hash_
          || req.scene_sha256!=res.scene_sha256 || req.scene_cdr.empty()
          || req.scene_cdr.size()>64*1024*1024 || !std::isfinite(req.budget_s)
          || req.budget_s<=0 || req.budget_s>10)
        throw std::invalid_argument("identity, scene bytes or budget mismatch");
      for (double q:req.seed_joints) if (!std::isfinite(q)) throw std::invalid_argument("nonfinite seed");
      rclcpp::SerializedMessage serialized(req.scene_cdr.size());
      auto& wire=serialized.get_rcl_serialized_message();
      std::memcpy(wire.buffer,req.scene_cdr.data(),req.scene_cdr.size());wire.buffer_length=req.scene_cdr.size();
      moveit_msgs::msg::PlanningScene snapshot;
      rclcpp::Serialization<moveit_msgs::msg::PlanningScene>().deserialize_message(&serialized,&snapshot);
      if (snapshot.is_diff || snapshot.robot_state.is_diff || snapshot.robot_model_name!=model_->getName())
        throw std::invalid_argument("full model-matched scene required");
      planning_scene::PlanningScene scene(model_);
      if (!scene.setPlanningSceneMsg(snapshot)) throw std::invalid_argument("invalid full scene");
      moveit::core::RobotState state(scene.getCurrentState());
      state.setJointGroupPositions(group,req.seed_joints.data());state.update();
      if (!state.satisfiesBounds(group)) throw std::invalid_argument("seed outside joint bounds");
      const auto& p=req.world_from_tool;
      Eigen::Quaterniond orientation(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
      Eigen::Isometry3d target=Eigen::Isometry3d::Identity();
      target.translation()=Eigen::Vector3d(p.position.x,p.position.y,p.position.z);
      if (!orientation.coeffs().allFinite() || std::abs(orientation.squaredNorm()-1)>1e-6
          || !target.matrix().allFinite()) throw std::invalid_argument("invalid target pose");
      target.linear()=orientation.toRotationMatrix();
      auto valid=[&](moveit::core::RobotState& sample) {
        if (elapsed()>=req.budget_s || ++res.checked_states>1000 || !sample.satisfiesBounds(group)) return false;
        sample.update();
        collision_detection::CollisionRequest query;query.contacts=true;query.max_contacts=1;
        collision_detection::CollisionResult result;scene.checkCollision(query,result,sample);
        if (!result.contacts.empty()) contact=result.contacts.begin()->first.first+" / "+result.contacts.begin()->first.second;
        return !result.collision;
      };
      bool solved=false;
      if (elapsed()<req.budget_s) {
        auto callback=[&](moveit::core::RobotState* sample,const moveit::core::JointModelGroup* g,const double* values) {
          sample->setJointGroupPositions(g,values);return valid(*sample);
        };
        solved=state.setFromIK(group,target,tcp->getName(),std::min(.05,req.budget_s-elapsed()),callback);
      }
      state.update();
      const auto actual=state.getGlobalLinkTransform(tcp);
      solved=solved && (actual.translation()-target.translation()).norm()<1e-4
          && Eigen::Quaterniond(actual.linear()).angularDistance(orientation)<1e-3 && valid(state);
      res.complete=elapsed()<req.budget_s && res.checked_states<=1000;
      res.success=res.complete && solved;
      res.reason=res.success?"EXACT_TARGET_CHECKED":res.complete?"NO_VALID_IK":"DEADLINE_OR_CHECK_CAP";
      if (res.success) {
        std::vector<double> values;state.copyJointGroupPositions(group,values);
        std::copy(values.begin(),values.end(),res.joints.begin());
      }
    } catch (const std::exception& error) {
      res.complete=false;res.success=false;res.reason=std::string("INVALID_QUERY: ")+error.what();
    }
    res.elapsed_s=elapsed();
    if (!res.success) std::fill(res.joints.begin(),res.joints.end(),0.);
    std::ostringstream diagnostic;diagnostic<<"{\"last_contact\":"<<std::quoted(contact)<<"}";
    res.diagnostics_json=diagnostic.str();
  }
 private:
  moveit::core::RobotModelConstPtr model_;
  std::string model_hash_;
};
} // namespace luggage_planning
