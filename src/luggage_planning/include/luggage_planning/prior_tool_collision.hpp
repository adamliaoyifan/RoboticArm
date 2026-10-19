#pragma once
// Rigid end-effector/world screening only. This is NOT an arm route validator.
// A private PlanningScene owns the snapshot. No live scene/ACM is accessible.
#include <cmath>
#include <memory>
#include <set>
#include <stdexcept>
#include <geometric_shapes/shapes.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>

namespace luggage_planning {
class PriorToolCollision {
public:
  PriorToolCollision(const moveit::core::RobotModelConstPtr& model,
                     const moveit_msgs::msg::PlanningScene& snapshot,
                     const std::string& tcp_name)
      : model_(model), scene_(model) {
    if (snapshot.is_diff) throw std::invalid_argument("full scene required");
    tcp_ = model_->getLinkModel(tcp_name);
    if (!tcp_) throw std::invalid_argument("unknown TCP");
    root_ = model_->getRigidlyConnectedParentLinkModel(tcp_);
    if (!scene_.setPlanningSceneMsg(snapshot))
      throw std::invalid_argument("invalid frozen scene");
    moveit::core::RobotState reference(model_);
    reference.setToDefaultValues(); reference.update();
    tcp_from_root_ = reference.getGlobalLinkTransform(tcp_).inverse() *
                     reference.getGlobalLinkTransform(root_);
    for (const auto* link : model_->getLinkModelsWithCollisionGeometry()) {
      if (model_->getRigidlyConnectedParentLinkModel(link) == root_)
        rigid_links_.insert(link->getName());
    }
    if (rigid_links_.empty()) throw std::invalid_argument("empty rigid tool geometry");
  }

  collision_detection::CollisionResult check(
      const Eigen::Isometry3d& world_tcp, const Eigen::Isometry3d& world_box,
      const Eigen::Vector3d& size, const std::string& payload_id) const {
    if (!world_tcp.matrix().allFinite() || !world_box.matrix().allFinite() ||
        !size.allFinite() || size.minCoeff() <= 0 || payload_id.empty())
      throw std::invalid_argument("invalid candidate geometry");
    if (scene_.getWorld()->hasObject(payload_id))
      throw std::invalid_argument("payload also present in snapshot world");
    moveit::core::RobotState state(scene_.getCurrentState());
    // Replace only this private candidate state's attachments. Original scene
    // remains immutable. This query represents the explicitly supplied payload.
    state.clearAttachedBodies();
    state.update();
    const Eigen::Isometry3d world_root = world_tcp * tcp_from_root_;
    std::vector<shapes::ShapeConstPtr> shapes{std::make_shared<shapes::Box>(size.x(), size.y(), size.z())};
    EigenSTL::vector_Isometry3d origins{Eigen::Isometry3d::Identity()};
    state.attachBody(payload_id, world_root.inverse()*world_box, shapes, origins,
                     std::set<std::string>{}, root_->getName());
    state.update();
    // Explicit MoveIt API for positioning a rigid subassembly without IK.
    // Refresh collision-body caches explicitly before the const collision API.
    state.updateStateWithLinkAt(root_, world_root);
    state.updateCollisionBodyTransforms();
    const auto& actual_tcp = state.getGlobalLinkTransform(tcp_);
    if (!actual_tcp.matrix().isApprox(world_tcp.matrix(), 1e-9))
      throw std::runtime_error("rigid TCP transform mismatch");
    auto acm = scene_.getAllowedCollisionMatrix();
    const auto world_ids = scene_.getWorld()->getObjectIds();
    // Scope this query to the rigid tool, not the unplaced arm. Do not exempt
    // any rigid-tool/world or payload/world pair. This local filter is never
    // used by the final whole-arm route certifier.
    for (const auto& link : model_->getLinkModelNamesWithCollisionGeometry())
      if (!rigid_links_.count(link)) acm.setEntry(link, world_ids, true);
    collision_detection::CollisionRequest request;
    request.contacts = true; request.max_contacts = 100;
    request.max_contacts_per_pair = 4;
    collision_detection::CollisionResult result;
    scene_.getCollisionEnv()->checkRobotCollision(request, result, state, acm);
    return result;
  }
  const std::set<std::string>& rigidLinks() const { return rigid_links_; }
private:
  moveit::core::RobotModelConstPtr model_;
  planning_scene::PlanningScene scene_;
  const moveit::core::LinkModel* tcp_;
  const moveit::core::LinkModel* root_;
  Eigen::Isometry3d tcp_from_root_;
  std::set<std::string> rigid_links_;
};
}  // namespace luggage_planning
