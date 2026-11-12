// Joint connections for provisional whole-operation planning. Every request owns
// a private full scene. No ApplyPlanningScene, controller or vacuum interfaces.
#include "luggage_msgs/srv/plan_snapshot_joint_path_v2.hpp"
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <openssl/sha.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>

namespace {
using Service = luggage_msgs::srv::PlanSnapshotJointPathV2;
using Clock = std::chrono::steady_clock;
using State = moveit::core::RobotState;
const std::vector<std::string> JOINTS = {
    "elfin_joint1", "elfin_joint2", "elfin_joint3",
    "elfin_joint4", "elfin_joint5", "elfin_joint6"};

std::string sha(const unsigned char *data, size_t size) {
  unsigned char bytes[SHA256_DIGEST_LENGTH];
  SHA256(data, size, bytes);
  std::string out;
  for (auto byte : bytes) {
    out += "0123456789abcdef"[byte >> 4];
    out += "0123456789abcdef"[byte & 15];
  }
  return out;
}
std::string sha(const std::string &text) {
  return sha(reinterpret_cast<const unsigned char *>(text.data()), text.size());
}
double max_delta(const std::vector<double> &a, const std::vector<double> &b) {
  double delta = 0;
  for (size_t j = 0; j < a.size(); ++j)
    delta = std::max(delta, std::abs(a[j] - b[j]));
  return delta;
}

class Connector {
public:
  explicit Connector(const rclcpp::Node::SharedPtr &node)
      : node_(node), loader_(node, "robot_description", false),
        model_(loader_.getModel()) {
    if (!model_ || model_->getModelFrame() != "world")
      throw std::invalid_argument("world-frame robot model required");
    group_ = model_->getJointModelGroup("elfin_arm");
    if (!group_ || group_->getVariableNames() != JOINTS)
      throw std::invalid_argument("canonical six-joint elfin_arm required");
    std::string xml, semantic;
    node_->get_parameter("robot_description", xml);
    node_->get_parameter("robot_description_semantic", semantic);
    model_hash_ = sha(xml + semantic);
    // Fixed planner, no start-state repair adapters. A repaired endpoint would
    // sever the endpoint-branch binding of the complete operation plan.
    pipeline_ = std::make_shared<planning_pipeline::PlanningPipeline>(
        model_, node_, "ompl", "ompl_interface/OMPLPlanner", std::vector<std::string>{});
    pipeline_->displayComputedMotionPlans(false);
    pipeline_->publishReceivedRequests(false);
    pipeline_->checkSolutionPaths(true);
    if (!node_->has_parameter("joint_check_step_rad"))
      node_->declare_parameter<double>("joint_check_step_rad", .01);
    if (!node_->has_parameter("max_checked_states"))
      node_->declare_parameter<int>("max_checked_states", 50000);
    step_ = node_->get_parameter("joint_check_step_rad").as_double();
    point_limit_ = node_->get_parameter("max_checked_states").as_int();
    if (!std::isfinite(step_) || step_ <= 0 || step_ > .01 ||
        point_limit_ < 2 || point_limit_ > 50000)
      throw std::invalid_argument("invalid collision-check resolution or state cap");
  }

  void handle(const Service::Request &req, Service::Response &res) {
    const auto started = Clock::now();
    auto elapsed = [&]() {
      return std::chrono::duration<double>(Clock::now() - started).count();
    };
    res.request_id = req.request_id;
    res.snapshot_id = req.snapshot_id;
    res.model_sha256 = model_hash_;
    res.scene_sha256 = sha(req.scene_cdr.data(), req.scene_cdr.size());
    std::string last_contact;
    int planner_code = 0;
    try {
      if (req.request_id.empty() || req.snapshot_id.empty() ||
          req.model_sha256 != model_hash_ || req.scene_sha256 != res.scene_sha256 ||
          !std::isfinite(req.budget_s) || req.budget_s <= 0 || req.budget_s > 10 ||
          req.scene_cdr.empty())
        throw std::invalid_argument("identity, scene bytes or budget mismatch");
      rclcpp::SerializedMessage serialized(req.scene_cdr.size());
      auto &wire = serialized.get_rcl_serialized_message();
      std::memcpy(wire.buffer, req.scene_cdr.data(), req.scene_cdr.size());
      wire.buffer_length = req.scene_cdr.size();
      moveit_msgs::msg::PlanningScene snapshot;
      rclcpp::Serialization<moveit_msgs::msg::PlanningScene> serializer;
      serializer.deserialize_message(&serialized, &snapshot);
      if (snapshot.is_diff || snapshot.robot_state.is_diff ||
          snapshot.robot_model_name != model_->getName())
        throw std::invalid_argument("full model-matched scene required");
      auto scene = std::make_shared<planning_scene::PlanningScene>(model_);
      if (!scene->setPlanningSceneMsg(snapshot))
        throw std::invalid_argument("invalid full planning scene");
      kinematic_constraints::KinematicConstraintSet constraints(model_);
      if (!constraints.add(req.path_constraints, scene->getTransforms()))
        throw std::invalid_argument("invalid path constraints");
      const std::vector<double> start(req.start_joints.begin(), req.start_joints.end());
      const std::vector<double> goal(req.goal_joints.begin(), req.goal_joints.end());
      for (const auto &values : {start, goal})
        for (double value : values)
          if (!std::isfinite(value))
            throw std::invalid_argument("nonfinite joints");
      auto valid = [&](const std::vector<double> &q) {
        if (elapsed() >= req.budget_s)
          throw std::runtime_error("DEADLINE");
        if (res.checked_states >= static_cast<unsigned>(point_limit_))
          throw std::runtime_error("STATE_CHECK_CAP");
        ++res.checked_states;
        State state(scene->getCurrentState());
        state.setJointGroupPositions(group_, q);
        state.update();
        if (!state.satisfiesBounds(group_))
          throw std::runtime_error("JOINT_BOUNDS");
        if (!constraints.decide(state).satisfied)
          throw std::runtime_error("PATH_CONSTRAINT");
        collision_detection::CollisionRequest query;
        query.contacts = true;
        query.max_contacts = 1;
        collision_detection::CollisionResult result;
        scene->checkCollision(query, result, state);
        if (result.collision) {
          if (!result.contacts.empty())
            last_contact = result.contacts.begin()->first.first + " / " +
                           result.contacts.begin()->first.second;
          throw std::runtime_error("STATE_COLLISION");
        }
      };
      valid(start);
      valid(goal);
      State start_state(scene->getCurrentState()), goal_state(start_state);
      start_state.setJointGroupPositions(group_, start);
      goal_state.setJointGroupPositions(group_, goal);
      start_state.update();
      goal_state.update();
      scene->setCurrentState(start_state);
      planning_interface::MotionPlanRequest query;
      query.group_name = group_->getName();
      query.path_constraints = req.path_constraints;
      query.planner_id = "RRTConnectkConfigDefault";
      query.num_planning_attempts = 1;
      query.allowed_planning_time = req.budget_s - elapsed();
      if (query.allowed_planning_time <= 0)
        throw std::runtime_error("DEADLINE");
      moveit::core::robotStateToRobotStateMsg(start_state, query.start_state);
      query.goal_constraints.push_back(
          kinematic_constraints::constructGoalConstraints(goal_state, group_, 1e-9));
      planning_interface::MotionPlanResponse result;
      const bool planned = pipeline_->generatePlan(scene, query, result);
      planner_code = result.error_code_.val;
      if (elapsed() >= req.budget_s)
        throw std::runtime_error("DEADLINE");
      if (!planned || result.error_code_.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
          !result.trajectory_ || result.trajectory_->getWayPointCount() == 0)
        throw std::runtime_error("NO_CONNECTION");
      std::vector<std::vector<double>> vertices;
      for (size_t i = 0; i < result.trajectory_->getWayPointCount(); ++i) {
        std::vector<double> q;
        result.trajectory_->getWayPoint(i).copyJointGroupPositions(group_, q);
        vertices.push_back(q);
      }
      if (max_delta(vertices.front(), start) > 1e-6 ||
          max_delta(vertices.back(), goal) > 1e-6)
        throw std::runtime_error("PLANNED_ENDPOINT_MISMATCH");
      // Exact requested endpoints are reinstated only within numerical solver
      // tolerance, then every resulting segment is checked again below.
      vertices.front() = start;
      vertices.back() = goal;
      if (vertices.size() == 1)
        vertices.push_back(goal);
      res.geometric_path.joint_names = JOINTS;
      auto append = [&](const std::vector<double> &q) {
        valid(q);
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = q;
        res.geometric_path.points.push_back(point);
      };
      append(start);
      for (size_t i = 1; i < vertices.size(); ++i) {
        const auto &a = vertices[i - 1];
        const auto &b = vertices[i];
        const int count = std::max(1, static_cast<int>(std::ceil(max_delta(a, b) / step_)));
        for (int k = 1; k <= count; ++k) {
          std::vector<double> q(JOINTS.size());
          for (size_t j = 0; j < q.size(); ++j)
            q[j] = k == count ? b[j] : a[j] + (b[j] - a[j]) * double(k) / count;
          append(q);
        }
      }
      res.complete = res.success = true;
      res.reason = "CHECKED_GEOMETRIC_CONNECTION";
    } catch (const std::invalid_argument &error) {
      res.reason = std::string("INVALID_QUERY: ") + error.what();
    } catch (const std::exception &error) {
      res.reason = error.what();
      // A bounded search failure is not a proof of physical unreachability.
      res.complete = res.reason == "STATE_COLLISION" || res.reason == "JOINT_BOUNDS";
    }
    if (!res.success)
      res.geometric_path = trajectory_msgs::msg::JointTrajectory();
    std::ostringstream detail;
    detail << "{\"joint_check_step_rad\":" << step_
           << ",\"max_checked_states\":" << point_limit_
           << ",\"planner_code\":" << planner_code
           << ",\"last_contact\":" << std::quoted(last_contact) << "}";
    res.diagnostics_json = detail.str();
    res.elapsed_s = elapsed();
  }

private:
  rclcpp::Node::SharedPtr node_;
  robot_model_loader::RobotModelLoader loader_;
  moveit::core::RobotModelPtr model_;
  const moveit::core::JointModelGroup *group_;
  planning_pipeline::PlanningPipelinePtr pipeline_;
  std::string model_hash_;
  double step_;
  int point_limit_;
};
} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "snapshot_joint_path", rclcpp::NodeOptions()
          .automatically_declare_parameters_from_overrides(true));
  Connector connector(node);
  auto service = node->create_service<Service>(
      "/joint_operation/plan_connection",
      [&](Service::Request::SharedPtr req, Service::Response::SharedPtr res) {
        connector.handle(*req, *res);
      });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
