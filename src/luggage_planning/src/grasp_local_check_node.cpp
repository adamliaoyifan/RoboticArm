// Bounded multi-seed local reachability. Owns private scenes, never publishes
// or applies them. Output is untimed geometric evidence, not execution
// authority.
#include "luggage_msgs/srv/check_grasp_local_paths.hpp"
#include <chrono>
#include <geometric_shapes/shapes.h>
#include <iomanip>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <openssl/sha.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

using Service = luggage_msgs::srv::CheckGraspLocalPaths;
using State = moveit::core::RobotState;
using Clock = std::chrono::steady_clock;
namespace {
Eigen::Isometry3d pose(const geometry_msgs::msg::Pose &p) {
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y,
                       p.orientation.z);
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  if (!q.coeffs().allFinite() || std::abs(q.squaredNorm() - 1) > 1e-6 ||
      !t.matrix().allFinite())
    throw std::invalid_argument("invalid pose");
  t.linear() = q.toRotationMatrix();
  return t;
}
std::string sha(const std::string &s) {
  unsigned char bytes[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char *>(s.data()), s.size(), bytes);
  std::string out;
  for (auto b : bytes) {
    out += "0123456789abcdef"[b >> 4];
    out += "0123456789abcdef"[b & 15];
  }
  return out;
}
class Checker {
public:
  Checker(moveit::core::RobotModelConstPtr model, const Service::Request &req)
      : model_(model), scene_(model), req_(req), started_(Clock::now()) {
    if (model_->getModelFrame() != "world")
      throw std::invalid_argument("world model frame required");
    group_ = model_->getJointModelGroup("elfin_arm");
    tcp_ = model_->getLinkModel("suction_contact_frame");
    if (!group_ || !tcp_ || !group_->getSolverInstance())
      throw std::runtime_error("IK_SOLVER_UNAVAILABLE");
    if (req.scene.is_diff || !scene_.setPlanningSceneMsg(req.scene))
      throw std::invalid_argument("full scene required");
    if (scene_.getWorld()->hasObject(payload_) ||
        !req.scene.robot_state.attached_collision_objects.empty())
      throw std::invalid_argument(
          "payload identity conflict or existing attachment");
    for (double d : req.box_wdh)
      if (!std::isfinite(d) || d <= 0)
        throw std::invalid_argument("invalid dimensions");
    g_ = pose(req.box_from_tcp);
    shape_ = std::make_shared<shapes::Box>(req.box_wdh[0], req.box_wdh[1],
                                           req.box_wdh[2]);
    // Same cup-only contact permission as production pickup. No sensor, arm,
    // floor, cargo or container contacts are exempted.
    scene_.getAllowedCollisionMatrixNonConst().setEntry(payload_,
                                                        "suction_panel", true);
  }
  bool expired() const { return elapsed() >= req_.budget_s; }
  double elapsed() const {
    return std::chrono::duration<double>(Clock::now() - started_).count();
  }
  void run(Service::Response &res) {
    search(pose(req_.pick_box_world), pose(req_.pick_approach_world),
           pose(req_.pick_lift_world), false, res.pick_branches);
    if (!expired())
      search(pose(req_.place_box_world), pose(req_.place_entry_world),
             pose(req_.place_retreat_world), true, res.place_branches);
    res.complete = !expired();
    res.reason = expired()                    ? "DEADLINE"
                 : res.pick_branches.empty()  ? "NO_PICK_LOCAL_BRANCH"
                 : res.place_branches.empty() ? "NO_PLACE_LOCAL_BRANCH"
                                              : "LOCAL_PATHS_VALID";
    if (expired()) {
      res.pick_branches.clear();
      res.place_branches.clear();
    }
    std::ostringstream out;
    out << "{\"ik_attempts\":" << ik_ << ",\"collision_checks\":" << checks_
        << ",\"collision_rejections\":" << collisions_
        << ",\"joint_jump_rejections\":" << jumps_
        << ",\"seed_limit\":8,\"branches_per_end\":2,\"translation_step_m\":0."
           "005,\"joint_step_rad\":0.01}";
    auto diagnostic = out.str();
    diagnostic.pop_back();
    std::ostringstream detail;
    detail << diagnostic << ",\"last_contact\":" << std::quoted(last_contact_)
           << "}";
    res.diagnostics_json = detail.str();
  }

private:
  void payload(State &state, const Eigen::Isometry3d &box, bool carrying) {
    state.clearAttachedBodies();
    scene_.getWorldNonConst()->removeObject(payload_);
    if (carrying) {
      std::vector<shapes::ShapeConstPtr> shapes{shape_};
      EigenSTL::vector_Isometry3d origins{Eigen::Isometry3d::Identity()};
      state.attachBody(payload_, g_.inverse(), shapes, origins,
                       std::set<std::string>{"suction_panel"}, tcp_->getName());
    } else
      scene_.getWorldNonConst()->addToObject(payload_, shape_, box);
    state.update();
  }
  bool valid(State &state) {
    if (expired() || !state.satisfiesBounds(group_))
      return false;
    state.update();
    ++checks_;
    collision_detection::CollisionRequest q;
    q.contacts = true;
    q.max_contacts = 1;
    collision_detection::CollisionResult r;
    scene_.checkCollision(q, r, state);
    if (r.collision) {
      ++collisions_;
      if (!r.contacts.empty())
        last_contact_ = r.contacts.begin()->first.first + " / " +
                        r.contacts.begin()->first.second;
    }
    return !r.collision;
  }
  bool solve(State &state, const Eigen::Isometry3d &target) {
    if (expired())
      return false;
    ++ik_;
    auto cb = [this](State *s, const moveit::core::JointModelGroup *g,
                     const double *values) {
      s->setJointGroupPositions(g, values);
      return valid(*s);
    };
    if (!state.setFromIK(group_, target, tcp_->getName(),
                         std::min(.05, req_.budget_s - elapsed()), cb))
      return false;
    state.update();
    auto actual = state.getGlobalLinkTransform(tcp_);
    return !expired() &&
           (actual.translation() - target.translation()).norm() < 1e-4 &&
           Eigen::Quaterniond(actual.linear())
                   .angularDistance(Eigen::Quaterniond(target.linear())) <
               1e-3 &&
           valid(state);
  }
  void append(trajectory_msgs::msg::JointTrajectory &path, const State &state) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    state.copyJointGroupPositions(group_, point.positions);
    path.points.push_back(point);
  }
  bool path(State &state, const Eigen::Isometry3d &target,
            trajectory_msgs::msg::JointTrajectory &result) {
    result.joint_names = group_->getVariableNames();
    append(result, state);
    const auto start = state.getGlobalLinkTransform(tcp_);
    Eigen::Quaterniond qa(start.linear()), qb(target.linear());
    const int count = std::max(
        1, static_cast<int>(std::ceil(std::max(
               (start.translation() - target.translation()).norm() / .005,
               qa.angularDistance(qb) / .03))));
    if (count > 400)
      return false;
    for (int i = 1; i <= count; ++i) {
      double f = double(i) / count;
      Eigen::Isometry3d goal = Eigen::Isometry3d::Identity();
      goal.translation() =
          (1 - f) * start.translation() + f * target.translation();
      goal.linear() = qa.slerp(f, qb).toRotationMatrix();
      State previous(state);
      if (!solve(state, goal))
        return false;
      std::vector<double> a, b;
      previous.copyJointGroupPositions(group_, a);
      state.copyJointGroupPositions(group_, b);
      double delta = 0;
      for (size_t j = 0; j < a.size(); ++j)
        delta = std::max(delta, std::abs(a[j] - b[j]));
      if (delta > .15) {
        ++jumps_;
        return false;
      }
      int steps = std::max(1, static_cast<int>(std::ceil(delta / .01)));
      for (int k = 1; k <= steps; ++k) {
        State sample(previous);
        previous.interpolate(state, double(k) / steps, sample);
        if (!valid(sample))
          return false;
        append(result, sample);
      }
    }
    return !expired();
  }
  void search(const Eigen::Isometry3d &box, const Eigen::Isometry3d &entry,
              const Eigen::Isometry3d &exit, bool incoming_carrying,
              std::vector<luggage_msgs::msg::GraspLocalBranch> &branches) {
    const auto contact = box * g_;
    for (unsigned seed = 0; seed < 8 && branches.size() < 2 && !expired();
         ++seed) {
      State state(scene_.getCurrentState());
      payload(state, box, incoming_carrying);
      if (seed > 0) {
        std::vector<double> q;
        state.copyJointGroupPositions(group_, q);
        for (size_t j = 0; j < q.size(); ++j) {
          const auto &b =
              model_->getVariableBounds(group_->getVariableNames()[j]);
          const double low = b.position_bounded_ ? b.min_position_ : -M_PI,
                       high = b.position_bounded_ ? b.max_position_ : M_PI;
          q[j] = low + (high - low) * std::fmod(.5 + seed * .61803398875 +
                                                    j * .41421356237,
                                                1.0);
        }
        state.setJointGroupPositions(group_, q);
        state.update();
      }
      if (!solve(state, entry))
        continue;
      luggage_msgs::msg::GraspLocalBranch branch;
      branch.seed_index = seed;
      if (!path(state, contact, branch.incoming))
        continue;
      payload(state, box, !incoming_carrying);
      if (!valid(state) || !path(state, exit, branch.outgoing))
        continue;
      bool duplicate = false;
      for (const auto &b : branches) {
        double d = 0;
        const auto &x = b.incoming.points.back().positions;
        const auto &y = branch.incoming.points.back().positions;
        for (size_t j = 0; j < x.size(); ++j)
          d = std::max(d, std::abs(x[j] - y[j]));
        if (d < .05)
          duplicate = true;
      }
      if (!duplicate)
        branches.push_back(branch);
    }
  }
  moveit::core::RobotModelConstPtr model_;
  planning_scene::PlanningScene scene_;
  const Service::Request &req_;
  Clock::time_point started_;
  const moveit::core::JointModelGroup *group_;
  const moveit::core::LinkModel *tcp_;
  Eigen::Isometry3d g_;
  shapes::ShapeConstPtr shape_;
  const std::string payload_ = "prior-local-box";
  std::string last_contact_;
  unsigned ik_ = 0, checks_ = 0, collisions_ = 0, jumps_ = 0;
};
} // namespace
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "grasp_local_check",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));
  robot_model_loader::RobotModelLoader loader(node, "robot_description", true);
  auto model = loader.getModel();
  if (!model)
    throw std::runtime_error("model unavailable");
  std::string xml, semantic;
  node->get_parameter("robot_description", xml);
  node->get_parameter("robot_description_semantic", semantic);
  auto hash = sha(xml + semantic);
  auto service = node->create_service<Service>(
      "/placement_prior/check_local_paths",
      [&](Service::Request::SharedPtr req, Service::Response::SharedPtr res) {
        const auto start = Clock::now();
        res->request_id = req->request_id;
        res->snapshot_id = req->snapshot_id;
        res->model_sha256 = hash;
        try {
          if (req->request_id.empty() || req->snapshot_id.empty() ||
              req->model_sha256 != hash || !std::isfinite(req->budget_s) ||
              req->budget_s <= 0 || req->budget_s > 10)
            throw std::invalid_argument("invalid identity or budget");
          Checker(model, *req).run(*res);
        } catch (const std::exception &e) {
          res->complete = false;
          res->reason = std::string("INVALID_QUERY: ") + e.what();
          res->pick_branches.clear();
          res->place_branches.clear();
        }
        res->elapsed_s =
            std::chrono::duration<double>(Clock::now() - start).count();
      });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
