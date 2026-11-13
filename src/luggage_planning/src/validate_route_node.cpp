// ST09-I: the batched route validation service (ValidateRouteV2).
//
// Certifies the GIVEN joint states of a whole route against a FROZEN
// private planning-scene copy using MoveIt's native state, collision and
// distance checking. The service never re-plans, never runs IK, never
// modifies its input and never touches the live planning scene.
//
// Per leg it verifies the frozen sampling discipline (knots included,
// temporal and spatial spacing bounds), per-sample joint limits, the
// continuous controller velocity/acceleration bounds and time monotonicity,
// then checks every sample for collision (contacts reported, designated
// support-pair contacts reported separately for certifier legality
// judgement) and measures the per-sample minimum distance with the
// exempt pairs excluded.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <openssl/sha.h>

#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/planning_scene.hpp>

#include "luggage_msgs/msg/route_contact.hpp"
#include "luggage_msgs/msg/route_leg_result_v2.hpp"
#include "luggage_msgs/msg/route_leg_v2.hpp"
#include "luggage_msgs/srv/validate_route_v2.hpp"
#include "luggage_planning/trajectory_derivative_bounds.hpp"

namespace {

using luggage_msgs::msg::RouteContact;
using luggage_msgs::msg::RouteLegResultV2;
using luggage_msgs::srv::ValidateRouteV2;

const double kTimeToleranceNs = 2.0;       // float64 round trip slack
const double kKnotMatchTolRad = 1e-9;      // knot-vs-sample position match

int64_t duration_ns(const builtin_interfaces::msg::Duration &duration) {
  return static_cast<int64_t>(duration.sec) * 1000000000LL +
         static_cast<int64_t>(duration.nanosec);
}

RouteContact contact_from(const collision_detection::Contact &contact,
                          uint32_t sample_index) {
  RouteContact message;
  message.body_1 = contact.body_name_1;
  message.body_2 = contact.body_name_2;
  message.body_type_1 = static_cast<uint8_t>(contact.body_type_1);
  message.body_type_2 = static_cast<uint8_t>(contact.body_type_2);
  message.sample_index = sample_index;
  message.depth = contact.depth;
  message.position.x = contact.pos.x();
  message.position.y = contact.pos.y();
  message.position.z = contact.pos.z();
  message.normal.x = contact.normal.x();
  message.normal.y = contact.normal.y();
  message.normal.z = contact.normal.z();
  return message;
}

bool is_support_pair(const collision_detection::Contact &contact,
                     const std::string &payload_id,
                     const std::string &support_id) {
  return (contact.body_name_1 == payload_id &&
          contact.body_name_2 == support_id) ||
         (contact.body_name_1 == support_id &&
          contact.body_name_2 == payload_id);
}

std::string sha256_hex(const std::string &material) {
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char *>(material.data()),
         material.size(), digest);
  static const char *hex = "0123456789abcdef";
  std::string out;
  out.reserve(SHA256_DIGEST_LENGTH * 2);
  for (unsigned char byte : digest) {
    out.push_back(hex[byte >> 4]);
    out.push_back(hex[byte & 0x0f]);
  }
  return out;
}

class ValidateRouteNode : public rclcpp::Node {
public:
  ValidateRouteNode() : Node("validate_route_v2") {
    const std::string urdf_xml =
        declare_parameter<std::string>("robot_description", "");
    const std::string srdf_xml =
        declare_parameter<std::string>("robot_description_semantic", "");
    if (urdf_xml.empty() || srdf_xml.empty()) {
      throw std::runtime_error(
          "validate_route_v2 requires robot_description and "
          "robot_description_semantic parameters");
    }
    auto urdf = std::make_shared<urdf::Model>();
    if (!urdf->initString(urdf_xml)) {
      throw std::runtime_error("validate_route_v2: URDF parse failed");
    }
    auto srdf = std::make_shared<srdf::Model>();
    if (!srdf->initString(*urdf, srdf_xml)) {
      throw std::runtime_error("validate_route_v2: SRDF parse failed");
    }
    model_ = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    model_sha256_ = sha256_hex(urdf_xml + srdf_xml);
    // Diagnostic switch. Production launches leave this true. An offline
    // A/B sets it false to separate distanceRobot from collision cost.
    // It is not a second certification budget.
    measure_distance_ = declare_parameter<bool>("measure_distance", true);
    service_ = create_service<ValidateRouteV2>(
        "/validate_route_v2",
        [this](const std::shared_ptr<ValidateRouteV2::Request> request,
               std::shared_ptr<ValidateRouteV2::Response> response) {
          handle(request, response);
        });
    RCLCPP_INFO(get_logger(),
                "validate_route_v2 ready (model root %s, %zu variables)",
                model_->getName().c_str(), model_->getVariableCount());
  }

private:
  void handle(const std::shared_ptr<ValidateRouteV2::Request> &request,
              std::shared_ptr<ValidateRouteV2::Response> response) {
    const auto start = std::chrono::steady_clock::now();
    response->request_id = request->request_id;
    response->route_digest = request->route_digest;
    response->scene_digest = request->scene_digest;
    response->model_sha256 = model_sha256_;
    response->result = 0;  // OK until proven otherwise
    response->all_valid = false;
    const double budget_s = request->budget_s > 0.0 ? request->budget_s
                                                    : 2.0;

    // ---- request shape validation -------------------------------
    std::string detail;
    if (!validate_request_shape(request, detail)) {
      response->result = 1;  // INVALID_REQUEST
      response->detail = detail;
      finish(response, start);
      return;
    }

    // ---- the frozen private scene copies (one per load state) ---
    std::vector<std::shared_ptr<planning_scene::PlanningScene>> scenes;
    try {
      for (const auto &scene_msg : request->load_state_scenes) {
        scenes.push_back(
            std::make_shared<planning_scene::PlanningScene>(model_));
        scenes.back()->setPlanningSceneMsg(scene_msg);
      }
    } catch (const std::exception &error) {
      response->result = 2;  // SCENE_LOAD_FAILED
      response->detail = std::string("scene load: ") + error.what();
      finish(response, start);
      return;
    }

    // ---- shared joint mapping (all legs share joint_names) ------
    const auto &names = request->legs[0].joint_names;
    const size_t n_joints = names.size();
    std::vector<int> variable_indices(n_joints, -1);
    std::vector<double> position_lower(n_joints, 0.0);
    std::vector<double> position_upper(n_joints, 0.0);
    for (size_t j = 0; j < n_joints; ++j) {
      if (!model_->hasJointModel(names[j])) {
        response->result = 1;
        response->detail = "unknown joint " + names[j];
        finish(response, start);
        return;
      }
      const moveit::core::JointModel *joint =
          model_->getJointModel(names[j]);
      variable_indices[j] = joint->getFirstVariableIndex();
      const auto &bounds = joint->getVariableBounds();
      if (!bounds.empty()) {
        position_lower[j] = bounds[0].min_position_;
        position_upper[j] = bounds[0].max_position_;
      } else {
        position_lower[j] = -M_PI;
        position_upper[j] = M_PI;
      }
    }
    const size_t n_variables = model_->getVariableCount();
    std::vector<double> scratch(n_variables, 0.0);
    const auto velocity_limits = limit_vector(request->joint_velocity_limits,
                                              n_joints);
    const auto acceleration_limits =
        limit_vector(request->joint_acceleration_limits, n_joints);

    bool budget_exceeded = false;
    for (size_t leg_index = 0; leg_index < request->legs.size();
         ++leg_index) {
      const auto &leg = request->legs[leg_index];
      if (leg.load_state_index >= scenes.size()) {
        response->result = 1;
        response->detail = "leg " + leg.edge_id +
                           " names unknown load state " +
                           std::to_string(leg.load_state_index);
        break;
      }
      RouteLegResultV2 result = validate_leg(
          request, leg_index, scenes[leg.load_state_index], names,
          variable_indices,
          position_lower, position_upper, velocity_limits,
          acceleration_limits, scratch, n_variables, budget_s, start,
          budget_exceeded);
      response->leg_results.push_back(result);
      if (budget_exceeded) {
        response->result = 3;  // BUDGET_EXCEEDED
        response->detail =
            "budget exceeded in leg " + leg.edge_id;
        break;
      }
    }
    response->all_valid =
        response->result == 0 &&
        !response->leg_results.empty() &&
        std::all_of(response->leg_results.begin(),
                    response->leg_results.end(),
                    [](const RouteLegResultV2 &leg) {
                      return leg.valid;
                    });
    finish(response, start);
  }

  static std::vector<double> limit_vector(
      const std::vector<double> &values, size_t n_joints) {
    if (values.size() == n_joints) {
      return values;
    }
    return std::vector<double>(n_joints, 0.0);  // 0 = no limit
  }

  bool validate_request_shape(
      const std::shared_ptr<ValidateRouteV2::Request> &request,
      std::string &detail) const {
    if (request->legs.empty()) {
      detail = "no legs";
      return false;
    }
    if (request->load_state_scenes.empty()) {
      detail = "no load_state_scenes";
      return false;
    }
    for (const auto &scene : request->load_state_scenes) {
      if (scene.is_diff) {
        detail = "load state scenes must be full scenes (is_diff=false)";
        return false;
      }
    }
    const auto &names = request->legs[0].joint_names;
    if (names.empty()) {
      detail = "empty joint_names";
      return false;
    }
    for (const auto &leg : request->legs) {
      if (leg.joint_names != names) {
        detail = "joint order differs across legs";
        return false;
      }
      if (leg.sample_positions_flat.size() !=
          static_cast<size_t>(leg.n_samples) * names.size()) {
        detail = "sample matrix shape mismatch on " + leg.edge_id;
        return false;
      }
      if (leg.sample_times_ns.size() != static_cast<size_t>(leg.n_samples)) {
        detail = "sample time count mismatch on " + leg.edge_id;
        return false;
      }
    }
    return true;
  }

  RouteLegResultV2 validate_leg(
      const std::shared_ptr<ValidateRouteV2::Request> &request,
      size_t leg_index,
      const std::shared_ptr<planning_scene::PlanningScene> &scene,
      const std::vector<std::string> &names,
      const std::vector<int> &variable_indices,
      const std::vector<double> &position_lower,
      const std::vector<double> &position_upper,
      const std::vector<double> &velocity_limits,
      const std::vector<double> &acceleration_limits,
      std::vector<double> &scratch, size_t /*n_variables*/,
      double budget_s, const std::chrono::steady_clock::time_point &start,
      bool &budget_exceeded) {
    const auto &leg = request->legs[leg_index];
    const auto leg_start = std::chrono::steady_clock::now();
    RouteLegResultV2 result;
    result.edge_id = leg.edge_id;
    result.n_samples = leg.n_samples;
    result.valid = false;
    int64_t collision_ns = 0;
    int64_t distance_ns = 0;
    uint32_t distance_queries = 0;
    auto finish_leg = [&]() -> RouteLegResultV2 {
      result.leg_time_ns = static_cast<double>(elapsed_ns(leg_start));
      result.collision_time_ns = static_cast<double>(collision_ns);
      result.distance_time_ns = static_cast<double>(distance_ns);
      result.distance_queries = distance_queries;
      return result;
    };
    const size_t n_joints = names.size();
    const size_t n_samples = leg.n_samples;

    // distance ACM: the declared support pair is exempt from the
    // measured distance (legal zero clearance must not masquerade as a
    // positive distance or fail the leg); the collision pass keeps it
    // checked so its contacts are still reported.
    collision_detection::AllowedCollisionMatrix distance_acm =
        scene->getAllowedCollisionMatrix();
    const bool has_support =
        !leg.support_payload_id.empty() && !leg.support_object_id.empty();
    if (has_support) {
      distance_acm.setEntry(leg.support_payload_id, leg.support_object_id,
                            true);
    }

    // ---- sampling discipline: monotone times, spacing bounds ----
    for (size_t i = 0; i + 1 < n_samples; ++i) {
      if (leg.sample_times_ns[i + 1] <= leg.sample_times_ns[i]) {
        result.reason = "TIME_NOT_MONOTONIC";
        result.detail = "sample " + std::to_string(i + 1);
        result.first_invalid_sample = static_cast<uint32_t>(i + 1);
        result.leg_time_ns = elapsed_ns(leg_start);
        return finish_leg();
      }
      const double dt = leg.sample_times_ns[i + 1] -
                        leg.sample_times_ns[i];
      if (request->max_sample_dt_ns > 0.0 &&
          dt > request->max_sample_dt_ns + kTimeToleranceNs) {
        result.reason = "SAMPLING_DISCIPLINE";
        result.detail = "dt " + std::to_string(dt) + " ns over bound";
        result.first_invalid_sample = static_cast<uint32_t>(i + 1);
        result.leg_time_ns = elapsed_ns(leg_start);
        return finish_leg();
      }
    }

    // ---- knots included, monotone, within limits ----------------
    size_t sample_cursor = 0;
    for (size_t k = 0; k < leg.knots.points.size(); ++k) {
      const auto &point = leg.knots.points[k];
      const int64_t knot_ns = duration_ns(point.time_from_start);
      if (k > 0 &&
          knot_ns <= duration_ns(
                         leg.knots.points[k - 1].time_from_start)) {
        result.reason = "TIME_NOT_MONOTONIC";
        result.detail = "knot " + std::to_string(k);
        result.leg_time_ns = elapsed_ns(leg_start);
        return finish_leg();
      }
      if (point.velocities.size() == n_joints) {
        for (size_t j = 0; j < n_joints; ++j) {
          if (velocity_limits[j] > 0.0 &&
              std::abs(point.velocities[j]) >
                  velocity_limits[j] * (1.0 + 1e-9)) {
            result.reason = "VELOCITY_LIMIT";
            result.detail = names[j] + " knot " + std::to_string(k);
            result.leg_time_ns = elapsed_ns(leg_start);
            return finish_leg();
          }
        }
      }
      if (point.accelerations.size() == n_joints) {
        for (size_t j = 0; j < n_joints; ++j) {
          if (acceleration_limits[j] > 0.0 &&
              std::abs(point.accelerations[j]) >
                  acceleration_limits[j] * (1.0 + 1e-9)) {
            result.reason = "ACCELERATION_LIMIT";
            result.detail = names[j] + " knot " + std::to_string(k);
            result.leg_time_ns = elapsed_ns(leg_start);
            return finish_leg();
          }
        }
      }
      // the knot must appear among the samples with its positions
      while (sample_cursor < n_samples &&
             leg.sample_times_ns[sample_cursor] <
                 static_cast<double>(knot_ns) - kTimeToleranceNs) {
        ++sample_cursor;
      }
      if (sample_cursor >= n_samples ||
          std::abs(leg.sample_times_ns[sample_cursor] -
                   static_cast<double>(knot_ns)) > kTimeToleranceNs) {
        result.reason = "SAMPLING_DISCIPLINE";
        result.detail = "knot " + std::to_string(k) +
                        " (t=" + std::to_string(knot_ns) +
                        " ns) not sampled";
        result.leg_time_ns = elapsed_ns(leg_start);
        return finish_leg();
      }
      for (size_t j = 0; j < n_joints; ++j) {
        const double sample_value =
            leg.sample_positions_flat[sample_cursor * n_joints + j];
        const double knot_value =
            point.positions.size() == n_joints ? point.positions[j] : 0.0;
        if (std::abs(sample_value - knot_value) > kKnotMatchTolRad) {
          result.reason = "SAMPLING_DISCIPLINE";
          result.detail = "knot " + std::to_string(k) + " joint " +
                          names[j] + " not represented by its sample";
          result.first_invalid_sample =
              static_cast<uint32_t>(sample_cursor);
          result.leg_time_ns = elapsed_ns(leg_start);
          return finish_leg();
        }
      }
    }

    // ---- continuous controller derivative bounds -----------------
    for (size_t k=1; k<leg.knots.points.size(); ++k) {
      if (elapsed_s(start)>budget_s) {
        budget_exceeded=true;result.reason="BUDGET_EXCEEDED";
        result.detail="continuous derivative checks";return finish_leg();
      }
      const auto &left=leg.knots.points[k-1];
      const auto &right=leg.knots.points[k];
      const auto shape_ok=[n_joints](const std::vector<double> &v) {
        return v.empty() || v.size()==n_joints;
      };
      if (left.positions.size()!=n_joints || right.positions.size()!=n_joints ||
          !shape_ok(left.velocities) || !shape_ok(right.velocities) ||
          !shape_ok(left.accelerations) || !shape_ok(right.accelerations)) {
        result.reason="SAMPLING_DISCIPLINE";result.detail="invalid interpolation knot arrays";
        return finish_leg();
      }
      const bool has_v=left.velocities.size()==n_joints && right.velocities.size()==n_joints;
      const bool has_a=has_v && left.accelerations.size()==n_joints && right.accelerations.size()==n_joints;
      const int order=has_a ? 5 : has_v ? 3 : 1;
      const double dt=(duration_ns(right.time_from_start)-duration_ns(left.time_from_start))*1e-9;
      for (size_t j=0; j<n_joints; ++j) {
        try {
          const auto bounds=luggage_planning::trajectory_bounds::interval(dt,left.positions[j],right.positions[j],
              has_v ? left.velocities[j] : 0.,has_v ? right.velocities[j] : 0.,
              has_a ? left.accelerations[j] : 0.,has_a ? right.accelerations[j] : 0.,order);
          if ((velocity_limits[j]>0. && bounds.first>velocity_limits[j]*(1.+1e-9)) ||
              (acceleration_limits[j]>0. && bounds.second>acceleration_limits[j]*(1.+1e-9))) {
            result.reason=velocity_limits[j]>0. && bounds.first>velocity_limits[j]*(1.+1e-9) ?
                "VELOCITY_LIMIT" : "ACCELERATION_LIMIT";
            result.detail=names[j]+" continuous interval "+std::to_string(k-1)+
                " max_v="+std::to_string(bounds.first)+" max_a="+std::to_string(bounds.second);
            return finish_leg();
          }
        } catch (const std::invalid_argument &error) {
          result.reason="SAMPLING_DISCIPLINE";result.detail=error.what();return finish_leg();
        }
      }
    }

    // ---- per-sample state checks --------------------------------
    moveit::core::RobotState &state = scene->getCurrentStateNonConst();
    state.setToDefaultValues();
    {
      const double *positions = state.getVariablePositions();
      scratch.assign(positions, positions + scratch.size());
    }
    result.min_distance_m.assign(n_samples,
                                 std::numeric_limits<double>::max());
    bool have_tcp_constraint = !leg.tcp_link.empty();
    const Eigen::Isometry3d *tcp_frame = nullptr;
    if (have_tcp_constraint) {
      if (!scene->getRobotModel()->hasLinkModel(leg.tcp_link)) {
        result.reason = "MODEL_MISMATCH";
        result.detail = "tcp link " + leg.tcp_link + " unknown";
        result.leg_time_ns = elapsed_ns(leg_start);
        return finish_leg();
      }
    }
    Eigen::Vector3d previous_tcp;
    bool have_previous_tcp = false;

    for (size_t i = 0; i < n_samples; ++i) {
      if (elapsed_s(start) > budget_s) {
        budget_exceeded = true;
        result.reason = "BUDGET_EXCEEDED";
        result.detail = "elapsed " +
                        std::to_string(elapsed_s(start)) + " s";
        result.first_invalid_sample = static_cast<uint32_t>(i);
        result.leg_time_ns = elapsed_ns(leg_start);
        return finish_leg();
      }
      const double *row = &leg.sample_positions_flat[i * n_joints];
      for (size_t j = 0; j < n_joints; ++j) {
        scratch[variable_indices[j]] = row[j];
        if (row[j] < position_lower[j] - 1e-9 ||
            row[j] > position_upper[j] + 1e-9) {
          result.reason = "JOINT_LIMIT";
          result.detail = names[j] + " sample " + std::to_string(i) +
                          " value " + std::to_string(row[j]);
          result.first_invalid_sample = static_cast<uint32_t>(i);
          result.leg_time_ns = elapsed_ns(leg_start);
          return finish_leg();
        }
      }
      try {
        state.setVariablePositions(scratch.data());
      } catch (const std::exception &error) {
        result.reason = "MODEL_MISMATCH";
        result.detail = std::string("setVariablePositions: ") +
                        error.what();
        result.first_invalid_sample = static_cast<uint32_t>(i);
        result.leg_time_ns = elapsed_ns(leg_start);
        return finish_leg();
      }
      state.update();

      // spatial sampling discipline via TCP position
      if (!leg.spatial_check_link.empty()) {
        const Eigen::Isometry3d &transform =
            state.getGlobalLinkTransform(leg.spatial_check_link);
        if (have_previous_tcp &&
            request->max_sample_step_m > 0.0 &&
            (transform.translation() - previous_tcp).norm() >
                request->max_sample_step_m * (1.0 + 1e-6)) {
          result.reason = "SAMPLING_DISCIPLINE";
          result.detail = "spatial step over bound at sample " +
                          std::to_string(i);
          result.first_invalid_sample = static_cast<uint32_t>(i);
          result.leg_time_ns = elapsed_ns(leg_start);
          return finish_leg();
        }
        previous_tcp = transform.translation();
        have_previous_tcp = true;
      }

      // optional TCP pose constraint
      if (have_tcp_constraint) {
        tcp_frame = &state.getGlobalLinkTransform(leg.tcp_link);
        const double position_error =
            (tcp_frame->translation() -
             Eigen::Vector3d(leg.tcp_pose.position.x,
                             leg.tcp_pose.position.y,
                             leg.tcp_pose.position.z))
                .norm();
        if (leg.tcp_position_tol_m > 0.0 &&
            position_error > leg.tcp_position_tol_m) {
          result.reason = "TCP_CONSTRAINT";
          result.detail = "position error " +
                          std::to_string(position_error) + " at sample " +
                          std::to_string(i);
          result.first_invalid_sample = static_cast<uint32_t>(i);
          result.leg_time_ns = elapsed_ns(leg_start);
          return finish_leg();
        }
        if (leg.tcp_orientation_tol_rad > 0.0) {
          Eigen::Quaterniond actual(tcp_frame->linear());
          Eigen::Quaterniond expected(
              leg.tcp_pose.orientation.w, leg.tcp_pose.orientation.x,
              leg.tcp_pose.orientation.y, leg.tcp_pose.orientation.z);
          expected.normalize();
          const double dot = std::min(
              1.0, std::abs(actual.coeffs().dot(expected.coeffs())));
          const double angle = 2.0 * std::acos(dot);
          if (angle > leg.tcp_orientation_tol_rad) {
            result.reason = "TCP_CONSTRAINT";
            result.detail = "orientation error " +
                            std::to_string(angle) + " at sample " +
                            std::to_string(i);
            result.first_invalid_sample = static_cast<uint32_t>(i);
            result.leg_time_ns = elapsed_ns(leg_start);
            return finish_leg();
          }
        }
      }

      // collision with contacts available on demand
      collision_detection::CollisionRequest collision_request;
      collision_request.contacts = false;
      collision_request.verbose = false;
      collision_detection::CollisionResult collision_result;
      const auto collision_started = std::chrono::steady_clock::now();
      scene->checkCollision(collision_request, collision_result, state);
      collision_ns += elapsed_ns(collision_started);
      if (collision_result.collision) {
        collision_detection::CollisionRequest contact_request;
        contact_request.contacts = true;
        contact_request.max_contacts = 32;
        contact_request.max_contacts_per_pair = 8;
        contact_request.verbose = false;
        collision_detection::CollisionResult contact_result;
        const auto contact_started = std::chrono::steady_clock::now();
        scene->checkCollision(contact_request, contact_result, state);
        collision_ns += elapsed_ns(contact_started);
        bool forbidden = false;
        for (const auto &item : contact_result.contacts) {
          for (const auto &contact : item.second) {
            if (has_support &&
                is_support_pair(contact, leg.support_payload_id,
                                leg.support_object_id)) {
              result.support_contacts.push_back(
                  contact_from(contact, static_cast<uint32_t>(i)));
            } else {
              forbidden = true;
              if (result.contacts.size() < 32) {
                result.contacts.push_back(
                    contact_from(contact, static_cast<uint32_t>(i)));
              }
            }
          }
        }
        if (forbidden) {
          result.reason = "COLLISION";
          std::string bodies;
          for (const auto &contact : result.contacts) {
            bodies += contact.body_1 + "/" + contact.body_2 + " ";
          }
          result.detail = bodies + "at sample " + std::to_string(i);
          result.first_invalid_sample = static_cast<uint32_t>(i);
          result.leg_time_ns = elapsed_ns(leg_start);
          return finish_leg();
        }
      }

      // measured minimum distance (exempt pairs excluded), at the
      // requested cadence; unmeasured entries stay NaN
      const uint32_t stride = request->distance_stride > 0
                                  ? request->distance_stride
                                  : 1u;
      result.min_distance_m[i] = std::numeric_limits<double>::quiet_NaN();
      if (measure_distance_ && i % stride == 0) {
        collision_detection::DistanceRequest distance_request;
        distance_request.enable_nearest_points = false;
        distance_request.type =
            collision_detection::DistanceRequestTypes::GLOBAL;
        distance_request.acm = &distance_acm;
        distance_request.verbose = false;
        collision_detection::DistanceResult distance_result;
        const auto distance_started = std::chrono::steady_clock::now();
        try {
          scene->getCollisionEnv()->distanceRobot(distance_request,
                                                  distance_result, state);
          distance_ns += elapsed_ns(distance_started);
          distance_queries += 1;
          result.min_distance_m[i] =
              distance_result.minimum_distance.distance;
        } catch (const std::exception &error) {
          distance_ns += elapsed_ns(distance_started);
          distance_queries += 1;
          result.reason = "MODEL_MISMATCH";
          result.detail = std::string("distance: ") + error.what();
          result.first_invalid_sample = static_cast<uint32_t>(i);
          result.leg_time_ns = elapsed_ns(leg_start);
          return finish_leg();
        }
      }
    }

    result.valid = true;
    result.first_invalid_sample = static_cast<uint32_t>(n_samples);
    result.leg_time_ns = elapsed_ns(leg_start);
    return finish_leg();
  }

  static double elapsed_s(
      const std::chrono::steady_clock::time_point &start) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         start)
        .count();
  }

  static int64_t elapsed_ns(
      const std::chrono::steady_clock::time_point &start) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now() - start)
        .count();
  }

  void finish(std::shared_ptr<ValidateRouteV2::Response> response,
              const std::chrono::steady_clock::time_point &start) {
    response->service_time_ns = elapsed_ns(start);
  }

  moveit::core::RobotModelPtr model_;
  std::string model_sha256_;
  bool measure_distance_{true};
  rclcpp::Service<ValidateRouteV2>::SharedPtr service_;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ValidateRouteNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
