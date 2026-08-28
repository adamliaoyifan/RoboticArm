#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/utils/moveit_error_code.h>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace
{
double max_abs_error(const std::vector<double> & a, const std::vector<double> & b)
{
  double max_err = 0.0;
  for (size_t i = 0; i < a.size() && i < b.size(); ++i) {
    max_err = std::max(max_err, std::abs(a[i] - b[i]));
  }
  return max_err;
}

std::vector<double> joints_of(
  moveit::planning_interface::MoveGroupInterface & move_group)
{
  auto state = move_group.getCurrentState(2.0);
  if (!state) {
    throw std::runtime_error("failed to read current robot state");
  }
  std::vector<double> values;
  state->copyJointGroupPositions(move_group.getName(), values);
  return values;
}

double wait_near(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const std::vector<double> & target, double max_error)
{
  double err = 1e9;
  for (int k = 0; k < 50; ++k) {
    err = max_abs_error(joints_of(move_group), target);
    if (err <= max_error) {
      return err;
    }
    std::this_thread::sleep_for(50ms);
  }
  return err;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("move_to_joint_goal");

  const int repeat = node->declare_parameter<int>("repeat", 1);
  const double max_error = node->declare_parameter<double>("max_error", 0.01);
  const auto start_joints = node->declare_parameter<std::vector<double>>(
    "start_joints", std::vector<double>{0.0, -0.5, 0.0, 0.0, 0.0, 0.0});
  const auto goal_joints = node->declare_parameter<std::vector<double>>(
    "goal_joints", std::vector<double>{0.3, -1.0, 0.5, -0.8, 0.4, 0.3});

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });
  std::this_thread::sleep_for(500ms);

  moveit::planning_interface::MoveGroupInterface move_group(
    node, "elfin_arm", std::shared_ptr<tf2_ros::Buffer>(), rclcpp::Duration::from_seconds(15.0));
  move_group.startStateMonitor(5.0);
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(3);

  int failures = 0;
  double worst = 0.0;
  for (int i = 0; i < repeat; ++i) {
    move_group.setJointValueTarget(goal_joints);
    auto go_result = move_group.move();
    const double goal_err = wait_near(move_group, goal_joints, max_error);
    worst = std::max(worst, goal_err);
    const bool goal_ok = goal_err <= max_error;
    if (!goal_ok) {
      RCLCPP_ERROR(
        node->get_logger(), "repeat %d goal failed (code=%s err=%.6f)", i,
        moveit::core::error_code_to_string(go_result).c_str(), goal_err);
      ++failures;
    } else if (!go_result) {
      RCLCPP_WARN(
        node->get_logger(),
        "repeat %d goal joints ok (err=%.6f) but MoveIt code=%s", i, goal_err,
        moveit::core::error_code_to_string(go_result).c_str());
    }

    move_group.setJointValueTarget(start_joints);
    auto back_result = move_group.move();
    const double start_err = wait_near(move_group, start_joints, max_error);
    worst = std::max(worst, start_err);
    const bool start_ok = start_err <= max_error;
    if (!start_ok) {
      RCLCPP_ERROR(
        node->get_logger(), "repeat %d return failed (code=%s err=%.6f)", i,
        moveit::core::error_code_to_string(back_result).c_str(), start_err);
      ++failures;
    } else if (!back_result) {
      RCLCPP_WARN(
        node->get_logger(),
        "repeat %d return joints ok (err=%.6f) but MoveIt code=%s", i, start_err,
        moveit::core::error_code_to_string(back_result).c_str());
    } else {
      RCLCPP_INFO(
        node->get_logger(), "repeat %d ok goal_err=%.6f start_err=%.6f", i, goal_err, start_err);
    }
  }

  RCLCPP_INFO(
    node->get_logger(), "done repeats=%d failures=%d worst_abs_err=%.6f", repeat, failures, worst);

  rclcpp::shutdown();
  spinner.join();
  return failures == 0 ? 0 : 1;
}
