#include <HR_Pro.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace
{
constexpr unsigned int kBoxId = 0;
constexpr unsigned int kRobotId = 0;
constexpr double kDefaultAccelDeg = 60.0;
constexpr int kStateRefuse = 20018;
constexpr int kFsmEnabling = 23;
constexpr int kFsmStandby = 33;
constexpr int kFsmDisable = 24;
constexpr int kFsmBlackout = 7;
constexpr int kFsmElectrifying = 8;
constexpr int kFsmElectricBoxDisconnect = 2;
constexpr int kFsmElectricBoxConnecting = 3;

std::string errorString(int code)
{
  std::string message;
  if (HRIF_GetErrorCodeStr(kBoxId, code, message) == 0 && !message.empty())
  {
    std::ostringstream out;
    out << message << " (" << code << ")";
    return out.str();
  }
  std::ostringstream out;
  out << "error code " << code;
  return out.str();
}

double durationToSec(const ros::Duration& duration)
{
  return duration.toSec();
}

std::vector<double> radiansToDegrees(const std::vector<double>& radians)
{
  std::vector<double> degrees;
  degrees.reserve(radians.size());
  for (double value : radians)
  {
    degrees.push_back(value * 180.0 / M_PI);
  }
  return degrees;
}

std::vector<double> degreesToRadians(const std::vector<double>& degrees)
{
  std::vector<double> radians;
  radians.reserve(degrees.size());
  for (double value : degrees)
  {
    radians.push_back(value * M_PI / 180.0);
  }
  return radians;
}
}  // namespace

class CppCpsTrajectoryExecutor
{
public:
  CppCpsTrajectoryExecutor()
    : nh_()
    , pnh_("~")
    , server_(nh_, "follow_joint_trajectory",
              boost::bind(&CppCpsTrajectoryExecutor::executeCb, this, _1), false)
  {
    pnh_.param<std::string>("robot_ip", robot_ip_, "192.168.0.10");
    pnh_.param<int>("robot_port", robot_port_, 10003);
    pnh_.param<double>("default_velocity_deg", default_velocity_deg_, 30.0);
    pnh_.param<double>("max_velocity_deg", max_velocity_deg_, 60.0);
    pnh_.param<double>("poll_interval_s", poll_interval_s_, 0.05);
    pnh_.param<double>("blend_radius_mm", blend_radius_mm_, 5.0);
    pnh_.param<double>("final_blend_radius_mm", final_blend_radius_mm_, 0.0);
    pnh_.param<double>("controller_start_timeout_s", controller_start_timeout_s_, 30.0);
    pnh_.param<bool>("power_off_on_disconnect", power_off_on_disconnect_, false);

    if (!pnh_.getParam("joint_names", joint_names_))
    {
      joint_names_ = {"elfin_joint1", "elfin_joint2", "elfin_joint3",
                      "elfin_joint4", "elfin_joint5", "elfin_joint6"};
    }
    current_positions_rad_.assign(joint_names_.size(), 0.0);

    joint_pub_ = root_nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    joint_timer_ = root_nh_.createTimer(ros::Duration(0.01), &CppCpsTrajectoryExecutor::publishJointStates, this);

    server_.start();
    ROS_INFO("C++ CPS trajectory executor ready");
  }

  ~CppCpsTrajectoryExecutor()
  {
    disconnect();
  }

  bool connect()
  {
    ROS_INFO_STREAM("Connecting to Huayan controller " << robot_ip_ << ":" << robot_port_);

    const int connect_ret = HRIF_Connect(kBoxId, robot_ip_.c_str(), static_cast<unsigned short>(robot_port_));
    if (connect_ret != 0)
    {
      ROS_ERROR_STREAM("HRIF_Connect failed: " << errorString(connect_ret));
      return false;
    }

    int fsm_id = -1;
    std::string fsm_desc;
    readFsm(fsm_id, fsm_desc);
    if (fsm_id >= 0)
    {
      ROS_INFO_STREAM("Current robot FSM: " << fsm_id << " (" << fsm_desc << ")");
    }

    if (fsm_id == 5 || fsm_id == 22)
    {
      ROS_ERROR(
          "Robot is in fault/e-stop state. Clear faults on the teach pendant first.");
      return false;
    }

    const bool connect2box_allow_refuse =
        fsm_id < 0 ||
        (fsm_id != kFsmElectricBoxDisconnect && fsm_id != kFsmElectricBoxConnecting);
    const bool electrify_allow_refuse = fsm_id >= kFsmEnabling;

    if (!callStep("HRIF_Connect2Box", []() { return HRIF_Connect2Box(kBoxId); },
                  connect2box_allow_refuse) ||
        !callStep("HRIF_Electrify", []() { return HRIF_Electrify(kBoxId); },
                  electrify_allow_refuse))
    {
      return false;
    }

    if (fsm_id == kFsmBlackout || fsm_id == kFsmElectrifying || fsm_id == 6)
    {
      ROS_INFO("Waiting for robot to finish powering up...");
      if (!waitForControllerBoot(controller_start_timeout_s_))
      {
        ROS_ERROR("Robot did not finish powering up within timeout");
        return false;
      }
      readFsm(fsm_id, fsm_desc);
      if (fsm_id >= 0)
      {
        ROS_INFO_STREAM("Robot FSM after power-up: " << fsm_id << " (" << fsm_desc << ")");
      }
    }

    const bool connect_controller_allow_refuse =
        controllerStarted() || fsm_id >= kFsmEnabling;
    if (!callStep("HRIF_Connect2Controller",
                  []() { return HRIF_Connect2Controller(kBoxId); },
                  connect_controller_allow_refuse))
    {
      return false;
    }

    if (!controllerStarted() &&
        !waitForControllerBoot(controller_start_timeout_s_))
    {
      ROS_ERROR("Controller did not start within timeout");
      return false;
    }

    if (isEnabled())
    {
      ROS_INFO("Robot servos already enabled");
    }
    else
    {
      if (fsm_id == kFsmDisable)
      {
        callStep("HRIF_GrpReset", []() { return HRIF_GrpReset(kBoxId, kRobotId); }, true);
        ros::Duration(0.3).sleep();
      }
      if (!callStep("HRIF_GrpEnable", []() { return HRIF_GrpEnable(kBoxId, kRobotId); }, false))
      {
        return false;
      }
      ros::Duration(0.3).sleep();
    }

    refreshPositions();
    ready_ = true;
    ROS_INFO("Huayan controller ready");
    return true;
  }

private:
  template <typename Fn>
  bool callStep(const std::string& name, Fn fn, bool allow_state_refuse = false)
  {
    const int ret = fn();
    if (ret == 0)
    {
      return true;
    }
    if (allow_state_refuse && ret == kStateRefuse)
    {
      ROS_INFO_STREAM(name << " skipped: " << errorString(ret));
      return true;
    }
    ROS_ERROR_STREAM(name << " failed: " << errorString(ret));
    return false;
  }

  void readFsm(int& fsm_id, std::string& fsm_desc) const
  {
    fsm_id = -1;
    fsm_desc.clear();
    const int ret = HRIF_ReadCurFSM(kBoxId, kRobotId, fsm_id, fsm_desc);
    if (ret != 0)
    {
      fsm_id = -1;
      fsm_desc.clear();
    }
  }

  bool waitForControllerBoot(double timeout_s) const
  {
    const ros::Time deadline = ros::Time::now() + ros::Duration(timeout_s);
    while (ros::ok() && ros::Time::now() < deadline)
    {
      if (controllerStarted())
      {
        return true;
      }
      int fsm_id = -1;
      std::string fsm_desc;
      readFsm(fsm_id, fsm_desc);
      if (fsm_id == 5 || fsm_id == 22)
      {
        return false;
      }
      ros::Duration(0.5).sleep();
    }
    return controllerStarted();
  }

  bool controllerStarted() const
  {
    int started = 0;
    return HRIF_IsControllerStarted(kBoxId, started) == 0 && started == 1;
  }

  bool isEnabled() const
  {
    int moving = 0;
    int enable = 0;
    int error_state = 0;
    int error_code = 0;
    int error_axis = 0;
    int breaking = 0;
    int pause = 0;
    int emergency = 0;
    int safeguard = 0;
    int electrify = 0;
    int box_connected = 0;
    int blending_done = 0;
    int in_pos = 0;
    if (HRIF_ReadRobotState(kBoxId, kRobotId, moving, enable, error_state, error_code, error_axis,
                            breaking, pause, emergency, safeguard, electrify, box_connected,
                            blending_done, in_pos) != 0)
    {
      return false;
    }
    return enable == 1;
  }

  void disconnect()
  {
    if (!ready_ && !HRIF_IsConnected(kBoxId))
    {
      return;
    }
    if (isEnabled())
    {
      HRIF_GrpDisable(kBoxId, kRobotId);
      ros::Duration(0.2).sleep();
    }
    if (power_off_on_disconnect_)
    {
      HRIF_Blackout(kBoxId);
      ros::Duration(0.2).sleep();
    }
    HRIF_DisConnect(kBoxId);
    ready_ = false;
  }

  void executeCb(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
  {
    trajectory_msgs::JointTrajectory trajectory = goal->trajectory;
    std::string error;
    if (!normalizeTrajectory(trajectory, error))
    {
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      result.error_string = error;
      server_.setAborted(result, error);
      return;
    }

    if (!ready_ && !connect())
    {
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
      result.error_string = "Robot is not ready";
      server_.setAborted(result, result.error_string);
      return;
    }

    std::vector<double> times_s;
    times_s.reserve(trajectory.points.size());
    for (const auto& point : trajectory.points)
    {
      times_s.push_back(durationToSec(point.time_from_start));
    }

    for (std::size_t idx = 0; idx < trajectory.points.size(); ++idx)
    {
      if (server_.isPreemptRequested() || !ros::ok())
      {
        stop();
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
        result.error_string = "Trajectory preempted";
        server_.setPreempted(result, result.error_string);
        return;
      }

      const auto& point = trajectory.points[idx];
      const std::vector<double> joints_deg = radiansToDegrees(point.positions);
      const bool is_last = (idx + 1 == trajectory.points.size());
      const double velocity = estimateVelocity(idx, trajectory.points, times_s, joints_deg);
      const double accel = std::max(kDefaultAccelDeg, velocity * 2.0);
      const double radius = is_last ? final_blend_radius_mm_ : blend_radius_mm_;

      const int ret = HRIF_WayPoint(
          kBoxId, kRobotId, 0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          joints_deg[0], joints_deg[1], joints_deg[2],
          joints_deg[3], joints_deg[4], joints_deg[5],
          "TCP", "Base", velocity, accel, radius,
          1, 0, 0, 0, std::to_string(idx));

      if (ret != 0)
      {
        stop();
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
        result.error_string = "HRIF_WayPoint failed: " + errorString(ret);
        server_.setAborted(result, result.error_string);
        return;
      }

      if (!waitForWaypoint(is_last))
      {
        return;
      }
    }

    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    server_.setSucceeded(result, "Trajectory execution complete");
  }

  bool waitForWaypoint(bool is_last)
  {
    ros::Rate rate(1.0 / poll_interval_s_);
    while (ros::ok())
    {
      if (server_.isPreemptRequested())
      {
        stop();
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
        result.error_string = "Trajectory preempted";
        server_.setPreempted(result, result.error_string);
        return false;
      }

      refreshPositions();
      publishFeedback();

      bool done = false;
      const int ret = is_last ? HRIF_IsMotionDone(kBoxId, kRobotId, done)
                              : HRIF_IsBlendingDone(kBoxId, kRobotId, done);
      if (ret != 0)
      {
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
        result.error_string = "Motion state query failed: " + errorString(ret);
        server_.setAborted(result, result.error_string);
        return false;
      }
      if (done)
      {
        return true;
      }
      rate.sleep();
    }
    return false;
  }

  bool normalizeTrajectory(trajectory_msgs::JointTrajectory& trajectory, std::string& error) const
  {
    if (trajectory.points.empty())
    {
      return true;
    }
    if (trajectory.joint_names.size() != joint_names_.size())
    {
      error = "Unexpected joint count";
      return false;
    }

    std::vector<std::size_t> order;
    order.reserve(joint_names_.size());
    for (const auto& name : joint_names_)
    {
      const auto it = std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), name);
      if (it == trajectory.joint_names.end())
      {
        error = "Missing joint " + name;
        return false;
      }
      order.push_back(static_cast<std::size_t>(std::distance(trajectory.joint_names.begin(), it)));
    }

    for (auto& point : trajectory.points)
    {
      if (point.positions.size() != joint_names_.size())
      {
        error = "Trajectory point has wrong position count";
        return false;
      }
      reorder(point.positions, order);
      if (!point.velocities.empty())
      {
        reorder(point.velocities, order);
      }
      if (!point.accelerations.empty())
      {
        reorder(point.accelerations, order);
      }
    }
    trajectory.joint_names = joint_names_;
    return true;
  }

  static void reorder(std::vector<double>& values, const std::vector<std::size_t>& order)
  {
    std::vector<double> reordered;
    reordered.reserve(order.size());
    for (std::size_t index : order)
    {
      reordered.push_back(values[index]);
    }
    values.swap(reordered);
  }

  double estimateVelocity(std::size_t idx,
                          const std::vector<trajectory_msgs::JointTrajectoryPoint>& points,
                          const std::vector<double>& times_s,
                          const std::vector<double>& joints_deg) const
  {
    const auto& point = points[idx];
    if (!point.velocities.empty())
    {
      double max_vel = 0.0;
      for (double value : point.velocities)
      {
        max_vel = std::max(max_vel, std::abs(value * 180.0 / M_PI));
      }
      if (max_vel > 0.0)
      {
        return std::min(max_vel, max_velocity_deg_);
      }
    }

    if (idx + 1 < points.size())
    {
      const std::vector<double> next_deg = radiansToDegrees(points[idx + 1].positions);
      const double dt = times_s[idx + 1] - times_s[idx];
      if (dt > 1e-6)
      {
        double max_delta = 0.0;
        for (std::size_t i = 0; i < joints_deg.size(); ++i)
        {
          max_delta = std::max(max_delta, std::abs(next_deg[i] - joints_deg[i]));
        }
        if (max_delta > 0.0)
        {
          return std::min(max_delta / dt, max_velocity_deg_);
        }
      }
    }

    return std::min(default_velocity_deg_, max_velocity_deg_);
  }

  void refreshPositions()
  {
    double j1 = 0.0;
    double j2 = 0.0;
    double j3 = 0.0;
    double j4 = 0.0;
    double j5 = 0.0;
    double j6 = 0.0;
    if (HRIF_ReadActJointPos(kBoxId, kRobotId, j1, j2, j3, j4, j5, j6) != 0)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(position_mutex_);
    current_positions_rad_ = degreesToRadians({j1, j2, j3, j4, j5, j6});
  }

  void stop()
  {
    HRIF_GrpStop(kBoxId, kRobotId);
    ros::Duration(0.1).sleep();
    HRIF_GrpReset(kBoxId, kRobotId);
  }

  void publishFeedback()
  {
    control_msgs::FollowJointTrajectoryFeedback feedback;
    feedback.header.stamp = ros::Time::now();
    feedback.joint_names = joint_names_;
    {
      std::lock_guard<std::mutex> lock(position_mutex_);
      feedback.actual.positions = current_positions_rad_;
    }
    server_.publishFeedback(feedback);
  }

  void publishJointStates(const ros::TimerEvent&)
  {
    if (ready_ || HRIF_IsConnected(kBoxId))
    {
      refreshPositions();
    }

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.name = joint_names_;
    {
      std::lock_guard<std::mutex> lock(position_mutex_);
      msg.position = current_positions_rad_;
    }
    joint_pub_.publish(msg);
  }

  ros::NodeHandle root_nh_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> server_;
  ros::Publisher joint_pub_;
  ros::Timer joint_timer_;

  std::string robot_ip_;
  int robot_port_ = 10003;
  double default_velocity_deg_ = 30.0;
  double max_velocity_deg_ = 60.0;
  double poll_interval_s_ = 0.05;
  double blend_radius_mm_ = 5.0;
  double final_blend_radius_mm_ = 0.0;
  double controller_start_timeout_s_ = 30.0;
  bool power_off_on_disconnect_ = false;
  bool ready_ = false;

  std::vector<std::string> joint_names_;
  std::vector<double> current_positions_rad_;
  std::mutex position_mutex_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cpp_cps_trajectory_executor");
  CppCpsTrajectoryExecutor executor;

  ros::NodeHandle pnh("~");
  bool connect_on_start = true;
  pnh.param<bool>("connect_on_start", connect_on_start, true);
  if (connect_on_start)
  {
    executor.connect();
  }

  ros::spin();
  return 0;
}
