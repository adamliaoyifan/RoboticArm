// Offline geometric recovery of an exact saved Cartesian edge. No execution.
// A different grasp preserves every requested BOX pose, including the endpoint.
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/srv/get_cartesian_path.hpp>

std::string readFile(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error(path);
  return {std::istreambuf_iterator<char>(stream), {}};
}
template<class T> T decode(const std::string& path) {
  auto bytes = readFile(path);
  rclcpp::SerializedMessage serialized(bytes.size());
  auto& raw = serialized.get_rcl_serialized_message();
  memcpy(raw.buffer, bytes.data(), bytes.size()); raw.buffer_length = bytes.size();
  T value; rclcpp::Serialization<T>().deserialize_message(&serialized, &value);
  return value;
}
template<class T> void save(const std::string& path, const T& value) {
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<T>().serialize_message(&value, &serialized);
  auto& raw = serialized.get_rcl_serialized_message();
  std::ofstream stream(path, std::ios::binary);
  stream.write(reinterpret_cast<char*>(raw.buffer), raw.buffer_length);
  if (!stream) throw std::runtime_error("cannot preserve " + path);
}
Eigen::Isometry3d transform(const geometry_msgs::msg::Pose& pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  result.linear() = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x,
                                      pose.orientation.y, pose.orientation.z).normalized().toRotationMatrix();
  return result;
}
geometry_msgs::msg::Pose message(const Eigen::Isometry3d& transform) {
  geometry_msgs::msg::Pose pose;
  pose.position.x=transform.translation().x(); pose.position.y=transform.translation().y();
  pose.position.z=transform.translation().z();
  Eigen::Quaterniond q(transform.linear());
  pose.orientation.x=q.x(); pose.orientation.y=q.y(); pose.orientation.z=q.z(); pose.orientation.w=q.w();
  return pose;
}

int main(int argc, char** argv) {
  if (argc != 3) { std::cerr << "usage: prior_cartesian_replay CAPTURE NEW_OUTPUT\n"; return 2; }
  const std::string capture=argv[1], output=argv[2];
  if (!std::filesystem::create_directory(output)) throw std::runtime_error("output must be new");
  rclcpp::init(argc,argv);
  std::cout << std::setprecision(17);
  auto original=decode<moveit_msgs::srv::GetCartesianPath::Request>(capture+"/req-1a/edge-0000.request.cdr");
  auto original_scene=decode<moveit_msgs::msg::PlanningScene>(capture+"/req-1a/edge-0000.scene.cdr");
  rclcpp::NodeOptions options; options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides({
    rclcpp::Parameter("robot_description",readFile(capture+"/robot.urdf")),
    rclcpp::Parameter("robot_description_semantic",readFile(capture+"/robot.srdf")),
    rclcpp::Parameter("robot_description_kinematics.elfin_arm.kinematics_solver","kdl_kinematics_plugin/KDLKinematicsPlugin"),
    rclcpp::Parameter("robot_description_kinematics.elfin_arm.kinematics_solver_search_resolution",.005),
    rclcpp::Parameter("robot_description_kinematics.elfin_arm.kinematics_solver_timeout",.05)});
  auto node=std::make_shared<rclcpp::Node>("prior_cartesian_replay",options);
  robot_model_loader::RobotModelLoader loader(node);
  auto model=loader.getModel(); auto group=model->getJointModelGroup(original.group_name);
  auto link=model->getLinkModel(original.link_name);
  // resource_retriever logs missing package meshes but RobotModel still loads.
  // Fail closed rather than certifying a robot with its collision bodies gone.
  for(const auto& entry:model->getURDF()->links_) {
    const auto& urdf_link=entry.second;
    if((urdf_link->collision || !urdf_link->collision_array.empty()) &&
       model->getLinkModel(entry.first)->getShapes().empty())
      throw std::runtime_error("collision geometry failed to load: "+entry.first);
  }
  if (!group || !link || !group->getSolverInstance() || original.header.frame_id!=model->getModelFrame()
      || original.waypoints.empty() || !original.path_constraints.joint_constraints.empty()
      || !original.path_constraints.orientation_constraints.empty()
      || !original.path_constraints.position_constraints.empty())
    throw std::runtime_error("fixture requires direct model-frame path and empty constraints");
  const auto& attached=original_scene.robot_state.attached_collision_objects;
  if (attached.size()!=1 || attached[0].link_name!=original.link_name
      || attached[0].object.header.frame_id!=original.link_name
      || attached[0].object.primitives.size()!=1)
    throw std::runtime_error("one TCP-relative primitive payload required");
  Eigen::Isometry3d old_tcp_box=transform(attached[0].object.pose);
  Eigen::Isometry3d old_goal=transform(original.waypoints.back());
  Eigen::Isometry3d box_goal=old_goal*old_tcp_box;
  for (int millimetres : {0,20,40,80}) {
    std::string directory=output+"/offset-"+std::to_string(millimetres);
    std::filesystem::create_directory(directory);
    auto request=original; auto scene_message=original_scene;
    auto shifted_goal=old_goal; shifted_goal.translation().x()+=millimetres/1000.;
    Eigen::Isometry3d new_tcp_box=shifted_goal.inverse()*box_goal;
    // No ACM exemption, margin relaxation or collision-disable control.
    scene_message.robot_state.attached_collision_objects[0].object.pose=message(new_tcp_box);
    planning_scene::PlanningScene scene(model); scene.setPlanningSceneMsg(scene_message);
    auto start=scene.getCurrentState();
    moveit::core::robotStateMsgToRobotState(original.start_state,start); start.update();
    auto desired_start=start.getGlobalLinkTransform(link)*old_tcp_box*new_tcp_box.inverse();
    int collision_rejects=0;
    moveit::core::GroupStateValidityCallbackFn valid=[&](moveit::core::RobotState* state,
        const moveit::core::JointModelGroup* joints,const double* positions) {
      state->setJointGroupPositions(joints,positions); state->update();
      collision_detection::CollisionRequest query; query.group_name=original.group_name;
      query.contacts=true; query.max_contacts=20;
      collision_detection::CollisionResult result; scene.checkCollision(query,result,*state);
      if(result.collision) {
        ++collision_rejects;
        for(const auto& pair:result.contacts)
          std::cout << "CONTACT "<<millimetres<<" "<<pair.first.first<<" "<<pair.first.second<<"\n";
      }
      return !result.collision && state->satisfiesBounds(joints);
    };
    bool start_ok=millimetres==0 || start.setFromIK(group,desired_start,original.link_name,.05,valid);
    start.update();
    EigenSTL::vector_Isometry3d waypoints;
    for(auto& waypoint:request.waypoints) {
      Eigen::Isometry3d shifted=transform(waypoint)*old_tcp_box*new_tcp_box.inverse();
      waypoint=message(shifted); waypoints.push_back(shifted);
    }
    moveit::core::robotStateToRobotStateMsg(start,request.start_state);
    save(directory+"/request.cdr",request); save(directory+"/scene.cdr",scene_message);
    if(!start_ok) { std::cout<<"RESULT "<<millimetres<<" START_IK_FAILED\n"; continue; }
    auto state=start; std::vector<moveit::core::RobotStatePtr> path;
    double fraction=moveit::core::CartesianInterpolator::computeCartesianPath(&state,group,path,link,
        waypoints,true,moveit::core::MaxEEFStep(request.max_step),
        moveit::core::JumpThreshold(request.jump_threshold),valid);
    robot_trajectory::RobotTrajectory trajectory(model,request.group_name);
    for(const auto& point:path) trajectory.addSuffixWayPoint(point,0.);
    moveit_msgs::msg::RobotTrajectory geometry; trajectory.getRobotTrajectoryMsg(geometry);
    save(directory+"/geometric.cdr",geometry);
    trajectory_processing::TimeOptimalTrajectoryGeneration timing;
    bool timed=!path.empty() && timing.computeTimeStamps(trajectory,1.);
    moveit_msgs::msg::RobotTrajectory timed_message; trajectory.getRobotTrajectoryMsg(timed_message);
    save(directory+"/timed.cdr",timed_message);
    int invalid_timed=0;
    for(size_t i=0;i<trajectory.getWayPointCount();++i) {
      auto sample=trajectory.getWayPoint(i); std::vector<double> q;
      sample.copyJointGroupPositions(group,q);
      if(!valid(&sample,group,q.data())) ++invalid_timed;
    }
    double endpoint_error=path.empty() ? -1. :
        ((path.back()->getGlobalLinkTransform(link)*new_tcp_box).translation()-box_goal.translation()).norm();
    std::cout<<"RESULT "<<millimetres<<" fraction "<<fraction<<" geometric_states "<<path.size()
             <<" box_endpoint_error_m "<<endpoint_error<<" timing_ok "<<timed
             <<" timed_states "<<trajectory.getWayPointCount()<<" invalid_timed "<<invalid_timed
             <<" collision_rejects "<<collision_rejects<<"\n";
  }
  rclcpp::shutdown();
}
