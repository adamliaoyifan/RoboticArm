// Preserved candidate geometry, old/new private scenes; no ROS graph or motion.
#include <fstream>
#include <iostream>
#include <rclcpp/serialization.hpp>
#include <srdfdom/model.h>
#include <urdf/model.h>
#include <luggage_msgs/srv/check_placement_tool_v2.hpp>
#include "luggage_planning/prior_tool_collision.hpp"

std::string readFile(const std::string& path) {
  std::ifstream file(path,std::ios::binary);
  if (!file) throw std::runtime_error(path);
  return {std::istreambuf_iterator<char>(file),{}};
}
template<class T> T decode(const std::string& path) {
  auto bytes=readFile(path); rclcpp::SerializedMessage serialized(bytes.size());
  auto& raw=serialized.get_rcl_serialized_message();
  memcpy(raw.buffer,bytes.data(),bytes.size()); raw.buffer_length=bytes.size();
  T message; rclcpp::Serialization<T>().deserialize_message(&serialized,&message);
  return message;
}
Eigen::Isometry3d transform(const geometry_msgs::msg::Pose& pose) {
  auto result=Eigen::Isometry3d::Identity();
  result.translation()=Eigen::Vector3d(pose.position.x,pose.position.y,pose.position.z);
  result.linear()=Eigen::Quaterniond(pose.orientation.w,pose.orientation.x,
                                   pose.orientation.y,pose.orientation.z).toRotationMatrix();
  return result;
}
int main(int argc,char**argv) {
  if (argc<5) { std::cerr<<"usage: prior_volume_replay URDF SRDF NEW_SCENE REQUEST...\n";return 2; }
  auto urdf=std::make_shared<urdf::Model>();auto srdf=std::make_shared<srdf::Model>();
  if (!urdf->initString(readFile(argv[1])) || !srdf->initString(*urdf,readFile(argv[2])))return 3;
  auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
  auto replacement=decode<moveit_msgs::msg::PlanningScene>(argv[3]);
  for (int argument=4;argument<argc;++argument) {
    auto request=decode<luggage_msgs::srv::CheckPlacementToolV2::Request>(argv[argument]);
    if (request.tcp_poses.size()!=request.box_poses.size())throw std::runtime_error("pose count mismatch");
    for (int mode=0;mode<2;++mode) {
      luggage_planning::PriorToolCollision checker(model,mode ? replacement : request.scene,request.tcp_link);
      for (size_t index=0;index<request.tcp_poses.size();++index) {
        auto result=checker.check(transform(request.tcp_poses[index]),transform(request.box_poses[index]),
          Eigen::Vector3d(request.box_wdh[0],request.box_wdh[1],request.box_wdh[2]),request.payload_id);
        std::cout<<"RESULT "<<request.request_id<<' '<<mode<<' '<<index<<' '<<result.collision<<'\n';
        for (const auto& contact:result.contacts)
          std::cout<<"CONTACT "<<request.request_id<<' '<<mode<<' '<<index<<' '
                   <<contact.first.first<<' '<<contact.first.second<<'\n';
      }
    }
  }
}
