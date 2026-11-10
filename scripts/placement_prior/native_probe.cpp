// Functional rehearsal against preserved CDR, no ROS graph/services/execution.
#include <fstream>
#include <iomanip>
#include <iostream>
#include <rclcpp/serialization.hpp>
#include <srdfdom/model.h>
#include <urdf/model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include "luggage_planning/prior_tool_collision.hpp"
std::string read(const std::string& p){std::ifstream f(p,std::ios::binary);if(!f)throw std::runtime_error(p);return {std::istreambuf_iterator<char>(f),{}};}
template<class T>T decode(const std::string& p){auto b=read(p);rclcpp::SerializedMessage s(b.size());auto& raw=s.get_rcl_serialized_message();memcpy(raw.buffer,b.data(),b.size());raw.buffer_length=b.size();T out;rclcpp::Serialization<T>().deserialize_message(&s,&out);return out;}
Eigen::Isometry3d pose(const geometry_msgs::msg::Pose& p){Eigen::Isometry3d t=Eigen::Isometry3d::Identity();t.translation()=Eigen::Vector3d(p.position.x,p.position.y,p.position.z);t.linear()=Eigen::Quaterniond(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z).toRotationMatrix();return t;}
int main(int argc,char**argv){
 if(argc!=2){std::cerr<<"usage: prior_native_probe ACQUISITION_DIRECTORY\n";return 2;}
 std::string root=argv[1];std::cout<<std::setprecision(17);
 auto urdf=std::make_shared<urdf::Model>();auto srdf=std::make_shared<srdf::Model>();
 if(!urdf->initString(read(root+"/robot.urdf")) || !srdf->initString(*urdf,read(root+"/robot.srdf")))return 3;
 auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
 auto req=decode<moveit_msgs::srv::GetCartesianPath::Request>(root+"/req-1a/edge-0000.request.cdr");
 auto scene_msg=decode<moveit_msgs::msg::PlanningScene>(root+"/req-1a/edge-0000.scene.cdr");
 planning_scene::PlanningScene scene(model);scene.setPlanningSceneMsg(scene_msg);
 auto state=scene.getCurrentState();moveit::core::robotStateMsgToRobotState(req.start_state,state);state.update();
 std::vector<const moveit::core::AttachedBody*> bodies;state.getAttachedBodies(bodies);
 if(bodies.size()!=1 || bodies[0]->getShapes().size()!=1 || bodies[0]->getShapes()[0]->type!=shapes::BOX)throw std::runtime_error("one box payload required");
 const auto* box=static_cast<const shapes::Box*>(bodies[0]->getShapes()[0].get());
 Eigen::Vector3d size(box->size[0],box->size[1],box->size[2]);
 auto tcp=pose(req.waypoints.back());
 Eigen::Isometry3d tcp_box=state.getGlobalLinkTransform(req.link_name).inverse()*bodies[0]->getGlobalCollisionBodyTransforms()[0];
 Eigen::Isometry3d world_box=tcp*tcp_box;
 luggage_planning::PriorToolCollision checker(model,scene_msg,req.link_name);
 Eigen::Quaterniond bq(world_box.linear());std::cout<<"BOX_QUAT "<<bq.x()<<" "<<bq.y()<<" "<<bq.z()<<" "<<bq.w()<<"\n";
 std::cout<<"BOX size "<<size.transpose()<<" target "<<world_box.translation().transpose()<<"\n";
 for(auto& name:checker.rigidLinks())std::cout<<"RIGID "<<name<<"\n";
 bool baseline_collision=false, offset_clear=false;
 for(double dx:{0.,.01,.02,.03,.04,.06,.08}){
  auto shifted=tcp;shifted.translation().x()+=dx;
  auto result=checker.check(shifted,world_box,size,bodies[0]->getName());
  if(dx==0.)baseline_collision=result.collision;if(dx==.02)offset_clear=!result.collision;
  std::cout<<"OFFSET_WORLD_X "<<dx<<" collision "<<result.collision<<" tcp "<<shifted.translation().transpose()<<"\n";
  Eigen::Isometry3d grasp=world_box.inverse()*shifted;
  std::cout<<"GRASP_BOX "<<grasp.translation().transpose()<<"\n";
  for(auto& p:result.contacts)std::cout<<"CONTACT "<<p.first.first<<" "<<p.first.second<<"\n";
 }
 if(!baseline_collision || !offset_clear)throw std::runtime_error("native regression: centre must collide and 20mm shift must clear rigid tool");
}
