// Offline whole-robot endpoint checks. Does not certify connecting trajectories.
#include <fstream>
#include <iostream>
#include <rclcpp/serialization.hpp>
#include <srdfdom/model.h>
#include <urdf/model.h>
#include <moveit/planning_scene/planning_scene.h>

std::string readFile(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) throw std::runtime_error("Cannot read " + path);
  return {std::istreambuf_iterator<char>(file), {}};
}

int main(int argc, char** argv) {
  if (argc != 5) {
    std::cerr << "usage: prior_observation_check URDF SRDF SCENE_CDR JOINT_ROWS\n";
    return 2;
  }
  auto urdf = std::make_shared<urdf::Model>();
  auto srdf = std::make_shared<srdf::Model>();
  if (!urdf->initString(readFile(argv[1])) || !srdf->initString(*urdf, readFile(argv[2]))) return 3;
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  auto bytes = readFile(argv[3]);
  rclcpp::SerializedMessage serialized(bytes.size());
  auto& raw = serialized.get_rcl_serialized_message();
  memcpy(raw.buffer, bytes.data(), bytes.size());
  raw.buffer_length = bytes.size();
  moveit_msgs::msg::PlanningScene message;
  rclcpp::Serialization<moveit_msgs::msg::PlanningScene>().deserialize_message(&serialized, &message);
  planning_scene::PlanningScene scene(model);
  scene.setPlanningSceneMsg(message);
  for (const auto& id : {"airport_container_real", "robot_pedestal"}) {
    auto object = scene.getWorld()->getObject(id);
    if (!object || object->shapes_.empty())
      throw std::runtime_error(std::string("Missing scene geometry: ") + id);
    std::cout << "WORLD " << id << " shapes " << object->shapes_.size() << '\n';
  }
  for (const auto* link : model->getLinkModelsWithCollisionGeometry()) {
    if (link->getShapes().empty()) throw std::runtime_error("Empty robot collision geometry");
  }
  std::vector<const moveit::core::AttachedBody*> attached;
  scene.getCurrentState().getAttachedBodies(attached);
  if (!attached.empty())
    throw std::runtime_error("Observation expects no attached payload");
  std::ifstream rows(argv[4]);
  if (!rows) return 4;
  int index;
  while (rows >> index) {
    auto state = scene.getCurrentState();
    for (int joint = 1; joint <= 6; ++joint) {
      double value;
      if (!(rows >> value)) throw std::runtime_error("Incomplete joint row");
      state.setVariablePosition("elfin_joint" + std::to_string(joint), value);
    }
    state.update();
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 20;
    collision_detection::CollisionResult result;
    scene.checkCollision(request, result, state);
    std::cout << "CANDIDATE " << index << " bounds " << state.satisfiesBounds()
              << " collision " << result.collision << '\n';
    for (const auto& contact : result.contacts)
      std::cout << "CONTACT " << index << ' ' << contact.first.first << ' '
                << contact.first.second << '\n';
  }
}
