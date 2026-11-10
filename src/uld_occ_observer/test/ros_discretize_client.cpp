#include "uld_occ_observer/srv/integrate_observation.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cstring>
#include <cstdlib>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 8) {
    std::cerr << "usage: ros_discretize_client ax ay az bx by bz geometry_hash\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("discretize_client");
  auto client = node->create_client<uld_occ_observer::srv::IntegrateObservation>(
      "/uld_occ_observer/integrate_observation");
  if (!client->wait_for_service(std::chrono::seconds(20))) {
    std::cerr << "service unavailable\n";
    rclcpp::shutdown();
    return 3;
  }
  auto request = std::make_shared<uld_occ_observer::srv::IntegrateObservation::Request>();
  request->sensor_id = "test";
  request->observation_id = "1";
  request->clock_domain = "ros";
  request->geometry_hash = argv[7];
  request->epoch = 1;
  request->cloud.header.frame_id = "optical";
  request->cloud.header.stamp.sec = 1;
  request->cloud.height = 1;
  request->cloud.width = 2;
  request->cloud.is_bigendian = false;
  request->cloud.point_step = 12;
  request->cloud.row_step = 24;
  for (const char* name : {"x", "y", "z"}) {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = static_cast<std::uint32_t>(request->cloud.fields.size() * 4);
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    request->cloud.fields.push_back(field);
  }
  request->cloud.data.resize(24);
  for (int i = 0; i < 6; ++i) {
    float value = std::strtof(argv[i + 1], nullptr);
    std::memcpy(request->cloud.data.data() + i * 4, &value, 4);
  }
  request->uld_from_optical.header.frame_id = "container_link";
  request->uld_from_optical.header.stamp.sec = 1;
  request->uld_from_optical.child_frame_id = "optical";
  request->uld_from_optical.transform.rotation.w = 1;
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(20)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    std::cerr << "service timed out\n";
    rclcpp::shutdown();
    return 3;
  }
  const auto response = future.get();
  std::cout << response->receipt.status << " " << response->receipt.free_keys << " "
            << response->receipt.hit_keys << "\n";
  rclcpp::shutdown();
  return response->receipt.status == "INTEGRATED" ? 0 : 3;
}
