// Offline FCL check of a captured mesh/box pair. No ROS or simulator writes.
// Input: vertex/triangle counts and data, then cases (label, mesh transform,
// box size, box transform). Transform matrices are row-major 4x4.
#include <fcl/fcl.h>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

fcl::Transform3d read_transform() {
  Eigen::Matrix4d matrix;
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 4; ++col) std::cin >> matrix(row, col);
  return fcl::Transform3d(matrix);
}

int main() {
  std::size_t vertices_count, triangles_count;
  if (!(std::cin >> vertices_count >> triangles_count)) return 2;
  std::vector<fcl::Vector3d> vertices(vertices_count);
  for (auto &vertex : vertices) std::cin >> vertex[0] >> vertex[1] >> vertex[2];
  std::vector<fcl::Triangle> triangles;
  for (std::size_t index = 0; index < triangles_count; ++index) {
    int a, b, c;
    std::cin >> a >> b >> c;
    triangles.emplace_back(a, b, c);
  }
  auto mesh = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
  mesh->beginModel();
  mesh->addSubModel(vertices, triangles);
  mesh->endModel();
  std::size_t cases;
  std::cin >> cases;
  for (std::size_t index = 0; index < cases; ++index) {
    std::string label;
    std::cin >> label;
    auto mesh_transform = read_transform();
    double x, y, z;
    std::cin >> x >> y >> z;
    auto box_transform = read_transform();
    auto box = std::make_shared<fcl::Boxd>(x, y, z);
    fcl::CollisionObjectd first(mesh, mesh_transform), second(box, box_transform);
    fcl::CollisionRequestd request;
    request.enable_contact = true; request.num_max_contacts = 16;
    fcl::CollisionResultd collision;
    fcl::collide(&first, &second, request, collision);
    fcl::DistanceRequestd distance_request;
    fcl::DistanceResultd distance;
    fcl::distance(&first, &second, distance_request, distance);
    std::cout << label << " " << collision.isCollision() << " "
              << std::setprecision(17) << distance.min_distance;
    std::vector<fcl::Contactd> contacts;
    collision.getContacts(contacts);
    std::cout << " " << contacts.size();
    for (const auto &contact : contacts)
      std::cout << " " << contact.pos.x() << " " << contact.pos.y() << " " << contact.pos.z() << " " << contact.penetration_depth;
    std::cout << "\n";
  }
  return std::cin.fail() ? 3 : 0;
}
