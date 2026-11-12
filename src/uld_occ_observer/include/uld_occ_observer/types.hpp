#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace uld_occ_observer {

class Error : public std::runtime_error {
 public:
  explicit Error(const std::string& reason) : std::runtime_error(reason) {}
};

struct Vec3 {
  double x{}, y{}, z{};
};

// Value DTOs are copied at boundaries. Algorithms never retain mutable aliases.
struct RigidTransform {
  std::array<double, 9> rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
  Vec3 translation;
  std::int64_t stamp_ns{};
  void validate(std::int64_t acquisition_stamp_ns) const;
  Vec3 apply(const Vec3& point) const;
};

struct ObservationIdentity {
  std::string sensor_id;
  std::string observation_id;
  std::string optical_frame;
  std::string clock_domain;
  std::string geometry_hash;
  std::uint64_t epoch{};
  std::int64_t stamp_ns{};
};

struct OpticalObservation {
  ObservationIdentity identity;
  std::vector<Vec3> points_m;
  RigidTransform uld_from_optical;
  // Legacy DTO name retained; XYZ returns bypass camera optical-Z filtering.
  bool lidar_xyz{false};
};

struct DepthZConfig {
  double min_m{0.52};
  double max_m{6.0};
  void validate() const;
};

struct FilterResult {
  std::vector<Vec3> optical_points;
  std::vector<std::uint8_t> accepted_mask;
  std::size_t raw_count{}, nonfinite_count{}, below_count{}, above_count{};
};

struct UldObservation {
  ObservationIdentity identity;
  std::vector<Vec3> points;
  Vec3 origin;
  std::size_t raw_count{}, rejected_count{};
};

struct CellIndex {
  int x{}, y{}, z{};
  bool operator<(const CellIndex& other) const {
    return std::array<int,3>{x,y,z} < std::array<int,3>{other.x,other.y,other.z};
  }
  bool operator==(const CellIndex& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

bool finite(const Vec3& point);

}  // namespace uld_occ_observer
