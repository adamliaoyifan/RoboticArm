#pragma once
// Performance-load generator. Classification is geometric and does not trust
// which sampler proposed a point. This header does not include product headers.
#include "independent_oracle.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace occ_load {

inline constexpr const char* kGeneratorVersion = "o6-load-v1";
inline constexpr double kInteriorBoxAttempt = 0.6;

enum class EndpointClass { Interior, ExteriorCross, ExteriorMiss, Invalid };

struct Quotas {
  std::size_t interior{};
  std::size_t cross{};
  std::size_t miss{};
};

struct ClassCounts {
  std::size_t interior{};
  std::size_t cross{};
  std::size_t miss{};
  std::size_t invalid{};
};

struct Box {
  observer_oracle::Vec3 center;
  observer_oracle::Vec3 half;
};

struct Cloud {
  std::vector<observer_oracle::Vec3> optical;
  ClassCounts counts;
  Quotas quotas;
  std::string sha256;
  std::size_t attempts{};
};

Quotas quotas_for(std::size_t valid_count, double outside_fraction, double cross_fraction);

std::uint32_t frame_seed(const std::string& group_id, int base_seed, int round_index);

observer_oracle::Rigid sensor_pose();

std::vector<Box> make_boxes(int count, const observer_oracle::Geometry& geometry);

EndpointClass classify_endpoint(const observer_oracle::Geometry& geometry,
                                const observer_oracle::Rigid& pose,
                                const observer_oracle::Vec3& optical,
                                double z_min, double z_max);

ClassCounts classify_cloud(const observer_oracle::Geometry& geometry,
                           const observer_oracle::Rigid& pose,
                           const std::vector<observer_oracle::Vec3>& optical,
                           double z_min, double z_max);

Cloud generate_cloud(const observer_oracle::Geometry& geometry,
                     const std::vector<Box>& boxes,
                     std::size_t valid_count, double outside_fraction, double cross_fraction,
                     std::uint32_t seed, double z_min, double z_max);

struct CheckResult {
  bool pass{false};
  std::string json;
};

CheckResult self_check(const observer_oracle::Geometry& loaded, double z_min, double z_max);

}  // namespace occ_load
