#pragma once
#include "uld_occ_observer/types.hpp"
#include <cstdint>
#include <mutex>
#include <optional>
#include <vector>

namespace uld_occ_observer {
struct Chamfer {
  double floor_y{}, wall_y{}, wall_z{};
};

// Produced by the existing geometry kernel exporter, never by scene guesses.
struct GeometryDescriptor {
  std::string geometry_hash;
  std::string artifact_digest;
  double length{}, width{}, floor_z{}, ceiling_z{};
  std::optional<Chamfer> chamfer;
};

// Filled only for the call that actually runs initialization. Waiting callers stay at performed=false.
struct DomainInitObservation {
  bool performed{false};
  std::uint64_t scanned_cells{0};
  double milliseconds{0};
};

class UldDomain {
 public:
  UldDomain(GeometryDescriptor descriptor, double resolution);
  bool contains(const Vec3& point) const;
  bool active(const CellIndex& index) const;
  std::optional<CellIndex> index_of(const Vec3& point) const;
  Vec3 center(const CellIndex& index) const;
  Vec3 octomap_coordinates(const Vec3& point) const;
  Vec3 origin() const { return origin_; }
  CellIndex shape() const { return shape_; }
  double resolution() const { return resolution_; }
  const GeometryDescriptor& descriptor() const { return descriptor_; }
  // Active cell counts per column. The domain does not change, so this is computed once.
  // measure_time reads a clock only inside the call that executes initialization.
  const std::vector<std::uint32_t>& column_active_counts(
      DomainInitObservation* observation = nullptr, bool measure_time = false) const;
 private:
  const GeometryDescriptor descriptor_;
  const double resolution_;
  Vec3 origin_;
  CellIndex shape_;
  mutable std::once_flag active_once_;
  mutable std::vector<std::uint32_t> active_counts_;
};
}  // namespace uld_occ_observer
