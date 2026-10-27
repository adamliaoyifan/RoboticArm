#pragma once

#include "uld_occ_observer/profiling.hpp"
#include "uld_occ_observer/types.hpp"
#include <octomap/OcTree.h>
#include <functional>
#include <memory>
#include <vector>

namespace uld_occ_observer {
class UldDomain;

// Default false. When true, one representative endpoint per OcTreeKey casts the
// free ray. Occupied cells are still collected from every original endpoint.
struct OccupancyScanOptions {
  bool discretize_endpoints = false;
};

struct OccupancyScan {
  octomap::KeySet free_cells;
  octomap::KeySet occupied_cells;
  std::size_t rays_attempted{};
  std::size_t endpoints_used{};
  // Optional diagnostics. Zero means not collected unless profile mode is Stages or Detailed.
  std::uint64_t ray_keys_generated{};
  std::uint64_t active_visits{};
  std::uint64_t inactive_visits{};
  std::uint64_t free_insert_duplicates{};
  std::uint64_t free_unique_before_hit_removal{};
};

struct OccupancyMutation {
  CellIndex index;
  bool hit{};
  bool existed{};
  bool old_occupied{};
  bool occupied{};
  float old_log_odds{};
  float new_log_odds{};
};

CellIndex cell_index_from_key(const octomap::OcTreeKey& key);
octomap::OcTreeKey key_from_cell_index(CellIndex index);

// Sole owner of the OcTree. Ray keys, KeySet dedup, and probability writes live here.
class OccupancyGrid {
 public:
  explicit OccupancyGrid(double resolution);
  OccupancyScan collect(const Vec3& origin_uld, const std::vector<Vec3>& endpoints_uld,
                        const UldDomain& domain, OccupancyScanOptions options,
                        UpdateProfile* profile);
  std::vector<OccupancyMutation> apply(const OccupancyScan& scan, UpdateProfile* profile);
  void reset();
  std::string serialize() const;
  static OccupancyGrid fromCheckpointBytes(const std::string& tree_bytes,
                                           double expected_resolution);
  void validateLeaves(const UldDomain& domain) const;
  void forEachLeaf(const std::function<void(CellIndex, float)>& visit) const;
  std::size_t nodeCount() const;
  std::size_t memoryUsage() const;
 private:
  void configure();
  void castFreeRay(OccupancyScan& scan, UpdateProfile& profile, const UldDomain& domain,
                   const octomap::point3d& origin, const Vec3& endpoint_uld,
                   const octomap::point3d& endpoint_om, const octomap::OcTreeKey& endpoint_key,
                   bool admit_occupied);
  OccupancyScan collectDiscretized(const Vec3& origin_uld, const std::vector<Vec3>& endpoints_uld,
                                   const UldDomain& domain, UpdateProfile* profile);
  double resolution_;
  std::unique_ptr<octomap::OcTree> tree_;
};
}  // namespace uld_occ_observer
