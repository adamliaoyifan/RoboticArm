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
  // Overwrites out. On failure out may contain a partial result; do not apply it.
  // KeySet is a TR1 hashtable, so the caller keeps ownership instead of a returned copy.
  void collect(OccupancyScan& out, const Vec3& origin_uld, const std::vector<Vec3>& endpoints_uld,
               const UldDomain& domain, OccupancyScanOptions options, OccupancyDiagnostic& diagnostic);
  std::vector<OccupancyMutation> apply(const OccupancyScan& scan, OccupancyDiagnostic& diagnostic);
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
  void castFreeRay(OccupancyScan& scan, OccupancyDiagnostic& diagnostic, const UldDomain& domain,
                   const octomap::point3d& origin, octomap::KeyRay& ray, const Vec3& endpoint_uld,
                   const octomap::point3d& endpoint_om, const octomap::OcTreeKey& endpoint_key,
                   bool admit_occupied);
  void collectDiscretized(OccupancyScan& out, const Vec3& origin_uld,
                          const std::vector<Vec3>& endpoints_uld, const UldDomain& domain,
                          OccupancyDiagnostic& diagnostic);
  double resolution_;
  std::unique_ptr<octomap::OcTree> tree_;
};
}  // namespace uld_occ_observer
