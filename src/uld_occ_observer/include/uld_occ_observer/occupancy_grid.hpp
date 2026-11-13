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
  const std::vector<std::uint8_t>* structure_terminal = nullptr;
};

struct OccupancyScan {
  // Unique first-visit order, with all occupied keys removed before completion.
  std::vector<octomap::OcTreeKey> free_cells;
  octomap::KeySet occupied_cells;
  std::size_t rays_attempted{};
  std::size_t endpoints_used{};
 private:
  friend class OccupancyGrid;
  bool complete_{false};
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
  // Limit is for two-bit scratch only, not the output lists or probability tree.
  explicit OccupancyGrid(double resolution, std::size_t bitmap_limit_bytes = 64U * 1024U * 1024U);
  // Overwrites out. On failure out may contain a partial result; do not apply it.
  // apply() rejects an incomplete scan. Scratch is reset even after a failed frame.
  void collect(OccupancyScan& out, const Vec3& origin_uld, const std::vector<Vec3>& endpoints_uld,
               const UldDomain& domain, OccupancyScanOptions options, OccupancyDiagnostic& diagnostic);
  // Complete scan required. Successful return includes parent maintenance.
  // Not transactional: after a write/allocation exception, discard/reset/restore
  // the owning store before reuse; tree, blocks and identity may disagree.
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
  enum class Visit { Inactive, FirstActive, RepeatedActive };
  void beginFrame(const UldDomain& domain);
  Visit visitCell(CellIndex index, const UldDomain& domain);
  void configure();
  // Copies the old state before any write: a search may return a coarse leaf.
  void applyKey(const octomap::OcTreeKey& key, bool hit,
                std::vector<OccupancyMutation>& mutations);
  void castFreeRay(OccupancyScan& scan, OccupancyDiagnostic& diagnostic, const UldDomain& domain,
                   const octomap::point3d& origin, octomap::KeyRay& ray, const Vec3& endpoint_uld,
                   const octomap::point3d& endpoint_om, const octomap::OcTreeKey& endpoint_key,
                   bool admit_occupied);
  void collectDiscretized(OccupancyScan& out, const Vec3& origin_uld,
                          const std::vector<Vec3>& endpoints_uld, const UldDomain& domain,
                          OccupancyDiagnostic& diagnostic);
  double resolution_;
  std::unique_ptr<octomap::OcTree> tree_;
  // Owned by this grid, used only under the Store's existing writer lock.
  // 32 cells/word: 00 unseen, 01 inactive, 10 active. Never indexed outside shape.
  std::size_t bitmap_limit_bytes_;
  CellIndex frame_shape_{};
  std::vector<std::uint64_t> frame_visits_;
};
}  // namespace uld_occ_observer
