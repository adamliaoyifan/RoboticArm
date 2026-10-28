#include "uld_occ_observer/occupancy_grid.hpp"
#include "uld_occ_observer/geometry.hpp"
#include <octomap/OcTree.h>
#include <cmath>
#include <sstream>

namespace uld_occ_observer {
namespace {
UpdateProfile& profile_of(UpdateProfile* profile, UpdateProfile& disabled) {
  return profile ? *profile : disabled;
}
}

CellIndex cell_index_from_key(const octomap::OcTreeKey& key) {
  return {static_cast<int>(key[0]) - 32768, static_cast<int>(key[1]) - 32768,
          static_cast<int>(key[2]) - 32768};
}

octomap::OcTreeKey key_from_cell_index(CellIndex index) {
  return {static_cast<octomap::key_type>(index.x + 32768),
          static_cast<octomap::key_type>(index.y + 32768),
          static_cast<octomap::key_type>(index.z + 32768)};
}

OccupancyGrid::OccupancyGrid(double resolution)
    : resolution_(resolution), tree_(std::make_unique<octomap::OcTree>(resolution)) {
  configure();
}

void OccupancyGrid::configure() {
  tree_->setProbHit(.7);
  tree_->setProbMiss(.4);
  tree_->setOccupancyThres(.5);
  tree_->setClampingThresMin(.1192);
  tree_->setClampingThresMax(.971);
}

void OccupancyGrid::castFreeRay(OccupancyScan& scan, UpdateProfile& prof, const UldDomain& domain,
                                const octomap::point3d& origin, octomap::KeyRay& ray, const Vec3& endpoint_uld,
                                const octomap::point3d& endpoint_om, const octomap::OcTreeKey& endpoint_key,
                                bool admit_occupied) {
  if (prof.enabled()) ++prof.rays_attempted;
  ++scan.rays_attempted;
  ++scan.endpoints_used;
  { ProfileSpan ray_span(prof, ProfileStage::RayGeneration, true);
    if (!tree_->computeRayKeys(origin, endpoint_om, ray)) throw Error("RAY_PREPARATION_FAILED");
  }
  ProfileSpan filter_span(prof, ProfileStage::RayFilterDedup, true);
  std::uint64_t active_visits = 0;
  const bool diagnose = prof.enabled();
  if (diagnose) {
    prof.ray_keys_generated += ray.size();
    scan.ray_keys_generated += ray.size();
  }
  for (const auto& key : ray) {
    const auto index = cell_index_from_key(key);
    if (domain.active(index)) {
      const bool inserted = scan.free_cells.insert(key).second;
      if (diagnose) {
        ++active_visits;
        ++prof.active_ray_key_visits;
        ++scan.active_visits;
        if (!inserted) {
          ++prof.free_insert_duplicates;
          ++scan.free_insert_duplicates;
        }
      }
    } else if (diagnose) {
      ++prof.inactive_ray_key_visits;
      ++scan.inactive_visits;
    }
  }
  if (diagnose) {
    if (active_visits) ++prof.rays_with_active_keys;
    else ++prof.rays_without_active_keys;
  }
  if (admit_occupied && domain.contains(endpoint_uld)) {
    const auto index = cell_index_from_key(endpoint_key);
    if (domain.active(index)) scan.occupied_cells.insert(endpoint_key);
  }
}

static void clear_scan(OccupancyScan& scan) {
  scan.free_cells.clear();
  scan.occupied_cells.clear();
  scan.rays_attempted = 0;
  scan.endpoints_used = 0;
  scan.ray_keys_generated = 0;
  scan.active_visits = 0;
  scan.inactive_visits = 0;
  scan.free_insert_duplicates = 0;
  scan.free_unique_before_hit_removal = 0;
}

void OccupancyGrid::collect(OccupancyScan& scan, const Vec3& origin_uld,
                                const std::vector<Vec3>& endpoints_uld, const UldDomain& domain,
                                OccupancyScanOptions options, UpdateProfile* profile) {
  clear_scan(scan);
  if (options.discretize_endpoints) {
    collectDiscretized(scan, origin_uld, endpoints_uld, domain, profile);
    return;
  }
  UpdateProfile disabled;
  auto& prof = profile_of(profile, disabled);
  { ProfileSpan span(prof, ProfileStage::RayPreparation);
    const auto origin = domain.octomap_coordinates(origin_uld);
    octomap::OcTreeKey origin_key;
    if (!tree_->coordToKeyChecked(origin.x, origin.y, origin.z, origin_key))
      throw Error("ORIGIN_OUT_OF_KEY_RANGE");
    const octomap::point3d origin_point(origin.x, origin.y, origin.z);
    octomap::KeyRay ray;
    for (const auto& endpoint : endpoints_uld) {
      const auto point = domain.octomap_coordinates(endpoint);
      octomap::OcTreeKey endpoint_key;
      if (!tree_->coordToKeyChecked(point.x, point.y, point.z, endpoint_key))
        throw Error("ENDPOINT_OUT_OF_KEY_RANGE");
      castFreeRay(scan, prof, domain, origin_point, ray, endpoint,
                  octomap::point3d(point.x, point.y, point.z), endpoint_key, true);
    }
    if (prof.enabled()) {
      prof.free_unique_before_hit_removal = scan.free_cells.size();
      scan.free_unique_before_hit_removal = scan.free_cells.size();
    }
    for (const auto& hit : scan.occupied_cells) scan.free_cells.erase(hit);
  }
}

void OccupancyGrid::collectDiscretized(OccupancyScan& scan, const Vec3& origin_uld,
                                                   const std::vector<Vec3>& endpoints_uld,
                                                   const UldDomain& domain, UpdateProfile* profile) {
  UpdateProfile disabled;
  auto& prof = profile_of(profile, disabled);
  { ProfileSpan span(prof, ProfileStage::RayPreparation);
    const auto origin = domain.octomap_coordinates(origin_uld);
    octomap::OcTreeKey origin_key;
    if (!tree_->coordToKeyChecked(origin.x, origin.y, origin.z, origin_key))
      throw Error("ORIGIN_OUT_OF_KEY_RANGE");
    struct Endpoint {
      Vec3 uld;
      octomap::point3d octomap;
      octomap::OcTreeKey key;
    };
    std::vector<Endpoint> endpoints;
    endpoints.reserve(endpoints_uld.size());
    for (const auto& endpoint : endpoints_uld) {
      const auto point = domain.octomap_coordinates(endpoint);
      octomap::OcTreeKey endpoint_key;
      if (!tree_->coordToKeyChecked(point.x, point.y, point.z, endpoint_key))
        throw Error("ENDPOINT_OUT_OF_KEY_RANGE");
      endpoints.push_back({endpoint, octomap::point3d(point.x, point.y, point.z), endpoint_key});
      if (domain.contains(endpoint)) {
        const auto index = cell_index_from_key(endpoint_key);
        if (domain.active(index)) scan.occupied_cells.insert(endpoint_key);
      }
    }
    std::vector<Endpoint> rays;
    octomap::KeySet seen_endpoints;
    for (const auto& endpoint : endpoints) {
      if (seen_endpoints.insert(endpoint.key).second) rays.push_back(endpoint);
    }
    const octomap::point3d origin_point(origin.x, origin.y, origin.z);
    octomap::KeyRay ray;
    for (const auto& endpoint : rays)
      castFreeRay(scan, prof, domain, origin_point, ray, endpoint.uld, endpoint.octomap, endpoint.key, false);
    if (prof.enabled()) {
      prof.free_unique_before_hit_removal = scan.free_cells.size();
      scan.free_unique_before_hit_removal = scan.free_cells.size();
    }
    for (const auto& hit : scan.occupied_cells) scan.free_cells.erase(hit);
  }
}

std::vector<OccupancyMutation> OccupancyGrid::apply(const OccupancyScan& scan,
                                                    UpdateProfile* profile) {
  UpdateProfile disabled;
  auto& prof = profile_of(profile, disabled);
  std::vector<OccupancyMutation> mutations;
  auto update = [&](const octomap::OcTreeKey& key, bool hit) {
    bool existed, old_occupied, occupied;
    float old_log_odds, new_log_odds;
    { ProfileSpan tree_span(prof, ProfileStage::TreeUpdate, true);
      const auto* before = tree_->search(key);
      existed = before != nullptr;
      old_log_odds = existed ? before->getLogOdds() : 0;
      old_occupied = existed && tree_->isNodeOccupied(before);
      auto* after = tree_->updateNode(key, hit, false);
      occupied = tree_->isNodeOccupied(after);
      new_log_odds = after->getLogOdds();
    }
    if (existed && old_log_odds == new_log_odds) return;
    mutations.push_back({cell_index_from_key(key), hit, existed, old_occupied, occupied,
                         old_log_odds, new_log_odds});
  };
  for (const auto& key : scan.free_cells) update(key, false);
  for (const auto& key : scan.occupied_cells) update(key, true);
  return mutations;
}

void OccupancyGrid::reset() {
  tree_ = std::make_unique<octomap::OcTree>(resolution_);
  configure();
}

std::string OccupancyGrid::serialize() const {
  std::ostringstream stream(std::ios::binary);
  if (!tree_->write(stream)) throw Error("CHECKPOINT_FAILED");
  return stream.str();
}

OccupancyGrid OccupancyGrid::fromCheckpointBytes(const std::string& tree_bytes,
                                                 double expected_resolution) {
  std::istringstream stream(tree_bytes, std::ios::binary);
  std::unique_ptr<octomap::AbstractOcTree> raw(octomap::AbstractOcTree::read(stream));
  auto* restored = dynamic_cast<octomap::OcTree*>(raw.get());
  if (!restored || restored->getResolution() != expected_resolution || !stream)
    throw Error("INVALID_CHECKPOINT_TREE");
  restored->expand();
  raw.release();
  OccupancyGrid grid(expected_resolution);
  grid.tree_.reset(restored);
  grid.configure();
  return grid;
}

void OccupancyGrid::validateLeaves(const UldDomain& domain) const {
  for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it) {
    const auto index = cell_index_from_key(it.getKey());
    if (!domain.active(index) || !std::isfinite(it->getLogOdds()) ||
        it->getLogOdds() < octomap::logodds(.1192) - 1e-6 ||
        it->getLogOdds() > octomap::logodds(.971) + 1e-6)
      throw Error("INVALID_CHECKPOINT_CELL");
  }
}

void OccupancyGrid::forEachLeaf(const std::function<void(CellIndex, float)>& visit) const {
  for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it)
    visit(cell_index_from_key(it.getKey()), it->getLogOdds());
}

std::size_t OccupancyGrid::nodeCount() const { return tree_->size(); }
std::size_t OccupancyGrid::memoryUsage() const { return tree_->memoryUsage(); }
}  // namespace uld_occ_observer
