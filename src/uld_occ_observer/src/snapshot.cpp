#include "uld_occ_observer/snapshot.hpp"
#include <algorithm>

namespace uld_occ_observer {
Snapshot::Snapshot(SnapshotIdentity identity, std::shared_ptr<const UldDomain> domain,
                   BlockRegistry blocks, std::shared_ptr<SurfaceCache> surface_cache)
    : identity_(std::move(identity)), domain_(std::move(domain)), blocks_(std::move(blocks)),
      surface_cache_(surface_cache ? std::move(surface_cache) : std::make_shared<SurfaceCache>()) {}

std::shared_ptr<const Snapshot> Snapshot::with_identity(SnapshotIdentity identity) const {
  if (identity.map_revision != identity_.map_revision || identity.epoch != identity_.epoch)
    throw Error("SNAPSHOT_IDENTITY_MISMATCH");
  return std::make_shared<const Snapshot>(std::move(identity), domain_, blocks_, surface_cache_);
}

CellIndex Snapshot::block_index(CellIndex i) { return {i.x/8, i.y/8, i.z/8}; }
std::uint64_t Snapshot::known_entry_count() const {
  std::uint64_t count = 0;
  for (const auto& entry : blocks_) count += entry.second->size();
  return count;
}
CellSample Snapshot::cell(CellIndex index) const {
  CellSample result{index, domain_->active(index), false, {}};
  if (!result.active) return result;
  const auto block = blocks_.find(block_index(index));
  if (block == blocks_.end()) return result;
  const auto item = block->second->find(index);
  if (item != block->second->end()) { result.known = true; result.value = item->second; }
  return result;
}

CellPage Snapshot::read_cells(CellIndex begin, CellIndex end, std::size_t cursor,
                              std::size_t limit) const {
  const auto shape = domain_->shape();
  if (begin.x < 0 || begin.y < 0 || begin.z < 0 || end.x > shape.x ||
      end.y > shape.y || end.z > shape.z || begin.x >= end.x ||
      begin.y >= end.y || begin.z >= end.z || limit == 0 || limit > 65536)
    throw Error("INVALID_ROI_OR_PAGE");
  const std::size_t dx = end.x-begin.x, dy = end.y-begin.y, dz = end.z-begin.z;
  const std::size_t total = dx*dy*dz;
  if (cursor > total) throw Error("INVALID_CURSOR");
  CellPage result;
  const auto stop = cursor + std::min(limit, total-cursor);
  result.cells.reserve(stop-cursor);
  for (auto offset = cursor; offset < stop; ++offset) {
    result.cells.push_back(cell({begin.x + static_cast<int>(offset%dx),
        begin.y + static_cast<int>((offset/dx)%dy),
        begin.z + static_cast<int>(offset/(dx*dy))}));
  }
  result.next_cursor = stop;
  result.done = stop == total;
  return result;
}

std::shared_ptr<const std::vector<SurfaceColumn>> Snapshot::surface(SurfaceProfile* profile) const {
  using Clock = std::chrono::steady_clock;
  const auto started = profile ? Clock::now() : Clock::time_point{};
  if (profile) {
    *profile = SurfaceProfile{};
    profile->domain_init_collected = true;
  }
  std::lock_guard<std::mutex> lock(surface_cache_->mutex);
  const auto locked = profile ? Clock::now() : Clock::time_point{};
  if (profile) profile->lock_wait_ms = std::chrono::duration<double,std::milli>(locked-started).count();
  if (surface_cache_->value) {
    if (profile) {
      profile->cache_hit = true;
      profile->output_columns = surface_cache_->value->size();
      profile->total_ms = std::chrono::duration<double,std::milli>(Clock::now()-started).count();
    }
    return surface_cache_->value;
  }
  const auto shape = domain_->shape();
  DomainInitObservation init;
  const bool observe_init = profile != nullptr;
  const auto& active_counts = domain_->column_active_counts(observe_init ? &init : nullptr, observe_init);
  if (profile) {
    profile->domain_init_performed = init.performed;
    profile->domain_init_scanned_cells = init.scanned_cells;
    profile->domain_init_ms = init.milliseconds;
  }
  auto output = std::make_shared<std::vector<SurfaceColumn>>(active_counts.size());
  std::vector<int> highest_occupied(active_counts.size(), -1);
  for (int y = 0; y < shape.y; ++y) {
    for (int x = 0; x < shape.x; ++x) {
      const auto column_index = static_cast<std::size_t>(y) * shape.x + x;
      auto& column = (*output)[column_index];
      column.x = x;
      column.y = y;
      column.active_count = active_counts[column_index];
    }
  }
  std::uint64_t scanned = 0;
  const double ceiling = domain_->descriptor().ceiling_z;
  const double origin_z = domain_->origin().z;
  const double resolution = domain_->resolution();
  for (const auto& entry : blocks_) {
    for (const auto& item : *entry.second) {
      ++scanned;
      const auto& index = item.first;
      if (!domain_->active(index)) continue;
      const auto column_index = static_cast<std::size_t>(index.y) * shape.x + index.x;
      if (column_index >= output->size()) continue;
      auto& column = (*output)[column_index];
      ++column.known_count;
      if (item.second.occupied && index.z > highest_occupied[column_index]) {
        highest_occupied[column_index] = index.z;
        column.has_occupied = true;
        column.top_z = std::min(ceiling, origin_z + (index.z + 1) * resolution);
      }
    }
  }
  surface_cache_->value = output;
  if (profile) {
    profile->scanned_cells = scanned;
    profile->output_columns = output->size();
    const auto finished = Clock::now();
    profile->build_ms = std::chrono::duration<double,std::milli>(finished-locked).count();
    profile->total_ms = std::chrono::duration<double,std::milli>(finished-started).count();
  }
  return surface_cache_->value;
}
}  // namespace uld_occ_observer
