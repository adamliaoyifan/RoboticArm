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

std::shared_ptr<const std::vector<SurfaceColumn>> Snapshot::surface() const {
  std::lock_guard<std::mutex> lock(surface_cache_->mutex);
  if (surface_cache_->value) return surface_cache_->value;
  auto output = std::make_shared<std::vector<SurfaceColumn>>();
  const auto shape = domain_->shape();
  output->reserve(static_cast<std::size_t>(shape.x)*shape.y);
  for (int y = 0; y < shape.y; ++y) {
    for (int x = 0; x < shape.x; ++x) {
      SurfaceColumn column;
      column.x = x; column.y = y;
      for (int z = 0; z < shape.z; ++z) {
        const auto sample = cell({x,y,z});
        column.active_count += sample.active;
        column.known_count += sample.known;
        if (sample.known && sample.value.occupied) {
          column.has_occupied = true;
          column.top_z = std::min(domain_->descriptor().ceiling_z,
              domain_->origin().z + (z+1)*domain_->resolution());
        }
      }
      output->push_back(column);
    }
  }
  surface_cache_->value = output;
  return surface_cache_->value;
}
}  // namespace uld_occ_observer
