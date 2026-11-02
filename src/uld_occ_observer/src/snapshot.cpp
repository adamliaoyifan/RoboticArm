#include "uld_occ_observer/snapshot.hpp"
#include "uld_occ_observer/interface_trace.hpp"
#include "uld_occ_observer/block_storage.hpp"
#include "uld_occ_observer/resource_usage.hpp"
#include <algorithm>
#include <string>

namespace uld_occ_observer {
SurfaceCache::~SurfaceCache() {
  // Only the cache OBJECT is released here. The column buffer is owned by
  // its own shared_ptr (surface() hands it to callers), so its bytes are
  // subtracted by the buffer's deleter at the LAST release — a caller may
  // legitimately keep the buffer alive after the cache and snapshot are
  // gone, and those bytes stay attributed until that reader lets go.
  AllocationTotals::instance().subtract(
      "surface_caches", sizeof(SurfaceCache) + kControlBlockBytes);
}
Snapshot::~Snapshot() {
  auto& totals = AllocationTotals::instance();
  totals.subtract("snapshot_objects", sizeof(Snapshot) + kControlBlockBytes);
  totals.subtract("block_directory_nodes",
                  static_cast<std::uint64_t>(blocks_.size())
                      * map_node_bytes<BlockRegistry::value_type>());
}
Snapshot::Snapshot(SnapshotIdentity identity, std::shared_ptr<const UldDomain> domain,
                   BlockRegistry blocks, std::shared_ptr<SurfaceCache> surface_cache)
    : identity_(std::move(identity)), domain_(std::move(domain)), blocks_(std::move(blocks)),
      fresh_surface_cache_(!surface_cache),
      surface_cache_(surface_cache ? std::move(surface_cache)
                                   : std::make_shared<SurfaceCache>()) {
  // Occupancy accounting: the snapshot object plus the registry copy's
  // nodes. A cache object is counted exactly once, at its creation — the
  // freshness flag is captured BEFORE the parameter is moved into the
  // member: testing the moved-from parameter would see null and
  // double-count every shared cache (72 -> 144 bytes with a residual 72
  // after release).
  auto& totals = AllocationTotals::instance();
  totals.add("snapshot_objects", sizeof(Snapshot) + kControlBlockBytes);
  totals.add("block_directory_nodes",
             static_cast<std::uint64_t>(blocks_.size())
                 * map_node_bytes<BlockRegistry::value_type>());
  if (fresh_surface_cache_)
    totals.add("surface_caches", sizeof(SurfaceCache) + kControlBlockBytes);
}

std::shared_ptr<const Snapshot> Snapshot::with_identity(SnapshotIdentity identity) const {
  if (identity.map_revision != identity_.map_revision || identity.epoch != identity_.epoch)
    throw Error("SNAPSHOT_IDENTITY_MISMATCH");
  return std::make_shared<const Snapshot>(std::move(identity), domain_, blocks_, surface_cache_);
}

CellIndex Snapshot::block_index(CellIndex i) { return BlockLayout::block_coordinate(i); }
void Snapshot::for_each_known_cell(
    const std::function<void(const CellIndex&, const CellValue&)>& visitor) const {
  const BlockLayout layout(domain_->shape());
  for (const auto& entry : blocks_)
    entry.second->for_each_known([&](std::uint16_t offset, const CellValue& value) {
      visitor(layout.cell_index(entry.first, offset), value);
    });
}
void Snapshot::collect_blocks(std::vector<std::shared_ptr<const CellBlock>>& out) const {
  for (const auto& entry : blocks_) out.push_back(entry.second);
}
std::uint64_t Snapshot::surface_cache_allocated_bytes() const {
  std::lock_guard<std::mutex> lock(surface_cache_->mutex);
  if (!surface_cache_->value) return 0;
  return surface_cache_->value->capacity() * sizeof(SurfaceColumn);
}
const SurfaceCache* Snapshot::surface_cache() const { return surface_cache_.get(); }
std::uint64_t Snapshot::known_entry_count() const {
  std::uint64_t count = 0;
  for (const auto& entry : blocks_) count += entry.second->known_count();
  return count;
}
bool CellBlock::read(std::uint16_t offset, CellValue& value) const {
  if (offset >= kBlockCells) throw Error("INVALID_BLOCK_OFFSET");
  const auto word = static_cast<std::size_t>(offset) / 64u;
  const auto bit = offset % 64u;
  if ((known_[word] & (std::uint64_t{1} << bit)) == 0) {
    value = {};
    return false;
  }
  value = values_[offset];
  return true;
}
bool CellBlock::write(std::uint16_t offset, CellValue value) {
  if (offset >= kBlockCells) throw Error("INVALID_BLOCK_OFFSET");
  const auto word = static_cast<std::size_t>(offset) / 64u;
  const auto bit = offset % 64u;
  const auto mask = std::uint64_t{1} << bit;
  const bool first = (known_[word] & mask) == 0;
  if (first) {
    known_[word] |= mask;
    ++known_count_;
  }
  values_[offset] = value;
  return first;
}
CellSample Snapshot::cell(CellIndex index) const {
  CellSample result{index, domain_->active(index), false, {}};
  if (!result.active) return result;
  const BlockLayout layout(domain_->shape());
  const auto address = layout.locate(index);
  const auto block = blocks_.find(address.block_index);
  if (block == blocks_.end()) return result;
  result.known = block->second->read(address.local_offset, result.value);
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
  interface_trace_event("roi_begin", "", "", "query");
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
  interface_trace_event("roi_end", "", std::to_string(result.cells.size()).c_str(), "query");
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
  const bool cache_hit = static_cast<bool>(surface_cache_->value);
  interface_trace_event("surface_begin", "", cache_hit ? "hit" : "miss", "query");
  const auto locked = profile ? Clock::now() : Clock::time_point{};
  if (profile) profile->lock_wait_ms = std::chrono::duration<double,std::milli>(locked-started).count();
  if (surface_cache_->value) {
    if (profile) {
      profile->cache_hit = true;
      profile->output_columns = surface_cache_->value->size();
      profile->total_ms = std::chrono::duration<double,std::milli>(Clock::now()-started).count();
    }
    interface_trace_event("surface_end", "", "hit", "query");
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
  // The buffer's bytes ride its own shared ownership: added here, released
  // by the deleter at the last owner (cache replacement, cache destruction,
  // or an external reader letting go — whichever happens last).
  const auto buffer_bytes = static_cast<std::uint64_t>(active_counts.size())
      * sizeof(SurfaceColumn);
  AllocationTotals::instance().add("surface_caches", buffer_bytes);
  auto output = std::shared_ptr<std::vector<SurfaceColumn>>(
      new std::vector<SurfaceColumn>(active_counts.size()),
      [buffer_bytes](std::vector<SurfaceColumn>* columns) {
        AllocationTotals::instance().subtract("surface_caches", buffer_bytes);
        delete columns;
      });
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
  const BlockLayout layout(shape);
  for (const auto& entry : blocks_) {
    entry.second->for_each_known([&](std::uint16_t offset, const CellValue& value) {
      ++scanned;
      const auto index = layout.cell_index(entry.first, offset);
      if (!domain_->active(index)) return;
      const auto column_index = static_cast<std::size_t>(index.y) * shape.x + index.x;
      if (column_index >= output->size()) return;
      auto& column = (*output)[column_index];
      ++column.known_count;
      if (value.occupied && index.z > highest_occupied[column_index]) {
        highest_occupied[column_index] = index.z;
        column.has_occupied = true;
        column.top_z = std::min(ceiling, origin_z + (index.z + 1) * resolution);
      }
    });
  }
  // Occupancy accounting on a rebuild: the new buffer was added at its
  // allocation above; the old buffer's bytes leave with its deleter when
  // the last owner (possibly an external reader) releases it.
  surface_cache_->value = output;
  if (profile) {
    profile->scanned_cells = scanned;
    profile->output_columns = output->size();
    const auto finished = Clock::now();
    profile->build_ms = std::chrono::duration<double,std::milli>(finished-locked).count();
    profile->total_ms = std::chrono::duration<double,std::milli>(finished-started).count();
  }
  interface_trace_event("surface_end", "", "miss", "query");
  return surface_cache_->value;
}
}  // namespace uld_occ_observer
