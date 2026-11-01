#pragma once
#include "uld_occ_observer/resource_usage.hpp"
#include "uld_occ_observer/geometry.hpp"
#include "uld_occ_observer/profiling.hpp"
#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <vector>

namespace uld_occ_observer {
struct SnapshotIdentity {
  std::uint64_t epoch{}, map_revision{}, observed_revision{};
  std::int64_t last_integrated_stamp_ns{};
  std::string geometry_hash, config_digest, clock_domain;
};
struct CellValue { float log_odds{}; bool occupied{}; };
// One edge length for block coordinates, the directory, and this storage.
inline constexpr int kBlockEdge = 8;
inline constexpr int kBlockCells = kBlockEdge * kBlockEdge * kBlockEdge;

// Fixed slots. Unknown is only the clear bit in known_; a zero log-odds value
// can still be a known free cell. Writes go through write(); callers do not
// touch the arrays or the count.
class CellBlock {
 public:
  // Occupancy accounting: each object (with its shared control block) is
  // released exactly once, wherever its last owner drops it.
  ~CellBlock() {
    AllocationTotals::instance().subtract(
        "block_allocations", sizeof(CellBlock) + kControlBlockBytes);
  }
  bool read(std::uint16_t offset, CellValue& value) const;
  // True only when this offset was unknown. A later probability change returns false.
  bool write(std::uint16_t offset, CellValue value);
  std::size_t known_count() const { return known_count_; }
  template <class Visitor>
  void for_each_known(Visitor&& visitor) const {
    for (std::size_t word = 0; word < known_.size(); ++word) {
      auto bits = known_[word];
      while (bits != 0) {
        const int bit = __builtin_ctzll(bits);
        const auto offset = static_cast<std::uint16_t>(word * 64u + static_cast<unsigned>(bit));
        visitor(offset, values_[offset]);
        bits &= bits - 1;
      }
    }
  }

 private:
  std::array<CellValue, kBlockCells> values_{};
  std::array<std::uint64_t, kBlockCells / 64> known_{};
  std::size_t known_count_{};
};
using BlockRegistry = std::map<CellIndex, std::shared_ptr<const CellBlock>>;
struct CellSample { CellIndex index; bool active{}, known{}; CellValue value; };
struct CellPage { std::vector<CellSample> cells; std::size_t next_cursor{}; bool done{}; };
struct SurfaceColumn {
  int x{}, y{};
  std::size_t active_count{}, known_count{};
  bool has_occupied{};
  double top_z{};
};

struct SurfaceCache {
  ~SurfaceCache();
  std::mutex mutex;
  std::shared_ptr<const std::vector<SurfaceColumn>> value;
};

class Snapshot {
 public:
  Snapshot(SnapshotIdentity identity, std::shared_ptr<const UldDomain> domain,
           BlockRegistry blocks, std::shared_ptr<SurfaceCache> surface_cache = nullptr);
  const SnapshotIdentity& identity() const { return identity_; }
  CellSample cell(CellIndex index) const;
  CellPage read_cells(CellIndex begin, CellIndex end, std::size_t cursor,
                      std::size_t limit) const;
  std::shared_ptr<const std::vector<SurfaceColumn>> surface(SurfaceProfile* profile = nullptr) const;
  std::shared_ptr<const Snapshot> with_identity(SnapshotIdentity identity) const;
  std::size_t block_count() const { return blocks_.size(); }
  std::uint64_t directory_entries() const { return blocks_.size(); }
  void collect_blocks(std::vector<std::shared_ptr<const CellBlock>>& out) const;
  std::uint64_t surface_cache_allocated_bytes() const;
  const SurfaceCache* surface_cache() const;
  std::uint64_t known_entry_count() const;
  static CellIndex block_index(CellIndex index);
  ~Snapshot();
 private:
  const SnapshotIdentity identity_;
  const std::shared_ptr<const UldDomain> domain_;
  const BlockRegistry blocks_;
  // Declared before surface_cache_ so the init list can use it: true when
  // this snapshot CREATED the cache (counted once here), false when it
  // shares an existing one (never re-counted).
  const bool fresh_surface_cache_;
  const std::shared_ptr<SurfaceCache> surface_cache_;
};
}  // namespace uld_occ_observer
