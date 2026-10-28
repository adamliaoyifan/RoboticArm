#pragma once
#include "uld_occ_observer/geometry.hpp"
#include "uld_occ_observer/profiling.hpp"
#include <map>
#include <memory>
#include <mutex>

namespace uld_occ_observer {
struct SnapshotIdentity {
  std::uint64_t epoch{}, map_revision{}, observed_revision{};
  std::int64_t last_integrated_stamp_ns{};
  std::string geometry_hash, config_digest, clock_domain;
};
struct CellValue { float log_odds{}; bool occupied{}; };
using CellBlock = std::map<CellIndex, CellValue>;
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
  std::uint64_t known_entry_count() const;
  static CellIndex block_index(CellIndex index);
 private:
  const SnapshotIdentity identity_;
  const std::shared_ptr<const UldDomain> domain_;
  const BlockRegistry blocks_;
  const std::shared_ptr<SurfaceCache> surface_cache_;
};
}  // namespace uld_occ_observer
