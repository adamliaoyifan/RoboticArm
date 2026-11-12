#include "uld_occ_observer/initial_empty.hpp"
#include "uld_occ_observer/observer.hpp"
#include "uld_occ_observer/block_storage.hpp"

namespace uld_occ_observer {
BlockRegistry initial_empty_blocks(const UldDomain& domain, std::size_t& known) {
  BlockRegistry result;
  OccupancyDiagnostic diagnostic(ProfileMode::Off);
  BlockUpdateBatch batch(BlockLayout(domain.shape()), result, diagnostic);
  known = 0;
  const auto shape = domain.shape();
  for (int z = 0; z < shape.z; ++z)
    for (int y = 0; y < shape.y; ++y)
      for (int x = 0; x < shape.x; ++x)
        if (domain.active({x,y,z})) {
          batch.write({x,y,z}, CellValue{0.f, false, true});
          ++known;
        }
  batch.commit(result);
  return result;
}
bool declaration_only(const SnapshotIdentity& id) {
  return !id.initialization_id.empty() && !id.declared_by.empty() &&
      id.initialization_stamp_ns > 0 && id.observed_revision == 0 &&
      id.last_integrated_stamp_ns == 0;
}
std::int64_t evidence_stamp(const SnapshotIdentity& id) {
  return declaration_only(id) ? id.initialization_stamp_ns : id.last_integrated_stamp_ns;
}
SnapshotIdentity ObservationStore::initialize_empty(std::uint64_t expected_epoch,
    const std::string& id, const std::string& authority, std::int64_t stamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (id.empty() || authority.empty() || stamp <= 0) throw Error("INVALID_EMPTY_DECLARATION");
  // Replay never erases sensor updates or starts a second epoch.
  if (initialization_id_ == id && declared_by_ == authority &&
      (expected_epoch == epoch_ || expected_epoch + 1 == epoch_)) return identity_unlocked();
  if (epoch_ != expected_epoch) throw Error("EPOCH_MISMATCH");
  if (initialization_used_) throw Error("INITIALIZATION_ALREADY_USED");
  std::size_t known = 0;
  auto candidate = initial_empty_blocks(*domain_, known);
  // All allocations completed before replacing live state.
  AllocationTotals::instance().subtract("block_directory_nodes",
      blocks_.size() * map_node_bytes<BlockRegistry::value_type>());
  grid_.reset(); blocks_ = std::move(candidate); known_cells_ = known;
  ++epoch_; map_revision_ = 1; observed_revision_ = 0; last_integrated_stamp_ns_ = 0;
  initialization_id_ = id; declared_by_ = authority; initialization_stamp_ns_ = stamp;
  initialization_used_ = true;
  sensor_stamps_.clear(); fingerprints_.clear(); fingerprint_order_.clear();
  cached_snapshot_.reset();
  published_epoch_.store(epoch_, std::memory_order_release);
  return identity_unlocked();
}
}
