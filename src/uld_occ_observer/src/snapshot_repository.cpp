#include "uld_occ_observer/snapshot_repository.hpp"
#include <set>

namespace uld_occ_observer {
void SnapshotRepository::expire_unlocked(Clock::time_point now, std::uint64_t epoch) {
  for (auto it = entries_.begin(); it != entries_.end();) {
    if (it->second.expires <= now || it->second.snapshot->identity().epoch != epoch)
      it = entries_.erase(it);
    else ++it;
  }
}
SnapshotLease SnapshotRepository::acquire(std::int64_t now_ns, std::int64_t max_age_ns,
    bool allow_stale, std::uint64_t expected_epoch,
    std::optional<std::uint64_t> expected_revision) {
  // Copy the latest snapshot before taking the repository mutex so this method
  // never waits for the store lock while holding the handle table.
  const auto latest = store_.snapshot();
  std::lock_guard<std::mutex> lock(mutex_);
  const auto epoch = store_.published_epoch();
  if (latest->identity().epoch != epoch) throw Error("EPOCH_MISMATCH");
  const auto now = Clock::now();
  expire_unlocked(now, epoch);
  if (expected_epoch && expected_epoch != epoch) throw Error("EPOCH_MISMATCH");
  auto selected = latest;
  if (expected_revision && *expected_revision != latest->identity().map_revision) {
    selected.reset();
    for (const auto& entry : entries_) {
      if (entry.second.snapshot->identity().map_revision == *expected_revision) {
        selected = entry.second.snapshot; break;
      }
    }
    if (!selected) throw Error("REVISION_UNAVAILABLE");
  }
  const auto& identity = selected->identity();
  if (now_ns < identity.last_integrated_stamp_ns || max_age_ns < 0) throw Error("INVALID_QUERY_TIME");
  const auto age = now_ns - identity.last_integrated_stamp_ns;
  if (age > max_age_ns && !allow_stale) throw Error("STALE");
  std::set<std::uint64_t> revisions;
  revisions.insert(identity.map_revision);
  for (const auto& entry : entries_) revisions.insert(entry.second.snapshot->identity().map_revision);
  if (entries_.size() >= 8 || revisions.size() > 2) throw Error("SNAPSHOT_PRESSURE");
  const auto handle = std::to_string(identity.epoch) + ":" + std::to_string(++next_handle_);
  entries_.emplace(handle, Entry{selected, now + ttl_});
  return {handle, identity, age, age > max_age_ns};
}
std::shared_ptr<const Snapshot> SnapshotRepository::get(const std::string& handle) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto epoch = store_.published_epoch();
  expire_unlocked(Clock::now(), epoch);
  const auto entry = entries_.find(handle);
  if (entry == entries_.end()) throw Error("INVALID_OR_EXPIRED_HANDLE");
  return entry->second.snapshot;
}
void SnapshotRepository::release(const std::string& handle) {
  std::lock_guard<std::mutex> lock(mutex_);
  entries_.erase(handle);
}
void SnapshotRepository::set_ttl_for_test(std::chrono::milliseconds ttl) { ttl_ = ttl; }
void SnapshotRepository::set_inflight_response_bytes(std::uint64_t bytes) {
  inflight_bytes_.store(bytes, std::memory_order_relaxed);
  auto peak = inflight_peak_.load(std::memory_order_relaxed);
  while (bytes > peak && !inflight_peak_.compare_exchange_weak(peak, bytes)) {}
}
void SnapshotRepository::set_queue_counts(std::uint64_t entry, std::uint64_t entry_peak,
                                          std::uint64_t ledger, std::uint64_t ledger_peak) {
  entry_messages_.store(entry, std::memory_order_relaxed);
  entry_peak_.store(entry_peak, std::memory_order_relaxed);
  ledger_records_.store(ledger, std::memory_order_relaxed);
  ledger_peak_.store(ledger_peak, std::memory_order_relaxed);
}
ResourceUsage SnapshotRepository::usage() {
  std::vector<std::shared_ptr<const CellBlock>> store_blocks;
  store_.collect_blocks(store_blocks);
  const auto store_directories = store_.directory_entries();
  std::set<const CellBlock*> current;
  for (const auto& block : store_blocks) current.insert(block.get());
  std::lock_guard<std::mutex> lock(mutex_);
  std::set<const CellBlock*> unique = current;
  std::uint64_t directories = store_directories;
  std::uint64_t surface = 0;
  std::uint64_t old_bytes = 0;
  for (const auto& entry : entries_) {
    directories += entry.second.snapshot->directory_entries();
    surface += entry.second.snapshot->surface_cache_allocated_bytes();
    std::vector<std::shared_ptr<const CellBlock>> owned;
    entry.second.snapshot->collect_blocks(owned);
    for (const auto& block : owned) {
      if (!unique.insert(block.get()).second) continue;
      if (!current.count(block.get())) old_bytes += sizeof(CellBlock);
    }
  }
  const auto shared = static_cast<std::uint64_t>(unique.size()) * sizeof(CellBlock);
  block_peak_ = std::max(block_peak_, shared);
  directory_peak_ = std::max(directory_peak_, directories);
  surface_peak_ = std::max(surface_peak_, surface);
  ResourceUsage usage;
  usage.shared_block_bytes = shared;
  usage.shared_block_bytes_peak = block_peak_;
  usage.old_view_block_bytes = old_bytes;
  usage.view_directory_entries = directories;
  usage.view_directory_entries_peak = directory_peak_;
  usage.surface_cache_bytes = surface;
  usage.surface_cache_bytes_peak = surface_peak_;
  usage.inflight_response_bytes = inflight_bytes_.load(std::memory_order_relaxed);
  usage.inflight_response_bytes_peak = inflight_peak_.load(std::memory_order_relaxed);
  usage.entry_queue_messages = entry_messages_.load(std::memory_order_relaxed);
  usage.entry_queue_messages_peak = entry_peak_.load(std::memory_order_relaxed);
  usage.receipt_ledger_records = ledger_records_.load(std::memory_order_relaxed);
  usage.receipt_ledger_records_peak = ledger_peak_.load(std::memory_order_relaxed);
  usage.uncovered_allocations = true;
  return usage;
}
}  // namespace uld_occ_observer
