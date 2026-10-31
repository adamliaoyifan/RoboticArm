#include "uld_occ_observer/snapshot_repository.hpp"
#include <algorithm>
#include <set>

namespace uld_occ_observer {
InflightResponse::InflightResponse(SnapshotRepository* repository, std::uint64_t bytes)
    : repository_(bytes == 0 ? nullptr : repository), bytes_(bytes) {
  if (repository_ && bytes_) repository_->adjust_inflight(static_cast<std::int64_t>(bytes_));
}
InflightResponse::InflightResponse(InflightResponse&& other) noexcept
    : repository_(other.repository_), bytes_(other.bytes_) {
  other.repository_ = nullptr;
  other.bytes_ = 0;
}
InflightResponse& InflightResponse::operator=(InflightResponse&& other) noexcept {
  if (this == &other) return *this;
  release();
  repository_ = other.repository_;
  bytes_ = other.bytes_;
  other.repository_ = nullptr;
  other.bytes_ = 0;
  return *this;
}
InflightResponse::~InflightResponse() { release(); }
void InflightResponse::release() noexcept {
  if (repository_ && bytes_) repository_->adjust_inflight(-static_cast<std::int64_t>(bytes_));
  repository_ = nullptr;
  bytes_ = 0;
}
InflightResponse SnapshotRepository::begin_response(std::uint64_t bytes) {
  return InflightResponse(this, bytes);
}
void SnapshotRepository::adjust_inflight(std::int64_t delta) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (delta >= 0) inflight_bytes_ += static_cast<std::uint64_t>(delta);
  else {
    const auto decrease = static_cast<std::uint64_t>(-delta);
    inflight_bytes_ = decrease >= inflight_bytes_ ? 0 : inflight_bytes_ - decrease;
  }
  inflight_peak_ = std::max(inflight_peak_, inflight_bytes_);
}
void SnapshotRepository::expire_unlocked(Clock::time_point now, std::uint64_t epoch) {
  for (auto it = entries_.begin(); it != entries_.end();) {
    if (it->second.expires <= now || it->second.snapshot->identity().epoch != epoch)
      it = entries_.erase(it);
    else ++it;
  }
}
void SnapshotRepository::track_unlocked(const std::shared_ptr<const Snapshot>& snapshot) {
  if (!snapshot) return;
  readers_.erase(std::remove_if(readers_.begin(), readers_.end(),
      [](const std::weak_ptr<const Snapshot>& reader) { return reader.expired(); }), readers_.end());
  readers_.push_back(snapshot);
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
  track_unlocked(selected);
  return {handle, identity, age, age > max_age_ns};
}
std::shared_ptr<const Snapshot> SnapshotRepository::get(const std::string& handle) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto epoch = store_.published_epoch();
  expire_unlocked(Clock::now(), epoch);
  const auto entry = entries_.find(handle);
  if (entry == entries_.end()) throw Error("INVALID_OR_EXPIRED_HANDLE");
  track_unlocked(entry->second.snapshot);
  return entry->second.snapshot;
}
void SnapshotRepository::release(const std::string& handle) {
  std::lock_guard<std::mutex> lock(mutex_);
  entries_.erase(handle);
}
void SnapshotRepository::set_ttl_for_test(std::chrono::milliseconds ttl) { ttl_ = ttl; }
void SnapshotRepository::account(const ResourceAccounts& accounts) {
  std::lock_guard<std::mutex> lock(mutex_);
  accounts_ = accounts;
}
ResourceUsage SnapshotRepository::usage() {
  std::vector<std::shared_ptr<const CellBlock>> store_blocks;
  store_.collect_blocks(store_blocks);
  const auto store_directories = store_.directory_entries();
  const auto retained = store_.retained_snapshot();
  std::vector<std::shared_ptr<const Snapshot>> views;
  std::uint64_t inflight = 0;
  std::uint64_t inflight_peak = 0;
  ResourceAccounts accounts;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    expire_unlocked(Clock::now(), store_.published_epoch());
    readers_.erase(std::remove_if(readers_.begin(), readers_.end(),
        [](const std::weak_ptr<const Snapshot>& reader) { return reader.expired(); }), readers_.end());
    for (const auto& entry : entries_) views.push_back(entry.second.snapshot);
    for (const auto& reader : readers_)
      if (auto snapshot = reader.lock()) views.push_back(std::move(snapshot));
    if (retained) views.push_back(retained);
    inflight = inflight_bytes_;
    inflight_peak = inflight_peak_;
    accounts = accounts_;
  }
  std::set<const CellBlock*> current;
  for (const auto& block : store_blocks) if (block) current.insert(block.get());
  std::set<const CellBlock*> unique = current;
  std::set<const Snapshot*> seen_snapshots;
  std::set<const SurfaceCache*> seen_caches;
  std::uint64_t directories = store_directories;
  std::uint64_t surface = 0;
  std::uint64_t old_bytes = 0;
  for (const auto& snapshot : views) {
    if (!snapshot || !seen_snapshots.insert(snapshot.get()).second) continue;
    directories += snapshot->directory_entries();
    if (const auto* cache = snapshot->surface_cache()) {
      if (seen_caches.insert(cache).second) surface += snapshot->surface_cache_allocated_bytes();
    }
    std::vector<std::shared_ptr<const CellBlock>> owned;
    snapshot->collect_blocks(owned);
    for (const auto& block : owned) {
      if (!block || !unique.insert(block.get()).second) continue;
      if (!current.count(block.get())) old_bytes += sizeof(CellBlock);
    }
  }
  const auto shared = static_cast<std::uint64_t>(unique.size()) * sizeof(CellBlock);
  std::lock_guard<std::mutex> lock(mutex_);
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
  usage.inflight_response_bytes = inflight;
  usage.inflight_response_bytes_peak = std::max(inflight_peak, inflight_peak_);
  usage.entry_queue_messages = accounts.entry_messages;
  usage.entry_queue_messages_peak = accounts.entry_message_peak;
  usage.entry_queue_payload_bytes = accounts.entry_payload_bytes;
  usage.entry_queue_payload_bytes_peak = accounts.entry_payload_bytes_peak;
  usage.receipt_ledger_records = accounts.ledger_records;
  usage.receipt_ledger_records_peak = accounts.ledger_record_peak;
  usage.receipt_ledger_bytes = accounts.ledger_bytes;
  usage.receipt_ledger_bytes_peak = accounts.ledger_bytes_peak;
  usage.completion_records = accounts.completion_records;
  usage.completion_records_peak = accounts.completion_records_peak;
  usage.uncovered_allocations = true;
  usage.certification = "NOT_EVALUATED";
  return usage;
}
}  // namespace uld_occ_observer
