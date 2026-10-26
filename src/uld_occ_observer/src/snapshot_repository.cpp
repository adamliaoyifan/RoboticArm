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
  std::lock_guard<std::mutex> lock(mutex_);
  const auto latest = store_.snapshot();
  const auto now = Clock::now();
  expire_unlocked(now, latest->identity().epoch);
  if (expected_epoch && expected_epoch != latest->identity().epoch) throw Error("EPOCH_MISMATCH");
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
  const auto age = now_ns-identity.last_integrated_stamp_ns;
  if (age > max_age_ns && !allow_stale) throw Error("STALE");
  std::set<std::uint64_t> revisions;
  revisions.insert(identity.map_revision);
  for (const auto& entry : entries_) revisions.insert(entry.second.snapshot->identity().map_revision);
  if (entries_.size() >= 8 || revisions.size() > 2) throw Error("SNAPSHOT_PRESSURE");
  const auto handle = std::to_string(identity.epoch) + ":" + std::to_string(++next_handle_);
  entries_.emplace(handle, Entry{selected, now + std::chrono::seconds(5)});
  return {handle, identity, age, age > max_age_ns};
}
std::shared_ptr<const Snapshot> SnapshotRepository::get(const std::string& handle) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto epoch = store_.diagnostics().identity.epoch;
  expire_unlocked(Clock::now(), epoch);
  const auto entry = entries_.find(handle);
  if (entry == entries_.end()) throw Error("INVALID_OR_EXPIRED_HANDLE");
  return entry->second.snapshot;
}
void SnapshotRepository::release(const std::string& handle) {
  std::lock_guard<std::mutex> lock(mutex_);
  entries_.erase(handle);
}
}  // namespace uld_occ_observer
