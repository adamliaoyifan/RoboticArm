#pragma once
#include "uld_occ_observer/observer.hpp"
#include <chrono>

namespace uld_occ_observer {
struct SnapshotLease {
  std::string handle;
  SnapshotIdentity identity;
  std::int64_t age_ns{};
  bool stale{};
};

// Handles own immutable snapshots, not mutable OcTree pointers. Access begins by
// copying shared ownership under this mutex; ongoing reads survive TTL expiry.
class SnapshotRepository {
 public:
  explicit SnapshotRepository(ObservationStore& store) : store_(store) {}
  SnapshotLease acquire(std::int64_t now_ns, std::int64_t max_age_ns,
      bool allow_stale = false, std::uint64_t expected_epoch = 0,
      std::optional<std::uint64_t> expected_revision = std::nullopt);
  std::shared_ptr<const Snapshot> get(const std::string& handle);
  void release(const std::string& handle);
 private:
  using Clock = std::chrono::steady_clock;
  struct Entry { std::shared_ptr<const Snapshot> snapshot; Clock::time_point expires; };
  void expire_unlocked(Clock::time_point now, std::uint64_t current_epoch);
  ObservationStore& store_;
  std::mutex mutex_;
  std::map<std::string, Entry> entries_;
  std::uint64_t next_handle_{};
};
}  // namespace uld_occ_observer
