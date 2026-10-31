#pragma once
#include "uld_occ_observer/observer.hpp"
#include "uld_occ_observer/resource_usage.hpp"
#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <optional>

namespace uld_occ_observer {
struct SnapshotLease {
  std::string handle;
  SnapshotIdentity identity;
  std::int64_t age_ns{};
  bool stale{};
};

// Handles own immutable snapshots. get() copies shared ownership and does not
// take the store mutex, so a read of an existing handle can finish while
// integrate holds that mutex. TTL expiry does not cancel a read already started.
class SnapshotRepository {
 public:
  explicit SnapshotRepository(ObservationStore& store) : store_(store) {}
  SnapshotLease acquire(std::int64_t now_ns, std::int64_t max_age_ns,
      bool allow_stale = false, std::uint64_t expected_epoch = 0,
      std::optional<std::uint64_t> expected_revision = std::nullopt);
  std::shared_ptr<const Snapshot> get(const std::string& handle);
  void release(const std::string& handle);
  ResourceUsage usage();
  void set_inflight_response_bytes(std::uint64_t bytes);
  void set_queue_counts(std::uint64_t entry, std::uint64_t entry_peak,
                        std::uint64_t ledger, std::uint64_t ledger_peak);
  void set_ttl_for_test(std::chrono::milliseconds ttl);
  static constexpr std::chrono::seconds kDefaultTtl{5};
 private:
  using Clock = std::chrono::steady_clock;
  struct Entry { std::shared_ptr<const Snapshot> snapshot; Clock::time_point expires; };
  void expire_unlocked(Clock::time_point now, std::uint64_t current_epoch);
  ObservationStore& store_;
  mutable std::mutex mutex_;
  std::map<std::string, Entry> entries_;
  std::uint64_t next_handle_{};
  std::chrono::milliseconds ttl_{kDefaultTtl};
  mutable std::uint64_t block_peak_{}, directory_peak_{}, surface_peak_{};
  std::atomic<std::uint64_t> inflight_bytes_{}, inflight_peak_{};
  std::atomic<std::uint64_t> entry_messages_{}, entry_peak_{}, ledger_records_{}, ledger_peak_{};
};
}  // namespace uld_occ_observer
