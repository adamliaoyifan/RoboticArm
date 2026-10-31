#pragma once
#include "uld_occ_observer/observer.hpp"
#include "uld_occ_observer/resource_usage.hpp"
#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <vector>

namespace uld_occ_observer {
struct SnapshotLease {
  std::string handle;
  SnapshotIdentity identity;
  std::int64_t age_ns{};
  bool stale{};
};

class SnapshotRepository;

// Adds its bytes on construction and removes them on destruction. Concurrent
// responses sum. Move leaves the source empty so a response is counted once.
class InflightResponse {
 public:
  InflightResponse() = default;
  InflightResponse(const InflightResponse&) = delete;
  InflightResponse& operator=(const InflightResponse&) = delete;
  InflightResponse(InflightResponse&& other) noexcept;
  InflightResponse& operator=(InflightResponse&& other) noexcept;
  ~InflightResponse();
 private:
  friend class SnapshotRepository;
  InflightResponse(SnapshotRepository* repository, std::uint64_t bytes);
  void release() noexcept;
  SnapshotRepository* repository_{};
  std::uint64_t bytes_{};
};

// Handles own immutable snapshots. get() copies shared ownership and does not
// take the store mutex, so a read of an existing handle can finish while
// integrate holds that mutex. TTL expiry does not cancel a read already started.
// usage() counts each block and each Surface cache object once, including a
// snapshot that a caller still holds after release or reset.
class SnapshotRepository {
 public:
  explicit SnapshotRepository(ObservationStore& store) : store_(store) {}
  SnapshotLease acquire(std::int64_t now_ns, std::int64_t max_age_ns,
      bool allow_stale = false, std::uint64_t expected_epoch = 0,
      std::optional<std::uint64_t> expected_revision = std::nullopt);
  std::shared_ptr<const Snapshot> get(const std::string& handle);
  void release(const std::string& handle);
  ResourceUsage usage();
  InflightResponse begin_response(std::uint64_t bytes);
  void account(const ResourceAccounts& accounts);
  void set_ttl_for_test(std::chrono::milliseconds ttl);
  static constexpr std::chrono::seconds kDefaultTtl{5};
 private:
  friend class InflightResponse;
  using Clock = std::chrono::steady_clock;
  struct Entry { std::shared_ptr<const Snapshot> snapshot; Clock::time_point expires; };
  void expire_unlocked(Clock::time_point now, std::uint64_t current_epoch);
  void track_unlocked(const std::shared_ptr<const Snapshot>& snapshot);
  void adjust_inflight(std::int64_t delta);
  ObservationStore& store_;
  mutable std::mutex mutex_;
  std::map<std::string, Entry> entries_;
  std::vector<std::weak_ptr<const Snapshot>> readers_;
  std::uint64_t next_handle_{};
  std::chrono::milliseconds ttl_{kDefaultTtl};
  std::uint64_t block_peak_{}, directory_peak_{}, surface_peak_{};
  std::uint64_t inflight_bytes_{}, inflight_peak_{};
  ResourceAccounts accounts_{};
};
}  // namespace uld_occ_observer
