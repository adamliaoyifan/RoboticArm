#pragma once
#include "uld_occ_observer/snapshot.hpp"
#include <chrono>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <string>

namespace uld_occ_observer {
// Wait state machine used by the deferred WaitSnapshot service. Success follows
// observed_revision from the completion record. A map_revision match alone is not enough.
class SnapshotWaitBoard {
 public:
  static constexpr std::size_t kMaxWaiting = 8;
  struct Request {
    std::uint64_t expected_epoch{};
    std::string sensor_id;
    std::string observation_id;
    std::int64_t max_age_ms{};
    std::int64_t timeout_ms{};
    std::chrono::steady_clock::time_point started{};
  };
  struct Outcome {
    std::string status;
    std::uint64_t observed_revision{};
    std::uint64_t map_revision{};
    std::int64_t age_ns{};
    std::string reason;
  };
  struct Completion {
    std::uint64_t epoch{};
    std::string sensor_id;
    std::string observation_id;
    std::string status;
    std::uint64_t observed_revision{};
    std::string reason;
    std::int64_t input_stamp_ns{};
  };
  static constexpr std::size_t kMaxCompletions = 4096;
  std::optional<Outcome> try_begin(std::size_t waiting, const Request& request,
                                   std::uint64_t current_epoch) const;
  void note_completion(const Completion& completion);
  std::optional<Outcome> poll(const Request& request, std::chrono::steady_clock::time_point now,
                              std::int64_t now_ns, std::uint64_t current_epoch,
                              const SnapshotIdentity* latest) const;
  std::optional<Completion> find(std::uint64_t epoch, const std::string& sensor_id,
                                 const std::string& observation_id) const;
  void drop_other_epochs(std::uint64_t epoch);
  std::size_t size() const;
  std::size_t peak() const;
  std::uint64_t accounted_bytes() const;
  std::uint64_t accounted_bytes_peak() const;
 private:
  static std::string key(std::uint64_t epoch, const std::string& sensor_id, const std::string& observation_id);
  void trim_unlocked();
  std::uint64_t accounted_bytes_unlocked() const;
  mutable std::mutex mutex_;
  std::map<std::string, Completion> completions_;
  std::deque<std::string> order_;
  std::size_t peak_{};
  mutable std::uint64_t byte_peak_{};
};
}  // namespace uld_occ_observer
