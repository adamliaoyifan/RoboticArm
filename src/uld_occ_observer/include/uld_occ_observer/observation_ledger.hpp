#pragma once
#include "uld_occ_observer/observer.hpp"
#include <chrono>
#include <map>
#include <mutex>
#include <optional>

namespace uld_occ_observer {
// Terminal ledger. In-flight records are never evicted. Terminals stay at least
// 120 seconds. A full ledger rejects a new admission instead of dropping one.
class ObservationLedger {
 public:
  static constexpr std::size_t kCapacity = 4096;
  static constexpr std::chrono::seconds kRetention{120};
  struct Record {
    std::uint64_t epoch{};
    std::string state;
    UpdateReceipt receipt;
    bool in_flight{true};
    double queue_ms{};
    std::chrono::steady_clock::time_point updated{};
  };
  struct Admit {
    bool accepted{false};
    std::string status;
    std::string observation_id;
  };
  Admit admit(std::uint64_t epoch, const std::string& sensor_id, const std::string& observation_id,
              std::chrono::steady_clock::time_point now);
  void mark(std::uint64_t epoch, const std::string& sensor_id, const std::string& observation_id,
            const std::string& state, bool in_flight, std::chrono::steady_clock::time_point now);
  // Writes an existing in-flight record and returns. A missing or already terminal id
  // is left unchanged. The caller publishes only after this returns.
  bool complete(std::uint64_t epoch, const std::string& sensor_id, const std::string& observation_id,
                UpdateReceipt receipt, std::chrono::steady_clock::time_point now, double queue_ms = 0);
  std::optional<Record> query(std::uint64_t epoch, const std::string& sensor_id,
                              const std::string& observation_id) const;
  void expire(std::chrono::steady_clock::time_point now);
  void set_retention(std::chrono::milliseconds retention);
  void drop_terminal_other_epochs(std::uint64_t epoch);
  std::size_t size() const;
  std::size_t peak() const;
  std::size_t in_flight() const;
  std::uint64_t accounted_bytes() const;
  std::uint64_t accounted_bytes_peak() const;
 private:
  static std::string key(std::uint64_t epoch, const std::string& sensor_id, const std::string& observation_id);
  bool make_room(std::chrono::steady_clock::time_point now);
  std::uint64_t accounted_bytes_unlocked() const;
  mutable std::mutex mutex_;
  std::map<std::string, Record> records_;
  std::size_t peak_{};
  std::chrono::milliseconds retention_{kRetention};
  mutable std::uint64_t byte_peak_{};
};
}  // namespace uld_occ_observer
