#pragma once
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace uld_occ_observer {
// Bounded raw-message queue. The subscription callback only admits; it does not
// wait for TF or the store lock. Capacity 2. A refusal keeps the caller's id.
class ObservationIngress {
 public:
  static constexpr std::size_t kCapacity = 2;
  struct Item {
    std::string sensor_id;
    std::string observation_id;
    std::uint64_t epoch{};
    std::shared_ptr<void> payload;
    std::uint64_t payload_bytes{};
  };
  struct Result {
    bool accepted{false};
    std::string status;
    std::string observation_id;
  };
  Result try_admit(Item item);
  std::optional<Item> try_pop();
  std::size_t size() const;
  std::size_t peak() const;
  std::uint64_t payload_bytes() const;
  std::uint64_t payload_peak() const;
  // Retained chunk capacity of the deque, from the owning container's own
  // chunk model. libstdc++ deque allocates fixed-size chunks and never frees
  // them on pop, so an empty queue retains at least the chunks its high-water
  // mark filled (plus one spare). This counts real retained capacity, not a
  // per-message estimate.
  std::uint64_t allocated_bytes() const;
 private:
  void report_levels_locked();
  mutable std::mutex mutex_;
  std::deque<Item> queue_;
  std::size_t peak_{};
  std::uint64_t queued_payload_{};
  std::uint64_t payload_peak_{};
};
}  // namespace uld_occ_observer
