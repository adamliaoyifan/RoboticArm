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
 private:
  mutable std::mutex mutex_;
  std::deque<Item> queue_;
  std::size_t peak_{};
};
}  // namespace uld_occ_observer
