#include "uld_occ_observer/observation_ingress.hpp"

namespace uld_occ_observer {
ObservationIngress::Result ObservationIngress::try_admit(Item item) {
  std::lock_guard<std::mutex> lock(mutex_);
  Result result;
  result.observation_id = item.observation_id;
  if (queue_.size() >= kCapacity) {
    result.status = "QUEUE_FULL";
    return result;
  }
  queue_.push_back(std::move(item));
  peak_ = std::max(peak_, queue_.size());
  result.accepted = true;
  result.status = "QUEUED";
  return result;
}
std::optional<ObservationIngress::Item> ObservationIngress::try_pop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (queue_.empty()) return std::nullopt;
  Item item = std::move(queue_.front());
  queue_.pop_front();
  return item;
}
std::size_t ObservationIngress::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return queue_.size();
}
std::size_t ObservationIngress::peak() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return peak_;
}
}  // namespace uld_occ_observer
