#include "uld_occ_observer/snapshot_wait_board.hpp"
#include <algorithm>

namespace uld_occ_observer {
std::string SnapshotWaitBoard::key(std::uint64_t epoch, const std::string& sensor_id,
                                   const std::string& observation_id) {
  return std::to_string(epoch) + "|" + sensor_id + "|" + observation_id;
}
std::optional<SnapshotWaitBoard::Outcome> SnapshotWaitBoard::try_begin(
    std::size_t waiting, const Request& request, std::uint64_t current_epoch) const {
  if (waiting >= kMaxWaiting) return Outcome{"WAIT_PRESSURE", 0, 0, 0, {}};
  if (request.expected_epoch != current_epoch) return Outcome{"EPOCH_MISMATCH", 0, 0, 0, {}};
  return std::nullopt;
}
void SnapshotWaitBoard::trim_unlocked() {
  while (completions_.size() > kMaxCompletions) {
    if (order_.empty()) break;
    const auto oldest = order_.front();
    order_.pop_front();
    if (!completions_.count(oldest)) continue;
    if (completions_.size() == 1) {
      order_.push_back(oldest);
      break;
    }
    completions_.erase(oldest);
  }
}
void SnapshotWaitBoard::note_completion(const Completion& completion) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto id = key(completion.epoch, completion.sensor_id, completion.observation_id);
  if (!completions_.count(id)) order_.push_back(id);
  completions_[id] = completion;
  trim_unlocked();
  peak_ = std::max(peak_, completions_.size());
  byte_peak_ = std::max(byte_peak_, accounted_bytes_unlocked());
}
std::optional<SnapshotWaitBoard::Outcome> SnapshotWaitBoard::poll(
    const Request& request, std::chrono::steady_clock::time_point now, std::int64_t now_ns,
    std::uint64_t current_epoch, const SnapshotIdentity* latest) const {
  if (request.expected_epoch != current_epoch) return Outcome{"EPOCH_MISMATCH", 0, 0, 0, {}};
  const bool timed_out = request.timeout_ms >= 0 &&
      now - request.started >= std::chrono::milliseconds(request.timeout_ms);
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = completions_.find(key(request.expected_epoch, request.sensor_id, request.observation_id));
  if (found != completions_.end() && found->second.status != "COMPLETED") {
    Outcome outcome;
    outcome.status = "REJECTED";
    outcome.reason = found->second.reason.empty() ? found->second.status : found->second.reason;
    if (outcome.reason == "REJECTED") outcome.reason.clear();
    outcome.observed_revision = found->second.observed_revision;
    return outcome;
  }
  if (found != completions_.end() && latest && latest->epoch == request.expected_epoch &&
      latest->observed_revision >= found->second.observed_revision) {
    const auto age_ns = now_ns - latest->last_integrated_stamp_ns;
    if (age_ns >= 0 && age_ns <= request.max_age_ms * static_cast<std::int64_t>(1000000))
      return Outcome{"OK", latest->observed_revision, latest->map_revision, age_ns, {}};
  }
  if (timed_out) return Outcome{"TIMEOUT", 0, 0, 0, {}};
  return std::nullopt;
}
std::optional<SnapshotWaitBoard::Completion> SnapshotWaitBoard::find(
    std::uint64_t epoch, const std::string& sensor_id, const std::string& observation_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = completions_.find(key(epoch, sensor_id, observation_id));
  if (found == completions_.end()) return std::nullopt;
  return found->second;
}
void SnapshotWaitBoard::drop_other_epochs(std::uint64_t epoch) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto it = completions_.begin(); it != completions_.end();) {
    if (it->second.epoch != epoch) it = completions_.erase(it);
    else ++it;
  }
  order_.erase(std::remove_if(order_.begin(), order_.end(), [&](const std::string& id) {
    return !completions_.count(id);
  }), order_.end());
}
std::size_t SnapshotWaitBoard::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return completions_.size();
}
std::size_t SnapshotWaitBoard::peak() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return peak_;
}
std::uint64_t SnapshotWaitBoard::accounted_bytes_unlocked() const {
  std::uint64_t bytes = 0;
  for (const auto& completion : completions_) {
    bytes += sizeof(Completion);
    bytes += completion.first.capacity();
    bytes += completion.second.sensor_id.capacity();
    bytes += completion.second.observation_id.capacity();
    bytes += completion.second.status.capacity();
    bytes += completion.second.reason.capacity();
  }
  return bytes;
}
std::uint64_t SnapshotWaitBoard::accounted_bytes() const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto bytes = accounted_bytes_unlocked();
  byte_peak_ = std::max(byte_peak_, bytes);
  return bytes;
}
std::uint64_t SnapshotWaitBoard::accounted_bytes_peak() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return std::max(byte_peak_, accounted_bytes_unlocked());
}
}  // namespace uld_occ_observer
