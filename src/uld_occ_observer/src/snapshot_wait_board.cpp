#include "uld_occ_observer/snapshot_wait_board.hpp"

namespace uld_occ_observer {
std::string SnapshotWaitBoard::key(std::uint64_t epoch, const std::string& sensor_id,
                                   const std::string& observation_id) {
  return std::to_string(epoch) + "|" + sensor_id + "|" + observation_id;
}
std::optional<SnapshotWaitBoard::Outcome> SnapshotWaitBoard::try_begin(
    std::size_t waiting, const Request& request, std::uint64_t current_epoch) const {
  if (waiting >= kMaxWaiting) return Outcome{"WAIT_PRESSURE", 0, 0, 0};
  if (request.expected_epoch != current_epoch) return Outcome{"EPOCH_MISMATCH", 0, 0, 0};
  return std::nullopt;
}
void SnapshotWaitBoard::note_completion(const Completion& completion) {
  std::lock_guard<std::mutex> lock(mutex_);
  completions_[key(completion.epoch, completion.sensor_id, completion.observation_id)] = completion;
}
std::optional<SnapshotWaitBoard::Outcome> SnapshotWaitBoard::poll(
    const Request& request, std::chrono::steady_clock::time_point now, std::int64_t now_ns,
    std::uint64_t current_epoch, const SnapshotIdentity* latest) const {
  if (request.expected_epoch != current_epoch) return Outcome{"EPOCH_MISMATCH", 0, 0, 0};
  if (now - request.started >= std::chrono::milliseconds(request.timeout_ms))
    return Outcome{"TIMEOUT", 0, 0, 0};
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = completions_.find(key(request.expected_epoch, request.sensor_id, request.observation_id));
  if (found == completions_.end()) return std::nullopt;
  if (found->second.status != "COMPLETED")
    return Outcome{found->second.status, found->second.observed_revision, 0, 0};
  if (!latest || latest->epoch != request.expected_epoch ||
      latest->observed_revision < found->second.observed_revision)
    return std::nullopt;
  const auto age_ns = now_ns - latest->last_integrated_stamp_ns;
  if (age_ns < 0 || age_ns > request.max_age_ms * 1000000) return std::nullopt;
  return Outcome{"OK", latest->observed_revision, latest->map_revision, age_ns};
}
}  // namespace uld_occ_observer
