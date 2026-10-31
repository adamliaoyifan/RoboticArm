#include "uld_occ_observer/observation_ledger.hpp"

namespace uld_occ_observer {
std::string ObservationLedger::key(std::uint64_t epoch, const std::string& sensor_id,
                                   const std::string& observation_id) {
  return std::to_string(epoch) + "|" + sensor_id + "|" + observation_id;
}
bool ObservationLedger::make_room(std::chrono::steady_clock::time_point now) {
  if (records_.size() < kCapacity) return true;
  for (auto it = records_.begin(); it != records_.end();) {
    const bool expired = !it->second.in_flight && it->second.state == "EXPIRED";
    const bool old_terminal = !it->second.in_flight && now - it->second.updated >= kRetention;
    if (expired || old_terminal) it = records_.erase(it);
    else ++it;
  }
  return records_.size() < kCapacity;
}
ObservationLedger::Admit ObservationLedger::admit(std::uint64_t epoch, const std::string& sensor_id,
                                                  const std::string& observation_id,
                                                  std::chrono::steady_clock::time_point now) {
  std::lock_guard<std::mutex> lock(mutex_);
  Admit result;
  result.observation_id = observation_id;
  const auto id = key(epoch, sensor_id, observation_id);
  const auto existing = records_.find(id);
  if (existing != records_.end() && existing->second.state != "EXPIRED") {
    result.status = "DUPLICATE";
    return result;
  }
  if (!make_room(now)) {
    result.status = "LEDGER_FULL";
    return result;
  }
  Record record;
  record.state = "QUEUED";
  record.in_flight = true;
  record.updated = now;
  record.receipt.sensor_id = sensor_id;
  record.receipt.observation_id = observation_id;
  records_[id] = std::move(record);
  peak_ = std::max(peak_, records_.size());
  result.accepted = true;
  result.status = "QUEUED";
  return result;
}
void ObservationLedger::mark(std::uint64_t epoch, const std::string& sensor_id,
                             const std::string& observation_id, const std::string& state,
                             bool in_flight, std::chrono::steady_clock::time_point now) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto record = records_.find(key(epoch, sensor_id, observation_id));
  if (record == records_.end()) return;
  record->second.state = state;
  record->second.in_flight = in_flight;
  record->second.updated = now;
}
void ObservationLedger::complete(std::uint64_t epoch, const std::string& sensor_id,
                                 const std::string& observation_id, UpdateReceipt receipt,
                                 std::chrono::steady_clock::time_point now) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto& record = records_[key(epoch, sensor_id, observation_id)];
  record.receipt = std::move(receipt);
  record.state = record.receipt.status == "INTEGRATED" ? "COMPLETED" : "REJECTED";
  record.in_flight = false;
  record.updated = now;
  peak_ = std::max(peak_, records_.size());
}
std::optional<ObservationLedger::Record> ObservationLedger::query(
    std::uint64_t epoch, const std::string& sensor_id, const std::string& observation_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto record = records_.find(key(epoch, sensor_id, observation_id));
  if (record == records_.end()) return std::nullopt;
  return record->second;
}
void ObservationLedger::expire(std::chrono::steady_clock::time_point now) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& record : records_) {
    if (!record.second.in_flight && now - record.second.updated >= kRetention)
      record.second.state = "EXPIRED";
  }
}
std::size_t ObservationLedger::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return records_.size();
}
std::size_t ObservationLedger::peak() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return peak_;
}
std::size_t ObservationLedger::in_flight() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::size_t count = 0;
  for (const auto& record : records_) if (record.second.in_flight) ++count;
  return count;
}
}  // namespace uld_occ_observer
