#include "uld_occ_observer/observation_ledger.hpp"
#include <algorithm>

namespace uld_occ_observer {
std::string ObservationLedger::key(std::uint64_t epoch, const std::string& sensor_id,
                                   const std::string& observation_id) {
  return std::to_string(epoch) + "|" + sensor_id + "|" + observation_id;
}
bool ObservationLedger::make_room(std::chrono::steady_clock::time_point now) {
  if (records_.size() < kCapacity) return true;
  for (auto it = records_.begin(); it != records_.end();) {
    const bool expired = !it->second.in_flight && it->second.state == "EXPIRED";
    const bool old_terminal = !it->second.in_flight && now - it->second.updated >= retention_;
    if (expired || old_terminal) it = records_.erase(it);
    else ++it;
  }
  return records_.size() < kCapacity;
}
ObservationLedger::Admit ObservationLedger::admit(std::uint64_t epoch, const std::string& sensor_id,
                                                  const std::string& observation_id,
                                                  std::chrono::steady_clock::time_point now,
                                                  std::int64_t entered_ns) {
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
  record.epoch = epoch;
  record.state = "QUEUED";
  record.in_flight = true;
  record.updated = now;
  record.entered_ns = entered_ns;
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
  if (record == records_.end() || !record->second.in_flight) return;
  record->second.state = state;
  record->second.in_flight = in_flight;
  record->second.updated = now;
}
bool ObservationLedger::complete(std::uint64_t epoch, const std::string& sensor_id,
                                 const std::string& observation_id, UpdateReceipt receipt,
                                 std::chrono::steady_clock::time_point now, double queue_ms) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto record = records_.find(key(epoch, sensor_id, observation_id));
  if (record == records_.end() || !record->second.in_flight) return false;
  record->second.receipt = std::move(receipt);
  // The entry moment is measured at admission, not at completion; it is
  // stamped once here and never overwritten by later seals.
  if (record->second.entered_ns != 0)
    record->second.receipt.callback_enter_ns = record->second.entered_ns;
  record->second.state = record->second.receipt.status == "INTEGRATED" ? "COMPLETED" : "REJECTED";
  record->second.in_flight = false;
  record->second.queue_ms = queue_ms;
  record->second.updated = now;
  peak_ = std::max(peak_, records_.size());
  byte_peak_ = std::max(byte_peak_, accounted_bytes_unlocked());
  return true;
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
    if (!record.second.in_flight && now - record.second.updated >= retention_)
      record.second.state = "EXPIRED";
  }
}
void ObservationLedger::set_retention(std::chrono::milliseconds retention) {
  std::lock_guard<std::mutex> lock(mutex_);
  retention_ = retention;
}
void ObservationLedger::drop_terminal_other_epochs(std::uint64_t epoch) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto it = records_.begin(); it != records_.end();) {
    if (!it->second.in_flight && it->second.epoch != epoch) it = records_.erase(it);
    else ++it;
  }
}
std::uint64_t ObservationLedger::accounted_bytes_unlocked() const {
  std::uint64_t bytes = 0;
  for (const auto& record : records_) {
    bytes += sizeof(Record);
    bytes += record.first.capacity();
    bytes += record.second.state.capacity();
    bytes += record.second.receipt.status.capacity();
    bytes += record.second.receipt.sensor_id.capacity();
    bytes += record.second.receipt.observation_id.capacity();
    bytes += record.second.receipt.identity.geometry_hash.capacity();
    bytes += record.second.receipt.identity.config_digest.capacity();
    bytes += record.second.receipt.identity.clock_domain.capacity();
  }
  return bytes;
}
std::uint64_t ObservationLedger::accounted_bytes() const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto bytes = accounted_bytes_unlocked();
  byte_peak_ = std::max(byte_peak_, bytes);
  return bytes;
}
std::uint64_t ObservationLedger::accounted_bytes_peak() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return std::max(byte_peak_, accounted_bytes_unlocked());
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
