#include "uld_occ_observer/validity.hpp"
#include "uld_occ_observer/initial_empty.hpp"

#include <algorithm>
#include <chrono>

namespace uld_occ_observer {
namespace {
std::int64_t steady_now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
}  // namespace

std::string to_string(ValidityState state) {
  switch (state) {
    case ValidityState::kValid: return "VALID";
    case ValidityState::kInvalid: return "INVALID";
    case ValidityState::kBuilding: return "BUILDING";
  }
  return "UNKNOWN";
}

ValidityBoard::ValidityBoard() {
  auto initial = std::make_shared<ValiditySnapshot>();
  initial->state = ValidityState::kBuilding;
  initial->epoch = 1;
  initial->generation = 1;
  initial->reason = "boot";
  current_ = initial;
  events_.push_back(
      ValidityEvent{1, initial->state, initial->epoch, initial->reason, "",
                    initial->transition_mono_ns});
}

std::vector<ValidityEvent> ValidityBoard::events() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return std::vector<ValidityEvent>(events_.begin(), events_.end());
}

std::shared_ptr<const ValiditySnapshot> ValidityBoard::transition_unlocked(
    ValidityState state, std::uint64_t epoch, const std::string& reason,
    const std::string& reset_id, std::int64_t now_mono_ns) {
  auto next = std::make_shared<ValiditySnapshot>(*current_);
  const auto generation = next->generation + 1;
  next->state = state;
  next->epoch = epoch;
  next->generation = generation;
  next->reason = reason;
  next->reset_id = reset_id;
  next->transition_mono_ns = now_mono_ns;
  // Non-VALID transitions drop the published identity: there is no trusted
  // map to describe. VALID identity is filled by the publish paths.
  next->map_revision = next->observed_revision = 0;
  next->last_integrated_stamp_ns = 0;
  next->geometry_hash.clear();
  next->config_digest.clear();
  next->clock_domain.clear();
  events_.push_back(ValidityEvent{generation, state, epoch, reason, reset_id,
                                  now_mono_ns});
  while (events_.size() > kMaxEvents) events_.pop_front();
  current_ = next;
  return next;
}

std::shared_ptr<const ValiditySnapshot> ValidityBoard::commit_reset(
    std::uint64_t new_epoch, const std::string& reason,
    const std::string& reset_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (last_reset_epoch_ == new_epoch && last_reset_id_ == reset_id) {
    // Duplicate reset within the epoch: idempotent replay, no transition.
    return current_;
  }
  last_reset_id_ = reset_id;
  last_reset_epoch_ = new_epoch;
  return transition_unlocked(ValidityState::kInvalid, new_epoch, reason,
                             reset_id, steady_now_ns());
}

std::shared_ptr<const ValiditySnapshot> ValidityBoard::begin_building(
    std::uint64_t epoch) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_->epoch != epoch ||
      current_->state == ValidityState::kBuilding)
    return current_;
  return transition_unlocked(ValidityState::kBuilding, epoch, "store_ready",
                             current_->reset_id, steady_now_ns());
}

std::shared_ptr<const ValiditySnapshot> ValidityBoard::try_publish_valid(
    std::uint64_t epoch, std::uint64_t known_cells,
    std::uint64_t map_revision, std::uint64_t observed_revision,
    std::int64_t last_integrated_stamp_ns, std::int64_t publish_stamp_ns,
    std::int64_t now_mono_ns, std::int64_t max_age_ns,
    const std::string& geometry_hash, const std::string& config_digest,
    const std::string& clock_domain, const std::string& reason) {
  std::shared_ptr<const ValiditySnapshot> next;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_->state != ValidityState::kBuilding || current_->epoch != epoch)
      return current_;
    if (known_cells == 0) return current_;  // no accepted observation yet
    if (last_integrated_stamp_ns <= 0 ||
        publish_stamp_ns - last_integrated_stamp_ns > max_age_ns)
      return current_;  // stale publication cannot prove readiness
    auto candidate = std::make_shared<ValiditySnapshot>(*current_);
    candidate->state = ValidityState::kValid;
    candidate->generation = current_->generation + 1;
    candidate->reason = reason;
    candidate->transition_mono_ns = now_mono_ns;
    candidate->map_revision = map_revision;
    candidate->observed_revision = observed_revision;
    candidate->last_integrated_stamp_ns = last_integrated_stamp_ns;
    candidate->geometry_hash = geometry_hash;
    candidate->config_digest = config_digest;
    candidate->clock_domain = clock_domain;
    events_.push_back(ValidityEvent{candidate->generation,
                                    ValidityState::kValid, epoch, reason,
                                    candidate->reset_id, now_mono_ns});
    while (events_.size() > kMaxEvents) events_.pop_front();
    current_ = candidate;
    next = candidate;
  }
  return next;
}

std::shared_ptr<const ValiditySnapshot> ValidityBoard::publish_restored(
    std::uint64_t epoch, std::uint64_t map_revision,
    std::uint64_t observed_revision, std::int64_t last_integrated_stamp_ns,
    const std::string& geometry_hash, const std::string& config_digest,
    const std::string& clock_domain) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_->epoch != epoch ||
      current_->state == ValidityState::kValid)
    return current_;
  auto candidate = std::make_shared<ValiditySnapshot>(*current_);
  candidate->state = ValidityState::kValid;
  candidate->generation = current_->generation + 1;
  candidate->reason = "restored";
  candidate->transition_mono_ns = steady_now_ns();
  candidate->map_revision = map_revision;
  candidate->observed_revision = observed_revision;
  candidate->last_integrated_stamp_ns = last_integrated_stamp_ns;
  candidate->geometry_hash = geometry_hash;
  candidate->config_digest = config_digest;
  candidate->clock_domain = clock_domain;
  events_.push_back(ValidityEvent{candidate->generation,
                                  ValidityState::kValid, epoch, "restored",
                                  candidate->reset_id,
                                  candidate->transition_mono_ns});
  while (events_.size() > kMaxEvents) events_.pop_front();
  current_ = candidate;
  return candidate;
}

std::shared_ptr<const ValiditySnapshot> ValidityBoard::publish_initialized(const SnapshotIdentity& id) {
  if (!declaration_only(id) || id.map_revision == 0) throw Error("INVALID_EMPTY_DECLARATION");
  const auto epoch = id.epoch;
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_->epoch != epoch ||
      current_->state == ValidityState::kValid)
    return current_;
  auto candidate = std::make_shared<ValiditySnapshot>(*current_);
  candidate->state = ValidityState::kValid;
  candidate->generation = current_->generation + 1;
  candidate->reason = "declared_initial_empty";
  candidate->transition_mono_ns = steady_now_ns();
  candidate->map_revision = id.map_revision;
  candidate->observed_revision = id.observed_revision;
  candidate->last_integrated_stamp_ns = id.last_integrated_stamp_ns;
  candidate->geometry_hash = id.geometry_hash;
  candidate->config_digest = id.config_digest;
  candidate->clock_domain = id.clock_domain;
  events_.push_back(ValidityEvent{candidate->generation,
                                  ValidityState::kValid, epoch, "declared_initial_empty",
                                  candidate->reset_id,
                                  candidate->transition_mono_ns});
  while (events_.size() > kMaxEvents) events_.pop_front();
  current_ = candidate;
  return candidate;
}

bool ValidityBoard::admits(std::uint64_t epoch) const {
  const auto snapshot = load();
  return snapshot->state == ValidityState::kValid && snapshot->epoch == epoch;
}

}  // namespace uld_occ_observer
