#pragma once
#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace uld_occ_observer {

// Reset validity state machine (plan OCTOMAP_RESET_VALIDITY_NEXT_20261103 §2).
//   VALID(e)    -- new-epoch map published and readiness-proven; only epoch e
//                  may be leased.
//   INVALID(e)  -- reset committed: epoch e-1 invalidated atomically before
//                  any other new request can observe old state. New reads are
//                  refused INVALIDATED; old-epoch writes terminate
//                  EPOCH_MISMATCH (store-side).
//   BUILDING(e) -- store emptied and accepting epoch-e observations; no
//                  placement reads. First accepted new-epoch observation with
//                  identity match, integrity and age <= max_age publishes
//                  VALID(e). A floor prior or restored pre-reset snapshot can
//                  never satisfy readiness (epoch must equal e).
enum class ValidityState { kInvalid, kBuilding, kValid };
std::string to_string(ValidityState state);

struct ValiditySnapshot {
  ValidityState state{ValidityState::kBuilding};
  std::uint64_t epoch{1};
  // Monotonic across every transition (reset, building, valid); two observers
  // compare generation to order what they saw without holding any lock.
  std::uint64_t generation{1};
  std::string reason;      // reset reason, "store_ready", "observed", "restored"
  std::string reset_id;    // id of the reset that opened the current chain
  std::int64_t transition_mono_ns{};
  // Identity fields as of a VALID publish (empty otherwise).
  std::uint64_t map_revision{}, observed_revision{};
  std::int64_t last_integrated_stamp_ns{};
  std::string geometry_hash, config_digest, clock_domain;
};

// Latched, replayable transition record. A restarting consumer replays these
// from the status service or the transient-local topic; a transient event
// alone is deliberately insufficient (plan §2).
struct ValidityEvent {
  std::uint64_t generation{};
  ValidityState state{};
  std::uint64_t epoch{};
  std::string reason, reset_id;
  std::int64_t transition_mono_ns{};
};

// Leaf lock. Lock order: this mutex is never taken while holding the store,
// repository, surface cache, wait, adapter, ledger or wait-board locks, and
// never held across ROS serialization or waits (observer_node.cpp table #7).
// Transitions are one logical step: state, epoch, generation and latched event
// update under the mutex; readers get an immutable snapshot atomically.
class ValidityBoard {
 public:
  static constexpr std::size_t kMaxEvents = 64;

  // Fresh boot: BUILDING(1) with an empty store ready for observations.
  ValidityBoard();

  std::shared_ptr<const ValiditySnapshot> load() const {
    return std::atomic_load(&current_);
  }
  std::vector<ValidityEvent> events() const;

  // Reset linearization: the caller has already bumped the store epoch to
  // new_epoch (store_->reset) and dropped the old snapshot. Publishing
  // INVALID here means every acquire that linearizes later must refuse.
  // Idempotency guard: returns the current snapshot unchanged when reset_id
  // already committed this exact epoch (duplicate reset within an epoch is a
  // replay, not a new transition).
  std::shared_ptr<const ValiditySnapshot> commit_reset(
      std::uint64_t new_epoch, const std::string& reason,
      const std::string& reset_id);
  // The emptied store is ready for new observations.
  std::shared_ptr<const ValiditySnapshot> begin_building(std::uint64_t epoch);
  // BUILDING(e) -> VALID(e) when readiness holds. The caller supplies the
  // candidate publish identity; this returns the (possibly unchanged)
  // snapshot and records a transition only when the predicate passes.
  // Documented readiness predicate (plan §3): state==BUILDING, epoch match,
  // known_cells > 0 (an accepted new-epoch observation), and
  // publish_stamp_ns - last_integrated_stamp_ns <= max_age_ns at
  // publication. Age is judged in the snapshot's own clock domain (the
  // input stamp), while the latched event's transition_mono_ns stays in the
  // steady monotonic domain that orders events for consumers.
  std::shared_ptr<const ValiditySnapshot> try_publish_valid(
      std::uint64_t epoch, std::uint64_t known_cells,
      std::uint64_t map_revision, std::uint64_t observed_revision,
      std::int64_t last_integrated_stamp_ns, std::int64_t publish_stamp_ns,
      std::int64_t now_mono_ns, std::int64_t max_age_ns,
      const std::string& geometry_hash, const std::string& config_digest,
      const std::string& clock_domain, const std::string& reason);
  // Explicit publish for a verified checkpoint restore (integrity proven by
  // the restore path's fingerprint check; still requires the epoch match).
  std::shared_ptr<const ValiditySnapshot> publish_restored(
      std::uint64_t epoch, std::uint64_t map_revision,
      std::uint64_t observed_revision, std::int64_t last_integrated_stamp_ns,
      const std::string& geometry_hash, const std::string& config_digest,
      const std::string& clock_domain);

  // Lease admission helper: an acquire/read may proceed only in VALID(e) for
  // the epoch it is about to lease.
  bool admits(std::uint64_t epoch) const;

  const std::string& last_reset_id() const { return last_reset_id_; }
  std::uint64_t last_reset_epoch() const { return last_reset_epoch_; }

 private:
  std::shared_ptr<const ValiditySnapshot> transition_unlocked(
      ValidityState state, std::uint64_t epoch, const std::string& reason,
      const std::string& reset_id, std::int64_t now_mono_ns);

  mutable std::mutex mutex_;  // guards events_, last_reset_id_/epoch_, writers
  std::shared_ptr<const ValiditySnapshot> current_;
  std::deque<ValidityEvent> events_;
  std::string last_reset_id_;
  std::uint64_t last_reset_epoch_{};
};

}  // namespace uld_occ_observer
