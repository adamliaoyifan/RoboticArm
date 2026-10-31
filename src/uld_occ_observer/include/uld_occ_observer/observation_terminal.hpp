#pragma once
#include "uld_occ_observer/observation_ledger.hpp"
#include "uld_occ_observer/snapshot_wait_board.hpp"

namespace uld_occ_observer {
struct SealedTerminal {
  bool stored{false};
  std::string state;
  UpdateReceipt receipt;
  double queue_ms{};
};

// Persists an in-flight ledger row, then records the same fact for waiters.
// A second call does not overwrite the first terminal. Publish only after this returns.
SealedTerminal seal_inflight(ObservationLedger& ledger, SnapshotWaitBoard& board,
                             std::uint64_t epoch, const std::string& sensor_id,
                             const std::string& observation_id, UpdateReceipt receipt,
                             std::chrono::steady_clock::time_point now, double queue_ms = 0);

// A rejection that could not take a ledger row, such as LEDGER_FULL. Waiters and a
// later status query still see REJECTED plus the reason. This does not evict anyone.
void note_unstored_rejection(SnapshotWaitBoard& board, std::uint64_t epoch,
                             const std::string& sensor_id, const std::string& observation_id,
                             const std::string& reason, std::int64_t input_stamp_ns);
}  // namespace uld_occ_observer
